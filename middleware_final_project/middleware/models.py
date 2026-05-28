"""雷达、光电、无线电探测模型。

参数按设计文档默认值实现，后续实验室设备指标变化时优先改配置文件，
不要在业务代码里硬编码设备参数。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Tuple

from .config import MiddlewareConfig
from .domain import DetectionReport, PayloadType, TargetState, TrackState
from .utils import (
    GeoPoint,
    angle_in_ranges,
    azimuth_deg_from_enu,
    clamp,
    distance_m_from_enu,
    elevation_deg_from_enu,
    geodetic_to_enu,
    linear_score,
    signed_angle_delta_deg,
    spherical_to_enu,
)


@dataclass(slots=True)
class TargetTrackMemory:
    """单目标在某一类载荷里的状态记忆。"""

    state: TrackState = TrackState.INIT
    hit_streak: int = 0
    miss_streak: int = 0
    first_hit_timestamp_us: Optional[int] = None
    emitted_lost: bool = False


@dataclass(slots=True)
class StateUpdate:
    state: TrackState
    should_report: bool
    just_lost: bool
    hit_streak: int
    miss_streak: int
    elapsed_since_first_hit_s: float


@dataclass(slots=True)
class CommonDetectionStateMachine:
    """统一报送状态机：INIT→CANDIDATE→TRACKING→LOST_PENDING→LOST。"""

    n_detect: int
    n_lost: int
    memories: Dict[str, TargetTrackMemory] = field(default_factory=dict)

    def update(self, target_id: str, detected_now: bool, timestamp_us: int, required_hits: Optional[int] = None) -> StateUpdate:
        required_hits = max(1, int(required_hits or self.n_detect))
        mem = self.memories.setdefault(target_id, TargetTrackMemory())

        if detected_now:
            mem.hit_streak += 1
            mem.miss_streak = 0
            mem.emitted_lost = False
            if mem.first_hit_timestamp_us is None:
                mem.first_hit_timestamp_us = timestamp_us
            mem.state = TrackState.TRACKING if mem.hit_streak >= required_hits else TrackState.CANDIDATE
        else:
            mem.hit_streak = 0
            mem.first_hit_timestamp_us = None
            if mem.state in (TrackState.TRACKING, TrackState.LOST_PENDING):
                mem.miss_streak += 1
                mem.state = TrackState.LOST if mem.miss_streak >= self.n_lost else TrackState.LOST_PENDING
            else:
                mem.miss_streak += 1
                mem.state = TrackState.LOST if mem.miss_streak >= self.n_lost else TrackState.INIT

        elapsed_s = 0.0
        if mem.first_hit_timestamp_us is not None:
            elapsed_s = max(0.0, (timestamp_us - mem.first_hit_timestamp_us) / 1_000_000.0)
        just_lost = mem.state == TrackState.LOST and not mem.emitted_lost
        if just_lost:
            mem.emitted_lost = True
        return StateUpdate(mem.state, mem.state == TrackState.TRACKING, just_lost, mem.hit_streak, mem.miss_streak, elapsed_s)


def _payload_site(payload_config: Dict[str, Any], global_config: MiddlewareConfig) -> GeoPoint:
    site = payload_config.get("site") or {}
    return GeoPoint(
        lat=float(site.get("lat", global_config.origin.lat)),
        lon=float(site.get("lon", global_config.origin.lon)),
        alt=float(site.get("alt", global_config.origin.alt)),
    )


def _target_relative_geometry(target: TargetState, site: GeoPoint) -> Tuple[float, float, float, float, float, float]:
    x, y, z = geodetic_to_enu(target.lat, target.lon, target.alt, site)
    distance = distance_m_from_enu(x, y, z)
    azimuth = azimuth_deg_from_enu(x, y)
    elevation = elevation_deg_from_enu(x, y, z)
    return x, y, z, distance, azimuth, elevation


def _bounded_gauss(rng: random.Random, sigma: float, max_abs: float | None = None) -> float:
    if sigma <= 0:
        return 0.0
    value = rng.gauss(0.0, sigma)
    return clamp(value, -max_abs, max_abs) if max_abs is not None else value


def _target_frequency_mhz(target: TargetState, cfg: Dict[str, Any]) -> float:
    """无线电频率：设计输入表没有必填频率，因此支持扩展字段和默认值。"""
    body = (target.raw or {}).get("body", {}) if isinstance(target.raw, dict) else {}
    for key in ("FrequencyMHz", "CommFrequencyMHz", "frequency_mhz", "FreqMHz", "RadioFrequencyMHz"):
        if body.get(key) not in (None, ""):
            return float(body[key])
    for key in ("FrequencyKHz", "frequency_khz", "FreqKHz"):
        if body.get(key) not in (None, ""):
            return float(body[key]) / 1000.0
    return float(cfg.get("default_target_frequency_mhz", 2400.0))


def _frequency_in_bands(freq_mhz: float, bands_mhz: Iterable[Iterable[float]]) -> bool:
    for band in bands_mhz:
        vals = list(band)
        if len(vals) == 2 and float(vals[0]) <= freq_mhz <= float(vals[1]):
            return True
    return False


@dataclass(slots=True)
class RadarModel:
    """雷达模型：距离/角度/速度门限 + Pd 随机命中 + 测量误差注入。"""

    config: MiddlewareConfig
    rng: random.Random = field(init=False)
    state_machine: CommonDetectionStateMachine = field(init=False)

    def __post_init__(self) -> None:
        cfg = self.config.radar
        self.rng = random.Random(cfg.get("random_seed", 20260521))
        self.state_machine = CommonDetectionStateMachine(int(cfg.get("n_detect", 3)), int(cfg.get("n_lost", 5)))

    def process(self, targets: Iterable[TargetState]) -> List[DetectionReport]:
        cfg = self.config.radar
        site = _payload_site(cfg, self.config)
        r_min = float(cfg.get("r_min_m", 50.0))
        r_max = float(cfg.get("r_max_m", 3000.0))
        azimuth_ranges = cfg.get("azimuth_ranges_deg", [[0.0, 360.0]])
        el_min = float(cfg.get("elevation_min_deg", -5.0))
        el_max = float(cfg.get("elevation_max_deg", 40.0))
        min_speed = float(cfg.get("min_detectable_speed_mps", 2.0))
        base_pd = float(cfg.get("pd", 0.95))
        pd_at_max = float(cfg.get("pd_at_rmax", base_pd))
        low_speed_factor = float(cfg.get("low_speed_pd_factor", 0.35))
        sigma_r = float(cfg.get("sigma_r_m", 5.0))
        sigma_az = float(cfg.get("sigma_az_deg", 0.4))
        sigma_el = float(cfg.get("sigma_el_deg", 0.4))
        rcs = float(cfg.get("default_rcs_sqm", 0.01))

        reports: List[DetectionReport] = []
        for target in targets:
            x, y, z, distance, azimuth, elevation = _target_relative_geometry(target, site)
            geometry_ok = (
                r_min <= distance <= r_max
                and angle_in_ranges(azimuth, azimuth_ranges)
                and el_min <= elevation <= el_max
            )
            pd = linear_score(distance, r_min, r_max, base_pd, pd_at_max)
            if target.speed_mps() < min_speed:
                pd *= low_speed_factor
            hit = geometry_ok and self.rng.random() <= pd
            update = self.state_machine.update(target.target_id, hit, target.timestamp_us)
            if not update.should_report:
                continue

            # 满足稳定跟踪后，按设计文档对距离/方位/俯仰叠加高斯误差。
            measured_r = max(0.0, distance + _bounded_gauss(self.rng, sigma_r))
            measured_az = (azimuth + _bounded_gauss(self.rng, sigma_az)) % 360.0
            measured_el = elevation + _bounded_gauss(self.rng, sigma_el)
            mx, my, mz = spherical_to_enu(measured_r, measured_az, measured_el)
            snr = linear_score(distance, r_min, r_max, float(cfg.get("snr_near_db", 40.0)), float(cfg.get("snr_far_db", 20.0)))
            reports.append(
                DetectionReport(
                    payload_type=PayloadType.RADAR,
                    target=target,
                    state=TrackState.TRACKING,
                    confidence=max(0.0, min(1.0, pd)),
                    distance_m=measured_r,
                    azimuth_deg=measured_az,
                    elevation_deg=measured_el,
                    snr_db=snr,
                    rcs_sqm=rcs,
                    extra={"measurement_enu_m": (mx, my, mz), "raw_geometry_enu_m": (x, y, z), "pd": pd},
                )
            )
        return reports


@dataclass(slots=True)
class ElectroOpticalModel:
    """光电模型：距离门限、云台覆盖、FOV 视场、捕获确认、置信度、脱靶量和目标框。"""

    config: MiddlewareConfig
    state_machine: CommonDetectionStateMachine = field(init=False)

    def __post_init__(self) -> None:
        cfg = self.config.electro_optical
        self.state_machine = CommonDetectionStateMachine(int(cfg.get("n_detect", 3)), int(cfg.get("n_lost", 8)))

    def process(self, targets: Iterable[TargetState]) -> List[DetectionReport]:
        cfg = self.config.electro_optical
        site = _payload_site(cfg, self.config)
        r_min = float(cfg.get("r_min_m", 40.0))
        r_max = float(cfg.get("r_max_m", 2000.0))
        gimbal_azimuth_ranges = cfg.get("gimbal_azimuth_ranges_deg", [[0.0, 360.0]])
        gimbal_el_min = float(cfg.get("gimbal_elevation_min_deg", -40.0))
        gimbal_el_max = float(cfg.get("gimbal_elevation_max_deg", 85.0))
        fov_h = float(cfg.get("fov_horizontal_deg", cfg.get("fov_deg", 60.0)))
        fov_v = float(cfg.get("fov_vertical_deg", cfg.get("fov_deg", 60.0)))
        confidence_threshold = float(cfg.get("confidence_threshold", 0.6))
        image_width = int(cfg.get("image_width", 640))
        image_height = int(cfg.get("image_height", 512))
        n_detect = int(cfg.get("n_detect", 3))
        capture_delay_s = float(cfg.get("capture_delay_s", 0.5))
        refresh_rate_hz = float(cfg.get("refresh_rate_hz", 30.0))
        # 设计要求“持续满足 3 帧 或 0.5~5s”后输出；这里取二者较小值，保证不迟于配置捕获延迟。
        required_hits = min(max(1, n_detect), max(1, int(round(capture_delay_s * refresh_rate_hz))))
        auto_pointing = bool(cfg.get("auto_point_to_target", True))
        configured_pan = float(cfg.get("optical_axis_azimuth_deg", 0.0))
        configured_tilt = float(cfg.get("optical_axis_elevation_deg", 0.0))
        channel_id = int(cfg.get("channel_id", 1))

        reports: List[DetectionReport] = []
        for target in targets:
            x, y, z, distance, azimuth, elevation = _target_relative_geometry(target, site)
            in_distance = r_min <= distance <= r_max
            in_gimbal = angle_in_ranges(azimuth, gimbal_azimuth_ranges) and gimbal_el_min <= elevation <= gimbal_el_max

            # auto_point_to_target=True 用于仿真联调：默认云台能跟随目标进入光轴附近。
            pan = azimuth if auto_pointing else configured_pan
            tilt = elevation if auto_pointing else configured_tilt
            delta_h = signed_angle_delta_deg(azimuth, pan)
            delta_v = elevation - tilt
            in_fov = abs(delta_h) <= fov_h / 2.0 and abs(delta_v) <= fov_v / 2.0

            confidence = self._confidence(target, distance, r_min, r_max, delta_h, delta_v, fov_h, fov_v)
            detected = in_distance and in_gimbal and in_fov and confidence >= confidence_threshold
            update = self.state_machine.update(target.target_id, detected, target.timestamp_us, required_hits=required_hits)
            if not update.should_report:
                continue

            rect = self._image_rect(delta_h, delta_v, distance, fov_h, fov_v, image_width, image_height, r_min, r_max)
            reports.append(
                DetectionReport(
                    payload_type=PayloadType.EO,
                    target=target,
                    state=TrackState.TRACKING,
                    confidence=confidence,
                    distance_m=distance,
                    azimuth_deg=azimuth,
                    elevation_deg=elevation,
                    channel_id=channel_id,
                    offset_h_mrad=delta_h * 17.453292519943293,
                    offset_v_mrad=delta_v * 17.453292519943293,
                    rect=rect,
                    extra={"relative_enu_m": (x, y, z), "optical_axis_deg": (pan, tilt), "fov_deg": (fov_h, fov_v)},
                )
            )
        return reports

    @staticmethod
    def _confidence(target: TargetState, distance: float, r_min: float, r_max: float, delta_h: float, delta_v: float, fov_h: float, fov_v: float) -> float:
        distance_score = linear_score(distance, r_min, r_max, 1.0, 0.55)
        h_norm = abs(delta_h) / max(fov_h / 2.0, 1e-6)
        v_norm = abs(delta_v) / max(fov_v / 2.0, 1e-6)
        fov_score = max(0.0, 1.0 - 0.35 * max(h_norm, v_norm))
        type_factor = {"uav": 1.0, "manned_aircraft": 0.95, "vehicle": 0.8, "ship": 0.8, "bird": 0.7, "unknown": 0.75}.get(target.target_type.value, 0.75)
        return round(max(0.0, min(1.0, distance_score * fov_score * type_factor * max(0.0, min(1.0, target.quality)))), 3)

    @staticmethod
    def _image_rect(delta_h: float, delta_v: float, distance: float, fov_h: float, fov_v: float, image_width: int, image_height: int, r_min: float, r_max: float) -> Tuple[int, int, int, int]:
        cx = image_width / 2.0 + (delta_h / max(fov_h, 1e-6)) * image_width
        cy = image_height / 2.0 - (delta_v / max(fov_v, 1e-6)) * image_height
        size_score = linear_score(distance, r_min, r_max, 1.0, 0.25)
        width = max(12, int(round(80 * size_score)))
        height = max(10, int(round(60 * size_score)))
        return int(round(cx - width / 2.0)), int(round(cy - height / 2.0)), width, height


@dataclass(slots=True)
class RadioDetectionModel:
    """无线电模型：3km 距离、-5~55°俯仰、频段匹配、截获延迟、测向误差和置信度。"""

    config: MiddlewareConfig
    rng: random.Random = field(init=False)
    state_machine: CommonDetectionStateMachine = field(init=False)

    def __post_init__(self) -> None:
        cfg = self.config.radio
        self.rng = random.Random(cfg.get("random_seed", 20260522))
        self.state_machine = CommonDetectionStateMachine(int(cfg.get("n_detect", 3)), int(cfg.get("n_lost", 8)))

    def process(self, targets: Iterable[TargetState]) -> List[DetectionReport]:
        cfg = self.config.radio
        site = _payload_site(cfg, self.config)
        r_max = float(cfg.get("detection_range_m", 3000.0))
        el_min = float(cfg.get("elevation_min_deg", -5.0))
        el_max = float(cfg.get("elevation_max_deg", 55.0))
        bands = cfg.get("frequency_bands_mhz", [[300.0, 6000.0]])
        max_targets = int(cfg.get("max_concurrent_targets", 30))
        threshold_percent = float(cfg.get("confidence_threshold", 60.0))
        intercept_delay_s = float(cfg.get("intercept_delay_s", 3.0))
        refresh_period_s = float(cfg.get("refresh_period_s", 1.0))
        required_hits = max(int(cfg.get("n_detect", 3)), max(1, int(math.ceil(intercept_delay_s / max(refresh_period_s, 1e-6)))))
        direction_error_max = float(cfg.get("direction_error_max_deg", 3.0))
        direction_error_sigma = float(cfg.get("direction_error_sigma_deg", direction_error_max / 3.0))
        station_id = int(cfg.get("station_id", 1))
        bandwidth_khz = int(cfg.get("default_bandwidth_khz", 20000))

        evaluated = []
        for target in targets:
            x, y, z, distance, azimuth, elevation = _target_relative_geometry(target, site)
            freq_mhz = _target_frequency_mhz(target, cfg)
            band_match = _frequency_in_bands(freq_mhz, bands)
            signal_strength_db = self._signal_strength_db(distance, r_max)
            confidence_percent = self._confidence_percent(target, distance, r_max, signal_strength_db, band_match)
            eligible = distance <= r_max and el_min <= elevation <= el_max and band_match and confidence_percent >= threshold_percent
            evaluated.append((eligible, confidence_percent, distance, target, x, y, z, azimuth, elevation, freq_mhz, signal_strength_db))

        # 并发限制：置信度高、距离近的目标优先占用侦测能力。
        eligible_targets = [item for item in evaluated if item[0]]
        eligible_targets.sort(key=lambda item: (-item[1], item[2]))
        allowed_ids = {item[3].target_id for item in eligible_targets[:max_targets]}

        reports: List[DetectionReport] = []
        for eligible, confidence_percent, distance, target, x, y, z, azimuth, elevation, freq_mhz, signal_strength_db in evaluated:
            detected = eligible and target.target_id in allowed_ids
            update = self.state_machine.update(target.target_id, detected, target.timestamp_us, required_hits=required_hits)
            if not update.should_report and not update.just_lost:
                continue
            measured_azimuth = -1.0 if update.just_lost else (azimuth + _bounded_gauss(self.rng, direction_error_sigma, direction_error_max)) % 360.0
            reports.append(
                DetectionReport(
                    payload_type=PayloadType.RADIO,
                    target=target,
                    state=TrackState.LOST if update.just_lost else TrackState.TRACKING,
                    confidence=max(0.0, min(1.0, confidence_percent / 100.0)),
                    distance_m=0.0 if update.just_lost else distance,
                    azimuth_deg=measured_azimuth,
                    elevation_deg=elevation,
                    frequency_khz=int(round(freq_mhz * 1000.0)),
                    bandwidth_khz=bandwidth_khz,
                    signal_strength_db=int(round(signal_strength_db)),
                    station_id=station_id,
                    extra={"relative_enu_m": (x, y, z), "confidence_percent": confidence_percent},
                )
            )
        return reports

    @staticmethod
    def _signal_strength_db(distance: float, r_max: float) -> float:
        return linear_score(distance, 0.0, r_max, -35.0, -90.0)

    @staticmethod
    def _confidence_percent(target: TargetState, distance: float, r_max: float, signal_strength_db: float, band_match: bool) -> float:
        distance_score = linear_score(distance, 0.0, r_max, 1.0, 0.35)
        strength_score = max(0.0, min(1.0, (signal_strength_db + 100.0) / 70.0))
        band_score = 1.0 if band_match else 0.0
        type_factor = {"uav": 1.0, "manned_aircraft": 0.85, "unknown": 0.75, "vehicle": 0.65, "ship": 0.65, "bird": 0.35}.get(target.target_type.value, 0.75)
        return round(max(0.0, min(100.0, 100.0 * distance_score * strength_score * band_score * type_factor * target.quality)), 1)


@dataclass(slots=True)
class PayloadModelSuite:
    """一次性运行雷达、光电、无线电三类模型。"""

    config: MiddlewareConfig
    include_message_parse: bool = True
    radar_model: RadarModel = field(init=False)
    eo_model: ElectroOpticalModel = field(init=False)
    radio_model: RadioDetectionModel = field(init=False)

    def __post_init__(self) -> None:
        self.radar_model = RadarModel(self.config)
        self.eo_model = ElectroOpticalModel(self.config)
        self.radio_model = RadioDetectionModel(self.config)

    def process(self, targets: Iterable[TargetState]) -> List[DetectionReport]:
        target_list = list(targets)
        reports: List[DetectionReport] = []
        reports.extend(self.radar_model.process(target_list))
        reports.extend(self.eo_model.process(target_list))
        reports.extend(self.radio_model.process(target_list))
        if self.include_message_parse:
            reports.extend(self._message_parse_reports(target_list))
        return reports

    @staticmethod
    def _message_parse_reports(targets: Iterable[TargetState]) -> List[DetectionReport]:
        # 报文解析不是物理探测模型；这里保留它作为输出链路兼容项。
        reports: List[DetectionReport] = []
        for target in targets:
            reports.append(
                DetectionReport(
                    payload_type=PayloadType.MESSAGE_PARSE,
                    target=target,
                    state=TrackState.TRACKING,
                    confidence=max(0.0, min(1.0, target.quality)),
                    distance_m=distance_m_from_enu(target.x, target.y, target.z),
                    azimuth_deg=azimuth_deg_from_enu(target.x, target.y),
                    elevation_deg=elevation_deg_from_enu(target.x, target.y, target.z),
                )
            )
        return reports
