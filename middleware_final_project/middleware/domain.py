"""中间件内部统一数据对象。

外部 JSON 先进 TargetState，雷达/光电/无线电模型统一读取 TargetState，
输出端再把 DetectionReport 编码成对应协议报文。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Tuple


class TargetType(str, Enum):
    MANNED_AIRCRAFT = "manned_aircraft"
    UAV = "uav"
    SHIP = "ship"
    VEHICLE = "vehicle"
    BIRD = "bird"
    UNKNOWN = "unknown"


class PayloadType(str, Enum):
    RADAR = "radar"
    EO = "electro_optical"
    RADIO = "radio"
    MESSAGE_PARSE = "message_parse"


class TrackState(str, Enum):
    INIT = "INIT"
    CANDIDATE = "CANDIDATE"
    TRACKING = "TRACKING"
    LOST_PENDING = "LOST_PENDING"
    LOST = "LOST"


@dataclass
class TargetState:
    """统一目标状态。

    x/y/z 是相对全局配置 origin 的 ENU 坐标，单位 m；
    vx/vy/vz 是 ENU 速度，单位 m/s。
    """

    target_id: str
    source_type: str
    timestamp_us: int
    lat: float
    lon: float
    alt: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    heading: float
    pitch: float
    roll: float
    target_type: TargetType = TargetType.UNKNOWN
    quality: float = 1.0
    raw: Dict[str, Any] = field(default_factory=dict)

    target_name: str = ""
    registration: str = ""
    accode: str = ""
    flight_id: str = ""
    next_point: str = ""
    ground_speed_mps: float = 0.0
    tas_mps: float = 0.0

    def horizontal_speed_mps(self) -> float:
        return (self.vx**2 + self.vy**2) ** 0.5

    def speed_mps(self) -> float:
        return (self.vx**2 + self.vy**2 + self.vz**2) ** 0.5


@dataclass
class DetectionReport:
    """载荷模型输出的中性报告，后续由输出端编码成具体协议。"""

    payload_type: PayloadType
    target: TargetState
    state: TrackState = TrackState.TRACKING
    confidence: float = 1.0
    distance_m: float = 0.0
    azimuth_deg: float = 0.0
    elevation_deg: float = 0.0
    snr_db: float = 30.0
    rcs_sqm: float = 0.01
    frequency_khz: int = 2400000
    bandwidth_khz: int = 20000
    signal_strength_db: int = -50
    station_id: int = 1
    channel_id: int = 1
    offset_h_mrad: float = 0.0
    offset_v_mrad: float = 0.0
    rect: Tuple[int, int, int, int] = (300, 220, 80, 60)
    extra: Dict[str, Any] = field(default_factory=dict)

    @property
    def is_lost(self) -> bool:
        return self.state == TrackState.LOST


@dataclass
class OutputPacket:
    """已经编码完成、等待发送的输出报文。"""

    name: str
    protocol: str
    data: bytes
    content_type: str = "application/octet-stream"
    metadata: Dict[str, Any] = field(default_factory=dict)
