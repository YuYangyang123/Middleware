"""几何、时间、CRC、ID 等小工具。只依赖 Python 标准库。"""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from typing import Any, Iterable, Tuple

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


# @dataclass(slots=True)
@dataclass
class GeoPoint:
    lat: float
    lon: float
    alt: float


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def scale_int(value: float, scale: float, lower: int, upper: int) -> int:
    return int(clamp(round(value * scale), lower, upper))


def json_dumps_bytes(payload: Any) -> bytes:
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def stable_u32_id(text: str) -> int:
    """把字符串稳定映射成 uint32，保证同一目标每次航迹号一致。"""
    digest = hashlib.blake2s((text or "unknown").encode("utf-8"), digest_size=4).digest()
    return int.from_bytes(digest, "big", signed=False)


def now_ms() -> int:
    return int(time.time() * 1000)


def now_us() -> int:
    return int(time.time() * 1_000_000)


def timestamp_to_us(value: Any, mode: str = "auto") -> int:
    """把上游时间戳统一成微秒；无法识别时用系统当前时间。"""
    if value in (None, ""):
        return now_us()
    try:
        ts = float(value)
    except (TypeError, ValueError):
        return now_us()
    if mode == "ms":
        return int(ts * 1000)
    if mode == "s":
        return int(ts * 1_000_000)
    if mode == "us":
        return int(ts)
    # auto：根据数量级粗略判断秒/毫秒/微秒。
    if ts < 10_000_000_000:
        return int(ts * 1_000_000)
    if ts < 10_000_000_000_000:
        return int(ts * 1000)
    return int(ts)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * math.cos(lon)
    y = (n + alt_m) * cos_lat * math.sin(lon)
    z = (n * (1 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def ecef_to_enu(x: float, y: float, z: float, origin: GeoPoint) -> Tuple[float, float, float]:
    ox, oy, oz = geodetic_to_ecef(origin.lat, origin.lon, origin.alt)
    dx, dy, dz = x - ox, y - oy, z - oz
    lat0 = math.radians(origin.lat)
    lon0 = math.radians(origin.lon)
    sin_lat, cos_lat = math.sin(lat0), math.cos(lat0)
    sin_lon, cos_lon = math.sin(lon0), math.cos(lon0)
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def geodetic_to_enu(lat_deg: float, lon_deg: float, alt_m: float, origin: GeoPoint) -> Tuple[float, float, float]:
    return ecef_to_enu(*geodetic_to_ecef(lat_deg, lon_deg, alt_m), origin)


def azimuth_deg_from_enu(x_east: float, y_north: float) -> float:
    # 设计文档要求：方位角基于正北顺时针，范围 [0, 360)。
    return (math.degrees(math.atan2(x_east, y_north)) + 360.0) % 360.0


def elevation_deg_from_enu(x_east: float, y_north: float, z_up: float) -> float:
    return math.degrees(math.atan2(z_up, math.hypot(x_east, y_north)))


def distance_m_from_enu(x_east: float, y_north: float, z_up: float) -> float:
    return math.sqrt(x_east * x_east + y_north * y_north + z_up * z_up)


def velocity_from_heading_speed(heading_deg: float, horizontal_speed_mps: float, vertical_speed_mps: float = 0.0):
    h = math.radians(heading_deg % 360.0)
    return horizontal_speed_mps * math.sin(h), horizontal_speed_mps * math.cos(h), vertical_speed_mps


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def angle_in_ranges(angle: float, ranges: Iterable[Iterable[float]]) -> bool:
    """支持普通角度区间，也支持 [350, 10] 这种跨 0° 区间。"""
    a = angle % 360.0
    for item in ranges:
        vals = list(item)
        if len(vals) != 2:
            continue
        raw_start, raw_end = float(vals[0]), float(vals[1])
        if abs(raw_end - raw_start) >= 360.0:
            return True
        start, end = raw_start % 360.0, raw_end % 360.0
        if start <= end and start <= a <= end:
            return True
        if start > end and (a >= start or a <= end):
            return True
    return False


def linear_score(value: float, min_value: float, max_value: float, near_score: float = 1.0, far_score: float = 0.0) -> float:
    if max_value <= min_value:
        return near_score
    ratio = clamp((value - min_value) / (max_value - min_value), 0.0, 1.0)
    return near_score + (far_score - near_score) * ratio


def signed_angle_delta_deg(target_angle: float, reference_angle: float) -> float:
    return (target_angle - reference_angle + 180.0) % 360.0 - 180.0


def spherical_to_enu(range_m: float, azimuth_deg: float, elevation_deg: float) -> Tuple[float, float, float]:
    az = math.radians(azimuth_deg)
    el = math.radians(elevation_deg)
    horizontal = range_m * math.cos(el)
    return horizontal * math.sin(az), horizontal * math.cos(az), range_m * math.sin(el)


def target_type_code(target_type: str, payload: str) -> int:
    # 可按真实协议继续扩展；当前遵循“无人机 0x03，未识别 0x05”的约定。
    if target_type == "uav":
        return 0x03
    if target_type == "manned_aircraft":
        return 0x01 if payload != "radar" else 0x05
    return 0x05
