"""上游 JSON 输入解析：字段校验、单位转换、目标 ID 生成、WGS84→ENU。"""

from __future__ import annotations

import itertools
import json
import math
from typing import Any, Dict, Iterable, List

from .config import MiddlewareConfig
from .domain import TargetState, TargetType
from .utils import geodetic_to_enu, timestamp_to_us, velocity_from_heading_speed

_AUTO_ID_COUNTER = itertools.count(1)


def _to_float(value: Any, default: float = 0.0) -> float:
    if value in (None, ""):
        return default
    return float(value)


def _as_target_type(value: str) -> TargetType:
    try:
        return TargetType(value)
    except ValueError:
        return TargetType.UNKNOWN


def choose_target_id(header: Dict[str, Any], body: Dict[str, Any]) -> str:
    """目标 ID 优先级：ACCode > FlightID > MessageId > 内部自增 ID。"""
    for key in ("ACCode", "FlightID"):
        value = str(body.get(key, "")).strip()
        if value:
            return value
    message_id = str(header.get("MessageId", "")).strip()
    if message_id:
        return message_id
    return f"AUTO-{next(_AUTO_ID_COUNTER):06d}"


def validate_message(message: Dict[str, Any]) -> None:
    """只做输入端必要校验；不合格直接抛 ValueError，由服务端记录后丢弃。"""
    if not isinstance(message, dict):
        raise ValueError("message must be a JSON object")
    header = message.get("header")
    body = message.get("body")
    if not isinstance(header, dict):
        raise ValueError("missing or invalid header object")
    if not isinstance(body, dict):
        raise ValueError("missing or invalid body object")
    for field in ["Latitude", "Longitude", "Altitude", "Heading", "Pitch", "Roll"]:
        if field not in body:
            raise ValueError(f"missing required body field: {field}")
        try:
            float(body[field])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"body.{field} must be numeric") from exc
    lat = float(body["Latitude"])
    lon = float(body["Longitude"])
    if not -90 <= lat <= 90:
        raise ValueError("body.Latitude must be in [-90, 90]")
    if not -180 <= lon <= 180:
        raise ValueError("body.Longitude must be in [-180, 180]")
    for speed_field in ("GS", "TAS"):
        if body.get(speed_field) in (None, ""):
            continue
        if float(body[speed_field]) < 0:
            raise ValueError(f"body.{speed_field} must be >= 0")


def quality_score(message: Dict[str, Any]) -> float:
    body = message.get("body", {})
    score = 1.0
    for optional in ("Callsign", "Reg", "NextPoint"):
        if not str(body.get(optional, "")).strip():
            score -= 0.03
    if not body.get("GS") and not body.get("TAS"):
        score -= 0.08
    return max(0.0, min(1.0, round(score, 3)))


def parse_target_state(message: bytes | str | Dict[str, Any], config: MiddlewareConfig) -> TargetState:
    """把单条外部 JSON 消息转成内部 TargetState。"""
    if isinstance(message, bytes):
        message = message.decode("utf-8")
    obj = json.loads(message) if isinstance(message, str) else message
    validate_message(obj)

    header = obj.get("header", {})
    body = obj.get("body", {})

    lat = _to_float(body.get("Latitude"))
    lon = _to_float(body.get("Longitude"))
    alt = _to_float(body.get("Altitude"))
    heading = _to_float(body.get("Heading")) % 360.0
    pitch = _to_float(body.get("Pitch"))
    roll = _to_float(body.get("Roll"))
    gs_mps = _to_float(body.get("GS")) / 3.6       # km/h → m/s
    tas_mps = _to_float(body.get("TAS")) * 0.514444  # kt → m/s

    horizontal_speed = gs_mps if gs_mps > 0 else tas_mps * math.cos(math.radians(pitch))
    vertical_speed = tas_mps * math.sin(math.radians(pitch)) if tas_mps > 0 else 0.0
    vx, vy, vz = velocity_from_heading_speed(heading, horizontal_speed, vertical_speed)
    x, y, z = geodetic_to_enu(lat, lon, alt, config.origin)

    return TargetState(
        target_id=choose_target_id(header, body),
        source_type=str(header.get("MessageType") or "MannedAircraft"),
        timestamp_us=timestamp_to_us(header.get("Timestamp"), config.timestamp_mode),
        lat=lat,
        lon=lon,
        alt=alt,
        x=x,
        y=y,
        z=z,
        vx=vx,
        vy=vy,
        vz=vz,
        heading=heading,
        pitch=pitch,
        roll=roll,
        target_type=_as_target_type(config.target_type_default),
        quality=quality_score(obj),
        raw=obj,
        target_name=str(body.get("Callsign") or body.get("FlightID") or "unknown"),
        registration=str(body.get("Reg") or ""),
        accode=str(body.get("ACCode") or ""),
        flight_id=str(body.get("FlightID") or ""),
        next_point=str(body.get("NextPoint") or ""),
        ground_speed_mps=gs_mps,
        tas_mps=tas_mps,
    )


def parse_target_states(frame: Any, config: MiddlewareConfig) -> List[TargetState]:
    """支持单目标 JSON，也支持一个 TCP 帧里放目标数组。"""
    if isinstance(frame, list):
        messages: Iterable[Any] = frame
    elif isinstance(frame, dict) and isinstance(frame.get("targets"), list):
        # 预留兼容：如果上游以后采用 {"targets": [...]} 也能处理。
        messages = frame["targets"]
    else:
        messages = [frame]
    return [parse_target_state(item, config) for item in messages]
