"""配置读取。实验室部署时通常只需要改 config/default_config.json。"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict

from .utils import GeoPoint


@dataclass
class MiddlewareConfig:
    timestamp_mode: str = "auto"
    target_type_default: str = "manned_aircraft"
    origin: GeoPoint = field(default_factory=lambda: GeoPoint(31.123456, 120.123456, 0.0))
    radar: Dict[str, Any] = field(default_factory=dict)
    electro_optical: Dict[str, Any] = field(default_factory=dict)
    radio: Dict[str, Any] = field(default_factory=dict)
    message_parse: Dict[str, Any] = field(default_factory=dict)
    network: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MiddlewareConfig":
        origin_data = data.get("origin", {})
        origin = GeoPoint(
            lat=float(origin_data.get("lat", 31.123456)),
            lon=float(origin_data.get("lon", 120.123456)),
            alt=float(origin_data.get("alt", 0.0)),
        )
        return cls(
            timestamp_mode=data.get("timestamp_mode", "auto"),
            target_type_default=data.get("target_type_default", "manned_aircraft"),
            origin=origin,
            radar=data.get("radar", {}),
            electro_optical=data.get("electro_optical", {}),
            radio=data.get("radio", {}),
            message_parse=data.get("message_parse", {}),
            network=data.get("network", {}),
        )

    @classmethod
    def load(cls, path: str | Path) -> "MiddlewareConfig":
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(json.load(f))
