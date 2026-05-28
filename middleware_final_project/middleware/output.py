"""输出协议编码与下游转发。

编码职责：DetectionReport -> OutputPacket；
发送职责：OutputPacket -> 配置指定的 TCP/UDP 下游软件。
"""

from __future__ import annotations

import json
import logging
import socket
import struct
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional

from .config import MiddlewareConfig
from .domain import DetectionReport, OutputPacket, PayloadType
from .utils import clamp, crc16_modbus, json_dumps_bytes, now_ms, now_us, scale_int, stable_u32_id, target_type_code

LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class RadarFrameBuilder:
    """雷达点迹 TCP 二进制帧。"""

    frame_id: int = 0
    processing_frame_id: int = 0
    endian: str = "<"
    header_word: int = 0x55AA55AA
    command_track_upload: int = 0x00030002

    def build(self, reports: Iterable[DetectionReport]) -> OutputPacket:
        radar_reports = [r for r in reports if r.payload_type == PayloadType.RADAR and not r.is_lost]
        self.frame_id = (self.frame_id + 1) & 0xFFFFFFFF
        self.processing_frame_id = (self.processing_frame_id + 1) & 0xFFFFFFFF
        body = struct.pack(
            self.endian + "IIQIQIIH",
            self.command_track_upload,
            self.frame_id,
            now_us(),
            self.processing_frame_id,
            now_ms(),
            0,
            3_600_000,
            len(radar_reports),
        )
        for report in radar_reports:
            t = report.target
            measured_enu = report.extra.get("measurement_enu_m") if isinstance(report.extra, dict) else None
            x_m, y_m, z_m = measured_enu if measured_enu and len(measured_enu) == 3 else (t.x, t.y, t.z)
            body += struct.pack(
                self.endian + "iiiiiiIhiB3x",
                scale_int(x_m, 100.0, -2_147_483_648, 2_147_483_647),
                scale_int(y_m, 100.0, -2_147_483_648, 2_147_483_647),
                scale_int(z_m, 100.0, -2_147_483_648, 2_147_483_647),
                scale_int(t.vx, 100.0, -2_147_483_648, 2_147_483_647),
                scale_int(t.vy, 100.0, -2_147_483_648, 2_147_483_647),
                scale_int(t.vz, 100.0, -2_147_483_648, 2_147_483_647),
                stable_u32_id(t.target_id),
                scale_int(report.snr_db, 100.0, -32768, 32767),
                scale_int(report.rcs_sqm, 100.0, -2_147_483_648, 2_147_483_647),
                target_type_code(t.target_type.value, "radar"),
            )
        crc = crc16_modbus(body)
        payload = body + struct.pack(self.endian + "H", crc)
        frame = struct.pack(self.endian + "II", self.header_word, len(payload)) + payload
        return OutputPacket("radar_track_upload_1001.bin", "TCP/BINARY", frame, metadata={"target_count": len(radar_reports), "frame_id": self.frame_id})




@dataclass(slots=True)
class RadarStatusFrameBuilder:
    """雷达状态数据上传 1002。

    设计文档明确 1002 为“雷达状态数据上传/TCP/周期性输出设备正常或异常状态”，
    但没有像 1001 点迹那样给出完整二进制字段表。因此这里采用与雷达点迹一致的
    0x55AA55AA 帧头、帧长度、命令、帧 ID、时间戳、CRC16-MODBUS 结构，
    状态字段保持最小化：0=正常，1=异常；后续拿到厂家接口表时只需替换本 builder。
    """

    frame_id: int = 0
    endian: str = "<"
    header_word: int = 0x55AA55AA
    command_status_upload: int = 0x00030003
    device_id: int = 1

    def build(self, work_status: int = 0) -> OutputPacket:
        self.frame_id = (self.frame_id + 1) & 0xFFFFFFFF
        body = struct.pack(
            self.endian + "IIQIB3x",
            self.command_status_upload,
            self.frame_id,
            now_us(),
            int(self.device_id),
            int(work_status),
        )
        crc = crc16_modbus(body)
        payload = body + struct.pack(self.endian + "H", crc)
        frame = struct.pack(self.endian + "II", self.header_word, len(payload)) + payload
        return OutputPacket(
            "radar_status_upload_1002.bin",
            "TCP/BINARY",
            frame,
            metadata={"frame_id": self.frame_id, "work_status": work_status},
        )

@dataclass(slots=True)
class ElectroOpticalUdpBuilder:
    """光电 UDP 头 + JSON。"""

    message_id: int = 0
    channel_id: int = 1
    endian: str = ">"

    def build_ivp_report(self, reports: Iterable[DetectionReport]) -> OutputPacket:
        eo_reports = [r for r in reports if r.payload_type == PayloadType.EO and not r.is_lost]
        self.message_id = (self.message_id + 1) & 0xFFFFFFFF
        payload = {
            "cmd": "ivpReport",
            "channelId": self.channel_id,
            "timeStamp": now_ms(),
            "type": "ALARM_INPUT_IVP_OBJECT_DET",
            "targets": [self._target_json(r) for r in eo_reports],
        }
        json_bytes = json_dumps_bytes(payload)
        header = struct.pack(self.endian + "IHHI", self.message_id, 1, 0, len(json_bytes))
        return OutputPacket("eo_ivpReport_1004.udp", "UDP/JSON", header + json_bytes, "application/json+udp", {"target_count": len(eo_reports), "message_id": self.message_id})

    def build_ptz_status_report(self) -> OutputPacket:
        self.message_id = (self.message_id + 1) & 0xFFFFFFFF
        payload = {"cmd": "PTZStatusReport", "channelId": self.channel_id, "timeStamp": now_ms(), "pan": 0.0, "tilt": 0.0, "zoom": 1.0, "status": "NORMAL"}
        json_bytes = json_dumps_bytes(payload)
        header = struct.pack(self.endian + "IHHI", self.message_id, 1, 0, len(json_bytes))
        return OutputPacket("eo_PTZStatusReport_1005.udp", "UDP/JSON", header + json_bytes, "application/json+udp", {"message_id": self.message_id})

    @staticmethod
    def _target_json(report: DetectionReport) -> dict:
        t = report.target
        x, y, w, h = report.rect
        x = int(clamp(x, 0, 639))
        y = int(clamp(y, 0, 511))
        w = int(clamp(w, 1, 640 - x))
        h = int(clamp(h, 1, 512 - y))
        return {
            "ID": stable_u32_id(t.target_id),
            "targetType": target_type_code(t.target_type.value, "eo"),
            "confidence": round(clamp(report.confidence, 0.0, 1.0), 3),
            "distance": round(report.distance_m, 2),
            "offsetH": round(report.offset_h_mrad, 3),
            "offsetV": round(report.offset_v_mrad, 3),
            "GPS": [round(t.lon, 7), round(t.lat, 7)],
            "altitude": round(t.alt, 2),
            "rect": {"x": x, "y": y, "width": w, "height": h},
        }


def _varstr(text: str) -> bytes:
    encoded = (text or "").encode("utf-8")[:65535]
    return struct.pack(">H", len(encoded)) + encoded




@dataclass(slots=True)
class RadioDeviceFrameBuilder:
    """无线电侦测设备数据 1010。

    设计文档要求 1010 周期输出“设备位置与工作状态”，且无线电链路采用
    0xEEEEEEEE/0xAAAAAAAA 的 TCP 二进制帧。这里沿用无线电目标帧的起止标志、
    版本号、帧长度、时间戳、包编号结构，数据区放置监测站 ID、经纬高、工作状态。
    """

    packet_id: int = 0
    station_id: int = 1
    lon: float = 0.0
    lat: float = 0.0
    alt: float = 0.0
    start_word: int = 0xEEEEEEEE
    end_word: int = 0xAAAAAAAA
    major_version: int = 2
    minor_version: int = 3
    endian: str = ">"

    def build_device_frame(self, target_count: int = 0, work_status: int = 0) -> OutputPacket:
        self.packet_id = (self.packet_id + 1) & 0xFFFFFFFF
        # message_type=1010 方便下游在同一 TCP 通道中区分设备数据和目标数据。
        body = struct.pack(
            self.endian + "BBQIIIddfBH",
            self.major_version,
            self.minor_version,
            now_ms(),
            self.packet_id,
            1010,
            int(self.station_id),
            float(self.lon),
            float(self.lat),
            float(self.alt),
            int(work_status),
            int(max(0, target_count)),
        )
        frame_length = 4 + 4 + len(body) + 4
        frame = struct.pack(self.endian + "II", self.start_word, frame_length) + body + struct.pack(self.endian + "I", self.end_word)
        return OutputPacket(
            "radio_device_1010.bin",
            "TCP/BINARY",
            frame,
            metadata={"packet_id": self.packet_id, "station_id": self.station_id, "work_status": work_status, "target_count": target_count},
        )

@dataclass(slots=True)
class RadioTargetFrameBuilder:
    """无线电侦测 TCP 二进制帧。"""

    packet_id: int = 0
    start_word: int = 0xEEEEEEEE
    end_word: int = 0xAAAAAAAA
    major_version: int = 2
    minor_version: int = 3
    endian: str = ">"

    def build_target_frame(self, reports: Iterable[DetectionReport]) -> OutputPacket:
        radio_reports = [r for r in reports if r.payload_type == PayloadType.RADIO]
        self.packet_id = (self.packet_id + 1) & 0xFFFFFFFF
        body = struct.pack(self.endian + "BBQIH", self.major_version, self.minor_version, now_ms(), self.packet_id, len(radio_reports))
        for report in radio_reports:
            body += self._target_record(report)
        frame_length = 4 + 4 + len(body) + 4
        frame = struct.pack(self.endian + "II", self.start_word, frame_length) + body + struct.pack(self.endian + "I", self.end_word)
        return OutputPacket("radio_target_1011.bin", "TCP/BINARY", frame, metadata={"target_count": len(radio_reports), "packet_id": self.packet_id})

    def _target_record(self, report: DetectionReport) -> bytes:
        t = report.target
        target_name = "" if report.is_lost else (t.target_name or t.target_id)
        unique_id = t.accode or t.flight_id or t.target_id
        fixed = struct.pack(
            self.endian + "IIfddfIIhBB",
            stable_u32_id(t.target_id),
            int(report.station_id),
            float(report.azimuth_deg if not report.is_lost else -1.0),
            float(t.lon),
            float(t.lat),
            float(t.alt),
            int(max(0, report.frequency_khz)),
            int(max(0, report.bandwidth_khz)),
            int(clamp(report.signal_strength_db, -32768, 32767)),
            int(clamp(round(report.confidence * 100), 0, 100)),
            1,
        )
        distance = struct.pack(self.endian + "f", float(report.distance_m if not report.is_lost else 0.0))
        return fixed + distance + _varstr(unique_id) + _varstr(target_name) + _varstr("OFDM")


@dataclass(slots=True)
class MessageParseJsonBuilder:
    """报文解析 JSON 输出，作为无线电链路的兼容输出。"""

    sn: str = "SIM-PARSER-001"
    method: str = "tracer_heart"

    def build_device_osd(self) -> OutputPacket:
        payload = {"tid": str(uuid.uuid4()), "bid": str(uuid.uuid4()), "timestamp": now_ms(), "timeStamp": now_ms(), "method": self.method, "sn": self.sn, "status": "NORMAL", "workMode": "SIMULATED"}
        return OutputPacket("message_device_1012.json", "TOPIC/JSON", json_dumps_bytes(payload), "application/json")

    def build_target_json(self, reports: Iterable[DetectionReport]) -> OutputPacket:
        parse_reports = [r for r in reports if r.payload_type == PayloadType.MESSAGE_PARSE and not r.is_lost]
        payload = {"tid": str(uuid.uuid4()), "bid": str(uuid.uuid4()), "timestamp": now_ms(), "timeStamp": now_ms(), "method": self.method, "sn": self.sn, "uavItemList": [self._uav_item(r) for r in parse_reports]}
        return OutputPacket("message_parse_1013.json", "TOPIC/JSON", json_dumps_bytes(payload), "application/json", {"target_count": len(parse_reports)})

    @staticmethod
    def _uav_item(report: DetectionReport) -> dict:
        t = report.target
        return {
            "droneName": t.target_name or "unknown",
            "serialNum": t.target_id,
            "direction": round(report.azimuth_deg, 2),
            "speed": round(t.horizontal_speed_mps(), 2),
            "verticalSpeed": round(t.vz, 2),
            "height": round(t.alt, 2),
            "geodeticAltitude": round(t.alt, 2),
            "operatorAltitude": round(t.alt, 2),
            "longitude": round(t.lon, 7),
            "latitude": round(t.lat, 7),
            "pilotLongitude": 0.0,
            "pilotLatitude": 0.0,
            "homeLongitude": 0.0,
            "homeLatitude": 0.0,
            "pressureAltitude": round(t.alt, 2),
            "aliveTime": 0,
            "targetMask": 0,
        }


@dataclass(slots=True)
class OutputDispatcher:
    """汇总所有输出报文。"""

    radar_builder: RadarFrameBuilder = field(default_factory=RadarFrameBuilder)
    radar_status_builder: RadarStatusFrameBuilder = field(default_factory=RadarStatusFrameBuilder)
    eo_builder: ElectroOpticalUdpBuilder = field(default_factory=ElectroOpticalUdpBuilder)
    radio_device_builder: RadioDeviceFrameBuilder = field(default_factory=RadioDeviceFrameBuilder)
    radio_builder: RadioTargetFrameBuilder = field(default_factory=RadioTargetFrameBuilder)
    message_builder: MessageParseJsonBuilder = field(default_factory=MessageParseJsonBuilder)

    @classmethod
    def from_config(cls, config: MiddlewareConfig) -> "OutputDispatcher":
        obj = cls()
        obj.radar_status_builder.device_id = int(config.radar.get("device_id", 1))
        obj.eo_builder.channel_id = int(config.electro_optical.get("channel_id", 1))
        radio_site = config.radio.get("site") or {}
        obj.radio_device_builder.station_id = int(config.radio.get("station_id", 1))
        obj.radio_device_builder.lon = float(radio_site.get("lon", config.origin.lon))
        obj.radio_device_builder.lat = float(radio_site.get("lat", config.origin.lat))
        obj.radio_device_builder.alt = float(radio_site.get("alt", config.origin.alt))
        obj.message_builder.sn = str(config.message_parse.get("sn", "SIM-PARSER-001"))
        obj.message_builder.method = str(config.message_parse.get("method", "tracer_heart"))
        return obj

    def build_all(self, reports: Iterable[DetectionReport]) -> List[OutputPacket]:
        report_list = list(reports)
        radio_target_count = sum(1 for r in report_list if r.payload_type == PayloadType.RADIO and not r.is_lost)
        return [
            self.radar_builder.build(report_list),
            self.radar_status_builder.build(work_status=0),
            self.eo_builder.build_ivp_report(report_list),
            self.eo_builder.build_ptz_status_report(),
            self.radio_device_builder.build_device_frame(target_count=radio_target_count, work_status=0),
            self.radio_builder.build_target_frame(report_list),
            self.message_builder.build_device_osd(),
            self.message_builder.build_target_json(report_list),
        ]


class TcpPersistentSender:
    """TCP 下游发送器：断线后下次发送自动重连。"""

    def __init__(self, host: str, port: int, timeout_s: float = 3.0):
        self.address = (host, int(port))
        self.timeout_s = timeout_s
        self.sock: Optional[socket.socket] = None

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def _connect(self) -> socket.socket:
        if self.sock is None:
            self.sock = socket.create_connection(self.address, timeout=self.timeout_s)
        return self.sock

    def send(self, data: bytes) -> None:
        try:
            self._connect().sendall(data)
        except OSError:
            # 连接异常时关闭后重试一次，避免下游重启导致中间件永久失效。
            self.close()
            self._connect().sendall(data)


class UdpSender:
    """UDP 下游发送器。"""

    def __init__(self, host: str, port: int):
        self.address = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def close(self) -> None:
        self.sock.close()

    def send(self, data: bytes) -> None:
        self.sock.sendto(data, self.address)


class FileMirrorSink:
    """可选：把输出报文镜像到本地目录，便于现场抓包前先看文件。"""

    def __init__(self, out_dir: str | Path):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)

    def send(self, packet: OutputPacket) -> None:
        (self.out_dir / packet.name).write_bytes(packet.data)


class NetworkOutputRouter:
    """按 packet.name 把报文转发到配置里的下游地址。"""

    def __init__(self, config: MiddlewareConfig):
        network = config.network
        self.senders: Dict[str, object] = {}
        self.mirror = FileMirrorSink(network["mirror_output_dir"]) if network.get("mirror_output_dir") else None
        outputs = network.get("outputs", {})
        for packet_name, route in outputs.items():
            if not route.get("enabled", True):
                continue
            protocol = str(route.get("protocol", "")).lower()
            host = str(route.get("host", "127.0.0.1"))
            port = int(route.get("port", 0))
            if protocol == "tcp":
                self.senders[packet_name] = TcpPersistentSender(host, port, float(route.get("timeout_s", 3.0)))
            elif protocol == "udp":
                self.senders[packet_name] = UdpSender(host, port)
            elif protocol == "file":
                self.senders[packet_name] = FileMirrorSink(route.get("dir", "out"))
            else:
                LOGGER.warning("unknown output protocol for %s: %s", packet_name, protocol)

    def send_all(self, packets: Iterable[OutputPacket]) -> None:
        for packet in packets:
            if self.mirror:
                self.mirror.send(packet)
            sender = self.senders.get(packet.name)
            if sender is None:
                continue
            try:
                sender.send(packet.data) if not isinstance(sender, FileMirrorSink) else sender.send(packet)
                LOGGER.info("sent %-28s bytes=%d meta=%s", packet.name, len(packet.data), packet.metadata)
            except OSError as exc:
                # 输出失败不应导致输入监听退出，实验室联调时下游经常会先后启动。
                LOGGER.warning("send failed: packet=%s error=%s", packet.name, exc)

    def close(self) -> None:
        for sender in self.senders.values():
            close = getattr(sender, "close", None)
            if close:
                close()
