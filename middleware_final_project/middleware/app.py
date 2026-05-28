"""中间件主入口。

在线模式：监听上游仿真系统 TCP JSON，实时运行模型并转发到下游；
演示模式：读取本地 sample_input.json，跑通模型和输出编码，用于部署前自检。
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import threading
from pathlib import Path
from typing import Any, List

from .config import MiddlewareConfig
from .domain import OutputPacket, TargetState
from .models import PayloadModelSuite
from .network import ThreadingJsonTcpServer
from .output import FileMirrorSink, NetworkOutputRouter, OutputDispatcher
from .parser import parse_target_states

LOGGER = logging.getLogger(__name__)


class RealtimeMiddlewareService:
    """输入解析 -> 载荷模型 -> 输出编码 -> 下游转发 的实时流水线。"""

    def __init__(self, config: MiddlewareConfig, router: NetworkOutputRouter | None = None):
        self.config = config
        self.model_suite = PayloadModelSuite(config)
        self.dispatcher = OutputDispatcher.from_config(config)
        self.router = router or NetworkOutputRouter(config)
        self.lock = threading.Lock()  # 多 TCP 客户端并发时保护模型状态机。

    def handle_frame(self, frame: Any) -> None:
        try:
            targets: List[TargetState] = parse_target_states(frame, self.config)
        except Exception as exc:  # noqa: BLE001 - 在线服务不能因为坏帧退出
            LOGGER.warning("drop invalid upstream frame: %s", exc)
            return
        if not targets:
            return
        with self.lock:
            reports = self.model_suite.process(targets)
            packets = self.dispatcher.build_all(reports)
            self.router.send_all(packets)
        LOGGER.info("frame processed: targets=%d reports=%d packets=%d", len(targets), len(reports), len(packets))

    def close(self) -> None:
        self.router.close()


def run_online(config_path: str) -> None:
    """启动在线 TCP 监听服务。"""
    config = MiddlewareConfig.load(config_path)
    network_cfg = config.network
    host = str(network_cfg.get("input_tcp_host", "0.0.0.0"))
    port = int(network_cfg.get("input_tcp_port", 1102))
    service = RealtimeMiddlewareService(config)
    server = ThreadingJsonTcpServer((host, port), service.handle_frame)

    stop_event = threading.Event()

    def _stop(signum, frame):  # type: ignore[no-untyped-def]
        LOGGER.info("received signal %s, stopping...", signum)
        stop_event.set()
        server.shutdown()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    LOGGER.info("middleware online, listening on %s:%s", host, port)
    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        server.server_close()
        service.close()
        LOGGER.info("middleware stopped")


def run_demo(config_path: str, input_path: str, out_dir: str) -> List[Path]:
    """本地离线自检：不连下游，把输出报文写到 out_dir。"""
    config = MiddlewareConfig.load(config_path)
    with open(input_path, "r", encoding="utf-8") as f:
        frame = json.load(f)
    targets = parse_target_states(frame, config)
    model_suite = PayloadModelSuite(config)
    dispatcher = OutputDispatcher.from_config(config)

    # 重复投喂几帧，保证雷达/光电/无线电的 N_detect、截获延迟状态机能进入 TRACKING。
    reports = []
    repeat_frames = int(config.network.get("demo_repeat_frames", 5))
    for _ in range(max(1, repeat_frames)):
        reports = model_suite.process(targets)
    packets: List[OutputPacket] = dispatcher.build_all(reports)

    sink = FileMirrorSink(out_dir)
    paths: List[Path] = []
    for packet in packets:
        sink.send(packet)
        paths.append(Path(out_dir) / packet.name)
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description="realtime detection data conversion middleware")
    parser.add_argument("--config", default="config/default_config.json", help="配置文件路径")
    parser.add_argument("--demo", default="", help="离线自检输入 JSON；不填则进入在线监听模式")
    parser.add_argument("--out-dir", default="out", help="demo 模式输出目录")
    parser.add_argument("--log-level", default="INFO", help="日志级别，例如 INFO/DEBUG/WARNING")
    args = parser.parse_args()

    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO), format="%(asctime)s %(levelname)s %(name)s - %(message)s")

    if args.demo:
        paths = run_demo(args.config, args.demo, args.out_dir)
        for path in paths:
            print(path)
    else:
        run_online(args.config)


if __name__ == "__main__":
    main()
