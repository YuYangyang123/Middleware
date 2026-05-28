"""实时 TCP 输入监听。

TCP 是字节流，没有天然“每帧边界”。本文件用 JSONDecoder 做增量解析，
支持：
1. 推荐格式：一行一个 JSON；
2. 兼容格式：多个 JSON 对象连续发送，中间只有空白字符。
"""

from __future__ import annotations

import json
import logging
import socketserver
from typing import Any, Callable, List

LOGGER = logging.getLogger(__name__)


class JsonStreamFramer:
    """把 TCP 字节流切成一个个 JSON 对象。"""

    def __init__(self, max_buffer_chars: int = 2_000_000):
        self.decoder = json.JSONDecoder()
        self.buffer = ""
        self.max_buffer_chars = max_buffer_chars

    def feed(self, data: bytes) -> List[Any]:
        self.buffer += data.decode("utf-8", errors="replace")
        objects: List[Any] = []
        while True:
            self.buffer = self.buffer.lstrip()
            if not self.buffer:
                break
            try:
                obj, index = self.decoder.raw_decode(self.buffer)
            except json.JSONDecodeError:
                # 多数情况下是 JSON 还没收全，继续等下一段 TCP 数据。
                if len(self.buffer) > self.max_buffer_chars:
                    LOGGER.warning("drop oversized/invalid tcp input buffer, chars=%d", len(self.buffer))
                    self.buffer = ""
                break
            objects.append(obj)
            self.buffer = self.buffer[index:]
        return objects


class JsonTcpHandler(socketserver.BaseRequestHandler):
    """每个上游 TCP 连接对应一个 handler。"""

    on_frame: Callable[[Any], None]
    recv_size: int = 65536

    def handle(self) -> None:
        peer = self.client_address
        framer = JsonStreamFramer()
        LOGGER.info("upstream connected: %s", peer)
        try:
            while True:
                data = self.request.recv(self.recv_size)
                if not data:
                    break
                for frame in framer.feed(data):
                    self.on_frame(frame)
        finally:
            LOGGER.info("upstream disconnected: %s", peer)


class ThreadingJsonTcpServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address, on_frame: Callable[[Any], None]):
        handler_cls = type("ConfiguredJsonTcpHandler", (JsonTcpHandler,), {"on_frame": staticmethod(on_frame)})
        super().__init__(server_address, handler_cls)
