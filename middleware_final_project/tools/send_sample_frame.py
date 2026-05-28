"""本地联调用：把 examples/sample_input.json 按 1Hz 发给中间件监听端口。"""

from __future__ import annotations

import argparse
import json
import socket
import time
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="send sample JSON frames to middleware TCP input")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=1102)
    parser.add_argument("--input", default="examples/sample_input.json")
    parser.add_argument("--count", type=int, default=6)
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()

    frame = json.loads(Path(args.input).read_text(encoding="utf-8"))
    with socket.create_connection((args.host, args.port), timeout=3.0) as sock:
        for i in range(args.count):
            frame["header"]["Timestamp"] = int(time.time() * 1000)
            # 加换行是推荐方式；服务端也兼容多个 JSON 直接连续发送。
            sock.sendall((json.dumps(frame, ensure_ascii=False) + "\n").encode("utf-8"))
            print(f"sent frame {i + 1}/{args.count}")
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
