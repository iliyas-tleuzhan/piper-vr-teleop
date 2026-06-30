#!/usr/bin/env python3
"""Print Quest visualization UDP packets for local debugging."""

from __future__ import annotations

import argparse
import json
import socket


def main() -> int:
    parser = argparse.ArgumentParser(description="Receive Piper Quest visualization UDP packets")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5055)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    print(f"Listening for Piper visualization packets on {args.host}:{args.port}")
    while True:
        data, address = sock.recvfrom(65535)
        text = data.decode("utf-8", errors="replace")
        try:
            packet = json.loads(text)
            text = json.dumps(packet, indent=2, sort_keys=True)
        except json.JSONDecodeError:
            pass
        print(f"\n[{address[0]}:{address[1]}]\n{text}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
