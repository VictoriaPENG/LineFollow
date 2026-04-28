#!/usr/bin/env python3
"""Save JPEG frames from the web debug dashboard MJPEG streams."""

from __future__ import annotations

import argparse
import datetime as dt
import os
import time
import urllib.error
import urllib.request


DEFAULT_STREAMS = ("source", "detect", "binary", "curve")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Save current frames from /stream/*.mjpg on the line-follow web dashboard."
    )
    parser.add_argument("--host", default="127.0.0.1", help="Dashboard host or board IP.")
    parser.add_argument("--port", type=int, default=8091, help="Dashboard HTTP port.")
    parser.add_argument(
        "--streams",
        nargs="+",
        default=list(DEFAULT_STREAMS),
        help="Stream names to save, for example: source detect binary curve.",
    )
    parser.add_argument(
        "--out-dir",
        default="web_debug_frames",
        help="Root output directory for saved frames.",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=5.0,
        help="HTTP timeout for each stream.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="How many snapshots to save. Use values >1 for repeated capture.",
    )
    parser.add_argument(
        "--interval-sec",
        type=float,
        default=1.0,
        help="Delay between repeated snapshots.",
    )
    return parser.parse_args()


def read_one_jpeg(url: str, timeout_sec: float) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout_sec) as response:
        content_type = response.headers.get("Content-Type", "")
        if "multipart/x-mixed-replace" not in content_type:
            data = response.read()
            if data.startswith(b"\xff\xd8") and data.endswith(b"\xff\xd9"):
                return data
            raise RuntimeError(f"unexpected content type: {content_type}")

        buffer = b""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            chunk = response.read(4096)
            if not chunk:
                break
            buffer += chunk
            start = buffer.find(b"\xff\xd8")
            end = buffer.find(b"\xff\xd9", start + 2 if start >= 0 else 0)
            if start >= 0 and end >= 0:
                return buffer[start : end + 2]

    raise TimeoutError(f"no JPEG frame received from {url}")


def save_snapshot(args: argparse.Namespace, index: int) -> None:
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    if args.count > 1:
        stamp = f"{stamp}_{index:03d}"
    out_dir = os.path.join(args.out_dir, stamp)
    os.makedirs(out_dir, exist_ok=True)

    for stream in args.streams:
        url = f"http://{args.host}:{args.port}/stream/{stream}.mjpg"
        try:
            frame = read_one_jpeg(url, args.timeout_sec)
        except (TimeoutError, RuntimeError, urllib.error.URLError) as exc:
            print(f"skip {stream}: {exc}")
            continue

        path = os.path.join(out_dir, f"{stream}.jpg")
        with open(path, "wb") as fp:
            fp.write(frame)
        print(f"saved {stream}: {path}")


def main() -> int:
    args = parse_args()
    for index in range(max(1, args.count)):
        save_snapshot(args, index)
        if index + 1 < args.count:
            time.sleep(max(0.0, args.interval_sec))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
