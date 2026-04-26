#!/usr/bin/env python3

import argparse
import shutil
import subprocess
import time
from typing import Iterable


DEFAULT_SERVICES = [
    "line-follow-hybrid.service",
]

DEFAULT_NODES = [
    "/line_follow_angle_node",
    "/line_follow_motor_model_node",
    "/motor_driver_control_node",
    "/joystick_drive_node",
    "/remote_long_press_start_line_follow_node",
    "/web_debug_dashboard_node",
]

DEFAULT_TOPICS = [
    "/line_follow/line_angle_deg",
    "/motor_speed_cmd",
    "/joystick_override_active",
    "/joystick_motor_speed_cmd",
]


def run_command(cmd: list[str]) -> tuple[int, str]:
    try:
        result = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return 127, f"command not found: {cmd[0]}"

    output = (result.stdout or "").strip()
    error = (result.stderr or "").strip()
    if output:
        return result.returncode, output
    if error:
        return result.returncode, error
    return result.returncode, ""


def print_section(title: str) -> None:
    print(f"\n=== {title} ===")


def check_services(services: Iterable[str]) -> None:
    print_section("systemd services")
    for service in services:
        code, text = run_command(["systemctl", "is-active", service])
        state = text if text else f"exit={code}"
        print(f"{service}: {state}")


def get_node_set() -> set[str]:
    code, text = run_command(["ros2", "node", "list"])
    if code != 0:
        raise RuntimeError(text or "ros2 node list failed")
    return {line.strip() for line in text.splitlines() if line.strip()}


def check_nodes(nodes: Iterable[str]) -> None:
    print_section("ros2 nodes")
    node_set = get_node_set()
    for node in nodes:
        print(f"{node}: {'ok' if node in node_set else 'missing'}")

    print("\nall nodes:")
    for node in sorted(node_set):
        print(node)


def check_topic_presence(topics: Iterable[str]) -> None:
    print_section("ros2 topics")
    code, text = run_command(["ros2", "topic", "list"])
    if code != 0:
        raise RuntimeError(text or "ros2 topic list failed")

    topic_set = {line.strip() for line in text.splitlines() if line.strip()}
    for topic in topics:
        print(f"{topic}: {'ok' if topic in topic_set else 'missing'}")


def check_topic_detail(topic: str) -> None:
    code, info = run_command(["ros2", "topic", "info", topic, "-v"])
    if code != 0:
        print(f"{topic}: info unavailable: {info}")
        return

    publishers = 0
    subscriptions = 0
    for line in info.splitlines():
        line = line.strip()
        if line.startswith("Publisher count:"):
            publishers = int(line.split(":", 1)[1].strip())
        elif line.startswith("Subscription count:"):
            subscriptions = int(line.split(":", 1)[1].strip())
    print(f"{topic}: pub={publishers} sub={subscriptions}")


def check_topics(topics: Iterable[str]) -> None:
    check_topic_presence(topics)
    print("\ntopic endpoints:")
    for topic in topics:
        check_topic_detail(topic)


def print_timestamp() -> None:
    print(time.strftime("%Y-%m-%d %H:%M:%S"))


def clear_screen() -> None:
    print("\033[2J\033[H", end="")


def ensure_commands() -> None:
    missing = [
        name for name in ("systemctl", "ros2")
        if shutil.which(name) is None
    ]
    if missing:
        raise SystemExit(f"missing commands: {', '.join(missing)}")


def monitor(services: list[str], nodes: list[str], topics: list[str]) -> None:
    print_timestamp()
    check_services(services)
    try:
        check_nodes(nodes)
        check_topics(topics)
    except RuntimeError as exc:
        print_section("ros2 status")
        print(exc)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Monitor line_follow services, nodes, and topics.",
    )
    parser.add_argument(
        "--watch",
        action="store_true",
        help="refresh continuously",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=2.0,
        help="refresh interval seconds when --watch is enabled",
    )
    parser.add_argument(
        "--service",
        action="append",
        dest="services",
        default=[],
        help="additional service to check",
    )
    parser.add_argument(
        "--node",
        action="append",
        dest="nodes",
        default=[],
        help="additional node to check",
    )
    parser.add_argument(
        "--topic",
        action="append",
        dest="topics",
        default=[],
        help="additional topic to check",
    )
    return parser.parse_args()


def merge_defaults(defaults: list[str], extra: list[str]) -> list[str]:
    merged = list(defaults)
    for item in extra:
        if item not in merged:
            merged.append(item)
    return merged


def main() -> None:
    args = parse_args()
    ensure_commands()

    services = merge_defaults(DEFAULT_SERVICES, args.services)
    nodes = merge_defaults(DEFAULT_NODES, args.nodes)
    topics = merge_defaults(DEFAULT_TOPICS, args.topics)

    if not args.watch:
        monitor(services, nodes, topics)
        return

    try:
        while True:
            clear_screen()
            monitor(services, nodes, topics)
            time.sleep(max(args.interval, 0.2))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
