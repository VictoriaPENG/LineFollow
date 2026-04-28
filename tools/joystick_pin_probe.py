#!/usr/bin/env python3
"""
摇杆 GPIO 探针工具。

用于在正式运行摇杆节点之前，先确认：
1. 某个物理方向到底对应哪个引脚
2. 输入是高有效还是低有效
3. 去抖阈值是否合适

默认监视 BOARD 编号的 11、15、13、16 四个引脚。
"""

from __future__ import annotations

import argparse
import os
import sys
import time


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    # 兼容“源码目录直接运行”的场景。
    sys.path.insert(0, REPO_ROOT)

try:
    from line_follow.gpio_compat import load_gpio_module
    from line_follow.gpio_input_filter import DebouncedDigitalInput
except ModuleNotFoundError:
    from src.gpio_compat import load_gpio_module
    from src.gpio_input_filter import DebouncedDigitalInput


GPIO = load_gpio_module()


def parse_args():
    """解析探针参数，便于快速切换监视引脚和采样模式。"""
    parser = argparse.ArgumentParser(description="Probe joystick GPIO pins")
    parser.add_argument("--pins", nargs="+", type=int, default=[11, 15, 13, 16])
    parser.add_argument("--mode", choices=("BOARD", "BCM"), default="BOARD")
    parser.add_argument("--active-low", action="store_true", default=True)
    parser.add_argument("--active-high", action="store_false", dest="active_low")
    parser.add_argument("--interval", type=float, default=0.05)
    parser.add_argument("--debounce-activate-count", type=int, default=5)
    parser.add_argument("--debounce-deactivate-count", type=int, default=8)
    return parser.parse_args()


def main() -> int:
    """循环打印原始电平和去抖结果，辅助现场接线排查。"""
    args = parse_args()
    mode = GPIO.BOARD if args.mode == "BOARD" else GPIO.BCM
    GPIO.setwarnings(False)
    GPIO.setmode(mode)
    filters = {
        pin: DebouncedDigitalInput(
            active_low=args.active_low,
            activate_count=args.debounce_activate_count,
            deactivate_count=args.debounce_deactivate_count,
        )
        for pin in args.pins
    }

    try:
        for pin in args.pins:
            if hasattr(GPIO, "PUD_UP") and hasattr(GPIO, "PUD_DOWN"):
                pud = GPIO.PUD_UP if args.active_low else GPIO.PUD_DOWN
                GPIO.setup(pin, GPIO.IN, pull_up_down=pud)
            else:
                GPIO.setup(pin, GPIO.IN)

        print(
            "Joystick pin probe started. Move the joystick in each direction and watch which pin becomes active."
        )
        print("expected default mapping: 11=forward, 15=reverse, 13=left, 16=right")
        print(
            f"mode={args.mode}, active_low={args.active_low}, pins={args.pins}, "
            f"debounce_activate_count={args.debounce_activate_count}, "
            f"debounce_deactivate_count={args.debounce_deactivate_count}. Press Ctrl+C to exit."
        )

        last_snapshot = None
        while True:
            snapshot = []
            raw_active_pins = []
            filtered_active_pins = []
            for pin in args.pins:
                raw_level = int(GPIO.input(pin))
                raw_active, filtered_active, _ = filters[pin].update(raw_level)
                pending_count = filters[pin].pending_count
                snapshot.append((pin, raw_level, raw_active, filtered_active, pending_count))
                if raw_active:
                    raw_active_pins.append(pin)
                if filtered_active:
                    filtered_active_pins.append(pin)

            frozen = tuple(snapshot)
            if frozen != last_snapshot:
                state_text = ", ".join(
                    f"pin {pin}: raw={raw}, raw_active={raw_active}, filtered_active={filtered_active}, pending={pending}"
                    for pin, raw, raw_active, filtered_active, pending in snapshot
                )
                print(state_text)
                if raw_active_pins:
                    print(f"raw active pins -> {raw_active_pins}")
                else:
                    print("raw active pins -> []")
                if filtered_active_pins:
                    print(f"filtered active pins -> {filtered_active_pins}")
                else:
                    print("filtered active pins -> []")
                print("-" * 60)
                last_snapshot = frozen

            time.sleep(max(0.01, args.interval))
    except KeyboardInterrupt:
        print("probe stopped")
        return 0
    except Exception as exc:
        print(f"probe failed: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            GPIO.cleanup()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
