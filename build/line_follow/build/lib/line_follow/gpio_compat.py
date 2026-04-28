from __future__ import annotations


def load_gpio_module():
    """按常见优先级加载板端 GPIO 模块。

    这个工程可能运行在不同硬件平台上，因此这里不把 GPIO
    实现绑定为某一个库，而是按顺序尝试导入多种常见实现。
    这样上层节点只关心“拿到一个可用 GPIO 模块”即可。
    """
    module_errors = []
    for module_name in ("Hobot.GPIO", "Jetson.GPIO", "RPi.GPIO"):
        try:
            module = __import__(module_name, fromlist=["GPIO"])
            return module
        except Exception as exc:  # pragma: no cover - depends on board environment
            module_errors.append(f"{module_name}: {exc}")

    raise RuntimeError(
        "No supported GPIO Python module found. Tried: "
        + "; ".join(module_errors)
    )
