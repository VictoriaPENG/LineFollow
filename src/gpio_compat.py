from __future__ import annotations


def load_gpio_module():
    """Try common GPIO modules used on embedded Linux boards."""
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

