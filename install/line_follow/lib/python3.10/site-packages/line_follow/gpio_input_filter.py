from __future__ import annotations


class DebouncedDigitalInput:
    """对数字量 GPIO 输入做去抖滤波。

    通过“连续多次采样一致才确认切换”的方式，把容易抖动的原始
    电平变成更稳定的逻辑状态，避免误触发车辆动作。
    """

    def __init__(
        self,
        *,
        active_low: bool,
        activate_count: int = 3,
        deactivate_count: int = 3,
    ) -> None:
        # `active_low=True` 表示低电平视为按下/有效。
        self.active_low = bool(active_low)
        # 激活和释放阈值分开配置，便于适配不同硬件噪声特征。
        self.activate_count = max(1, int(activate_count))
        self.deactivate_count = max(1, int(deactivate_count))
        self.filtered_active = False
        self.last_raw_active = None
        self.pending_count = 0

    def update(self, raw_level: int):
        """输入一次原始采样，返回原始态、稳定态和是否切换。"""
        raw_level = int(raw_level)
        raw_active = (raw_level == 0) if self.active_low else (raw_level != 0)
        changed = False

        # 原始状态已经与稳定状态一致时，不存在切换趋势，清空等待计数。
        if raw_active == self.filtered_active:
            self.pending_count = 0
            self.last_raw_active = raw_active
            return raw_active, self.filtered_active, changed

        # 只有连续多次观察到同一种新状态，才继续累积切换计数。
        if raw_active != self.last_raw_active:
            self.last_raw_active = raw_active
            self.pending_count = 1
        else:
            self.pending_count += 1

        # 激活和释放可采用不同阈值，在稳定性和灵敏度之间取平衡。
        threshold = self.activate_count if raw_active else self.deactivate_count
        if self.pending_count >= threshold:
            self.filtered_active = raw_active
            self.pending_count = 0
            changed = True

        return raw_active, self.filtered_active, changed
