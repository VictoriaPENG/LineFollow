from __future__ import annotations


class DebouncedDigitalInput:
    """Debounce a digital GPIO input using consecutive stable samples."""

    def __init__(
        self,
        *,
        active_low: bool,
        activate_count: int = 3,
        deactivate_count: int = 3,
    ) -> None:
        self.active_low = bool(active_low)
        self.activate_count = max(1, int(activate_count))
        self.deactivate_count = max(1, int(deactivate_count))
        self.filtered_active = False
        self.last_raw_active = None
        self.pending_count = 0

    def update(self, raw_level: int):
        raw_level = int(raw_level)
        raw_active = (raw_level == 0) if self.active_low else (raw_level != 0)
        changed = False

        if raw_active == self.filtered_active:
            self.pending_count = 0
            self.last_raw_active = raw_active
            return raw_active, self.filtered_active, changed

        if raw_active != self.last_raw_active:
            self.last_raw_active = raw_active
            self.pending_count = 1
        else:
            self.pending_count += 1

        threshold = self.activate_count if raw_active else self.deactivate_count
        if self.pending_count >= threshold:
            self.filtered_active = raw_active
            self.pending_count = 0
            changed = True

        return raw_active, self.filtered_active, changed
