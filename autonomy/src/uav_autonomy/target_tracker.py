from __future__ import annotations

from .models import Detection, TrackedTarget


TARGET_LABELS = {"person", "vehicle", "target"}


class TargetTrackerNode:
    def __init__(self, lost_timeout_s: float) -> None:
        self._lost_timeout_s = lost_timeout_s
        self._tracked: TrackedTarget | None = None

    def update(self, detections: list[Detection], now_s: float) -> TrackedTarget | None:
        target_dets = [d for d in detections if d.label in TARGET_LABELS]
        if target_dets:
            best = max(target_dets, key=lambda d: d.confidence)
            self._tracked = TrackedTarget(
                rel_position_m=best.rel_position_m,
                confidence=best.confidence,
                last_seen_s=now_s,
            )
            return self._tracked

        if self._tracked and (now_s - self._tracked.last_seen_s) <= self._lost_timeout_s:
            return self._tracked

        self._tracked = None
        return None
