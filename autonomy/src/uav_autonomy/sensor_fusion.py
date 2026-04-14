from __future__ import annotations

from .models import Detection, FusedWorld, SensorFrame


class SensorFusionNode:
    def __init__(self) -> None:
        self._prev_nearest: float | None = None
        self._prev_ts: float | None = None

    def run(self, frame: SensorFrame, detections: list[Detection]) -> FusedWorld:
        obstacle_distances = [
            d.rel_position_m.x
            for d in detections
            if d.label == "obstacle" and d.rel_position_m.x > 0.0 and d.confidence >= 0.5
        ]
        radar_distances = [d for d in frame.radar_ranges_m if d > 0.0]

        nearest = None
        if obstacle_distances or radar_distances:
            nearest = min(obstacle_distances + radar_distances)

        closure = 0.0
        if nearest is not None and self._prev_nearest is not None and self._prev_ts is not None:
            dt = frame.timestamp_s - self._prev_ts
            if dt > 0:
                closure = max(0.0, (self._prev_nearest - nearest) / dt)

        cam_near = min(obstacle_distances) if obstacle_distances else None
        radar_near = min(radar_distances) if radar_distances else None
        disagreement = False
        if cam_near is not None and radar_near is not None:
            disagreement = abs(cam_near - radar_near) > 4.0

        self._prev_nearest = nearest
        self._prev_ts = frame.timestamp_s

        return FusedWorld(
            nearest_obstacle_m=nearest,
            obstacle_closure_rate_mps=closure,
            sensor_disagreement=disagreement,
        )
