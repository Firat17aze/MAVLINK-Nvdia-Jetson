from __future__ import annotations

from .models import Detection, SensorFrame, Vector3


class PerceptionInferenceNode:
    def run(self, frame: SensorFrame) -> list[Detection]:
        detections = list(frame.camera_detections)
        for rng in frame.radar_ranges_m:
            if 0.0 < rng < 50.0:
                conf = max(0.5, min(0.95, 1.0 - (rng / 120.0)))
                detections.append(
                    Detection(
                        label="obstacle",
                        confidence=conf,
                        rel_position_m=Vector3(x=rng, y=0.0, z=0.0),
                    )
                )
        return detections
