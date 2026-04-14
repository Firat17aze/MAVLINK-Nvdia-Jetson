from __future__ import annotations

from .config import MissionConfig, SafetyConfig
from .models import FusedWorld, MissionIntent, MissionState, SensorFrame, TrackedTarget, Vector3


class MissionManager:
    def __init__(self, mission_cfg: MissionConfig, safety_cfg: SafetyConfig) -> None:
        self._mission_cfg = mission_cfg
        self._safety_cfg = safety_cfg

    def generate(
        self,
        frame: SensorFrame,
        fused: FusedWorld,
        target: TrackedTarget | None,
    ) -> MissionIntent:
        if frame.operator_override:
            return MissionIntent(
                state=MissionState.HOLD,
                requested_velocity_mps=Vector3(),
                requested_yaw_rate_rps=0.0,
                confidence=1.0,
                generated_at_s=frame.timestamp_s,
                rationale="operator_override",
            )

        nearest = fused.nearest_obstacle_m
        if nearest is not None and nearest <= self._safety_cfg.emergency_brake_distance_m:
            return MissionIntent(
                state=MissionState.AVOID_OBSTACLE,
                requested_velocity_mps=Vector3(x=-1.0, y=0.0, z=0.0),
                requested_yaw_rate_rps=0.2,
                confidence=0.95,
                generated_at_s=frame.timestamp_s,
                rationale="emergency_obstacle_brake",
            )

        if target is None:
            return MissionIntent(
                state=MissionState.SEEK_TARGET,
                requested_velocity_mps=Vector3(x=2.0, y=0.0, z=0.0),
                requested_yaw_rate_rps=0.15,
                confidence=0.8,
                generated_at_s=frame.timestamp_s,
                rationale="target_not_visible",
            )

        desired = self._mission_cfg.target_tracking.desired_follow_distance_m
        distance = target.rel_position_m.x
        delta = distance - desired
        vx = max(-2.0, min(4.0, delta * 0.25))
        vy = max(-2.0, min(2.0, target.rel_position_m.y * 0.20))

        return MissionIntent(
            state=MissionState.TRACK_TARGET,
            requested_velocity_mps=Vector3(x=vx, y=vy, z=0.0),
            requested_yaw_rate_rps=max(-0.35, min(0.35, target.rel_position_m.y * 0.05)),
            confidence=target.confidence,
            generated_at_s=frame.timestamp_s,
            rationale="target_tracking",
        )
