from __future__ import annotations

from .config import SystemConfig
from .models import (
    FusedWorld,
    HealthState,
    MissionIntent,
    MissionState,
    OffboardSetpoint,
    SafetyDecision,
    SafetyStatus,
    SensorFrame,
    Vector3,
)


class SafetyGuardian:
    def __init__(self, config: SystemConfig) -> None:
        self._config = config

    def evaluate(
        self,
        frame: SensorFrame,
        intent: MissionIntent,
        fused: FusedWorld,
        health: HealthState,
        now_s: float,
    ) -> SafetyDecision:
        faults: list[str] = []

        if now_s - intent.generated_at_s > self._config.safety.command_timeout_s:
            faults.append("stale_intent")

        if now_s - health.timestamp_s > self._config.safety.health_timeout_s:
            faults.append("stale_health")

        if not health.companion_alive:
            faults.append("companion_not_alive")

        if health.cpu_pct > self._config.health.max_cpu_pct:
            faults.append("cpu_over_limit")

        if health.inference_latency_ms > self._config.health.max_inference_latency_ms:
            faults.append("inference_latency_over_limit")

        if health.link_rtt_ms > self._config.health.max_link_rtt_ms:
            faults.append("link_rtt_over_limit")

        if fused.obstacle_closure_rate_mps > self._config.safety.max_obstacle_closure_rate_mps:
            faults.append("obstacle_closure_rate_high")

        if fused.sensor_disagreement:
            faults.append("sensor_disagreement")

        if intent.state == MissionState.TRACK_TARGET and intent.confidence < self._config.safety.confidence.min_tracking:
            faults.append("tracking_confidence_low")

        radius = (frame.position_ned_m.x ** 2 + frame.position_ned_m.y ** 2) ** 0.5
        altitude_m = -frame.position_ned_m.z
        gf = self._config.mission.geofence
        if radius > gf.max_radius_m:
            faults.append("geofence_radius_exceeded")
        if altitude_m < gf.min_alt_m or altitude_m > gf.max_alt_m:
            faults.append("geofence_altitude_exceeded")

        if faults:
            return SafetyDecision(
                status=SafetyStatus.HOLD,
                setpoint=OffboardSetpoint(0.0, 0.0, 0.0, 0.0),
                reason=";".join(faults),
                faults=tuple(faults),
            )

        speed_cfg = self._config.mission.speed
        clamped: Vector3 = intent.requested_velocity_mps.clamp_xy(speed_cfg.max_xy_mps).clamp_z(
            speed_cfg.max_z_mps
        )

        reason = "approved"
        if clamped != intent.requested_velocity_mps:
            reason = "approved_clamped"

        return SafetyDecision(
            status=SafetyStatus.APPROVE,
            setpoint=OffboardSetpoint(
                vx_mps=clamped.x,
                vy_mps=clamped.y,
                vz_mps=clamped.z,
                yaw_rate_rps=intent.requested_yaw_rate_rps,
            ),
            reason=reason,
            faults=(),
        )
