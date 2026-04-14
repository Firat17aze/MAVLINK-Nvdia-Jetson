from pathlib import Path

from uav_autonomy.config import load_system_config
from uav_autonomy.models import (
    FusedWorld,
    HealthState,
    MissionIntent,
    MissionState,
    SafetyStatus,
    SensorFrame,
    Vector3,
)
from uav_autonomy.safety_guardian import SafetyGuardian


def _config():
    return load_system_config(Path("configs/autonomy.yaml"))


def _frame() -> SensorFrame:
    return SensorFrame(
        timestamp_s=10.0,
        position_ned_m=Vector3(10.0, 10.0, -30.0),
        velocity_ned_mps=Vector3(),
    )


def _health(ts: float = 10.0) -> HealthState:
    return HealthState(
        timestamp_s=ts,
        cpu_pct=40.0,
        inference_latency_ms=40.0,
        link_rtt_ms=30.0,
        companion_alive=True,
        mavlink_heartbeat_age_s=0.1,
    )


def test_speed_is_clamped_and_approved() -> None:
    cfg = _config()
    guardian = SafetyGuardian(cfg)

    intent = MissionIntent(
        state=MissionState.TRACK_TARGET,
        requested_velocity_mps=Vector3(25.0, 0.0, 0.0),
        requested_yaw_rate_rps=0.0,
        confidence=0.9,
        generated_at_s=10.0,
        rationale="test",
    )

    decision = guardian.evaluate(
        frame=_frame(),
        intent=intent,
        fused=FusedWorld(nearest_obstacle_m=30.0, obstacle_closure_rate_mps=0.2, sensor_disagreement=False),
        health=_health(),
        now_s=10.0,
    )

    assert decision.status == SafetyStatus.APPROVE
    assert decision.setpoint.vx_mps <= cfg.mission.speed.max_xy_mps


def test_stale_intent_results_in_hold() -> None:
    cfg = _config()
    guardian = SafetyGuardian(cfg)

    intent = MissionIntent(
        state=MissionState.SEEK_TARGET,
        requested_velocity_mps=Vector3(2.0, 0.0, 0.0),
        requested_yaw_rate_rps=0.1,
        confidence=0.8,
        generated_at_s=1.0,
        rationale="stale",
    )

    decision = guardian.evaluate(
        frame=_frame(),
        intent=intent,
        fused=FusedWorld(nearest_obstacle_m=None, obstacle_closure_rate_mps=0.0, sensor_disagreement=False),
        health=_health(ts=10.0),
        now_s=10.0,
    )

    assert decision.status == SafetyStatus.HOLD
    assert "stale_intent" in decision.reason


def test_geofence_violation_results_in_hold() -> None:
    cfg = _config()
    guardian = SafetyGuardian(cfg)

    frame = SensorFrame(
        timestamp_s=10.0,
        position_ned_m=Vector3(1000.0, 0.0, -30.0),
        velocity_ned_mps=Vector3(),
    )
    intent = MissionIntent(
        state=MissionState.SEEK_TARGET,
        requested_velocity_mps=Vector3(2.0, 0.0, 0.0),
        requested_yaw_rate_rps=0.0,
        confidence=0.8,
        generated_at_s=10.0,
        rationale="geofence",
    )

    decision = guardian.evaluate(
        frame=frame,
        intent=intent,
        fused=FusedWorld(nearest_obstacle_m=None, obstacle_closure_rate_mps=0.0, sensor_disagreement=False),
        health=_health(),
        now_s=10.0,
    )

    assert decision.status == SafetyStatus.HOLD
    assert "geofence_radius_exceeded" in decision.reason
