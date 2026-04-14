from pathlib import Path

from uav_autonomy.pipeline import AutonomyPipeline
from uav_autonomy.models import Detection, FailsafeAction, SafetyStatus, SensorFrame, Vector3


def _pipeline() -> AutonomyPipeline:
    return AutonomyPipeline.from_file(Path("configs/autonomy.yaml"))


def _frame(ts: float = 5.0, battery_pct: float = 60.0, nav_ok: bool = True) -> SensorFrame:
    return SensorFrame(
        timestamp_s=ts,
        position_ned_m=Vector3(0.0, 0.0, -25.0),
        velocity_ned_mps=Vector3(),
        camera_detections=[Detection("target", 0.9, Vector3(28.0, 1.0, 0.0))],
        radar_ranges_m=[32.0],
        battery_pct=battery_pct,
        nav_ok=nav_ok,
    )


def test_nominal_tracking_produces_approved_command() -> None:
    pipe = _pipeline()
    result = pipe.step(
        _frame(),
        cpu_pct=30.0,
        inference_latency_ms=35.0,
        link_rtt_ms=25.0,
        companion_alive=True,
        mavlink_heartbeat_age_s=0.1,
    )

    assert result.decision.status == SafetyStatus.APPROVE
    assert result.failsafe_action == FailsafeAction.NONE
    assert result.mavlink_packet["message"] == "SET_POSITION_TARGET_LOCAL_NED"


def test_companion_loss_triggers_px4_failsafe_hint() -> None:
    pipe = _pipeline()
    result = pipe.step(
        _frame(),
        cpu_pct=30.0,
        inference_latency_ms=35.0,
        link_rtt_ms=25.0,
        companion_alive=False,
        mavlink_heartbeat_age_s=5.0,
    )

    assert result.failsafe_action == FailsafeAction.PX4_FAILSAFE
    assert result.mavlink_packet["mode_hint"] == FailsafeAction.PX4_FAILSAFE.value


def test_low_battery_triggers_rtl_hint() -> None:
    pipe = _pipeline()
    result = pipe.step(
        _frame(battery_pct=15.0),
        cpu_pct=30.0,
        inference_latency_ms=35.0,
        link_rtt_ms=25.0,
        companion_alive=True,
        mavlink_heartbeat_age_s=0.2,
    )

    assert result.failsafe_action == FailsafeAction.RTL


def test_nav_fault_triggers_loiter_manual() -> None:
    pipe = _pipeline()
    result = pipe.step(
        _frame(nav_ok=False),
        cpu_pct=30.0,
        inference_latency_ms=35.0,
        link_rtt_ms=25.0,
        companion_alive=True,
        mavlink_heartbeat_age_s=0.2,
    )

    assert result.failsafe_action == FailsafeAction.LOITER_MANUAL
