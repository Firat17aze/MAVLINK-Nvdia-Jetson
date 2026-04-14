from pathlib import Path

from uav_autonomy.config import load_system_config
from uav_autonomy.mission_manager import MissionManager
from uav_autonomy.models import Detection, FusedWorld, MissionState, SensorFrame, TrackedTarget, Vector3


def _config():
    return load_system_config(Path("configs/autonomy.yaml"))


def _frame(ts: float = 1.0) -> SensorFrame:
    return SensorFrame(
        timestamp_s=ts,
        position_ned_m=Vector3(0.0, 0.0, -20.0),
        velocity_ned_mps=Vector3(0.0, 0.0, 0.0),
    )


def test_obstacle_brake_state_is_selected() -> None:
    cfg = _config()
    mgr = MissionManager(cfg.mission, cfg.safety)

    intent = mgr.generate(
        _frame(),
        FusedWorld(nearest_obstacle_m=5.0, obstacle_closure_rate_mps=1.2, sensor_disagreement=False),
        target=None,
    )

    assert intent.state == MissionState.AVOID_OBSTACLE
    assert intent.requested_velocity_mps.x < 0.0


def test_target_tracking_state_is_selected() -> None:
    cfg = _config()
    mgr = MissionManager(cfg.mission, cfg.safety)

    target = TrackedTarget(rel_position_m=Vector3(30.0, 2.0, 0.0), confidence=0.9, last_seen_s=1.0)
    intent = mgr.generate(
        _frame(),
        FusedWorld(nearest_obstacle_m=40.0, obstacle_closure_rate_mps=0.0, sensor_disagreement=False),
        target=target,
    )

    assert intent.state == MissionState.TRACK_TARGET
    assert intent.requested_velocity_mps.x > 0.0


def test_seek_target_when_none_visible() -> None:
    cfg = _config()
    mgr = MissionManager(cfg.mission, cfg.safety)

    intent = mgr.generate(
        _frame(),
        FusedWorld(nearest_obstacle_m=None, obstacle_closure_rate_mps=0.0, sensor_disagreement=False),
        target=None,
    )

    assert intent.state == MissionState.SEEK_TARGET


def test_operator_override_forces_hold() -> None:
    cfg = _config()
    mgr = MissionManager(cfg.mission, cfg.safety)

    frame = SensorFrame(
        timestamp_s=1.0,
        position_ned_m=Vector3(0.0, 0.0, -20.0),
        velocity_ned_mps=Vector3(0.0, 0.0, 0.0),
        camera_detections=[
            Detection(label="target", confidence=0.92, rel_position_m=Vector3(25.0, 1.0, 0.0))
        ],
        operator_override=True,
    )
    intent = mgr.generate(
        frame,
        FusedWorld(nearest_obstacle_m=None, obstacle_closure_rate_mps=0.0, sensor_disagreement=False),
        target=TrackedTarget(
            rel_position_m=Vector3(25.0, 1.0, 0.0),
            confidence=0.92,
            last_seen_s=1.0,
        ),
    )

    assert intent.state == MissionState.HOLD
