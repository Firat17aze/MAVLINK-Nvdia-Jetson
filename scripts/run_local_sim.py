from __future__ import annotations

from pathlib import Path

from uav_autonomy.models import Detection, SensorFrame, Vector3
from uav_autonomy.pipeline import AutonomyPipeline


def make_frame(ts: float, target_x: float, obstacle_m: float, battery_pct: float) -> SensorFrame:
    detections = [Detection("target", 0.88, Vector3(target_x, 1.0, 0.0))]
    return SensorFrame(
        timestamp_s=ts,
        position_ned_m=Vector3(20.0, 10.0, -30.0),
        velocity_ned_mps=Vector3(),
        camera_detections=detections,
        radar_ranges_m=[obstacle_m],
        battery_pct=battery_pct,
    )


def main() -> None:
    pipeline = AutonomyPipeline.from_file(Path("configs/autonomy.yaml"))

    scenarios = [
        make_frame(ts=1.0, target_x=36.0, obstacle_m=25.0, battery_pct=70.0),
        make_frame(ts=2.0, target_x=30.0, obstacle_m=14.0, battery_pct=65.0),
        make_frame(ts=3.0, target_x=24.0, obstacle_m=6.5, battery_pct=60.0),
        make_frame(ts=4.0, target_x=21.0, obstacle_m=20.0, battery_pct=15.0),
    ]

    for frame in scenarios:
        result = pipeline.step(
            frame,
            cpu_pct=42.0,
            inference_latency_ms=60.0,
            link_rtt_ms=55.0,
            companion_alive=True,
            mavlink_heartbeat_age_s=0.1,
        )
        print(
            {
                "time": frame.timestamp_s,
                "mission_state": result.intent.state.value,
                "safety": result.decision.status.value,
                "failsafe": result.failsafe_action.value,
                "mode_hint": result.mavlink_packet["mode_hint"],
                "vx": result.mavlink_packet["vx_mps"],
                "vy": result.mavlink_packet["vy_mps"],
                "reason": result.decision.reason,
            }
        )


if __name__ == "__main__":
    main()
