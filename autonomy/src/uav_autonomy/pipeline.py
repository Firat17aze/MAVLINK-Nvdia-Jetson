from __future__ import annotations

from pathlib import Path

from .config import SystemConfig, load_system_config
from .failsafe_manager import FailsafeManager
from .health_monitor import HealthMonitor
from .mavlink_bridge import MavlinkBridge
from .mission_manager import MissionManager
from .models import FaultSnapshot, PipelineResult, SensorFrame
from .perception_inference import PerceptionInferenceNode
from .safety_guardian import SafetyGuardian
from .sensor_fusion import SensorFusionNode
from .target_tracker import TargetTrackerNode


class AutonomyPipeline:
    def __init__(self, config: SystemConfig) -> None:
        self.config = config
        self.perception = PerceptionInferenceNode()
        self.fusion = SensorFusionNode()
        self.tracker = TargetTrackerNode(config.mission.target_tracking.lost_timeout_s)
        self.mission = MissionManager(config.mission, config.safety)
        self.guardian = SafetyGuardian(config)
        self.mavlink = MavlinkBridge(config)
        self.health = HealthMonitor()
        self.failsafe = FailsafeManager()

    @classmethod
    def from_file(cls, config_path: str | Path) -> "AutonomyPipeline":
        return cls(load_system_config(config_path))

    def step(
        self,
        frame: SensorFrame,
        *,
        cpu_pct: float,
        inference_latency_ms: float,
        link_rtt_ms: float,
        companion_alive: bool,
        mavlink_heartbeat_age_s: float,
        now_s: float | None = None,
    ) -> PipelineResult:
        now = frame.timestamp_s if now_s is None else now_s

        detections = self.perception.run(frame)
        fused = self.fusion.run(frame, detections)
        tracked = self.tracker.update(detections, now)
        intent = self.mission.generate(frame, fused, tracked)

        health = self.health.build(
            now_s=now,
            cpu_pct=cpu_pct,
            inference_latency_ms=inference_latency_ms,
            link_rtt_ms=link_rtt_ms,
            companion_alive=companion_alive,
            mavlink_heartbeat_age_s=mavlink_heartbeat_age_s,
        )

        decision = self.guardian.evaluate(
            frame=frame,
            intent=intent,
            fused=fused,
            health=health,
            now_s=now,
        )

        faults = FaultSnapshot(
            ai_timeout=(now - intent.generated_at_s) > self.config.safety.command_timeout_s,
            companion_loss=not companion_alive,
            low_battery=frame.battery_pct < 20.0,
            nav_fault=not frame.nav_ok,
        )

        failsafe_action = self.failsafe.select(faults)
        packet = self.mavlink.to_setpoint_message(decision, failsafe_action)

        return PipelineResult(
            intent=intent,
            decision=decision,
            failsafe_action=failsafe_action,
            mavlink_packet=packet,
            tracked_target=tracked,
            fused_world=fused,
        )
