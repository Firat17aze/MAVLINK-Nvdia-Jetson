from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class MissionState(str, Enum):
    HOLD = "HOLD"
    SEEK_TARGET = "SEEK_TARGET"
    TRACK_TARGET = "TRACK_TARGET"
    AVOID_OBSTACLE = "AVOID_OBSTACLE"


class SafetyStatus(str, Enum):
    APPROVE = "APPROVE"
    HOLD = "HOLD"
    REJECT = "REJECT"


class FailsafeAction(str, Enum):
    NONE = "NONE"
    HOLD = "HOLD"
    PX4_FAILSAFE = "PX4_FAILSAFE"
    RTL = "RTL"
    LOITER_MANUAL = "LOITER_MANUAL"


@dataclass(frozen=True)
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def clamp_xy(self, max_xy: float) -> "Vector3":
        mag_sq = self.x * self.x + self.y * self.y
        if mag_sq <= max_xy * max_xy or mag_sq == 0:
            return self
        scale = max_xy / (mag_sq ** 0.5)
        return Vector3(self.x * scale, self.y * scale, self.z)

    def clamp_z(self, max_abs_z: float) -> "Vector3":
        z = max(-max_abs_z, min(max_abs_z, self.z))
        return Vector3(self.x, self.y, z)


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    rel_position_m: Vector3


@dataclass(frozen=True)
class TrackedTarget:
    rel_position_m: Vector3
    confidence: float
    last_seen_s: float


@dataclass(frozen=True)
class SensorFrame:
    timestamp_s: float
    position_ned_m: Vector3
    velocity_ned_mps: Vector3
    camera_detections: list[Detection] = field(default_factory=list)
    radar_ranges_m: list[float] = field(default_factory=list)
    battery_pct: float = 100.0
    gnss_ok: bool = True
    nav_ok: bool = True
    rf_link_ok: bool = True
    lte_link_ok: bool = True
    operator_override: bool = False


@dataclass(frozen=True)
class FusedWorld:
    nearest_obstacle_m: float | None
    obstacle_closure_rate_mps: float
    sensor_disagreement: bool


@dataclass(frozen=True)
class MissionIntent:
    state: MissionState
    requested_velocity_mps: Vector3
    requested_yaw_rate_rps: float
    confidence: float
    generated_at_s: float
    rationale: str


@dataclass(frozen=True)
class HealthState:
    timestamp_s: float
    cpu_pct: float
    inference_latency_ms: float
    link_rtt_ms: float
    companion_alive: bool
    mavlink_heartbeat_age_s: float


@dataclass(frozen=True)
class FaultSnapshot:
    ai_timeout: bool
    companion_loss: bool
    low_battery: bool
    nav_fault: bool


@dataclass(frozen=True)
class OffboardSetpoint:
    vx_mps: float
    vy_mps: float
    vz_mps: float
    yaw_rate_rps: float
    frame: str = "LOCAL_NED"


@dataclass(frozen=True)
class SafetyDecision:
    status: SafetyStatus
    setpoint: OffboardSetpoint
    reason: str
    faults: tuple[str, ...] = ()


@dataclass(frozen=True)
class PipelineResult:
    intent: MissionIntent
    decision: SafetyDecision
    failsafe_action: FailsafeAction
    mavlink_packet: dict
    tracked_target: TrackedTarget | None
    fused_world: FusedWorld
