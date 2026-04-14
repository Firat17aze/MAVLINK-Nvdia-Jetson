from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import yaml


@dataclass(frozen=True)
class GeofenceConfig:
    max_radius_m: float
    min_alt_m: float
    max_alt_m: float


@dataclass(frozen=True)
class SpeedConfig:
    max_xy_mps: float
    max_z_mps: float


@dataclass(frozen=True)
class TargetTrackingConfig:
    desired_follow_distance_m: float
    lost_timeout_s: float


@dataclass(frozen=True)
class MissionConfig:
    mode: str
    geofence: GeofenceConfig
    speed: SpeedConfig
    target_tracking: TargetTrackingConfig


@dataclass(frozen=True)
class ConfidenceConfig:
    min_detection: float
    min_tracking: float


@dataclass(frozen=True)
class SafetyConfig:
    confidence: ConfidenceConfig
    command_timeout_s: float
    health_timeout_s: float
    max_obstacle_closure_rate_mps: float
    emergency_brake_distance_m: float


@dataclass(frozen=True)
class FailsafeConfig:
    ai_timeout_action: str
    companion_loss_action: str
    low_battery_action: str
    nav_fault_action: str
    lte_fallback_enabled: bool


@dataclass(frozen=True)
class MavlinkConfig:
    system_id: int
    component_id: int
    target_system_id: int
    target_component_id: int
    offboard_rate_hz: int


@dataclass(frozen=True)
class TelemetryConfig:
    primary: str
    secondary: str


@dataclass(frozen=True)
class CommsConfig:
    mavlink: MavlinkConfig
    telemetry: TelemetryConfig


@dataclass(frozen=True)
class HealthConfig:
    max_cpu_pct: float
    max_inference_latency_ms: float
    max_link_rtt_ms: float


@dataclass(frozen=True)
class SystemConfig:
    mission: MissionConfig
    safety: SafetyConfig
    failsafe: FailsafeConfig
    comms: CommsConfig
    health: HealthConfig


def _read_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def load_system_config(path: str | Path) -> SystemConfig:
    raw = _read_yaml(Path(path))

    mission = MissionConfig(
        mode=raw["mission"]["mode"],
        geofence=GeofenceConfig(**raw["mission"]["geofence"]),
        speed=SpeedConfig(**raw["mission"]["speed"]),
        target_tracking=TargetTrackingConfig(**raw["mission"]["target_tracking"]),
    )
    safety = SafetyConfig(
        confidence=ConfidenceConfig(**raw["safety"]["confidence"]),
        command_timeout_s=raw["safety"]["command_timeout_s"],
        health_timeout_s=raw["safety"]["health_timeout_s"],
        max_obstacle_closure_rate_mps=raw["safety"]["max_obstacle_closure_rate_mps"],
        emergency_brake_distance_m=raw["safety"]["emergency_brake_distance_m"],
    )
    failsafe = FailsafeConfig(**raw["failsafe"])
    comms = CommsConfig(
        mavlink=MavlinkConfig(**raw["comms"]["mavlink"]),
        telemetry=TelemetryConfig(**raw["comms"]["telemetry"]),
    )
    health = HealthConfig(**raw["health"])

    return SystemConfig(
        mission=mission,
        safety=safety,
        failsafe=failsafe,
        comms=comms,
        health=health,
    )
