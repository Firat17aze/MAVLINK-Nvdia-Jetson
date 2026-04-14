from __future__ import annotations

from .config import SystemConfig
from .models import FailsafeAction, SafetyDecision, SafetyStatus


class MavlinkBridge:
    def __init__(self, config: SystemConfig) -> None:
        self._cfg = config.comms.mavlink

    def to_setpoint_message(self, decision: SafetyDecision, failsafe: FailsafeAction) -> dict:
        mode = "OFFBOARD" if decision.status == SafetyStatus.APPROVE else "HOLD"
        if failsafe != FailsafeAction.NONE:
            mode = failsafe.value

        return {
            "message": "SET_POSITION_TARGET_LOCAL_NED",
            "system_id": self._cfg.system_id,
            "component_id": self._cfg.component_id,
            "target_system_id": self._cfg.target_system_id,
            "target_component_id": self._cfg.target_component_id,
            "frame": decision.setpoint.frame,
            "type_mask": "VELOCITY_ONLY",
            "vx_mps": decision.setpoint.vx_mps,
            "vy_mps": decision.setpoint.vy_mps,
            "vz_mps": decision.setpoint.vz_mps,
            "yaw_rate_rps": decision.setpoint.yaw_rate_rps,
            "mode_hint": mode,
            "rate_hz": self._cfg.offboard_rate_hz,
            "safe_path": True,
        }
