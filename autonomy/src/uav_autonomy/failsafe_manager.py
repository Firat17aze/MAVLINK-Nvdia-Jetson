from __future__ import annotations

from .models import FailsafeAction, FaultSnapshot


class FailsafeManager:
    def select(self, faults: FaultSnapshot) -> FailsafeAction:
        if faults.nav_fault:
            return FailsafeAction.LOITER_MANUAL
        if faults.low_battery:
            return FailsafeAction.RTL
        if faults.companion_loss:
            return FailsafeAction.PX4_FAILSAFE
        if faults.ai_timeout:
            return FailsafeAction.HOLD
        return FailsafeAction.NONE
