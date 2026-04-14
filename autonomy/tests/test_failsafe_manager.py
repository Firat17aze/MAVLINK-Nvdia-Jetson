from uav_autonomy.failsafe_manager import FailsafeManager
from uav_autonomy.models import FailsafeAction, FaultSnapshot


def test_maps_ai_timeout_to_hold() -> None:
    manager = FailsafeManager()
    action = manager.select(
        FaultSnapshot(ai_timeout=True, companion_loss=False, low_battery=False, nav_fault=False)
    )
    assert action == FailsafeAction.HOLD


def test_maps_companion_loss_to_px4_failsafe() -> None:
    manager = FailsafeManager()
    action = manager.select(
        FaultSnapshot(ai_timeout=False, companion_loss=True, low_battery=False, nav_fault=False)
    )
    assert action == FailsafeAction.PX4_FAILSAFE


def test_maps_low_battery_to_rtl() -> None:
    manager = FailsafeManager()
    action = manager.select(
        FaultSnapshot(ai_timeout=False, companion_loss=False, low_battery=True, nav_fault=False)
    )
    assert action == FailsafeAction.RTL


def test_maps_nav_fault_to_loiter_manual() -> None:
    manager = FailsafeManager()
    action = manager.select(
        FaultSnapshot(ai_timeout=False, companion_loss=False, low_battery=False, nav_fault=True)
    )
    assert action == FailsafeAction.LOITER_MANUAL


def test_priority_prefers_nav_fault_over_other_faults() -> None:
    manager = FailsafeManager()
    action = manager.select(
        FaultSnapshot(ai_timeout=True, companion_loss=True, low_battery=True, nav_fault=True)
    )
    assert action == FailsafeAction.LOITER_MANUAL
