from pathlib import Path

from flight_stack.px4.scripts.check_param_profile import validate_param_profile


def test_px4_param_profile_validates() -> None:
    errors = validate_param_profile(Path("flight_stack/px4/params/offboard_profile.params"))
    assert errors == []


def test_companion_guard_module_assets_exist() -> None:
    cmake = Path("flight_stack/px4/firmware/companion_guard/CMakeLists.txt")
    source = Path("flight_stack/px4/firmware/companion_guard/companion_guard.cpp")

    assert cmake.exists()
    assert source.exists()


def test_companion_guard_has_required_guard_actions() -> None:
    source = Path("flight_stack/px4/firmware/companion_guard/companion_guard.cpp").read_text(
        encoding="utf-8"
    )
    assert "NAVIGATION_STATE_OFFBOARD" in source
    assert "VEHICLE_CMD_NAV_LOITER_UNLIM" in source
    assert "VEHICLE_CMD_NAV_RETURN_TO_LAUNCH" in source
