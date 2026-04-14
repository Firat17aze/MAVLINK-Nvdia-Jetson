from __future__ import annotations

from pathlib import Path

REQUIRED_KEYS = {
    "MAV_1_CONFIG",
    "MAV_1_MODE",
    "MAV_1_RATE",
    "SER_TEL2_BAUD",
    "COM_RC_IN_MODE",
    "COM_OBL_ACT",
    "COM_OF_LOSS_T",
    "NAV_RCL_ACT",
    "GF_ACTION",
    "GF_MAX_HOR_DIST",
    "GF_MAX_VER_DIST",
    "MIS_TAKEOFF_ALT",
}


def parse_param_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    with path.open("r", encoding="utf-8") as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    return values


def validate_param_profile(path: Path) -> list[str]:
    errors: list[str] = []
    values = parse_param_file(path)

    missing = sorted(REQUIRED_KEYS.difference(values.keys()))
    if missing:
        errors.append(f"missing keys: {', '.join(missing)}")

    try:
        baud = int(values.get("SER_TEL2_BAUD", "0"))
        if baud < 57600:
            errors.append("SER_TEL2_BAUD must be >= 57600")
    except ValueError:
        errors.append("SER_TEL2_BAUD must be an integer")

    try:
        offboard_timeout = float(values.get("COM_OF_LOSS_T", "0"))
        if offboard_timeout <= 0.0 or offboard_timeout > 2.0:
            errors.append("COM_OF_LOSS_T must be in (0.0, 2.0]")
    except ValueError:
        errors.append("COM_OF_LOSS_T must be numeric")

    try:
        radius = float(values.get("GF_MAX_HOR_DIST", "0"))
        if radius < 50.0:
            errors.append("GF_MAX_HOR_DIST must be >= 50")
    except ValueError:
        errors.append("GF_MAX_HOR_DIST must be numeric")

    return errors


def main() -> None:
    path = Path("flight_stack/px4/params/offboard_profile.params")
    errors = validate_param_profile(path)
    if errors:
        for err in errors:
            print(err)
        raise SystemExit(1)
    print("offboard_profile.params validation passed")


if __name__ == "__main__":
    main()
