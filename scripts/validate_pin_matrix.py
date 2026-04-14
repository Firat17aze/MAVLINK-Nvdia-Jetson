from __future__ import annotations

import csv
from pathlib import Path

REQUIRED_COLUMNS = [
    "FC port",
    "FC pin",
    "signal",
    "voltage domain",
    "peer endpoint",
    "cable/part number",
    "bench validation result",
]
VALID_RESULTS = {"PASS", "FAIL", "PENDING"}


def validate_pin_matrix(path: Path) -> list[str]:
    errors: list[str] = []
    with path.open("r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            return ["Missing CSV header"]

        missing_cols = [c for c in REQUIRED_COLUMNS if c not in reader.fieldnames]
        if missing_cols:
            errors.append(f"Missing required columns: {missing_cols}")
            return errors

        for idx, row in enumerate(reader, start=2):
            for column in REQUIRED_COLUMNS:
                if not str(row.get(column, "")).strip():
                    errors.append(f"Row {idx}: empty '{column}'")
            result = str(row.get("bench validation result", "")).strip().upper()
            if result not in VALID_RESULTS:
                errors.append(f"Row {idx}: invalid bench result '{result}'")

    return errors


def main() -> None:
    path = Path("hardware/pin_matrix.csv")
    errors = validate_pin_matrix(path)
    if errors:
        for err in errors:
            print(err)
        raise SystemExit(1)
    print("pin_matrix.csv validation passed")


if __name__ == "__main__":
    main()
