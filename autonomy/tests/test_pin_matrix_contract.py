from pathlib import Path

from scripts.validate_pin_matrix import validate_pin_matrix


def test_pin_matrix_schema_and_values_are_valid() -> None:
    errors = validate_pin_matrix(Path("hardware/pin_matrix.csv"))
    assert errors == []
