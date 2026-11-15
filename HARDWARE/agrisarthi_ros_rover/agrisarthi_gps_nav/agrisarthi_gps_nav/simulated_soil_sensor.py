"""Deterministic simulation-only soil readings; never accesses serial hardware."""

from __future__ import annotations

from .soil_sensor import SoilReadResult


class SimulatedSoilSensor:
    def __init__(self, fail_attempts: int = 0):
        self.fail_attempts_remaining = max(0, int(fail_attempts))
        self.successful_samples = 0

    def fail_next(self, count: int) -> None:
        self.fail_attempts_remaining += max(0, int(count))

    def read(self) -> SoilReadResult:
        if self.fail_attempts_remaining:
            self.fail_attempts_remaining -= 1
            return SoilReadResult(False, None, "SIMULATED_SENSOR_READ_FAILURE", 1)
        self.successful_samples += 1
        sample = self.successful_samples
        # Synthetic bounded values solely for software integration testing.
        readings = {
            "nitrogen": 68 + 2 * sample,
            "phosphorus": 31 + sample,
            "potassium": 150 + 3 * sample,
            "ph": round(6.1 + 0.1 * sample, 1),
            "moisture": round(42.0 + 3.0 * sample, 1),
            "temperature": round(24.0 + 0.5 * sample, 1),
            "ec": 620 + 20 * sample,
        }
        return SoilReadResult(True, readings, None, 1)
