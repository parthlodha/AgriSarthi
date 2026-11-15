import json
import tempfile
import unittest
from pathlib import Path

from agrisarthi_gps_nav.mission_data import (
    DuplicateWaypointError,
    MissionDataError,
    append_record,
    atomic_write,
    load_document,
    new_document,
    validate_document,
)


def sample_record(waypoint_id=1):
    return {
        "waypoint_id": waypoint_id,
        "timestamp": "2026-09-26T20:15:30Z",
        "gps": {"latitude": 19.076, "longitude": 72.8777},
        "soil": {"nitrogen": 72, "phosphorus": 38, "potassium": 165, "ph": 6.4,
                 "moisture": 58.3, "temperature": 27.5, "ec": 1450},
    }


class MissionDataTests(unittest.TestCase):
    def setUp(self):
        self.doc = new_document("ROVER_01", "MISSION_001",
                                {"latitude": 19.076, "longitude": 72.8777})

    def test_record_schema_contains_only_requested_fields(self):
        result = append_record(self.doc, sample_record())
        self.assertEqual(set(result), {"rover_id", "mission_id", "home_point", "records"})
        self.assertEqual(set(result["records"][0]), {"waypoint_id", "timestamp", "gps", "soil"})
        self.assertNotIn("rover", result["records"][0])
        for telemetry in ("battery", "speed", "status"):
            self.assertNotIn(telemetry, result["records"][0])

    def test_rejects_rover_telemetry_or_extra_record_fields(self):
        record = sample_record()
        record["rover"] = {"battery": 87, "speed": 0.0, "status": "SAMPLED"}
        with self.assertRaisesRegex(MissionDataError, "only waypoint_id"):
            append_record(self.doc, record)

    def test_failed_or_nonfinite_soil_data_is_rejected(self):
        record = sample_record()
        record["soil"]["nitrogen"] = float("nan")
        with self.assertRaisesRegex(MissionDataError, "finite"):
            append_record(self.doc, record)

    def test_duplicate_waypoint_is_rejected(self):
        once = append_record(self.doc, sample_record())
        with self.assertRaises(DuplicateWaypointError):
            append_record(once, sample_record())

    def test_incremental_atomic_file_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "MISSION_001.json"
            atomic_write(path, self.doc)
            loaded = load_document(path)
            loaded = append_record(loaded, sample_record())
            atomic_write(path, loaded)
            self.assertEqual(load_document(path), loaded)
            self.assertEqual(set(json.loads(path.read_text())["records"][0]),
                             {"waypoint_id", "timestamp", "gps", "soil"})

    def test_exact_top_level_schema_rejects_telemetry(self):
        malformed = dict(self.doc, battery=87)
        with self.assertRaisesRegex(MissionDataError, "only rover_id"):
            validate_document(malformed)


if __name__ == "__main__":
    unittest.main()
