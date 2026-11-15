"""Strict soil-sample mission JSON schema and crash-safe persistence."""

from __future__ import annotations

import json
import math
import os
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Any

ROOT_KEYS = {"rover_id", "mission_id", "home_point", "records"}
RECORD_KEYS = {"waypoint_id", "timestamp", "gps", "soil"}
COORDINATE_KEYS = {"latitude", "longitude"}
SOIL_KEYS = {"nitrogen", "phosphorus", "potassium", "ph", "moisture", "temperature", "ec"}


class MissionDataError(ValueError):
    pass


class DuplicateWaypointError(MissionDataError):
    pass


def _validate_coordinate(point: Any, label: str) -> dict[str, float]:
    if not isinstance(point, dict) or set(point) != COORDINATE_KEYS:
        raise MissionDataError(f"{label} must contain only latitude and longitude")
    try:
        lat, lon = float(point["latitude"]), float(point["longitude"])
    except (TypeError, ValueError) as exc:
        raise MissionDataError(f"{label} coordinates must be numeric") from exc
    if not math.isfinite(lat) or not math.isfinite(lon) or not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise MissionDataError(f"{label} coordinates are outside WGS84 bounds")
    return {"latitude": lat, "longitude": lon}


def validate_record(record: Any) -> dict:
    if not isinstance(record, dict) or set(record) != RECORD_KEYS:
        raise MissionDataError("each record must contain only waypoint_id, timestamp, gps and soil")
    waypoint_id = record["waypoint_id"]
    if isinstance(waypoint_id, bool) or not isinstance(waypoint_id, (int, str)) or str(waypoint_id) == "":
        raise MissionDataError("waypoint_id must be a non-empty string or integer")
    timestamp = record["timestamp"]
    if not isinstance(timestamp, str) or not timestamp.endswith("Z"):
        raise MissionDataError("timestamp must be a UTC ISO 8601 string ending in Z")
    try:
        datetime.fromisoformat(timestamp[:-1] + "+00:00")
    except ValueError as exc:
        raise MissionDataError("timestamp is not valid ISO 8601") from exc
    gps = _validate_coordinate(record["gps"], "record GPS")
    soil = record["soil"]
    if not isinstance(soil, dict) or set(soil) != SOIL_KEYS:
        raise MissionDataError("soil must contain the seven required measurements and no extra fields")
    normalized_soil = {}
    for key in SOIL_KEYS:
        value = soil[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise MissionDataError(f"soil.{key} must be a finite numeric reading")
        normalized_soil[key] = value
    return {"waypoint_id": waypoint_id, "timestamp": timestamp, "gps": gps, "soil": normalized_soil}


def validate_document(document: Any) -> dict:
    if not isinstance(document, dict) or set(document) != ROOT_KEYS:
        raise MissionDataError("mission document must contain only rover_id, mission_id, home_point and records")
    if not isinstance(document["rover_id"], str) or not document["rover_id"].strip():
        raise MissionDataError("rover_id must be a non-empty string")
    if not isinstance(document["mission_id"], str) or not document["mission_id"].strip():
        raise MissionDataError("mission_id must be a non-empty string")
    home = _validate_coordinate(document["home_point"], "home_point")
    records = document["records"]
    if not isinstance(records, list):
        raise MissionDataError("records must be an array")
    normalized = [validate_record(record) for record in records]
    seen = set()
    for record in normalized:
        key = str(record["waypoint_id"])
        if key in seen:
            raise DuplicateWaypointError(f"duplicate waypoint record {key}")
        seen.add(key)
    return {"rover_id": document["rover_id"], "mission_id": document["mission_id"],
            "home_point": home, "records": normalized}


def new_document(rover_id: str, mission_id: str, home_point: dict) -> dict:
    return validate_document({"rover_id": rover_id, "mission_id": mission_id,
                              "home_point": home_point, "records": []})


def append_record(document: dict, record: dict) -> dict:
    current = validate_document(document)
    normalized = validate_record(record)
    key = str(normalized["waypoint_id"])
    if any(str(existing["waypoint_id"]) == key for existing in current["records"]):
        raise DuplicateWaypointError(f"waypoint {key} is already saved")
    current["records"].append(normalized)
    return validate_document(current)


def atomic_write(path: str | Path, document: dict) -> None:
    """Validate and atomically replace a JSON file, flushing data and directory metadata."""
    path = Path(path).expanduser()
    normalized = validate_document(document)
    path.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8") as stream:
            json.dump(normalized, stream, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def load_document(path: str | Path) -> dict:
    try:
        with Path(path).expanduser().open(encoding="utf-8") as stream:
            parsed = json.load(stream, parse_constant=lambda value: (_ for _ in ()).throw(
                MissionDataError(f"invalid JSON numeric constant {value}")))
    except json.JSONDecodeError as exc:
        raise MissionDataError(f"invalid mission JSON: {exc.msg}") from exc
    return validate_document(parsed)
