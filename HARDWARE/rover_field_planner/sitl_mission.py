"""Deterministic four-waypoint mission fixture for the laptop SITL demo."""

from datetime import datetime, timezone


def make_sitl_four_waypoint_mission(mission_id: str, rover_id: str,
                                    home_lat: float = 37.4219999,
                                    home_lon: float = -122.0840575) -> dict:
    d = 0.000035
    edge = 0.00010
    boundary = [
        {"latitude": home_lat - edge, "longitude": home_lon - edge},
        {"latitude": home_lat - edge, "longitude": home_lon + edge},
        {"latitude": home_lat + edge, "longitude": home_lon + edge},
        {"latitude": home_lat + edge, "longitude": home_lon - edge},
    ]
    offsets = ((d, -d), (d, d), (-d, d), (-d, -d))
    waypoints = [{"id": i, "latitude": home_lat + dy, "longitude": home_lon + dx}
                 for i, (dy, dx) in enumerate(offsets, 1)]
    return {
        "schema_version": 1,
        "mission_id": mission_id,
        "mission_name": "deterministic_four_waypoint_sitl",
        "rover_id": rover_id,
        "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z"),
        "coordinate_system": "WGS84",
        "coordinate_order": "latitude_longitude",
        "home_point": {"latitude": home_lat, "longitude": home_lon},
        "boundary": boundary,
        "spacing": {"between_lines_m": 0.0, "between_points_m": 0.0},
        "waypoint_count": 4,
        "waypoints": waypoints,
    }
