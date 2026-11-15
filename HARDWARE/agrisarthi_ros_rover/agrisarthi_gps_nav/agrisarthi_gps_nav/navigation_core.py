"""ROS-independent mission validation and waypoint control logic."""

from __future__ import annotations

import json
import math
from typing import Any

EARTH_RADIUS_M = 6_371_000.0


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1 - a)))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def angle_error(target_deg: float, current_deg: float) -> float:
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def _coordinate(point: Any, label: str) -> tuple[float, float]:
    if not isinstance(point, dict):
        raise ValueError(f"{label} must be an object")
    try:
        lat, lon = float(point["latitude"]), float(point["longitude"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain numeric latitude and longitude") from exc
    if not math.isfinite(lat) or not math.isfinite(lon) or not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError(f"{label} has invalid WGS84 coordinates")
    return lat, lon


def _local_xy(point: tuple[float, float], origin: tuple[float, float]) -> tuple[float, float]:
    lat, lon = point
    lat0, lon0 = origin
    x = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * EARTH_RADIUS_M
    return x, y


def _orientation(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_intersect(a: tuple[float, float], b: tuple[float, float],
                        c: tuple[float, float], d: tuple[float, float]) -> bool:
    o1, o2 = _orientation(a, b, c), _orientation(a, b, d)
    o3, o4 = _orientation(c, d, a), _orientation(c, d, b)
    eps = 1e-8
    if o1 * o2 < -eps and o3 * o4 < -eps:
        return True

    def collinear_on_segment(p, q, r):
        return (abs(_orientation(p, q, r)) <= eps
                and min(p[0], q[0]) - eps <= r[0] <= max(p[0], q[0]) + eps
                and min(p[1], q[1]) - eps <= r[1] <= max(p[1], q[1]) + eps)

    return (collinear_on_segment(a, b, c) or collinear_on_segment(a, b, d)
            or collinear_on_segment(c, d, a) or collinear_on_segment(c, d, b))


def _on_segment(point: tuple[float, float], a: tuple[float, float],
                b: tuple[float, float], tolerance_m: float = 0.25) -> bool:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    if length2 == 0:
        return math.hypot(point[0] - a[0], point[1] - a[1]) <= tolerance_m
    t = max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2))
    return math.hypot(point[0] - (a[0] + t * dx), point[1] - (a[1] + t * dy)) <= tolerance_m


def _point_in_polygon(point: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    inside = False
    x, y = point
    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]
        if _on_segment(point, a, b):
            return True
        if (a[1] > y) != (b[1] > y):
            cross_x = (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]
            if x < cross_x:
                inside = not inside
    return inside


def validate_mission(payload: str | dict[str, Any], max_waypoints: int = 5000) -> dict[str, Any]:
    """Validate a mission and return normalized coordinates and its geofence."""
    if isinstance(payload, str):
        try:
            packet = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSON: {exc.msg}") from exc
    else:
        packet = payload
    if not isinstance(packet, dict):
        raise ValueError("mission must be a JSON object")
    raw_boundary = packet.get("boundary")
    if not isinstance(raw_boundary, list) or len(raw_boundary) < 3:
        raise ValueError("mission must include a boundary with at least three corners")
    boundary_geo = [_coordinate(p, f"boundary corner {i + 1}") for i, p in enumerate(raw_boundary)]
    # GeoJSON-style polygon rings may repeat the first corner at the end.
    if len(boundary_geo) > 3 and boundary_geo[0] == boundary_geo[-1]:
        boundary_geo.pop()
    if len(boundary_geo) < 3:
        raise ValueError("mission boundary must contain at least three distinct corners")
    origin = (sum(p[0] for p in boundary_geo) / len(boundary_geo),
              sum(p[1] for p in boundary_geo) / len(boundary_geo))
    boundary = [_local_xy(p, origin) for p in boundary_geo]
    area2 = sum(boundary[i][0] * boundary[(i + 1) % len(boundary)][1]
                - boundary[(i + 1) % len(boundary)][0] * boundary[i][1]
                for i in range(len(boundary)))
    n = len(boundary)
    for i in range(n):
        a, b = boundary[i], boundary[(i + 1) % n]
        if math.dist(a, b) < 0.1:
            raise ValueError("mission boundary contains duplicate or near-duplicate corners")
        for j in range(i + 1, n):
            if j == i or j == (i + 1) % n or i == (j + 1) % n:
                continue
            if _segments_intersect(a, b, boundary[j], boundary[(j + 1) % n]):
                raise ValueError("mission boundary self-intersects")
    if abs(area2) < 2.0:
        raise ValueError("mission boundary is degenerate or smaller than one square metre")
    raw_waypoints = packet.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("waypoints must be a non-empty array")
    if len(raw_waypoints) > max_waypoints:
        raise ValueError(f"mission exceeds the {max_waypoints} waypoint limit")
    points = []
    waypoint_ids = set()
    for i, raw in enumerate(raw_waypoints):
        if not isinstance(raw, dict):
            raise ValueError(f"waypoint {i + 1} must be an object")
        lat, lon = _coordinate(raw, f"waypoint {i + 1}")
        if not _point_in_polygon(_local_xy((lat, lon), origin), boundary):
            raise ValueError(f"waypoint {i + 1} lies outside the mission boundary")
        waypoint_id = raw.get("id", i + 1)
        if (isinstance(waypoint_id, bool) or not isinstance(waypoint_id, (int, str))
                or str(waypoint_id) == ""):
            raise ValueError(f"waypoint {i + 1} has an invalid ID")
        key = str(waypoint_id)
        if key in waypoint_ids:
            raise ValueError(f"duplicate waypoint ID {key!r}")
        waypoint_ids.add(key)
        points.append({"id": waypoint_id, "latitude": lat, "longitude": lon})
    if "home_point" in packet:
        home_lat, home_lon = _coordinate(packet["home_point"], "home_point")
        if not _point_in_polygon(_local_xy((home_lat, home_lon), origin), boundary):
            raise ValueError("home_point lies outside the mission boundary")
        result_home = {"latitude": home_lat, "longitude": home_lon}
    else:
        result_home = None
    result = dict(packet)
    result["mission_id"] = str(packet.get("mission_id", "UNNAMED"))
    result["boundary"] = [{"latitude": lat, "longitude": lon} for lat, lon in boundary_geo]
    result["waypoints"] = points
    if result_home is not None:
        result["home_point"] = result_home
    return result


class NavigationController:
    """Safety state machine and steering calculation independent of ROS."""

    def __init__(self, *, waypoint_tolerance_m: float = 1.5, gps_timeout_s: float = 2.0,
                 heading_timeout_s: float = 1.0, max_linear_speed: float = 0.18,
                 max_angular_speed: float = 0.35, heading_kp: float = 0.012,
                 turn_in_place_error_deg: float = 55.0,
                 max_heading_error_for_full_speed_deg: float = 35.0,
                 minimum_speed_scale: float = 0.2, slow_down_distance_m: float = 3.0,
                 return_home: bool = False):
        self.tolerance = max(0.1, waypoint_tolerance_m)
        self.gps_timeout = max(0.1, gps_timeout_s)
        self.heading_timeout = max(0.1, heading_timeout_s)
        self.max_linear = max(0.0, max_linear_speed)
        self.max_angular = max(0.0, max_angular_speed)
        self.heading_kp = max(0.0, heading_kp)
        self.turn_threshold = max(0.0, turn_in_place_error_deg)
        self.full_speed_error = max(1.0, max_heading_error_for_full_speed_deg)
        self.minimum_speed_scale = max(0.0, min(1.0, minimum_speed_scale))
        self.slow_down_distance = max(self.tolerance, slow_down_distance_m)
        self.return_home_enabled = bool(return_home)
        self.home_point: tuple[float, float] | None = None
        self.returning_home = False
        self.mission_id = ""
        self.points: list[dict[str, Any]] = []
        self.boundary: list[tuple[float, float]] = []
        self.origin = (0.0, 0.0)
        self.index = 0
        self.arrived = False
        self.started = False
        self.estop = False
        self.fault: str | None = None
        self.state = "IDLE"
        self.gps: tuple[float, float] | None = None
        self.gps_t = 0.0
        self.heading: float | None = None
        self.heading_t = 0.0

    def load_mission(self, payload: str | dict[str, Any], max_waypoints: int = 5000) -> dict[str, Any]:
        self.clear_mission("MISSION_REJECTED")
        try:
            packet = validate_mission(payload, max_waypoints=max_waypoints)
        except (ValueError, TypeError) as exc:
            self.state = "MISSION_REJECTED"
            raise ValueError(str(exc)) from exc
        self.mission_id = packet["mission_id"]
        self.points = packet["waypoints"]
        self.home_point = ((packet["home_point"]["latitude"], packet["home_point"]["longitude"])
                           if self.return_home_enabled and "home_point" in packet else None)
        self.origin = (sum(p["latitude"] for p in packet["boundary"]) / len(packet["boundary"]),
                       sum(p["longitude"] for p in packet["boundary"]) / len(packet["boundary"]))
        self.boundary = [_local_xy((p["latitude"], p["longitude"]), self.origin)
                         for p in packet["boundary"]]
        self.index = 0
        self.arrived = False
        self.returning_home = False
        self.fault = None
        self.state = "MISSION_READY"
        return packet

    def clear_mission(self, state: str = "IDLE") -> None:
        self.started = False
        self.mission_id = ""
        self.points = []
        self.boundary = []
        self.index = 0
        self.arrived = False
        self.home_point = None
        self.returning_home = False
        self.fault = None
        self.state = state

    def start(self) -> bool:
        if (self.estop or self.fault or self.arrived or not self.points
                or (self.index >= len(self.points) and not self.returning_home)):
            return False
        self.started = True
        self.state = "WAITING_FOR_SENSORS"
        return True

    def pause(self, state: str = "PAUSED") -> None:
        self.started = False
        self.state = state

    def complete_waypoint(self, waypoint_id: Any) -> str:
        """Advance only after the sampling authority acknowledges a saved sample."""
        if not self.arrived or self.index >= len(self.points):
            raise ValueError("waypoint completion received while not held at a waypoint")
        current_id = self.points[self.index]["id"]
        if str(waypoint_id) != str(current_id):
            raise ValueError(f"waypoint completion ID {waypoint_id!r} does not match {current_id!r}")
        self.index += 1
        self.arrived = False
        self.started = False
        if self.index >= len(self.points) and self.home_point is not None:
            self.returning_home = True
            self.state = "RETURNING_HOME"
        else:
            self.state = "MISSION_COMPLETE" if self.index >= len(self.points) else "WAYPOINT_COMPLETE"
        return self.state

    def set_gps(self, lat: float, lon: float, now: float, valid: bool = True) -> None:
        if not valid or not math.isfinite(lat) or not math.isfinite(lon) or not -90 <= lat <= 90 or not -180 <= lon <= 180:
            self.gps = None
            return
        self.gps = (lat, lon)
        self.gps_t = now

    def set_heading(self, heading_deg: float, now: float) -> None:
        if not math.isfinite(heading_deg):
            self.heading = None
            return
        self.heading = heading_deg % 360.0
        self.heading_t = now

    def step(self, now: float) -> dict[str, Any]:
        stop = {"left": 0.0, "right": 0.0, "linear": 0.0, "angular": 0.0, "enable": False}
        if self.estop:
            self.pause("ESTOP")
            return {**stop, "state": self.state, "detail": "software E-stop active"}
        if self.fault:
            self.pause(self.fault)
            return {**stop, "state": self.state, "detail": "navigation fault; load a new mission"}
        if not self.points:
            if self.state not in ("MISSION_REJECTED", "IDLE"):
                self.state = "IDLE"
            return {**stop, "state": self.state, "detail": "no valid mission"}
        if not self.started:
            if self.arrived:
                return {**stop, "state": "WAYPOINT_REACHED", "detail": "waiting for sample completion"}
            if self.state not in ("MISSION_READY", "PAUSED", "WAYPOINT_COMPLETE", "RETURNING_HOME", "MISSION_COMPLETE"):
                self.state = "MISSION_READY"
            return {**stop, "state": self.state, "detail": "explicit mission start required"}
        gps_ok = self.gps is not None and now - self.gps_t <= self.gps_timeout
        heading_ok = self.heading is not None and now - self.heading_t <= self.heading_timeout
        if not gps_ok or not heading_ok:
            self.pause("WAITING_FOR_SENSORS")
            reason = "GPS invalid or stale" if not gps_ok else "heading invalid or stale"
            return {**stop, "state": self.state, "detail": f"{reason}; explicit start required after recovery"}
        assert self.gps is not None and self.heading is not None
        if not _point_in_polygon(_local_xy(self.gps, self.origin), self.boundary):
            self.fault = "GEOFENCE_BREACH"
            self.pause(self.fault)
            return {**stop, "state": self.state, "detail": "GPS position is outside mission geofence"}
        point = ({"id": "__HOME__", "latitude": self.home_point[0], "longitude": self.home_point[1]}
                 if self.returning_home else self.points[self.index])
        dist = distance_m(*self.gps, point["latitude"], point["longitude"])
        if dist <= self.tolerance:
            if self.returning_home:
                self.returning_home = False
                self.pause("MISSION_COMPLETE")
                return {**stop, "state": self.state, "detail": "returned to home point", "distance_m": dist}
            self.arrived = True
            self.pause("WAYPOINT_REACHED")
            return {**stop, "state": self.state,
                    "detail": f"waypoint {point['id']} reached; sampling completion required",
                    "waypoint_id": point["id"], "distance_m": dist}
        bearing = bearing_deg(*self.gps, point["latitude"], point["longitude"])
        error = angle_error(bearing, self.heading)
        # Positive clockwise heading error means turn right: left wheel faster.
        angular = max(-self.max_angular, min(self.max_angular, self.heading_kp * error))
        if abs(error) >= self.turn_threshold:
            linear = 0.0
        else:
            heading_scale = max(self.minimum_speed_scale, 1.0 - abs(error) / self.full_speed_error)
            distance_scale = min(1.0, max(0.0, dist / self.slow_down_distance))
            linear = self.max_linear * heading_scale * distance_scale
        left, right = linear + angular, linear - angular
        peak = max(1.0, abs(left), abs(right))
        left, right = left / peak, right / peak
        self.state = "NAVIGATING"
        return {"left": left, "right": right, "linear": linear, "angular": angular,
                "enable": True, "state": self.state,
                "detail": f"wp={self.index + 1}/{len(self.points)} distance={dist:.2f}m bearing={bearing:.1f} heading_error={error:.1f}",
                "distance_m": dist, "bearing_deg": bearing, "heading_error_deg": error}
