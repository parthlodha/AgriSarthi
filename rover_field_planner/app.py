"""Farmer-friendly field-boundary mission planner."""
from __future__ import annotations

import json
import math
import os
import re
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

import folium
import streamlit as st
from streamlit_folium import st_folium
from shapely.geometry import LineString, Polygon, MultiLineString, GeometryCollection
from sitl_mission import make_sitl_four_waypoint_mission

st.set_page_config(page_title="Rover Field Planner", page_icon="🌱", layout="wide", initial_sidebar_state="collapsed")
st.markdown("""
<style>
.block-container{padding-top:1rem;max-width:1500px}
.hero{background:linear-gradient(120deg,#155e36,#21864f);padding:1.15rem 1.4rem;border-radius:16px;color:white;margin-bottom:1rem}
.hero h1{color:white;margin:0;font-size:2rem}.hero p{color:#effff4;margin:.35rem 0 0}
div.stButton>button,div.stDownloadButton>button{min-height:3rem;font-size:1.02rem;font-weight:600;border-radius:10px}
.help{color:#536579;font-size:.92rem}
</style>
<div class="hero"><h1>🌱 Rover Field Planner</h1><p>Mark your field boundary. The rover will plan soil-sampling passes automatically.</p></div>
""", unsafe_allow_html=True)

MISSIONS_DIR = Path("missions")
MISSIONS_DIR.mkdir(exist_ok=True)
DEFAULT_ROS_URL = os.environ.get("AGRISARTHI_ROSBRIDGE_URL", "ws://192.168.1.12:9090")
DEFAULT_GPS_TOPIC = os.environ.get("AGRISARTHI_GPS_TOPIC", "/gps/fix")
SITL_DEMO_ENABLED = os.environ.get("AGRISARTHI_SITL_DEMO", "0").lower() in ("1", "true", "yes")
SITL_HOME_LAT = float(os.environ.get("AGRISARTHI_SITL_HOME_LAT", "37.4219999"))
SITL_HOME_LON = float(os.environ.get("AGRISARTHI_SITL_HOME_LON", "-122.0840575"))

def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")

def valid_coord(lat, lon):
    try:
        lat, lon = float(lat), float(lon)
        return math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180
    except (TypeError, ValueError):
        return False

def next_mission_id():
    ids = []
    for p in MISSIONS_DIR.glob("mission_*.json"):
        m = re.fullmatch(r"mission_(\d{3,})\.json", p.name)
        if m:
            ids.append(int(m.group(1)))
    return f"{max(ids, default=0) + 1:03d}"

def local_xy(lat, lon, lat0, lon0):
    # Local equirectangular projection, suitable for small fields.
    r = 6371000.0
    x = math.radians(lon - lon0) * r * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * r
    return x, y

def latlon_xy(x, y, lat0, lon0):
    r = 6371000.0
    lat = lat0 + math.degrees(y / r)
    c = max(1e-8, math.cos(math.radians(lat0)))
    lon = lon0 + math.degrees(x / (r * c))
    return lat, lon

def line_parts(geom):
    if geom.is_empty:
        return []
    if isinstance(geom, LineString):
        return [geom]
    if isinstance(geom, (MultiLineString, GeometryCollection)):
        out = []
        for g in geom.geoms:
            out.extend(line_parts(g))
        return out
    return []

def generate_lawnmower(boundary, line_spacing_m, point_spacing_m):
    """Create serpentine parallel passes inside the boundary; return lat/lon sampling points and route."""
    if len(boundary) < 3:
        raise ValueError("Mark at least three boundary corners.")
    lat0 = sum(p["latitude"] for p in boundary) / len(boundary)
    lon0 = sum(p["longitude"] for p in boundary) / len(boundary)
    xy = [local_xy(p["latitude"], p["longitude"], lat0, lon0) for p in boundary]
    poly = Polygon(xy)
    if not poly.is_valid or poly.is_empty or poly.area < 1:
        raise ValueError("The boundary is invalid or too small. Redraw the field boundary.")
    minx, miny, maxx, maxy = poly.bounds
    # Passes are vertical in local coordinates (roughly north-south).
    x_values = []
    x = minx + line_spacing_m / 2.0
    while x < maxx:
        x_values.append(x)
        x += line_spacing_m
    if not x_values:
        x_values = [(minx + maxx) / 2.0]
    samples = []
    route = []
    pass_index = 0
    for x in x_values:
        inter = poly.intersection(LineString([(x, miny - 1), (x, maxy + 1)]))
        segments = sorted(line_parts(inter), key=lambda seg: seg.bounds[1])
        for seg in segments:
            coords = list(seg.coords)
            if len(coords) < 2:
                continue
            start, end = coords[0], coords[-1]
            if pass_index % 2:
                start, end = end, start
            length = math.dist(start, end)
            n = max(1, int(math.floor(length / point_spacing_m)))
            seg_points = []
            for i in range(n + 1):
                d = min(i * point_spacing_m, length)
                t = 0 if length == 0 else d / length
                px = start[0] + (end[0] - start[0]) * t
                py = start[1] + (end[1] - start[1]) * t
                seg_points.append((px, py))
            # Ensure the pass endpoint is represented.
            if math.dist(seg_points[-1], end) > 0.5:
                seg_points.append(end)
            for px, py in seg_points:
                lat, lon = latlon_xy(px, py, lat0, lon0)
                p = {"latitude": round(lat, 7), "longitude": round(lon, 7)}
                if not samples or math.hypot(px - local_xy(samples[-1]["latitude"], samples[-1]["longitude"], lat0, lon0)[0],
                                             py - local_xy(samples[-1]["latitude"], samples[-1]["longitude"], lat0, lon0)[1]) > 0.5:
                    samples.append(p)
                    route.append([lat, lon])
            pass_index += 1
    if len(samples) < 2:
        raise ValueError("Could not create a route. Try a larger field or smaller spacing.")
    boundary_area_m2 = poly.area
    return samples, route, boundary_area_m2

def publish_ros(url, topic, payload):
    """Publish mission JSON as std_msgs/msg/String through rosbridge."""
    import websocket
    if not url.startswith(("ws://", "wss://")):
        return False, "The ROS connection address must start with ws:// or wss://."
    ws = None
    try:
        ws = websocket.create_connection(url, timeout=5)
        ws.send(json.dumps({"op": "advertise", "topic": topic, "type": "std_msgs/msg/String"}))
        ws.send(json.dumps({"op": "publish", "topic": topic, "msg": {"data": payload}}))
        time.sleep(0.1)  # Let rosbridge dispatch the publish before closing this short-lived socket.
        return True, "Mission sent to the ROS bridge. This confirms transmission only, not rover acceptance."
    except Exception as e:
        return False, f"Could not reach ROS bridge: {type(e).__name__}: {e}"
    finally:
        if ws:
            try: ws.close()
            except Exception: pass

def publish_ros_bool(url, topic, value):
    """Publish a std_msgs/msg/Bool command through rosbridge."""
    import websocket
    if not url.startswith(("ws://", "wss://")):
        return False, "The ROS connection address must start with ws:// or wss://."
    ws = None
    try:
        ws = websocket.create_connection(url, timeout=5)
        ws.send(json.dumps({"op": "advertise", "topic": topic, "type": "std_msgs/msg/Bool"}))
        ws.send(json.dumps({"op": "publish", "topic": topic, "msg": {"data": bool(value)}}))
        time.sleep(0.1)  # Avoid dropping the final message when the client closes immediately.
        command = "START" if value else "STOP/PAUSE"
        return True, f"{command} command sent to {topic}. This confirms transmission only, not rover state."
    except Exception as e:
        return False, f"Could not reach ROS bridge: {type(e).__name__}: {e}"
    finally:
        if ws:
            try: ws.close()
            except Exception: pass


def request_one_gps_fix(url: str, topic: str, timeout: float = 10.0):
    """Subscribe once, return the first valid NavSatFix, then close the WebSocket."""
    import websocket

    if not url.startswith(("ws://", "wss://")):
        return None, "The ROS connection address must start with ws:// or wss://."

    ws = None
    try:
        ws = websocket.create_connection(url, timeout=timeout)
        ws.settimeout(timeout)
        ws.send(json.dumps({
            "op": "subscribe",
            "topic": topic,
            "type": "sensor_msgs/msg/NavSatFix",
            "throttle_rate": 0,
            "queue_length": 1,
        }))
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                raw = ws.recv()
            except websocket.WebSocketTimeoutException:
                break
            if not raw:
                break
            packet = json.loads(raw)
            if packet.get("op") != "publish" or packet.get("topic") != topic:
                continue
            msg = packet.get("msg", {})
            lat, lon = msg.get("latitude"), msg.get("longitude")
            if not valid_coord(lat, lon):
                continue
            covariance = msg.get("position_covariance") or []
            horizontal_variance = None
            if len(covariance) >= 5:
                try:
                    horizontal_variance = max(float(covariance[0]), float(covariance[4]))
                except (TypeError, ValueError):
                    pass
            return {
                "latitude": float(lat),
                "longitude": float(lon),
                "altitude": float(msg.get("altitude", 0.0) or 0.0),
                "horizontal_variance_m2": horizontal_variance,
                "received_at": time.time(),
            }, None
        return None, f"No valid GPS fix received within {timeout:.0f} seconds."
    except Exception as exc:
        return None, f"{type(exc).__name__}: {exc}"
    finally:
        if ws is not None:
            try:
                ws.send(json.dumps({"op": "unsubscribe", "topic": topic}))
            except Exception:
                pass


def request_ros_snapshot(url: str, query_topic: str, status_topic: str, data_topic: str,
                         timeout: float = 5.0):
    """Request a Pi mission status/result snapshot using only rosbridge topics."""
    import websocket

    if not url.startswith(("ws://", "wss://")):
        return None, None, "The ROS connection address must start with ws:// or wss://."
    request_id = uuid.uuid4().hex
    ws = None
    try:
        ws = websocket.create_connection(url, timeout=timeout)
        ws.settimeout(0.5)
        for topic in (status_topic, data_topic):
            ws.send(json.dumps({"op": "subscribe", "topic": topic,
                                "type": "std_msgs/msg/String", "throttle_rate": 0, "queue_length": 1}))
        ws.send(json.dumps({"op": "advertise", "topic": query_topic, "type": "std_msgs/msg/String"}))
        query = json.dumps({"request_id": request_id, "action": "GET_SNAPSHOT"})
        ws.send(json.dumps({"op": "publish", "topic": query_topic, "msg": {"data": query}}))
        status = data = None
        status_received = data_received = False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not (status_received and data_received):
            try:
                packet = json.loads(ws.recv())
            except websocket.WebSocketTimeoutException:
                continue
            if packet.get("op") != "publish" or packet.get("topic") not in (status_topic, data_topic):
                continue
            try:
                response = json.loads(packet.get("msg", {}).get("data", ""))
            except (json.JSONDecodeError, TypeError, AttributeError):
                continue
            if response.get("request_id") != request_id:
                continue
            if packet["topic"] == status_topic:
                status = response.get("status")
                status_received = True
            else:
                data = response.get("data")
                data_received = True
        if not status_received and not data_received:
            return None, None, "No response from the Raspberry Pi mission manager."
        if not status_received or not data_received:
            return status, data, "Raspberry Pi returned only part of the mission snapshot."
        return status, data, None
    except Exception as exc:
        return None, None, f"{type(exc).__name__}: {exc}"
    finally:
        if ws is not None:
            for topic in (status_topic, data_topic):
                try:
                    ws.send(json.dumps({"op": "unsubscribe", "topic": topic}))
                except Exception:
                    pass
            try:
                ws.close()
            except Exception:
                pass


# State
for key, default in {
    "boundary": [], "map_center": [18.5204, 73.8567], "gps_map_centered": False,
    "last_map_click": None, "boundary_closed": False, "preview_points": [],
    "preview_route": [], "preview_error": None, "preview_area": None, "last_saved_file": None,
    "rover_gps_fix": None, "rover_gps_attempted": False, "rover_gps_error": None,
    "rover_snapshot_status": None, "rover_snapshot_data": None,
    "rover_snapshot_attempted": False, "rover_snapshot_error": None
}.items():
    if key not in st.session_state:
        st.session_state[key] = default

with st.expander("⚙️ Project settings (administrator only)"):
    st.text_input("Rover ID", value="ROVER_01", key="rover_id")
    st.text_input("ROS bridge WebSocket URL", value=DEFAULT_ROS_URL, key="ros_url",
                  help="On cloud hosting, localhost is the cloud server—not the rover.")
    st.text_input("ROS mission topic", value="/mission/waypoints_json", key="ros_topic")
    st.text_input("ROS mission start/stop topic", value="/mission/start", key="ros_control_topic")
    st.text_input("ROS GPS topic", value=DEFAULT_GPS_TOPIC, key="ros_gps_topic")
    st.text_input("Mission query topic", value="/mission/query", key="ros_query_topic")
    st.text_input("Mission status response topic", value="/mission/status_response", key="ros_status_response_topic")
    st.text_input("Mission data response topic", value="/mission/data_response", key="ros_data_response_topic")
    st.caption("Mission JSON uses std_msgs/msg/String. Start/stop uses std_msgs/msg/Bool: true starts/resumes; false pauses navigation while an active stationary sampling cycle completes. This is not an emergency stop. Use a secured connection; do not expose rosbridge publicly without access controls.")

if SITL_DEMO_ENABLED:
    st.subheader("Software-in-the-loop mission")
    st.caption("This sends four deterministic demo waypoints to the local ROS simulation; sensor values are synthetic.")
    if st.button("Send four-waypoint SITL mission", key="send_sitl_mission"):
        demo_id = "SITL_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        demo = make_sitl_four_waypoint_mission(
            demo_id, st.session_state.get("rover_id", "ROVER_01"))
        payload = json.dumps(demo, separators=(",", ":"))
        filename = f"mission_{demo_id}.json"
        save_path = MISSIONS_DIR / filename
        save_path.write_text(json.dumps(demo, indent=2), encoding="utf-8")
        ok, message = publish_ros(st.session_state.get("ros_url", DEFAULT_ROS_URL),
                                  st.session_state.get("ros_topic", "/mission/waypoints_json"), payload)
        st.session_state.last_send_result = (ok, message, filename, payload)
        if ok:
            st.success(f"SITL mission {demo_id} sent. Start it explicitly using Start / Resume.")
        else:
            st.error(f"Mission saved on this PC but not sent: {message}")

# One-time rover GPS acquisition. The result is stored in session state and never polled.
gps_url = st.session_state.get("ros_url", DEFAULT_ROS_URL)
gps_topic = st.session_state.get("ros_gps_topic", "/gps/fix")
if not st.session_state.rover_gps_attempted:
    with st.spinner("Requesting one GPS fix from the rover..."):
        fix, error = request_one_gps_fix(gps_url, gps_topic, timeout=10.0)
    st.session_state.rover_gps_fix = fix
    st.session_state.rover_gps_error = error
    st.session_state.rover_gps_attempted = True
    if fix and not st.session_state.gps_map_centered:
        st.session_state.map_center = [fix["latitude"], fix["longitude"]]
        st.session_state.gps_map_centered = True

gps_data = st.session_state.rover_gps_fix
if gps_data:
    st.success(f"GPS fix acquired · {gps_topic}")
    st.caption(
        f"Rover position (one-time fix): {gps_data['latitude']:.7f}, "
        f"{gps_data['longitude']:.7f}"
    )
else:
    st.warning(f"Rover GPS fix not acquired: {st.session_state.rover_gps_error}")
    if st.button("Retry GPS fix"):
        st.session_state.rover_gps_attempted = False
        st.rerun()

st.subheader("Rover mission status and saved samples")
st.caption("Fetches the current mission state and persisted sample JSON from the Raspberry Pi over rosbridge.")
if st.button("Refresh rover status and data"):
    status, data, error = request_ros_snapshot(
        st.session_state.get("ros_url", DEFAULT_ROS_URL),
        st.session_state.get("ros_query_topic", "/mission/query"),
        st.session_state.get("ros_status_response_topic", "/mission/status_response"),
        st.session_state.get("ros_data_response_topic", "/mission/data_response"),
    )
    st.session_state.rover_snapshot_status = status
    st.session_state.rover_snapshot_data = data
    st.session_state.rover_snapshot_error = error
    st.session_state.rover_snapshot_attempted = True
if st.session_state.rover_snapshot_error:
    st.warning(f"Raspberry Pi snapshot unavailable: {st.session_state.rover_snapshot_error}")
if st.session_state.rover_snapshot_status is not None:
    st.json(st.session_state.rover_snapshot_status, expanded=False)
if st.session_state.rover_snapshot_data:
    st.download_button("Download mission sample JSON",
                       data=json.dumps(st.session_state.rover_snapshot_data, indent=2),
                       file_name=f"{st.session_state.rover_snapshot_data.get('mission_id', 'mission')}_samples.json",
                       mime="application/json")

# Spacing controls
st.subheader("1. Set soil-sampling spacing")
space1, space2, space3 = st.columns([1, 1, 1.3])
with space1:
    line_spacing = st.number_input("Between lines (metres)", min_value=1.0, max_value=100.0, value=5.0, step=0.5,
                                   help="Distance between parallel rover passes.")
with space2:
    point_spacing = st.number_input("Between sample points (metres)", min_value=1.0, max_value=100.0, value=2.0, step=0.5,
                                    help="Distance between sampling locations along each pass.")
with space3:
    st.info("Choose spacing, then tap the corners around the edge of your field. Use at least 3 corners.")

map_col, side_col = st.columns([1.7, 1], gap="large")
with map_col:
    def render_map():
        loc = gps_data
        st.subheader("2. Mark your field boundary")
        st.caption("Tap the map around the field edge in order. When all corners are marked, press “Finish boundary”.")
        fmap = folium.Map(location=st.session_state.map_center, zoom_start=17 if loc else 15, control_scale=True, tiles="OpenStreetMap")
        boundary = st.session_state.boundary
        boundary_locations = [[p["latitude"], p["longitude"]] for p in boundary]
        if len(boundary) >= 3:
            # Show a filled polygon as soon as three boundary points have been selected.
            # The polygon is only used for visual feedback until the user finishes the boundary.
            folium.Polygon(
                locations=boundary_locations,
                color="#13834a",
                weight=3,
                fill=True,
                fill_color="#13834a",
                fill_opacity=0.16,
                dash_array=None if st.session_state.boundary_closed else "6,5",
                tooltip="Field boundary",
            ).add_to(fmap)
        elif len(boundary) == 2:
            folium.PolyLine(boundary_locations, color="#13834a", weight=3, dash_array="6,5").add_to(fmap)
        for i, p in enumerate(boundary, 1):
            folium.CircleMarker([p["latitude"], p["longitude"]], radius=6, color="#ffffff", weight=2,
                                fill=True, fill_color="#13834a", fill_opacity=1,
                                tooltip=f"Boundary corner {i}").add_to(fmap)
        if loc:
            folium.CircleMarker([loc["latitude"], loc["longitude"]], radius=8, color="#0878e8", fill=True,
                                fill_color="#0878e8", fill_opacity=.9, tooltip="Rover one-time GPS fix").add_to(fmap)

        # Live route preview
        preview = []
        area = None
        if st.session_state.boundary_closed and len(boundary) >= 3:
            try:
                preview, route, area = generate_lawnmower(boundary, float(line_spacing), float(point_spacing))
                st.session_state.preview_points = preview
                st.session_state.preview_route = route
                st.session_state.preview_area = area
                st.session_state.preview_error = None
                if len(route) >= 2:
                    folium.PolyLine(route, color="#1677ff", weight=3, opacity=.95, tooltip="Planned rover route").add_to(fmap)
                for i, p in enumerate(preview, 1):
                    folium.CircleMarker([p["latitude"], p["longitude"]], radius=3, color="#ffffff", weight=1,
                                        fill=True, fill_color="#1677ff", fill_opacity=.95,
                                        tooltip=f"Sample point {i}").add_to(fmap)
            except Exception as e:
                st.session_state.preview_error = str(e)
                st.session_state.preview_points = []
                st.session_state.preview_route = []
                st.session_state.preview_area = None
        result = st_folium(fmap, height=600, use_container_width=True, returned_objects=["last_clicked"], key="boundary_map")
        click = result.get("last_clicked") if isinstance(result, dict) else None
        if click and valid_coord(click.get("lat"), click.get("lng")):
            key = (round(float(click["lat"]), 7), round(float(click["lng"]), 7))
            if key != st.session_state.last_map_click:
                if not st.session_state.boundary_closed:
                    st.session_state.boundary.append({"latitude": key[0], "longitude": key[1]})
                    st.session_state.last_map_click = key
                    st.session_state.last_saved_file = None
                    st.rerun()

    render_map()

with side_col:
    st.subheader(f"Boundary corners: {len(st.session_state.boundary)}")
    if not st.session_state.boundary:
        st.info("Tap the map to mark the first corner of your field.")
    elif not st.session_state.boundary_closed:
        st.write("Corners are added in the order you tap them.")
        if st.button("✅ Finish boundary", type="primary", use_container_width=True, disabled=len(st.session_state.boundary) < 3):
            st.session_state.boundary_closed = True
            st.session_state.last_map_click = None
            st.rerun()
        if st.button("↩️ Undo last corner", use_container_width=True):
            st.session_state.boundary.pop()
            st.session_state.last_map_click = None
            st.rerun()
        if st.button("🗑️ Clear boundary", use_container_width=True):
            st.session_state.boundary = []
            st.session_state.boundary_closed = False
            st.session_state.last_map_click = None
            st.session_state.preview_points = []
            st.session_state.preview_area = None
            st.rerun()
    else:
        st.success("Field boundary closed.")
        if st.session_state.preview_error:
            st.error(st.session_state.preview_error)
        elif st.session_state.preview_points:
            st.metric("Estimated sample points", len(st.session_state.preview_points))
            if st.session_state.preview_area is not None:
                st.metric("Approx. field area", f"{st.session_state.preview_area:.0f} m²")
            st.caption("Blue route and dots show the current preview. Change spacing above to update it.")
        if st.button("✏️ Redraw boundary", use_container_width=True):
            st.session_state.boundary = []
            st.session_state.boundary_closed = False
            st.session_state.last_map_click = None
            st.session_state.preview_points = []
            st.session_state.preview_route = []
            st.session_state.last_saved_file = None
            st.rerun()

    st.divider()
    st.subheader("3. Send mission")
    can_send = st.session_state.boundary_closed and len(st.session_state.preview_points) >= 2 and not st.session_state.preview_error
    st.warning("Check that the blue route stays inside your field and the area is clear before sending.")
    if st.button("🚜 Send mission to rover", type="primary", use_container_width=True, disabled=not can_send):
        mission_id = next_mission_id()
        points = st.session_state.preview_points
        mission = {
            "schema_version": 1,
            "mission_id": mission_id,
            "mission_name": f"field_mission_{mission_id}",
            "rover_id": st.session_state.get("rover_id", "ROVER_01"),
            "created_at": now_iso(),
            "coordinate_system": "WGS84",
            "coordinate_order": "latitude_longitude",
            "boundary": [
                {"latitude": round(p["latitude"], 7), "longitude": round(p["longitude"], 7)}
                for p in st.session_state.boundary
            ],
            "spacing": {"between_lines_m": float(line_spacing), "between_points_m": float(point_spacing)},
            "waypoint_count": len(points),
            "waypoints": [{"id": i, "latitude": p["latitude"], "longitude": p["longitude"]} for i, p in enumerate(points, 1)]
        }
        payload = json.dumps(mission, separators=(",", ":"))
        filename = f"mission_{mission_id}.json"
        save_path = MISSIONS_DIR / filename
        # Persist the generated mission before transmission so it remains available even if ROS is offline.
        save_path.write_text(json.dumps(mission, indent=2), encoding="utf-8")
        st.session_state.last_saved_file = str(save_path)
        try:
            ok, message = publish_ros(st.session_state.get("ros_url", DEFAULT_ROS_URL),
                                      st.session_state.get("ros_topic", "/mission/waypoints_json"), payload)
        except ImportError:
            ok, message = False, "WebSocket package missing. Install the requirements."
        st.session_state.last_send_result = (ok, message, filename, payload)
        if ok:
            st.success(f"Mission {mission_id} sent. Saved as {filename}.")
        else:
            st.error(f"Mission saved as {filename}, but it was not sent to ROS. {message}")
        st.download_button("⬇️ Download saved mission", data=save_path.read_text(encoding="utf-8"),
                           file_name=filename, mime="application/json", use_container_width=True, key=f"download_{mission_id}")

    if st.session_state.last_saved_file and Path(st.session_state.last_saved_file).exists():
        st.caption(f"Latest saved file: {Path(st.session_state.last_saved_file).name}")

    st.divider()
    st.subheader("4. Mission controls")
    st.caption("Start/resume requires a mission to be accepted by the Pi mission manager. Stop pauses navigation; an active stationary sampling cycle completes safely. This is not an emergency stop.")
    control_col1, control_col2 = st.columns(2)
    with control_col1:
        if st.button("▶️ Start / Resume", type="primary", use_container_width=True):
            try:
                ok, message = publish_ros_bool(
                    st.session_state.get("ros_url", DEFAULT_ROS_URL),
                    st.session_state.get("ros_control_topic", "/mission/start"),
                    True,
                )
            except ImportError:
                ok, message = False, "WebSocket package missing. Install the requirements."
            if ok:
                st.success(message)
            else:
                st.error(message)
    with control_col2:
        if st.button("⏸️ Stop / Pause", use_container_width=True):
            try:
                ok, message = publish_ros_bool(
                    st.session_state.get("ros_url", DEFAULT_ROS_URL),
                    st.session_state.get("ros_control_topic", "/mission/start"),
                    False,
                )
            except ImportError:
                ok, message = False, "WebSocket package missing. Install the requirements."
            if ok:
                st.warning(message)
            else:
                st.error(message)

st.divider()
st.caption("Mission files are saved in the app's missions/ folder. On cloud hosting, local files may be temporary and can be lost on redeploy/restart; use persistent storage for permanent records. ROS must validate the mission and acknowledge acceptance before movement.")
