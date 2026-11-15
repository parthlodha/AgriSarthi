"""Farmer-friendly field-boundary mission planner."""
from __future__ import annotations

import json
import math
import re
from datetime import datetime, timezone
from pathlib import Path

import folium
import streamlit as st
from streamlit_folium import st_folium
from shapely.geometry import LineString, Polygon, MultiLineString, GeometryCollection

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
    if not poly.is_valid:
        poly = poly.buffer(0)
    if poly.is_empty or not poly.is_valid or poly.area < 1:
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
        command = "START" if value else "STOP/PAUSE"
        return True, f"{command} command sent to {topic}. This confirms transmission only, not rover state."
    except Exception as e:
        return False, f"Could not reach ROS bridge: {type(e).__name__}: {e}"
    finally:
        if ws:
            try: ws.close()
            except Exception: pass

# State
for key, default in {
    "boundary": [], "device_location": None, "map_center": [18.5204, 73.8567],
    "last_map_click": None, "boundary_closed": False, "preview_points": [],
    "preview_route": [], "preview_error": None, "last_saved_file": None
}.items():
    if key not in st.session_state:
        st.session_state[key] = default

with st.expander("⚙️ Project settings (administrator only)"):
    st.text_input("Rover ID", value="ROVER_01", key="rover_id")
    st.text_input("ROS bridge WebSocket URL", value="ws://localhost:9090", key="ros_url",
                  help="On cloud hosting, localhost is the cloud server—not the rover.")
    st.text_input("ROS mission topic", value="/mission/waypoints_json", key="ros_topic")
    st.text_input("ROS mission start/stop topic", value="/mission/start", key="ros_control_topic")
    st.caption("Mission JSON uses std_msgs/msg/String. Start/stop uses std_msgs/msg/Bool on the control topic: true starts/resumes; false pauses the mission. Use a secured connection; do not expose rosbridge publicly without access controls.")

# Location
loc_col1, loc_col2 = st.columns([1, 2])
with loc_col1:
    if st.button("📍 Find my location", use_container_width=True):
        st.session_state.request_location = True
    try:
        from streamlit_geolocation import streamlit_geolocation
        if st.session_state.get("request_location", False):
            geo = streamlit_geolocation()
            if isinstance(geo, dict) and valid_coord(geo.get("latitude"), geo.get("longitude")):
                st.session_state.device_location = {
                    "latitude": float(geo["latitude"]), "longitude": float(geo["longitude"]),
                    "accuracy_m": float(geo.get("accuracy", 0) or 0)
                }
                st.session_state.map_center = [float(geo["latitude"]), float(geo["longitude"])]
                st.session_state.request_location = False
                st.success("Location found. The map is centered on your location.")
            elif isinstance(geo, dict) and geo.get("error"):
                st.warning("Location unavailable. Check browser permission and try again.")
    except ImportError:
        st.warning("Location support is not installed. Run pip install -r requirements.txt.")
with loc_col2:
    st.markdown("<p class='help'>Allow location access when your browser asks. Location requires HTTPS (or localhost) and may be approximate on desktop computers.</p>", unsafe_allow_html=True)

loc = st.session_state.device_location
if loc:
    st.caption(f"Map location: {loc['latitude']:.6f}, {loc['longitude']:.6f} · reported accuracy ±{loc['accuracy_m']:.0f} m. This is the device location, not necessarily the rover.")

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
                            fill_color="#0878e8", fill_opacity=.9, tooltip="Your device location").add_to(fmap)

    # Live route preview
    preview = []
    area = None
    if st.session_state.boundary_closed and len(boundary) >= 3:
        try:
            preview, route, area = generate_lawnmower(boundary, float(line_spacing), float(point_spacing))
            st.session_state.preview_points = preview
            st.session_state.preview_route = route
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
            st.rerun()
    else:
        st.success("Field boundary closed.")
        if st.session_state.preview_error:
            st.error(st.session_state.preview_error)
        elif st.session_state.preview_points:
            st.metric("Estimated sample points", len(st.session_state.preview_points))
            if area is not None:
                st.metric("Approx. field area", f"{area:.0f} m²")
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
            ok, message = publish_ros(st.session_state.get("ros_url", "ws://localhost:9090"),
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
    st.caption("Start/resume requires a mission to have been sent and accepted by the ROS navigator. Stop sends false to the same control topic; the current navigator implementation treats this as pause, not an emergency stop.")
    control_col1, control_col2 = st.columns(2)
    with control_col1:
        if st.button("▶️ Start / Resume", type="primary", use_container_width=True):
            try:
                ok, message = publish_ros_bool(
                    st.session_state.get("ros_url", "ws://localhost:9090"),
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
                    st.session_state.get("ros_url", "ws://localhost:9090"),
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
