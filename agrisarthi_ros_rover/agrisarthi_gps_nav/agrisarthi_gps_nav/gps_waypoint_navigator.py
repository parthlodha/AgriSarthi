#!/usr/bin/env python3
import json, math, socket, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, String

def distance_m(a,b,c,d):
    r=6371000.0; p1,p2=math.radians(a),math.radians(c); dp=math.radians(c-a); dl=math.radians(d-b)
    x=math.sin(dp/2)**2+math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*r*math.atan2(math.sqrt(x),math.sqrt(max(0.0,1-x)))
def bearing_deg(a,b,c,d):
    p1,p2=math.radians(a),math.radians(c); dl=math.radians(d-b)
    return (math.degrees(math.atan2(math.sin(dl)*math.cos(p2), math.cos(p1)*math.sin(p2)-math.sin(p1)*math.cos(p2)*math.cos(dl)))+360)%360
def angle_error(target,current): return (target-current+180)%360-180

class GPSWaypointNavigator(Node):
    def __init__(self):
        super().__init__('gps_waypoint_navigator')
        defaults={'mission_topic':'/mission/waypoints_json','mission_file':'~/agrisarthi_ws/missions/current_mission.json','start_topic':'/mission/start','gps_topic':'/gps/fix','heading_topic':'/heading_deg','estop_topic':'/cmd/estop','status_topic':'/rover/nav_status','esp32_ip':'192.168.1.50','esp32_port':5005,'command_rate_hz':10.0,'gps_timeout_s':2.0,'heading_timeout_s':1.0,'waypoint_tolerance_m':1.5,'max_linear_speed':0.18,'max_angular_speed':0.35,'heading_kp':0.012,'turn_in_place_error_deg':55.0,'max_heading_error_for_full_speed_deg':35.0,'minimum_speed_scale':0.2,'require_valid_gps_status':True}
        for k,v in defaults.items(): self.declare_parameter(k,v)
        p=lambda k:self.get_parameter(k).value
        self.esp=(p('esp32_ip'),int(p('esp32_port'))); self.rate=max(1.0,float(p('command_rate_hz')))
        self.gps_timeout=float(p('gps_timeout_s')); self.heading_timeout=float(p('heading_timeout_s')); self.tol=float(p('waypoint_tolerance_m'))
        self.max_linear=abs(float(p('max_linear_speed'))); self.max_angular=abs(float(p('max_angular_speed'))); self.kp=float(p('heading_kp'))
        self.turn_threshold=abs(float(p('turn_in_place_error_deg'))); self.full_speed_error=max(1.0,abs(float(p('max_heading_error_for_full_speed_deg')))); self.min_scale=max(0.0,min(1.0,float(p('minimum_speed_scale')))); self.require_valid=bool(p('require_valid_gps_status'))
        self.mission_file=str(p('mission_file')); self.started=False
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); self.seq=0; self.mission_id=''; self.points=[]; self.index=0
        self.gps=None; self.gps_t=0.0; self.heading=None; self.heading_t=0.0; self.estop=False; self.state='IDLE'; self.last_status=0.0
        self.create_subscription(String,p('mission_topic'),self.on_mission,10); self.create_subscription(Bool,p('start_topic'),self.on_start,10); self.create_subscription(NavSatFix,p('gps_topic'),self.on_gps,10)
        self.create_subscription(Float32,p('heading_topic'),self.on_heading,10); self.create_subscription(Bool,p('estop_topic'),self.on_estop,10)
        self.status_pub=self.create_publisher(String,p('status_topic'),10); self.timer=self.create_timer(1/self.rate,self.tick)
        self.get_logger().info(f'Ready. UDP motor endpoint {self.esp[0]}:{self.esp[1]}; mission file={self.mission_file}')
        self.load_saved_mission()
    def on_mission(self,msg):
        try:
            packet=json.loads(msg.data); raw=packet.get('waypoints')
            if not isinstance(raw,list) or not raw: raise ValueError('waypoints must be a non-empty array')
            pts=[]
            for i,w in enumerate(raw):
                lat=float(w['latitude']); lon=float(w['longitude'])
                if not math.isfinite(lat) or not math.isfinite(lon) or not -90<=lat<=90 or not -180<=lon<=180: raise ValueError(f'invalid coordinates at waypoint {i+1}')
                pts.append({'id':w.get('id',i+1),'latitude':lat,'longitude':lon})
            # Persist the full packet locally before acknowledging it. The rover does not
            # need Streamlit, internet, or rosbridge after this point.
            from pathlib import Path
            mission_path = Path(self.mission_file).expanduser()
            mission_path.parent.mkdir(parents=True, exist_ok=True)
            temp_path = mission_path.with_suffix(mission_path.suffix + '.tmp')
            temp_path.write_text(json.dumps(packet, indent=2), encoding='utf-8')
            temp_path.replace(mission_path)
            self.send_cmd(0,0,False); self.points=pts; self.index=0; self.mission_id=str(packet.get('mission_id','UNNAMED'))
            self.started=False; self.state='MISSION_READY'
            self.get_logger().info(f'Persisted mission {self.mission_id}: {len(pts)} waypoints. Publish true on /mission/start to begin.')
            self.status('MISSION_READY',f'Saved to {self.mission_file}; waiting for explicit start command')
        except (ValueError,TypeError,KeyError,json.JSONDecodeError) as e:
            self.get_logger().error(f'Rejected mission JSON: {e}'); self.send_cmd(0,0,False); self.status('MISSION_REJECTED',str(e))
    def load_saved_mission(self):
        from pathlib import Path
        path = Path(self.mission_file).expanduser()
        if not path.is_file():
            self.get_logger().info('No saved mission file found; waiting for a mission from the planner.')
            return
        try:
            packet = json.loads(path.read_text(encoding='utf-8'))
            # Reuse the validator and persistence path; this remains READY, never auto-starts.
            self.on_mission(String(data=json.dumps(packet)))
            self.started = False
            if self.points:
                self.state = 'MISSION_READY'
                self.status('MISSION_READY', f'Loaded local mission file {path}; explicit start required')
                self.get_logger().info(f'Loaded saved mission {self.mission_id}; NOT started automatically.')
        except (OSError, json.JSONDecodeError) as e:
            self.get_logger().error(f'Could not load saved mission {path}: {e}')

    def on_start(self, msg):
        if not msg.data:
            self.started = False
            self.state = 'PAUSED'
            self.send_cmd(0, 0, False)
            self.status('PAUSED', 'Mission paused; publish true to resume from current waypoint.')
            self.get_logger().warning('Mission paused; motor stop command sent.')
            return
        if self.estop:
            self.send_cmd(0, 0, False)
            self.status('ESTOP', 'Cannot start while E-stop is active.')
            return
        if not self.points:
            self.send_cmd(0, 0, False)
            self.status('NO_MISSION', 'No valid mission is loaded.')
            return
        if self.index >= len(self.points):
            self.status('MISSION_COMPLETE', 'Mission is already complete; send a new mission to run again.')
            return
        self.started = True
        self.state = 'WAITING_FOR_SENSORS'
        self.get_logger().info(f'Mission {self.mission_id} start requested.')
        self.status('START_REQUESTED', 'Waiting for fresh GPS and heading before moving.')

    def on_gps(self,msg):
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude) or not -90<=msg.latitude<=90 or not -180<=msg.longitude<=180: self.gps=None; return
        if self.require_valid and msg.status.status<0: self.gps=None; return
        self.gps=(float(msg.latitude),float(msg.longitude)); self.gps_t=time.monotonic()
    def on_heading(self,msg):
        if not math.isfinite(float(msg.data)): self.heading=None; return
        self.heading=float(msg.data)%360; self.heading_t=time.monotonic()
    def on_estop(self,msg):
        self.estop=bool(msg.data)
        if self.estop: self.state='ESTOP'; self.send_cmd(0,0,False); self.status('ESTOP','Software E-stop active')
    def send_cmd(self,left,right,enable,linear=0.0,angular=0.0):
        self.seq+=1; obj={'type':'motor_cmd','mission_id':self.mission_id,'seq':self.seq,'enable':bool(enable),'left':round(float(left),4),'right':round(float(right),4),'linear':round(float(linear),4),'angular':round(float(angular),4)}
        try: self.sock.sendto((json.dumps(obj,separators=(',',':'))+'\n').encode(),self.esp)
        except OSError as e: self.get_logger().warning(f'UDP send failed: {e}')
    def status(self,state,detail):
        m=String(); m.data=json.dumps({'state':state,'mission_id':self.mission_id,'waypoint_index':self.index,'waypoint_count':len(self.points),'detail':detail}); self.status_pub.publish(m)
    def status_throttled(self,state,detail):
        now=time.monotonic()
        if now-self.last_status>=1: self.status(state,detail); self.last_status=now
    def tick(self):
        if self.estop: self.state='ESTOP'; self.send_cmd(0,0,False); self.status_throttled(self.state,'E-stop active'); return
        if not self.points: self.state='IDLE'; self.send_cmd(0,0,False); return
        if not self.started: self.send_cmd(0,0,False); self.status_throttled(self.state if self.state in ('MISSION_READY','PAUSED','MISSION_COMPLETE') else 'MISSION_READY','Mission is saved but not started'); return
        now=time.monotonic(); gps_ok=self.gps is not None and now-self.gps_t<=self.gps_timeout; head_ok=self.heading is not None and now-self.heading_t<=self.heading_timeout
        if not gps_ok or not head_ok:
            self.state='WAITING_FOR_SENSORS'; self.send_cmd(0,0,False); self.status_throttled(self.state,'GPS stale/invalid' if not gps_ok else 'Heading stale/invalid'); return
        if self.index>=len(self.points): self.state='MISSION_COMPLETE'; self.started=False; self.send_cmd(0,0,False); self.status_throttled(self.state,'All waypoints reached'); return
        lat,lon=self.gps; target=self.points[self.index]; dist=distance_m(lat,lon,target['latitude'],target['longitude'])
        if dist<=self.tol:
            self.get_logger().info(f"Reached waypoint {target['id']} ({self.index+1}/{len(self.points)}), {dist:.2f} m")
            self.index+=1
            if self.index>=len(self.points): self.state='MISSION_COMPLETE'; self.started=False; self.send_cmd(0,0,False); self.status(self.state,'All waypoints reached'); return
            target=self.points[self.index]; dist=distance_m(lat,lon,target['latitude'],target['longitude'])
        bearing=bearing_deg(lat,lon,target['latitude'],target['longitude']); err=angle_error(bearing,self.heading)
        angular=max(-self.max_angular,min(self.max_angular,self.kp*err))
        if abs(err)>=self.turn_threshold: linear=0.0
        else: linear=self.max_linear*max(self.min_scale,1-abs(err)/self.full_speed_error)
        left=linear-angular; right=linear+angular; peak=max(1.0,abs(left),abs(right)); left/=peak; right/=peak
        self.state='NAVIGATING'; self.send_cmd(left,right,True,linear,angular)
        self.status_throttled(self.state,f'wp={self.index+1}/{len(self.points)} distance={dist:.2f}m bearing={bearing:.1f} heading_error={err:.1f}')
    def destroy_node(self):
        for _ in range(3): self.send_cmd(0,0,False); time.sleep(.02)
        self.sock.close(); super().destroy_node()
def main(args=None):
    rclpy.init(args=args); node=GPSWaypointNavigator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
