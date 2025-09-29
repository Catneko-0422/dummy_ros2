#!/usr/bin/env python3
import math, json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import Trigger, SetBool
from dummy_robot_bridge.srv import MoveJ, MoveL, MoveLPartial, SetFloat64, SetTwoFloat64, SetUInt32
from tf2_ros import TransformBroadcaster

def rpy_to_quat(roll, pitch, yaw):
    import math
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*cp*cy
    return (qx,qy,qz,qw)

class DummyBridgeNode(Node):
    def __init__(self):
        super().__init__('dummy_bridge_node')
        # Params
        self.declare_parameter('serial_number','')
        self.declare_parameter('port','')
        self.declare_parameter('session_required', True)
        self.declare_parameter('degrees_mode', False)
        self.declare_parameter('joint_state_rate_hz', 30.0)
        self.declare_parameter('ee_state_rate_hz', 30.0)
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('frame_tool', 'tool0')

        self.serial = self.get_parameter('serial_number').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.session_required = self.get_parameter('session_required').get_parameter_value().bool_value
        self.degrees_mode = self.get_parameter('degrees_mode').get_parameter_value().bool_value
        self.frame_base = self.get_parameter('frame_base').get_parameter_value().string_value
        self.frame_tool = self.get_parameter('frame_tool').get_parameter_value().string_value

        ident = self.serial or (self.port.strip('/').replace('/','_') if self.port else 'UNKNOWN')
        self.ns = f"/dummy/{ident}"

        # pubs
        self.pub_connected = self.create_publisher(Bool, f"{self.ns}/state/connected", 10)
        self.pub_voltage   = self.create_publisher(Float64, f"{self.ns}/state/voltage", 10)
        self.pub_temp      = self.create_publisher(Float64, f"{self.ns}/state/temperature", 10)
        self.pub_js        = self.create_publisher(JointState, f"{self.ns}/joint_states", 10)
        self.pub_pose      = self.create_publisher(PoseStamped, f"{self.ns}/ee_pose", 10)
        self.pub_rpy       = self.create_publisher(Float64MultiArray, f"{self.ns}/ee_rpy", 10)
        self.pub_echo      = self.create_publisher(String, f"{self.ns}/echo", 50)

        # subs (commands)
        self.sub_cli = self.create_subscription(String, f"{self.ns}/cli", self.on_cli, 50)
        self.sub_joint = self.create_subscription(Float64MultiArray, f"{self.ns}/joint_targets", self.on_joint_targets, 10)
        self.sub_cart  = self.create_subscription(Float64MultiArray, f"{self.ns}/cartesian_target", self.on_cartesian_target, 10)

        # global joint_states passthrough
        self.sub_global_js = self.create_subscription(JointState, "/joint_states", self.on_global_joint_states, 10)

        # services
        self.create_service(Trigger, f"{self.ns}/reboot", self.srv_reboot)
        self.create_service(SetBool, f"{self.ns}/set_enable", self.srv_set_enable)
        self.create_service(SetBool, f"{self.ns}/set_rgb_enable", self.srv_set_rgb_enable)
        self.create_service(SetUInt32, f"{self.ns}/set_rgb_mode", self.srv_set_rgb_mode)
        self.create_service(SetFloat64, f"{self.ns}/set_joint_speed", self.srv_set_joint_speed)
        self.create_service(SetFloat64, f"{self.ns}/set_joint_acc", self.srv_set_joint_acc)
        self.create_service(SetUInt32, f"{self.ns}/set_command_mode", self.srv_set_command_mode)
        self.create_service(MoveJ, f"{self.ns}/move_j", self.srv_move_j)
        self.create_service(MoveL, f"{self.ns}/move_l", self.srv_move_l)
        self.create_service(MoveLPartial, f"{self.ns}/move_l_partial", self.srv_move_l_partial)
        self.create_service(Trigger, f"{self.ns}/robot/homing", self.srv_homing)
        self.create_service(Trigger, f"{self.ns}/robot/resting", self.srv_resting)
        self.create_service(Trigger, f"{self.ns}/robot/calibrate_home_offset", self.srv_calib_home)

        # fibre RPC channels (topic gateway provided by discover_node)
        self.req_pub = self.create_publisher(String, "fibre/request", 10)
        self._resp_buffer = {}
        self.sub_resp = self.create_subscription(String, "fibre/response", self._on_rpc_resp, 50)

        # TF
        self.br = TransformBroadcaster(self)

        # connection state
        self._set_connected(False)
        if self.session_required:
            self._open_session()

        # periodic EE state
        hz = float(self.get_parameter('ee_state_rate_hz').value)
        if hz > 0.0:
            self.create_timer(max(0.01, 1.0/hz), self._tick_ee)

    # --------- Utils ---------
    def _set_connected(self, val: bool):
        m = Bool(); m.data = bool(val)
        self.pub_connected.publish(m)

    def _rpc(self, op: str, path: str, args=None, timeout=1.0):
        req_id = int(time.time()*1000) & 0x7fffffff
        msg = {"id": req_id, "op": op, "path": path, "args": (args or [])}
        s = String(); s.data = json.dumps(msg, ensure_ascii=False)
        self.req_pub.publish(s)
        # wait for response
        t_end = self.get_clock().now().nanoseconds + int(timeout*1e9)
        while self.get_clock().now().nanoseconds < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if req_id in self._resp_buffer:
                res = self._resp_buffer.pop(req_id)
                return res
        return None

    def _on_rpc_resp(self, m: String):
        try:
            data = json.loads(m.data)
            req_id = data.get("id")
            if req_id is not None:
                self._resp_buffer[req_id] = data
        except Exception:
            pass

    def _call(self, path: str, *args, timeout=1.0):
        res = self._rpc("call", path, list(args), timeout=timeout)
        return (res or {})

    def _read(self, path: str, timeout=1.0):
        res = self._rpc("read", path, [], timeout=timeout)
        return (res or {})

    def _open_session(self):
        res = self._read("serial_number", timeout=1.0)
        ok = bool(res and res.get("ok"))
        self._set_connected(ok)
        if ok:
            try:
                serial = str(int(res.get("result")))
                self.ns = f"/dummy/{serial}"
                self.get_logger().info(f"Session active. Using namespace: {self.ns}")
            except Exception:
                pass

    # ---------- Publishers ----------
    def on_global_joint_states(self, msg: JointState):
        js = JointState()
        js.header = msg.header
        js.name = list(msg.name)
        js.position = [ (math.radians(p) if self.degrees_mode else p) for p in msg.position ]
        self.pub_js.publish(js)

    def _tick_ee(self):
        res = self._call("get_cartesian_position", timeout=0.2)
        if not (res and res.get("ok") and isinstance(res.get("result"), list) and len(res["result"])==6):
            return
        x,y,z, r,p,yaw = res["result"]
        if self.degrees_mode:
            r = math.radians(r); p = math.radians(p); yaw = math.radians(yaw)
        qx,qy,qz,qw = rpy_to_quat(r,p,yaw)
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_base
        ps.pose.position.x = float(x); ps.pose.position.y = float(y); ps.pose.position.z = float(z)
        ps.pose.orientation.x = qx; ps.pose.orientation.y = qy; ps.pose.orientation.z = qz; ps.pose.orientation.w = qw
        self.pub_pose.publish(ps)
        arr = Float64MultiArray(); arr.data = [float(x),float(y),float(z),float(r),float(p),float(yaw)]
        self.pub_rpy.publish(arr)
        tf = TransformStamped()
        tf.header = ps.header
        tf.child_frame_id = self.frame_tool
        tf.transform.translation.x = ps.pose.position.x
        tf.transform.translation.y = ps.pose.position.y
        tf.transform.translation.z = ps.pose.position.z
        tf.transform.rotation = ps.pose.orientation
        self.br.sendTransform(tf)

    # ---------- CLI / Topic Commands ----------
    def on_cli(self, msg: String):
        cmd = msg.data.strip()
        if not cmd: return
        if cmd.upper() == "!START":
            self.srv_set_enable(type('R',(),{'data':True}))
            return
        if cmd.upper() in ("!STOP","!DISABLE"):
            self.srv_set_enable(type('R',(),{'data':False}))
            return
        if cmd.upper() == "!HOME":
            self.srv_homing(None)
            return
        if cmd.upper() in ("!RESET","!REBOOT"):
            self.srv_reboot(None)
            return
        if cmd.upper() == "!CALIBRATION":
            self.srv_calib_home(None)
            return
        if cmd.upper().startswith("#GETJPOS"):
            res = self._read("robot.joint_all.angle", timeout=0.5)
            if not (res and res.get("ok")):
                angles = []
                for i in range(1,7):
                    r = self._read(f"robot.joint_{i}.angle", timeout=0.2)
                    if r and r.get("ok"): angles.append(float(r["result"]))
                out = String(); out.data = json.dumps({"JA": angles})
                self.pub_echo.publish(out); return
            out = String(); out.data = json.dumps({"JA": res["result"]})
            self.pub_echo.publish(out); return
        if cmd.startswith("&"):
            parts = [p for p in cmd[1:].split(",") if p!='']
            if len(parts)==6:
                vals = [float(x) for x in parts]
                if self.degrees_mode: vals = [math.radians(x) for x in vals]
                self._call("move_j", *vals, timeout=2.0); return
        if cmd.startswith("@"):
            parts = [p for p in cmd[1:].split(",") if p!='']
            if len(parts)==6:
                x,y,z,a,b,c = [float(x) for x in parts]
                if self.degrees_mode:
                    a=math.radians(a); b=math.radians(b); c=math.radians(c)
                self._call("move_l", x,y,z,a,b,c, timeout=2.0); return
        m = String(); m.data = f"UNHANDLED: {cmd}"
        self.pub_echo.publish(m)

    def on_joint_targets(self, arr: Float64MultiArray):
        vals = list(arr.data)
        if len(vals)!=6: return
        if self.degrees_mode: vals = [math.radians(x) for x in vals]
        self._call("move_j", *vals, timeout=2.0)

    def on_cartesian_target(self, arr: Float64MultiArray):
        v = list(arr.data)
        if len(v)!=6: return
        x,y,z,r,p,yaw = v
        if self.degrees_mode:
            r = math.radians(r); p = math.radians(p); yaw = math.radians(yaw)
        self._call("move_l", x,y,z,r,p,yaw, timeout=2.0)

    # ---------- Service handlers ----------
    def srv_reboot(self, req, res=None):
        ok = self._call("reboot", timeout=1.0).get("ok", False)
        if res is None: return
        res.success = bool(ok); res.message = ""
        return res

    def srv_set_enable(self, req, res=None):
        ok = self._call("set_enable", bool(req.data), timeout=1.0).get("ok", False)
        if res is None: return
        res.success = bool(ok); res.message = ""
        return res

    def srv_set_rgb_enable(self, req, res):
        ok = self._call("set_rgb_enable", bool(req.data), timeout=1.0).get("ok", False)
        res.success = bool(ok); res.message = ""
        return res

    def srv_set_rgb_mode(self, req, res):
        ok = self._call("set_rgb_mode", int(req.data), timeout=1.0).get("ok", False)
        res.success = bool(ok); res.message = ""
        return res

    def srv_set_joint_speed(self, req, res):
        ok = self._call("set_joint_speed", float(req.data), timeout=1.0).get("ok", False)
        res.success = bool(ok); res.message = ""
        return res

    def srv_set_joint_acc(self, req, res):
        ok = self._call("set_joint_acc", float(req.data), timeout=1.0).get("ok", False)
        res.success = bool(ok); res.message = ""
        return res

    def srv_set_command_mode(self, req, res):
        ok = self._call("set_command_mode", int(req.data), timeout=1.0).get("ok", False)
        res.success = bool(ok); res.message = ""
        return res

    def srv_move_j(self, req, res):
        vals = [float(x) for x in req.joints]
        if self.degrees_mode: vals = [math.degrees(x) for x in vals]
        r = self._call("move_j", *vals, timeout=2.0)
        res.success = bool(r.get("ok", False)); res.message = ""
        return res

    def srv_move_l(self, req, res):
        x,y,z,rp,pp,yp = [float(x) for x in req.pose]
        if self.degrees_mode: rp=math.degrees(rp); pp=math.degrees(pp); yp=math.degrees(yp)
        r = self._call("move_l", x,y,z,rp,pp,yp, timeout=2.0)
        res.success = bool(r.get("ok", False)); res.message = ""
        return res

    def srv_move_l_partial(self, req, res):
        target = list(req.pose); mask = list(req.mask)
        cur = self._call("get_cartesian_position", timeout=0.5)
        curv = cur.get("result", [0,0,0,0,0,0]) if cur and cur.get("ok") else [0,0,0,0,0,0]
        for i in range(6):
            if not mask[i]:
                target[i] = curv[i]
        x,y,z,rp,pp,yp = target
        if self.degrees_mode: rp=math.degrees(rp); pp=math.degrees(pp); yp=math.degrees(yp)
        r = self._call("move_l", x,y,z,rp,pp,yp, timeout=2.0)
        res.success = bool(r.get("ok", False)); res.message = "mask applied"
        return res

    def srv_homing(self, req, res):
        r = self._call("robot.homing", timeout=1.0)
        res.success = bool(r.get("ok", False)); res.message = ""
        return res

    def srv_resting(self, req, res):
        r = self._call("robot.resting", timeout=1.0)
        res.success = bool(r.get("ok", False)); res.message = ""
        return res

    def srv_calib_home(self, req, res):
        r = self._call("robot.calibrate_home_offset", timeout=1.0)
        res.success = bool(r.get("ok", False)); res.message = ""
        return res

def main():
    rclpy.init()
    node = DummyBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
