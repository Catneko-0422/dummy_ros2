"""
FibreDiscoverNode（ROS 2 節點，增強版：/joint_states + 併發保護）
==============================================================
目的：
  在原本 discover_node 的基礎上新增：
  1) 以 **/joint_states**（sensor_msgs/JointState）發佈整批關節狀態。
  2) 可選的「每關節一個 topic」發布（/robot/<joint_name>/angle，std_msgs/Float64）。
  3) 基本的 **I/O 併發保護（Lock）**，避免多處同時對 Fibre 送 RPC。
  4) 可調參數：發布頻率、是否啟用 per-joint topics、QoS。

你可以直接把本檔覆蓋原檔使用；若要最小改動，搜尋標記：
  # === NEW ===  /  # === CHANGED ===

Topic / Service 介面：
  - 發佈：'fibre/devices'（String, JSON）— 裝置發現事件
  - 訂閱：'fibre/request'（String, JSON）— JSON RPC 閘道（read/write/call）
  - 發佈：'fibre/response'（String, JSON）— JSON RPC 回應
  - 發佈：'/joint_states'（JointState）— 標準關節狀態
  - （可選）發佈：'/robot/<joint_name>/angle'（Float64）— 單關節角度

請求/回應 JSON 格式請見先前版本檔頭；此版僅新增狀態發布與鎖。
"""

import json
import threading
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
# === NEW === QoS & JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

import fibre
from fibre.utils import Event, Logger
from fibre.protocol import ChannelBrokenException, ChannelDamagedException


class FibreDiscoverNode(Node):
    """ROS 2 節點：包裝 fibre 裝置發現 + JSON RPC + JointState 發布。

    參數（ROS 參數伺服器）：
      - path: 'usb,serial'（也可 'tcp:ip:port'）
      - serial_number: 指定序號
      - verbose: 顯示 fibre 內部 log
      - single_shot: True → 第一台即停並建立 session
      - hold_session: True → 持續保持通道
      - extra_vid_pid: 允許的 USB VID:PID 清單（十六進位字串）
      - serial_baudrate: 覆寫 serial 鮑率（若可用）

      # === NEW ===
      - publish_rate_hz: JointState 發布頻率（預設 50.0）
      - enable_per_joint_topics: 是否同時發布每關節 topic（預設 False）
      - joint_qos_reliability: 'best_effort' | 'reliable'（預設 'best_effort'）
    """

    def __init__(self):
        super().__init__('fibre_discover_node')

        # === 讀取 ROS 參數 ===
        self.path = self.declare_parameter('path', 'usb,serial').get_parameter_value().string_value
        serial_param = self.declare_parameter('serial_number', '').get_parameter_value().string_value
        self.serial_number: Optional[str] = serial_param or None
        self.verbose = self.declare_parameter('verbose', False).get_parameter_value().bool_value

        self.single_shot = self.declare_parameter('single_shot', True).get_parameter_value().bool_value
        self.hold_session = self.declare_parameter('hold_session', True).get_parameter_value().bool_value

        # === NEW === JointState 相關參數
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 50.0).value)
        self.enable_per_joint_topics = bool(self.declare_parameter('enable_per_joint_topics', False).value)
        self.joint_qos_reliability = str(self.declare_parameter('joint_qos_reliability', 'best_effort').value).lower()

        # 額外允許的 USB VID/PID（若裝置沒在內建清單，這裡補進去）
        self.extra_vid_pid = self.declare_parameter('extra_vid_pid', ['1209:0d32']).get_parameter_value().string_array_value
        try:
            from fibre import usbbulk_transport as ub
            for pair in self.extra_vid_pid:
                try:
                    vid_str, pid_str = pair.split(':')
                    vid = int(vid_str, 16); pid = int(pid_str, 16)
                    if (vid, pid) not in ub.WELL_KNOWN_VID_PID_PAIRS:
                        ub.WELL_KNOWN_VID_PID_PAIRS.append((vid, pid))
                except Exception as ex:
                    self.get_logger().warn(f'Bad VID:PID "{pair}": {ex}')
        except Exception as ex:
            self.get_logger().warn(f'Cannot patch VID/PID list: {ex}')

        # 覆寫 Serial 預設鮑率（如果 serial 傳輸層可用）
        try:
            import fibre.serial_transport as st
            st.DEFAULT_BAUDRATE = self.declare_parameter('serial_baudrate', 115200).get_parameter_value().integer_value
        except Exception:
            pass

        # === 建立 ROS Topic/TX/RX ===
        self.pub_devices = self.create_publisher(String, 'fibre/devices', 10)
        self.pub_resp    = self.create_publisher(String, 'fibre/response', 10)
        self.sub_req     = self.create_subscription(String, 'fibre/request', self.handle_request, 10)

        # === NEW === JointState Publisher 與 QoS
        qos = QoSProfile(
            reliability=(ReliabilityPolicy.BEST_EFFORT if self.joint_qos_reliability == 'best_effort' else ReliabilityPolicy.RELIABLE),
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', qos)
        # per-joint publisher 會動態建立並快取在此 dict
        self.per_joint_pubs: Dict[str, Any] = {}

        # === 狀態物件與 I/O 鎖 ===
        self.logger_fibre = Logger(verbose=self.verbose)
        self.search_stop = Event()
        self.channel_termination = Event() if not self.hold_session else None
        self.devices: Dict[str, Dict] = {}
        self.session_obj = None
        # === NEW ===：所有對 fibre 的 I/O 走同一把鎖，避免通道併發
        self.io_lock = threading.Lock()

        self.get_logger().info(
            f"Fibre discovery started. path={self.path}, serial={self.serial_number or '*'}, Hz={self.publish_rate_hz}, per_joint={self.enable_per_joint_topics}"
        )

        # 掃描裝置的背景執行緒
        self.thread = threading.Thread(target=self.discovery_loop, daemon=True)
        self.thread.start()

        # === NEW === 啟動 JointState 定時發布
        if self.publish_rate_hz > 0:
            period = max(0.001, 1.0 / self.publish_rate_hz)
            self.timer = self.create_timer(period, self.publish_joint_states)
        else:
            self.timer = None

    # -------- 掃描主迴圈：找到就（視參數）停止，並保存 session --------
    def discovery_loop(self):
        def did_discover(obj=None, *args):
            info: Dict[str, Any] = {"path": self.path}
            ch = getattr(obj, "__channel__", None)
            tr = getattr(ch, "transport", None) or getattr(ch, "_transport", None)

            if tr is not None:
                info["transport"] = tr.__class__.__name__
                dev = getattr(tr, "dev", None) or getattr(tr, "device", None)
                try:
                    if dev and getattr(dev, "serial_number", None):
                        info["serial"] = str(dev.serial_number)
                    if dev and hasattr(dev, "idVendor"):
                        info["vid"] = f"{int(dev.idVendor):04x}"
                    if dev and hasattr(dev, "idProduct"):
                        info["pid"] = f"{int(dev.idProduct):04x}"
                except Exception:
                    pass
            else:
                info["transport"] = "Unknown"

            key = info.get("serial") or f'{info.get("transport")}@{self.path}'
            self.devices[key] = info
            self.publish_device('add', info)

            if self.single_shot:
                self.session_obj = obj
                self.search_stop.set()
                self.get_logger().info("Session established and discovery stopped.")

        try:
            fibre.discovery.find_all(
                path=self.path,
                serial_number=self.serial_number,
                did_discover_object_callback=did_discover,
                search_cancellation_token=self.search_stop,
                channel_termination_token=self.channel_termination,
                logger=self.logger_fibre
            )
        except Exception as e:
            self.get_logger().error(f"Discovery loop error: {e}")

    def publish_device(self, action: str, info: Dict):
        msg = String()
        data = dict(info)
        data['action'] = action
        msg.data = json.dumps(data, ensure_ascii=False)
        self.pub_devices.publish(msg)

    # -------- 將 'a.b.c' 這種路徑解析成 RemoteObject 的屬性/方法 --------
    def resolve_path(self, root: Any, path: str):
        node = root
        for part in path.split('.'):
            if not part:
                continue
            node = getattr(node, part)
        return node

    # === NEW ===：安全取得遠端值（自動處理 callable / get_value）
    def _read_value(self, target):
        """盡力把遠端物件轉成值：
        - 若 target 可呼叫（函式），呼叫並回傳結果
        - 若有 get_value 方法，呼叫 get_value()
        - 否則直接回傳（可能是 proxy，視 fibre 版本而定）
        """
        try:
            if callable(target):
                return target()
            getv = getattr(target, 'get_value', None)
            if callable(getv):
                return getv()
        except Exception as e:
            raise e
        return target

    # === NEW === JointState 發布邏輯（50Hz 預設）
    def publish_joint_states(self):
        if self.session_obj is None:
            return
        try:
            with self.io_lock:
                names = None
                pos = None
                # 嘗試取整批 joint_all
                try:
                    names = list(self._read_value(self.session_obj.robot.joint_all.names))
                except Exception:
                    pass
                try:
                    pos = list(self._read_value(self.session_obj.robot.joint_all.angle))
                except Exception:
                    pass

                # 若沒有 joint_all，就逐關節嘗試（joint_1..joint_6 等）
                if names is None or pos is None:
                    names = []
                    pos = []
                    for i in range(1, 12+1):  # 最多試 12 軸，依需求調整
                        try:
                            jnode = getattr(self.session_obj.robot, f'joint_{i}')
                        except AttributeError:
                            break
                        try:
                            angle = self._read_value(getattr(jnode, 'angle'))
                            names.append(f'joint_{i}')
                            pos.append(float(angle))
                        except Exception:
                            break

            # 發布 JointState（單一時間戳）
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names or []
            msg.position = pos or []
            self.pub_joint_states.publish(msg)

            # 可選：每關節一個 topic（角度）
            if self.enable_per_joint_topics and msg.name and msg.position:
                for jn, jv in zip(msg.name, msg.position):
                    topic = f'/robot/{jn}/angle'
                    pub = self.per_joint_pubs.get(topic)
                    if pub is None:
                        # 使用同樣的 QoS，且深度 1 即可
                        qos = QoSProfile(
                            reliability=(ReliabilityPolicy.BEST_EFFORT if self.joint_qos_reliability == 'best_effort' else ReliabilityPolicy.RELIABLE),
                            history=HistoryPolicy.KEEP_LAST,
                            depth=1,
                            durability=DurabilityPolicy.VOLATILE,
                        )
                        pub = self.create_publisher(Float64, topic, qos)
                        self.per_joint_pubs[topic] = pub
                    m = Float64(); m.data = float(jv)
                    pub.publish(m)

        except Exception as e:
            self.get_logger().warn(f'Publish joint_states failed: {e}')

    def handle_request(self, msg: String):
        self.get_logger().info(f"[REQ] {msg.data}")

        def to_jsonable(x):
            try:
                if isinstance(x, (str, int, float, bool)) or x is None:
                    return x
                if isinstance(x, bytes):
                    return x.decode('utf-8', errors='ignore')
                if isinstance(x, (list, tuple)):
                    return [to_jsonable(i) for i in x]
                if isinstance(x, dict):
                    return {str(k): to_jsonable(v) for k, v in x.items()}
                try:
                    import numpy as np
                    if isinstance(x, (np.generic,)):
                        return x.item()
                except Exception:
                    pass
                return repr(x)
            except Exception:
                return repr(x)

        try:
            req = json.loads(msg.data)
            req_id = req.get("id")
            op = req.get("op")
            path = req.get("path")
            args = req.get("args", [])

            if self.session_obj is None:
                raise RuntimeError("No active session. Device not established yet.")

            # === CHANGED ===：所有對 session_obj 的操作加鎖
            with self.io_lock:
                target = self.resolve_path(self.session_obj, path)

                if op == "read":
                    value = self._read_value(target)
                    res = {"id": req_id, "ok": True, "result": to_jsonable(value)}

                elif op in ("write", "set"):
                    if callable(target):
                        value = target(*args)
                        res = {"id": req_id, "ok": True, "result": to_jsonable(value)}
                    else:
                        parent_path, attr = path.rsplit('.', 1)
                        parent = self.resolve_path(self.session_obj, parent_path) if parent_path else self.session_obj
                        if not args:
                            raise ValueError("write op expects args[0] as value")
                        setattr(parent, attr, args[0])
                        res = {"id": req_id, "ok": True, "result": None}

                elif op == "call":
                    if not callable(target):
                        raise TypeError(f"Target at '{path}' is not callable")
                    value = target(*args)
                    res = {"id": req_id, "ok": True, "result": to_jsonable(value)}
                else:
                    raise ValueError(f"Unknown op: {op}")

        except Exception as e:
            res = {"id": req.get("id") if 'req' in locals() else None, "ok": False, "error": str(e)}

        out = String()
        out.data = json.dumps(res, ensure_ascii=False)
        self.pub_resp.publish(out)
        self.get_logger().info(f"[RESP] {out.data}")


def main():
    rclpy.init()
    node = FibreDiscoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
