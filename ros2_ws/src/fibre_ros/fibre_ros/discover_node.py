import json
import threading
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import fibre
from fibre.utils import Event, Logger
from fibre.protocol import ChannelBrokenException, ChannelDamagedException

class FibreDiscoverNode(Node):
    def __init__(self):
        super().__init__('fibre_discover_node')

        # === 參數 ===
        self.path = self.declare_parameter('path', 'usb,serial').get_parameter_value().string_value
        serial_param = self.declare_parameter('serial_number', '').get_parameter_value().string_value
        self.serial_number: Optional[str] = serial_param or None
        self.verbose = self.declare_parameter('verbose', False).get_parameter_value().bool_value

        self.single_shot = self.declare_parameter('single_shot', True).get_parameter_value().bool_value
        self.hold_session = self.declare_parameter('hold_session', True).get_parameter_value().bool_value

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

        try:
            import fibre.serial_transport as st
            st.DEFAULT_BAUDRATE = self.declare_parameter('serial_baudrate', 115200).get_parameter_value().integer_value
        except Exception:
            pass

        # === ROS 通道 ===
        self.pub_devices = self.create_publisher(String, 'fibre/devices', 10)
        self.pub_resp    = self.create_publisher(String, 'fibre/response', 10)
        self.sub_req     = self.create_subscription(String, 'fibre/request', self.handle_request, 10)

        # === 狀態 ===
        self.logger_fibre = Logger(verbose=self.verbose)
        self.search_stop = Event()
        self.channel_termination = Event() if not self.hold_session else None

        self.devices: Dict[str, Dict] = {}
        self.session_obj = None  # RemoteObject
        self.get_logger().info(f"Fibre discovery started. path={self.path}, serial={self.serial_number or '*'}")

        self.thread = threading.Thread(target=self.discovery_loop, daemon=True)
        self.thread.start()

    # -------- 掃描：找到就停，並留下 session --------
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
                self.search_stop.set()  # 停止 find_all 的掃描 loop
                self.get_logger().info("Session established and discovery stopped.")

        try:
            fibre.discovery.find_all(
                path=self.path,
                serial_number=self.serial_number,
                did_discover_object_callback=did_discover,
                search_cancellation_token=self.search_stop,
                channel_termination_token=self.channel_termination,  # 若 hold_session=True 會傳 None，OK
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

    # -------- 解析字串路徑並讀 / 寫 / 呼叫 --------
    def resolve_path(self, root: Any, path: str):
        node = root
        for part in path.split('.'):
            if not part:
                continue
            node = getattr(node, part)
        return node

    def handle_request(self, msg: String):
        # 方便除錯：把請求寫到 rosout
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

            target = self.resolve_path(self.session_obj, path)

            if op == "read":
                value = target  
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

