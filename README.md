# dummy_ros2

ROS 2 packages for **Dummy-Robot**

This package helps you connect to Dummy-Robot via ROS 2.  
It performs: **Discover the JSON interface → stop scanning → establish a persistent session**, after which you can control the robot by publishing commands to topics.

---

## Features

- **Single-shot discovery** – stop scanning when the first Dummy-Robot is found.
- **Persistent session (`hold_session`)** – keep the underlying channel open during operation.
- **Unified command channel** – send JSON requests on `/fibre/request`, receive JSON replies on `/fibre/response`.
- **Plug-and-play** – supports both `usb` and `serial`; common ODrive VID/PIDs are whitelisted (and can be customized).

---

## Requirements

- ROS 2 (Foxy or a compatible distro)
- Python 3.8+
- Recommended system packages:
  - `libusb-1.0-0` (when using USB)
  - `python3-serial` (when using serial)
- User group membership: `dialout` and `plugdev` (so you don’t need `sudo` for serial/USB)
  - Quick checks: `lsusb` to list USB devices; `ls /dev/ttyACM*` for serial devices.

---

## Build

```zsh
# 1) Workspace
cd ~/ros2_ws

# 2) Build only the fibre_ros package
colcon build --packages-select fibre_ros --symlink-install
```

---

## Source the environment (every terminal)

```zsh
# If your shell is bash, change .zsh to .bash
source /opt/ros/foxy/setup.zsh
source ~/ros2_ws/install/setup.zsh
```

---

## Quick start (discover + create session)

```zsh
# Can be run in any directory (as long as you have sourced the env)
ros2 launch fibre_ros discover.launch.py   path:='usb,serial'   single_shot:=true   hold_session:=true   verbose:=true
```

If you see:

```
[INFO] [fibre_discover]: Session established and discovery stopped.
```

you are connected to the Dummy-Robot; scanning is stopped and the session is kept alive.

---

## Monitor replies (so you don’t miss messages)

Open a **second terminal** (with the environment sourced) and start listening **before** sending any requests:

```zsh
ros2 topic echo /fibre/response
```

---

## Send commands

All commands are published to `/fibre/request` as a `std_msgs/String` containing a JSON object.

### 1) Homing

```zsh
ros2 topic pub -1 /fibre/request std_msgs/String "data: '{"id":"home","op":"call","path":"robot.homing"}'"
```

Expected reply (printed on `/fibre/response`):

```json
{ "id": "home", "ok": true, "result": null }
```

> Homing is asynchronous on the device side; allow time for the motion to complete.

### 2) Enable the robot and set motion limits (recommended before motion)

```zsh
# Enable
ros2 topic pub -1 /fibre/request std_msgs/String "data: '{"id":"en","op":"call","path":"robot.set_enable","args":[true]}'"

# Optional: set joint speed and acceleration (units depend on firmware)
ros2 topic pub -1 /fibre/request std_msgs/String "data: '{"id":"spd","op":"call","path":"robot.set_joint_speed","args":[0.5]}'"
ros2 topic pub -1 /fibre/request std_msgs/String "data: '{"id":"acc","op":"call","path":"robot.set_joint_acc","args":[1.0]}'"
```

### 3) Read telemetry

```zsh
# Single joint angle
ros2 topic pub -1 /fibre/request std_msgs/String "data: '{"id":"rj1","op":"read","path":"robot.joint_1.angle"}'"

# All joints (if your firmware exposes a joint_all aggregate)
ros2 topic pub -1 /fibre/request std_msgs/String "data: '{"id":"rall","op":"read","path":"robot.joint_all.angle"}'"
```

### 4) Move in joint space

```zsh
ros2 topic pub -1 /fibre/request std_msgs/String "data: '{"id":"mvj","op":"call","path":"robot.move_j","args":[0,0,0,0,0,0]}'"
```

---

## Topic protocol

- **Request** topic: `/fibre/request` (`std_msgs/String`)
  - JSON fields:
    - `id` (string): caller-defined request ID to correlate replies
    - `op` (string): `"read"`, `"write"` or `"call"`
    - `path` (string): target path, e.g. `robot.joint_1.angle` or `robot.homing`
    - `args` (array, optional): arguments for `call`/`write`
- **Response** topic: `/fibre/response` (`std_msgs/String`)
  - JSON fields:
    - `id`: echoes your request ID
    - `ok`: `true` on success, `false` on error
    - `result`: return value (may be `null`)
    - `error`: error message when `ok:false`
- **Device events**: `/fibre/devices` (`std_msgs/String`)
  - Emits a record like `{"action":"add", ...}` when a device is connected.

---

## Launch parameters (`discover.launch.py`)

| Parameter         | Type   | Default         | Description                                                 |
| ----------------- | ------ | --------------- | ----------------------------------------------------------- |
| `path`            | string | `'usb'`         | `'usb'`, `'serial:/dev/ttyACM0'`, or `'usb,serial'`.        |
| `single_shot`     | bool   | `true`          | Stop after the first device is found.                       |
| `hold_session`    | bool   | `true`          | Keep the channel open (do not terminate after discover).    |
| `verbose`         | bool   | `false`         | Print low-level transport and JSON introspection.           |
| `serial_baudrate` | int    | device-specific | Baudrate when using serial transports.                      |
| `extra_vid_pid`   | list   | `[]`            | Extra USB VID:PID pairs to whitelist, e.g. `['1209:0d32']`. |

Example with an explicit serial device:

```zsh
ros2 launch fibre_ros discover.launch.py path:='serial:/dev/ttyACM0' single_shot:=true hold_session:=true
```

---

## Troubleshooting

- **No responses received**  
  Always start `ros2 topic echo /fibre/response` **before** publishing requests; responses are not replayed.
- **Permission denied**  
  Ensure your user is in `dialout` and `plugdev`. Re-login or open a new terminal after changing groups.
- **USB claimed by a kernel driver**  
  Unplug/replug the device. If the issue persists, try the serial transport (`serial:/dev/ttyACM0`).
- **`read` fails on some fields**  
  Prefer `call`-style APIs when available (many firmwares expose data via functions).
- **Session dropped**  
  Relaunch the discovery with `single_shot:=true` and `hold_session:=true` to re-establish the session.

---

## Advanced (optional)

- **Define current pose as “home”** (if supported by your firmware):

  ```zsh
  # Calibrate and apply home offset at the current pose
  ros2 topic pub -1 /fibre/request std_msgs/String   "data: '{"id":"cal","op":"call","path":"robot.calibrate_home_offset"}'"

  ros2 topic pub -1 /fibre/request std_msgs/String   "data: '{"id":"apply","op":"call","path":"robot.apply_home_offset"}'"
  ```

- **Enable on boot**:

  ```zsh
  ros2 topic pub -1 /fibre/request std_msgs/String   "data: '{"id":"boot","op":"call","path":"robot.set_enable_on_boot","args":[true]}'"
  ```

---

## Notes

- Start the **response monitor** first to avoid missing replies:
  ```zsh
  ros2 topic echo /fibre/response
  ```
- Use `verbose:=true` during bring-up to see the discovered JSON interface for your device. This helps you confirm available paths (e.g., `robot.homing`, `robot.move_j`, `robot.joint_1.angle`, etc.).
