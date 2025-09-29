# Dummy ROS2 Bridge 說明文件（修正版）

_最後更新：2025-09-29_

---

## 目標與原則

- **保留既有 CLI**：仍可直接送 `@x,y,z,a,b,c`、`&j1..j6`、`!START`、`#GETJPOS` 等。
- **標準 ROS2 型別**：JointState、PoseStamped、Float64、Bool、Trigger…。
- **單位一致**：ROS2 內部一律使用 **SI 制**：m / rad。若韌體是 deg，橋接層負責轉換。
- **可多裝置**：以裝置 `serial_number` 作為命名空間，如：`/dummy/ABCD/...`。
- **熱插拔/重連**：discover 與控制分離；裝置狀態明確可觀察。

---

## 1. 韌體提供的接口（摘要）

> 來源：你提供的 `fibre` 與 `simple_cli` 清單

### 1.1 fibre（層級摘要）

```
/robot
├─ serial_number : uint64 [r]
├─ get_temperature() -> float
├─ get_voltage() -> float
├─ robot/
│  ├─ calibrate_home_offset() -> void
│  ├─ homing() -> void
│  ├─ resting() -> void
│  ├─ joint_1..joint_6/
│  │  ├─ angle : float [r]
│  │  ├─ reboot() -> void
│  │  ├─ get_temperature() -> uint32
│  │  ├─ set_enable_temperature(enable: bool) -> void
│  │  ├─ erase_configs() -> void
│  │  ├─ set_enable(enable: bool) -> void
│  │  ├─ set_position_with_time(pos: float, time: float) -> void
│  │  ├─ set_position(pos: float) -> void
│  │  ├─ set_velocity(vel: float) -> void
│  │  ├─ set_velocity_limit(vel: float) -> void
│  │  ├─ set_current(current: float) -> void
│  │  ├─ set_current_limit(current: float) -> void
│  │  ├─ set_node_id(id: uint32) -> void
│  │  ├─ set_acceleration(acc: float) -> void
│  │  ├─ apply_home_offset() -> void
│  │  ├─ do_calibration() -> void
│  │  ├─ set_enable_on_boot(enable: bool) -> void
│  │  ├─ set_dce_kp/kv/ki/kd(…: int32) -> void
│  │  ├─ set_enable_stall_protect(enable: bool) -> void
│  │  └─ update_angle() -> void
│  ├─ joint_all/
│  └─ hand/
│     ├─ set_angle(angle: float) -> void
│     ├─ set_enable(enable: bool) -> void
│     └─ set_current_limit(current: float) -> void
├─ reboot() -> void
├─ set_enable(enable: bool) -> void
├─ set_rgb_enable(enable: bool) -> void
├─ set_rgb_mode(mode: uint32) -> void
├─ move_j(j1..j6: float) -> result: bool
├─ move_l(x,y,z,a,b,c: float) -> result: bool
├─ set_joint_speed(speed: float) -> void
├─ set_joint_acc(acc: float) -> void
├─ set_command_mode(mode: uint32) -> void
└─ tuning/
   ├─ set_tuning_freq_amp(freq: float, amp: float) -> void
   └─ set_tuning_flag(flag: uint8) -> void
```

### 1.2 simple_cli（直通指令）

```
!START  !STOP  !HOME  !CALIBRATION  !RESET  !DISABLE
#GETJPOS  #GETLPOS  #SET_DCE_KP  #SET_DCE_KI  #SET_DCE_KD  #REBOOT  #CMDMODE

@x,y,z,a,b,c        // 例：@175,44,160,-135,-13.54,-14.6
&j1,j2,j3,j4,j5,j6  // 例：&0,0,134,35,66,0
```

> 假設韌體亦會回傳類似 `JA: ...`（六軸角）與 `CP: ...`（笛卡兒位姿）行，或透過 `#GETJPOS/#GETLPOS` 單次回覆。

---

## 2. 節點架構（**推薦**）

### 2.1 `discover_node`（裝置掃描 / 會話管理）

- **職責**：掃描 USB/Serial，建立/關閉/reconnect 會話；分配命名空間
- **Topics**
  - `/fibre/devices : std_msgs/String`  
    JSON 陣列字串，例：`[{"path":"/dev/ttyACM0","serial":"ABCD"}]`
  - `/fibre/session_events : std_msgs/String`  
    例：`{"event":"opened","session_id":"sess-01","serial":"ABCD","ns":"/dummy/ABCD"}`
- **Services**
  - `/fibre/open`（req: `serial|path`；res: `ok,bool; session_id,string; ns,string; port,string`）
  - `/fibre/close`（req: `session_id`；res: `ok`）
  - `/fibre/reconnect`（req: `session_id`；res: `ok`）

### 2.2 `dummy_node`（主控制/橋接；**保留 @ / & / ! / #**）

- 啟動 → 呼叫 `/fibre/open`（帶序號或路徑）→ 成功後在 `ns=/dummy/<serial>` 下啟用所有 I/O。

---

## 3. ROS2 介面規格（於 `/<ns>` 下）

> 以下 `/<ns>` 指 `/dummy/<serial>`，例如 `/dummy/ABCD`。

### 3.1 Topics（狀態輸出）

- `/<ns>/state/connected : std_msgs/Bool`（latched）
- `/<ns>/state/voltage : std_msgs/Float64`（Hz 可設，預設 1 Hz）
- `/<ns>/state/temperature : std_msgs/Float64`（預設 1 Hz）
- `/<ns>/joint_states : sensor_msgs/JointState`
  - `name = ["joint_1",...,"joint_6"]`
  - `position[6]` 單位：**rad**
  - 來源：`#GETJPOS` 或 streaming `JA:`，節流頻率 `joint_state_rate_hz`（預設 30 Hz）
- `/<ns>/ee_pose : geometry_msgs/PoseStamped`（**主**）
  - `header.frame_id = "base_link"`，姿態用四元數
  - 若韌體以 RPY 回傳，橋接層轉為四元數
- `/<ns>/ee_rpy : std_msgs/Float64MultiArray`（**副**，長度 6：x,y,z, roll,pitch,yaw）
  - 單位：m / rad
- `/<ns>/echo : std_msgs/String`
  - 串口原始回顯（除錯）

> **TF**：同時發布 `base_link -> tool0` 的 `TransformStamped`。

### 3.2 Topics（命令輸入；保留原味）

- `/<ns>/cli : std_msgs/String`

  - 可直接送 `@...`、`&...`、`!START`、`#GETJPOS` 等。

- `/<ns>/joint_targets : std_msgs/Float64MultiArray[6]`

  - 轉為 `&j1..j6`（必要時度 ↔ 弧轉換）

- `/<ns>/cartesian_target : std_msgs/Float64MultiArray[6]`
  - 轉為 `@x,y,z,a,b,c`（RPY；必要時度 ↔ 弧轉換）

### 3.3 Services

- 系統：

  - `/<ns>/reboot : std_srvs/Trigger`
  - `/<ns>/set_enable : std_srvs/SetBool`
  - `/<ns>/set_rgb_enable : std_srvs/SetBool`
  - `/<ns>/set_rgb_mode : dummy_robot_bridge/SetUInt32`（如需無號整數）
  - `/<ns>/set_joint_speed : std_srvs/SetFloat64`
  - `/<ns>/set_joint_acc : std_srvs/SetFloat64`
  - `/<ns>/set_command_mode : dummy_robot_bridge/SetUInt32`

- 動作：

  - `/<ns>/move_j : dummy_robot_bridge/MoveJ`
    - `request: float64[6] joints`（rad）
    - `response: bool success, string message`
  - `/<ns>/move_l : dummy_robot_bridge/MoveL`
    - `request: float64[6] pose`（x,y,z, r,p,y；m,rad）
    - `response: bool success, string message`
  - `/<ns>/move_l_partial : dummy_robot_bridge/MoveLPartial`（**建議新增**）
    - `request: float64[6] pose, bool[6] mask`（`mask[i]=true` 表示覆蓋第 i 項，其餘保留目前值）
    - `response: bool success, string message`

- 姿態：

  - `/<ns>/robot/homing : std_srvs/Trigger`
  - `/<ns>/robot/resting : std_srvs/Trigger`
  - `/<ns>/robot/calibrate_home_offset : std_srvs/Trigger`

- 關節（可選，若需逐軸微調）：
  - `/<ns>/robot/joint_X/set_enable : std_srvs/SetBool`
  - `/<ns>/robot/joint_X/set_current : std_srvs/SetFloat64`
  - `/<ns>/robot/joint_X/do_calibration : std_srvs/Trigger`
  - `/<ns>/robot/joint_X/set_velocity : std_srvs/SetFloat64`
  - `/<ns>/robot/joint_X/set_dce_k{p,v,i,d} : std_srvs/SetFloat64`（或 `SetInt32`，看韌體）
  - `/<ns>/robot/joint_X/set_position : std_srvs/SetFloat64`
  - `/<ns>/robot/joint_X/set_position_with_time : dummy_robot_bridge/SetTwoFloat64`

> **命名約定**：以 64-bit 浮點為主；整數需求再用 `SetUInt32`。

### 3.4（選）Actions

- 若需持續回饋/可取消：
  - `/<ns>/move_j_action : MoveJ.action`
  - `/<ns>/move_l_action : MoveL.action`
  - Feedback：目前誤差、剩餘時間；Result：success/message

---

## 4. 參數（`dummy_node` 常用）

- `port`（string）：序列埠（由 `/fibre/open` 回傳時可忽略此參數）
- `baud`（int，預設 115200）
- `joint_state_rate_hz`（int，預設 30）
- `ee_state_rate_hz`（int，預設 30）
- `frame_base`（string，預設 `"base_link"`）
- `frame_tool`（string，預設 `"tool0"`）
- `degrees_mode`（bool，預設 `false`）：若韌體回傳/接收為度，橋接層進行轉換
- `session_required`（bool，預設 `true`）：未連線時不啟動 I/O

---

## 5. QoS 與時間基準

- 狀態類（`joint_states`、`ee_pose`、`voltage`、`temperature`）：`SensorDataQoS`（reliable/best effort 視需要）。
- `connected` 使用 latched（transient local）。
- 以 node 本地時鐘為準；Pose/TF 使用 `rclcpp::Clock(RCL_SYSTEM_TIME)` 或 ROS 預設。

---

## 6. 斷線/重連行為

- 一旦串口/會話中斷：
  - 停止狀態發布，`/<ns>/state/connected=false`（latched）。
  - 所有服務立即回 `success=false, message="Disconnected"`。
  - 觸發重連（指數退避），成功後發布 `connected=true` 並在 `diag` 留存「recovered after Xs」。

---

## 7. CLI 直通（保留 @ / & / ! / #）

- 任何時候可將一整行字串送到 `/<ns>/cli`：
  - `!START`、`!DISABLE`、`#GETJPOS`、`@175,44,160,-135,-13.54,-14.6`、`&0,0,134,35,66,0`…
- 所有裝置原始輸出行會在 `/<ns>/echo` 回顯。
- 建議在橋接層預設 **短暫等待** `ok/done/complete/error` 等關鍵詞後再返回，並將回應行同步丟到 `echo`。

---

## 8. 範例操作

### 8.1 啟動 discover 並建立會話

```bash
# 啟 discover_node
ros2 run dummy_bridge discover_node

# dummy_node 啟動時會自行呼叫 /fibre/open
ros2 run dummy_bridge dummy_node --ros-args -p session_required:=true
```

### 8.2 直接使用原味指令

```bash
# 使能
ros2 topic pub -1 /dummy/ABCD/cli std_msgs/String "data: '!START'"

# 送末端位姿（RPY）
ros2 topic pub -1 /dummy/ABCD/cli std_msgs/String "data: '@175,44,160,-135,-13.54,-14.6'"

# 送六關節角
ros2 topic pub -1 /dummy/ABCD/cli std_msgs/String "data: '&0,0,134,35,66,0'"
```

### 8.3 用語意友善 Topic

```bash
ros2 topic pub -1 /dummy/ABCD/joint_targets std_msgs/Float64MultiArray '{data:[0,0,134,35,66,0]}'
ros2 topic pub -1 /dummy/ABCD/cartesian_target std_msgs/Float64MultiArray '{data:[175,44,160,-135,-13.54,-14.6]}'
```

### 8.4 讀狀態

```bash
ros2 topic echo /dummy/ABCD/joint_states
ros2 topic echo /dummy/ABCD/ee_pose
ros2 topic echo /dummy/ABCD/echo
```

### 8.5 服務呼叫

```bash
ros2 service call /dummy/ABCD/move_j dummy_robot_bridge/MoveJ "{joints: [0,0,2.34,0.6,1.15,0.0]}"
ros2 service call /dummy/ABCD/move_l dummy_robot_bridge/MoveL "{pose: [0.175,0.044,0.160,-2.356,-0.236,-0.255]}"
ros2 service call /dummy/ABCD/robot/homing std_srvs/srv/Trigger "{}"
ros2 service call /dummy/ABCD/set_enable std_srvs/srv/SetBool "{data: true}"
```

> 上例中 `move_l` 的 RPY 以 **rad** 表示；若你想沿用度，請將 `degrees_mode:=true`，橋接層會轉換。

---

## 9. 自訂訊息/服務（檔名建議）

- `srv/MoveJ.srv`
  ```
  float64[6] joints
  ---
  bool success
  string message
  ```
- `srv/MoveL.srv`
  ```
  float64[6] pose  # x y z r p y (m, rad)
  ---
  bool success
  string message
  ```
- `srv/MoveLPartial.srv`
  ```
  float64[6] pose
  bool[6] mask
  ---
  bool success
  string message
  ```
- `srv/SetTwoFloat64.srv`
  ```
  float64 a
  float64 b
  ---
  bool success
  string message
  ```
- `srv/SetUInt32.srv`
  ```
  uint32 data
  ---
  bool success
  string message
  ```

---

## 10. 風險與備註

- **度/弧轉換**：韌體是度、ROS2 是弧；請務必在橋接層統一（`degrees_mode` 參數）。
- **競態**：避免用多個細碎 Topic 拼裝 `move_l/x..c`；採用 `move_l_partial` 或一次性 `move_l`。
- **QoS**：在高頻遙測場景（>50Hz），建議改為 BestEffort 以避免堆積。
- **安全**：提供 `/estop : Trigger`（映射 `!STOP`），且在斷線時立刻回報 `connected=false`。

---

## 11. 變更摘要（相對原方案）

- 修正錯誤型別（`Float3` → `Float64`/標準型別）。
- `ee_state` 改為 `PoseStamped`（主）+ `Float64MultiArray`（副 RPY）。
- 加入 `connected`、`diag`、TF、命名空間 by serial。
- 新增 `move_l_partial` 以避免競態。
- 提供 CLI 直通 `/<ns>/cli` 與 `/<ns>/echo`。

---

## 其他需求

- Lifecycle Node（Active 才發布資料）。
- Action 版本 `move_j_action`/`move_l_action`（回饋與取消）。
- 參數伺服（dynamic_reconfigure 等價物）。
- 錯誤碼標準化（以 `diagnostic_msgs` 為核心）。

---
