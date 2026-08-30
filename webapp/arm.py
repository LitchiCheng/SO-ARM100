"""SO-ARM100 机械臂控制 / 遥测 / 配置后端（硬件 + 安全层）。

所有外部指令源（Web UI；未来的 ZMQ / 策略推理接入）都只能通过
ArmController 的公开方法访问电机 —— 这里是唯一的 choke point。
"""

import asyncio
import glob
import json
import os
import threading
import time

from hardware import macro
from hardware import FeetechMotor as fm
from hardware.FeetechMotor import (
    offset_deg,
    direction,
    lower_limits_deg,
    upper_limits_deg,
)

K_DEG2RAW = 4096.0 / 360.0   # 度 → raw 步
K_RAW2DEG = 360.0 / 4096.0  # raw 步 → 度
RAW_PER_DEG = K_DEG2RAW

ZEROING_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "zeroing.json")


def _initial_p_ref() -> list:
    """显示 0° 对应的固件 P 值（未标零时的出厂/MuJoCo 零位约定）：
    deg = ((P-2048)·K - off)·dir 为 0 的解"""
    return [2048 + offset_deg[i] * direction[i] / K_RAW2DEG for i in range(6)]

# 串口设备节点：USB 重新插入后可能被重新分配（ttyACM1...），可用环境变量覆盖，
# 或运行时通过 POST /api/reconnect {"port": ...} 切换
PORT = os.environ.get("SOARM_DEVICE", "/dev/ttyACM0")
MOTOR_IDS = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
]
TELEMETRY_HZ = 5.0
MAX_GOAL_SPEED = 2000  # 步/s；SCS 上 Goal_Speed=0 表示不限速，故拒绝 0
PRESET_SPEED = 500     # 与 examples/middle_all_joint.py 一致
PRESETS = {
    "middle": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "rest": [0.0, -90.0, 90.0, 63.0, 0.0, 0.0],
}

# 通用配置写仅允许此白名单 name -> (min, max)。
# 有意排除：ID / Baud_Rate / Lock / Torque_Enable / Offset / P/D/I
# （各有专用端点）以及 Goal_* 等危险项。
WRITABLE_CONFIG = {
    "Min_Angle_Limit": (0, 4095),
    "Max_Angle_Limit": (0, 4095),
    "Max_Temperature_Limit": (30, 99),
    "Max_Voltage_Limit": (8, 16),
    "Min_Voltage_Limit": (6, 15),
    "Max_Torque_Limit": (0, 100),
    "Minimum_Startup_Force": (0, 1000),
    "CW_Dead_Zone": (0, 30),
    "CCW_Dead_Zone": (0, 30),
    "Protection_Current": (0, 1000),
    "Acceleration": (0, 255),
    "Maximum_Acceleration": (0, 1000),
}

# 遥测字段 -> 输出 key（每字段一次批量 GroupSyncRead，覆盖 6 电机）
TELEMETRY_FIELD_KEYS = {
    "Present_Position": "pos_raw",
    "Goal_Position": "goal_raw",
    "Present_Speed": "speed",
    "Present_Load": "load",
    "Present_Voltage": "voltage",
    "Present_Temperature": "temp",
    "Status": "status",
    "Moving": "moving",
    "Present_Current": "current",
    "Torque_Enable": "torque",
}


class LockedError(RuntimeError):
    """机械臂未解锁（armed）时下发运动指令被拒绝"""


class PoseStore:
    """命名点位表（JSON 文件，跨浏览器/设备持久化）"""

    def __init__(self, path: str):
        self.path = path
        self._lock = threading.Lock()

    def _read_file(self) -> dict:
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                d = json.load(f)
            return d if isinstance(d, dict) else {}
        except (FileNotFoundError, json.JSONDecodeError):
            return {}

    def _write_file(self, d: dict):
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(d, f, ensure_ascii=False, indent=1)
        os.replace(tmp, self.path)

    def list(self) -> dict:
        with self._lock:
            return self._read_file()

    def save(self, name: str, values: list) -> list:
        """保存 6 关节位形。values 为固件 P 空间原始值（0~4095，与零位约定无关，
        因此标零不影响已存点位的物理位形）"""
        name = str(name).strip()
        if not (1 <= len(name) <= 20):
            raise ValueError(f"点位名需 1~20 字符，收到 {name!r}")
        if any(c in name for c in '{}"/\\'):
            raise ValueError("点位名不能包含 { } \" / \\ 字符")
        vals = [int(round(float(v))) for v in values]
        if len(vals) != 6 or any(v != v or not (0 <= v <= 4095) for v in vals):
            raise ValueError("点位必须是 6 个 0~4095 的固件位置值")
        with self._lock:
            d = self._read_file()
            d[name] = vals
            self._write_file(d)
        return vals

    def delete(self, name: str):
        with self._lock:
            d = self._read_file()
            if name not in d:
                raise ValueError(f"点位 {name!r} 不存在")
            del d[name]
            self._write_file(d)


class ArmController:
    """单实例 FeetechMotor + 单 PortHandler + 单 RLock，所有串口事务在锁内。"""

    def __init__(self, port: str = PORT):
        self.port = port
        self._motor = fm.FeetechMotor(1, port)
        self._motor.printFlag(False)
        self._lock = threading.RLock()
        self._estop_flag = threading.Event()  # E-stop 期间遥测环提前放弃本周期
        self._armed = False
        self._connected = False
        self._start = time.time()
        self._error_count = 0
        self._tune = None  # PID 自动调参状态（见 start_pid_tune / tune_status）
        # 标零状态（见 zero_joint / zeroing.json 持久化）：
        # _p_ref[i]     显示 0° 对应的固件 P 值（标零后 = 2048）
        # _viz_offset[i] 累计偏移：显示值 + 该值 = MuJoCo 零位约定（3D 模型用）
        # _zeroed_at[i]  最近一次标零时间（None = 未标零）
        # 机械限位窗口随标零平移（物理行程不变）：[lo_model - viz, hi_model - viz]
        z = self._load_zeroing()
        self._p_ref = z.get("p_ref", _initial_p_ref())
        self._viz_offset = z.get("viz_offset_deg", [0.0] * 6)
        self._zeroed_at = z.get("zeroed_at", [None] * 6)

    # ---------------- 标零状态持久化 ----------------

    def _load_zeroing(self) -> dict:
        try:
            with open(ZEROING_PATH, "r", encoding="utf-8") as f:
                d = json.load(f)
            if not isinstance(d, dict):
                return {}
            for key, default in (("p_ref", _initial_p_ref()),
                                 ("viz_offset_deg", [0.0] * 6),
                                 ("zeroed_at", [None] * 6)):
                if len(d.get(key, [])) != 6:
                    d[key] = default
            return d
        except (FileNotFoundError, json.JSONDecodeError, OSError, ValueError):
            return {}

    def _save_zeroing(self):
        d = {"p_ref": self._p_ref, "viz_offset_deg": self._viz_offset,
             "zeroed_at": self._zeroed_at}
        tmp = ZEROING_PATH + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(d, f, ensure_ascii=False, indent=1)
        os.replace(tmp, ZEROING_PATH)

    # ---------------- 生命周期 ----------------

    def connect(self):
        with self._lock:
            self._motor.connect()
            self._connected = True

    def disconnect(self):
        with self._lock:
            try:
                self._motor.disconnect()
            except Exception:
                # 启动时 openPort 失败（内部 ser=None）时 closePort 抛 AttributeError，
                # 句柄本就未打开，丢弃引用即可
                self._motor.port_handler = None
            finally:
                self._connected = False

    @property
    def is_connected(self):
        return self._connected

    @property
    def armed(self):
        return self._armed

    def set_armed(self, armed: bool):
        self._armed = bool(armed)
        return self._armed

    def status(self):
        with self._lock:
            ph = self._motor.port_handler
            port_baud = ph.getBaudRate() if (self._connected and ph is not None) else None
        return {
            "connected": self._connected,
            "port": self.port,
            "port_baudrate": port_baud,
            "armed": self._armed,
            "uptime_s": round(time.time() - self._start, 1),
            "telemetry_hz": TELEMETRY_HZ,
            "error_count": self._error_count,
        }

    # ---------------- 限位 / 重连 ----------------

    def get_limits(self) -> list:
        """机械限位（当前显示约定，随标零平移）+ 固件 Min/Max_Angle_Limit 实时值。
        固件窗口比机械限位更窄时电机目标会被钳位（"某些位置不动"的根因）；
        串口断开/读取失败时 hw_*_deg 为 null。
        """
        hw = [(None, None)] * 6
        if self._connected:
            try:
                with self._lock:
                    lo = self._read("Min_Angle_Limit", MOTOR_IDS, num_retry=1)
                    hi = self._read("Max_Angle_Limit", MOTOR_IDS, num_retry=1)
                for idx, mid in enumerate(MOTOR_IDS):
                    a, b = self._p2deg(mid, lo[idx]), self._p2deg(mid, hi[idx])
                    hw[idx] = (min(a, b), max(a, b))  # dir=-1 的电机在显示系 min/max 互换
            except (ConnectionError, ValueError):
                pass
        out = []
        for i in range(1, 7):
            wlo, whi = self._window_deg(i)
            out.append({
                "id": i, "name": JOINT_NAMES[i - 1],
                "lower_deg": wlo, "upper_deg": whi,
                "viz_offset_deg": self._viz_offset[i - 1],
                "zeroed_at": self._zeroed_at[i - 1],
                "hw_lower_deg": hw[i - 1][0], "hw_upper_deg": hw[i - 1][1],
            })
        return out

    def reconnect(self, port: str = None) -> dict:
        """关闭并重新打开串口（USB 插拔后旧句柄/设备节点失效时），再读一次 Model 验证总线。
        可传 port 切换设备节点（重新插入后常被重新分配为 ttyACM1 等）。
        """
        if port:
            port = os.fspath(port)
            if not port.startswith("/dev/"):
                raise ValueError(f"端口 {port} 非法（需以 /dev/ 开头）")
            self.port = port
        self.disconnect()
        try:
            self.connect()
        except Exception as e:
            cands = sorted(glob.glob("/dev/ttyACM*"))
            hint = f"；当前可用设备节点: {', '.join(cands)}" if cands else "；未发现 /dev/ttyACM* 设备（USB 是否已插入？）"
            raise ConnectionError(f"重连失败（{self.port} 打不开）{hint}: {e}")
        with self._lock:
            self._read("Model", MOTOR_IDS, num_retry=2)
        return self.status()

    # ---------------- 串口原语（调用方须持锁；RLock 可重入） ----------------

    def _read(self, data_name, motor_ids, num_retry=3):
        return self._motor.read_with_motor_ids(motor_ids, data_name, num_retry=num_retry)

    def _write(self, data_name, motor_ids, values, num_retry=3):
        self._motor.write_with_motor_ids(motor_ids, data_name, values, num_retry=num_retry)

    # ---------------- 坐标系换算（标零感知） ----------------
    # 固件坐标系：P = (raw - Offset) mod 4096（raw 为编码器计数，2048 = 机械中心；
    # 由 examples/offset_all_joint.py 的 setOffsetCurrent 逻辑反推验证）。
    # 显示坐标系：deg = (P - p_ref)·K·dir，p_ref = 显示 0° 对应的 P（标零后为 2048）。

    def _p2deg(self, mid: int, p) -> float:
        i = mid - 1
        return round((p - self._p_ref[i]) * K_RAW2DEG * direction[i], 2)

    def _deg2raw(self, mid: int, deg) -> tuple:
        """显示约定 deg → (固件 raw 0~4095, 钳位后 deg)；钳位到当前约定的机械限位"""
        i = mid - 1
        lo, hi = lower_limits_deg[i] - self._viz_offset[i], upper_limits_deg[i] - self._viz_offset[i]
        v = max(min(deg, hi), lo)
        raw = (self._p_ref[i] + v * K_DEG2RAW * direction[i]) % 4096
        return int(max(0, min(round(raw), 4095))), round(v, 2)

    def _window_deg(self, mid: int):
        """当前显示约定下的机械限位窗口（标零后随零位平移，物理行程不变）"""
        i = mid - 1
        return lower_limits_deg[i] - self._viz_offset[i], upper_limits_deg[i] - self._viz_offset[i]

    # ---------------- 标零（单关节：当前位置 = 0°） ----------------

    def zero_joint(self, motor_id: int) -> dict:
        """把当前位置设为该关节的新 0°（写 Offset 寄存器，一次写入完成，不做 Offset=0 两步流程）。

        · 不产生运动（只改坐标基准），调用前应确认关节已静止
        · 该关节的显示/滑杆/限位窗口此后相对新零位；3D 模型经 viz_offset 保持 MuJoCo 对齐
        · 该关节固件限位寄存器重置为全范围 0~4095（与 examples/offset_all_joint.py 行为一致）；
          UI 下发仍按平移后的机械限位窗口钳位
        """
        if not (1 <= motor_id <= 6):
            raise ValueError("motor_id 必须在 1~6")
        i = motor_id - 1
        with self._lock:
            p = self._read("Present_Position", [motor_id], num_retry=1)[0]
            o = self._read("Offset", [motor_id], num_retry=1)[0]
            d_now = self._p2deg(motor_id, p)  # 标零前的显示值（限位窗口平移量 / 3D 偏移增量）
            o_new = (o + p - 2048) % 4096  # 令当前位置满足 (raw - O') mod 4096 = 2048
            self._write("Offset", [motor_id], [int(o_new)])
            self._write("Min_Angle_Limit", [motor_id], [0])
            self._write("Max_Angle_Limit", [motor_id], [4095])
            rb_offset = self._read("Offset", [motor_id])[0]
            rb_p = self._read("Present_Position", [motor_id], num_retry=1)[0]
        # 更新显示约定（串口锁外；状态仅 UI 线程读取，标零本身低频）
        self._p_ref[i] = 2048.0
        self._viz_offset[i] += d_now
        self._zeroed_at[i] = time.strftime("%Y-%m-%d %H:%M:%S")
        self._save_zeroing()
        wlo, whi = self._window_deg(motor_id)
        return {
            "ok": True, "id": motor_id,
            "display_deg_before": d_now,
            "offset_before": o, "offset_after": rb_offset,
            "present_after": rb_p, "present_deg_after": self._p2deg(motor_id, rb_p),
            "limits_reset": True,
            "window_deg": [wlo, whi],
        }

    # ---------------- 遥测 ----------------

    def read_telemetry(self) -> dict:
        """单周期遥测（内部持锁）。字段读取失败填 None 并计入 errors。"""
        with self._lock:
            return self._snapshot_locked()

    def _snapshot_locked(self) -> dict:
        raw = {}
        errors = 0
        for field in TELEMETRY_FIELD_KEYS:
            if self._estop_flag.is_set():
                break
            try:
                raw[field] = self._read(field, MOTOR_IDS, num_retry=1)
            except ConnectionError:
                raw[field] = [None] * len(MOTOR_IDS)
                errors += 1
        if errors:
            self._error_count += errors

        joints = []
        for idx, mid in enumerate(MOTOR_IDS):
            j = {"id": mid, "name": JOINT_NAMES[idx]}
            for field, key in TELEMETRY_FIELD_KEYS.items():
                vals = raw.get(field)
                j[key] = vals[idx] if vals else None
            j["pos_deg"] = self._p2deg(mid, j["pos_raw"]) if j["pos_raw"] is not None else None
            j["goal_deg"] = self._p2deg(mid, j["goal_raw"]) if j["goal_raw"] is not None else None
            j["viz_offset_deg"] = self._viz_offset[idx]  # 3D 模型用：显示值 + 该值 = MuJoCo 零位系
            if j["voltage"] is not None:
                j["voltage"] = round(j["voltage"] * 0.1, 1)  # SCS: raw * 0.1 V
            joints.append(j)
        return {"ts": time.time(), "joints": joints, "errors": errors}

    # ---------------- 运动（须 armed，否则 LockedError -> 423） ----------------

    def _require_armed(self):
        t = self._tune
        if t and t.get("state") in ("starting", "waiting", "tuning"):
            raise LockedError(f"PID 自动调参进行中（J{t['mid']}），暂时拒绝其他运动指令（E-stop/扭矩/速度不受影响）")
        if not self._armed:
            raise LockedError("机械臂未解锁（armed），已拒绝运动指令")

    def set_position(self, motor_id: int, deg: float) -> dict:
        self._require_armed()
        if not (1 <= motor_id <= 6):
            raise ValueError("motor_id 必须在 1~6")
        raw, clamped = self._deg2raw(motor_id, float(deg))
        with self._lock:
            self._write("Goal_Position", [motor_id], [raw])
        return {"id": motor_id, "requested": float(deg), "clamped_deg": clamped, "raw": raw}

    def apply_preset(self, name: str) -> dict:
        self._require_armed()
        if name not in PRESETS:
            raise ValueError(f"未知预设：{name}（可选：{list(PRESETS)}）")
        targets = PRESETS[name]  # MuJoCo 零位约定的物理位形
        results = []
        with self._lock:
            self._write("Goal_Speed", MOTOR_IDS, [PRESET_SPEED] * len(MOTOR_IDS))
            for idx, (mid, deg) in enumerate(zip(MOTOR_IDS, targets)):
                deg_disp = deg - self._viz_offset[idx]  # 转当前显示约定
                raw, clamped = self._deg2raw(mid, deg_disp)
                results.append({"id": mid, "requested": deg,
                                "clamped_deg": round(clamped + self._viz_offset[idx], 2),
                                "raw": raw})
                self._write("Goal_Position", [mid], [raw])
        return {"ok": True, "preset": name, "results": results}

    def apply_pose(self, values: list, speed: int = PRESET_SPEED) -> dict:
        """应用保存的 6 关节点位（armed + 调参忙碌双重门控）。
        values 为固件 P 空间原始值（0~4095，与坐标系无关）——标零不影响已存点位的物理位形。
        """
        self._require_armed()
        if not isinstance(speed, int) or not (1 <= speed <= MAX_GOAL_SPEED):
            raise ValueError(f"speed 必须在 [1, {MAX_GOAL_SPEED}]")
        try:
            vals = [float(v) for v in values]
        except (TypeError, ValueError):
            raise ValueError("values 必须是 6 个固件位置值")
        if len(vals) != 6 or any(v != v or not (0 <= v <= 4095) for v in vals):
            raise ValueError("点位必须是 6 个 0~4095 的固件位置值")
        results = []
        with self._lock:
            self._write("Goal_Speed", MOTOR_IDS, [speed] * len(MOTOR_IDS))
            for mid, p in zip(MOTOR_IDS, vals):
                deg = self._p2deg(mid, p)
                raw, clamped = self._deg2raw(mid, deg)
                results.append({"id": mid, "deg": deg, "clamped_deg": clamped, "raw": raw})
                self._write("Goal_Position", [mid], [raw])
        return {"ok": True, "results": results}

    # ---------------- E-stop / 扭矩 / 速度 ----------------

    def estop(self) -> dict:
        """急停：全关节立即释放扭矩（自由状态）。无条件可用，无需确认。"""
        self._estop_flag.set()
        try:
            with self._lock:
                self._write("Torque_Enable", MOTOR_IDS, [0] * len(MOTOR_IDS))
        finally:
            self._estop_flag.clear()
        return {"ok": True, "mode": 0, "motors": MOTOR_IDS}

    def set_torque(self, mode: int, motor_ids=None) -> dict:
        """mode: 0=free / 1=enable / 2=damp"""
        if mode not in (0, 1, 2):
            raise ValueError("mode 必须为 0(自由) / 1(使能) / 2(阻尼)")
        ids = list(motor_ids) if motor_ids else list(MOTOR_IDS)
        with self._lock:
            self._write("Torque_Enable", ids, [mode] * len(ids))
        return {"ok": True, "mode": mode, "motors": ids}

    def set_speed(self, speed: int, motor_ids=None) -> dict:
        if not isinstance(speed, int) or not (1 <= speed <= MAX_GOAL_SPEED):
            raise ValueError(f"speed 必须在 [1, {MAX_GOAL_SPEED}]（0 在 SCS 上表示不限速，拒绝）")
        ids = list(motor_ids) if motor_ids else list(MOTOR_IDS)
        with self._lock:
            self._write("Goal_Speed", ids, [speed] * len(ids))
        return {"ok": True, "speed": speed, "motors": ids}

    # ---------------- PID / Offset / 配置 ----------------

    def get_pid(self, motor_id: int) -> dict:
        with self._lock:
            p = self._read("P_Coefficient", motor_id)
            d = self._read("D_Coefficient", motor_id)
            i = self._read("I_Coefficient", motor_id)
        return {"p": p, "d": d, "i": i}

    def set_pid(self, motor_id: int, p: int, i: int, d: int) -> dict:
        for name, v in (("p", p), ("i", i), ("d", d)):
            if not (0 <= int(v) <= 255):
                raise ValueError(f"{name} 必须在 [0, 255]")
        with self._lock:
            self._write("P_Coefficient", [motor_id], [int(p)])
            self._write("D_Coefficient", [motor_id], [int(d)])
            self._write("I_Coefficient", [motor_id], [int(i)])
            readback = self.get_pid(motor_id)
        return {"ok": True, "readback": readback}

    # ---------------- PID 自动调参（保守步进响应法） ----------------

    def start_pid_tune(self, motor_id: int, step_deg: float = 10.0) -> dict:
        """对单个关节做步进响应自动调参：小步长（默认 ±10°）往复运动，
        按超调/稳定时间迭代 P/D，I 保持原值；最多 4 轮，结束后写入最优 P/D/I
        并把目标回写到起点。必须 armed；调参期间其他运动指令被拒绝（E-stop 不受影响）。
        """
        if not (1 <= motor_id <= 6):
            raise ValueError("motor_id 必须在 1~6")
        step_deg = float(step_deg)
        if not (5.0 <= step_deg <= 20.0):
            raise ValueError("step_deg 必须在 [5, 20]")
        self._require_armed()
        self._tune = {"mid": motor_id, "state": "starting", "msg": "准备中", "trial": 0, "result": None}
        t = threading.Thread(target=self._tune_worker, args=(motor_id, step_deg), daemon=True)
        t.start()
        return self.tune_status()

    def tune_status(self) -> dict | None:
        return self._tune

    def _tune_set(self, **kw):
        if self._tune:
            self._tune.update(kw)

    def _wait_settled(self, mid: int, timeout: float = 5.0):
        """等关节静止后返回当前 raw 位置；扭矩关闭/超时返回 None"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            # 注意：read_with_motor_ids 传 int 返回标量、传 list 返回 list —— 调参里统一用 list
            with self._lock:
                try:
                    torque = self._read("Torque_Enable", [mid], num_retry=1)
                    speed = self._read("Present_Speed", [mid], num_retry=1)
                except ConnectionError:
                    return None
            if torque[0] == 0 or abs(speed[0]) >= 4:
                time.sleep(0.1)
                continue
            time.sleep(0.1)  # 再确认一次
            with self._lock:
                try:
                    speed2 = self._read("Present_Speed", [mid], num_retry=1)
                    pos = self._read("Present_Position", [mid], num_retry=1)
                except ConnectionError:
                    return None
            if abs(speed2[0]) < 4:
                return pos[0]
        return None

    def _sample_step(self, mid: int, p0_raw: int, target_raw: int) -> dict:
        """跟踪一次步进：返回 {ok, reason, over_raw, ts_s}（超调量 raw / 首次进入目标窗口的秒数）"""
        d = 1 if target_raw >= p0_raw else -1
        t0 = time.time()
        over_raw = 0
        ts_s = None
        settle_until = None
        while True:
            elapsed = time.time() - t0
            if elapsed > 12.0:
                return {"ok": False, "reason": "12 秒内未稳定", "over_raw": over_raw, "ts_s": ts_s}
            with self._lock:
                try:
                    pos = self._read("Present_Position", [mid], num_retry=1)
                    spd = self._read("Present_Speed", [mid], num_retry=1)
                    torque = self._read("Torque_Enable", [mid], num_retry=1)
                except ConnectionError:
                    return {"ok": False, "reason": "通信失败", "over_raw": over_raw, "ts_s": ts_s}
            if torque[0] == 0:
                return {"ok": False, "reason": "扭矩被关闭（E-stop？）", "over_raw": over_raw, "ts_s": ts_s}
            err = d * (pos[0] - target_raw)
            over_raw = max(over_raw, err)
            if err <= 3:  # 进入目标 ±0.26° 窗口
                if ts_s is None:
                    ts_s = elapsed
                settle_until = time.time() + 0.4
            elif settle_until is not None:
                settle_until = None
            if settle_until is not None and abs(spd[0]) < 4 and time.time() >= settle_until:
                return {"ok": True, "reason": "", "over_raw": over_raw, "ts_s": ts_s}
            time.sleep(0.05)

    def _tune_worker(self, mid: int, step_deg: float):
        orig = None
        p0_raw = None
        try:
            with self._lock:
                orig = self.get_pid(mid)
            self._tune_set(state="waiting", msg="等待关节静止…")
            p0_raw = self._wait_settled(mid)
            if p0_raw is None:
                self._tune_done("failed", "等待稳定失败（扭矩是否关闭？）", None)
                return

            wlo, whi = self._window_deg(mid)
            lo_deg = wlo + 3.0
            hi_deg = whi - 3.0
            lo_raw = self._deg2raw(mid, lo_deg)[0]
            hi_raw = self._deg2raw(mid, hi_deg)[0]
            if hi_raw - lo_raw < 40:  # 限位窗口太窄，不做步进
                self._tune_done("failed", "机械限位窗口过窄，无法安全步进", None)
                return

            # 朝限位窗口中心方向步进（避开两端硬限位）
            center_raw = (lo_raw + hi_raw) // 2
            sign = 1 if center_raw >= p0_raw else -1
            step_raw = int(step_deg * RAW_PER_DEG)
            target_raw = min(max(p0_raw + sign * step_raw, lo_raw), hi_raw)
            if abs(target_raw - p0_raw) < 25:
                self._tune_done("failed", "当前位置离限位太近，无法安全步进", None)
                return

            p = orig["p"] if orig["p"] > 0 else 60
            d_ = orig["d"] if orig["d"] > 0 else 20
            i = orig["i"]  # I 保持原值不动
            best = {"p": p, "d": d_, "i": i, "cost": None}
            trials = []
            for n in range(1, 5):
                self._tune_set(state="tuning", trial=n, msg=f"第 {n}/4 轮：P={p} D={d_} 步进 {step_deg:.0f}°")
                with self._lock:
                    self._write("P_Coefficient", [mid], [p])
                    self._write("D_Coefficient", [mid], [d_])
                    self._write("I_Coefficient", [mid], [i])
                    self._write("Goal_Position", [mid], [target_raw])
                s = self._sample_step(mid, p0_raw, target_raw)
                if not s["ok"]:
                    self._tune_done("failed", f"第 {n} 轮中止：{s['reason']}", None)
                    return
                over_deg = s["over_raw"] / RAW_PER_DEG
                cost = s["ts_s"] + over_deg * 2.5
                trials.append({"trial": n, "p": p, "d": d_, "ts_s": round(s["ts_s"], 2),
                               "over_deg": round(over_deg, 2), "cost": round(cost, 2)})
                if best["cost"] is None or cost < best["cost"]:
                    best = {"p": p, "d": d_, "i": i, "cost": cost}
                # 已够快且几乎无超调 → 提前收工
                if s["ts_s"] < 1.5 and over_deg < 1.0:
                    break
                # 简单规则更新（保守：每轮小幅度）
                if over_deg > 4.0:          # 超调过大 → 降 P、加 D
                    p = max(5, int(p * 0.75))
                    d_ = min(100, int(d_ * 1.4) + 2)
                elif s["ts_s"] > 4.0:       # 太慢 → 加 P
                    p = min(200, int(p * 1.3) + 1)
                    d_ = min(100, int(d_ * 1.1))
                else:                       # 略偏慢 → 温和加 P
                    p = min(200, int(p * 1.1) + 1)
                    d_ = min(100, int(d_ * 1.05) + 1)
                sign = -sign  # 下一轮反方向步进（避免长期单侧偏置）
                target_raw = min(max(p0_raw + sign * step_raw, lo_raw), hi_raw)

            # 写入最优 P/D/I，目标回写到起点
            self._tune_set(state="tuning", trial=5, msg="写入最优参数并回到起点…")
            with self._lock:
                self._write("P_Coefficient", [mid], [best["p"]])
                self._write("D_Coefficient", [mid], [best["d"]])
                self._write("I_Coefficient", [mid], [best["i"]])
                self._write("Goal_Position", [mid], [p0_raw])
            # 等它回到起点（最多 6s）；期间若扭矩被关闭，则把目标设为当前位置
            t_end = time.time()
            while time.time() - t_end < 6.0:
                with self._lock:
                    try:
                        torque = self._read("Torque_Enable", [mid], num_retry=1)
                        pos = self._read("Present_Position", [mid], num_retry=1)
                        spd = self._read("Present_Speed", [mid], num_retry=1)
                    except ConnectionError:
                        break
                if torque[0] == 0:
                    with self._lock:
                        self._write("Goal_Position", [mid], [pos[0]])
                    break
                if abs(spd[0]) < 4:
                    break
                time.sleep(0.1)
            with self._lock:
                readback = self.get_pid(mid)
            result = {
                "original": {"p": orig["p"], "d": orig["d"], "i": orig["i"]},
                "applied": {"p": best["p"], "d": best["d"], "i": best["i"]},
                "readback": readback,
                "trials": trials,
                "step_deg": step_deg,
            }
            self._tune_done("done", "完成", result)
        except Exception as e:  # noqa: BLE001 - 调参失败不应让线程静默死掉
            # 尽力恢复：写回原始 PID + 目标回当前位置（两步独立，避免一步失败掩盖另一）
            restore_note = ""
            try:
                with self._lock:
                    if orig:
                        self._write("P_Coefficient", [mid], [orig["p"]])
                        self._write("D_Coefficient", [mid], [orig["d"]])
                        self._write("I_Coefficient", [mid], [orig["i"]])
            except Exception:
                restore_note = "（警告：原始 PID 恢复失败，请手动检查）"
            try:
                with self._lock:
                    cur = self._read("Present_Position", [mid], num_retry=1)[0]
                    self._write("Goal_Position", [mid], [cur])
            except Exception:
                restore_note += "（警告：目标回位失败）"
            self._tune_done("failed", f"异常：{e}{restore_note}", None)

    def _tune_done(self, state: str, msg: str, result):
        self._tune_set(state=state, msg=msg, result=result,
                       trial=self._tune.get("trial") if self._tune else 0)

    def get_offset(self, motor_id: int) -> int:
        with self._lock:
            return self._read("Offset", motor_id)

    def set_offset(self, motor_id: int, offset: int) -> dict:
        offset = int(offset)
        if not (0 <= offset <= 4095):
            raise ValueError("offset 必须在 [0, 4095]")
        with self._lock:
            self._write("Offset", [motor_id], [offset])
            readback = self._read("Offset", motor_id)
        return {"ok": True, "readback": readback}

    def home_offset(self, motor_id: int) -> dict:
        """自动回中（setOffsetCurrent）：约 2 秒，且会把 Min/Max_Angle_Limit 重置为 0/4095"""
        with self._lock:
            self._motor.setMotorId(motor_id)
            self._motor.setOffsetCurrent()
            homing = self._read("Offset", motor_id)
        return {"ok": True, "homing_offset": homing, "min_limit": 0, "max_limit": 4095}

    def read_config(self, motor_id: int) -> dict:
        """全控制表读取；个别固件不支持的条目直接省略。"""
        with self._lock:
            config = {}
            for key in macro.SCS_SERIES_CONTROL_TABLE:
                try:
                    config[key] = self._read(key, motor_id)
                except ConnectionError:
                    continue
            return config

    def write_config(self, motor_id: int, name: str, value: int) -> dict:
        if name not in WRITABLE_CONFIG:
            raise ValueError(f"配置项 {name} 不在可写白名单中")
        lo, hi = WRITABLE_CONFIG[name]
        value = int(value)
        if not (lo <= value <= hi):
            raise ValueError(f"{name} 必须在 [{lo}, {hi}]")
        with self._lock:
            self._write(name, [motor_id], [value])
            readback = self._read(name, motor_id)
        return {"ok": True, "name": name, "readback": readback}

    # ---------------- 波特率 ----------------

    def get_baudrate(self) -> dict:
        with self._lock:
            ph = self._motor.port_handler
            port_baud = ph.getBaudRate() if ph is not None else None
            try:
                idx = self._read("Baud_Rate", 1)
            except ConnectionError:
                idx = None
        table = macro.SCS_SERIES_BAUDRATE_TABLE
        return {
            "port_baudrate": port_baud,
            "motor_baud_index": idx,
            "motor_baud_bps": table.get(idx) if idx is not None else None,
            "table": {str(k): v for k, v in table.items()},
        }

    def set_baudrate(self, index: int) -> dict:
        """广播 ID0 写 Baud_Rate+Lock -> 切 host 侧速率 -> 读回校验（失败抛 ConnectionError）"""
        table = macro.SCS_SERIES_BAUDRATE_TABLE
        if index not in table:
            raise ValueError(f"未知波特率索引：{index}（可选：{sorted(table)}）")
        new_baud = table[index]
        with self._lock:
            self._write("Baud_Rate", [0], [index])
            self._write("Lock", [0], [1])
            self._motor.port_handler.setBaudRate(new_baud)
            self._read("Baud_Rate", 1)  # 校验；失败则总线不可达，需恢复旧波特率
        return {"ok": True, "index": index, "port_baudrate": new_baud}


class TelemetryBroadcaster:
    """线程安全 publish -> 每 WS 客户端一个 asyncio.Queue（满则丢最旧）"""

    def __init__(self):
        self._subs: set[asyncio.Queue] = set()
        self._loop = None
        self.latest = None

    def bind(self, loop):
        self._loop = loop

    def subscribe(self) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue(maxsize=8)
        self._subs.add(q)
        return q

    def unsubscribe(self, q):
        self._subs.discard(q)

    def publish(self, snap: dict):
        self.latest = snap
        if self._loop is None or not self._subs:
            return
        try:
            self._loop.call_soon_threadsafe(self._deliver, snap)
        except RuntimeError:  # 事件循环已关闭（关停中）
            pass

    def _deliver(self, snap):
        for q in list(self._subs):
            if q.full():
                try:
                    q.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            q.put_nowait(snap)


class TelemetryThread(threading.Thread):
    """daemon 线程：以 TELEMETRY_HZ 轮询遥测并经 broadcaster 发布"""

    def __init__(self, ctrl: ArmController, bc: TelemetryBroadcaster):
        super().__init__(daemon=True, name="telemetry")
        self._ctrl = ctrl
        self._bc = bc
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        period = 1.0 / TELEMETRY_HZ
        while not self._stop_event.is_set():
            t0 = time.time()
            try:
                self._bc.publish(self._ctrl.read_telemetry())
            except Exception:
                # 总线异常：errors=-1 标记帧，前端显示"总线错误"
                self._bc.publish({"ts": time.time(), "joints": [], "errors": -1})
            self._stop_event.wait(max(0.0, period - (time.time() - t0)))
