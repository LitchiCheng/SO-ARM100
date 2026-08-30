"""SO-ARM100 Web 控制台：FastAPI 服务入口。

启动:  uv run python -m webapp.server
端口:  默认 8765（本机 8000 被 vllm 占用），环境变量 SOARM_PORT 可覆盖。
"""

import asyncio
import os
import time
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .arm import (
    PRESET_SPEED,
    ArmController,
    LockedError,
    PoseStore,
    TelemetryBroadcaster,
    TelemetryThread,
)
from .arm import JOINT_NAMES  # noqa: F401  (re-exported for convenience)

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")

ctrl = ArmController()
broadcaster = TelemetryBroadcaster()
poses = PoseStore(os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.json"))


@asynccontextmanager
async def lifespan(app: FastAPI):
    loop = asyncio.get_running_loop()
    broadcaster.bind(loop)
    try:
        ctrl.connect()
        print(f"[webapp] connected to {ctrl.port}")
    except Exception as e:  # noqa: BLE001 - 允许 unconnected 模式启动
        print(f"[webapp] WARNING: cannot open {ctrl.port} ({e}); running unconnected")
    thread = TelemetryThread(ctrl, broadcaster)
    thread.start()
    yield
    thread.stop()
    thread.join(timeout=2)
    if ctrl.is_connected:
        ctrl.disconnect()


app = FastAPI(title="SO-ARM100", lifespan=lifespan)


# ---------------- 请求体模型 ----------------

class ArmedBody(BaseModel):
    armed: bool


class PositionBody(BaseModel):
    deg: float


class TorqueBody(BaseModel):
    mode: int
    confirm: bool = False


class SpeedBody(BaseModel):
    speed: int
    motors: list[int] | None = None


class PresetBody(BaseModel):
    name: str
    confirm: bool = False


class PIDBody(BaseModel):
    p: int
    i: int
    d: int
    confirm: bool = False


class OffsetBody(BaseModel):
    offset: int
    confirm: bool = False


class ConfirmBody(BaseModel):
    confirm: bool = False


class ConfigWriteBody(BaseModel):
    name: str
    value: int
    confirm: bool = False


class BaudBody(BaseModel):
    index: int
    confirm: bool = False


class PoseSaveBody(BaseModel):
    name: str


class PoseDeleteBody(BaseModel):
    name: str


class PoseApplyBody(BaseModel):
    name: str
    confirm: bool = False
    speed: int = PRESET_SPEED


class TuneBody(BaseModel):
    confirm: bool = False
    step_deg: float = 10.0


# ---------------- 辅助 ----------------

def _require_confirm(confirm: bool, what: str):
    if not confirm:
        raise HTTPException(status_code=400, detail=f"该操作（{what}）需要 confirm: true")


def _map_errors(func, *args, **kwargs):
    try:
        return func(*args, **kwargs)
    except LockedError as e:
        raise HTTPException(status_code=423, detail=str(e))
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except ConnectionError as e:
        raise HTTPException(status_code=502, detail=str(e))


def _check_mid(mid: int):
    if not (1 <= mid <= 6):
        raise HTTPException(status_code=404, detail=f"电机 ID 必须在 1~6，收到 {mid}")


# ---------------- 状态 / 遥测 ----------------

@app.get("/api/status")
def status():
    return ctrl.status()


@app.get("/api/limits")
def limits():
    """机械限位（模块常量）+ 固件 Min/Max_Angle_Limit 实时换算值（mujoco 零位系）"""
    return ctrl.get_limits()


class ReconnectBody(BaseModel):
    port: str | None = None  # 可选：USB 重插后设备节点被重新分配时切换，如 "/dev/ttyACM1"


@app.post("/api/reconnect")
def reconnect(body: ReconnectBody = None):
    """USB 插拔后重新打开串口并验证总线可达（不动作、无确认）"""
    return _map_errors(ctrl.reconnect, body.port if body else None)


@app.get("/api/telemetry/latest")
def telemetry_latest():
    return broadcaster.latest or {}


@app.post("/api/arm")
def arm(body: ArmedBody):
    return {"armed": ctrl.set_armed(body.armed)}


# ---------------- 运动 / 安全 ----------------

@app.post("/api/estop")
def estop():
    return _map_errors(ctrl.estop)


@app.post("/api/joints/{mid}/position")
def joint_position(mid: int, body: PositionBody):
    _check_mid(mid)
    return _map_errors(ctrl.set_position, mid, body.deg)


@app.post("/api/torque/all")
def torque_all(body: TorqueBody):
    if body.mode == 0:
        _require_confirm(body.confirm, "全部关闭扭矩（手臂将自由下垂）")
    return _map_errors(ctrl.set_torque, body.mode)


@app.post("/api/joints/{mid}/torque")
def joint_torque(mid: int, body: TorqueBody):
    _check_mid(mid)
    if body.mode == 0:
        _require_confirm(body.confirm, "关闭该关节扭矩")
    return _map_errors(ctrl.set_torque, body.mode, [mid])


@app.post("/api/speed")
def speed(body: SpeedBody):
    return _map_errors(ctrl.set_speed, body.speed, body.motors)


@app.post("/api/presets")
def presets(body: PresetBody):
    _require_confirm(body.confirm, "预设运动")
    return _map_errors(ctrl.apply_preset, body.name)


# ---------------- 点位（保存 / 应用） ----------------

def _latest_pose_values():
    """取最近遥测帧的 6 关节固件位置（pos_raw，与零位约定无关）；过旧/缺失时报 409"""
    snap = broadcaster.latest
    if not snap or not snap.get("joints"):
        raise HTTPException(status_code=409, detail="尚无遥测数据（服务是否已连接？）")
    if time.time() - snap.get("ts", 0) > 3.0:
        raise HTTPException(status_code=409, detail="遥测数据过旧（>3s），请稍后重试")
    vals = [j["pos_raw"] for j in snap["joints"]]
    if any(v is None for v in vals):
        raise HTTPException(status_code=409, detail="遥测不完整（有关节读数缺失），无法保存点位")
    return vals


@app.get("/api/poses")
def list_poses():
    """点位表：{name: {"raw": [固件位置], "deg": [当前显示约定的角度]}}"""
    d = poses.list()
    return {
        name: {"raw": vals,
               "deg": [ctrl._p2deg(i + 1, v) for i, v in enumerate(vals)]}
        for name, vals in d.items()
    }


@app.post("/api/poses")
def save_pose(body: PoseSaveBody):
    vals = _latest_pose_values()
    return _map_errors(poses.save, body.name, vals)


@app.post("/api/poses/delete")
def delete_pose(body: PoseDeleteBody):
    _map_errors(poses.delete, body.name)
    return {"ok": True}


@app.post("/api/poses/apply")
def apply_pose(body: PoseApplyBody):
    _require_confirm(body.confirm, "应用点位（全部 6 关节将运动）")
    d = poses.list()
    if body.name not in d:
        raise HTTPException(status_code=404, detail=f"点位 {body.name!r} 不存在")
    return _map_errors(ctrl.apply_pose, d[body.name], body.speed)  # raw 值，坐标系无关


# ---------------- 配置 ----------------

@app.get("/api/joints/{mid}/pid")
def get_pid(mid: int):
    _check_mid(mid)
    return _map_errors(ctrl.get_pid, mid)


@app.post("/api/joints/{mid}/pid")
def set_pid(mid: int, body: PIDBody):
    _check_mid(mid)
    _require_confirm(body.confirm, "写入 PID")
    return _map_errors(ctrl.set_pid, mid, body.p, body.i, body.d)


@app.post("/api/joints/{mid}/pid/tune")
def start_pid_tune(mid: int, body: TuneBody = None):
    _check_mid(mid)
    _require_confirm(body.confirm if body else False, "PID 自动调参（关节将往复小幅运动约 1~2 分钟）")
    return _map_errors(ctrl.start_pid_tune, mid, body.step_deg if body else 10.0)


@app.get("/api/joints/{mid}/pid/tune")
def tune_status(mid: int):
    _check_mid(mid)
    st = ctrl.tune_status()
    if st is None or st["mid"] != mid:
        return {"mid": mid, "state": "idle", "msg": "", "trial": 0, "result": None}
    return st


@app.get("/api/joints/{mid}/offset")
def get_offset(mid: int):
    _check_mid(mid)
    return {"offset": _map_errors(ctrl.get_offset, mid)}


@app.post("/api/joints/{mid}/offset")
def set_offset(mid: int, body: OffsetBody):
    _check_mid(mid)
    _require_confirm(body.confirm, "写入 Offset")
    return _map_errors(ctrl.set_offset, mid, body.offset)


@app.post("/api/joints/{mid}/offset/home")
def home_offset(mid: int, body: ConfirmBody):
    _check_mid(mid)
    _require_confirm(body.confirm, "自动回中（约 2 秒，且会重置角度限位为 0/4095）")
    return _map_errors(ctrl.home_offset, mid)


@app.post("/api/joints/{mid}/zero")
def zero_joint(mid: int, body: ConfirmBody):
    """单关节标零：把当前位置设为该关节的新 0°（不产生运动）"""
    _check_mid(mid)
    _require_confirm(body.confirm, "标零该关节（当前位置将变为 0°，且固件限位重置为全范围）")
    return _map_errors(ctrl.zero_joint, mid)


@app.get("/api/joints/{mid}/config")
def read_config(mid: int):
    _check_mid(mid)
    return _map_errors(ctrl.read_config, mid)


@app.post("/api/joints/{mid}/config")
def write_config(mid: int, body: ConfigWriteBody):
    _check_mid(mid)
    _require_confirm(body.confirm, f"写入配置 {body.name}")
    return _map_errors(ctrl.write_config, mid, body.name, body.value)


@app.get("/api/baudrate")
def get_baudrate():
    return _map_errors(ctrl.get_baudrate)


@app.post("/api/baudrate")
def set_baudrate(body: BaudBody):
    _require_confirm(body.confirm, "切换波特率（通信将短暂中断，失败需恢复旧波特率）")
    return _map_errors(ctrl.set_baudrate, body.index)


# ---------------- WebSocket 遥测（只读） ----------------

@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket):
    await ws.accept()
    q = broadcaster.subscribe()
    try:
        while True:
            snap = await q.get()
            await ws.send_json({"type": "telemetry", **snap})
    except Exception:  # noqa: BLE001 - 连接关闭/发送失败均退出
        pass
    finally:
        broadcaster.unsubscribe(q)


# 静态资源最后挂载，保证 /api、/ws 优先匹配
app.mount("/", StaticFiles(directory=STATIC_DIR, html=True), name="static")


def main():
    port = int(os.environ.get("SOARM_PORT", "8765"))
    print(f"[webapp] serving on http://0.0.0.0:{port}")
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


if __name__ == "__main__":
    main()
