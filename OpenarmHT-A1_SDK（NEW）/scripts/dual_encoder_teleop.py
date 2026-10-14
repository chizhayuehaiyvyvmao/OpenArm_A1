"""OpenArm 双编码器主臂遥操：一个 SDK，四个 CAN，每周期统一发送。

运行期间主臂只排队状态请求；从臂用 MIT + 重力补偿。
两边分别保存软件基准，不改硬件零位。所有离线检查均不创建 Robot。
SDK 自身的初始化/析构仍按底层实现执行，析构会广播 brake。
"""
from __future__ import annotations

import copy
import math
import signal
import tempfile
import time
from pathlib import Path

import numpy as np
import yaml

import encoder_teleop_single as single


ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / "robot_param/dual_encoder_teleop.yaml"
DT = single.CONTROL_DT
SIDES = ("left", "right")


class LeaderFault(RuntimeError):
    pass


class FollowerFault(RuntimeError):
    pass


class StopRequest:
    def __init__(self):
        self.requested = False
        self.immediate = False

    def handle(self, signum, _frame):
        if self.requested:
            self.immediate = True
            print("\n[STOP] 再次中断：双臂立即软件停止，不再等待回零。", flush=True)
        else:
            self.requested = True
            print("\n[EXIT] 请求退出：双臂将回零后停止。", flush=True)

    def check(self):
        if self.immediate:
            raise single.ImmediateStop("用户请求立即停止双臂")


def read_yaml(path):
    with Path(path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def load_settings(path=CONFIG):
    path = Path(path)
    settings = read_yaml(path)["arms"]
    if set(settings) != set(SIDES):
        raise ValueError("双臂配置必须恰好包含 left 和 right")
    ports = []
    for side in SIDES:
        item = settings[side]
        for key in ("leader_can", "follower_can"):
            port = item[key]
            if type(port) is not int or port not in (1, 2, 3, 4):
                raise ValueError(f"{side}.{key} 必须为 CAN1~4")
            ports.append(port)
        signs = np.asarray(item["joint_signs"], dtype=float)
        if signs.shape != (7,) or not np.all(np.isin(signs, (-1, 1))):
            raise ValueError(f"{side}.joint_signs 必须是七个 ±1")
        if item["gripper_sign"] not in (-1, 1):
            raise ValueError(f"{side}.gripper_sign 必须是 ±1")
        item["config_path"] = (path.parent / item["follower_config"]).resolve()
        follower = read_yaml(item["config_path"])
        item["motor_path"] = (
            item["config_path"].parent / follower["robot"]["param_file"]
        ).resolve()
        port_config = read_yaml(item["motor_path"])["robot"]["CANboard"]["No_1_CANboard"]["CANport"]["CANport_1"]
        if port_config["serial_id"] != item["follower_can"]:
            raise ValueError(f"{side} 从臂 CAN 口与原电机 YAML 不一致")
    if len(set(ports)) != 4:
        raise ValueError("两个主臂和两个从臂必须使用四个不同的 CAN 口")
    return settings


def channel_name(side, role, motor_id):
    return f"{role}_{side}_{motor_id}"


def build_motor_config(settings):
    """复用当前电机类型，给同 ID 的四组设备分配唯一名称。"""
    result = copy.deepcopy(read_yaml(settings["right"]["motor_path"]))
    root = result["robot"]
    root["robot_name"] = "openarm_dual_encoder_teleop"
    root["motor_cmd_rate_limit_hz"] = 200
    # true 避免 SDK 的 false 析构分支给所有设备发送 reset。
    root["exit_motor_brake_flag"] = True
    board = root["CANboard"]["No_1_CANboard"]
    board["CANport_num"] = 4
    board["CANport"] = {}
    root["CANboard_num"] = 1
    leader_template = read_yaml(single.LEADER_CONFIG)["robot"]["CANboard"]["No_1_CANboard"]["CANport"]["CANport_1"]
    for side in SIDES:
        follower_template = read_yaml(settings[side]["motor_path"])["robot"]["CANboard"]["No_1_CANboard"]["CANport"]["CANport_1"]
        for role, template in (("leader", leader_template), ("follower", follower_template)):
            port = copy.deepcopy(template)
            can = settings[side][f"{role}_can"]
            port["serial_id"] = can
            motors = list(port["motor"].values())
            if port["motor_num"] != 8 or sorted(m["id"] for m in motors) != list(range(1, 9)):
                raise ValueError(f"{side}/{role} 电机配置必须为 ID1~8")
            for motor in motors:
                motor["name"] = channel_name(side, role, motor["id"])
            board["CANport"][f"CANport_{can}"] = port
    board["CANport"] = dict(sorted(board["CANport"].items()))
    return result


class ArmModel:
    """只加载 YAML/URDF，不调用 SDK 构造器。"""
    def __init__(self, item):
        import pinocchio as pin
        self.pin = pin
        config = read_yaml(item["config_path"])
        robot = config["robot"]
        self.lower = np.asarray(robot["joint_limits"]["lower"], dtype=float)
        self.upper = np.asarray(robot["joint_limits"]["upper"], dtype=float)
        self.velocity = np.asarray(robot["velocity_limits"], dtype=float)
        self.torque = np.asarray(robot["max_torque"], dtype=float)
        self.gripper = np.array([robot["gripper_limits"]["lower"], robot["gripper_limits"]["upper"]], dtype=float)
        for value in (self.lower, self.upper, self.velocity, self.torque):
            if value.shape != (7,) or not np.all(np.isfinite(value)):
                raise ValueError("从臂参数必须为七个有限数值")
        if (np.any(self.lower + 2 * single.JOINT_LIMIT_MARGIN >= self.upper)
                or np.any(self.velocity <= 0) or np.any(self.torque <= 0)
                or not np.all(np.isfinite(self.gripper)) or self.gripper[0] >= self.gripper[1]
                or np.any(self.lower > 0) or np.any(self.upper < 0)
                or not self.gripper[0] <= 0 <= self.gripper[1]):
            raise ValueError("从臂限位、回零位置或速度/力矩配置无效")
        self.signs = np.asarray(config["kinematics"]["joint_signs"], dtype=float)
        if self.signs.shape != (7,) or not np.all(np.isin(self.signs, (-1, 1))):
            raise ValueError("URDF 电机坐标符号必须是七个 ±1")
        self.model = pin.buildModelFromUrdf(str((item["config_path"].parent / config["urdf"]["file_path"]).resolve()))
        names = config["kinematics"]["joint_names"]
        if len(names) != 7 or len(set(names)) != 7 or not all(self.model.existJointName(n) for n in names):
            raise ValueError("URDF 关节与从臂 YAML 不匹配")
        joints = [self.model.joints[self.model.getJointId(n)] for n in names]
        if any(j.nq != 1 or j.nv != 1 for j in joints):
            raise ValueError("从臂模型必须使用七个单自由度关节")
        self.q_indices = [j.idx_q for j in joints]
        self.v_indices = [j.idx_v for j in joints]
        a, b = self.lower * self.signs, self.upper * self.signs
        if (np.any(np.minimum(a, b) < self.model.lowerPositionLimit[self.q_indices] - 1e-4)
                or np.any(np.maximum(a, b) > self.model.upperPositionLimit[self.q_indices] + 1e-4)):
            raise ValueError("从臂 YAML 限位经坐标变换后超出 URDF")
        self.model.gravity.linear = np.array([0., 0., -9.81])
        self.data = self.model.createData()

    def gravity(self, q):
        full = self.pin.neutral(self.model)
        full[self.q_indices] = self.signs * q
        gravity = self.pin.computeGeneralizedGravity(self.model, self.data, full)
        return np.clip(self.signs * gravity[self.v_indices], -self.torque, self.torque)


def bind_channels(robot, settings):
    motors = list(robot.get_motors())
    by_name = {motor.get_motor_name(): motor for motor in motors}
    expected = {channel_name(s, r, i) for s in SIDES for r in ("leader", "follower") for i in range(1, 9)}
    if len(motors) != 32 or len(by_name) != 32 or set(by_name) != expected:
        raise RuntimeError("SDK 设备名称/数量与四 CAN、32 个电机配置不一致")
    channels = {}
    for side in SIDES:
        for role in ("leader", "follower"):
            group = [by_name[channel_name(side, role, i)] for i in range(1, 9)]
            if [int(m.get_motor_id()) for m in group] != list(range(1, 9)):
                raise RuntimeError(f"{side}/{role} ID 顺序不正确")
            channels[side, role] = group
    return channels


class Bus:
    def __init__(self, robot, settings, models, stop):
        self.robot, self.settings, self.models, self.stop = robot, settings, models, stop
        self.channels = bind_channels(robot, settings)
        self.last_send = -math.inf

    def flush(self):
        self.stop.check()
        remaining = self.last_send + DT - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        self.stop.check()
        self.robot.motor_send_cmd()
        self.last_send = time.monotonic()

    def read(self, role):
        """先复制本周期反馈，任一侧无效时不发送下一帧运动。"""
        result = {}
        fault_class = LeaderFault if role == "leader" else FollowerFault
        for side in SIDES:
            values = []
            for i, motor in enumerate(self.channels[side, role], 1):
                state = motor.get_current_motor_state()
                pos, vel, torque, timestamp, fault = (
                    float(state.position), float(state.velocity), float(state.torque),
                    float(state.time), int(state.fault),
                )
                can = self.settings[side][f"{role}_can"]
                label = f"{self.settings[side]['label']} {role} CAN{can} ID{i}"
                age = time.time() - timestamp
                if (single.version_tuple(motor) == (0, 0, 0) or fault
                        or not np.all(np.isfinite([pos, vel, torque, timestamp]))
                        or max(abs(pos), abs(vel), abs(torque)) >= single.INVALID_FEEDBACK_ABS
                        or age < -0.01 or age > single.FEEDBACK_STALE_TIMEOUT):
                    raise fault_class(f"{label} 反馈无效/断开: age={age:.3f}s fault=0x{fault:02X}")
                values.append(pos)
            result[side] = np.asarray(values)
        return result

    def query_all(self):
        for group in self.channels.values():
            for motor in group:
                motor.send_state_cmd()
        self.flush()

    def send(self, targets, velocities):
        # 所有计算和校验都在写 SDK 缓存之前完成，避免单侧异常导致半帧发送。
        prepared = {}
        for side in SIDES:
            q = np.asarray(targets[side], dtype=float)
            vel = np.asarray(velocities[side], dtype=float)
            model = self.models[side]
            if q.shape != (8,) or vel.shape != (7,) or not np.all(np.isfinite(np.r_[q, vel])):
                raise FollowerFault(f"{side} 控制向量无效")
            if (np.any(q[:7] < model.lower - 1e-8) or np.any(q[:7] > model.upper + 1e-8)
                    or not model.gripper[0] <= q[7] <= model.gripper[1]
                    or np.any(abs(vel) > model.velocity + 1e-8)):
                raise FollowerFault(f"{side} 控制目标超出从臂配置限位")
            gravity = model.gravity(q[:7])
            if gravity.shape != (7,) or not np.all(np.isfinite(gravity)):
                raise FollowerFault(f"{side} 重力补偿无效")
            prepared[side] = q, vel, gravity
        self.stop.check()
        for side in SIDES:
            for motor in self.channels[side, "leader"]:
                motor.send_state_cmd()
            q, vel, gravity = prepared[side]
            for i, motor in enumerate(self.channels[side, "follower"][:7]):
                motor.pos_vel_tqe_kp_kd(float(q[i]), float(vel[i]), float(gravity[i]), float(single.KP[i]), float(single.KD[i]))
            self.channels[side, "follower"][7].pos_vel_tqe_kp_kd(float(q[7]), 0., 0., single.GRIPPER_KP, single.GRIPPER_KD)
        self.flush()

    def stop_followers(self):
        # 独立清理路径不检查 stop.immediate，第二次 Ctrl+C 仍必须能发送停止。
        for side in SIDES:
            for motor in self.channels[side, "leader"]:
                motor.send_state_cmd()
            for motor in self.channels[side, "follower"]:
                motor.stop()
        remaining = self.last_send + DT - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        self.robot.motor_send_cmd()
        self.last_send = time.monotonic()


def wait_tick(start):
    remaining = DT - (time.monotonic() - start)
    if remaining > 0:
        time.sleep(remaining)


def home_both(bus, label):
    bus.stop.check()
    initial = bus.read("follower")
    duration = single.HOME_MIN_DURATION
    for side in SIDES:
        model = bus.models[side]
        q = initial[side]
        if (np.any(q[:7] < model.lower) or np.any(q[:7] > model.upper)
                or not model.gripper[0] <= q[7] <= model.gripper[1]):
            raise FollowerFault(f"{bus.settings[side]['label']} 当前反馈超限，无法生成回零轨迹")
        duration = max(duration, 1.875 * np.max(abs(q[:7])) / single.HOME_SPEED,
                       1.875 * abs(q[7]) / single.MAX_GRIPPER_VELOCITY)
        print(f"[{label}] {bus.settings[side]['label']} 当前={single.format_vector(q[:7])} 夹爪={q[7]:+.3f}", flush=True)
    print(f"[{label}] 双臂同时回零，用时 {duration:.1f}s", flush=True)
    start = time.monotonic()
    while True:
        tick = time.monotonic()
        bus.stop.check()
        actual = bus.read("follower")
        elapsed = tick - start
        targets, velocities = {}, {}
        for side in SIDES:
            q, v = single.quintic_sample(initial[side], np.zeros(8), duration, elapsed)
            targets[side], velocities[side] = q, v[:7]
        bus.send(targets, velocities)
        if elapsed >= duration:
            errors = {s: float(np.max(abs(actual[s]))) for s in SIDES}
            if max(errors.values()) <= single.HOME_TOLERANCE:
                print(f"[{label}] 双臂和夹爪回零完成: 左={errors['left']:.3f}, 右={errors['right']:.3f} rad", flush=True)
                return
            if elapsed > duration + single.HOME_SETTLE_TIMEOUT:
                raise FollowerFault(f"双臂回零未到位: {errors}")
        wait_tick(tick)


def capture_references(bus):
    print("[ALIGN] 两条主臂自然下垂、夹爪闭合并保持不动，正在同步采集软件基准。", flush=True)
    samples = {s: [] for s in SIDES}
    zero_q, zero_v = {s: np.zeros(8) for s in SIDES}, {s: np.zeros(7) for s in SIDES}
    for _ in range(single.LEADER_REFERENCE_SAMPLES):
        if bus.stop.requested:
            return None
        tick = time.monotonic()
        leaders = bus.read("leader")
        bus.read("follower")
        for side in SIDES:
            samples[side].append(leaders[side])
        bus.send(zero_q, zero_v)  # 采样时两条从臂继续同时保持零位。
        wait_tick(tick)
    references = {}
    for side in SIDES:
        values = np.asarray(samples[side])
        moving = np.where(np.ptp(values, axis=0) > single.LEADER_REFERENCE_MAX_SPAN)[0]
        if moving.size:
            raise LeaderFault(f"{bus.settings[side]['label']} 采集基准时移动: ID{(moving + 1).tolist()}")
        references[side] = np.median(values, axis=0)
        print(f"[ALIGN] {bus.settings[side]['label']} 基准={single.format_vector(references[side])}", flush=True)
    return references


class Mapper:
    def __init__(self, settings, models, references):
        self.settings, self.models = settings, models
        self.references = {s: np.array(references[s], dtype=float, copy=True) for s in SIDES}
        self.previous_leader = {s: self.references[s].copy() for s in SIDES}
        self.previous_target = {s: np.zeros(8) for s in SIDES}
        self.last_sample = None

    def map(self, leaders, now):
        targets, velocities, limited = {}, {}, {}
        sample_dt = DT if self.last_sample is None else max(now - self.last_sample, 1e-4)
        for side in SIDES:
            q = np.asarray(leaders[side], dtype=float)
            if q.shape != (8,) or not np.all(np.isfinite(q)):
                raise LeaderFault(f"{side} 编码器反馈无效")
            counts = np.rint(q * single.ENCODER_COUNTS_PER_TURN / (2 * math.pi))
            if np.any(abs(counts) >= single.SIGNED_COUNT_EDGE):
                raise LeaderFault(f"{side} 编码器接近 16 位计数边界")
            step = abs(q - self.previous_leader[side])
            if np.any(step > single.LEADER_SAMPLE_JUMP_LIMIT):
                raise LeaderFault(f"{side} 编码器反馈跳变: ID{(np.where(step > single.LEADER_SAMPLE_JUMP_LIMIT)[0] + 1).tolist()}")
            signs = np.r_[self.settings[side]["joint_signs"], self.settings[side]["gripper_sign"]]
            raw = signs * (q - self.references[side])
            model = self.models[side]
            target = np.clip(raw, np.r_[model.lower + single.JOINT_LIMIT_MARGIN, model.gripper[0]],
                             np.r_[model.upper - single.JOINT_LIMIT_MARGIN, model.gripper[1]])
            targets[side] = target
            velocities[side] = np.clip((target[:7] - self.previous_target[side][:7]) / sample_dt, -model.velocity, model.velocity)
            limited[side] = np.where(abs(raw - target) > 1e-9)[0] + 1
        # 两侧都有效后才更新状态。
        self.previous_leader = {s: np.array(leaders[s], copy=True) for s in SIDES}
        self.previous_target = {s: targets[s].copy() for s in SIDES}
        self.last_sample = now
        return targets, velocities, limited


def run_teleop(bus, references):
    mapper = Mapper(bus.settings, bus.models, references)
    error_since = {s: None for s in SIDES}
    last_report = time.monotonic()
    cycles = 0
    print("[RUN] 左右主臂分别 1:1 控制左右从臂及夹爪，MIT 模式，单周期统一发送。", flush=True)
    print("[RUN] Ctrl+C 双臂回零退出；再次 Ctrl+C 立即软件停止。", flush=True)
    while not bus.stop.requested:
        tick = time.monotonic()
        leaders = bus.read("leader")
        actual = bus.read("follower")
        targets, velocities, limited = mapper.map(leaders, tick)
        errors = {s: float(np.max(abs(targets[s][:7] - actual[s][:7]))) for s in SIDES}
        for side in SIDES:
            if errors[side] > single.FOLLOW_ERROR_LIMIT:
                if error_since[side] is None:
                    error_since[side] = tick
                if tick - error_since[side] >= single.FOLLOW_ERROR_CYCLES * DT:
                    raise FollowerFault(f"{bus.settings[side]['label']} 持续跟随误差过大: {errors[side]:.3f} rad")
            else:
                error_since[side] = None
        bus.send(targets, velocities)
        cycles += 1
        if tick - last_report >= single.PRINT_INTERVAL:
            print(f"[TELEOP] {cycles / (tick - last_report):.0f}Hz " + " | ".join(
                f"{bus.settings[s]['label']} err={errors[s]:.3f}rad 夹爪={targets[s][7]:+.2f}/{actual[s][7]:+.2f} 限幅ID={limited[s].tolist()}"
                for s in SIDES), flush=True)
            last_report, cycles = tick, 0
        wait_tick(tick)


def execute(bus):
    """便于用假 SDK 检查生命周期；任一从臂故障时直接停止双臂。"""
    motion_started = False
    can_home = True
    exit_code = 0
    try:
        for _ in range(8):
            if bus.stop.requested:
                return 0
            bus.query_all()
            time.sleep(0.03)
        bus.read("leader")
        bus.read("follower")
        if bus.stop.requested:
            return 0
        print("[OK] 两个主臂、两个从臂及四个夹爪的 32 个反馈全部有效。", flush=True)
        motion_started = True
        home_both(bus, "START")
        if not bus.stop.requested:
            references = capture_references(bus)
            if references is not None:
                run_teleop(bus, references)
    except single.ImmediateStop as error:
        can_home, exit_code = False, 130
        print(f"[STOP] {error}", flush=True)
    except FollowerFault as error:
        can_home, exit_code = False, 1
        print(f"[ERROR] {error}；双臂停止，不执行反馈不可靠的回零。", flush=True)
    except Exception as error:
        exit_code = 1
        print(f"[ERROR] {error}", flush=True)
    finally:
        if motion_started and can_home and not bus.stop.immediate:
            try:
                # 主臂断开仍可退出回零；每帧继续要求两条从臂反馈有效。
                home_both(bus, "EXIT")
            except Exception as error:
                exit_code = 1
                print(f"[WARN] 退出回零中止: {error}", flush=True)
        try:
            bus.stop_followers()
            print("[STOP] 已发送左右从臂和夹爪软件停止命令。", flush=True)
        except Exception as error:
            exit_code = 1
            print(f"[ERROR] 双臂停止发送失败: {error}", flush=True)
    return exit_code


def main():
    settings = load_settings()
    models = {side: ArmModel(settings[side]) for side in SIDES}
    motor_config = build_motor_config(settings)
    print("=" * 72)
    print("OpenArm 双臂编码器遥操：7 关节 + 夹爪，各自独立映射")
    for side in SIDES:
        item = settings[side]
        print(f"{item['label']}: CAN{item['leader_can']} 主臂 -> CAN{item['follower_can']} 从臂；方向={item['joint_signs']}，夹爪={item['gripper_sign']:+d}")
    print("启动双臂回零，然后采集两条主臂当前姿态的软件基准；不写硬件零位。")
    print("请让两条主臂自然下垂、夹爪闭合并保持不动，清空双臂工作区。")
    print("=" * 72, flush=True)
    if len(list(Path('/dev').glob('ttyACM*'))) < 4:
        raise RuntimeError("至少需要四个 /dev/ttyACM* 串口节点，未进入原生 SDK")
    stop = StopRequest()
    previous = {sig: signal.signal(sig, stop.handle) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        with tempfile.TemporaryDirectory(prefix="openarm_dual_teleop_") as directory:
            config_path = Path(directory) / "motors.yaml"
            config_path.write_text(yaml.safe_dump(motor_config, allow_unicode=True, sort_keys=False), encoding="utf-8")
            robot = single.htr.Robot(str(config_path))
            try:
                bus = Bus(robot, settings, models, stop)
            except Exception:
                # 绑定失败时不运动；只在可识别的从臂组排队停止。
                for motor in robot.get_motors():
                    if motor.get_motor_name().startswith("follower_"):
                        motor.stop()
                    else:
                        motor.send_state_cmd()
                robot.motor_send_cmd()
                raise
            return execute(bus)
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
