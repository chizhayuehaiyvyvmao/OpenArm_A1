#!/usr/bin/env python3
"""CAN3 同构编码器主臂遥操 CAN2 OpenArm 右臂。

参考 Panthera 的 5_teleop_control.py：Leader 与 Follower 使用各自的 CAN 口和
SDK 对象。这里的 Leader 是纯编码器主臂，遥操期间只请求状态，
不发送置零、位置、速度或力矩命令。Follower 使用 Openarm 类
的 MIT 模式跟随。

连接约定：
  CAN3：编码器主臂，ID 1~7 为关节，ID 8 为夹爪
  CAN2：OpenArm 右从臂，ID 1~7 为关节，ID 8 为夹爪

本程序不修改编码器硬件零位。右臂回零后，自动把主臂当前的
自然下垂姿态记为本次遥操的软件基准零位，再按相对变化 1:1 跟随。
首次 Ctrl+C、
主臂断开或程序异常会先回零再停止；
连续第二次 Ctrl+C 会立即停止。
"""

from __future__ import annotations

import math
import signal
import time
from pathlib import Path
from typing import List, Sequence, Tuple

import numpy as np
import yaml

import hightorque_robot as htr
from Lib import Openarm


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
LEADER_CONFIG = (
    PROJECT_DIR
    / "robot_param"
    / "motor_param"
    / "7dof_openarm_encoder_leader_can3.yaml"
)
FOLLOWER_CONFIG = PROJECT_DIR / "robot_param" / "Openarm_right.yaml"

JOINT_COUNT = 7
CONTROL_DT = 0.0052
PRINT_INTERVAL = 0.20
LEADER_READ_SETTLE = 0.0052

# 实机逐轴确认：J1~J7 编码器方向均与右从臂电机方向相反。
# J3/J5 为旋转轴，同样需要取反；幅度为 1:1。
JOINT_SIGNS = -np.ones(JOINT_COUNT, dtype=float)
JOINT_SCALES = np.ones(JOINT_COUNT, dtype=float)
# 主夹爪闭合时的编码器变化方向与右从臂夹爪电机相反。
GRIPPER_SIGN = -1.0
GRIPPER_SCALE = 1.0

# 使用项目轨迹控制程序已使用的快速 MIT 增益。
KP = np.array([30.0, 50.0, 60.0, 25.0, 15.0, 10.0, 10.0])
KD = np.array([3.0, 5.0, 6.0, 2.5, 1.5, 1.0, 1.0])
GRIPPER_KP = 5.0
GRIPPER_KD = 0.5

HOME_MIN_DURATION = 3.0
HOME_SPEED = 0.20
HOME_TOLERANCE = 0.06
HOME_SETTLE_TIMEOUT = 3.0
# 这些速度只用于启动/退出回零；遥操阶段直接跟随主臂目标。
MAX_GRIPPER_VELOCITY = 0.50
JOINT_LIMIT_MARGIN = 0.05
GRIPPER_LIMIT_MARGIN = 0.0
LEADER_SAMPLE_JUMP_LIMIT = 0.25
FEEDBACK_STALE_TIMEOUT = 0.25
FOLLOW_ERROR_LIMIT = 0.35
FOLLOW_ERROR_CYCLES = 50
INVALID_FEEDBACK_ABS = 900.0
ENCODER_COUNTS_PER_TURN = 10000
SIGNED_COUNT_EDGE = 32760
LEADER_REFERENCE_SAMPLES = 20
LEADER_REFERENCE_MAX_SPAN = 0.05

shutdown_requested = False
force_stop_requested = False
shutdown_signal_count = 0


class ImmediateStop(RuntimeError):
    pass


def format_vector(values: Sequence[float], digits: int = 3) -> str:
    return "[" + ", ".join(f"{float(value): .{digits}f}" for value in values) + "]"


def version_tuple(motor) -> Tuple[int, int, int]:
    version = motor.get_version()
    return int(version.major), int(version.minor), int(version.patch)


def feedback_error(label: str, motor, require_fresh: bool = True) -> str | None:
    state = motor.get_current_motor_state()
    values = np.array(
        [float(state.position), float(state.velocity), float(state.torque)]
    )
    if version_tuple(motor) == (0, 0, 0):
        return f"{label} 没有版本反馈"
    if not np.all(np.isfinite(values)) or np.any(np.abs(values) >= INVALID_FEEDBACK_ABS):
        return f"{label} 反馈无效: {values.tolist()}"
    if int(state.fault) != 0:
        return f"{label} fault=0x{int(state.fault):02X}"
    if require_fresh:
        timestamp = float(state.time)
        age = time.time() - timestamp if timestamp > 0.0 else math.inf
        if timestamp <= 0.0 or age < 0.0 or age > FEEDBACK_STALE_TIMEOUT:
            return f"{label} 反馈超时: age={age:.3f}s"
    return None


def validate_feedback(named_motors, require_fresh: bool = True) -> None:
    errors = [
        error
        for label, motor in named_motors
        if (error := feedback_error(label, motor, require_fresh)) is not None
    ]
    if errors:
        raise RuntimeError("；".join(errors))


class EncoderLeader:
    """CAN3 编码器主臂，只调用状态请求接口。"""

    def __init__(self, config_path: Path):
        self.robot = htr.Robot(str(config_path))
        self.motors = sorted(
            self.robot.get_motors(), key=lambda motor: int(motor.get_motor_id())
        )
        ids = [int(motor.get_motor_id()) for motor in self.motors]
        if ids != list(range(1, JOINT_COUNT + 2)):
            raise RuntimeError(f"CAN3 主臂应为 ID 1~8，实际为 {ids}")
        self.joint_motors = self.motors[:JOINT_COUNT]
        self.gripper_motor = self.motors[JOINT_COUNT]

        for _ in range(8):
            self.request_state()
            time.sleep(0.03)
        self.validate()

    def request_state(self) -> None:
        # SDK v4 的 send_get_motor_state_cmd() 内部已经立即发送。
        # 不再额外调用 motor_send_cmd()，否则同一请求会连发两次并触发 200 Hz 限速。
        self.robot.send_get_motor_state_cmd()

    def validate(self) -> None:
        validate_feedback(
            [(f"CAN3-J{index}", motor) for index, motor in enumerate(self.motors, 1)]
        )

    def read(self) -> Tuple[np.ndarray, float]:
        self.request_state()
        # 与随后 CAN2 的控制发送至少间隔 5 ms，避免触发 SDK 200 Hz 限速。
        time.sleep(LEADER_READ_SETTLE)
        self.validate()
        joints = np.array(
            [
                float(motor.get_current_motor_state().position)
                for motor in self.joint_motors
            ],
            dtype=float,
        )
        gripper = float(self.gripper_motor.get_current_motor_state().position)
        return joints, gripper

def follower_named_motors(robot: Openarm):
    motors = list(robot.Motors)
    if robot.motor_count != JOINT_COUNT or len(motors) != JOINT_COUNT + 1:
        raise RuntimeError(
            f"CAN2 右臂应为 7 个关节加 1 个夹爪，实际 motor_count={robot.motor_count}, "
            f"总电机数={len(motors)}"
        )
    return [(f"CAN2-J{i + 1}", motors[i]) for i in range(JOINT_COUNT)] + [
        ("CAN2-Gripper", motors[-1])
    ]


def refresh_follower(robot: Openarm, settle: float = 0.02) -> None:
    # SDK v4 状态请求接口内部已经发送。
    robot.send_get_motor_state_cmd()
    if settle > 0.0:
        time.sleep(settle)


def validate_follower(robot: Openarm) -> None:
    validate_feedback(follower_named_motors(robot))


def raw_joint_target_from_leader(
    leader_q: np.ndarray,
    leader_zero: np.ndarray,
) -> np.ndarray:
    if not np.all(np.isfinite(leader_q)) or not np.all(np.isfinite(leader_zero)):
        raise RuntimeError("CAN3 主臂关节反馈包含非有限数值")
    counts = np.rint(
        leader_q * ENCODER_COUNTS_PER_TURN / (2.0 * math.pi)
    ).astype(int)
    edge_indices = np.where(np.abs(counts) >= SIGNED_COUNT_EDGE)[0]
    if edge_indices.size:
        raise RuntimeError(
            f"CAN3 编码器接近 16 位计数边界: J{(edge_indices + 1).tolist()}, "
            f"count={counts[edge_indices].tolist()}"
        )

    return JOINT_SIGNS * JOINT_SCALES * (leader_q - leader_zero)


def clamp_joint_target(
    raw_target: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> np.ndarray:
    safe_lower = lower + JOINT_LIMIT_MARGIN
    safe_upper = upper - JOINT_LIMIT_MARGIN
    if np.any(safe_lower >= safe_upper):
        raise RuntimeError("右臂关节安全限位配置无效")
    return np.clip(raw_target, safe_lower, safe_upper)


def joint_target_from_leader(
    leader_q: np.ndarray,
    leader_zero: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> np.ndarray:
    raw_target = raw_joint_target_from_leader(leader_q, leader_zero)
    # 主臂可以超过从臂的机械行程；对应从臂关节在安全边界保持。
    return clamp_joint_target(raw_target, lower, upper)


def gripper_target_from_leader(
    leader_gripper: float,
    leader_gripper_zero: float,
    gripper_lower: float,
    gripper_upper: float,
) -> float:
    target = GRIPPER_SIGN * GRIPPER_SCALE * (
        float(leader_gripper) - float(leader_gripper_zero)
    )
    safe_lower = float(gripper_lower) + GRIPPER_LIMIT_MARGIN
    safe_upper = float(gripper_upper) - GRIPPER_LIMIT_MARGIN
    if not math.isfinite(target):
        raise RuntimeError(f"主臂夹爪目标不是有限数值: {target}")
    # 主夹爪允许超过从夹爪机械行程；从夹爪到边界后保持，不中断七关节遥操。
    return float(np.clip(target, safe_lower, safe_upper))


def command_follower_mit(
    robot: Openarm,
    q_command: np.ndarray,
    v_command: np.ndarray,
    gripper_command: float,
) -> None:
    gravity = np.asarray(robot.get_Gravity(q_command), dtype=float)
    gravity = np.clip(
        gravity,
        -np.asarray(robot.max_torque, dtype=float),
        np.asarray(robot.max_torque, dtype=float),
    )
    gripper_ok = robot.gripper_control_MIT(
        float(gripper_command),
        0.0,
        0.0,
        GRIPPER_KP,
        GRIPPER_KD,
        send=False,
    )
    arm_ok = robot.pos_vel_tqe_kp_kd(
        q_command.tolist(),
        v_command.tolist(),
        gravity.tolist(),
        KP.tolist(),
        KD.tolist(),
    )
    if gripper_ok is False or arm_ok is False:
        raise RuntimeError("CAN2 右臂拒绝 MIT 控制命令")


def quintic_sample(
    start: np.ndarray,
    target: np.ndarray,
    duration: float,
    elapsed: float,
):
    ratio = float(np.clip(elapsed / duration, 0.0, 1.0))
    blend = 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5
    blend_rate = (
        30.0 * ratio**2 - 60.0 * ratio**3 + 30.0 * ratio**4
    ) / duration
    position = (1.0 - blend) * start + blend * target
    velocity = blend_rate * (target - start)
    return position, velocity


def home_follower(robot: Openarm, gripper_target: float, label: str) -> bool:
    """使用 MIT 五次轨迹让七关节回零，并平滑对齐夹爪目标。"""
    refresh_follower(robot, settle=0.05)
    validate_follower(robot)
    q_start = np.asarray(robot.get_current_pos(), dtype=float)
    gripper_start = float(robot.get_current_pos_gripper())
    q_target = np.zeros(JOINT_COUNT, dtype=float)
    # 五次时间标度的峰值速度为 1.875 * distance / duration。
    duration = max(
        HOME_MIN_DURATION,
        1.875 * float(np.max(np.abs(q_start))) / HOME_SPEED,
        1.875 * abs(gripper_target - gripper_start) / MAX_GRIPPER_VELOCITY,
    )
    print(
        f"\n[{label}] 右臂回零 {format_vector(q_start)} -> "
        f"{format_vector(q_target)}，夹爪 {gripper_start:+.3f} -> "
        f"{gripper_target:+.3f} rad，用时 {duration:.1f}s"
    )

    start_time = time.monotonic()
    next_cycle = start_time
    while True:
        if force_stop_requested:
            raise ImmediateStop("用户请求立即停止")
        elapsed = time.monotonic() - start_time
        q_des, v_des = quintic_sample(q_start, q_target, duration, elapsed)
        gripper_des, _ = quintic_sample(
            np.array([gripper_start]),
            np.array([gripper_target]),
            duration,
            elapsed,
        )
        command_follower_mit(robot, q_des, v_des, float(gripper_des[0]))
        if elapsed >= duration:
            break
        next_cycle += CONTROL_DT
        remaining = next_cycle - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            next_cycle = time.monotonic()

    # 轨迹最后一帧与稳定保持第一帧之间留出 SDK 发送间隔。
    time.sleep(CONTROL_DT)
    deadline = time.monotonic() + HOME_SETTLE_TIMEOUT
    last_error = math.inf
    zero = np.zeros(JOINT_COUNT, dtype=float)
    while time.monotonic() < deadline:
        if force_stop_requested:
            raise ImmediateStop("用户请求立即停止")
        command_follower_mit(robot, zero, zero, gripper_target)
        q_actual = np.asarray(robot.get_current_pos(), dtype=float)
        last_error = float(np.max(np.abs(q_actual)))
        if last_error <= HOME_TOLERANCE:
            print(f"[{label}] 右臂回零完成，最大误差 {last_error:.3f} rad")
            return True
        time.sleep(CONTROL_DT)

    print(f"[{label}] 右臂回零未到位，最大误差 {last_error:.3f} rad")
    return False


def run_teleop(
    leader: EncoderLeader,
    follower: Openarm,
    leader_joint_zero: np.ndarray,
    leader_gripper_zero: float,
    gripper_command_ref: List[float],
) -> None:
    lower = np.asarray(follower.joint_limits["lower"], dtype=float)
    upper = np.asarray(follower.joint_limits["upper"], dtype=float)

    gripper_lower = float(follower.gripper_limits["lower"])
    gripper_upper = float(follower.gripper_limits["upper"])

    velocity_limits = np.asarray(follower.velocity_limits, dtype=float)
    leader_previous, leader_gripper = leader.read()
    q_target_previous = joint_target_from_leader(
        leader_previous, leader_joint_zero, lower, upper
    )
    gripper_target_from_leader(
        leader_gripper,
        leader_gripper_zero,
        gripper_lower,
        gripper_upper,
    )
    q_command = q_target_previous.copy()
    v_command = np.zeros(JOINT_COUNT, dtype=float)
    gripper_command = 0.0
    target_sample_time = time.monotonic()
    follow_error_cycles = 0
    loop_index = 0
    last_print = 0.0
    last_joint_limit_print = 0.0
    last_gripper_limit_print = 0.0
    rate_window_start = time.monotonic()
    rate_window_cycles = 0
    measured_rate_hz = 0.0
    next_cycle = time.monotonic()

    print("\n[RUN] CAN3 编码器主臂开始 1:1 遥操 CAN2 右臂和夹爪")
    print("[RUN] 最大响应直跟模式：目标 192 Hz，无软件位置渐变")
    print("[RUN] 第一次 Ctrl+C 会回零并退出")

    while not shutdown_requested:
        loop_index += 1
        rate_window_cycles += 1
        leader_q, leader_gripper = leader.read()
        step = np.abs(leader_q - leader_previous)
        jumped = np.where(step > LEADER_SAMPLE_JUMP_LIMIT)[0]
        if jumped.size:
            raise RuntimeError(
                f"CAN3 编码器反馈跳变: J{(jumped + 1).tolist()}, "
                f"delta={format_vector(step[jumped])}"
            )
        leader_previous = leader_q
        q_raw_target = raw_joint_target_from_leader(leader_q, leader_joint_zero)
        q_target = clamp_joint_target(q_raw_target, lower, upper)
        limited_joints = np.where(np.abs(q_raw_target - q_target) > 1e-9)[0]
        if (
            limited_joints.size
            and time.monotonic() - last_joint_limit_print >= 1.0
        ):
            details = ", ".join(
                f"J{i + 1} {q_raw_target[i]:+.3f}->{q_target[i]:+.3f}"
                for i in limited_joints
            )
            print(
                f"\n[JOINT-LIMIT] {details} rad；该关节保持在安全边界，"
                "其他关节继续遥操",
                flush=True,
            )
            last_joint_limit_print = time.monotonic()
        sample_time = time.monotonic()
        sample_dt = max(sample_time - target_sample_time, 1e-4)
        v_command = np.clip(
            (q_target - q_target_previous) / sample_dt,
            -velocity_limits,
            velocity_limits,
        )
        q_command = q_target.copy()
        q_target_previous = q_target.copy()
        target_sample_time = sample_time
        gripper_target = gripper_target_from_leader(
            leader_gripper,
            leader_gripper_zero,
            gripper_lower,
            gripper_upper,
        )
        gripper_raw_target = GRIPPER_SIGN * GRIPPER_SCALE * (
            float(leader_gripper) - float(leader_gripper_zero)
        )
        gripper_limited = abs(gripper_raw_target - gripper_target) > 1e-9
        if (
            gripper_limited
            and time.monotonic() - last_gripper_limit_print >= 1.0
        ):
            print(
                "\n[GRIP-LIMIT] 主夹爪目标 "
                f"{gripper_raw_target:+.3f} rad 已限制为 {gripper_target:+.3f} rad；"
                "七关节继续遥操",
                flush=True,
            )
            last_gripper_limit_print = time.monotonic()
        # 遥操直接下发当前主臂目标；电机 MIT 环负责实机跟随。
        gripper_command = float(gripper_target)
        gripper_command_ref[0] = gripper_command
        command_follower_mit(follower, q_command, v_command, gripper_command)

        if loop_index % 10 == 0:
            validate_follower(follower)

        q_actual = np.asarray(follower.get_current_pos(), dtype=float)
        gripper_actual = float(follower.get_current_pos_gripper())
        max_error = float(np.max(np.abs(q_command - q_actual)))
        if max_error > FOLLOW_ERROR_LIMIT:
            follow_error_cycles += 1
        else:
            follow_error_cycles = 0
        if follow_error_cycles >= FOLLOW_ERROR_CYCLES:
            raise RuntimeError(
                f"CAN2 右臂连续跟随误差过大: {max_error:.3f} rad"
            )

        now = time.monotonic()
        rate_window_elapsed = now - rate_window_start
        if rate_window_elapsed >= 1.0:
            measured_rate_hz = rate_window_cycles / rate_window_elapsed
            rate_window_start = now
            rate_window_cycles = 0
        if now - last_print >= PRINT_INTERVAL:
            print(
                "\r[TELEOP] rate={:.0f}Hz leader={} target={} actual={} max_err={:.3f}rad "
                "grip_enc={:+.2f} delta={:+.2f} target={:+.2f} "
                "cmd={:+.2f} actual={:+.2f} ".format(
                    measured_rate_hz,
                    format_vector(leader_q, 2),
                    format_vector(q_target, 2),
                    format_vector(q_actual, 2),
                    max_error,
                    leader_gripper,
                    leader_gripper - leader_gripper_zero,
                    gripper_target,
                    gripper_command,
                    gripper_actual,
                ),
                end="",
                flush=True,
            )
            last_print = now

        next_cycle += CONTROL_DT
        remaining = next_cycle - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            next_cycle = time.monotonic()
    print()


def handle_shutdown_signal(signum, _frame) -> None:
    global shutdown_requested, force_stop_requested, shutdown_signal_count
    shutdown_signal_count += 1
    if shutdown_signal_count == 1:
        shutdown_requested = True
        print(f"\n收到信号 {signum}：右臂将先回零，再停止。", flush=True)
    else:
        force_stop_requested = True
        print(f"\n再次收到信号 {signum}：立即停止，不再等待回零。", flush=True)


def install_signal_handlers() -> None:
    signal.signal(signal.SIGINT, handle_shutdown_signal)
    signal.signal(signal.SIGTERM, handle_shutdown_signal)


def print_leader_status(
    leader: EncoderLeader,
    q: np.ndarray,
    gripper: float,
) -> None:
    versions = ["v{}.{}.{}".format(*version_tuple(motor)) for motor in leader.motors]
    print(
        f"[OK] CAN3 主臂 8 个编码器在线: "
        f"q={format_vector(q)}, gripper={gripper:+.3f} rad"
    )
    print(f"     version={versions}")


def capture_leader_reference(leader: EncoderLeader) -> Tuple[np.ndarray, float]:
    joint_samples = []
    gripper_samples = []
    for _ in range(LEADER_REFERENCE_SAMPLES):
        q, gripper = leader.read()
        joint_samples.append(q)
        gripper_samples.append(gripper)
    joint_samples_array = np.asarray(joint_samples, dtype=float)
    gripper_samples_array = np.asarray(gripper_samples, dtype=float)
    joint_zero = np.median(joint_samples_array, axis=0)
    gripper_zero = float(np.median(gripper_samples_array))
    joint_span = np.ptp(joint_samples_array, axis=0)
    gripper_span = float(np.ptp(gripper_samples_array))
    moving = np.where(joint_span > LEADER_REFERENCE_MAX_SPAN)[0]
    if moving.size or gripper_span > LEADER_REFERENCE_MAX_SPAN:
        details = [f"J{i + 1} span={joint_span[i]:.3f}" for i in moving]
        if gripper_span > LEADER_REFERENCE_MAX_SPAN:
            details.append(f"gripper span={gripper_span:.3f}")
        raise RuntimeError(
            "采集主臂启动基准时检测到移动: " + ", ".join(details)
            + "；请保持主臂自然下垂不动后重新运行"
        )
    return joint_zero, gripper_zero


def main() -> int:
    print("=" * 72)
    print("OpenArm 右臂同构编码器遥操")
    print("Leader: CAN3 / ID1~7 关节 + ID8 夹爪 / 运行只读、启动自动采集基准")
    print("Follower: CAN2 / 7 关节 + 夹爪")
    print(
        f"Mapping: joints={JOINT_SIGNS.tolist()}, "
        f"gripper_sign={GRIPPER_SIGN:+.0f}, offset=startup_reference"
    )
    print("=" * 72)
    print("无需输入参数；正在连接。请让主臂自然下垂且保持不动，并清空右臂工作区。")

    install_signal_handlers()
    leader = None
    follower = None
    follower_control_started = False
    gripper_command_ref = [0.0]
    exit_code = 0

    try:
        print(f"\n初始化 CAN3 编码器主臂: {LEADER_CONFIG}")
        serial_nodes = sorted(Path("/dev").glob("ttyACM*"))
        if not serial_nodes:
            raise RuntimeError(
                "没有发现 /dev/ttyACM* 通信板设备。请接好通信板 USB 和电源后再运行；"
                "已在进入原生 SDK 前退出，避免 Serial Port List 为空时段错误。"
            )
        print("检测到串口节点: " + ", ".join(str(path) for path in serial_nodes))

        leader = EncoderLeader(LEADER_CONFIG)
        leader_q, leader_gripper = leader.read()

        # 先验证主臂，再连接会运动的从臂。
        with FOLLOWER_CONFIG.open("r", encoding="utf-8") as handle:
            follower_cfg = yaml.safe_load(handle)
        lower = np.asarray(follower_cfg["robot"]["joint_limits"]["lower"])
        upper = np.asarray(follower_cfg["robot"]["joint_limits"]["upper"])
        gripper_lower = float(follower_cfg["robot"]["gripper_limits"]["lower"])
        gripper_upper = float(follower_cfg["robot"]["gripper_limits"]["upper"])
        print_leader_status(leader, leader_q, leader_gripper)
        print("[OK] CAN3 只读连接完成；不会修改编码器硬件零位")

        if shutdown_requested:
            return 0

        print(f"\n初始化 CAN2 OpenArm 右从臂: {FOLLOWER_CONFIG}")
        follower = Openarm(str(FOLLOWER_CONFIG))
        for _ in range(5):
            refresh_follower(follower, settle=0.03)
        validate_follower(follower)
        follower_gripper = float(follower.get_current_pos_gripper())
        print(
            f"[OK] CAN2 右臂在线: q={format_vector(follower.get_current_pos())}, "
            f"夹爪={follower_gripper:+.3f} rad"
        )

        follower_control_started = True
        if not home_follower(follower, 0.0, "START"):
            raise RuntimeError("启动回零失败")

        if not shutdown_requested:
            print("[ALIGN] 正在把主臂当前自然下垂姿态采集为 OpenArm 零位，请保持不动")
            leader_joint_zero, leader_gripper_zero = capture_leader_reference(leader)
            joint_target_from_leader(
                leader_joint_zero, leader_joint_zero, lower, upper
            )
            gripper_target_from_leader(
                leader_gripper_zero,
                leader_gripper_zero,
                gripper_lower,
                gripper_upper,
            )
            gripper_command_ref[0] = 0.0
            print(
                "[ALIGN] 启动基准已采集："
                f"q={format_vector(leader_joint_zero)}, "
                f"gripper={leader_gripper_zero:+.3f} rad；从臂此时目标为全零"
            )
            run_teleop(
                leader,
                follower,
                leader_joint_zero,
                leader_gripper_zero,
                gripper_command_ref,
            )
    except ImmediateStop as error:
        print(f"\n[STOP] {error}")
        exit_code = 130
    except Exception as error:
        print(f"\n[ERROR] {error}")
        exit_code = 1
    finally:
        if follower is not None and follower_control_started and not force_stop_requested:
            try:
                if not home_follower(follower, 0.0, "EXIT"):
                    exit_code = 1
            except ImmediateStop:
                pass
            except Exception as error:
                print(f"[WARN] 退出回零失败: {error}")
                exit_code = 1

        if follower is not None:
            try:
                # 与最后一帧 MIT 命令留出 SDK 200 Hz 发送间隔。
                time.sleep(CONTROL_DT)
                follower.set_stop()
                print("[STOP] CAN2 右臂和夹爪已停止")
            except Exception as error:
                print(f"[WARN] CAN2 右臂停止失败: {error}")
                exit_code = 1

        # 编码器主臂始终只读，不主动调用 set_stop/reset/zero。
        if leader is not None:
            print("[STOP] CAN3 编码器主臂停止读取")

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
