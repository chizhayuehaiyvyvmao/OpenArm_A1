#!/usr/bin/env python3
"""
openarm_HT 双臂 Quest 手柄笛卡尔空间 QP 控制程序

功能：
    1. 通过 UDP 接收 Quest 左右手柄数据
    2. 左手柄控制左臂，右手柄控制右臂
    3. 当对应手柄 Squeeze 按下时，使用手柄位姿控制对应机械臂末端
    4. 使用速度级 QP 逆运动学进行笛卡尔空间控制
    5. 使用 MIT 模式 + 重力/摩擦前馈进行关节控制
    6. 支持双臂夹爪独立开合控制
"""

import os
import socket
import struct
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from Lib import Openarm
from Lib.qp_ik import PinocchioOSQPIK


PORT = 5005
FMT = "40f"
SIZE = struct.calcsize(FMT)

left_hand_data = None
right_hand_data = None
data_lock = threading.Lock()
running = True

position_scale = 1.0
rot_scale = 0.8
CONTROL_FREQUENCY = 120.0
CONTROL_PERIOD = 1.0 / CONTROL_FREQUENCY
HAND_PACKET_TIMEOUT = 0.25
HAND_VELOCITY_TIMEOUT = 0.06
MIT_VELOCITY_FEEDFORWARD_SCALE = 1.0
# VR 快速动作专用限制；慢速指令不会触碰这些上限。
VR_JOINT_VELOCITY_LIMIT = np.array(
    [2.0, 2.0, 2.0, 2.0, 2.4, 2.4, 2.4], dtype=float
)
VR_JOINT_ACCELERATION_LIMIT = np.array(
    [16.0, 16.0, 16.0, 16.0, 20.0, 20.0, 20.0], dtype=float
)
VR_MAX_LINEAR_VELOCITY = 1.4
VR_MAX_ANGULAR_VELOCITY = 3.5
# J4 是肘关节。左右臂在 J4=0 附近都会进入伸直奇异位形，既容易
# 来回抖动，也可能在重新屈肘时切换到相反的肘分支。
ELBOW_SINGULARITY_MARGIN = 0.05
# 仅在肘接近伸直边界时限制不可达目标积压；正常运动不截断。
ELBOW_ANTIWINDUP_THRESHOLD = 0.12
ELBOW_POSITION_BACKLOG_LIMIT = 0.04
# 正常状态下只用于状态报警，不截断手柄的完整位移和姿态映射。
VR_POSITION_LAG_WARNING = 0.08
VR_ROTATION_LAG_WARNING = np.deg2rad(20.0)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LEFT_CONFIG = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_left.yaml"))
RIGHT_CONFIG = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_right.yaml"))


def mirror_left_to_right(left_values):
    return [-value for value in left_values]


def fmt_hand(name, vals, received_at):
    return {
        "name": name,
        "seq": int(vals[0]),
        "hand_id": int(vals[1]),
        "pose_valid": int(vals[2]),
        # 位姿差分前不能舍入，否则会形成明显的速度台阶和跟手延迟。
        "pos": tuple(float(x) for x in vals[3:6]),
        "rot": tuple(float(x) for x in vals[6:10]),
        "thumbstick": tuple(round(x, 4) for x in vals[10:12]),
        "trigger": round(vals[12], 4),
        "squeeze": round(vals[13], 4),
        "thumbstick_click": int(vals[14]),
        "primary": int(vals[15]),
        "secondary": int(vals[16]),
        "trigger_pressed": int(vals[17]),
        "squeeze_pressed": int(vals[18]),
        "received_at": received_at,
    }


def transform_hand_to_robot_coords(hand_pos, hand_rot_matrix):
    robot_pos = np.array([
        -hand_pos[2],
        -hand_pos[0],
        hand_pos[1],
    ])
    transform = np.array([
        [0, 0, 1],
        [1, 0, 0],
        [0, -1, 0],
    ])
    robot_rot_matrix = transform @ hand_rot_matrix @ transform.T
    return robot_pos, robot_rot_matrix


def is_hand_data_valid(hand_info):
    if hand_info is None:
        return False
    if hand_info["pose_valid"] == 0:
        return False
    if np.linalg.norm(np.array(hand_info["pos"])) < 0.001:
        return False
    if np.linalg.norm(np.array(hand_info["rot"])) < 0.01:
        return False
    return True


def is_hand_data_all_zero(hand_info):
    if hand_info is None:
        return True
    if np.linalg.norm(np.array(hand_info["pos"])) > 0.001:
        return False
    if np.linalg.norm(np.array(hand_info["rot"])) > 0.001:
        return False
    if (
        abs(hand_info["trigger"]) > 0.001
        or abs(hand_info["squeeze"]) > 0.001
        or abs(hand_info["thumbstick"][0]) > 0.001
        or abs(hand_info["thumbstick"][1]) > 0.001
    ):
        return False
    return True


def quaternion_to_rotation_matrix(q):
    return R.from_quat([q[0], q[1], q[2], q[3]]).as_matrix()


def udp_receiver_thread():
    global left_hand_data, right_hand_data, running

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))
    print(f"UDP 接收器启动，监听端口: {PORT}")

    while running:
        try:
            data, _ = sock.recvfrom(4096)
            if len(data) != SIZE:
                continue
            vals = struct.unpack(FMT, data)
            left = vals[:20]
            right = vals[20:]
            received_at = time.perf_counter()
            with data_lock:
                left_hand_data = fmt_hand("LEFT", left, received_at)
                right_hand_data = fmt_hand("RIGHT", right, received_at)
        except Exception as exc:
            if running:
                print(f"UDP 接收错误: {exc}")

    sock.close()


def move_to_safe_position(robot, safe_joint_pos):
    print("\n" + "=" * 60)
    print("正在移动到安全位置...")
    print("=" * 60)

    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.3)

    current_pos = robot.get_current_pos()
    steps = 40
    duration = 2.0
    dt = duration / steps

    for i in range(steps + 1):
        alpha = i / steps
        interp_pos = (1 - alpha) * current_pos + alpha * np.asarray(safe_joint_pos)
        if i < steps:
            next_alpha = (i + 1) / steps
            next_pos = (1 - next_alpha) * current_pos + next_alpha * np.asarray(safe_joint_pos)
            vel = (next_pos - interp_pos) / dt
        else:
            vel = np.zeros(robot.motor_count)

        max_vel = np.array([1.0, 1.0, 1.0, 1.0, 1.5, 1.5, 1.5])
        vel = np.clip(vel, -max_vel, max_vel)
        robot.Joint_Pos_Vel(interp_pos.tolist(), vel.tolist())
        time.sleep(dt)

    success = robot.Joint_Pos_Vel(list(safe_joint_pos), [0.0] * robot.motor_count, iswait=True)
    if success:
        print("✓ 已到达安全位置")
    else:
        print("✗ 移动到安全位置失败")
    time.sleep(0.5)
    return success


def sync_targets_to_current_state(arm_state):
    robot = arm_state["robot"]
    current_joint_pos = robot.get_current_pos()
    current_fk = robot.forward_kinematics(current_joint_pos)
    arm_state["target_position"] = current_fk["position"].copy()
    arm_state["target_rotation"] = current_fk["rotation"].copy()
    arm_state["control_activated"] = False
    arm_state["last_hand_pos"] = None
    arm_state["last_hand_rot"] = None
    arm_state["last_hand_sequence"] = None
    arm_state["last_hand_sample_time"] = None
    arm_state["hand_twist"].fill(0.0)
    if "qp_solver" in arm_state:
        arm_state["qp_joint_pos"] = np.asarray(
            current_joint_pos, dtype=float
        ).copy()
        arm_state["qp_joint_vel"].fill(0.0)
        arm_state["qp_solver"].reset(
            posture_reference=arm_state["qp_joint_pos"]
        )
    return current_joint_pos


def limit_target_to_follow_window(arm_state, current_joint_pos):
    """测量跟随误差，并在伸直边界清除不可达位置目标的积压。"""
    current_fk = arm_state["robot"].forward_kinematics(current_joint_pos)
    current_position = np.asarray(current_fk["position"], dtype=float)
    current_rotation = np.asarray(current_fk["rotation"], dtype=float)

    position_delta = arm_state["target_position"] - current_position
    raw_position_error = float(np.linalg.norm(position_delta))

    elbow_near_straight = (
        abs(float(current_joint_pos[3]))
        <= ELBOW_ANTIWINDUP_THRESHOLD
    )
    reach_limited = (
        elbow_near_straight
        and raw_position_error > ELBOW_POSITION_BACKLOG_LIMIT
    )
    if reach_limited:
        # 只清除肘伸直后继续向不可达区域累积的目标。保留目标方向，
        # 手柄一旦向下或向内运动，机械臂无需先走完旧的积压距离。
        arm_state["target_position"] = (
            current_position
            + position_delta
            * (ELBOW_POSITION_BACKLOG_LIMIT / raw_position_error)
        )

    rotation_delta = arm_state["target_rotation"] @ current_rotation.T
    rotation_vector = R.from_matrix(rotation_delta).as_rotvec()
    raw_rotation_error = float(np.linalg.norm(rotation_vector))

    arm_state["follow_position_error"] = raw_position_error
    arm_state["follow_rotation_error"] = raw_rotation_error
    arm_state["reach_antiwindup_active"] = reach_limited
    arm_state["target_window_limited"] = (
        reach_limited
        or raw_position_error > VR_POSITION_LAG_WARNING
        or raw_rotation_error > VR_ROTATION_LAG_WARNING
    )


def hold_arm_at_encoder_position(arm_state):
    """数据中断或松开控制时立即停止上一条速度命令。"""
    robot = arm_state["robot"]
    current_position = sync_targets_to_current_state(arm_state)
    current_velocity = robot.get_current_vel()
    gravity_torque = robot.get_Gravity(current_position)
    friction_torque = robot.get_friction_compensation(
        current_velocity,
        arm_state["Fc"],
        arm_state["Fv"],
        arm_state["vel_threshold"],
    )
    hold_torque = gravity_torque + friction_torque
    torque_limit = np.asarray(robot.max_torque, dtype=float)
    hold_torque = np.clip(hold_torque, -torque_limit, torque_limit)
    robot.gripper_control_MIT(
        arm_state["last_gripper_pos"],
        arm_state["gripper_vel"],
        0.0,
        arm_state["gripper_kp"],
        arm_state["gripper_kd"],
        send=False,
    )
    robot.pos_vel_tqe_kp_kd(
        pos=current_position.tolist(),
        vel=[0.0] * robot.motor_count,
        tqe=hold_torque.tolist(),
        kp=arm_state["kp"],
        kd=arm_state["kd"],
    )


def execute_smooth_joint_reset(
    arm_state,
    target_joint_pos,
    duration=2.0,
    control_dt=0.01,
    label="复位",
):
    global running

    robot = arm_state["robot"]
    kp = arm_state["kp"]
    kd = arm_state["kd"]
    Fc = arm_state["Fc"]
    Fv = arm_state["Fv"]
    vel_threshold = arm_state["vel_threshold"]
    gripper_pos = arm_state["last_gripper_pos"]
    gripper_vel = arm_state["gripper_vel"]
    gripper_kp = arm_state["gripper_kp"]
    gripper_kd = arm_state["gripper_kd"]

    print(f"\n正在执行{arm_state['label']}{label}: {np.round(target_joint_pos, 3)}")
    start_joint_pos = sync_targets_to_current_state(arm_state)
    target_joint_pos = np.asarray(target_joint_pos, dtype=np.float64)
    tau_limit = np.asarray(robot.max_torque, dtype=np.float64)
    start_time = time.time()

    while running:
        elapsed = time.time() - start_time
        q_des, v_des, _ = robot.septic_interpolation(
            start_joint_pos,
            target_joint_pos,
            duration,
            elapsed,
        )

        current_joint_vel = robot.get_current_vel()
        gravity_torque = robot.get_Gravity(q_des)
        friction_torque = robot.get_friction_compensation(current_joint_vel, Fc, Fv, vel_threshold)
        feedforward_torque = np.clip(gravity_torque + friction_torque, -tau_limit, tau_limit)

        robot.pos_vel_tqe_kp_kd(
            pos=q_des.tolist(),
            vel=v_des.tolist(),
            tqe=feedforward_torque.tolist(),
            kp=kp,
            kd=kd,
        )
        robot.gripper_control_MIT(gripper_pos, gripper_vel, 0.0, gripper_kp, gripper_kd)

        if elapsed >= duration:
            break
        time.sleep(control_dt)

    sync_targets_to_current_state(arm_state)
    return True


def build_arm_state(label, robot, safe_joint_pos):
    current_joint_pos = robot.get_current_pos()
    current_fk = robot.forward_kinematics(current_joint_pos)
    qp_solver = PinocchioOSQPIK(
        robot,
        max_joint_velocity=float(np.max(VR_JOINT_VELOCITY_LIMIT)),
        max_joint_acceleration=float(
            np.max(VR_JOINT_ACCELERATION_LIMIT)
        ),
        max_linear_velocity=VR_MAX_LINEAR_VELOCITY,
        max_angular_velocity=VR_MAX_ANGULAR_VELOCITY,
        # 原版 7 轴 IK 每周期从当前关节角开始，天然偏向连续的近邻解。
        # QP 用软姿态项实现同样效果。J2 负责肩部外展/内收，J3 主要
        # 改变上臂扭转和肘朝向，因此提高这两轴的权重；它们仍会在
        # 末端位姿确实需要时参与运动，不是硬锁关节。
        posture_weight=1.2e-1,
        posture_gain=1.0,
        posture_joint_weights=[1.0, 10.0, 16.0, 1.0, 0.5, 0.5, 0.5],
        # 接近伸直时逐渐增大阻尼，抑制奇异方向上的速度来回翻转。
        singularity_threshold=0.12,
        singularity_damping=8.0e-2,
    )
    # openarm_HT 每臂是完整 7 自由度；全部关节都参与 QP，位置边界和
    # 临近边界减速仍由求解器负责。
    qp_solver.velocity_limit = VR_JOINT_VELOCITY_LIMIT.copy()
    qp_solver.acceleration_limit = VR_JOINT_ACCELERATION_LIMIT.copy()

    # 用安全初始姿态中 J4 的符号确定本臂的屈肘分支，并给 0 位留出
    # 安全带。它是 QP 的硬位置约束，不会被末端任务或零空间项越过。
    elbow_sign = float(np.sign(safe_joint_pos[3]))
    if elbow_sign < 0.0:
        qp_solver.upper[3] = min(
            qp_solver.upper[3], -ELBOW_SINGULARITY_MARGIN
        )
    elif elbow_sign > 0.0:
        qp_solver.lower[3] = max(
            qp_solver.lower[3], ELBOW_SINGULARITY_MARGIN
        )
    else:
        raise ValueError(f"{label}安全姿态的 J4 不能为 0")

    qp_solver.reset(posture_reference=current_joint_pos)
    return {
        "label": label,
        "robot": robot,
        "safe_joint_pos": np.asarray(safe_joint_pos, dtype=float),
        "target_position": current_fk["position"].copy(),
        "target_rotation": current_fk["rotation"].copy(),
        "control_activated": False,
        "last_hand_pos": None,
        "last_hand_rot": None,
        "last_hand_sequence": None,
        "last_hand_sample_time": None,
        "hand_twist": np.zeros(6, dtype=float),
        "current_hand_pos_robot": None,
        "kp": [30.0, 50.0, 60.0, 25.0, 15.0, 10.0, 10.0],
        "kd": [3.0, 5.0, 6.0, 2.5, 1.5, 1.0, 1.0],
        "Fc": np.array([0.20, 0.20, 0.15, 0.15, 0.0, 0.1, 0.0]),
        "Fv": np.array([0.06, 0.06, 0.04, 0.04, 0.0, 0.03, 0.0]),
        "vel_threshold": 0.02,
        "initial_joint_pos": current_joint_pos.copy(),
        "qp_solver": qp_solver,
        "qp_joint_pos": np.asarray(current_joint_pos, dtype=float).copy(),
        "qp_joint_vel": np.zeros(robot.motor_count, dtype=float),
        "actual_joint_vel": np.zeros(robot.motor_count, dtype=float),
        "actual_joint_pos": np.asarray(current_joint_pos, dtype=float).copy(),
        "qp_last_failure_print": 0.0,
        "follow_position_error": 0.0,
        "follow_rotation_error": 0.0,
        "target_window_limited": False,
        "reach_antiwindup_active": False,
        "gripper_open_pos": 1.6,
        "gripper_close_pos": 0.0,
        "gripper_vel": 0.0,
        "gripper_kp": 3.0,
        "gripper_kd": 0.30,
        "last_gripper_pos": 1.6,
        "last_primary_pressed": 0,
    }


def process_arm_control(
    arm_state,
    hand_info,
    squeeze_threshold,
    qp_dt,
    reset_move_duration,
    now,
):
    robot = arm_state["robot"]

    if (
        hand_info is not None
        and now - hand_info["received_at"] > HAND_PACKET_TIMEOUT
    ):
        hand_info = None

    if hand_info is None:
        arm_state["current_hand_pos_robot"] = None
        if arm_state["control_activated"]:
            print(f"\n{arm_state['label']}控制已停止 - 未收到手柄数据")
            hold_arm_at_encoder_position(arm_state)
        arm_state["control_activated"] = False
        arm_state["last_hand_pos"] = None
        arm_state["last_hand_rot"] = None
        arm_state["last_hand_sequence"] = None
        arm_state["last_hand_sample_time"] = None
        arm_state["hand_twist"].fill(0.0)
        arm_state["last_primary_pressed"] = 0
        return

    if is_hand_data_all_zero(hand_info):
        arm_state["current_hand_pos_robot"] = None
        if arm_state["control_activated"]:
            print(f"\n{arm_state['label']}控制已停止 - 手柄未检测到")
            hold_arm_at_encoder_position(arm_state)
        arm_state["control_activated"] = False
        arm_state["last_hand_pos"] = None
        arm_state["last_hand_rot"] = None
        arm_state["last_hand_sequence"] = None
        arm_state["last_hand_sample_time"] = None
        arm_state["hand_twist"].fill(0.0)
        robot.gripper_control_MIT(
            arm_state["last_gripper_pos"],
            arm_state["gripper_vel"],
            0.0,
            arm_state["gripper_kp"],
            arm_state["gripper_kd"],
        )
        arm_state["last_primary_pressed"] = 0
        return

    if not is_hand_data_valid(hand_info):
        arm_state["current_hand_pos_robot"] = None
        if arm_state["control_activated"]:
            print(f"\n{arm_state['label']}控制已停止 - 手柄姿态无效")
            hold_arm_at_encoder_position(arm_state)
        arm_state["control_activated"] = False
        arm_state["last_hand_pos"] = None
        arm_state["last_hand_rot"] = None
        arm_state["last_hand_sequence"] = None
        arm_state["last_hand_sample_time"] = None
        arm_state["hand_twist"].fill(0.0)
        robot.gripper_control_MIT(
            arm_state["last_gripper_pos"],
            arm_state["gripper_vel"],
            0.0,
            arm_state["gripper_kp"],
            arm_state["gripper_kd"],
        )
        arm_state["last_primary_pressed"] = 0
        return

    squeeze_value = hand_info["squeeze"]
    trigger_value = hand_info["trigger"]
    primary_pressed = hand_info["primary"]
    hand_pos_raw = np.array(hand_info["pos"])
    hand_rot_quat = hand_info["rot"]

    hand_pos_robot, hand_rot_robot = transform_hand_to_robot_coords(
        hand_pos_raw,
        quaternion_to_rotation_matrix(hand_rot_quat),
    )
    arm_state["current_hand_pos_robot"] = hand_pos_robot

    if squeeze_value > squeeze_threshold:
        gripper_target_pos = arm_state["gripper_open_pos"] - trigger_value * (
            arm_state["gripper_open_pos"] - arm_state["gripper_close_pos"]
        )
    else:
        gripper_target_pos = arm_state["last_gripper_pos"]
    robot.gripper_control_MIT(
        gripper_target_pos,
        arm_state["gripper_vel"],
        0.0,
        arm_state["gripper_kp"],
        arm_state["gripper_kd"],
        send=False,
    )
    arm_state["last_gripper_pos"] = gripper_target_pos

    if primary_pressed and not arm_state["last_primary_pressed"]:
        print(f"\n检测到{arm_state['label']} A 键按下，开始复位到初始关节角")
        execute_smooth_joint_reset(
            arm_state=arm_state,
            target_joint_pos=arm_state["initial_joint_pos"],
            duration=reset_move_duration,
            control_dt=CONTROL_PERIOD,
            label="A键复位",
        )
        arm_state["last_primary_pressed"] = primary_pressed
        time.sleep(0.05)
        return

    if squeeze_value > squeeze_threshold:
        if not arm_state["control_activated"]:
            # 激活瞬间以编码器和手柄当前位姿为共同零点，避免重新按下时跳变。
            sync_targets_to_current_state(arm_state)
            arm_state["last_hand_pos"] = hand_pos_robot.copy()
            arm_state["last_hand_rot"] = hand_rot_robot.copy()
            arm_state["last_hand_sequence"] = hand_info["seq"]
            arm_state["last_hand_sample_time"] = hand_info["received_at"]
            arm_state["hand_twist"].fill(0.0)
            arm_state["control_activated"] = True
            print(f"\n{arm_state['label']}控制已激活")
            print(f"  Squeeze 值: {squeeze_value:.2f}")

        # 同一个 UDP 包在多个控制周期中只能累计一次；速度使用真实包间隔。
        if hand_info["seq"] != arm_state["last_hand_sequence"]:
            sample_dt = (
                hand_info["received_at"]
                - arm_state["last_hand_sample_time"]
            )
            if not np.isfinite(sample_dt) or sample_dt <= 1e-6:
                sample_dt = qp_dt

            delta_hand_pos = (
                hand_pos_robot - arm_state["last_hand_pos"]
            ) * position_scale
            delta_hand_rot = hand_rot_robot @ arm_state["last_hand_rot"].T
            delta_rotvec = (
                R.from_matrix(delta_hand_rot).as_rotvec() * rot_scale
            )
            scaled_delta_hand_rot = R.from_rotvec(delta_rotvec).as_matrix()

            arm_state["target_position"] = (
                arm_state["target_position"] + delta_hand_pos
            )
            arm_state["target_rotation"] = (
                scaled_delta_hand_rot @ arm_state["target_rotation"]
            )
            arm_state["hand_twist"][:3] = delta_hand_pos / sample_dt
            arm_state["hand_twist"][3:] = delta_rotvec / sample_dt
            arm_state["last_hand_pos"] = hand_pos_robot.copy()
            arm_state["last_hand_rot"] = hand_rot_robot.copy()
            arm_state["last_hand_sequence"] = hand_info["seq"]
            arm_state["last_hand_sample_time"] = hand_info["received_at"]

        if (
            arm_state["last_hand_sample_time"] is None
            or now - arm_state["last_hand_sample_time"]
            > HAND_VELOCITY_TIMEOUT
        ):
            arm_state["hand_twist"].fill(0.0)
    else:
        if arm_state["control_activated"]:
            print(f"\n{arm_state['label']}控制已停止 - 同步编码器并保持当前位置")
            hold_arm_at_encoder_position(arm_state)
        arm_state["control_activated"] = False
        arm_state["last_hand_pos"] = None
        arm_state["last_hand_rot"] = None
        arm_state["last_hand_sequence"] = None
        arm_state["last_hand_sample_time"] = None
        arm_state["hand_twist"].fill(0.0)

    actual_joint_position = np.asarray(
        robot.get_current_pos(), dtype=float
    )
    arm_state["actual_joint_pos"] = actual_joint_position.copy()
    limit_target_to_follow_window(arm_state, actual_joint_position)

    target_transform = np.eye(4, dtype=float)
    target_transform[:3, :3] = arm_state["target_rotation"]
    target_transform[:3, 3] = arm_state["target_position"]
    qp_result = None
    try:
        qp_result = arm_state["qp_solver"].step(
            arm_state["qp_joint_pos"],
            target_transform,
            qp_dt,
            feedforward_twist=arm_state["hand_twist"],
        )
    except (RuntimeError, ValueError) as exc:
        now = time.perf_counter()
        if now - arm_state["qp_last_failure_print"] >= 1.0:
            print(f"\n{arm_state['label']} QP逆解失败，保持上一目标: {exc}")
            arm_state["qp_last_failure_print"] = now
        arm_state["qp_solver"].reset()
        arm_state["qp_joint_vel"].fill(0.0)

    joint_pos = qp_result.joint_position if qp_result is not None else None

    if joint_pos is not None:
        gravity_torque = robot.get_Gravity(joint_pos)
        current_joint_vel = robot.get_current_vel()
        arm_state["actual_joint_vel"] = np.asarray(
            current_joint_vel, dtype=float
        ).copy()
        friction_torque = robot.get_friction_compensation(
            current_joint_vel,
            arm_state["Fc"],
            arm_state["Fv"],
            arm_state["vel_threshold"],
        )
        feedforward_torque = gravity_torque + friction_torque
        tau_limit = np.asarray(robot.max_torque, dtype=float)
        feedforward_torque = np.clip(feedforward_torque, -tau_limit, tau_limit)
        target_vel = (
            qp_result.joint_velocity * MIT_VELOCITY_FEEDFORWARD_SCALE
        )
        success = robot.pos_vel_tqe_kp_kd(
            pos=joint_pos.tolist(),
            vel=target_vel.tolist(),
            tqe=feedforward_torque.tolist(),
            kp=arm_state["kp"],
            kd=arm_state["kd"],
        )
        if success:
            arm_state["qp_joint_pos"] = qp_result.joint_position.copy()
            arm_state["qp_joint_vel"] = qp_result.joint_velocity.copy()
    else:
        # QP 失败时也要把已缓存的夹爪命令发送出去。
        robot.motor_send_cmd()

    arm_state["last_primary_pressed"] = primary_pressed


def main():
    global running

    print("==" * 60)
    print("openarm_HT 双臂 Quest 手柄笛卡尔空间控制")
    print("控制模式: 速度级QP逆解 + MIT模式(PD控制 + 重力/摩擦前馈)")
    print("=" * 60)

    udp_thread = threading.Thread(target=udp_receiver_thread, daemon=True)
    udp_thread.start()

    print("\n等待 Quest 手柄连接...")
    connection_timeout = 10
    start_time = time.time()
    while left_hand_data is None and right_hand_data is None:
        time.sleep(0.1)
        if time.time() - start_time > connection_timeout:
            print("警告: 未在规定时间内检测到手柄连接，继续运行...")
            break
    print("已连接到 Quest 手柄")

    print("\n初始化 openarm_HT 双臂机械臂...")
    left_robot = Openarm(LEFT_CONFIG)
    right_robot = Openarm(RIGHT_CONFIG)

    if left_robot.model is None or right_robot.model is None:
        print("错误：未找到 Pinocchio 模型，无法计算逆运动学")
        running = False
        return

    left_safe = [0.3, 0.0, 0.0, -1.87, 0.0, 0.0, 0.0]
    right_safe = mirror_left_to_right(left_safe)

    if not move_to_safe_position(left_robot, left_safe):
        running = False
        return
    if not move_to_safe_position(right_robot, right_safe):
        running = False
        return

    left_arm = build_arm_state("左臂", left_robot, left_safe)
    right_arm = build_arm_state("右臂", right_robot, right_safe)

    print("\n控制说明：")
    print("  左手柄 Squeeze: 控制左臂")
    print("  右手柄 Squeeze: 控制右臂")
    print("  对应 Trigger: 控制对应夹爪")
    print("  对应 A 键: 复位对应机械臂")
    print("  Ctrl+C: 退出程序")
    print("\n开始控制，请小心操作！\n")

    control_rate = CONTROL_PERIOD
    squeeze_threshold = 0.5
    print_count = 0
    reset_move_duration = 2.0
    previous_loop_start = None
    next_loop_deadline = time.perf_counter()
    loop_period_window = []

    try:
        while running:
            loop_start = time.perf_counter()
            if previous_loop_start is None:
                qp_dt = control_rate
            else:
                measured_dt = loop_start - previous_loop_start
                qp_dt = float(np.clip(
                    measured_dt,
                    0.5 * control_rate,
                    2.0 * control_rate,
                ))
                loop_period_window.append(measured_dt)
                if len(loop_period_window) > 200:
                    del loop_period_window[0]
            previous_loop_start = loop_start

            with data_lock:
                left_info = left_hand_data.copy() if left_hand_data is not None else None
                right_info = right_hand_data.copy() if right_hand_data is not None else None

            process_arm_control(
                left_arm,
                left_info,
                squeeze_threshold,
                qp_dt,
                reset_move_duration,
                loop_start,
            )
            process_arm_control(
                right_arm,
                right_info,
                squeeze_threshold,
                qp_dt,
                reset_move_duration,
                loop_start,
            )

            left_fk = left_robot.forward_kinematics()
            right_fk = right_robot.forward_kinematics()

            print_count += 1
            if print_count % 20 == 0:
                left_status = "激活" if left_arm["control_activated"] else "停止"
                right_status = "激活" if right_arm["control_activated"] else "停止"
                left_pos = left_fk["position"]
                right_pos = right_fk["position"]
                left_packet_age = (
                    (loop_start - left_info["received_at"]) * 1000.0
                    if left_info is not None else float("nan")
                )
                right_packet_age = (
                    (loop_start - right_info["received_at"]) * 1000.0
                    if right_info is not None else float("nan")
                )
                left_qdot = np.max(np.abs(left_arm["qp_joint_vel"]))
                right_qdot = np.max(np.abs(right_arm["qp_joint_vel"]))
                left_actual_qdot = np.max(
                    np.abs(left_arm["actual_joint_vel"])
                )
                right_actual_qdot = np.max(
                    np.abs(right_arm["actual_joint_vel"])
                )
                left_elbow_deg = np.rad2deg(
                    left_arm["actual_joint_pos"][3]
                )
                right_elbow_deg = np.rad2deg(
                    right_arm["actual_joint_pos"][3]
                )
                actual_frequency = (
                    1.0 / np.median(loop_period_window)
                    if loop_period_window else 0.0
                )
                print(
                    f"\r左臂:{left_status} 目标[{left_arm['target_position'][0]:.3f}, {left_arm['target_position'][1]:.3f}, {left_arm['target_position'][2]:.3f}] "
                    f"当前[{left_pos[0]:.3f}, {left_pos[1]:.3f}, {left_pos[2]:.3f}] "
                    f"误差:{left_arm['follow_position_error']:.3f}"
                    f"{'限' if left_arm['reach_antiwindup_active'] else ('*' if left_arm['target_window_limited'] else '')} "
                    f"包:{left_packet_age:.0f}ms "
                    f"肘:{left_elbow_deg:+.1f}° "
                    f"qdot:{left_qdot:.2f}/{left_actual_qdot:.2f} | "
                    f"右臂:{right_status} 目标[{right_arm['target_position'][0]:.3f}, {right_arm['target_position'][1]:.3f}, {right_arm['target_position'][2]:.3f}] "
                    f"当前[{right_pos[0]:.3f}, {right_pos[1]:.3f}, {right_pos[2]:.3f}] "
                    f"误差:{right_arm['follow_position_error']:.3f}"
                    f"{'限' if right_arm['reach_antiwindup_active'] else ('*' if right_arm['target_window_limited'] else '')} "
                    f"包:{right_packet_age:.0f}ms "
                    f"肘:{right_elbow_deg:+.1f}° "
                    f"qdot:{right_qdot:.2f}/{right_actual_qdot:.2f} "
                    f"| {actual_frequency:.1f} Hz",
                    end="",
                )

            next_loop_deadline += control_rate
            sleep_time = next_loop_deadline - time.perf_counter()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_loop_deadline = time.perf_counter()

    except KeyboardInterrupt:
        print("\n\n程序被中断")
    finally:
        print("\n\n安全停止双臂机械臂...")
        for arm_state in (left_arm, right_arm):
            robot = arm_state["robot"]
            current_pos = robot.get_current_pos()
            current_vel = robot.get_current_vel()
            gravity_torque = robot.get_Gravity(current_pos)
            friction_torque = robot.get_friction_compensation(
                current_vel,
                arm_state["Fc"],
                arm_state["Fv"],
                arm_state["vel_threshold"],
            )
            hold_torque = gravity_torque + friction_torque
            tau_limit = np.asarray(robot.max_torque, dtype=float)
            hold_torque = np.clip(hold_torque, -tau_limit, tau_limit)
            robot.pos_vel_tqe_kp_kd(
                pos=current_pos.tolist(),
                vel=[0.0] * robot.motor_count,
                tqe=hold_torque.tolist(),
                kp=arm_state["kp"],
                kd=arm_state["kd"],
            )

        time.sleep(0.5)

        execute_smooth_joint_reset(left_arm, [0.0] * left_robot.motor_count, duration=2.5, label="退出回零")
        execute_smooth_joint_reset(right_arm, [0.0] * right_robot.motor_count, duration=2.5, label="退出回零")
        running = False
        print("程序结束")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback

        traceback.print_exc()
