#!/usr/bin/env python3
"""openarm_HT 左臂、右臂或双臂 QP 奇异位形与头顶画圆例程。

左右臂使用各自 YAML 中的关节名和 TCP frame，但必须加载同一份
双臂 URDF。左臂关节路径为 J1: 0 -> -180°、J4<0；右臂使用
镜像路径 J1: 0 -> +180°、J4>0。

流程：
    1. 双臂使用 MIT 同步移动到接近零位的伸直姿态
    2. 双臂在奇异区域内镜像举到头顶
    3. 双手在头顶 X-Y 平面画镜像圆
    4. QP 演示完成后，双臂使用 MIT 同步回零

所有实体电机指令都通过 MIT 五参数接口下发。
"""

import os
import time

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation

from Lib import Openarm
from Lib.qp_ik import PinocchioOSQPIK


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LEFT_CONFIG = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_left.yaml")
)
RIGHT_CONFIG = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_right.yaml")
)

CONTROL_FREQUENCY = 100.0
CONTROL_PERIOD = 1.0 / CONTROL_FREQUENCY
TELEMETRY_PRINT_PERIOD = 0.5
SHOULDER_JOINT_INDEX = 0
ELBOW_JOINT_INDEX = 3
ELBOW_SINGULARITY_MARGIN = 0.05
START_ELBOW_BEND = 0.10
SHOULDER_ARC_DURATION = 10.0
ARC_HOLD_DURATION = 1.0
CIRCLE_RADIUS = 0.06
CIRCLE_LOWERING = 0.10
CIRCLE_PREP_DURATION = 3.0
CIRCLE_DURATION = 8.0
CIRCLE_HOLD_DURATION = 0.5

MIT_KP = np.array([30.0, 50.0, 60.0, 25.0, 15.0, 10.0, 10.0])
MIT_KD = np.array([3.0, 5.0, 6.0, 2.5, 1.5, 1.0, 1.0])

EXPECTED_LEFT_JOINTS = tuple(
    f"l_joint{index}_link" for index in range(1, 8)
)
EXPECTED_RIGHT_JOINTS = tuple(
    f"r_joint{index}_link" for index in range(1, 8)
)
EXPECTED_LEFT_TCP = "l_joint7_link"
EXPECTED_RIGHT_TCP = "r_joint7_link"


def select_arm():
    print("\n请选择机械臂：")
    print("  1. 左臂")
    print("  2. 右臂")
    print("  3. 双臂")
    while True:
        choice = input("请输入 1/2/3: ").strip()
        if choice == "1":
            return [("左臂", LEFT_CONFIG, 1.0)]
        if choice == "2":
            return [("右臂", RIGHT_CONFIG, -1.0)]
        if choice == "3":
            return [
                ("左臂", LEFT_CONFIG, 1.0),
                ("右臂", RIGHT_CONFIG, -1.0),
            ]
        print("输入无效，请输入 1、2 或 3。")


def quintic_progress(value):
    """五次 smoothstep，使轨迹两端的速度和加速度为零。"""
    value = float(np.clip(value, 0.0, 1.0))
    return 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5


def pose_error(target_transform, actual_transform):
    position_error = float(np.linalg.norm(
        target_transform[:3, 3] - actual_transform[:3, 3]
    ))
    rotation_error = float(Rotation.from_matrix(
        target_transform[:3, :3] @ actual_transform[:3, :3].T
    ).magnitude())
    return position_error, rotation_error


def read_motor_feedback(robot):
    """读取七个手臂电机的实时反馈，不发送额外控制命令。"""
    motor_states = robot.get_current_state()
    if len(motor_states) != robot.motor_count:
        raise RuntimeError(
            f"反馈电机数错误: expected={robot.motor_count}, actual={len(motor_states)}"
        )
    return {
        "position": np.asarray([item.position for item in motor_states], dtype=float),
        "velocity": np.asarray([item.velocity for item in motor_states], dtype=float),
        "torque": np.asarray([item.torque for item in motor_states], dtype=float),
        "mode": [int(item.mode) for item in motor_states],
        "fault": [int(item.fault) for item in motor_states],
    }


def format_joint_values(values):
    return "[" + ", ".join(f"{value:+.3f}" for value in values) + "]"


def selected_scope(states):
    return "双臂" if len(states) == 2 else states[0]["label"]


def print_realtime_telemetry(phase, progress, states, observations):
    """以固定低频率输出命令、反馈、误差、速度、力矩和故障。"""
    print(f"\n[{phase}] progress={progress * 100.0:5.1f}%")
    for state, observation in zip(states, observations):
        command = observation["command_joint"]
        actual = observation["actual_joint"]
        error = command - actual
        print(
            f"  {state['label']} q_cmd={format_joint_values(command)}\n"
            f"       q_act={format_joint_values(actual)}\n"
            f"       q_err={format_joint_values(error)} "
            f"max={np.max(np.abs(error)):.3f} rad\n"
            f"       vel={format_joint_values(observation['actual_velocity'])}\n"
            f"       tau={format_joint_values(observation['actual_torque'])}\n"
            f"       mode={observation['mode']} fault={observation['fault']} "
            f"tcp_err={observation.get('position_error', float('nan')) * 1000.0:.1f} mm"
        )


def resolved_urdf_path(robot):
    relative_path = robot.config["urdf"]["file_path"]
    return os.path.realpath(os.path.join(robot.config_dir, relative_path))


def validate_arm_urdf(robot, label, expected_joints, expected_tcp):
    """验证某侧 YAML 与 Pinocchio 模型的关节和末端 frame。"""
    if robot.model is None:
        raise RuntimeError(f"{label}: Pinocchio URDF 模型未加载")
    if robot.motor_count != 7:
        raise RuntimeError(f"{label}: 需要 7 个手臂关节")
    if tuple(robot.joint_names) != expected_joints:
        raise RuntimeError(
            f"{label}: YAML joint_names 错误，"
            f"期望 {expected_joints}，实际 {tuple(robot.joint_names)}"
        )
    configured_tcp = robot.config["urdf"]["end_effector_link"]
    if configured_tcp != expected_tcp:
        raise RuntimeError(
            f"{label}: end_effector_link 应为 {expected_tcp}，"
            f"实际为 {configured_tcp}"
        )
    missing_joints = [
        name for name in expected_joints
        if not robot.model.existJointName(name)
    ]
    if missing_joints:
        raise RuntimeError(f"{label}: URDF 缺少关节 {missing_joints}")
    if not robot.model.existFrame(expected_tcp, pin.BODY):
        raise RuntimeError(f"{label}: URDF 缺少 BODY 末端 frame {expected_tcp}")
    expected_frame_id = robot.model.getFrameId(expected_tcp, pin.BODY)
    if robot.end_effector_frame_id != expected_frame_id:
        raise RuntimeError(f"{label}: Pinocchio TCP frame ID 不一致")


def validate_dual_urdf(left_robot, right_robot):
    """双臂下发命令前完成 URDF 结构与镜像 FK 验证。"""
    validate_arm_urdf(
        left_robot, "左臂", EXPECTED_LEFT_JOINTS, EXPECTED_LEFT_TCP
    )
    validate_arm_urdf(
        right_robot, "右臂", EXPECTED_RIGHT_JOINTS, EXPECTED_RIGHT_TCP
    )
    left_urdf = resolved_urdf_path(left_robot)
    right_urdf = resolved_urdf_path(right_robot)
    if not os.path.isfile(left_urdf):
        raise RuntimeError(f"URDF 文件不存在: {left_urdf}")
    if left_urdf != right_urdf:
        raise RuntimeError(
            "左右 YAML 必须加载同一份双臂 URDF："
            f"\n  左臂: {left_urdf}\n  右臂: {right_urdf}"
        )

    # 两个 Openarm 实例都应加载完整双臂模型，只是各自
    # 从 YAML 选择不同的七个主动关节和 TCP。
    all_joint_names = EXPECTED_LEFT_JOINTS + EXPECTED_RIGHT_JOINTS
    for label, robot in (("左臂实例", left_robot), ("右臂实例", right_robot)):
        missing = [
            name for name in all_joint_names
            if not robot.model.existJointName(name)
        ]
        missing_frames = [
            name for name in (EXPECTED_LEFT_TCP, EXPECTED_RIGHT_TCP)
            if not robot.model.existFrame(name)
        ]
        if missing or missing_frames:
            raise RuntimeError(
                f"{label}加载的不是完整双臂 URDF: "
                f"missing_joints={missing}, missing_frames={missing_frames}"
            )

    # 左右臂零附近伸直姿应关于机身 X-Z 平面近似镜像。
    left_start = np.zeros(7, dtype=float)
    left_start[ELBOW_JOINT_INDEX] = -START_ELBOW_BEND
    right_start = -left_start
    left_position = np.asarray(
        left_robot.forward_kinematics(left_start)["position"], dtype=float
    )
    right_position = np.asarray(
        right_robot.forward_kinematics(right_start)["position"], dtype=float
    )
    mirrored_left = left_position * np.array([1.0, -1.0, 1.0])
    mirror_error = float(np.linalg.norm(right_position - mirrored_left))
    if mirror_error > 0.015:
        raise RuntimeError(
            f"左右 URDF FK 镜像误差过大: {mirror_error:.4f} m"
        )
    print(f"URDF 验证通过: {left_urdf}")
    print(f"左右零附近 FK 镜像误差: {mirror_error * 1000.0:.1f} mm")


def validate_selected_urdf(selected_robots):
    """单臂检查本侧配置，双臂额外检查共同 URDF 和镜像 FK。"""
    if len(selected_robots) == 2:
        validate_dual_urdf(
            selected_robots[0][1],
            selected_robots[1][1],
        )
        return

    label, robot, _ = selected_robots[0]
    if label == "左臂":
        validate_arm_urdf(
            robot,
            label,
            EXPECTED_LEFT_JOINTS,
            EXPECTED_LEFT_TCP,
        )
    else:
        validate_arm_urdf(
            robot,
            label,
            EXPECTED_RIGHT_JOINTS,
            EXPECTED_RIGHT_TCP,
        )
    urdf_path = resolved_urdf_path(robot)
    if not os.path.isfile(urdf_path):
        raise RuntimeError(f"URDF 文件不存在: {urdf_path}")
    print(f"{label} URDF 验证通过: {urdf_path}")


def build_arm_state(label, robot, mirror_sign):
    """构建一侧手臂的镜像轨迹与 QP 约束。"""
    start_joint = np.zeros(7, dtype=float)
    start_joint[ELBOW_JOINT_INDEX] = -START_ELBOW_BEND * mirror_sign
    shoulder_margin = 0.05
    if mirror_sign > 0.0:
        shoulder_end = float(robot.joint_limits["lower"][0] + shoulder_margin)
    else:
        shoulder_end = float(robot.joint_limits["upper"][0] - shoulder_margin)
    elbow_branch_sign = -1.0 * mirror_sign
    circle_y_sign = mirror_sign
    return {
        "label": label,
        "robot": robot,
        "start_joint": start_joint,
        "shoulder_end": shoulder_end,
        "elbow_branch_sign": elbow_branch_sign,
        "circle_y_sign": circle_y_sign,
        "command_joint": None,
        "solver": None,
    }


def execute_dual_mit_joint_motion(
    states,
    targets,
    duration,
    label,
    tolerance=0.05,
    settle_timeout=5.0,
):
    """使用同一时钟的七次插值，以 MIT 模式同步移动双臂。"""
    print(f"\n{label}")
    print(f"   到位容差: {tolerance:.3f} rad")
    targets = [np.asarray(target, dtype=float) for target in targets]
    starts = [
        np.asarray(state["robot"].get_current_pos(), dtype=float)
        for state in states
    ]
    for state, target in zip(states, targets):
        robot = state["robot"]
        if target.shape != (robot.motor_count,):
            raise ValueError(f"{state['label']}目标关节维度错误")
        if robot.joint_limits is not None:
            lower = np.asarray(robot.joint_limits["lower"], dtype=float)
            upper = np.asarray(robot.joint_limits["upper"], dtype=float)
            if np.any(target < lower) or np.any(target > upper):
                raise RuntimeError(
                    f"{state['label']}目标超出原始关节限位: {target}"
                )

    start_time = time.perf_counter()
    next_deadline = start_time
    last_print = start_time - 1.0
    final_errors = [float("inf")] * len(states)
    while True:
        now = time.perf_counter()
        elapsed = now - start_time
        trajectory_time = min(elapsed, duration)
        commands = []
        for state, start, target in zip(states, starts, targets):
            robot = state["robot"]
            position, velocity, _ = robot.septic_interpolation(
                start,
                target,
                duration,
                trajectory_time,
            )
            if elapsed >= duration:
                position = target.copy()
                velocity = np.zeros(robot.motor_count, dtype=float)
            commands.append((state, position, velocity))

        # 先完成两臂的计算和限位检查，再近同时下发 MIT。
        for state, position, velocity in commands:
            robot = state["robot"]
            actual_joint = np.asarray(robot.get_current_pos(), dtype=float)
            gravity = np.asarray(robot.get_Gravity(actual_joint), dtype=float)
            torque_limit = np.asarray(robot.max_torque, dtype=float)
            gravity = np.clip(gravity, -torque_limit, torque_limit)
            success = robot.pos_vel_tqe_kp_kd(
                pos=position.tolist(),
                vel=velocity.tolist(),
                tqe=gravity.tolist(),
                kp=MIT_KP.tolist(),
                kd=MIT_KD.tolist(),
            )
            if not success:
                raise RuntimeError(f"{state['label']} MIT 关节指令被拒绝")

        telemetry = []
        final_errors = []
        for (state, position, _), target in zip(commands, targets):
            feedback = read_motor_feedback(state["robot"])
            feedback["command_joint"] = np.asarray(position, dtype=float)
            feedback["actual_joint"] = feedback["position"]
            feedback["actual_velocity"] = feedback["velocity"]
            feedback["actual_torque"] = feedback["torque"]
            telemetry.append(feedback)
            final_errors.append(float(np.max(np.abs(feedback["position"] - target))))
        if now - last_print >= TELEMETRY_PRINT_PERIOD:
            print_realtime_telemetry(
                label,
                min(elapsed / duration, 1.0),
                states,
                telemetry,
            )
            last_print = now

        if elapsed >= duration and all(
            error <= tolerance for error in final_errors
        ):
            print(f"\n   {selected_scope(states)} MIT 运动完成")
            for state, target in zip(states, targets):
                state["command_joint"] = target.copy()
            return
        if elapsed >= duration + settle_timeout:
            details = ", ".join(
                f"{state['label']}最大误差={error:.3f} rad"
                for state, error in zip(states, final_errors)
            )
            raise RuntimeError(f"{selected_scope(states)} MIT 移动超时: {details}")

        next_deadline += CONTROL_PERIOD
        remaining = next_deadline - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            next_deadline = time.perf_counter()


def move_both_to_start(states):
    execute_dual_mit_joint_motion(
        states,
        [state["start_joint"] for state in states],
        duration=8.0,
        label=f"{selected_scope(states)}使用 MIT 移动到零附近伸直姿态",
        tolerance=0.05,
    )
    time.sleep(0.5)


def configure_qp(states):
    for state in states:
        robot = state["robot"]
        command_joint = np.asarray(robot.get_current_pos(), dtype=float)
        if robot.joint_limits is None:
            raise RuntimeError(f"{state['label']}: YAML 未提供关节限位")
        original_lower = np.asarray(
            robot.joint_limits["lower"], dtype=float
        ).copy()
        original_upper = np.asarray(
            robot.joint_limits["upper"], dtype=float
        ).copy()
        solver = PinocchioOSQPIK(
            robot,
            position_weight=100.0,
            rotation_weight=5.0,
            position_gain=6.0,
            rotation_gain=4.0,
            damping=1e-3,
            smoothness=2e-2,
            posture_weight=0.0,
            max_joint_velocity=1.0,
            max_joint_acceleration=4.0,
            max_linear_velocity=0.15,
            max_angular_velocity=0.5,
            singularity_threshold=0.12,
            singularity_damping=8e-2,
        )
        if solver.lower is None or solver.upper is None:
            raise RuntimeError(f"{state['label']}: URDF/YAML 未提供关节限位")
        initial_branch = (
            state["elbow_branch_sign"]
            * command_joint[ELBOW_JOINT_INDEX]
        )
        required_branch_margin = (
            ELBOW_SINGULARITY_MARGIN + solver.joint_limit_margin
        )
        if initial_branch < required_branch_margin:
            raise RuntimeError(
                f"{state['label']}: J4 起始位置距换支点太近 "
                f"({command_joint[ELBOW_JOINT_INDEX]:+.3f} rad)"
            )
        if state["elbow_branch_sign"] < 0.0:
            solver.upper[ELBOW_JOINT_INDEX] = min(
                solver.upper[ELBOW_JOINT_INDEX],
                -ELBOW_SINGULARITY_MARGIN,
            )
        else:
            solver.lower[ELBOW_JOINT_INDEX] = max(
                solver.lower[ELBOW_JOINT_INDEX],
                ELBOW_SINGULARITY_MARGIN,
            )
        if (
            not np.array_equal(robot.joint_limits["lower"], original_lower)
            or not np.array_equal(robot.joint_limits["upper"], original_upper)
        ):
            raise RuntimeError(
                f"{state['label']}: QP 临时限位污染了机器人原始限位"
            )
        solver.reset()
        state["command_joint"] = command_joint
        state["solver"] = solver


def solve_dual_step(states, targets, dt):
    """先求解两侧 QP，通过安全检查后再近同时下发。"""
    pending = []
    for state, target in zip(states, targets):
        result = state["solver"].step(state["command_joint"], target, dt)
        robot = state["robot"]
        feedback = read_motor_feedback(robot)
        actual_joint = feedback["position"]
        tracking_error = float(np.max(np.abs(state["command_joint"] - actual_joint)))
        if tracking_error > 0.25:
            raise RuntimeError(
                f"{state['label']}关节跟踪误差过大: {tracking_error:.3f} rad"
            )
        branch_value = state["elbow_branch_sign"] * result.joint_position[ELBOW_JOINT_INDEX]
        if branch_value < ELBOW_SINGULARITY_MARGIN - 1e-6:
            raise RuntimeError(f"{state['label']} QP 肘部分支约束失效")
        actual_branch = state["elbow_branch_sign"] * actual_joint[ELBOW_JOINT_INDEX]
        if actual_branch <= 0.0:
            raise RuntimeError(f"{state['label']}实体 J4 已到换支点")
        pending.append((state, result, feedback))

    observations = []
    for state, result, feedback in pending:
        robot = state["robot"]
        actual_joint = feedback["position"]
        gravity = np.asarray(robot.get_Gravity(actual_joint), dtype=float)
        torque_limit = np.asarray(robot.max_torque, dtype=float)
        gravity = np.clip(gravity, -torque_limit, torque_limit)
        success = robot.pos_vel_tqe_kp_kd(
            pos=result.joint_position.tolist(),
            vel=[0.0] * robot.motor_count,
            tqe=gravity.tolist(),
            kp=MIT_KP.tolist(),
            kd=MIT_KD.tolist(),
        )
        if not success:
            raise RuntimeError(f"{state['label']} MIT 指令被拒绝")
        previous_command = state["command_joint"]
        state["command_joint"] = result.joint_position.copy()
        actual_transform = np.asarray(
            robot.forward_kinematics(actual_joint)["transform"], dtype=float
        )
        target = targets[len(observations)]
        position_error, rotation_error = pose_error(target, actual_transform)
        singular_value = float(np.linalg.svd(
            np.asarray(robot.get_jacobian(result.joint_position), dtype=float),
            compute_uv=False,
        )[-1])
        observations.append({
            "position_error": position_error,
            "rotation_error": rotation_error,
            "singular_value": singular_value,
            "joint_step": float(np.max(np.abs(result.joint_position - previous_command))),
            "joint_velocity": float(np.max(np.abs(result.joint_velocity))),
            "shoulder": float(result.joint_position[SHOULDER_JOINT_INDEX]),
            "elbow": float(result.joint_position[ELBOW_JOINT_INDEX]),
            "command_joint": result.joint_position.copy(),
            "actual_joint": actual_joint.copy(),
            "actual_velocity": feedback["velocity"].copy(),
            "actual_torque": feedback["torque"].copy(),
            "mode": feedback["mode"],
            "fault": feedback["fault"],
        })
    return observations


def execute_dual_raise(states):
    print(f"\n1. {selected_scope(states)}伸直奇异位形下举到头顶")
    start_time = time.perf_counter()
    previous_time = start_time
    next_deadline = start_time
    last_print = start_time - 1.0
    metrics = [
        {"min_sigma": float("inf"), "max_step": 0.0}
        for _ in states
    ]
    total_duration = SHOULDER_ARC_DURATION + ARC_HOLD_DURATION
    while True:
        now = time.perf_counter()
        elapsed = now - start_time
        if elapsed >= total_duration:
            break
        dt = float(np.clip(
            now - previous_time,
            0.5 * CONTROL_PERIOD,
            2.0 * CONTROL_PERIOD,
        ))
        previous_time = now
        progress = quintic_progress(elapsed / SHOULDER_ARC_DURATION)
        targets = []
        for state in states:
            reference = np.zeros(7, dtype=float)
            reference[SHOULDER_JOINT_INDEX] = (
                progress * state["shoulder_end"]
            )
            targets.append(np.asarray(
                state["robot"].forward_kinematics(reference)["transform"],
                dtype=float,
            ))
        observations = solve_dual_step(states, targets, dt)
        for metric, observation in zip(metrics, observations):
            metric["min_sigma"] = min(
                metric["min_sigma"], observation["singular_value"]
            )
            metric["max_step"] = max(
                metric["max_step"], observation["joint_step"]
            )
        if now - last_print >= TELEMETRY_PRINT_PERIOD:
            print_realtime_telemetry("QP举臂", progress, states, observations)
            last_print = now
        next_deadline += CONTROL_PERIOD
        remaining = next_deadline - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            next_deadline = time.perf_counter()
    print(f"\n\n   {selected_scope(states)}举起结果:")
    for state, metric in zip(states, metrics):
        print(
            f"   - {state['label']}: min_sigma={metric['min_sigma']:.6f}, "
            f"max_dq={np.degrees(metric['max_step']):.3f}°"
        )


def execute_dual_circles(states):
    print(
        f"\n2. {selected_scope(states)}在头顶画圆 "
        f"(半径 {CIRCLE_RADIUS * 100.0:.1f} cm)"
    )
    paths = []
    for state in states:
        start = np.asarray(state["robot"].forward_kinematics(
            state["command_joint"]
        )["transform"], dtype=float)
        overhead = start[:3, 3].copy()
        center = overhead.copy()
        center[2] -= CIRCLE_LOWERING
        entry = center + np.array([CIRCLE_RADIUS, 0.0, 0.0])
        paths.append({
            "start": start,
            "overhead": overhead,
            "center": center,
            "entry": entry,
        })

    start_time = time.perf_counter()
    previous_time = start_time
    next_deadline = start_time
    last_print = start_time - 1.0
    max_errors = [0.0 for _ in states]
    max_steps = [0.0 for _ in states]
    total_duration = (
        CIRCLE_PREP_DURATION + CIRCLE_DURATION + CIRCLE_HOLD_DURATION
    )
    while True:
        now = time.perf_counter()
        elapsed = now - start_time
        if elapsed >= total_duration:
            break
        dt = float(np.clip(
            now - previous_time,
            0.5 * CONTROL_PERIOD,
            2.0 * CONTROL_PERIOD,
        ))
        previous_time = now
        targets = []
        if elapsed < CIRCLE_PREP_DURATION:
            progress = 0.0
            prep = quintic_progress(elapsed / CIRCLE_PREP_DURATION)
            phase = "进入圆周"
            for path in paths:
                target = path["start"].copy()
                target[:3, 3] = (
                    path["overhead"]
                    + prep * (path["entry"] - path["overhead"])
                )
                targets.append(target)
        else:
            progress = quintic_progress(
                (elapsed - CIRCLE_PREP_DURATION) / CIRCLE_DURATION
            )
            angle = 2.0 * np.pi * progress
            phase = "画圆"
            for state, path in zip(states, paths):
                target = path["start"].copy()
                target[:3, 3] = path["center"] + np.array([
                    CIRCLE_RADIUS * np.cos(angle),
                    state["circle_y_sign"]
                    * CIRCLE_RADIUS * np.sin(angle),
                    0.0,
                ])
                targets.append(target)
        observations = solve_dual_step(states, targets, dt)
        for index, observation in enumerate(observations):
            max_errors[index] = max(
                max_errors[index], observation["position_error"]
            )
            max_steps[index] = max(
                max_steps[index], observation["joint_step"]
            )
        if now - last_print >= TELEMETRY_PRINT_PERIOD:
            print_realtime_telemetry(phase, progress, states, observations)
            last_print = now
        next_deadline += CONTROL_PERIOD
        remaining = next_deadline - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            next_deadline = time.perf_counter()
    print(f"\n\n   {selected_scope(states)}头顶画圆结果:")
    for index, state in enumerate(states):
        print(
            f"   - {state['label']}: max_err={max_errors[index]*1000:.1f} mm, "
            f"max_dq={np.degrees(max_steps[index]):.3f}°"
        )


def return_both_to_zero(states):
    execute_dual_mit_joint_motion(
        states,
        [np.zeros(7, dtype=float) for _ in states],
        duration=10.0,
        label=f"3. QP 演示完成，{selected_scope(states)}使用 MIT 回零",
        tolerance=0.05,
        settle_timeout=8.0,
    )
    print(f"   {selected_scope(states)}已使用 MIT 回到零位")


def stop_robot(robot, label):
    if robot is None:
        return
    try:
        robot.set_stop()
    except Exception as error:
        print(f"\n停止{label}电机时发生错误: {error}")


def main(selected_robots):
    print("=" * 68)
    print("openarm_HT QP 奇异位形与头顶画圆例程")
    print("=" * 68)
    print("电机控制模式: 全程 MIT（位置+速度+力矩+Kp+Kd）")
    validate_selected_urdf(selected_robots)
    states = [
        build_arm_state(label, robot, mirror_sign)
        for label, robot, mirror_sign in selected_robots
    ]
    targets = ", ".join(
        f"{state['label']}={state['shoulder_end']:+.4f} rad "
        f"({np.degrees(state['shoulder_end']):+.1f}°)"
        for state in states
    )
    print(f"J1 抬升目标: {targets}")
    if len(states) == 2 and abs(
        abs(states[0]["shoulder_end"]) - abs(states[1]["shoulder_end"])
    ) > 0.1:
        print("警告: 当前 URDF 的左右 J1 限位不对称，右臂抬升幅度会明显小于左臂。")
    print("请确保所选机械臂与头顶周围无障碍物，Ctrl+C 可随时停止。")
    move_both_to_start(states)
    configure_qp(states)
    for state in states:
        elbow_constraint = "J4 <= -0.05 rad" if state["elbow_branch_sign"] < 0 else "J4 >= +0.05 rad"
        print(
            f"{state['label']}约束: J1 -> "
            f"{np.degrees(state['shoulder_end']):.1f}°, "
            f"{elbow_constraint}"
        )
    execute_dual_raise(states)
    time.sleep(0.5)
    execute_dual_circles(states)
    time.sleep(0.5)
    return_both_to_zero(states)
    print("\n所选机械臂 QP 例程执行完成")


if __name__ == "__main__":
    selected_robots = []
    try:
        for label, config_path, mirror_sign in select_arm():
            selected_robots.append(
                (label, Openarm(config_path), mirror_sign)
            )
        main(selected_robots)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as error:
        print(f"\n错误: {error}")
    finally:
        for label, robot, _ in selected_robots:
            stop_robot(robot, label)
        print("所选机械臂所有电机已停止")
