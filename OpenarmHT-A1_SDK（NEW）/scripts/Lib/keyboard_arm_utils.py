#!/usr/bin/env python3
"""键盘控制脚本共用的左右臂选择、镜像和关节准备工具。"""

import os
import time

import numpy as np


# 双臂 VR 镜像初始姿态。右臂 J1/J4 使用镜像限位后，两侧可使用
# 完全反号的关节目标，并得到严格镜像的 TCP 位姿。
LEFT_SAFE_JOINTS = np.array(
    [0.300000, 0.0, 0.0, -1.870000, 0.0, 0.0, 0.0],
    dtype=float,
)
MIRROR_MATRIX = np.diag([1.0, -1.0, 1.0])
TWIST_MIRROR_SIGNS = np.array([1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
JOINT_COMMAND_SPEED = 0.20
JOINT_COMMAND_PERIOD = 0.01
JOINT_MOVE_TOLERANCE = 0.08
JOINT_MOVE_TIMEOUT = 25.0


def select_arm_specs(script_dir):
    left_config = os.path.normpath(
        os.path.join(script_dir, "..", "robot_param", "Openarm_left.yaml")
    )
    right_config = os.path.normpath(
        os.path.join(script_dir, "..", "robot_param", "Openarm_right.yaml")
    )
    print("\n请选择机械臂：")
    print("  1. 左臂")
    print("  2. 右臂")
    print("  3. 双臂")
    while True:
        choice = input("请输入 1/2/3: ").strip()
        if choice == "1":
            return [("左臂", left_config, 1.0)]
        if choice == "2":
            return [("右臂", right_config, -1.0)]
        if choice == "3":
            return [
                ("左臂", left_config, 1.0),
                ("右臂", right_config, -1.0),
            ]
        print("输入无效，请输入 1、2 或 3。")


def create_arm_states(openarm_class, script_dir):
    states = []
    for label, config_path, mirror_sign in select_arm_specs(script_dir):
        states.append({
            "label": label,
            "robot": openarm_class(config_path),
            "mirror_sign": mirror_sign,
        })
    return states


def mirror_translation(vector, mirror_sign):
    vector = np.asarray(vector, dtype=float)
    return vector if mirror_sign > 0.0 else MIRROR_MATRIX @ vector


def mirror_rotation(rotation, mirror_sign):
    rotation = np.asarray(rotation, dtype=float)
    return rotation if mirror_sign > 0.0 else MIRROR_MATRIX @ rotation @ MIRROR_MATRIX


def mirror_twist(twist, mirror_sign):
    twist = np.asarray(twist, dtype=float)
    return twist if mirror_sign > 0.0 else TWIST_MIRROR_SIGNS * twist


def safe_targets(states):
    return {
        state["label"]: LEFT_SAFE_JOINTS * state["mirror_sign"]
        for state in states
    }


def zero_targets(states):
    return {
        state["label"]: np.zeros(state["robot"].motor_count, dtype=float)
        for state in states
    }


def move_selected_joints(states, targets, description):
    print(f"\n{description}")
    deadline = time.monotonic() + JOINT_MOVE_TIMEOUT
    next_deadline = time.perf_counter()
    last_errors = {
        state["label"]: [float("inf")] * state["robot"].motor_count
        for state in states
    }
    reached = False

    while time.monotonic() < deadline:
        reached = True
        for state in states:
            label = state["label"]
            robot = state["robot"]
            target = np.asarray(targets[label], dtype=float)
            success = robot.Joint_Pos_Vel(
                target,
                [JOINT_COMMAND_SPEED] * robot.motor_count,
                robot.max_torque,
                iswait=False,
            )
            if not success:
                raise RuntimeError(f"{label}{description}命令未成功下发")
            actual = np.asarray(robot.get_current_pos(), dtype=float)
            errors = np.abs(actual - target).tolist()
            last_errors[label] = errors
            reached = reached and max(errors) <= JOINT_MOVE_TOLERANCE
        if reached:
            break
        next_deadline += JOINT_COMMAND_PERIOD
        remaining = next_deadline - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            next_deadline = time.perf_counter()

    for state in states:
        label = state["label"]
        print(f"{label}最大到位误差: {max(last_errors[label]):.3f} rad")
    if not reached:
        details = []
        for state in states:
            label = state["label"]
            values = ", ".join(
                f"J{index + 1}={error:.3f}"
                for index, error in enumerate(last_errors[label])
            )
            details.append(f"{label}[{values}]")
        raise RuntimeError(f"{description}超时: " + "; ".join(details))


def stop_states(states):
    for state in states:
        try:
            state["robot"].set_stop()
        except Exception as error:
            print(f"停止{state['label']}时发生错误: {error}")
