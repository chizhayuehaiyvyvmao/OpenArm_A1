#!/usr/bin/env python3
"""左臂、右臂或双臂 SSH 键盘末端阻抗控制。"""

import os
import time

import numpy as np
from scipy.spatial.transform import Rotation

from Lib import Openarm
from Lib.keyboard_arm_utils import (
    create_arm_states,
    mirror_rotation,
    mirror_translation,
    move_selected_joints,
    safe_targets,
    stop_states,
    zero_targets,
)
from Lib.terminal_keyboard import keyboard


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
arm_states = []
running = True
target_changed = False
position_delta = 0.005
rotation_delta = np.deg2rad(3.0)


def rotation_matrix_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rotation_matrix_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotation_matrix_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def on_press(key):
    global target_changed
    if not hasattr(key, "char") or not key.char:
        return
    char = key.char.lower()
    translations = {
        "w": [position_delta, 0.0, 0.0],
        "s": [-position_delta, 0.0, 0.0],
        "a": [0.0, position_delta, 0.0],
        "d": [0.0, -position_delta, 0.0],
        "q": [0.0, 0.0, position_delta],
        "e": [0.0, 0.0, -position_delta],
    }
    rotations = {
        "1": rotation_matrix_x(rotation_delta),
        "2": rotation_matrix_x(-rotation_delta),
        "3": rotation_matrix_y(rotation_delta),
        "4": rotation_matrix_y(-rotation_delta),
        "5": rotation_matrix_z(rotation_delta),
        "6": rotation_matrix_z(-rotation_delta),
    }
    if char in translations:
        for state in arm_states:
            state["target_position"] += mirror_translation(
                translations[char],
                state["mirror_sign"],
            )
        target_changed = True
    elif char in rotations:
        for state in arm_states:
            increment = mirror_rotation(rotations[char], state["mirror_sign"])
            state["target_rotation"] = increment @ state["target_rotation"]
        target_changed = True


def on_release(key):
    global running
    if key == keyboard.Key.esc:
        running = False
        print("\n检测到 ESC，准备退出...")
        return False


def main():
    global arm_states, running, target_changed
    running = True
    target_changed = False
    arm_states = []
    listener = None
    kd_hold = [0.5, 0.8, 0.8, 0.2, 0.2, 0.1, 0.1]
    try:
        arm_states = create_arm_states(Openarm, SCRIPT_DIR)
        move_selected_joints(
            arm_states,
            safe_targets(arm_states),
            "移动到镜像安全姿态",
        )
        for state in arm_states:
            robot = state["robot"]
            q = np.asarray(robot.get_current_pos(), dtype=float)
            fk = robot.forward_kinematics(q)
            state["target_position"] = np.asarray(fk["position"], dtype=float)
            state["target_rotation"] = np.asarray(fk["rotation"], dtype=float)
            state["nullspace_reference"] = q.copy()

        print("\nW/S:X  A/D:Y  Q/E:Z  1/2:Rx  3/4:Ry  5/6:Rz  ESC:退出")
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        cartesian_k = np.array([80.0, 80.0, 80.0, 2.0, 2.0, 2.0])
        cartesian_b = np.array([0.50, 0.50, 0.50, 0.10, 0.10, 0.10])
        wrench_limit = np.array([60.0, 60.0, 60.0, 8.0, 8.0, 8.0])
        last_print = time.monotonic() - 1.0
        while running:
            summaries = []
            for state in arm_states:
                robot = state["robot"]
                q = np.asarray(robot.get_current_pos(), dtype=float)
                q_dot = np.asarray(robot.get_current_vel(), dtype=float)
                fk = robot.forward_kinematics(q)
                current_position = np.asarray(fk["position"], dtype=float)
                current_rotation = np.asarray(fk["rotation"], dtype=float)
                jacobian = robot.get_jacobian(q)
                task_velocity = jacobian @ q_dot
                position_error = state["target_position"] - current_position
                rotation_error = Rotation.from_matrix(
                    state["target_rotation"] @ current_rotation.T
                ).as_rotvec()
                task_error = np.concatenate([position_error, rotation_error])
                wrench = cartesian_k * task_error - cartesian_b * task_velocity
                wrench = np.clip(wrench, -wrench_limit, wrench_limit)
                torque = jacobian.T @ wrench + np.asarray(robot.get_Gravity(q))
                torque = np.clip(
                    torque,
                    -np.asarray(robot.max_torque),
                    np.asarray(robot.max_torque),
                )
                zeros = [0.0] * robot.motor_count
                robot.pos_vel_tqe_kp_kd(zeros, zeros, torque, zeros, zeros)
                summaries.append(
                    f"{state['label']} ep={np.linalg.norm(position_error):.4f} "
                    f"er={np.linalg.norm(rotation_error):.4f}"
                )

            now = time.monotonic()
            if target_changed:
                target_changed = False
                print("\n目标位姿已更新")
            if now - last_print >= 0.2:
                print("\r" + " | ".join(summaries), end="")
                last_print = now
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        if listener is not None:
            listener.stop()
        for state in arm_states:
            robot = state["robot"]
            q = np.asarray(robot.get_current_pos(), dtype=float)
            gravity = np.asarray(robot.get_Gravity(q), dtype=float)
            robot.pos_vel_tqe_kp_kd(
                q,
                [0.0] * robot.motor_count,
                gravity,
                [0.0] * robot.motor_count,
                kd_hold,
            )
        time.sleep(0.3)
        if arm_states:
            try:
                move_selected_joints(arm_states, zero_targets(arm_states), "回到零位")
            finally:
                stop_states(arm_states)
        print("\n所选机械臂已停止")


if __name__ == "__main__":
    main()
