#!/usr/bin/env python3
"""左臂、右臂或双臂 SSH 键盘 DLS、零空间锁定与关节 PD 控制。"""

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
relock_requested = False
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
    global target_changed, relock_requested
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
    if char == "r":
        relock_requested = True
    elif char in translations:
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
    global arm_states, running, target_changed, relock_requested
    running = True
    target_changed = False
    relock_requested = False
    arm_states = []
    listener = None
    kp_pd = [25.0, 35.0, 40.0, 20.0, 10.0, 8.0, 8.0]
    kd_pd = [2.5, 3.5, 4.0, 2.0, 1.0, 0.8, 0.8]
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
            state["lock_posture"] = q.copy()

        print("\nW/S:X A/D:Y Q/E:Z 1/2:Rx 3/4:Ry 5/6:Rz R:重锁 ESC:退出")
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        task_gain = np.array([8.0, 8.0, 8.0, 7.0, 7.0, 7.0])
        twist_limit = np.array([0.5, 0.5, 0.5, 2.0, 2.0, 2.0])
        null_k = np.array([1.5, 2.0, 2.0, 1.2, 0.8, 0.6, 0.6])
        null_d = np.array([0.15, 0.20, 0.20, 0.12, 0.08, 0.05, 0.05])
        max_joint_velocity = np.array([1.2, 1.2, 1.2, 1.2, 1.8, 1.8, 1.8])
        last_print = time.monotonic() - 1.0
        while running:
            summaries = []
            for state in arm_states:
                robot = state["robot"]
                q = np.asarray(robot.get_current_pos(), dtype=float)
                q_dot = np.asarray(robot.get_current_vel(), dtype=float)
                if relock_requested:
                    state["lock_posture"] = q.copy()

                fk = robot.forward_kinematics(q)
                current_position = np.asarray(fk["position"], dtype=float)
                current_rotation = np.asarray(fk["rotation"], dtype=float)
                position_error = state["target_position"] - current_position
                rotation_error = Rotation.from_matrix(
                    state["target_rotation"] @ current_rotation.T
                ).as_rotvec()
                desired_twist = np.clip(
                    task_gain * np.concatenate([position_error, rotation_error]),
                    -twist_limit,
                    twist_limit,
                )
                jacobian = robot.get_jacobian(q)
                jacobian_pinv = Openarm.compute_damped_pseudoinverse(
                    jacobian,
                    damping=0.03,
                )
                q_dot_task = jacobian_pinv @ desired_twist
                null_projector = np.eye(robot.motor_count) - jacobian_pinv @ jacobian
                q_dot_null = null_k * (state["lock_posture"] - q) - null_d * q_dot
                q_dot_command = q_dot_task + null_projector @ q_dot_null
                q_dot_command = np.clip(
                    q_dot_command,
                    -max_joint_velocity,
                    max_joint_velocity,
                )
                q_desired = np.clip(
                    q + q_dot_command * 0.01,
                    np.asarray(robot.joint_limits["lower"]),
                    np.asarray(robot.joint_limits["upper"]),
                )
                gravity = np.asarray(robot.get_Gravity(q), dtype=float)
                robot.pos_vel_tqe_kp_kd(
                    q_desired,
                    q_dot_command,
                    gravity,
                    kp_pd,
                    kd_pd,
                )
                summaries.append(
                    f"{state['label']} ep={np.linalg.norm(position_error):.4f} "
                    f"null={np.linalg.norm(state['lock_posture'] - q):.4f}"
                )

            if relock_requested:
                print("\n已重新锁定所选机械臂当前臂角")
                relock_requested = False
            if target_changed:
                target_changed = False
            now = time.monotonic()
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
                kd_pd,
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
