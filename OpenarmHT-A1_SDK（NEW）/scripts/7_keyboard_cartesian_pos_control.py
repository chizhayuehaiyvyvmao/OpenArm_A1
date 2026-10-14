#!/usr/bin/env python3
"""左臂、右臂或双臂 SSH 键盘笛卡尔增量位置控制。"""

import os
import time

import numpy as np

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
position_changed = False
position_delta = 0.005
rotation_delta = 0.03


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
    global position_changed
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
        position_changed = True
    elif char in rotations:
        for state in arm_states:
            increment = mirror_rotation(rotations[char], state["mirror_sign"])
            state["target_rotation"] = state["target_rotation"] @ increment
        position_changed = True


def on_release(key):
    global running
    if key == keyboard.Key.esc:
        running = False
        print("\n检测到 ESC，准备退出...")
        return False


def main():
    global arm_states, running, position_changed
    running = True
    position_changed = False
    arm_states = []
    listener = None
    kp = [30.0, 50.0, 60.0, 25.0, 15.0, 10.0, 10.0]
    kd = [3.0, 5.0, 6.0, 2.5, 1.5, 1.0, 1.0]
    try:
        arm_states = create_arm_states(Openarm, SCRIPT_DIR)
        move_selected_joints(
            arm_states,
            safe_targets(arm_states),
            "移动到镜像安全姿态",
        )
        for state in arm_states:
            robot = state["robot"]
            robot.send_get_motor_state_cmd()
            robot.motor_send_cmd()
            joint = np.asarray(robot.get_current_pos(), dtype=float)
            fk = robot.forward_kinematics(joint)
            state["target_position"] = np.asarray(fk["position"], dtype=float)
            state["target_rotation"] = np.asarray(fk["rotation"], dtype=float)
            state["last_valid_joint"] = joint

        print("\nW/S:X  A/D:Y  Q/E:Z  1/2:Rx  3/4:Ry  5/6:Rz  ESC:退出")
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        last_print = time.monotonic() - 1.0
        while running:
            if position_changed:
                for state in arm_states:
                    robot = state["robot"]
                    solution = robot.inverse_kinematics(
                        state["target_position"].tolist(),
                        state["target_rotation"],
                        state["last_valid_joint"],
                        multi_init=False,
                    )
                    if solution is None:
                        print(f"\n{state['label']}逆运动学无解，保持上一目标")
                        continue
                    gravity = np.asarray(robot.get_Gravity(solution), dtype=float)
                    robot.pos_vel_tqe_kp_kd(
                        solution,
                        [0.0] * robot.motor_count,
                        gravity,
                        kp,
                        kd,
                    )
                    state["last_valid_joint"] = solution
                position_changed = False

            now = time.monotonic()
            if now - last_print >= 0.2:
                summaries = []
                for state in arm_states:
                    current = state["robot"].forward_kinematics()["position"]
                    summaries.append(
                        f"{state['label']} target={np.round(state['target_position'], 3)} "
                        f"actual={np.round(current, 3)}"
                    )
                print("\r" + " | ".join(summaries), end="")
                last_print = now
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        if listener is not None:
            listener.stop()
        if arm_states:
            try:
                move_selected_joints(arm_states, zero_targets(arm_states), "回到零位")
            finally:
                stop_states(arm_states)
        print("\n所选机械臂已停止")


if __name__ == "__main__":
    main()
