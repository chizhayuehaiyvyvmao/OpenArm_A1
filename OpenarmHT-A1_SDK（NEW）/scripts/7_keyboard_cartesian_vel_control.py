#!/usr/bin/env python3
"""左臂、右臂或双臂 SSH 键盘笛卡尔速度控制。"""

import os
import time

import numpy as np

from Lib import Openarm
from Lib.keyboard_arm_utils import (
    create_arm_states,
    mirror_twist,
    move_selected_joints,
    safe_targets,
    stop_states,
    zero_targets,
)
from Lib.terminal_keyboard import keyboard


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
end_velocity = np.zeros(6)
linear_speed = 0.3
angular_speed = 2.0
acceleration_factor = 0.02
running = True
keys_pressed = set()


def on_press(key):
    global end_velocity
    if hasattr(key, "char") and key.char:
        char = key.char.lower()
        keys_pressed.add(char)
        mapping = {
            "w": (0, linear_speed),
            "s": (0, -linear_speed),
            "a": (1, linear_speed),
            "d": (1, -linear_speed),
            "q": (2, linear_speed),
            "e": (2, -linear_speed),
            "1": (3, angular_speed),
            "2": (3, -angular_speed),
            "3": (4, angular_speed),
            "4": (4, -angular_speed),
            "5": (5, angular_speed),
            "6": (5, -angular_speed),
        }
        if char in mapping:
            index, value = mapping[char]
            end_velocity[index] = value


def on_release(key):
    global running
    if key == keyboard.Key.esc:
        running = False
        print("\n检测到 ESC，准备退出...")
        return False
    if hasattr(key, "char") and key.char:
        char = key.char.lower()
        keys_pressed.discard(char)
        for choices, index in (
            (("w", "s"), 0),
            (("a", "d"), 1),
            (("q", "e"), 2),
            (("1", "2"), 3),
            (("3", "4"), 4),
            (("5", "6"), 5),
        ):
            if char in choices:
                end_velocity[index] = 0.0


def main():
    global running, end_velocity
    running = True
    end_velocity[:] = 0.0
    states = []
    listener = None
    kd = [10.0, 15.0, 15.0, 10.0, 5.0, 5.0, 5.0]
    try:
        states = create_arm_states(Openarm, SCRIPT_DIR)
        for state in states:
            if state["robot"].model is None:
                raise RuntimeError(f"{state['label']}未加载 Pinocchio 模型")
            state["actual_velocity"] = np.zeros(6)

        move_selected_joints(states, safe_targets(states), "移动到镜像安全姿态")
        print("\nW/S:X  A/D:Y  Q/E:Z  1/2:Rx  3/4:Ry  5/6:Rz  ESC:退出")
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        control_period = 0.01
        last_print = time.monotonic() - 1.0
        while running:
            summaries = []
            for state in states:
                robot = state["robot"]
                q = np.asarray(robot.get_current_pos(), dtype=float)
                jacobian = robot.get_jacobian(q)
                manipulability = robot.get_manipulability(q)
                damping = 0.01
                speed_scale = 1.0
                if manipulability < 0.01:
                    damping *= 1.0 + (0.01 - manipulability) * 100.0
                    speed_scale = max(0.0, manipulability / 0.01)

                desired_twist = mirror_twist(
                    end_velocity * speed_scale,
                    state["mirror_sign"],
                )
                state["actual_velocity"] += (
                    desired_twist - state["actual_velocity"]
                ) * acceleration_factor
                q_dot = Openarm.compute_damped_pseudoinverse(
                    jacobian,
                    damping,
                ) @ state["actual_velocity"]
                q_dot = np.clip(
                    q_dot,
                    -np.array([2.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0]),
                    np.array([2.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0]),
                )
                robot.Joint_Vel(q_dot)
                position = robot.forward_kinematics(q)["position"]
                summaries.append(
                    f"{state['label']} pos={np.round(position, 3)} "
                    f"v={np.linalg.norm(state['actual_velocity'][:3]):.3f}"
                )

            now = time.monotonic()
            if now - last_print >= 0.2:
                print("\r" + " | ".join(summaries), end="")
                last_print = now
            time.sleep(control_period)
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        if listener is not None:
            listener.stop()
        end_velocity[:] = 0.0
        for state in states:
            robot = state["robot"]
            q = np.asarray(robot.get_current_pos(), dtype=float)
            gravity = np.asarray(robot.get_Gravity(q), dtype=float)
            robot.pos_vel_tqe_kp_kd(
                q,
                [0.0] * robot.motor_count,
                gravity,
                [0.0] * robot.motor_count,
                kd,
            )
        time.sleep(0.3)
        if states:
            try:
                move_selected_joints(states, zero_targets(states), "回到零位")
            finally:
                stop_states(states)
        print("\n所选机械臂已停止")


if __name__ == "__main__":
    main()
