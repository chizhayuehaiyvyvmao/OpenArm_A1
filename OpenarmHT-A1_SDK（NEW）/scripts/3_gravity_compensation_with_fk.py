#!/usr/bin/env python3
"""左臂、右臂或双臂重力补偿，并输出正运动学结果。"""

import os
import time

import numpy as np

from Lib import Openarm


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LEFT_CONFIG = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_left.yaml")
)
RIGHT_CONFIG = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_right.yaml")
)


def select_arm():
    print("\n请选择机械臂：")
    print("  1. 左臂")
    print("  2. 右臂")
    print("  3. 双臂")
    while True:
        choice = input("请输入 1/2/3: ").strip()
        if choice == "1":
            return [("左臂", LEFT_CONFIG)]
        if choice == "2":
            return [("右臂", RIGHT_CONFIG)]
        if choice == "3":
            return [("左臂", LEFT_CONFIG), ("右臂", RIGHT_CONFIG)]
        print("输入无效，请输入 1、2 或 3。")


def rotation_matrix_to_euler(rotation):
    sy = np.sqrt(rotation[0, 0] ** 2 + rotation[1, 0] ** 2)
    if sy >= 1e-6:
        x = np.arctan2(rotation[2, 1], rotation[2, 2])
        y = np.arctan2(-rotation[2, 0], sy)
        z = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        x = np.arctan2(-rotation[1, 2], rotation[1, 1])
        y = np.arctan2(-rotation[2, 0], sy)
        z = 0.0
    return np.degrees([x, y, z])


def print_matrix(matrix, title, precision=3):
    print(f"\n{title}:")
    for row in matrix:
        print("  [" + "  ".join(f"{value:8.{precision}f}" for value in row) + "]")


def run_cycle(label, robot):
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    current_angles = np.asarray(robot.get_current_pos(), dtype=float)
    gravity_torque = robot.get_Gravity(current_angles)
    torque_limit = np.asarray(robot.max_torque, dtype=float)
    gravity_torque = np.clip(gravity_torque, -torque_limit, torque_limit)
    zeros = [0.0] * robot.motor_count
    robot.pos_vel_tqe_kp_kd(zeros, zeros, gravity_torque, zeros, zeros)
    fk = robot.forward_kinematics(current_angles)

    print("\n" + "=" * 80)
    print(f"{label}重力补偿 + 正运动学")
    print("=" * 80)
    angles_deg = np.degrees(current_angles)
    print(f"关节角度 (度): {[f'{angle:7.2f}' for angle in angles_deg]}")
    print(f"重力补偿力矩 (Nm): {[f'{value:7.2f}' for value in gravity_torque]}")
    if fk is not None:
        position = fk["position"]
        rotation = fk["rotation"]
        euler = rotation_matrix_to_euler(rotation)
        print(
            f"末端位置 (m): x={position[0]:8.4f}, "
            f"y={position[1]:8.4f}, z={position[2]:8.4f}"
        )
        print_matrix(rotation, "旋转矩阵 (R)")
        print(
            f"欧拉角 (度): Roll={euler[0]:7.2f}, "
            f"Pitch={euler[1]:7.2f}, Yaw={euler[2]:7.2f}"
        )


def stop_robots(robots):
    for label, robot in robots:
        try:
            robot.set_stop()
        except Exception as error:
            print(f"停止{label}时发生错误: {error}")


def main():
    robots = []
    try:
        for label, config_path in select_arm():
            robots.append((label, Openarm(config_path)))
        while True:
            for label, robot in robots:
                run_cycle(label, robot)
            time.sleep(0.002)
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as error:
        print(f"\n错误: {error}")
    finally:
        stop_robots(robots)
        print("\n所选机械臂电机已停止")


if __name__ == "__main__":
    main()
