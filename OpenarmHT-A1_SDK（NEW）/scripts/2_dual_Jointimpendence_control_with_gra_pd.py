#!/usr/bin/env python3
"""左臂、右臂或双臂关节阻抗与重力补偿。"""

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

K = np.array([15.0, 15.0, 10.0, 10.0, 5.0, 3.0, 5.0])
B = np.array([0.5, 0.8, 0.8, 0.2, 0.2, 0.1, 0.1])
LEFT_Q_DES = np.array([-0.2, -0.2, 0.2, -0.4, 0.5, 0.2, -0.1])


def select_arm():
    print("\n请选择机械臂：")
    print("  1. 左臂")
    print("  2. 右臂")
    print("  3. 双臂")
    while True:
        choice = input("请输入 1/2/3: ").strip()
        if choice == "1":
            return [("左臂", LEFT_CONFIG, False)]
        if choice == "2":
            return [("右臂", RIGHT_CONFIG, True)]
        if choice == "3":
            return [
                ("左臂", LEFT_CONFIG, False),
                ("右臂", RIGHT_CONFIG, True),
            ]
        print("输入无效，请输入 1、2 或 3。")


def run_impedance(robot, desired_position):
    zeros = np.zeros(robot.motor_count)
    current_position = robot.get_current_pos()
    current_velocity = robot.get_current_vel()
    impedance = K * (desired_position - current_position) + B * (
        zeros - current_velocity
    )
    gravity = robot.get_Gravity()
    total = impedance + gravity
    torque_limit = np.asarray(robot.max_torque, dtype=float)
    total = np.clip(total, -torque_limit, torque_limit)
    robot.pos_vel_tqe_kp_kd(zeros, zeros, total, zeros, zeros)
    return impedance, gravity, total


def stop_robots(robots):
    for label, robot, _ in robots:
        try:
            robot.set_stop()
        except Exception as error:
            print(f"停止{label}时发生错误: {error}")


def main():
    robots = []
    try:
        for label, config_path, mirrored in select_arm():
            desired = -LEFT_Q_DES if mirrored else LEFT_Q_DES.copy()
            robots.append((label, Openarm(config_path), desired))

        while True:
            for label, robot, desired in robots:
                _, _, total = run_impedance(robot, desired)
                print(f"{label}总力矩: {[f'{value:.3f}' for value in total]}")
            print("-" * 60)
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as error:
        print(f"\n错误: {error}")
    finally:
        stop_robots(robots)
        print("\n所选机械臂电机已停止")


if __name__ == "__main__":
    main()
