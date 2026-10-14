#!/usr/bin/env python3
"""左臂、右臂或双臂重力补偿。"""

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

GRAVITY_COMPENSATION_SCALE = 0.8


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


def run_gravity(robot):
    zeros = [0.0] * robot.motor_count
    torque = GRAVITY_COMPENSATION_SCALE * robot.get_Gravity()
    torque = np.clip(
        torque,
        -np.asarray(robot.max_torque, dtype=float),
        np.asarray(robot.max_torque, dtype=float),
    )
    robot.pos_vel_tqe_kp_kd(zeros, zeros, torque, zeros, zeros)
    robot.gripper_control_MIT(0, 0, 0, 0, 0)
    return torque


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
                torque = run_gravity(robot)
                print(f"{label}重力补偿力矩: {torque}")
            print("-" * 60)
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
