#!/usr/bin/env python3
"""左臂、右臂或双臂关节 PD 控制示例。"""

import os
import time

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
            return [("左臂", LEFT_CONFIG, False)]
        if choice == "2":
            return [("右臂", RIGHT_CONFIG, True)]
        if choice == "3":
            return [
                ("左臂", LEFT_CONFIG, False),
                ("右臂", RIGHT_CONFIG, True),
            ]
        print("输入无效，请输入 1、2 或 3。")


def mirror_left_to_right(left_values):
    return [-value for value in left_values]


def stop_robots(robots):
    for label, robot, _ in robots:
        try:
            robot.set_stop()
        except Exception as error:
            print(f"停止{label}时发生错误: {error}")


def main():
    robots = []
    left_target = [-0.2, -0.2, 0.2, -0.4, 0.5, 0.2, -0.1]
    kp = [4.0, 10.0, 10.0, 2.0, 2.0, 1.0, 1.0]
    kd = [0.5, 0.8, 0.8, 0.2, 0.2, 0.1, 0.1]

    try:
        for label, config_path, mirrored in select_arm():
            robot = Openarm(config_path)
            target = mirror_left_to_right(left_target) if mirrored else left_target
            robots.append((label, robot, target))

        while True:
            for label, robot, target in robots:
                zeros = [0.0] * robot.motor_count
                robot.pos_vel_tqe_kp_kd(target, zeros, zeros, kp, kd)
                positions = robot.get_current_pos()
                print(f"{label}关节: {[f'{value:.3f}' for value in positions]}")
            print("-" * 60)
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as error:
        print(f"\n错误: {error}")
    finally:
        stop_robots(robots)
        print("\n所选机械臂电机已停止")


if __name__ == "__main__":
    main()
