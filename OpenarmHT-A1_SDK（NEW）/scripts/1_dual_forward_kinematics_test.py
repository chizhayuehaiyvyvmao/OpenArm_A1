#!/usr/bin/env python3
"""左臂、右臂或双臂正运动学测试。"""

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


def rotation_matrix_to_euler(rot):
    sy = np.sqrt(rot[0, 0] * rot[0, 0] + rot[1, 0] * rot[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(rot[2, 1], rot[2, 2])
        y = np.arctan2(-rot[2, 0], sy)
        z = np.arctan2(rot[1, 0], rot[0, 0])
    else:
        x = np.arctan2(-rot[1, 2], rot[1, 1])
        y = np.arctan2(-rot[2, 0], sy)
        z = 0.0
    return np.degrees([x, y, z])


def print_fk(label, robot):
    # 主动请求最新电机反馈，避免手动移动时仍显示缓存位置。
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    current_angles = robot.get_current_pos()
    fk = robot.forward_kinematics(current_angles)
    if fk is None:
        return
    euler = rotation_matrix_to_euler(fk["rotation"])
    pos = fk["position"]
    print(
        f"{label}: pos=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m, "
        f"rpy=({euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}) deg"
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
            print("\n" + "=" * 60)
            for label, robot in robots:
                print_fk(label, robot)
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
