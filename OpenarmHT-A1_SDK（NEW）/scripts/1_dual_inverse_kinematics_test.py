#!/usr/bin/env python3
"""左臂、右臂或双臂逆运动学验证；只计算，不下发运动。"""

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


def verify_current_pose(label, robot):
    # 主动请求最新电机反馈，避免一直读取初始化时的缓存状态。
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    current_angles = robot.get_current_pos()
    fk = robot.forward_kinematics(current_angles)
    if fk is None:
        return

    position = fk["position"]
    print(
        f"{label} 当前关节(rad): "
        f"{[f'{angle:.4f}' for angle in current_angles]}"
    )
    print(
        f"{label} TCP位置(m): "
        f"x={position[0]:.4f}, y={position[1]:.4f}, z={position[2]:.4f}"
    )

    ik_angles = robot.inverse_kinematics(
        fk["position"], fk["rotation"], current_angles
    )
    if ik_angles is not None:
        errors = np.abs(current_angles - ik_angles)
        print(f"{label} 最大逆解误差: {np.max(errors):.4f} rad")


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
                verify_current_pose(label, robot)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as error:
        print(f"\n错误: {error}")
    finally:
        stop_robots(robots)
        print("\n所选机械臂电机已停止")


if __name__ == "__main__":
    main()
