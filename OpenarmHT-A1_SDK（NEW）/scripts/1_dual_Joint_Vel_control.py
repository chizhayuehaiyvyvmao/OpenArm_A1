#!/usr/bin/env python3
"""左臂、右臂或双臂第一关节速度控制示例。"""

import math
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
            return [("左臂", LEFT_CONFIG, 1.0)]
        if choice == "2":
            return [("右臂", RIGHT_CONFIG, -1.0)]
        if choice == "3":
            return [
                ("左臂", LEFT_CONFIG, 1.0),
                ("右臂", RIGHT_CONFIG, -1.0),
            ]
        print("输入无效，请输入 1、2 或 3。")


def stop_robots(robots):
    for label, robot, _ in robots:
        try:
            robot.set_stop()
        except Exception as error:
            print(f"停止{label}时发生错误: {error}")


def main():
    robots = []
    try:
        for label, config_path, direction in select_arm():
            robots.append((label, Openarm(config_path), direction))

        while True:
            phase = 1.0 if (math.floor(time.time()) % 6) >= 3 else -1.0
            for label, robot, direction in robots:
                target_velocity = [0.2 * phase * direction] + [0.0] * (
                    robot.motor_count - 1
                )
                robot.Joint_Vel(target_velocity)
                print(
                    f"{label}目标速度: {target_velocity[0]:.2f} rad/s, "
                    f"当前位置: {robot.get_current_pos()[0]:.3f} rad"
                )
            print("-" * 40)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as error:
        print(f"\n错误: {error}")
    finally:
        stop_robots(robots)
        print("\n所选机械臂电机已停止")


if __name__ == "__main__":
    main()
