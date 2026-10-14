#!/usr/bin/env python3
"""左臂、右臂或双臂 moveJ 控制示例。"""

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

MOVE_TOLERANCE = 0.08
MOVE_TIMEOUT = 15.0


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


def target_for_arm(mirrored, left_target):
    return mirror_left_to_right(left_target) if mirrored else list(left_target)


def wait_for_selected(robots, targets):
    deadline = time.monotonic() + MOVE_TIMEOUT
    last_errors = {label: [float("inf")] for label, _, _ in robots}
    while time.monotonic() < deadline:
        all_reached = True
        for label, robot, _ in robots:
            reached, errors = robot.check_position_reached(
                targets[label], MOVE_TOLERANCE
            )
            last_errors[label] = errors
            all_reached = all_reached and reached
        if all_reached:
            return True, last_errors
        time.sleep(0.02)
    return False, last_errors


def move_selected(robots, name, left_target, duration):
    targets = {}
    print(f"\n{name}")
    for label, robot, mirrored in robots:
        target = target_for_arm(mirrored, left_target)
        targets[label] = target
        print(f"  {label}: {target}")
        commanded = robot.moveJ(
            target,
            duration=duration,
            max_tqu=robot.max_torque,
            iswait=False,
        )
        if not commanded:
            raise RuntimeError(f"{label}命令未成功下发")

    reached, errors = wait_for_selected(robots, targets)
    for label, _, _ in robots:
        print(f"{label}最大误差: {max(errors[label]):.3f} rad")
    if not reached:
        raise RuntimeError(f"{name}未在规定时间内到位")


def stop_robots(robots):
    for label, robot, _ in robots:
        try:
            robot.set_stop()
        except Exception as error:
            print(f"停止{label}时发生错误: {error}")


def run_demo(robots):
    zero_pos = [0.0] * 7
    pos1 = [-0.2, -0.2, 0.2, -0.4, 0.5, 0.2, -0.1]

    for label, robot, _ in robots:
        if robot.motor_count != 7:
            raise ValueError(f"{label}应为 7 关节，实际为 {robot.motor_count}")
        robot.send_get_motor_state_cmd()
        robot.motor_send_cmd()
    time.sleep(0.5)

    move_selected(robots, "1. 回到零位准备", zero_pos, duration=2.0)
    time.sleep(1.0)
    move_selected(robots, "2. 移动到目标位置", pos1, duration=3.0)
    for _, robot, _ in robots:
        robot.gripper_close()
    time.sleep(2.0)
    move_selected(robots, "3. 回到零位", zero_pos, duration=2.0)
    time.sleep(2.0)
    print("\n动作演示完成")


def main():
    robots = []
    try:
        for label, config_path, mirrored in select_arm():
            robots.append((label, Openarm(config_path), mirrored))
        run_demo(robots)
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as error:
        print(f"\n错误: {error}")
    finally:
        stop_robots(robots)
        print("\n所选机械臂电机已停止")


if __name__ == "__main__":
    main()
