#!/usr/bin/env python3
"""左臂、右臂或双臂位置速度动作演示。"""

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

MOVE_SPEED = 0.5
MOVE_TOLERANCE = 0.08
MOVE_TIMEOUT = 20.0
GRIPPER_OPEN_POS = 1.6
GRIPPER_CLOSE_POS = 0.0
GRIPPER_SPEED = 0.8
GRIPPER_MAX_TORQUE = 0.5

# URDF 中左右臂关于机身 XZ 平面镜像。结合关节轴和 TCP FK，
# J1~J7 的右臂关节量均取左臂关节量的相反数。
RIGHT_MIRROR_SIGNS = (-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0)


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
    if len(left_values) != len(RIGHT_MIRROR_SIGNS):
        raise ValueError(
            f"镜像映射需要 {len(RIGHT_MIRROR_SIGNS)} 个关节量，"
            f"实际为 {len(left_values)}"
        )
    return [
        0.0 if value == 0.0 else sign * value
        for sign, value in zip(RIGHT_MIRROR_SIGNS, left_values)
    ]


def target_for_arm(mirrored, left_target):
    return mirror_left_to_right(left_target) if mirrored else list(left_target)


def validate_target(robot, arm_name, target):
    if len(target) != robot.motor_count:
        raise ValueError(
            f"{arm_name}目标关节数为 {len(target)}，"
            f"机械臂关节数为 {robot.motor_count}"
        )
    if robot.joint_limits is None:
        return

    lower = robot.joint_limits["lower"]
    upper = robot.joint_limits["upper"]
    invalid = [
        (index + 1, value, lower[index], upper[index])
        for index, value in enumerate(target)
        if value < lower[index] or value > upper[index]
    ]
    if invalid:
        details = ", ".join(
            f"J{index}={value:.3f} 不在 [{low:.3f}, {high:.3f}]"
            for index, value, low, high in invalid
        )
        raise ValueError(f"{arm_name}目标超出关节限位: {details}")


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


def move_selected_to(robots, name, left_target, hold=0.8):
    targets = {}
    print(f"\n{name}")
    for label, robot, mirrored in robots:
        target = target_for_arm(mirrored, left_target)
        validate_target(robot, label, target)
        targets[label] = target
        print(f"  {label}: {target}")

    for label, robot, _ in robots:
        success = robot.Joint_Pos_Vel(
            targets[label],
            [MOVE_SPEED] * robot.motor_count,
            robot.max_torque,
            iswait=False,
        )
        if not success:
            raise RuntimeError(f"{name}的{label}命令未成功下发")

    reached, errors = wait_for_selected(robots, targets)
    if not reached:
        details = "，".join(
            f"{label}最大误差 {max(errors[label]):.3f} rad"
            for label, _, _ in robots
        )
        raise RuntimeError(f"{name}未在规定时间内到位：{details}")
    time.sleep(hold)


def grippers_open_close(robots, name):
    print(f"\n{name}: 打开夹爪")
    for _, robot, _ in robots:
        robot.gripper_open(
            pos=GRIPPER_OPEN_POS,
            vel=GRIPPER_SPEED,
            max_tqu=GRIPPER_MAX_TORQUE,
        )
    time.sleep(2.3)

    print(f"{name}: 闭合夹爪")
    for _, robot, _ in robots:
        robot.gripper_close(
            pos=GRIPPER_CLOSE_POS,
            vel=GRIPPER_SPEED,
            max_tqu=GRIPPER_MAX_TORQUE,
        )
    time.sleep(2.3)


def run_demo(robots):
    zero_pos = [0.0] * 7
    fold_lower_pos = [0.3, 0.0, 0.0, -0.9, 0.0, 0.0, 0.0]
    fold_lift_wrist_up_pos = [-0.7, 0.0, 0.0, -0.9, 0.0, 0.0, -0.8]
    forward_pos = [-0.4, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0]

    for label, robot, _ in robots:
        if robot.motor_count != len(RIGHT_MIRROR_SIGNS):
            raise ValueError(f"{label}应为 7 关节，实际为 {robot.motor_count}")
        robot.send_get_motor_state_cmd()
        robot.motor_send_cmd()
    time.sleep(0.5)

    move_selected_to(robots, "1. 回到零位准备", zero_pos, hold=1.0)
    for _, robot, _ in robots:
        robot.gripper_close(
            pos=GRIPPER_CLOSE_POS,
            vel=GRIPPER_SPEED,
            max_tqu=GRIPPER_MAX_TORQUE,
        )
    time.sleep(2.0)

    move_selected_to(robots, "2. 手臂折叠", fold_lower_pos)
    move_selected_to(
        robots,
        "3. 折叠状态抬起并上翘手腕",
        fold_lift_wrist_up_pos,
        hold=1.2,
    )
    grippers_open_close(robots, "4. 抬起位置")
    move_selected_to(robots, "5. 手臂前伸", forward_pos, hold=1.0)
    grippers_open_close(robots, "6. 前伸位置")
    move_selected_to(robots, "7. 收回手臂", fold_lower_pos)
    move_selected_to(robots, "8. 回到零位", zero_pos, hold=2.0)
    print("\n动作演示完成")


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
