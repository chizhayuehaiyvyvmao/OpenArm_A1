#!/usr/bin/env python3
"""
Openarm 单臂/双臂设置零点、状态读取和回零测试

用法：
    python 0_dual_robot_set_zero.py
"""

import os
import time

from Lib import Openarm


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LEFT_CONFIG = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_left.yaml"))
RIGHT_CONFIG = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_right.yaml"))

RETURN_SPEED = 0.20
MOVE_TOLERANCE = 0.08
MOVE_TIMEOUT = 15.0
MOVE_TIMEOUT_MARGIN = 5.0

ARM_CHOICES = {
    "1": "left",
    "2": "right",
    "3": "both",
    "left": "left",
    "right": "right",
    "both": "both",
    "左": "left",
    "右": "right",
    "双": "both",
}


def select_arm():
    """交互选择要设置零位的机械臂。"""
    print("\n请选择要设置零位的机械臂：")
    print("  1. 左臂")
    print("  2. 右臂")
    print("  3. 双臂")

    while True:
        try:
            choice = input("请输入 1/2/3: ").strip().lower()
        except EOFError as exc:
            raise SystemExit("未读取到机械臂选择") from exc

        selected = ARM_CHOICES.get(choice)
        if selected is not None:
            return selected
        print("输入无效，请输入 1、2 或 3。")


def print_robot_state(label, robot):
    """打印机器人状态信息"""
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    positions = robot.get_current_pos()
    velocities = robot.get_current_vel()
    gripper_state = robot.get_current_state_gripper()

    print("\n" + "=" * 50)
    print(f"{label} 状态信息")
    print("=" * 50)

    for i in range(robot.motor_count):
        print(f"关节{i+1}: 位置={positions[i]:7.3f} rad, 速度={velocities[i]:7.3f} rad/s")

    print(f"夹爪:   位置={gripper_state.position:7.3f} rad, 速度={gripper_state.velocity:7.3f} rad/s")


def print_selected_states(robots):
    """读取并打印所有已选择机械臂的状态。"""
    for label, robot in robots:
        print_robot_state(label, robot)


def monitor_states(robots):
    """连续读取状态，Ctrl+C 仅返回操作菜单。"""
    print("\n开始连续读取状态；按 Ctrl+C 返回操作菜单。")
    try:
        while True:
            print_selected_states(robots)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n已停止连续读取，返回操作菜单。")


def return_to_zero(robots):
    """让已选择的机械臂回到刚刚设置的关节零位。"""
    targets = {}
    start_positions = {}
    for label, robot in robots:
        robot.send_get_motor_state_cmd()
        robot.motor_send_cmd()
        targets[label] = [0.0] * robot.motor_count
        start_positions[label] = robot.get_current_pos()

    for label, robot in robots:
        # pos_vel_MAXtqe 的 velocity 参数是正的速度上限，不是带方向的速度。
        # 目标位置本身决定运动方向；这里不使用 moveJ 生成的有符号速度。
        commanded = robot.Joint_Pos_Vel(
            targets[label],
            [RETURN_SPEED] * robot.motor_count,
            max_tqu=robot.max_torque,
            iswait=False,
        )
        if not commanded:
            for _, selected_robot in robots:
                selected_robot.set_stop()
            print(f"{label}回零命令未成功下发，所选机械臂已刹车。")
            return False

    max_distance = max(
        abs(position)
        for positions in start_positions.values()
        for position in positions
    )
    timeout = max(MOVE_TIMEOUT, max_distance / RETURN_SPEED + MOVE_TIMEOUT_MARGIN)
    print(
        f"已下发回零命令：速度上限 {RETURN_SPEED:.2f} rad/s，"
        f"到位容差 {MOVE_TOLERANCE:.3f} rad，超时 {timeout:.1f} s"
    )

    deadline = time.monotonic() + timeout
    last_errors = {label: [float("inf")] for label, _ in robots}
    reached = False
    while time.monotonic() < deadline:
        reached = True
        for label, robot in robots:
            arm_reached, errors = robot.check_position_reached(
                targets[label], MOVE_TOLERANCE
            )
            last_errors[label] = errors
            reached = reached and arm_reached
        if reached:
            break
        time.sleep(0.02)

    for label, _ in robots:
        print(
            f"{label}回零最大误差: {max(last_errors[label]):.3f} rad "
            f"(容差 {MOVE_TOLERANCE:.3f} rad)"
        )
    if not reached:
        for _, robot in robots:
            robot.set_stop()
        print("回零超时：所选机械臂已刹车，请检查上面的误差和机械状态。")
        print_selected_states(robots)
        return False

    print("所选机械臂已回到零位。")
    print_selected_states(robots)
    return True


def set_current_as_zero(robots):
    """把已选择机械臂的当前姿态设置为零位。"""
    for label, robot in robots:
        print(f"正在设置{label}零位...")
        robot.set_reset_zero()
        robot.motor_send_cmd()
    time.sleep(1)
    print("所选机械臂零位设置完成。")
    print_selected_states(robots)


def operation_menu(robots):
    """状态读取、设置零位和回零菜单。"""
    while True:
        print("\n请选择操作：")
        print("  1. 读取一次状态")
        print("  2. 连续读取状态")
        print("  3. 把当前姿态设置为零位")
        print("  4. 回到已保存的零位")
        print("  0. 退出")
        choice = input("请输入 0/1/2/3/4: ").strip()

        if choice == "1":
            print_selected_states(robots)
        elif choice == "2":
            monitor_states(robots)
        elif choice == "3":
            set_current_as_zero(robots)
        elif choice == "4":
            return_to_zero(robots)
        elif choice == "0":
            print("程序结束。")
            return
        else:
            print("输入无效，请输入 0、1、2、3 或 4。")


def main():
    selected_arm = select_arm()
    selected_configs = []
    if selected_arm in ("left", "both"):
        selected_configs.append(("左臂", LEFT_CONFIG))
    if selected_arm in ("right", "both"):
        selected_configs.append(("右臂", RIGHT_CONFIG))

    print("=" * 50)
    print("Openarm 设置零位")
    print("=" * 50)
    print(f"目标机械臂: {' + '.join(label for label, _ in selected_configs)}")

    robots = []

    try:
        for label, config_path in selected_configs:
            print(f"\n实例化{label} Openarm...")
            robots.append((label, Openarm(config_path)))

        print_selected_states(robots)
        operation_menu(robots)

    except KeyboardInterrupt:
        for _, robot in robots:
            robot.set_stop()
        print("\n\n程序被中断，所选机械臂已刹车。")
    except Exception as exc:
        for _, robot in robots:
            robot.set_stop()
        print(f"\n错误: {exc}")
        raise


if __name__ == "__main__":
    main()
