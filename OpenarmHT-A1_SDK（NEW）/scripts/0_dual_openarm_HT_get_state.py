#!/usr/bin/env python3
"""
Openarm 单臂/双臂状态读取测试

用途：
    1. 启动时选择左臂、右臂或双臂
    2. 只实例化所选机械臂，循环读取关节、速度、力矩和夹爪状态
    3. 双臂模式可验证底层 SDK 是否允许同一进程双实例共存

用法：
    python 0_dual_openarm_HT_get_state.py
"""

import os
import time

from Lib import Openarm


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
    """交互选择要连接的机械臂。"""
    print("\n请选择要调试的机械臂：")
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


def refresh_state(robot, cycles=4):
    """多次刷新电机状态，减少读取到旧缓存的概率。"""
    for _ in range(cycles):
        robot.send_get_motor_state_cmd()
        robot.motor_send_cmd()


def print_arm_state(label, robot):
    refresh_state(robot)
    positions = robot.get_current_pos()
    velocities = robot.get_current_vel()
    torques = robot.get_current_torque()
    gripper_state = robot.get_current_state_gripper()

    print("\n" + "=" * 60)
    print(f"{label} 状态")
    print("=" * 60)
    for i in range(robot.motor_count):
        print(
            f"关节{i+1}: 位置={positions[i]:8.4f} rad, "
            f"速度={velocities[i]:8.4f} rad/s, 力矩={torques[i]:8.4f}"
        )
    print(
        f"夹爪:   位置={gripper_state.position:8.4f} rad, "
        f"速度={gripper_state.velocity:8.4f} rad/s"
    )


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    left_config = os.path.normpath(os.path.join(script_dir, "..", "robot_param", "Openarm_left.yaml"))
    right_config = os.path.normpath(os.path.join(script_dir, "..", "robot_param", "Openarm_right.yaml"))

    selected_arm = select_arm()
    selected_configs = []
    if selected_arm in ("left", "both"):
        selected_configs.append(("左臂", left_config))
    if selected_arm in ("right", "both"):
        selected_configs.append(("右臂", right_config))

    print("=" * 60)
    print("Openarm 状态读取测试")
    print("=" * 60)
    print(f"调试模式: {' + '.join(label for label, _ in selected_configs)}")
    for label, config_path in selected_configs:
        print(f"{label}配置: {config_path}")

    robots = []

    try:
        for label, config_path in selected_configs:
            print(f"\n实例化{label} Openarm...")
            robots.append((label, Openarm(config_path)))
            time.sleep(0.5)

        while True:
            for label, robot in robots:
                print_arm_state(label, robot)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n程序被中断")
    except Exception as e:
        print(f"\n错误: {e}")
        raise


if __name__ == "__main__":
    main()
