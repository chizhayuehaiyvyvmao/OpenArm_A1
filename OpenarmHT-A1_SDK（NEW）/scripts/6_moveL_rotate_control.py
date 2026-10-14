#!/usr/bin/env python3
"""左臂、右臂或双臂 moveL 末端姿态控制示例。"""

import os
import time

import numpy as np
from scipy.spatial.transform import Rotation

from Lib import Openarm


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LEFT_CONFIG = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_left.yaml")
)
RIGHT_CONFIG = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_right.yaml")
)
LEFT_REFERENCE_JOINTS = np.array(
    [-0.20, -0.20, 0.20, -0.40, 0.50, 0.20, -0.10],
    dtype=float,
)
JOINT_COMMAND_SPEED = 0.20
JOINT_COMMAND_PERIOD = 0.01
JOINT_MOVE_TOLERANCE = 0.08
JOINT_MOVE_TIMEOUT = 25.0


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


def print_orientation(label, rotation):
    euler = Rotation.from_matrix(rotation).as_euler("xyz", degrees=True)
    print(
        f"{label}: Roll={euler[0]:.2f}°, "
        f"Pitch={euler[1]:.2f}°, Yaw={euler[2]:.2f}°"
    )


def move_selected_joints(robots, targets, description):
    """以固定频率持续下发所有机械臂目标，并检查实时反馈。"""
    print(f"\n{description}")
    deadline = time.monotonic() + JOINT_MOVE_TIMEOUT
    last_errors = {label: [float("inf")] * 7 for label, _, _ in robots}
    reached = False
    next_deadline = time.perf_counter()
    last_print = time.monotonic() - 1.0
    while time.monotonic() < deadline:
        reached = True
        for label, robot, _ in robots:
            target = np.asarray(targets[label], dtype=float)
            success = robot.Joint_Pos_Vel(
                target,
                [JOINT_COMMAND_SPEED] * robot.motor_count,
                robot.max_torque,
                iswait=False,
            )
            if not success:
                raise RuntimeError(f"{label}{description}命令未成功下发")
            actual = np.asarray(robot.get_current_pos(), dtype=float)
            errors = np.abs(actual - target).tolist()
            arm_reached = max(errors) <= JOINT_MOVE_TOLERANCE
            last_errors[label] = errors
            reached = reached and arm_reached
        if reached:
            break
        now = time.monotonic()
        if now - last_print >= 0.5:
            status = ", ".join(
                f"{label}={max(last_errors[label]):.3f} rad"
                for label, _, _ in robots
            )
            print(f"  到位误差: {status}")
            last_print = now
        next_deadline += JOINT_COMMAND_PERIOD
        sleep_time = next_deadline - time.perf_counter()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_deadline = time.perf_counter()

    for label, _, _ in robots:
        print(f"{label}最大到位误差: {max(last_errors[label]):.3f} rad")
    if not reached:
        details = []
        for label, _, _ in robots:
            joint_errors = ", ".join(
                f"J{index + 1}={error:.3f}"
                for index, error in enumerate(last_errors[label])
            )
            details.append(f"{label}[{joint_errors}]")
        raise RuntimeError(
            f"{description}超时（容差 {JOINT_MOVE_TOLERANCE:.3f} rad）: "
            + "; ".join(details)
        )


def zero_targets(robots):
    return {
        label: np.zeros(robot.motor_count, dtype=float)
        for label, robot, _ in robots
    }


def reference_targets(robots):
    return {
        label: LEFT_REFERENCE_JOINTS * mirror_sign
        for label, _, mirror_sign in robots
    }


def move_rotations(robots, positions, rotations, description, duration=3.0):
    print(f"\n{description}")
    for label, robot, _ in robots:
        print_orientation(f"{label}目标姿态", rotations[label])
        success = robot.moveL(
            target_position=positions[label],
            target_rotation=rotations[label],
            duration=duration,
            use_spline=True,
        )
        if not success:
            raise RuntimeError(f"{label}{description}失败")


def stop_robots(robots):
    for label, robot, _ in robots:
        try:
            robot.set_stop()
        except Exception as error:
            print(f"停止{label}时发生错误: {error}")


def main():
    robots = []
    try:
        for label, config_path, mirror_sign in select_arm():
            robot = Openarm(config_path)
            robots.append((label, robot, mirror_sign))
        move_selected_joints(robots, zero_targets(robots), "初始回零")
        move_selected_joints(
            robots,
            reference_targets(robots),
            "移动到 moveL 参考姿态",
        )
        time.sleep(1.0)

        positions = {}
        base_rotations = {}
        for label, robot, _ in robots:
            current = robot.forward_kinematics()
            if current is None:
                raise RuntimeError(f"{label}无法获取当前位姿")
            positions[label] = np.asarray(current["position"], dtype=float)
            base_rotations[label] = np.asarray(current["rotation"], dtype=float)

        rotations = {
            label: base_rotations[label]
            @ robot.rotation_matrix_from_euler(0.0, 0.0, mirror_sign * np.pi / 6.0)
            for label, robot, mirror_sign in robots
        }
        move_rotations(robots, positions, rotations, "绕镜像 Z 轴旋转 30°")
        time.sleep(1.0)

        rotations = {}
        for label, robot, _ in robots:
            current = robot.forward_kinematics()
            rotations[label] = current["rotation"] @ robot.rotation_matrix_from_euler(
                0.0,
                np.pi / 12.0,
                0.0,
            )
        move_rotations(robots, positions, rotations, "绕末端 Y 轴旋转 15°")
        time.sleep(1.0)

        rotations = {}
        for label, robot, mirror_sign in robots:
            current = robot.forward_kinematics()
            rotations[label] = current["rotation"] @ robot.rotation_matrix_from_euler(
                -mirror_sign * np.pi / 12.0,
                0.0,
                0.0,
            )
        move_rotations(robots, positions, rotations, "绕镜像 X 轴旋转 15°")
        time.sleep(1.0)

        move_rotations(
            robots,
            positions,
            base_rotations,
            "回到参考姿态",
        )
        time.sleep(1.0)

        cone_angle = np.pi / 18.0
        step_count = 8
        for step in range(step_count):
            rotations = {}
            for label, robot, mirror_sign in robots:
                angle = mirror_sign * 2.0 * np.pi * step / step_count
                rotate_out = robot.rotation_matrix_from_euler(0.0, 0.0, angle)
                tilt = robot.rotation_matrix_from_euler(0.0, cone_angle, 0.0)
                rotate_back = robot.rotation_matrix_from_euler(0.0, 0.0, -angle)
                local_rotation = rotate_out @ tilt @ rotate_back
                rotations[label] = base_rotations[label] @ local_rotation
            move_rotations(
                robots,
                positions,
                rotations,
                f"圆锥运动 {step + 1}/{step_count}",
                duration=1.5,
            )
            time.sleep(0.5)

        move_selected_joints(robots, zero_targets(robots), "最终回零")
        time.sleep(2.0)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as error:
        print(f"\n错误: {error}")
    finally:
        stop_robots(robots)
        print("\n所选机械臂电机已停止")


if __name__ == "__main__":
    main()
