#!/usr/bin/env python3
"""左臂、右臂或双臂 moveL 笛卡尔位置控制示例。"""

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
        maximum = max(last_errors[label])
        print(f"{label}最大到位误差: {maximum:.3f} rad")
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


def move_all_to_reference(robots):
    move_selected_joints(
        robots,
        reference_targets(robots),
        "移动到 moveL 参考姿态",
    )


def run_relative_move(robots, description, left_offset, duration):
    print(f"\n{description}")
    for label, robot, mirror_sign in robots:
        current = robot.forward_kinematics()
        if current is None:
            raise RuntimeError(f"{label}无法获取当前位姿")
        offset = np.asarray(left_offset, dtype=float).copy()
        offset[1] *= mirror_sign
        target = np.asarray(current["position"], dtype=float) + offset
        success = robot.moveL(
            target_position=target,
            target_rotation=current["rotation"],
            duration=duration,
            use_spline=True,
        )
        if not success:
            raise RuntimeError(f"{label}{description}失败")


def run_square(robots):
    print("\n多段正方形路径")
    side_length = 0.035
    for label, robot, mirror_sign in robots:
        current = robot.forward_kinematics()
        if current is None:
            raise RuntimeError(f"{label}无法获取当前位姿")
        start = np.asarray(current["position"], dtype=float)
        rotation = current["rotation"]
        y_side = side_length * mirror_sign
        waypoints = [
            {"position": start, "rotation": rotation},
            {"position": start + [-side_length, 0.0, 0.0], "rotation": rotation},
            {"position": start + [-side_length, -y_side, 0.0], "rotation": rotation},
            {"position": start + [0.0, -y_side, 0.0], "rotation": rotation},
            {"position": start, "rotation": rotation},
        ]
        trajectory, fraction = robot.compute_cartesian_path(waypoints)
        if trajectory is None or fraction < 0.99:
            raise RuntimeError(
                f"{label}正方形路径不完整: {fraction * 100.0:.1f}%"
            )
        timestamps = robot.compute_time_parameterization(trajectory, duration=3.0)
        trajectory, timestamps, velocities = robot.smooth_trajectory_spline(
            trajectory,
            timestamps,
        )
        success = robot._execute_trajectory(
            trajectory,
            timestamps,
            velocities,
            max_tqu=None,
        )
        if not success:
            raise RuntimeError(f"{label}正方形轨迹执行失败")


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
            robots.append((label, Openarm(config_path), mirror_sign))

        move_selected_joints(robots, zero_targets(robots), "初始回零")
        move_all_to_reference(robots)
        time.sleep(1.0)

        run_relative_move(robots, "沿 -X 轴移动 5 cm", [-0.05, 0.0, 0.0], 2.5)
        time.sleep(1.0)
        move_all_to_reference(robots)
        run_relative_move(robots, "沿镜像 Y 轴移动 4 cm", [0.0, 0.04, 0.0], 2.0)
        time.sleep(0.5)
        move_all_to_reference(robots)
        run_relative_move(
            robots,
            "镜像对角线运动",
            [-0.03, -0.03, 0.03],
            2.5,
        )
        time.sleep(0.5)
        move_all_to_reference(robots)
        run_square(robots)

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
