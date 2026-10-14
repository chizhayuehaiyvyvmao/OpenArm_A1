#!/usr/bin/env python3
"""左臂、右臂或双臂正弦轨迹控制，并保存期望/实际曲线。"""

import os
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from Lib import Openarm


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LEFT_CONFIG = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_left.yaml")
)
RIGHT_CONFIG = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_right.yaml")
)

FREQUENCY = 0.5
DURATION = 20.0
CONTROL_RATE = 500.0
KP = [30.0, 50.0, 60.0, 25.0, 15.0, 10.0, 10.0]
KD = [3.0, 5.0, 6.0, 2.5, 1.5, 1.0, 1.0]
LEFT_INIT = np.array([-0.3, -0.5, 0.4, -0.6, -0.3, 0.0, -0.1])
PRESET_AMPLITUDES = np.array([0.30, 0.30, 0.30, 0.30, 0.25, 0.15, 0.15])


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


def plot_tracking_results(states):
    """单臂使用 7x1，双臂使用 7x2 自适应布局。"""
    labels = list(states)
    column_count = len(labels)
    figure, axes = plt.subplots(
        7,
        column_count,
        figsize=(7.0 * column_count, 17.0),
        sharex="col",
        squeeze=False,
    )

    display_names = {"左臂": "Left Arm", "右臂": "Right Arm"}
    for column, label in enumerate(labels):
        state = states[label]
        times = np.asarray(state["times"], dtype=float)
        desired = np.asarray(state["desired"], dtype=float)
        actual = np.asarray(state["actual"], dtype=float)
        if desired.ndim != 2 or desired.shape[0] == 0:
            raise RuntimeError(f"{label}没有可绘制的轨迹数据")

        rmse = np.sqrt(np.mean((actual - desired) ** 2, axis=0))
        print(f"{label}各关节 RMSE(rad): {np.round(rmse, 4)}")
        for joint_index in range(7):
            axis = axes[joint_index, column]
            axis.plot(
                times,
                desired[:, joint_index],
                label="Desired",
                linewidth=1.8,
            )
            axis.plot(
                times,
                actual[:, joint_index],
                label="Actual",
                linewidth=1.2,
                alpha=0.9,
            )
            axis.set_title(
                f"{display_names[label]} J{joint_index + 1} "
                f"(RMSE={rmse[joint_index]:.4f} rad)"
            )
            axis.set_ylabel("Position (rad)")
            axis.grid(True, alpha=0.3)
            if joint_index == 0:
                axis.legend(loc="upper right")
            if joint_index == 6:
                axis.set_xlabel("Time (s)")

    mode = "dual" if column_count == 2 else (
        "left" if labels[0] == "左臂" else "right"
    )
    figure.suptitle(
        f"Sin Trajectory Tracking - {mode.capitalize()}",
        fontsize=15,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.985))
    output_path = Path(__file__).resolve().with_name(
        f"sin_trajectory_tracking_{mode}.png"
    )
    figure.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(figure)
    print(f"轨迹对比图已保存到: {output_path}")


def prepare_state(label, robot, mirrored):
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    center = np.asarray(robot.get_current_pos(), dtype=float)
    lower = np.asarray(robot.joint_limits["lower"], dtype=float)
    upper = np.asarray(robot.joint_limits["upper"], dtype=float)
    safe = 0.8 * np.minimum(upper - center, center - lower)
    magnitude = np.minimum(safe, PRESET_AMPLITUDES)
    if np.any(magnitude <= 0.0):
        raise ValueError(f"{label}中心姿态附近没有安全正弦空间")
    amplitudes = -magnitude if mirrored else magnitude
    return {
        "center": center,
        "lower": lower,
        "upper": upper,
        "amplitudes": amplitudes,
        "times": [],
        "desired": [],
        "actual": [],
    }


def run_trajectory(robots):
    states = {
        label: prepare_state(label, robot, mirrored)
        for label, robot, mirrored in robots
    }
    omega = 2.0 * np.pi * FREQUENCY
    period = 1.0 / CONTROL_RATE
    start_time = time.perf_counter()
    next_deadline = start_time
    step = 0

    while True:
        elapsed = time.perf_counter() - start_time
        if elapsed >= DURATION:
            break

        for label, robot, _ in robots:
            state = states[label]
            position = state["center"] + state["amplitudes"] * np.sin(
                omega * elapsed
            )
            velocity = state["amplitudes"] * omega * np.cos(omega * elapsed)
            outside = (position < state["lower"]) | (position > state["upper"])
            position = np.clip(position, state["lower"], state["upper"])
            velocity[outside] = 0.0

            current = np.asarray(robot.get_current_pos(), dtype=float)
            gravity = np.asarray(robot.get_Gravity(current), dtype=float)
            torque_limit = np.asarray(robot.max_torque, dtype=float)
            gravity = np.clip(gravity, -torque_limit, torque_limit)
            robot.pos_vel_tqe_kp_kd(position, velocity, gravity, KP, KD)
            actual = np.asarray(robot.get_current_pos(), dtype=float)

            state["times"].append(elapsed)
            state["desired"].append(position.copy())
            state["actual"].append(actual.copy())
            if step % 50 == 0:
                error = np.linalg.norm(actual - position)
                print(
                    f"{label} t={elapsed:.2f}s J1期望={position[0]:.3f} "
                    f"J1实际={actual[0]:.3f} 误差={error:.4f}"
                )

        step += 1
        next_deadline += period
        sleep_time = next_deadline - time.perf_counter()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_deadline = time.perf_counter()

    for label, robot, _ in robots:
        state = states[label]
        robot.Joint_Pos_Vel(
            state["center"],
            [0.5] * robot.motor_count,
            robot.max_torque,
            iswait=True,
        )
    plot_tracking_results(states)


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
            robot = Openarm(config_path)
            init_position = -LEFT_INIT if mirrored else LEFT_INIT.copy()
            robots.append((label, robot, mirrored))
            robot.Joint_Pos_Vel(
                [0.0] * robot.motor_count,
                [0.5] * robot.motor_count,
                robot.max_torque,
                iswait=True,
            )
            success = robot.Joint_Pos_Vel(
                init_position,
                [0.5] * robot.motor_count,
                robot.max_torque,
                iswait=True,
            )
            if not success:
                raise RuntimeError(f"{label}未到达初始位置")
        time.sleep(1.0)

        run_trajectory(robots)
        for _, robot, _ in robots:
            robot.Joint_Pos_Vel(
                [0.0] * robot.motor_count,
                [0.5] * robot.motor_count,
                robot.max_torque,
                iswait=True,
            )
        time.sleep(2.0)
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as error:
        print(f"\n错误: {error}")
    finally:
        stop_robots(robots)
        print("\n所选机械臂电机已停止")


if __name__ == "__main__":
    main()
