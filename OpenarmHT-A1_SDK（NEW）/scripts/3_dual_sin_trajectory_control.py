#!/usr/bin/env python3
"""左臂、右臂或双臂七关节正弦轨迹与夹爪控制。"""

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

TRAJECTORY_FREQUENCY = 0.3
TRAJECTORY_DURATION = 10.0
MOTION_DIRECTION = -1.0
CONTROL_FREQUENCY = 100.0
RAMP_DURATION = 1.5
PRESET_AMPLITUDES = np.array([0.4, 0.12, 0.4, 0.5, 0.5, 0.5, 0.2])
JOINT_DIRECTIONS = np.array([1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0])
LEFT_CENTER = np.array([0.0, -0.2, 0.0, -0.4, 0.0, 0.0, 0.0])
RIGHT_CENTER = -LEFT_CENTER
GRIPPER_CLOSE_POSITION = 0.0
GRIPPER_OPEN_POSITION = 1.6
GRIPPER_SPEED = 1.0
GRIPPER_MAX_TORQUE = 0.5


def select_arm():
    print("\n请选择机械臂：")
    print("  1. 左臂")
    print("  2. 右臂")
    print("  3. 双臂")
    while True:
        choice = input("请输入 1/2/3: ").strip()
        if choice == "1":
            return [("左臂", LEFT_CONFIG, False, LEFT_CENTER.copy())]
        if choice == "2":
            return [("右臂", RIGHT_CONFIG, True, RIGHT_CENTER.copy())]
        if choice == "3":
            return [
                ("左臂", LEFT_CONFIG, False, LEFT_CENTER.copy()),
                ("右臂", RIGHT_CONFIG, True, RIGHT_CENTER.copy()),
            ]
        print("输入无效，请输入 1、2 或 3。")


def refresh_state(robot, wait=0.05):
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(wait)


def quintic_blend(value):
    value = float(np.clip(value, 0.0, 1.0))
    blend = 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5
    derivative = 30.0 * value**2 - 60.0 * value**3 + 30.0 * value**4
    return blend, derivative


def amplitude_envelope(elapsed):
    if elapsed < RAMP_DURATION:
        blend, derivative = quintic_blend(elapsed / RAMP_DURATION)
        return blend, derivative / RAMP_DURATION
    remaining = TRAJECTORY_DURATION - elapsed
    if remaining < RAMP_DURATION:
        blend, derivative = quintic_blend(remaining / RAMP_DURATION)
        return blend, -derivative / RAMP_DURATION
    return 1.0, 0.0


def validate_pose(robot, pose, label, margin=0.03):
    pose = np.asarray(pose, dtype=float)
    lower = np.asarray(robot.joint_limits["lower"], dtype=float) + margin
    upper = np.asarray(robot.joint_limits["upper"], dtype=float) - margin
    if pose.shape != (robot.motor_count,):
        raise ValueError(f"{label}目标维度错误: {pose.shape}")
    if np.any(pose < lower) or np.any(pose > upper):
        raise ValueError(f"{label}超出安全限位: {np.round(pose, 3)}")


def calculate_amplitudes(robots, margin=0.03):
    rooms = []
    for _, robot, _, center in robots:
        lower = np.asarray(robot.joint_limits["lower"], dtype=float) + margin
        upper = np.asarray(robot.joint_limits["upper"], dtype=float) - margin
        rooms.append(np.minimum(upper - center, center - lower))
    safe_magnitude = 0.8 * np.min(np.asarray(rooms), axis=0)
    magnitude = np.minimum(PRESET_AMPLITUDES, safe_magnitude)
    if np.any(magnitude <= 0.0):
        raise ValueError("所选机械臂中心姿态附近没有足够的安全运动空间")
    return magnitude * JOINT_DIRECTIONS


def trajectory_sample(elapsed, amplitudes, center):
    omega = 2.0 * np.pi * TRAJECTORY_FREQUENCY
    envelope, envelope_velocity = amplitude_envelope(elapsed)
    phase = MOTION_DIRECTION * omega * elapsed
    sine = np.sin(phase)
    cosine = np.cos(phase)
    wave = envelope * sine
    wave_velocity = (
        envelope_velocity * sine
        + envelope * MOTION_DIRECTION * omega * cosine
    )
    return center + amplitudes * wave, amplitudes * wave_velocity, wave


def gripper_position(elapsed, robots):
    close_position = GRIPPER_CLOSE_POSITION
    open_position = GRIPPER_OPEN_POSITION
    for _, robot, _, _ in robots:
        if robot.gripper_limits is not None:
            close_position = max(close_position, robot.gripper_limits["lower"])
            open_position = min(open_position, robot.gripper_limits["upper"])
    omega = 2.0 * np.pi * TRAJECTORY_FREQUENCY
    blend = 0.5 * (1.0 - np.cos(omega * elapsed))
    return close_position + (open_position - close_position) * blend


def queue_gripper(robot, position):
    robot.Motors[robot.gripper_id - 1].pos_vel_MAXtqe(
        float(position),
        GRIPPER_SPEED,
        GRIPPER_MAX_TORQUE,
    )


def send_selected(robots, positions, velocities):
    for label, robot, _, _ in robots:
        success = robot.Joint_Pos_Vel(
            np.asarray(positions[label], dtype=float).tolist(),
            np.abs(np.asarray(velocities[label], dtype=float)).tolist(),
            robot.max_torque,
            iswait=False,
        )
        if not success:
            raise RuntimeError(f"{label}位置速度命令发送失败")


def smooth_move_selected(robots, targets, duration, label):
    starts = {}
    for arm_label, robot, _, _ in robots:
        refresh_state(robot, wait=0.05)
        starts[arm_label] = np.asarray(robot.get_current_pos(), dtype=float)
        validate_pose(robot, targets[arm_label], f"{arm_label}{label}")

    period = 1.0 / CONTROL_FREQUENCY
    start_time = time.perf_counter()
    next_deadline = start_time
    while True:
        elapsed = time.perf_counter() - start_time
        ratio = min(elapsed / duration, 1.0)
        blend, derivative = quintic_blend(ratio)
        positions = {}
        velocities = {}
        for arm_label, _, _, _ in robots:
            delta = np.asarray(targets[arm_label]) - starts[arm_label]
            positions[arm_label] = starts[arm_label] + delta * blend
            velocities[arm_label] = delta * derivative / duration
        send_selected(robots, positions, velocities)
        if ratio >= 1.0:
            break
        next_deadline += period
        sleep_time = next_deadline - time.perf_counter()
        if sleep_time > 0.0:
            time.sleep(sleep_time)


def validate_trajectory(robots, amplitudes):
    for label, robot, _, center in robots:
        validate_pose(robot, center - amplitudes, f"{label}前内端")
        validate_pose(robot, center + amplitudes, f"{label}后外端")

    sample_times = np.linspace(0.0, TRAJECTORY_DURATION, 5000)
    max_velocity = np.max(
        np.abs(
            np.asarray(
                [
                    trajectory_sample(value, amplitudes, np.zeros(7))[1]
                    for value in sample_times
                ]
            )
        ),
        axis=0,
    )
    for label, robot, _, _ in robots:
        if np.any(max_velocity > np.asarray(robot.velocity_limits)):
            raise ValueError(f"{label}正弦轨迹超出速度限制")
    return max_velocity


def run_sine(robots):
    amplitudes = calculate_amplitudes(robots)
    max_velocity = validate_trajectory(robots, amplitudes)
    print(f"有符号振幅: {np.round(amplitudes, 3)} rad")
    print(f"最大关节速度: {np.round(max_velocity, 3)} rad/s")

    period = 1.0 / CONTROL_FREQUENCY
    start_time = time.perf_counter()
    next_deadline = start_time
    print_deadline = start_time
    while True:
        now = time.perf_counter()
        elapsed = now - start_time
        if elapsed >= TRAJECTORY_DURATION:
            break

        positions = {}
        velocities = {}
        waves = {}
        grip = gripper_position(elapsed, robots)
        for label, robot, _, center in robots:
            positions[label], velocities[label], waves[label] = trajectory_sample(
                elapsed, amplitudes, center
            )
            queue_gripper(robot, grip)
        send_selected(robots, positions, velocities)

        if now >= print_deadline:
            status = []
            for label, _, mirrored, _ in robots:
                forward = waves[label] < 0.0
                if mirrored:
                    forward = not forward
                status.append(f"{label}:{'前内' if forward else '后外'}")
            print(f"\r时间:{elapsed:5.2f}s | {' | '.join(status)} | 夹爪:{grip:.3f}", end="")
            print_deadline = now + 0.25

        next_deadline += period
        sleep_time = next_deadline - time.perf_counter()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_deadline = time.perf_counter()

    centers = {}
    zeros = {}
    for label, robot, _, center in robots:
        queue_gripper(robot, GRIPPER_CLOSE_POSITION)
        centers[label] = center
        zeros[label] = np.zeros(7)
    send_selected(robots, centers, zeros)
    print("\n正弦轨迹完成，夹爪已闭合")


def hold_current(robots):
    positions = {}
    velocities = {}
    for label, robot, _, _ in robots:
        refresh_state(robot, wait=0.05)
        positions[label] = np.asarray(robot.get_current_pos(), dtype=float)
        velocities[label] = np.zeros(robot.motor_count)
    send_selected(robots, positions, velocities)


def stop_robots(robots):
    for label, robot, _, _ in robots:
        try:
            robot.set_stop()
        except Exception as error:
            print(f"停止{label}时发生错误: {error}")


def main():
    robots = []
    try:
        for label, config_path, mirrored, center in select_arm():
            robot = Openarm(config_path)
            if robot.motor_count != 7:
                raise RuntimeError(f"{label}不是 7 关节机械臂")
            robots.append((label, robot, mirrored, center))

        center_targets = {label: center for label, _, _, center in robots}
        zero_targets = {label: np.zeros(7) for label, _, _, _ in robots}
        smooth_move_selected(robots, center_targets, 3.0, "进入正弦中心姿态")
        time.sleep(0.5)
        run_sine(robots)
        time.sleep(0.5)
        smooth_move_selected(robots, zero_targets, 3.0, "回到零位")
    except KeyboardInterrupt:
        print("\n轨迹被中断")
        if robots:
            hold_current(robots)
    except Exception as error:
        print(f"\n错误: {error}")
        if robots:
            hold_current(robots)
    finally:
        stop_robots(robots)
        print("\n所选机械臂电机已停止")


if __name__ == "__main__":
    main()
