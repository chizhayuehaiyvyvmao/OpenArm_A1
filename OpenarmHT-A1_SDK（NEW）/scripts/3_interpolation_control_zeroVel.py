#!/usr/bin/env python3
"""左臂、右臂或双臂零中间速度七次插值轨迹。"""

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
            return [("左臂", LEFT_CONFIG, False)]
        if choice == "2":
            return [("右臂", RIGHT_CONFIG, True)]
        if choice == "3":
            return [
                ("左臂", LEFT_CONFIG, False),
                ("右臂", RIGHT_CONFIG, True),
            ]
        print("输入无效，请输入 1、2 或 3。")


def mirror(values):
    return [-value for value in values]


def precise_sleep(duration):
    if duration <= 0:
        return
    end_time = time.perf_counter() + duration
    if duration > 0.001:
        time.sleep(duration - 0.001)
    while time.perf_counter() < end_time:
        pass


def execute_trajectory(robots, left_waypoints, durations, control_rate=100):
    if len(left_waypoints) != len(durations) + 1:
        raise ValueError("路径点数量应该比时间段数量多 1")

    waypoints = {
        label: [mirror(point) for point in left_waypoints]
        if mirrored
        else [list(point) for point in left_waypoints]
        for label, _, mirrored in robots
    }
    period = 1.0 / control_rate

    for segment, duration in enumerate(durations):
        steps = int(duration * control_rate)
        segment_start = time.perf_counter()
        for step in range(steps):
            target_time = segment_start + (step + 1) * period
            current_time = step * period
            for label, robot, _ in robots:
                position, velocity, _ = robot.septic_interpolation(
                    waypoints[label][segment],
                    waypoints[label][segment + 1],
                    duration,
                    current_time,
                )
                robot.Joint_Pos_Vel(
                    position,
                    abs(velocity),
                    robot.max_torque,
                    iswait=False,
                )
            precise_sleep(target_time - time.perf_counter())

    for label, robot, _ in robots:
        robot.Joint_Pos_Vel(
            waypoints[label][-1],
            [0.5] * robot.motor_count,
            robot.max_torque,
            iswait=False,
        )


def stop_robots(robots):
    for label, robot, _ in robots:
        try:
            robot.set_stop()
        except Exception as error:
            print(f"停止{label}时发生错误: {error}")


def main():
    robots = []
    left_waypoints = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [-0.2, -0.2, 0.2, -0.4, 0.5, 0.2, -0.1],
        [-0.2, 0.0, -0.1, -0.7, 0.0, -0.1, -1.1],
        [-0.2, -0.2, 0.2, -0.4, 0.5, 0.2, -0.1],
    ]
    durations = [1.2, 1.0, 1.0]

    try:
        for label, config_path, mirrored in select_arm():
            robots.append((label, Openarm(config_path), mirrored))

        for _, robot, _ in robots:
            zero = [0.0] * robot.motor_count
            success = robot.Joint_Pos_Vel(
                zero,
                [0.5] * robot.motor_count,
                [10.0] * robot.motor_count,
                iswait=True,
            )
            if not success:
                raise RuntimeError("零位准备未到位")
        time.sleep(1.0)

        execute_trajectory(robots, left_waypoints, durations)
        time.sleep(1.0)
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
