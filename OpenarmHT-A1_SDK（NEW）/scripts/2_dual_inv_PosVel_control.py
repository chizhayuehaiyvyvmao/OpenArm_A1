#!/usr/bin/env python3
"""左臂、右臂或双臂逆解位置速度控制。"""

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
    [-0.10, -0.10, 0.10, -0.20, 0.10, 0.10, -0.05],
    dtype=float,
)
IK_MAX_ITERATIONS = 50
IK_POSITION_TOLERANCE_M = 1e-4
IK_ROTATION_TOLERANCE_RAD = 1e-4


def rotation_error_angle(target_rotation, actual_rotation):
    """计算两个旋转矩阵之间的角度误差（rad）。"""
    rotation_delta = np.asarray(target_rotation).T @ np.asarray(actual_rotation)
    cosine = np.clip((np.trace(rotation_delta) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cosine))


def select_arm():
    print("\n请选择机械臂：")
    print("  1. 左臂")
    print("  2. 右臂")
    print("  3. 双臂")
    while True:
        choice = input("请输入 1/2/3: ").strip()
        if choice == "1":
            return [("左臂", LEFT_CONFIG, LEFT_REFERENCE_JOINTS.copy())]
        if choice == "2":
            return [("右臂", RIGHT_CONFIG, -LEFT_REFERENCE_JOINTS)]
        if choice == "3":
            return [
                ("左臂", LEFT_CONFIG, LEFT_REFERENCE_JOINTS.copy()),
                ("右臂", RIGHT_CONFIG, -LEFT_REFERENCE_JOINTS),
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
        for label, config_path, reference_joints in select_arm():
            robots.append((label, Openarm(config_path), reference_joints))

        for label, robot, _ in robots:
            zero = [0.0] * robot.motor_count
            success = robot.Joint_Pos_Vel(
                zero,
                [0.5] * robot.motor_count,
                robot.max_torque,
                iswait=True,
            )
            print(f"{label}零位准备状态: {success}")

        solutions = {}
        for label, robot, reference_joints in robots:
            robot.send_get_motor_state_cmd()
            robot.motor_send_cmd()
            target_pose = robot.forward_kinematics(reference_joints)
            if target_pose is None:
                solutions[label] = None
                print(f"{label}目标位姿正解失败，跳过目标运动")
                continue

            target_position = target_pose["position"]
            target_rotation = target_pose["rotation"]
            print(
                f"{label}目标TCP位置: "
                f"x={target_position[0]:.4f}, "
                f"y={target_position[1]:.4f}, "
                f"z={target_position[2]:.4f} m"
            )

            # 目标位姿就是由 reference_joints 正解生成的，因此这组关节角
            # 是已知可达解。用它作为单初值可避免默认 8×1000 次随机搜索。
            ik_start = time.monotonic()
            solution = robot.inverse_kinematics(
                target_position,
                target_rotation,
                init_q=reference_joints,
                max_iter=IK_MAX_ITERATIONS,
                multi_init=False,
            )
            ik_elapsed = time.monotonic() - ik_start
            solutions[label] = solution
            if solution is None:
                print(
                    f"{label}逆解失败（耗时 {ik_elapsed:.4f}s），"
                    "跳过目标运动"
                )
                continue

            solved_pose = robot.forward_kinematics(solution)
            if solved_pose is None:
                solutions[label] = None
                print(f"{label}逆解回验正解失败，跳过目标运动")
                continue

            position_error = float(
                np.linalg.norm(
                    np.asarray(solved_pose["position"])
                    - np.asarray(target_position)
                )
            )
            rotation_error = rotation_error_angle(
                target_rotation,
                solved_pose["rotation"],
            )
            print(
                f"{label}逆解完成: 耗时={ik_elapsed:.4f}s, "
                f"位置误差={position_error:.6f}m, "
                f"姿态误差={rotation_error:.6f}rad"
            )
            print(
                f"{label}逆解关节角: "
                f"{np.array2string(solution, precision=4, suppress_small=True)}"
            )
            if (
                position_error > IK_POSITION_TOLERANCE_M
                or rotation_error > IK_ROTATION_TOLERANCE_RAD
            ):
                solutions[label] = None
                print(f"{label}逆解回验超差，拒绝下发目标运动")

        for label, robot, _ in robots:
            solution = solutions[label]
            if solution is not None:
                success = robot.moveJ(
                    solution,
                    duration=3.0,
                    max_tqu=robot.max_torque,
                    iswait=True,
                )
                print(f"{label}目标执行状态: {success}")

        time.sleep(3.0)
        for label, robot, _ in robots:
            success = robot.moveJ(
                [0.0] * robot.motor_count,
                duration=3.0,
                max_tqu=robot.max_torque,
                iswait=True,
            )
            print(f"{label}回零状态: {success}")
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
