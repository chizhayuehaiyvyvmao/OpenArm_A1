#!/usr/bin/env python3
"""openarm_HT 左臂、右臂或双臂轨迹录制与回放集成例程。"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator

from Lib import Openarm, TrajectoryRecorder


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LEFT_CONFIG = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_left.yaml")
)
RIGHT_CONFIG = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "robot_param", "Openarm_right.yaml")
)

CONTROL_DT = 0.002
RECORD_DT = 0.01
PRINT_INTERVAL = 0.1
USE_FRICTION_COMPENSATION = False

FC = np.array([0.20, 0.20, 0.15, 0.15, 0.0, 0.1, 0.0])
FV = np.array([0.06, 0.06, 0.04, 0.04, 0.0, 0.03, 0.0])
VELOCITY_THRESHOLD = 0.02
PLAYBACK_DT = 0.01
# 手动示教记录里包含编码器抖动；0.31 s 平滑可以去掉加速度尖峰，
# 避免自动重定时为了极少数噪声点把整段轨迹放慢到 80 s 以上。
SMOOTH_WINDOW = 31
# original: 使用录制时间轴；safe: 按速度/加速度限制自动减速。
PLAYBACK_TIMING_MODE = "original"
J3_JOINT_INDEX = 2
J3_SOFTWARE_LIMIT = 1.58
PLAYBACK_JOINT_VELOCITY_LIMIT = 1.0
PLAYBACK_JOINT_ACCELERATION_LIMIT = 2.0
PLAYBACK_TRACKING_ABORT_ERROR = 0.50

KP_PLAY = [20.0, 30.0, 30.0, 12.0, 6.0, 5.0, 5.0]
KD_PLAY = [2.5, 3.5, 3.5, 1.5, 0.8, 0.6, 0.6]
KP_MOVE_TO_START = [30.0, 50.0, 60.0, 25.0, 15.0, 10.0, 10.0]
KD_MOVE_TO_START = [3.0, 5.0, 6.0, 2.5, 1.5, 1.0, 1.0]
GRIPPER_KP = 5.0
GRIPPER_KD = 0.5

MODE_ARMS = {
    "left": ("left",),
    "right": ("right",),
    "dual": ("left", "right"),
}
MODE_LABELS = {
    "left": "左臂",
    "right": "右臂",
    "dual": "双臂",
}


def choose_mode(requested_mode=None):
    """返回 left、right 或 dual；命令行未指定时交互选择。"""
    if requested_mode is not None:
        return requested_mode
    print("请选择轨迹录制模式:")
    print("  1. 左臂")
    print("  2. 右臂")
    print("  3. 双臂")
    choices = {
        "1": "left", "left": "left", "l": "left",
        "2": "right", "right": "right", "r": "right",
        "3": "dual", "dual": "dual", "d": "dual",
    }
    while True:
        answer = input("请输入 1/2/3: ").strip().lower()
        if answer in choices:
            return choices[answer]
        print("输入无效，请重新选择。")


def create_robots(mode):
    configs = {"left": LEFT_CONFIG, "right": RIGHT_CONFIG}
    robots = {}
    try:
        for arm in MODE_ARMS[mode]:
            robot = Openarm(configs[arm])
            if robot.joint_limits is None:
                raise RuntimeError(f"{MODE_LABELS[arm]}未配置关节限位")
            # 编码器在 J3 边界附近有小幅量化波动，五号录制/
            # 回放例程将该轴的运行时软限位统一为 ±1.58 rad。
            robot.joint_limits["lower"][J3_JOINT_INDEX] = (
                -J3_SOFTWARE_LIMIT
            )
            robot.joint_limits["upper"][J3_JOINT_INDEX] = (
                J3_SOFTWARE_LIMIT
            )
            robots[arm] = robot
            print(
                f"{MODE_LABELS[arm]} J3 软限位: "
                f"[-{J3_SOFTWARE_LIMIT:.2f}, {J3_SOFTWARE_LIMIT:.2f}] rad"
            )
    except Exception:
        stop_robots(robots)
        raise
    return robots


def refresh_states(robots):
    """录制前刷新电机状态，避免记录到未初始化值。"""
    for _ in range(10):
        for robot in robots.values():
            robot.send_get_motor_state_cmd()
        time.sleep(0.1)


def control_gravity_compensation(robot):
    """向一侧机械臂发送零刚度 MIT 重力补偿并返回状态。"""
    position = np.asarray(robot.get_current_pos(), dtype=float)
    velocity = np.asarray(robot.get_current_vel(), dtype=float)
    gripper_pos = float(robot.get_current_pos_gripper())
    gripper_vel = float(robot.get_current_vel_gripper())

    torque = np.asarray(robot.get_Gravity(position), dtype=float)
    if USE_FRICTION_COMPENSATION:
        torque += robot.get_friction_compensation(
            velocity, FC, FV, VELOCITY_THRESHOLD
        )
    torque_limit = np.asarray(robot.max_torque, dtype=float)
    torque = np.clip(torque, -torque_limit, torque_limit)
    zeros = np.zeros(robot.motor_count, dtype=float)
    robot.pos_vel_tqe_kp_kd(
        zeros.tolist(),
        zeros.tolist(),
        torque.tolist(),
        zeros.tolist(),
        zeros.tolist(),
    )
    robot.gripper_control_MIT(0.0, 0.0, 0.0, 0.0, 0.0)
    return {
        "pos": position,
        "vel": velocity,
        "gripper_pos": gripper_pos,
        "gripper_vel": gripper_vel,
    }


def print_states(samples):
    parts = []
    for arm, state in samples.items():
        label = "L" if arm == "left" else "R"
        joints = " ".join(
            f"J{index + 1}:{value:+.2f}"
            for index, value in enumerate(state["pos"])
        )
        parts.append(
            f"{label}[{joints}] G:{state['gripper_pos']:+.2f}"
        )
    print("\r" + " | ".join(parts) + "   ", end="", flush=True)


def stop_robots(robots):
    for arm, robot in robots.items():
        try:
            robot.set_stop()
        except Exception as error:
            print(f"\n停止{MODE_LABELS[arm]}时发生错误: {error}")


def record_trajectory(mode, output_path=None):
    if output_path is None:
        output_path = Path(SCRIPT_DIR) / time.strftime(
            f"trajectory_{mode}_%Y%m%d_%H%M%S.jsonl"
        )
    robots = create_robots(mode)
    recorder = None
    try:
        refresh_states(robots)
        recorder = TrajectoryRecorder(
            output_path,
            metadata={
                "mode": mode,
                "arms": list(MODE_ARMS[mode]),
                "record_dt": RECORD_DT,
                "control_dt": CONTROL_DT,
            },
        )
        print(
            f"\n开始录制{MODE_LABELS[mode]}轨迹: {output_path}"
        )
        print("数据: 关节位置/速度 + 夹爪位置/速度")
        print("Ctrl+C 结束录制")

        next_control_time = time.perf_counter()
        next_record_time = next_control_time
        last_print_time = next_control_time - PRINT_INTERVAL
        while True:
            samples = {
                arm: control_gravity_compensation(robot)
                for arm, robot in robots.items()
            }
            now = time.perf_counter()
            if now >= next_record_time:
                recorder.log_arms(samples)
                while next_record_time <= now:
                    next_record_time += RECORD_DT
            if now - last_print_time >= PRINT_INTERVAL:
                print_states(samples)
                last_print_time = now

            next_control_time += CONTROL_DT
            remaining = next_control_time - time.perf_counter()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_control_time = time.perf_counter()
    except KeyboardInterrupt:
        print("\n录制被用户结束")
    finally:
        if recorder is not None:
            recorder.close()
        stop_robots(robots)
        print("所选机械臂电机已停止")
    recorded_path = Path(output_path).resolve()
    if not recorded_path.is_file():
        raise RuntimeError("录制在轨迹文件创建前被中断")
    return recorded_path


def resolve_trajectory_file(path_value):
    candidate = Path(path_value)
    if candidate.is_file():
        return candidate.resolve()
    script_relative = Path(SCRIPT_DIR) / candidate
    if script_relative.is_file():
        return script_relative.resolve()
    return candidate


def find_trajectory_files():
    """按修改时间从新到旧返回可用 JSONL 轨迹。"""
    search_dirs = {Path(SCRIPT_DIR).resolve(), Path.cwd().resolve()}
    candidates = {}
    for directory in search_dirs:
        for path in directory.glob("*.jsonl"):
            if path.is_file():
                resolved = path.resolve()
                candidates[resolved] = resolved.stat().st_mtime
    return [
        item[0]
        for item in sorted(
            candidates.items(), key=lambda item: item[1], reverse=True
        )
    ]


def choose_trajectory_file(default_path=None):
    files = find_trajectory_files()
    if default_path is not None:
        default_path = Path(default_path).resolve()
        files = [default_path] + [
            path for path in files if path != default_path
        ]
    if not files:
        raise FileNotFoundError("当前目录和 scripts 目录中没有 JSONL 轨迹")

    print("\n可用轨迹（从新到旧）:")
    for index, path in enumerate(files, start=1):
        marker = " [默认]" if index == 1 else ""
        timestamp = time.strftime(
            "%Y-%m-%d %H:%M:%S", time.localtime(path.stat().st_mtime)
        )
        print(f"  {index}. {path.name}  {timestamp}{marker}")
    print("  p. 手动输入路径")
    while True:
        answer = input("请选择（直接回车使用默认）: ").strip()
        if not answer:
            return files[0]
        if answer.lower() == "p":
            manual = resolve_trajectory_file(input("轨迹路径: ").strip())
            if manual.is_file():
                return manual.resolve()
            print(f"文件不存在: {manual}")
            continue
        try:
            selected = int(answer)
        except ValueError:
            selected = 0
        if 1 <= selected <= len(files):
            return files[selected - 1]
        print("输入无效，请重新选择。")


def load_trajectory(filepath):
    metadata = None
    frames = []
    with open(filepath, "r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            if not line.strip():
                continue
            try:
                item = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(
                    f"第 {line_number} 行不是合法 JSON: {error}"
                ) from error
            if item.get("type") == "metadata":
                if metadata is not None:
                    raise ValueError("轨迹包含多个 metadata 记录")
                metadata = item
            elif item.get("type") in (None, "frame"):
                frames.append(item)
    if not frames:
        raise ValueError("轨迹文件没有采样帧")

    legacy = metadata is None and "arms" not in frames[0]
    if legacy:
        if any("t" not in frame or "pos" not in frame for frame in frames):
            raise ValueError("旧版轨迹帧字段不完整")
        return None, frames, (), True

    if metadata is None or metadata.get("format") != "openarm_trajectory":
        raise ValueError("新版轨迹缺少有效 metadata")
    if int(metadata.get("version", 0)) != 2:
        raise ValueError(f"不支持轨迹版本: {metadata.get('version')}")
    available_arms = tuple(metadata.get("arms", ()))
    if not available_arms:
        raise ValueError("metadata.arms 为空")
    for index, frame in enumerate(frames, start=1):
        if "t" not in frame or "arms" not in frame:
            raise ValueError(f"第 {index} 帧字段不完整")
        missing = [arm for arm in available_arms if arm not in frame["arms"]]
        if missing:
            raise ValueError(f"第 {index} 帧缺少机械臂: {missing}")
    return metadata, frames, available_arms, False


def choose_legacy_arm(requested_mode="auto"):
    if requested_mode in ("left", "right"):
        return requested_mode
    if requested_mode == "dual":
        raise ValueError("旧版单臂轨迹不能作为双臂轨迹回放")
    print("旧版轨迹没有左右臂信息:")
    print("  1. 左臂")
    print("  2. 右臂")
    choices = {
        "1": "left", "left": "left", "l": "left",
        "2": "right", "right": "right", "r": "right",
    }
    while True:
        answer = input("请输入 1/2: ").strip().lower()
        if answer in choices:
            return choices[answer]
        print("输入无效，请重新选择。")


def select_replay_mode(requested_mode, metadata, available_arms, legacy):
    if legacy:
        mode = choose_legacy_arm(requested_mode)
        if mode == "dual":
            raise ValueError("旧版轨迹只能选择左臂或右臂")
        return mode
    mode = metadata.get("mode") if requested_mode == "auto" else requested_mode
    if mode not in MODE_ARMS:
        raise ValueError(f"无效回放模式: {mode}")
    missing = [arm for arm in MODE_ARMS[mode] if arm not in available_arms]
    if missing:
        raise ValueError(
            f"轨迹不包含 {missing}，可用机械臂: {available_arms}"
        )
    return mode


def prepare_replay_frames(frames, arms, legacy):
    prepared = {}
    clipped_counts = {}
    for arm in arms:
        source = frames if legacy else [
            {"t": frame["t"], **frame["arms"][arm]}
            for frame in frames
        ]
        prepared[arm] = TrajectoryRecorder._prepare_playback_frames(
            source, PLAYBACK_DT, SMOOTH_WINDOW
        )
        arm_frames = prepared[arm]
        positions = np.asarray(
            [frame["pos"] for frame in arm_frames], dtype=float
        )
        original_j3 = positions[:, J3_JOINT_INDEX].copy()
        positions[:, J3_JOINT_INDEX] = np.clip(
            original_j3,
            -J3_SOFTWARE_LIMIT,
            J3_SOFTWARE_LIMIT,
        )
        clipped = int(np.count_nonzero(
            np.abs(positions[:, J3_JOINT_INDEX] - original_j3) > 1e-12
        ))
        for index, frame in enumerate(arm_frames):
            frame["pos"][J3_JOINT_INDEX] = float(
                positions[index, J3_JOINT_INDEX]
            )
        clipped_counts[arm] = clipped

    reference = np.asarray(
        [frame["t"] for frame in prepared[arms[0]]], dtype=float
    )
    for arm in arms[1:]:
        timestamps = np.asarray(
            [frame["t"] for frame in prepared[arm]], dtype=float
        )
        if timestamps.shape != reference.shape or not np.allclose(
            timestamps, reference, atol=1e-9
        ):
            raise RuntimeError("双臂重采样时间轴不一致")
    if len(reference) < 2:
        return prepared

    # 速度和加速度约束必须通过拉长时间轴实现。
    # 只裁剪 vel 字段不会改变位置目标的实际运动速度。
    maximum_speed = 0.0
    maximum_acceleration = 0.0
    joint_splines = {}
    for arm in arms:
        positions = np.asarray(
            [frame["pos"] for frame in prepared[arm]], dtype=float
        )
        spline = PchipInterpolator(reference, positions, axis=0)
        joint_splines[arm] = spline
        velocity = spline.derivative(1)(reference)
        acceleration = spline.derivative(2)(reference)
        maximum_speed = max(
            maximum_speed, float(np.max(np.abs(velocity)))
        )
        maximum_acceleration = max(
            maximum_acceleration,
            float(np.max(np.abs(acceleration))),
        )

    if PLAYBACK_TIMING_MODE == "original":
        for arm in arms:
            if clipped_counts[arm]:
                print(
                    f"{MODE_LABELS[arm]}: {clipped_counts[arm]} 个采样点的 "
                    f"J3 已裁剪到 ±{J3_SOFTWARE_LIMIT:.2f} rad"
                )
        print(
            f"原速回放: {reference[-1]:.2f}s, "
            f"速度峰值={maximum_speed:.2f} rad/s, "
            f"加速度峰值={maximum_acceleration:.2f} rad/s²"
        )
        print(
            "警告: 原速轨迹超过当前 1.0 rad/s 和 "
            "2.0 rad/s² 设定，请随时准备 Ctrl+C"
        )
        return prepared

    if PLAYBACK_TIMING_MODE != "safe":
        raise ValueError(
            f"不支持的回放时序模式: {PLAYBACK_TIMING_MODE}"
        )

    time_scale = max(
        1.0,
        maximum_speed / PLAYBACK_JOINT_VELOCITY_LIMIT,
        np.sqrt(
            maximum_acceleration / PLAYBACK_JOINT_ACCELERATION_LIMIT
        ),
    )
    scaled_source_time = (reference - reference[0]) * time_scale
    new_time = np.arange(
        0.0,
        scaled_source_time[-1] + PLAYBACK_DT * 0.5,
        PLAYBACK_DT,
    )
    retimed = {}
    for arm in arms:
        old_frames = prepared[arm]
        source_time = np.clip(
            new_time / time_scale + reference[0],
            reference[0],
            reference[-1],
        )
        spline = joint_splines[arm]
        new_positions = spline(source_time)
        new_velocities = spline.derivative(1)(source_time) / time_scale
        new_velocities = np.clip(
            new_velocities,
            -PLAYBACK_JOINT_VELOCITY_LIMIT,
            PLAYBACK_JOINT_VELOCITY_LIMIT,
        )
        has_gripper = "gripper_pos" in old_frames[0]
        if has_gripper:
            old_gripper = np.asarray(
                [frame["gripper_pos"] for frame in old_frames], dtype=float
            )
            gripper_spline = PchipInterpolator(reference, old_gripper)
            new_gripper = gripper_spline(source_time)
            new_gripper_velocity = (
                gripper_spline.derivative(1)(source_time) / time_scale
            )

        arm_frames = []
        for index, timestamp in enumerate(new_time):
            frame = {
                "t": float(timestamp),
                "pos": new_positions[index].tolist(),
                "vel": new_velocities[index].tolist(),
            }
            if has_gripper:
                frame["gripper_pos"] = float(new_gripper[index])
                frame["gripper_vel"] = float(
                    new_gripper_velocity[index]
                )
            arm_frames.append(frame)
        retimed[arm] = arm_frames
        if clipped_counts[arm]:
            print(
                f"{MODE_LABELS[arm]}: {clipped_counts[arm]} 个采样点的 "
                f"J3 已裁剪到 ±{J3_SOFTWARE_LIMIT:.2f} rad"
            )

    print(
        f"轨迹自动重定时: {reference[-1]:.2f}s -> "
        f"{new_time[-1]:.2f}s (x{time_scale:.2f})"
    )
    print(
        f"重定时前峰值: 速度={maximum_speed:.2f} rad/s, "
        f"加速度={maximum_acceleration:.2f} rad/s²"
    )
    return retimed


def move_to_replay_start(robots, prepared):
    """使用同一时钟的 MIT 七次插值移动到回放起点。"""
    print("正在使用 MIT 同步移动到轨迹起点...")
    first_frames = {arm: frames[0] for arm, frames in prepared.items()}
    starts = {
        arm: np.asarray(robot.get_current_pos(), dtype=float)
        for arm, robot in robots.items()
    }
    targets = {
        arm: np.asarray(first_frames[arm]["pos"], dtype=float)
        for arm in robots
    }
    gripper_starts = {
        arm: float(robot.get_current_pos_gripper())
        for arm, robot in robots.items()
    }
    gripper_targets = {
        arm: float(first_frames[arm].get(
            "gripper_pos", gripper_starts[arm]
        ))
        for arm in robots
    }
    max_distance = max(
        float(np.max(np.abs(targets[arm] - starts[arm])))
        for arm in robots
    )
    # 七次插值的峰值速度高于平均速度，按 0.5 rad/s
    # 峰值上限自动拉长时间，并保证至少 5 秒的平滑进入。
    duration = max(5.0, 2.2 * max_distance / 0.5)
    tolerance = 0.05
    settle_timeout = 8.0
    start_time = time.perf_counter()
    next_deadline = start_time
    last_print = start_time - 1.0
    errors = {arm: float("inf") for arm in robots}

    while True:
        now = time.perf_counter()
        elapsed = now - start_time
        trajectory_time = min(elapsed, duration)
        commands = {}
        for arm, robot in robots.items():
            position, velocity, _ = robot.septic_interpolation(
                starts[arm],
                targets[arm],
                duration,
                trajectory_time,
            )
            gripper_position, gripper_velocity, _ = (
                robot.septic_interpolation(
                    np.array([gripper_starts[arm]], dtype=float),
                    np.array([gripper_targets[arm]], dtype=float),
                    duration,
                    trajectory_time,
                )
            )
            if elapsed >= duration:
                position = targets[arm].copy()
                velocity = np.zeros(robot.motor_count, dtype=float)
                gripper_position = np.array([gripper_targets[arm]])
                gripper_velocity = np.zeros(1, dtype=float)
            commands[arm] = (
                position,
                velocity,
                float(gripper_position[0]),
                float(gripper_velocity[0]),
            )

        # 先计算完所有机械臂指令，再在同一周期内下发。
        for arm, robot in robots.items():
            position, velocity, gripper_position, gripper_velocity = (
                commands[arm]
            )
            actual = np.asarray(robot.get_current_pos(), dtype=float)
            gravity = np.asarray(robot.get_Gravity(actual), dtype=float)
            torque_limit = np.asarray(robot.max_torque, dtype=float)
            gravity = np.clip(gravity, -torque_limit, torque_limit)
            success = robot.pos_vel_tqe_kp_kd(
                position.tolist(),
                velocity.tolist(),
                gravity.tolist(),
                KP_MOVE_TO_START,
                KD_MOVE_TO_START,
            )
            if not success:
                raise RuntimeError(
                    f"{MODE_LABELS[arm]} MIT 起点指令被拒绝"
                )
            robot.gripper_control_MIT(
                gripper_position,
                gripper_velocity,
                0.0,
                GRIPPER_KP,
                GRIPPER_KD,
            )

        errors = {
            arm: float(np.max(np.abs(
                np.asarray(robot.get_current_pos(), dtype=float)
                - targets[arm]
            )))
            for arm, robot in robots.items()
        }
        if now - last_print >= 0.2:
            error_text = " | ".join(
                f"{MODE_LABELS[arm]} err={error:.3f} rad"
                for arm, error in errors.items()
            )
            print(
                f"\r  {min(elapsed / duration, 1.0) * 100.0:5.1f}% | "
                f"{error_text}   ",
                end="",
                flush=True,
            )
            last_print = now

        if elapsed >= duration and all(
            error <= tolerance for error in errors.values()
        ):
            print("\n已使用 MIT 到达轨迹起点")
            return
        if elapsed >= duration + settle_timeout:
            details = ", ".join(
                f"{MODE_LABELS[arm]}={error:.3f} rad"
                for arm, error in errors.items()
            )
            raise RuntimeError(f"MIT 移动到轨迹起点超时: {details}")

        next_deadline += PLAYBACK_DT
        remaining = next_deadline - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            next_deadline = time.perf_counter()


def send_replay_frame(robot, frame):
    actual_position = np.asarray(robot.get_current_pos(), dtype=float)
    # 重力补偿必须使用实体当前位形，不能使用已经
    # 跑到前方的轨迹目标，否则肩部落后时前馈力矩也会错。
    torque = np.asarray(robot.get_Gravity(actual_position), dtype=float)
    torque += robot.get_friction_compensation(
        frame["vel"], FC, FV, VELOCITY_THRESHOLD
    )
    torque_limit = np.asarray(robot.max_torque, dtype=float)
    torque = np.clip(torque, -torque_limit, torque_limit)
    success = robot.pos_vel_tqe_kp_kd(
        frame["pos"], frame["vel"], torque.tolist(), KP_PLAY, KD_PLAY
    )
    if not success:
        raise RuntimeError("MIT 回放指令被拒绝")
    if "gripper_pos" in frame:
        robot.gripper_control_MIT(
            frame["gripper_pos"],
            frame.get("gripper_vel", 0.0),
            0.0,
            GRIPPER_KP,
            GRIPPER_KD,
        )
    return float(np.max(np.abs(
        actual_position - np.asarray(frame["pos"], dtype=float)
    )))


def replay_frames(robots, prepared):
    arms = tuple(robots)
    frame_count = len(prepared[arms[0]])
    start_time = time.perf_counter()
    last_print = start_time - 1.0
    for index in range(frame_count):
        timestamp = prepared[arms[0]][index]["t"]
        while True:
            remaining = timestamp - (time.perf_counter() - start_time)
            if remaining <= 0.0:
                break
            if remaining > 0.0005:
                time.sleep(remaining - 0.0005)
        errors = {
            arm: send_replay_frame(robot, prepared[arm][index])
            for arm, robot in robots.items()
        }
        if max(errors.values()) > PLAYBACK_TRACKING_ABORT_ERROR:
            details = ", ".join(
                f"{MODE_LABELS[arm]}={error:.3f} rad"
                for arm, error in errors.items()
            )
            raise RuntimeError(f"回放跟踪误差过大: {details}")
        now = time.perf_counter()
        if now - last_print >= 0.2:
            details = " | ".join(
                f"{MODE_LABELS[arm]} err={error:.3f} rad"
                for arm, error in errors.items()
            )
            print(
                f"\r  {100.0 * index / max(frame_count - 1, 1):5.1f}% | "
                f"{details}   ",
                end="",
                flush=True,
            )
            last_print = now
    print()
    print("回放完成")


def replay_trajectory(filepath, requested_mode="auto"):
    filepath = Path(filepath).resolve()
    metadata, frames, available_arms, legacy = load_trajectory(filepath)
    mode = select_replay_mode(
        requested_mode, metadata, available_arms, legacy
    )
    arms = MODE_ARMS[mode]
    prepared = prepare_replay_frames(frames, arms, legacy)
    robots = create_robots(mode)
    try:
        print(f"\n轨迹文件: {filepath}")
        print(f"数据格式: {'旧版单臂' if legacy else 'v2'}")
        print(f"回放模式: {MODE_LABELS[mode]}")
        print(f"回放帧数: {len(prepared[arms[0]])}")
        move_to_replay_start(robots, prepared)
        replay_frames(robots, prepared)
    finally:
        stop_robots(robots)
        print("所选机械臂电机已停止")


def interactive_menu():
    """无参数运行时的录制/回放集成菜单。"""
    default_trajectory = None
    while True:
        available = find_trajectory_files()
        disk_default = available[0] if available else None
        current_default = default_trajectory or disk_default
        print("\n" + "=" * 60)
        print("openarm_HT 轨迹录制/回放")
        print("=" * 60)
        print("  1. 录制新轨迹")
        if current_default is not None:
            print(f"  2. 回放默认轨迹: {current_default.name}")
        else:
            print("  2. 回放默认轨迹（暂无）")
        print("  3. 选择历史轨迹回放")
        print("  4. 退出")
        answer = input("请输入 1/2/3/4: ").strip()
        try:
            if answer == "1":
                mode = choose_mode()
                default_trajectory = record_trajectory(mode)
                print(f"新轨迹已设为默认: {default_trajectory.name}")
            elif answer == "2":
                if current_default is None:
                    print("没有可回放的轨迹。")
                else:
                    replay_trajectory(current_default, "auto")
            elif answer == "3":
                selected = choose_trajectory_file(current_default)
                default_trajectory = selected
                replay_trajectory(selected, "auto")
            elif answer == "4":
                print("程序退出")
                return
            else:
                print("输入无效。")
        except KeyboardInterrupt:
            print("\n当前操作已中断")
        except Exception as error:
            print(f"\n错误: {error}")


def build_parser():
    parser = argparse.ArgumentParser(
        description="openarm_HT 左臂/右臂/双臂轨迹录制与回放"
    )
    subparsers = parser.add_subparsers(dest="action")
    record_parser = subparsers.add_parser("record", help="录制新轨迹")
    record_parser.add_argument(
        "--arm", choices=("left", "right", "dual"), required=True
    )
    record_parser.add_argument("-o", "--output", help="输出 JSONL 路径")
    replay_parser = subparsers.add_parser("replay", help="回放轨迹")
    replay_parser.add_argument(
        "trajectory_file", nargs="?", help="轨迹文件；默认使用最新文件"
    )
    replay_parser.add_argument(
        "--arm",
        choices=("auto", "left", "right", "dual"),
        default="auto",
    )
    subparsers.add_parser("list", help="列出历史轨迹")
    return parser


def command_line_main(arguments):
    if arguments.action is None:
        interactive_menu()
    elif arguments.action == "record":
        path = record_trajectory(arguments.arm, arguments.output)
        print(f"新轨迹: {path}")
    elif arguments.action == "replay":
        if arguments.trajectory_file:
            path = resolve_trajectory_file(arguments.trajectory_file)
            if not path.is_file():
                raise FileNotFoundError(f"轨迹文件不存在: {path}")
        else:
            files = find_trajectory_files()
            if not files:
                raise FileNotFoundError("没有可回放的 JSONL 轨迹")
            path = files[0]
            print(f"未指定文件，使用最新轨迹: {path.name}")
        replay_trajectory(path, arguments.arm)
    elif arguments.action == "list":
        files = find_trajectory_files()
        if not files:
            print("没有 JSONL 轨迹")
        for index, path in enumerate(files, start=1):
            print(f"{index:2d}. {path}")


if __name__ == "__main__":
    try:
        command_line_main(build_parser().parse_args())
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as error:
        print(f"\n错误: {error}")
        sys.exit(1)
