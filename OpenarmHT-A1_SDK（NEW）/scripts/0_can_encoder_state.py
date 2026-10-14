#!/usr/bin/env python3
"""读取 OpenArm 通信板指定 CAN 口上 ID=1 的电机编码器反馈。

物理 CAN 口由 SDK 的 ``serial_id`` 选择。脚本只调用状态请求接口，
不会发送位置、速度、力矩、回零或标定命令；退出时 SDK 自动 motor stop。
"""

import argparse
import math
import os
import tempfile
import time
from pathlib import Path

import yaml

import hightorque_robot as htr


SCRIPT_DIR = Path(__file__).resolve().parent
SOURCE_CONFIG = (
    SCRIPT_DIR.parent
    / "robot_param"
    / "motor_param"
    / "7dof_openarm_HT_params_right.yaml"
)
DEFAULT_CAN_PORT = 3
ENCODER_MOTOR_ID = 1
INVALID_FEEDBACK_ABS = 900.0
ENCODER_COUNTS_PER_TURN = 10000
SIGNED_COUNT_MIN = -32768
SIGNED_COUNT_MAX = 32767


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--can-port",
        type=int,
        choices=range(1, 8),
        default=DEFAULT_CAN_PORT,
        metavar="1..7",
        help=f"通信板物理 CAN 口，默认 CAN{DEFAULT_CAN_PORT}",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=0,
        help="采样次数；0 表示持续读取直到 Ctrl+C",
    )
    parser.add_argument(
        "--all-ids",
        action="store_true",
        help="扫描源配置中的 ID 1–8；默认只读取已确认的 ID=1",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.20,
        help="状态刷新间隔，默认 0.20 秒",
    )
    return parser.parse_args()


def make_can_read_config(physical_can_port, all_ids=False):
    """生成临时只读配置，不改动机械臂原配置。"""
    with SOURCE_CONFIG.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)

    robot = config["robot"]
    robot["robot_name"] = f"openarm_can{physical_can_port}_encoder_reader"
    robot["motor_timeout_ms"] = 0
    # 退出时也不额外发送制动/释放指令。
    robot["exit_motor_brake_flag"] = False

    can_board = robot["CANboard"]["No_1_CANboard"]
    can_board["CANport_num"] = 1
    can_port = can_board["CANport"]["CANport_1"]
    can_port["serial_id"] = physical_can_port
    if not all_ids:
        configured_motors = can_port["motor"]
        matching = [
            motor_config
            for motor_config in configured_motors.values()
            if int(motor_config["id"]) == ENCODER_MOTOR_ID
        ]
        if len(matching) != 1:
            raise RuntimeError(
                f"源配置中应恰好有一个 ID={ENCODER_MOTOR_ID}，实际 {len(matching)} 个"
            )
        encoder_config = dict(matching[0])
        encoder_config["name"] = f"can{physical_can_port}_encoder_id1"
        encoder_config["num"] = 1
        can_port["motor_num"] = 1
        can_port["motor"] = {"motor1": encoder_config}

    handle = tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".yaml",
        prefix=f"openarm_can{physical_can_port}_read_",
        delete=False,
        encoding="utf-8",
    )
    try:
        yaml.safe_dump(config, handle, allow_unicode=True, sort_keys=False)
        return Path(handle.name)
    finally:
        handle.close()


def version_tuple(motor):
    version = motor.get_version()
    return int(version.major), int(version.minor), int(version.patch)


def feedback_is_valid(state, version):
    values = (float(state.position), float(state.velocity), float(state.torque))
    return (
        version != (0, 0, 0)
        and all(math.isfinite(value) for value in values)
        and all(abs(value) < INVALID_FEEDBACK_ABS for value in values)
    )


def request_state(robot):
    # 未设置任何控制缓存时，motor_send_cmd 只发出状态请求。
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()


def main():
    args = parse_args()
    if args.samples < 0:
        raise ValueError("--samples 不能为负数")
    if args.interval < 0.02:
        raise ValueError("--interval 不能小于 0.02 秒")

    print(f"OPENARM CAN{args.can_port} ENCODER READ ONLY")
    print("只请求状态；不发送位置、速度、力矩、回零或标定命令。")
    print("退出时底层 SDK 会自动进入 motor stop。")
    print(f"物理端口: CAN{args.can_port} (serial_id={args.can_port})")
    print(
        "编码器设备: "
        + ("扫描 ID=1–8" if args.all_ids else f"motor ID={ENCODER_MOTOR_ID}")
    )
    print(f"扫描配置: {SOURCE_CONFIG}")

    temporary_config = make_can_read_config(args.can_port, args.all_ids)
    robot = None
    previous_positions = {}
    sample_index = 0
    try:
        robot = htr.Robot(str(temporary_config))
        motors = robot.get_motors()
        print(f"SDK 已创建 {len(motors)} 个候选电机对象，开始读取。")

        # 初始化后多刷新几次，避免把默认缓存误判为真实编码器值。
        for _ in range(5):
            request_state(robot)
            time.sleep(0.05)

        while args.samples == 0 or sample_index < args.samples:
            request_state(robot)
            time.sleep(0.02)
            sample_index += 1
            valid_count = 0
            timestamp = time.strftime("%H:%M:%S")
            print(f"\n[{timestamp}] sample={sample_index}")

            for motor in motors:
                motor_id = int(motor.get_motor_id())
                state = motor.get_current_motor_state()
                version = version_tuple(motor)
                valid = feedback_is_valid(state, version)
                if valid:
                    valid_count += 1
                    position = float(state.position)
                    previous = previous_positions.get(motor_id)
                    delta_text = "首次"
                    if previous is not None:
                        delta = position - previous
                        delta_text = (
                            f"Δ={delta:+.6f} rad/"
                            f"{math.degrees(delta):+.3f}°"
                        )
                    previous_positions[motor_id] = position
                    signed_count = int(round(
                        position * ENCODER_COUNTS_PER_TURN / (2.0 * math.pi)
                    ))
                    single_turn_count = signed_count % ENCODER_COUNTS_PER_TURN
                    accumulated_degrees = (
                        signed_count * 360.0 / ENCODER_COUNTS_PER_TURN
                    )
                    single_turn_degrees = (
                        single_turn_count * 360.0 / ENCODER_COUNTS_PER_TURN
                    )
                    turns = signed_count / ENCODER_COUNTS_PER_TURN
                    range_text = ""
                    if signed_count <= SIGNED_COUNT_MIN or signed_count >= SIGNED_COUNT_MAX:
                        range_text = " [达到16位计数边界]"
                    print(
                        f"  [有效] ID={motor_id:2d} "
                        f"type={motor.get_motor_enum_type()} "
                        f"version=v{version[0]}.{version[1]}.{version[2]} "
                        f"pos={position:+.6f} rad "
                        f"累计={accumulated_degrees:+.3f}° "
                        f"单圈={single_turn_degrees:7.3f}° "
                        f"圈数={turns:+.5f} "
                        f"计数={signed_count:+6d} "
                        f"单圈计数={single_turn_count:4d} "
                        f"vel={float(state.velocity):+.6f} rad/s "
                        f"torque={float(state.torque):+.4f} Nm "
                        f"mode={state.mode} fault=0x{int(state.fault):02X} "
                        f"{delta_text}{range_text}"
                    )

            if valid_count == 0:
                print("  未收到有效电机编码器反馈（版本为 0 或反馈值无效）。")
            else:
                print(f"  CAN{args.can_port} 有效反馈数量: {valid_count}")

            remaining = args.interval - 0.02
            if remaining > 0.0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\n停止读取。")
    finally:
        # 有意不调用 reset/zero 或任何运动控制；SDK 析构时自动 motor stop。
        try:
            os.unlink(temporary_config)
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    main()
