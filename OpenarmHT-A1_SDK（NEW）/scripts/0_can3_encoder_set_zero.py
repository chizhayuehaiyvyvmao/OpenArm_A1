#!/usr/bin/env python3
"""Set CAN3/CAN4 encoder leader ID1~8 hardware zero to the OpenArm zero pose.

Run this script only while the passive leader is held at the OpenArm zero pose.
CAN3 is the left leader and CAN4 is the right leader. It never connects to or
commands the CAN1/CAN2 follower arms.
"""

from __future__ import annotations

import argparse
import importlib.util
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
TELEOP_SCRIPT = SCRIPT_DIR / "5_teleop_control.py"
BASE_CONFIG = (
    SCRIPT_DIR.parent
    / "robot_param"
    / "motor_param"
    / "7dof_openarm_encoder_leader_can3.yaml"
)
ZERO_VERIFY_TOLERANCE = 0.05
ZERO_PROCESS_TIMEOUT = 30.0
PORT_LABELS = {3: "左", 4: "右"}


def load_teleop_support():
    spec = importlib.util.spec_from_file_location("openarm_encoder_teleop", TELEOP_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载遥操支持模块: {TELEOP_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_leader_config(can_port: int, destination: Path) -> Path:
    if can_port not in PORT_LABELS:
        raise ValueError(f"只允许编码器主臂 CAN3/CAN4，收到 CAN{can_port}")
    config = yaml.safe_load(BASE_CONFIG.read_text(encoding="utf-8"))
    robot = config["robot"]
    robot["robot_name"] = f"openarm_encoder_leader_can{can_port}_zero"
    can_config = robot["CANboard"]["No_1_CANboard"]["CANport"]["CANport_1"]
    can_config["serial_id"] = can_port
    destination.write_text(
        yaml.safe_dump(config, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return destination


def zero_worker(can_port: int) -> int:
    try:
        teleop = load_teleop_support()
        with tempfile.TemporaryDirectory(prefix=f"openarm_can{can_port}_zero_") as directory:
            config_path = write_leader_config(
                can_port, Path(directory) / f"encoder_leader_can{can_port}.yaml"
            )
            leader = teleop.EncoderLeader(config_path)
            before_q, before_gripper = leader.read()
            print(
                "[ZERO] 当前姿态将作为 OpenArm 零位："
                f"q={teleop.format_vector(before_q)}, "
                f"gripper={before_gripper:+.3f} rad",
                flush=True,
            )
            print(
                f"[ZERO] 正在把 CAN{can_port} ID1~8 写入硬件零位，"
                "请保持主臂不动…",
                flush=True,
            )
            leader.robot.set_reset_zero()
            time.sleep(0.8)
            for _ in range(8):
                leader.request_state()
                time.sleep(0.05)
            after_q, after_gripper = leader.read()
            max_abs = max(float(np.max(np.abs(after_q))), abs(after_gripper))
            if max_abs > ZERO_VERIFY_TOLERANCE:
                raise RuntimeError(
                    "置零后回读未接近零位，主臂可能在保存期间移动："
                    f"q={teleop.format_vector(after_q)}, "
                    f"gripper={after_gripper:+.3f} rad, max={max_abs:.3f} rad；"
                    "请放回 OpenArm 零位并重新运行本脚本"
                )
            print(
                f"[ZERO-OK] CAN{can_port} ID1~8 硬件零位已保存，"
                "与 OpenArm 零位一致："
                f"q={teleop.format_vector(after_q)}, "
                f"gripper={after_gripper:+.3f} rad",
                flush=True,
            )
        return 0
    except Exception as error:
        print(f"[ZERO-ERROR] {error}", flush=True)
        return 1


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="把左 CAN3 或右 CAN4 编码器主臂当前姿态写为硬件零位"
    )
    parser.add_argument(
        "--can-port",
        type=int,
        choices=tuple(PORT_LABELS),
        default=3,
        help="编码器主臂端口：3=左，4=右（默认：3）",
    )
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.worker:
        return zero_worker(args.can_port)

    print("=" * 72)
    print(f"CAN{args.can_port} {PORT_LABELS[args.can_port]}编码器主臂硬件置零")
    print("请先把七个关节和夹爪放到 OpenArm 零位，运行期间保持不动。")
    print(
        f"本程序只连接 CAN{args.can_port} 编码器主臂，"
        "不会连接或移动 CAN1/CAN2 从臂。"
    )
    print("=" * 72, flush=True)
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        "--can-port",
        str(args.can_port),
    ]
    try:
        result = subprocess.run(command, timeout=ZERO_PROCESS_TIMEOUT, check=False)
    except subprocess.TimeoutExpired:
        print(
            f"[ZERO-ERROR] 置零超过 {ZERO_PROCESS_TIMEOUT:.0f}s，已终止；"
            f"请检查 CAN{args.can_port} 通信和 ID1~8 反馈",
            flush=True,
        )
        return 1
    return int(result.returncode)


if __name__ == "__main__":
    raise SystemExit(main())
