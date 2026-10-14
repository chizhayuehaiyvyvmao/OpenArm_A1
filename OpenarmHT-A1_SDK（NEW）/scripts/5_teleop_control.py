#!/usr/bin/env python3
"""无参数入口：CAN3 左主臂 -> CAN1，CAN4 右主臂 -> CAN2。"""

import sys

# 保留 0_can3_encoder_set_zero.py 使用的三个接口；导入不会连接机械臂。
from encoder_teleop_single import EncoderLeader, LEADER_CONFIG, format_vector
from dual_encoder_teleop import main


if __name__ == "__main__":
    # 普通 python 启动也立即显示状态，不需要用户传 -u。
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(line_buffering=True)
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"[ERROR] 双臂遥操未能运行: {error}", flush=True)
        raise SystemExit(1)
