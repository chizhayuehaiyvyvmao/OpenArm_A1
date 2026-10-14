#!/usr/bin/env python3
"""左臂、右臂或双臂 SSH 键盘速度控制与完整 URDF 精确碰撞检查。"""

import os
import time

import numpy as np
import pinocchio as pin

from Lib import Openarm
from Lib.keyboard_arm_utils import (
    create_arm_states,
    mirror_twist,
    move_selected_joints,
    safe_targets,
    stop_states,
    zero_targets,
)
from Lib.terminal_keyboard import keyboard


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
end_velocity = np.zeros(6)
linear_speed = 0.10
angular_speed = 0.50
acceleration_factor = 0.08
running = True
keys_pressed = set()


class ExactSelectedArmsCollisionGuard:
    """使用完整 V2 collision 几何检查所选机械臂及双臂交叉碰撞。"""

    def __init__(self, states):
        self.states = states
        self.model = states[0]["robot"].model
        self.data = self.model.createData()
        robot = states[0]["robot"]
        self.urdf_path = os.path.realpath(
            os.path.join(robot.config_dir, robot.config["urdf"]["file_path"])
        )
        urdf_dir = os.path.dirname(self.urdf_path)
        old_dir = os.getcwd()
        try:
            os.chdir(urdf_dir)
            self.geometry_model = pin.buildGeomFromUrdf(
                self.model,
                self.urdf_path,
                pin.GeometryType.COLLISION,
                package_dirs=[urdf_dir, os.path.dirname(urdf_dir)],
            )
        finally:
            os.chdir(old_dir)

        self.geometry_model.addAllCollisionPairs()
        self.geometry_data = pin.GeometryData(self.geometry_model)
        self.selected_labels = {state["label"] for state in states}
        self.active_pair_indices = self._active_pairs()
        print(
            f"精确碰撞模型: {len(self.geometry_model.geometryObjects)} 个几何体, "
            f"{len(self.active_pair_indices)} 对非相邻检测对"
        )

    @staticmethod
    def _belongs_to_side(name, prefix):
        return name.startswith(prefix)

    def _active_pairs(self):
        active = []
        for index, pair in enumerate(self.geometry_model.collisionPairs):
            first = self.geometry_model.geometryObjects[pair.first]
            second = self.geometry_model.geometryObjects[pair.second]
            if "左臂" not in self.selected_labels and (
                self._belongs_to_side(first.name, "l_")
                or self._belongs_to_side(second.name, "l_")
            ):
                continue
            if "右臂" not in self.selected_labels and (
                self._belongs_to_side(first.name, "r_")
                or self._belongs_to_side(second.name, "r_")
            ):
                continue
            joint_a = first.parentJoint
            joint_b = second.parentJoint
            if joint_a == joint_b:
                continue
            if (
                self.model.parents[joint_a] == joint_b
                or self.model.parents[joint_b] == joint_a
            ):
                continue
            active.append(index)
        return active

    def model_configuration(self, positions):
        full = np.zeros(self.model.nq)
        for state in self.states:
            robot = state["robot"]
            values = np.asarray(positions[state["label"]], dtype=float)
            for index, joint_name in enumerate(robot.joint_names):
                joint_id = self.model.getJointId(joint_name)
                q_index = self.model.joints[joint_id].idx_q
                full[q_index] = robot.joint_signs[index] * values[index]
        return full

    def collisions(self, positions):
        q_full = self.model_configuration(positions)
        pin.computeCollisions(
            self.model,
            self.data,
            self.geometry_model,
            self.geometry_data,
            q_full,
            False,
        )
        result = []
        for pair_index in self.active_pair_indices:
            if self.geometry_data.collisionResults[pair_index].isCollision():
                pair = self.geometry_model.collisionPairs[pair_index]
                result.append((
                    self.geometry_model.geometryObjects[pair.first].name,
                    self.geometry_model.geometryObjects[pair.second].name,
                ))
        return result

    def check_prediction(self, velocities, horizon=0.20, steps=8):
        current = {
            state["label"]: np.asarray(
                state["robot"].get_current_pos(),
                dtype=float,
            )
            for state in self.states
        }
        for step in range(steps + 1):
            future_time = horizon * step / steps
            positions = {}
            for state in self.states:
                label = state["label"]
                robot = state["robot"]
                raw = current[label] + velocities[label] * future_time
                lower = np.asarray(robot.joint_limits["lower"], dtype=float)
                upper = np.asarray(robot.joint_limits["upper"], dtype=float)
                if np.any(raw < lower) or np.any(raw > upper):
                    joint = int(np.argmax(np.maximum(lower - raw, raw - upper))) + 1
                    return {
                        "kind": "joint_limit",
                        "label": label,
                        "joint": joint,
                        "time": future_time,
                    }
                positions[label] = raw
            collided = self.collisions(positions)
            if collided:
                return {
                    "kind": "collision",
                    "pair": collided[0],
                    "time": future_time,
                }
        return None


def on_press(key):
    if hasattr(key, "char") and key.char:
        char = key.char.lower()
        keys_pressed.add(char)
        mapping = {
            "w": (0, linear_speed),
            "s": (0, -linear_speed),
            "a": (1, linear_speed),
            "d": (1, -linear_speed),
            "q": (2, linear_speed),
            "e": (2, -linear_speed),
            "1": (3, angular_speed),
            "2": (3, -angular_speed),
            "3": (4, angular_speed),
            "4": (4, -angular_speed),
            "5": (5, angular_speed),
            "6": (5, -angular_speed),
        }
        if char in mapping:
            index, value = mapping[char]
            end_velocity[index] = value


def on_release(key):
    global running
    if key == keyboard.Key.esc:
        running = False
        print("\n检测到 ESC，准备退出...")
        return False
    if hasattr(key, "char") and key.char:
        char = key.char.lower()
        keys_pressed.discard(char)
        for choices, index in (
            (("w", "s"), 0),
            (("a", "d"), 1),
            (("q", "e"), 2),
            (("1", "2"), 3),
            (("3", "4"), 4),
            (("5", "6"), 5),
        ):
            if char in choices:
                end_velocity[index] = 0.0


def stop_motion(states, kd):
    for state in states:
        robot = state["robot"]
        q = np.asarray(robot.get_current_pos(), dtype=float)
        gravity = np.asarray(robot.get_Gravity(q), dtype=float)
        robot.pos_vel_tqe_kp_kd(
            q,
            [0.0] * robot.motor_count,
            gravity,
            [0.0] * robot.motor_count,
            kd,
        )


def main():
    global running, end_velocity
    running = True
    end_velocity[:] = 0.0
    states = []
    listener = None
    kd = [10.0, 15.0, 15.0, 10.0, 5.0, 5.0, 5.0]
    try:
        states = create_arm_states(Openarm, SCRIPT_DIR)
        move_selected_joints(states, safe_targets(states), "移动到镜像安全姿态")
        guard = ExactSelectedArmsCollisionGuard(states)
        initial_positions = {
            state["label"]: np.asarray(
                state["robot"].get_current_pos(),
                dtype=float,
            )
            for state in states
        }
        initial_collisions = guard.collisions(initial_positions)
        if initial_collisions:
            raise RuntimeError(f"初始姿态存在碰撞: {initial_collisions[0]}")

        for state in states:
            state["actual_twist"] = np.zeros(6)
        print("\nW/S:X  A/D:Y  Q/E:Z  1/2:Rx  3/4:Ry  5/6:Rz  ESC:退出")
        print("提示：使用精确网格碰撞，但不包含夹爪几何和实测制动距离。")
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        last_print = time.monotonic() - 1.0
        last_block = None
        while running:
            joint_velocities = {}
            summaries = []
            for state in states:
                robot = state["robot"]
                q = np.asarray(robot.get_current_pos(), dtype=float)
                jacobian = robot.get_jacobian(q)
                manipulability = robot.get_manipulability(q)
                damping = 0.01
                scale = 1.0
                if manipulability < 0.01:
                    damping *= 1.0 + (0.01 - manipulability) * 100.0
                    scale = max(0.0, manipulability / 0.01)
                desired = mirror_twist(
                    end_velocity * scale,
                    state["mirror_sign"],
                )
                state["actual_twist"] += (
                    desired - state["actual_twist"]
                ) * acceleration_factor
                q_dot = Openarm.compute_damped_pseudoinverse(
                    jacobian,
                    damping,
                ) @ state["actual_twist"]
                q_dot = np.clip(q_dot, -1.0, 1.0)
                joint_velocities[state["label"]] = q_dot
                position = robot.forward_kinematics(q)["position"]
                summaries.append(
                    f"{state['label']} pos={np.round(position, 3)} "
                    f"v={np.linalg.norm(state['actual_twist'][:3]):.3f}"
                )

            blocked = guard.check_prediction(joint_velocities)
            if blocked is not None:
                end_velocity[:] = 0.0
                for state in states:
                    state["actual_twist"][:] = 0.0
                    joint_velocities[state["label"]] = np.zeros(
                        state["robot"].motor_count
                    )
                if blocked != last_block:
                    if blocked["kind"] == "collision":
                        print(
                            f"\n预测碰撞，已停止: {blocked['pair'][0]} <-> "
                            f"{blocked['pair'][1]} @ {blocked['time']:.3f}s"
                        )
                    else:
                        print(
                            f"\n预测限位，已停止: {blocked['label']} "
                            f"J{blocked['joint']} @ {blocked['time']:.3f}s"
                        )
                last_block = blocked
            else:
                last_block = None

            for state in states:
                state["robot"].Joint_Vel(joint_velocities[state["label"]])

            now = time.monotonic()
            if now - last_print >= 0.2:
                status = "阻止" if blocked else "安全"
                print("\r" + " | ".join(summaries) + f" | {status}", end="")
                last_print = now
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        if listener is not None:
            listener.stop()
        end_velocity[:] = 0.0
        if states:
            stop_motion(states, kd)
            time.sleep(0.3)
            try:
                move_selected_joints(states, zero_targets(states), "回到零位")
            finally:
                stop_states(states)
        print("\n所选机械臂已停止")


if __name__ == "__main__":
    main()
