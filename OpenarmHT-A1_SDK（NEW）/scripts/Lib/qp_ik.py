"""openarm_HT 有界速度级 QP 逆运动学。"""

from dataclasses import dataclass

import numpy as np
import osqp
from scipy.sparse import csc_matrix, eye
from scipy.spatial.transform import Rotation


@dataclass
class QPIKResult:
    joint_position: np.ndarray
    joint_velocity: np.ndarray
    position_error: float
    rotation_error: float
    status: str
    iterations: int


class PinocchioOSQPIK:
    """求解一次带速度、加速度和位置边界的差分 IK。"""

    def __init__(
        self,
        robot,
        position_weight=100.0,
        rotation_weight=2.0,
        position_gain=8.0,
        rotation_gain=5.0,
        damping=1e-3,
        smoothness=2e-2,
        posture_weight=2e-2,
        posture_gain=1.0,
        posture_joint_weights=None,
        max_joint_velocity=1.0,
        max_joint_acceleration=4.0,
        max_linear_velocity=0.6,
        max_angular_velocity=1.5,
        joint_limit_margin=0.01,
        joint_limit_slowdown_distance=0.18,
        singularity_threshold=0.0,
        singularity_damping=0.0,
    ):
        self.robot = robot
        self.count = int(robot.motor_count)
        self.position_weight = float(position_weight)
        self.rotation_weight = float(rotation_weight)
        self.position_gain = float(position_gain)
        self.rotation_gain = float(rotation_gain)
        self.damping = float(damping)
        self.smoothness = float(smoothness)
        self.posture_weight = float(posture_weight)
        self.posture_gain = float(posture_gain)
        if posture_joint_weights is None:
            self.posture_joint_weights = np.ones(self.count, dtype=float)
        else:
            self.posture_joint_weights = np.asarray(
                posture_joint_weights, dtype=float
            )
            if self.posture_joint_weights.shape != (self.count,):
                raise ValueError("QP IK 姿态关节权重维度错误")
            if (
                not np.all(np.isfinite(self.posture_joint_weights))
                or np.any(self.posture_joint_weights < 0.0)
            ):
                raise ValueError("QP IK 姿态关节权重必须是有限非负数")
        self.max_linear_velocity = float(max_linear_velocity)
        self.max_angular_velocity = float(max_angular_velocity)
        self.joint_limit_margin = float(joint_limit_margin)
        self.joint_limit_slowdown_distance = float(
            joint_limit_slowdown_distance
        )
        self.singularity_threshold = float(singularity_threshold)
        self.singularity_damping = float(singularity_damping)
        if self.singularity_threshold < 0.0:
            raise ValueError("QP IK 奇异值阈值不能为负数")
        if self.singularity_damping < 0.0:
            raise ValueError("QP IK 奇异阻尼不能为负数")
        self.last_min_singular_value = float("inf")
        self.last_adaptive_damping = 0.0
        self.previous_velocity = np.zeros(self.count, dtype=float)
        self.posture_reference = None

        velocity_limit = np.full(
            self.count, float(max_joint_velocity), dtype=float
        )
        if getattr(robot, "velocity_limits", None) is not None:
            configured = np.asarray(robot.velocity_limits, dtype=float)
            if configured.shape != (self.count,):
                raise ValueError("QP IK 关节速度限制维度错误")
            velocity_limit = np.minimum(velocity_limit, configured)
        self.velocity_limit = velocity_limit

        acceleration_limit = np.full(
            self.count, float(max_joint_acceleration), dtype=float
        )
        if getattr(robot, "acceleration_limits", None) is not None:
            configured = np.asarray(robot.acceleration_limits, dtype=float)
            if configured.shape != (self.count,):
                raise ValueError("QP IK 关节加速度限制维度错误")
            acceleration_limit = np.minimum(acceleration_limit, configured)
        self.acceleration_limit = acceleration_limit

        self.lower = None
        self.upper = None
        if robot.joint_limits is not None:
            self.lower = np.asarray(
                robot.joint_limits["lower"], dtype=float
            ).copy()
            self.upper = np.asarray(
                robot.joint_limits["upper"], dtype=float
            ).copy()
            if self.lower.shape != (self.count,) or self.upper.shape != (
                self.count,
            ):
                raise ValueError("QP IK 关节位置限制维度错误")

    def reset(self, velocity=None, posture_reference=None):
        """清空跨周期速度；可同时更新 openarm_HT 冗余姿态参考。"""
        if velocity is None:
            self.previous_velocity.fill(0.0)
        else:
            velocity = np.asarray(velocity, dtype=float)
            if velocity.shape != (self.count,):
                raise ValueError("QP IK 速度维度错误")
            self.previous_velocity = velocity.copy()

        if posture_reference is not None:
            posture_reference = np.asarray(posture_reference, dtype=float)
            if posture_reference.shape != (self.count,):
                raise ValueError("QP IK 姿态参考维度错误")
            self.posture_reference = posture_reference.copy()

    @staticmethod
    def _clip_norm(vector, limit):
        vector = np.asarray(vector, dtype=float)
        if not np.isfinite(limit):
            return vector
        norm = float(np.linalg.norm(vector))
        if norm > limit:
            return vector * (limit / norm)
        return vector

    def _velocity_bounds(self, joint, dt):
        lower = -self.velocity_limit.copy()
        upper = self.velocity_limit.copy()

        acceleration_step = self.acceleration_limit * dt
        lower = np.maximum(
            lower, self.previous_velocity - acceleration_step
        )
        upper = np.minimum(
            upper, self.previous_velocity + acceleration_step
        )

        if self.lower is not None:
            safe_lower = self.lower + self.joint_limit_margin
            safe_upper = self.upper - self.joint_limit_margin
            position_lower = (safe_lower - joint) / dt
            position_upper = (safe_upper - joint) / dt
            constrained_lower = np.maximum(lower, position_lower)
            constrained_upper = np.minimum(upper, position_upper)

            slowdown = self.joint_limit_slowdown_distance
            if slowdown > 0.0:
                lower_scale = np.clip(
                    (joint - safe_lower) / slowdown, 0.0, 1.0
                )
                upper_scale = np.clip(
                    (safe_upper - joint) / slowdown, 0.0, 1.0
                )
                constrained_lower = np.maximum(
                    constrained_lower,
                    -self.velocity_limit * lower_scale,
                )
                constrained_upper = np.minimum(
                    constrained_upper,
                    self.velocity_limit * upper_scale,
                )

            # 位置安全边界与加速度边界冲突时，优先保证不会继续越限。
            conflict = constrained_lower > constrained_upper
            for index in np.flatnonzero(conflict):
                distance_lower = joint[index] - safe_lower[index]
                distance_upper = safe_upper[index] - joint[index]
                if distance_upper <= distance_lower:
                    constrained_lower[index] = constrained_upper[index]
                else:
                    constrained_upper[index] = constrained_lower[index]
            lower, upper = constrained_lower, constrained_upper

        if np.any(lower > upper):
            raise RuntimeError("QP IK 关节约束不可行")
        return lower, upper

    def step(self, joint, target_transform, dt, feedforward_twist=None):
        joint = np.asarray(joint, dtype=float)
        target = np.asarray(target_transform, dtype=float)
        if joint.shape != (self.count,) or target.shape != (4, 4):
            raise ValueError("QP IK 关节或目标位姿维度错误")
        if not np.all(np.isfinite(joint)) or not np.all(np.isfinite(target)):
            raise ValueError("QP IK 关节和目标位姿必须为有限值")
        if dt <= 0.0 or not np.isfinite(dt):
            raise ValueError("QP IK dt 必须为有限正数")

        if feedforward_twist is None:
            feedforward_twist = np.zeros(6, dtype=float)
        else:
            feedforward_twist = np.asarray(
                feedforward_twist, dtype=float
            )
            if feedforward_twist.shape != (6,):
                raise ValueError("QP IK 前馈速度必须为 6 维")
        if not np.all(np.isfinite(feedforward_twist)):
            raise ValueError("QP IK 前馈速度必须为有限值")

        fk = self.robot.forward_kinematics(joint)
        jacobian = self.robot.get_jacobian(joint)
        if fk is None or jacobian is None:
            raise RuntimeError("无法计算 openarm_HT QP IK 运动学")
        current = np.asarray(fk["transform"], dtype=float)
        jacobian = np.asarray(jacobian, dtype=float)
        if jacobian.shape != (6, self.count):
            raise ValueError(
                f"QP IK 雅可比应为 (6, {self.count})，实际为 {jacobian.shape}"
            )

        position_error_vector = target[:3, 3] - current[:3, 3]
        rotation_error_vector = Rotation.from_matrix(
            target[:3, :3] @ current[:3, :3].T
        ).as_rotvec()
        desired_linear = self._clip_norm(
            self.position_gain * position_error_vector
            + feedforward_twist[:3],
            self.max_linear_velocity,
        )
        desired_angular = self._clip_norm(
            self.rotation_gain * rotation_error_vector
            + feedforward_twist[3:],
            self.max_angular_velocity,
        )
        desired_twist = np.concatenate([desired_linear, desired_angular])

        weights = np.diag(
            [self.position_weight] * 3
            + [self.rotation_weight] * 3
        )
        hessian = jacobian.T @ weights @ jacobian
        weighted_jacobian = np.sqrt(weights) @ jacobian
        _, singular_values, right_singular_vectors = np.linalg.svd(
            weighted_jacobian, full_matrices=False
        )
        self.last_min_singular_value = float(singular_values[-1])
        adaptive_damping = np.zeros_like(singular_values)
        if self.singularity_threshold > 0.0:
            singularity_ratio = np.clip(
                1.0
                - singular_values
                / self.singularity_threshold,
                0.0,
                1.0,
            )
            adaptive_damping = (
                self.singularity_damping * singularity_ratio ** 2
            )
        self.last_adaptive_damping = float(
            np.max(adaptive_damping, initial=0.0)
        )
        # 只沿退化的右奇异向量增加阻尼；其它可操作方向保持原响应。
        adaptive_regularization = (
            right_singular_vectors.T
            @ np.diag(adaptive_damping)
            @ right_singular_vectors
        )
        hessian += (
            self.damping + self.smoothness
        ) * np.eye(self.count)
        hessian += adaptive_regularization
        gradient = -jacobian.T @ weights @ desired_twist
        gradient -= self.smoothness * self.previous_velocity

        # openarm_HT 是 7 自由度冗余臂。姿态任务只投影到主任务雅可比的
        # 零空间，避免为了收肘而牺牲末端位置和姿态跟踪。
        if self.posture_reference is not None and self.posture_weight > 0.0:
            posture_velocity = self.posture_gain * (
                self.posture_reference - joint
            )
            posture_velocity = np.clip(
                posture_velocity,
                -self.velocity_limit,
                self.velocity_limit,
            )
            jacobian_pinv = np.linalg.pinv(jacobian, rcond=1e-4)
            nullspace = np.eye(self.count) - jacobian_pinv @ jacobian
            posture_metric = np.diag(self.posture_joint_weights)
            projected_metric = (
                nullspace.T @ posture_metric @ nullspace
            )
            hessian += self.posture_weight * projected_metric
            gradient -= (
                self.posture_weight
                * projected_metric
                @ posture_velocity
            )

        lower, upper = self._velocity_bounds(joint, dt)
        solver = osqp.OSQP()
        solver.setup(
            P=csc_matrix((hessian + hessian.T) * 0.5),
            q=gradient,
            A=eye(self.count, format="csc"),
            l=lower,
            u=upper,
            verbose=False,
            polishing=False,
            warm_starting=True,
            max_iter=400,
            eps_abs=1e-4,
            eps_rel=1e-4,
            check_termination=25,
        )
        solver.warm_start(x=self.previous_velocity)
        solution = solver.solve(raise_error=False)
        if solution.x is None or solution.info.status_val not in (1, 2):
            raise RuntimeError(f"OSQP IK 失败: {solution.info.status}")

        velocity = np.clip(
            np.asarray(solution.x, dtype=float), lower, upper
        )
        next_joint = joint + velocity * dt
        if self.lower is not None:
            next_joint = np.clip(
                next_joint,
                self.lower + self.joint_limit_margin,
                self.upper - self.joint_limit_margin,
            )
        self.previous_velocity = velocity.copy()

        achieved = np.asarray(
            self.robot.forward_kinematics(next_joint)["transform"],
            dtype=float,
        )
        position_error = float(
            np.linalg.norm(target[:3, 3] - achieved[:3, 3])
        )
        rotation_error = float(
            Rotation.from_matrix(
                target[:3, :3] @ achieved[:3, :3].T
            ).magnitude()
        )
        return QPIKResult(
            joint_position=next_joint,
            joint_velocity=velocity,
            position_error=position_error,
            rotation_error=rotation_error,
            status=solution.info.status,
            iterations=int(solution.info.iter),
        )
