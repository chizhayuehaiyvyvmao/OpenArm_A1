#!/usr/bin/env python3
"""Offline-only validation for the OpenarmV2 URDF adaptation."""
import os
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
from Lib import Openarm  # noqa: E402


def make_offline_arm(config_name):
    config_path = REPO / "robot_param" / config_name
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    arm = Openarm.__new__(Openarm)
    arm.config = config
    arm.config_dir = str(config_path.parent)
    arm.motor_count = 7
    arm.joint_limits = {
        "lower": np.asarray(config["robot"]["joint_limits"]["lower"], dtype=float),
        "upper": np.asarray(config["robot"]["joint_limits"]["upper"], dtype=float),
    }
    arm.model = None
    arm.data = None
    arm.joint_ids = []
    arm.end_effector_frame_id = None
    arm.joint_signs = None
    arm._active_q_indices = []
    arm._active_v_indices = []
    arm._load_urdf_model()
    return arm


def main():
    left = make_offline_arm("Openarm_left.yaml")
    right = make_offline_arm("Openarm_right.yaml")
    assert left.model.nq == left.model.nv == 14
    assert right.model.nq == right.model.nv == 14
    expected_signs = np.array([1, -1, 1, 1, 1, -1, 1], dtype=float)
    np.testing.assert_array_equal(left.joint_signs, expected_signs)
    np.testing.assert_array_equal(right.joint_signs, expected_signs)

    q_left = np.array([-0.35, -0.2, 0.25, -0.4, 0.2, 0.1, -0.15])
    q_right = -q_left
    fk_left = left.forward_kinematics(q_left)
    fk_right = right.forward_kinematics(q_right)
    reflection = np.diag([1.0, -1.0, 1.0])
    np.testing.assert_allclose(
        fk_right["position"], reflection @ np.asarray(fk_left["position"]), atol=1e-9
    )
    np.testing.assert_allclose(
        fk_right["rotation"], reflection @ fk_left["rotation"] @ reflection, atol=1e-9
    )

    for arm, q in ((left, q_left), (right, q_right)):
        v = np.linspace(-0.1, 0.1, 7)
        a = np.linspace(0.05, -0.05, 7)
        assert arm.get_jacobian(q).shape == (6, 7)
        assert arm.get_Gravity(q).shape == (7,)
        assert arm.get_Coriolis(q, v).shape == (7, 7)
        assert arm.get_Coriolis_vector(q, v).shape == (7,)
        mass = arm.get_Mass_Matrix(q)
        assert mass.shape == (7, 7)
        np.testing.assert_allclose(mass, mass.T, atol=1e-10)
        assert arm.get_Inertia_Terms(q, a).shape == (7,)
        assert arm.get_Dynamics(q, v, a).shape == (7,)
        assert np.all(np.isfinite(mass))

    print("OPENARM_V2_OFFLINE_MODEL_OK")


if __name__ == "__main__":
    main()
