"""Read-only model metadata and FK; importing this module never imports the motor SDK."""
from pathlib import Path
import os
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

HOST_ROOT = Path(__file__).resolve().parents[1]
ROOT = Path(os.environ.get('OPENARM_SDK_ROOT', HOST_ROOT.parent / 'OpenarmHT-A1_SDK（NEW）')).resolve()
SIDES = ('left', 'right')


class Model:
    def __init__(self):
        self.config = {s: yaml.safe_load((ROOT / 'robot_param' / f'Openarm_{s}.yaml').read_text()) for s in SIDES}
        paths = [(ROOT / 'robot_param' / c['urdf']['file_path']).resolve() for c in self.config.values()]
        if paths[0] != paths[1]:
            raise ValueError('双臂必须使用同一 URDF')
        self.path = paths[0]
        self.xml = ET.parse(self.path).getroot()
        self.joints = list(self.xml.findall('joint'))
        self.metadata = {}
        for side, c in self.config.items():
            names = c['kinematics']['joint_names']
            signs = c['kinematics']['joint_signs']
            assert len(names) == len(signs) == 7
            for i, name in enumerate(names):
                joint = self.xml.find(f"joint[@name='{name}']")
                assert joint is not None and joint.get('type') == 'revolute'
                assert signs[i] in (-1, 1)
                lo, hi = sorted([c['robot']['joint_limits'][k][i] * signs[i] for k in ('lower', 'upper')])
                limit = joint.find('limit')
                assert lo >= float(limit.get('lower')) - 1e-4 and hi <= float(limit.get('upper')) + 1e-4
            self.metadata[side] = dict(c['robot'], joint_names=names, joint_signs=signs, end_link=c['urdf']['end_effector_link'])

    def fk(self, positions):
        angles = {n: q * sign for s in SIDES for n, q, sign in zip(self.metadata[s]['joint_names'], positions[s], self.metadata[s]['joint_signs'])}
        frames = {'base_link': np.eye(4)}
        pending = self.joints[:]
        while pending:
            progressed = False
            for j in pending[:]:
                parent = j.find('parent').get('link')
                if parent not in frames:
                    continue
                origin = j.find('origin')
                xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
                rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
                transform = np.eye(4)
                transform[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
                transform[:3, 3] = xyz
                if j.get('name') in angles:
                    axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
                    transform[:3, :3] = transform[:3, :3] @ Rotation.from_rotvec(axis * angles[j.get('name')]).as_matrix()
                frames[j.find('child').get('link')] = frames[parent] @ transform
                pending.remove(j)
                progressed = True
            if not progressed:
                raise ValueError('URDF 树不连通')
        return {s: {'position': frames[self.metadata[s]['end_link']][:3, 3].tolist(),
                    'rotation': frames[self.metadata[s]['end_link']][:3, :3].tolist()} for s in SIDES}


    def dynamics_at(self, positions, side):
        """Motor-coordinate gravity and world-aligned 6x7 end-frame Jacobian.

        Own Pinocchio data, independent of mutable SDK control data. No serial IO.
        """
        import pinocchio as pin
        if not hasattr(self, '_pin_model'):
            self._pin_model = pin.buildModelFromUrdf(str(self.path))
            self._pin_data = self._pin_model.createData()
        model, data = self._pin_model, self._pin_data
        q = pin.neutral(model)
        for arm in SIDES:
            meta = self.metadata[arm]
            for name, angle, sign in zip(meta['joint_names'], positions[arm], meta['joint_signs']):
                q[model.joints[model.getJointId(name)].idx_q] = angle * sign
        meta = self.metadata[side]
        indices = [model.joints[model.getJointId(name)].idx_v for name in meta['joint_names']]
        signs = np.asarray(meta['joint_signs'])
        gravity = pin.computeGeneralizedGravity(model, data, q)[indices] * signs
        frame = model.getFrameId(meta['end_link'], pin.BODY)
        jacobian = pin.computeFrameJacobian(model, data, q, frame, pin.LOCAL_WORLD_ALIGNED)
        return gravity, jacobian[:, indices] * signs[None, :]

    def estimate_wrench(self, positions, torques, side):
        """Quasi-static applied wrench: J.T F = gravity - measured motor torque.

        Friction, acceleration and model errors remain in the estimate.
        Display only; never used for control. This is not a force sensor.
        """
        gravity, jacobian = self.dynamics_at(positions, side)
        residual = gravity - np.asarray(torques[side])
        wrench = np.linalg.solve(jacobian @ jacobian.T + .05**2 * np.eye(6), jacobian @ residual)
        if not np.isfinite(wrench).all():
            raise ValueError('末端力估算结果无效')
        return wrench.tolist()


def number(value, low, high, label='数值'):
    if isinstance(value, bool):
        raise ValueError(f'{label}必须是有限数字')
    value = float(value)
    if not np.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{label}必须在 [{low:g}, {high:g}] 内')
    return value


def vector(value, lower, upper, label='关节'):
    if not isinstance(value, list) or len(value) != len(lower):
        raise ValueError(f'{label}需要 {len(lower)} 个数值')
    return np.array([number(x, lo, hi, f'{label}{i+1}') for i, (x, lo, hi) in enumerate(zip(value, lower, upper))])
