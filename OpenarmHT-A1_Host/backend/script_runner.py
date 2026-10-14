"""Run curated SDK examples with an RPC motor transport, never a second serial owner.

The original SDK retains its FK, IK and interpolation. Only hightorque_robot's
transport is replaced in this child process. Unsupported native calls fail closed.
"""
import json
import os
from pathlib import Path
import runpy
import signal
import sys
import types
import urllib.request
import urllib.error

ROOT = Path(os.environ.get('OPENARM_SDK_ROOT', Path(__file__).resolve().parents[2] / 'OpenarmHT-A1_SDK（NEW）')).resolve()
URL = os.environ['OPENARM_RPC_URL']
TOKEN = os.environ['OPENARM_SCRIPT_TOKEN']


def rpc(data):
    req = urllib.request.Request(URL + '/api/script-rpc', data=json.dumps(data).encode(), headers={'Content-Type': 'application/json', 'X-Script-Token': TOKEN})
    try:
        with urllib.request.urlopen(req, timeout=2) as response:
            value = json.load(response)
    except urllib.error.HTTPError as error:
        # Stop revokes the token before terminating the child. A finally-block
        # stop is then already satisfied; stale motion still fails closed.
        if error.code == 403:
            if data.get('stop'):
                return {'success': True}
            raise KeyboardInterrupt('网页控制已停止') from None
        detail = error.read().decode()
        try:
            detail = json.loads(detail).get('error', detail)
        except (ValueError, AttributeError):
            pass
        raise RuntimeError(detail) from error
    if not value.get('success'):
        raise RuntimeError(value.get('error', 'RPC failed'))
    return value


class Motor:
    def __init__(self, robot, index):
        self.robot, self.index = robot, index

    def get_motor_id(self):
        return self.index + 1

    def get_motor_name(self):
        return f'joint{self.index+1}'

    def get_motor_enum_type(self):
        return 'WEB_TRANSPORT'

    def get_current_motor_state(self):
        return types.SimpleNamespace(**self.robot.states[self.index])

    def pos_vel_MAXtqe(self, position, velocity, torque_max):
        self.robot.pending[str(self.index)] = ['pos_vel_MAXtqe', float(position), float(velocity), float(torque_max)]

    def pos_vel_tqe_kp_kd(self, position, velocity, torque, kp, kd):
        self.robot.pending[str(self.index)] = ['pos_vel_tqe_kp_kd', *map(float, (position, velocity, torque, kp, kd))]


class Robot:
    def __init__(self, config_path):
        name = Path(config_path).name
        if name not in ('Openarm_left.yaml', 'Openarm_right.yaml'):
            raise ValueError('网页仅支持当前 OpenArm 左右臂配置')
        self.side = 'left' if name == 'Openarm_left.yaml' else 'right'
        self.pending = {}
        self.states = rpc({'side': self.side, 'commands': {}})['states']
        self._motors = [Motor(self, i) for i in range(8)]

    def get_motors(self):
        return self._motors

    def send_get_motor_state_cmd(self):
        pass

    def motor_send_cmd(self):
        self.states = rpc({'side': self.side, 'commands': self.pending})['states']
        self.pending = {}

    def set_stop(self):
        self.pending = {}
        rpc({'side': self.side, 'stop': True})


fake = types.ModuleType('hightorque_robot')
fake.Robot = Robot
sys.modules['hightorque_robot'] = fake
sys.path.insert(0, str(ROOT / 'scripts'))
signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
os.chdir(ROOT / 'scripts')
runpy.run_path(str(ROOT / 'scripts' / sys.argv[1]), run_name='__main__')
