"""Single command owner. Demo has no native SDK imports or serial access."""
import glob
import sys
import time
import threading
from collections import deque
import numpy as np
from model import Model, ROOT, SIDES, number, vector


class Controller:
    def __init__(self, live=False):
        self.model = Model()
        self.live = live
        self.lock = threading.RLock()
        self.robots = {}
        self.enabled = False
        self.connected = not live
        self.fault = ''
        self.mode = 'position'
        self.heartbeat = time.monotonic()
        self.running = True
        self.speed = .25
        self.kp = np.array([15, 15, 10, 10, 5, 3, 5.])
        self.kd = np.array([.5, .8, .8, .2, .2, .1, .1])
        self.q = {s: np.zeros(7) for s in SIDES}
        self.v = {s: np.zeros(7) for s in SIDES}
        self.torque = {s: np.zeros(7) for s in SIDES}
        self.grip = {s: 0. for s in SIDES}
        self.grip_velocity = {s: 0. for s in SIDES}
        self.grip_torque = {s: 0. for s in SIDES}
        self.target = {s: self.q[s].copy() for s in SIDES}
        self.grip_target = self.grip.copy()
        self.active = list(SIDES) if not live else []
        self.waypoints = []
        self.trajectory = None
        self.progress = 0.
        self.script_active = False
        self.script_sides = list(SIDES)
        self.raw = {s: {} for s in SIDES}
        self.logs = deque(maxlen=200)
        self.feedback_times = {}
        self.feedback_seen = {}
        self.log('DEMO 就绪 · 运动学演示，不模拟真实动力学' if not live else 'LIVE 就绪 · 等待手动连接')

    def log(self, message):
        self.logs.append({'time': time.strftime('%H:%M:%S'), 'message': message})

    def positions(self, side, values):
        limits = self.model.metadata[side]['joint_limits']
        return vector(values, limits['lower'], limits['upper'])

    def require_control(self, script=False):
        if not self.connected or not self.enabled or self.fault:
            raise ValueError('请先连接并启用控制；故障后需重新连接')
        if self.script_active and not script:
            raise ValueError('脚本正在控制机械臂，请先停止脚本')

    def connect(self, selected):
        if selected not in ('left', 'right', 'both'):
            raise ValueError('机械臂选择无效')
        if (self.connected or self.robots) and self.live:
            raise ValueError('已经连接，请先断开再更换机械臂')
        self.active = list(SIDES) if selected == 'both' else [selected]
        if self.live:
            if not glob.glob('/dev/ttyACM*'):
                self.active = []
                raise ValueError('未发现 /dev/ttyACM*，未初始化 SDK')
            sys.path.insert(0, str(ROOT / 'scripts'))
            from Lib import Openarm
            try:
                for side in self.active:
                    robot = Openarm(str(ROOT / 'robot_param' / f'Openarm_{side}.yaml'))
                    self.robots[side] = robot
                    if robot.motor_count != 7 or len(robot.Motors) != 8:
                        raise ValueError(f'{side} 必须是 7 关节 + 1 夹爪')
                # Only query frames here. No calibration, homing, or motion commands.
                self.feedback_times.clear()
                self.feedback_seen.clear()
                for _ in range(10):
                    self.read_hardware()
                    time.sleep(.02)
            except BaseException:
                self.stop()
                self.robots.clear()
                self.active = []
                raise
        self.connected = True
        self.enabled = False
        self.fault = ''
        self.target = {s: self.q[s].copy() for s in SIDES}
        self.grip_target = self.grip.copy()
        self.log('已连接 ' + selected + ' · 仅读取状态')

    def read_hardware(self):
        for side, robot in self.robots.items():
            robot.send_get_motor_state_cmd()
            robot.motor_send_cmd()
            states = list(robot.get_current_state()) + [robot.get_current_state_gripper()]
            for i, state in enumerate(states):
                values = [state.position, state.velocity, state.torque]
                if not np.isfinite(values).all() or abs(state.position) >= 900 or int(state.fault):
                    raise ValueError(f'{side} M{i+1} 反馈无效或故障 {state.fault}')
                key = (side, i)
                if self.feedback_times.get(key) != state.time:
                    self.feedback_times[key] = state.time
                    self.feedback_seen[key] = time.monotonic()
                elif time.monotonic() - self.feedback_seen.get(key, 0) > .5:
                    raise ValueError(f'{side} M{i+1} 反馈时间戳超过 0.5 秒未更新')
            self.q[side] = np.array([x.position for x in states[:7]])
            self.positions(side, self.q[side].tolist())
            self.v[side] = np.array([x.velocity for x in states[:7]])
            self.torque[side] = np.array([x.torque for x in states[:7]])
            self.grip[side] = float(states[7].position)
            self.grip_velocity[side] = float(states[7].velocity)
            self.grip_torque[side] = float(states[7].torque)

    def enable(self):
        if not self.connected or self.fault:
            raise ValueError('尚未连接或存在故障')
        self.target = {s: self.q[s].copy() for s in SIDES}
        self.grip_target = self.grip.copy()
        self.mode = 'position'
        self.heartbeat = time.monotonic()
        self.enabled = True
        self.log('控制已启用 · 保持当前姿态')

    def stop(self):
        self.enabled = False
        self.trajectory = None
        self.raw = {s: {} for s in SIDES}
        self.target = {s: self.q[s].copy() for s in SIDES}
        self.grip_target = self.grip.copy()
        for side, robot in self.robots.items():
            try:
                robot.set_stop()
            except Exception as error:
                self.fault = f'{side} 软件停止失败: {error}'
        self.log('软件停止 · 已撤销控制，不回零')

    def disconnect(self):
        self.stop()
        self.robots.clear()
        self.connected = False
        self.active = []
        self.log('连接已断开')

    def move(self, data):
        self.require_control()
        if self.trajectory:
            raise ValueError('轨迹执行中，请先停止')
        side = data.get('side')
        if side not in self.active:
            raise ValueError('该臂未连接')
        target = self.positions(side, data.get('positions'))
        speed = number(data.get('speed', .25), .02, min(1., min(self.model.metadata[side]['velocity_limits'])), '速度')
        grip = number(data.get('gripper', self.grip_target[side]), **self.grip_bounds(side))
        self.target[side] = target
        self.speed = speed
        self.grip_target[side] = grip
        self.log(f'{side} 目标已提交')

    def grip_bounds(self, side):
        limits = self.model.metadata[side]['gripper_limits']
        return dict(low=limits['lower'], high=limits['upper'], label='夹爪')

    def set_mode(self, data):
        self.require_control()
        if self.trajectory:
            raise ValueError('请先停止轨迹')
        mode = data.get('mode')
        if mode not in ('position', 'gravity', 'gravity_friction', 'impedance'):
            raise ValueError('未知模式')
        kp = vector(data.get('kp', self.kp.tolist()), [0]*7, [100]*7, '刚度')
        kd = vector(data.get('kd', self.kd.tolist()), [0]*7, [10]*7, '阻尼')
        self.target = {s: self.q[s].copy() for s in SIDES}
        self.kp, self.kd, self.mode = kp, kd, mode
        self.log('控制模式: ' + mode)

    def validate_waypoints(self, items):
        if not isinstance(items, list) or len(items) > 100:
            raise ValueError('最多保存 100 个路点')
        result = []
        for i, item in enumerate(items):
            result.append({'name': str(item.get('name', f'路点 {i+1}'))[:60],
                           'duration': number(item.get('duration', 3), .5, 120, '段时长'),
                           'positions': {s: self.positions(s, item['positions'][s]).tolist() for s in SIDES},
                           'grippers': {s: number(item.get('grippers', {}).get(s, 0), **self.grip_bounds(s)) for s in SIDES}})
        return result

    def run_trajectory(self, speed=None):
        self.require_control()
        speed = self.speed if speed is None else number(speed, .02, 1., '速度')
        if self.trajectory:
            raise ValueError('轨迹正在执行')
        if not self.waypoints:
            raise ValueError('请先添加路点')
        points = self.validate_waypoints(self.waypoints)
        previous = {s: self.q[s].copy() for s in SIDES}
        for point in points:
            for s in self.active:
                delta = np.abs(np.asarray(point['positions'][s]) - previous[s])
                config = self.model.metadata[s]
                duration = point['duration']
                # Quintic smoothstep has max s'=1.875 and max |s''|=5.774.
                if np.any(1.875*delta/duration > np.minimum(config['velocity_limits'], speed) + 1e-9):
                    raise ValueError(f"{point['name']} 时长太短，请延长；超过所选速度限制")
                if np.any(5.774*delta/duration**2 > np.asarray(config['acceleration_limits'])):
                    raise ValueError(f"{point['name']} 超过加速度限制，请延长时长")
                previous[s] = np.array(point['positions'][s])
        self.mode = 'position'
        self.speed = speed
        self.trajectory = dict(points=points, index=0, start=time.monotonic(), origin={s: self.q[s].copy() for s in SIDES}, grip_origin=self.grip.copy())
        self.progress = 0.
        self.log(f'开始执行 {len(points)} 个双臂路点')

    def tick(self, dt):
        if self.live and self.connected:
            self.read_hardware()
        if not self.enabled:
            for s in SIDES:
                if not self.live:
                    self.v[s].fill(0)
                    self.grip_velocity[s] = 0.
            return
        if time.monotonic() - self.heartbeat > 2.:
            self.stop()
            self.log('页面心跳丢失，控制已撤销')
            return
        if self.trajectory:
            t = self.trajectory
            point = t['points'][t['index']]
            u = min(1., (time.monotonic() - t['start']) / point['duration'])
            blend = 10*u**3 - 15*u**4 + 6*u**5
            for s in self.active:
                self.target[s] = t['origin'][s] + blend*(np.array(point['positions'][s]) - t['origin'][s])
                self.grip_target[s] = t['grip_origin'][s] + blend*(point['grippers'][s] - t['grip_origin'][s])
            self.progress = (t['index'] + u) / len(t['points'])
            if u >= 1:
                # Do not report completion until observed feedback reaches the endpoint.
                reached = all(np.max(np.abs(self.q[s]-self.target[s])) < .03 and abs(self.grip[s]-self.grip_target[s]) < .05 for s in self.active)
                if not reached and time.monotonic()-t['start'] > point['duration']+5:
                    raise ValueError('轨迹到位超时')
                if reached:
                    t['index'] += 1
                    t['origin'] = {s: self.target[s].copy() for s in SIDES}
                    t['grip_origin'] = self.grip_target.copy()
                    t['start'] = time.monotonic()
                    if t['index'] == len(t['points']):
                        self.trajectory = None
                        self.log('轨迹到位完成')
        for s in self.active:
            if self.script_active and s in self.script_sides:
                if not self.live:
                    self.simulate_raw(s, dt)
                continue
            if self.live:
                robot = self.robots[s]
                if self.mode == 'position':
                    if not robot.Joint_Pos_Vel(self.target[s], [self.speed]*7):
                        raise ValueError('SDK 拒绝位置命令')
                else:
                    gravity = .8 * robot.get_Gravity()
                    if self.mode == 'gravity_friction':
                        gravity += robot.get_friction_compensation(self.v[s], np.array([.2,.2,.15,.15,0,.1,0]), np.array([.06,.06,.04,.04,0,.03,0]), .02)
                    kp, kd = (self.kp, self.kd) if self.mode == 'impedance' else (np.zeros(7), np.zeros(7))
                    robot.pos_vel_tqe_kp_kd(self.target[s], np.zeros(7), np.clip(gravity, -robot.max_torque, robot.max_torque), kp, kd)
                if self.mode in ('gravity', 'gravity_friction'):
                    robot.gripper_control_MIT(self.grip[s], 0, 0, 0, 0)
                else:
                    robot.gripper_control(self.grip_target[s], .3, .5)
            else:
                previous = self.q[s].copy()
                previous_grip = self.grip[s]
                if self.mode in ('position', 'impedance'):
                    self.q[s] += np.clip(self.target[s]-self.q[s], -self.speed*dt, self.speed*dt)
                    self.grip[s] += float(np.clip(self.grip_target[s]-self.grip[s], -.3*dt, .3*dt))
                self.v[s] = (self.q[s]-previous) / dt
                self.grip_velocity[s] = (self.grip[s]-previous_grip) / dt
                self.torque[s].fill(0)

    def script_commands(self, side, commands):
        self.require_control(script=True)
        if not self.script_active or side not in self.active or side not in self.script_sides:
            raise ValueError('脚本无控制权或机械臂未连接')
        validated = {}
        for key, cmd in commands.items():
            i = int(key)
            if not 0 <= i < 8 or not isinstance(cmd, list):
                raise ValueError('无效电机命令')
            mode, *args = cmd
            config = self.model.metadata[side]
            lo = config['joint_limits']['lower'][i] if i < 7 else config['gripper_limits']['lower']
            hi = config['joint_limits']['upper'][i] if i < 7 else config['gripper_limits']['upper']
            torque = config['max_torque'][i] if i < 7 else .5
            velocity_limit = min(1., config['velocity_limits'][i]) if i < 7 else 1.
            if mode == 'pos_vel_MAXtqe' and len(args) == 3:
                args = [number(args[0],lo,hi), number(args[1],0,velocity_limit), number(args[2],0,torque)]
            elif mode == 'pos_vel_tqe_kp_kd' and len(args) == 5:
                args = [number(args[0],lo,hi), number(args[1],-velocity_limit,velocity_limit), number(args[2],-torque,torque), number(args[3],0,100), number(args[4],0,10)]
            else:
                raise ValueError('网页脚本仅允许位置速度或 MIT 指令')
            validated[i] = [mode, *args]
        for i, cmd in validated.items():
            self.raw[side][i] = cmd
            if self.live:
                getattr(self.robots[side].Motors[i], cmd[0])(*cmd[1:])
        if self.live:
            self.robots[side].motor_send_cmd()

    def simulate_raw(self, side, dt):
        previous = self.q[side].copy()
        previous_grip = self.grip[side]
        for i, (mode, *args) in self.raw[side].items():
            position, velocity = args[:2]
            if mode == 'pos_vel_tqe_kp_kd' and args[3] == 0:
                continue
            speed = max(abs(velocity), .4) if mode == 'pos_vel_tqe_kp_kd' else abs(velocity)
            if i < 7:
                self.q[side][i] += np.clip(position-self.q[side][i], -speed*dt, speed*dt)
            else:
                self.grip[side] += float(np.clip(position-self.grip[side], -speed*dt, speed*dt))
        self.v[side] = (self.q[side]-previous)/dt
        self.grip_velocity[side] = (self.grip[side]-previous_grip)/dt

    def snapshot(self):
        return dict(live=self.live, connected=self.connected, enabled=self.enabled, active=self.active, mode=self.mode,
                    fault=self.fault, positions={s:self.q[s].tolist() for s in SIDES},
                    velocities={s:self.v[s].tolist() for s in SIDES}, torques={s:self.torque[s].tolist() for s in SIDES},
                    grippers=self.grip.copy(), target={s:self.target[s].tolist() for s in SIDES},
                    gripper_states={s:dict(position=self.grip[s], velocity=self.grip_velocity[s], torque=self.grip_torque[s]) for s in SIDES},
                    fk=self.model.fk(self.q), trajectory_running=self.trajectory is not None, progress=self.progress,
                    script_running=self.script_active, logs=list(self.logs), time=time.time())

    def loop(self):
        previous = time.monotonic()
        while self.running:
            start = time.monotonic()
            with self.lock:
                try:
                    self.tick(max(.001, min(.05, start-previous)))
                except Exception as error:
                    self.stop()
                    self.fault = str(error)
                    self.connected = False
                    self.log('故障: ' + self.fault)
            previous = start
            time.sleep(max(.001, .01-(time.monotonic()-start)))
