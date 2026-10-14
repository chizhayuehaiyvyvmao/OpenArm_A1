import argparse
import atexit
from collections import deque
import json
import logging
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlparse
from flask import Flask, jsonify, request, send_from_directory
from werkzeug.exceptions import HTTPException
from controller import Controller
from compat import register_original_ui
from model import ROOT, HOST_ROOT, SIDES

SUPPORTED = {
    '0_dual_openarm_HT_get_state.py',
    '1_dual_Joint_PosVel_control.py',
    '1_dual_moveJ_control.py',
    '2_dual_gravity_compensation_control.py',
    '2_dual_gravity_friction_compensation_control.py',
    '2_dual_Jointimpendence_control_with_gra_pd.py',
    '3_dual_sin_trajectory_control.py',
    '3_sin_trajectory_control_with_plot.py',
    '1_dual_Joint_PD_control.py',
    '1_dual_forward_kinematics_test.py',
    '1_dual_inverse_kinematics_test.py',
    '2_dual_Jointimpendence_control_with_gra_fri_pd.py',
    '3_gravity_compensation_with_fk.py',
}


SCRIPT_LIMITATIONS = {
    '3_interpolation_control_zeroVel.py': '该原例程默认轨迹超过网页 1 rad/s 速度限制，尚未适配；需调整轨迹段时长后重新验证。',
    '3_interpolation_control_nozeroVel.py': '该原例程默认轨迹超过网页 1 rad/s 速度限制，尚未适配；需调整轨迹段时长及端点速度后重新验证。',
}


class Scripts:
    def __init__(self, control):
        self.control = control
        self.process = None
        self.name = None
        self.token = None
        self.output = deque(maxlen=600)
        self.cancel_event = threading.Event()

    def start(self, name, choice, url):
        c = self.control
        c.require_control()
        if c.trajectory:
            raise ValueError('请先停止轨迹')
        if name not in SUPPORTED:
            raise ValueError('该脚本尚未适配网页传输；可查看源码并在终端使用')
        wanted = list(SIDES) if choice == 'both' else [choice]
        if choice not in ('left', 'right', 'both') or any(s not in c.active for s in wanted):
            raise ValueError('所选机械臂未连接')
        if self.process is not None and self.process.poll() is None:
            raise ValueError('已有脚本运行')
        self.cancel_event = threading.Event()
        self.token = secrets.token_urlsafe(32)
        env = dict(os.environ, OPENARM_SCRIPT_TOKEN=self.token, OPENARM_RPC_URL=url, PYTHONUNBUFFERED='1', MPLBACKEND='Agg')
        env.pop('LD_PRELOAD', None)
        self.output.clear()
        self.name = name
        c.raw = {s: {} for s in SIDES}
        c.script_sides = wanted
        c.script_active = True
        try:
            self.process = subprocess.Popen([sys.executable, '-u', str(Path(__file__).with_name('script_runner.py')), name], cwd=ROOT/'scripts', env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
            self.process.stdin.write({'left':'1', 'right':'2', 'both':'3'}[choice] + '\n')
            self.process.stdin.flush()
        except Exception:
            c.script_active = False
            self.token = None
            raise
        c.log('运行 SDK 脚本: ' + name)
        threading.Thread(target=self.watch, args=(self.process, self.cancel_event, tuple(wanted)), daemon=True).start()

    def return_to_zero(self, process, cancel_event, sides):
        """Home selected arm joints after success, with feedback and cancellable ownership.

        Never re-enable here: Stop, heartbeat loss, or faults cancel this path.
        Gripper commands are not added and non-selected arms receive no script commands.
        """
        c = self.control
        with c.lock:
            if self.process is not process or cancel_event.is_set():
                return False
            c.require_control(script=True)
            if not c.script_active or time.monotonic() - c.heartbeat > 2:
                return False
            speed = .25
            for side in sides:
                c.positions(side, [0.] * 7)
                c.raw[side] = {}
            distance = max(abs(float(q)) for side in sides for q in c.q[side])
            deadline = time.monotonic() + distance / speed + 5.
            self.output.append('[正常结束] 全部所选臂关节回零中；Stop 可立即中断。')
            c.log('例程正常结束 · 所选臂全部关节回零')
        while not cancel_event.is_set():
            with c.lock:
                if self.process is not process or cancel_event.is_set():
                    return False
                if not c.enabled or not c.connected or c.fault or not c.script_active or time.monotonic() - c.heartbeat > 2:
                    return False
                if all(max(abs(float(q)) for q in c.q[side]) <= .02 for side in sides):
                    self.output.append('[回零完成] 已确认全部所选臂关节到位。')
                    return True
                if time.monotonic() > deadline:
                    raise ValueError('回零到位超时')
                for side in sides:
                    meta = c.model.metadata[side]
                    c.script_commands(side, {str(i): ['pos_vel_MAXtqe', 0., min(speed, meta['velocity_limits'][i]), meta['max_torque'][i]] for i in range(7)})
            cancel_event.wait(.02)
        return False

    def watch(self, process, cancel_event, sides):
        failed = False
        for line in process.stdout:
            # These SDK examples catch exceptions and can still exit with code 0.
            if any(marker in line for marker in ('错误:', '错误：', '发生错误', 'Traceback', '程序被中断')):
                failed = True
            with self.control.lock:
                if self.process is process:
                    self.output.append(line.rstrip()[:2000])
        code = process.wait()
        with self.control.lock:
            if self.process is not process:
                return
            normal = code == 0 and not failed and not cancel_event.is_set() and self.token is not None
            self.token = None
        if normal:
            try:
                if not self.return_to_zero(process, cancel_event, sides):
                    with self.control.lock:
                        if self.process is process and not cancel_event.is_set():
                            self.output.append('[回零取消] 控制已失效；保持停止。')
            except Exception as error:
                with self.control.lock:
                    if self.process is process:
                        self.output.append('[回零失败] ' + str(error))
        with self.control.lock:
            if self.process is process:
                self.control.script_active = False
                self.control.stop()
                self.output.append('[已停止] 用户 Stop，不回零。' if cancel_event.is_set() else f'[脚本退出] code={code}')
                self.control.log('例程已停止 · 不再下发动作')

    def stop(self):
        # Revoke RPC before terminating, so a late callback cannot move a motor.
        with self.control.lock:
            self.cancel_event.set()
            self.token = None
            self.control.script_active = False
            self.control.stop()
            process = self.process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)


def create_app(live=False, start_loop=True, port=5001):
    app = Flask(__name__, static_folder=str(HOST_ROOT/'frontend/dist'), static_url_path='')
    app.config['MAX_CONTENT_LENGTH'] = 1024*1024
    c = Controller(live)
    scripts = Scripts(c)
    csrf = secrets.token_urlsafe(32)
    app.extensions['controller'] = c
    app.extensions['scripts'] = scripts

    @app.before_request
    def guard():
        # Local service; reject cross-site requests and require the bootstrap token.
        if request.host.split(':')[0] not in ('localhost', '127.0.0.1'):
            return jsonify(error='仅允许本机访问'), 403
        if request.method == 'POST' and request.path != '/api/script-rpc':
            origin = request.headers.get('Origin')
            if origin and urlparse(origin).hostname not in ('localhost', '127.0.0.1'):
                return jsonify(error='禁止跨站控制'), 403
            if not secrets.compare_digest(request.headers.get('X-Control-Token', ''), csrf):
                return jsonify(error='请刷新页面以获取控制会话'), 403

    @app.errorhandler(Exception)
    def error(exc):
        if isinstance(exc, HTTPException):
            return jsonify(success=False, error=exc.description), exc.code
        if isinstance(exc, (ValueError, KeyError, TypeError, OverflowError)):
            return jsonify(success=False, error=str(exc)), 400
        app.logger.exception('Request failed')
        return jsonify(success=False, error=str(exc)), 500

    @app.get('/')
    def index():
        return send_from_directory(app.static_folder, 'index.html')

    @app.get('/api/config')
    def config():
        return jsonify(name='OpenArm HT · A1', arms=c.model.metadata, live=live, token=csrf,
                       urdf='/model/urdf/' + c.model.path.name, version='1.0.0')

    @app.get('/model/<path:name>')
    def model_files(name):
        return send_from_directory(c.model.path.parent.parent, name)

    @app.get('/api/state')
    def state():
        with c.lock:
            return jsonify(c.snapshot())

    @app.post('/api/heartbeat')
    def heartbeat():
        with c.lock:
            c.heartbeat = time.monotonic()
        return jsonify(success=True)

    @app.post('/api/<action>')
    def action(action):
        data = request.get_json(silent=True) or {}
        if action in ('stop', 'disconnect'):
            scripts.stop()
            if action == 'disconnect':
                with c.lock:
                    c.disconnect()
            return jsonify(success=True)
        with c.lock:
            if action == 'connect':
                if live and data.get('confirm') != 'CONNECT':
                    raise ValueError('连接真机需要确认 CONNECT')
                c.connect(data.get('side', 'both'))
            elif action == 'enable':
                if live and data.get('confirm') != 'ENABLE':
                    raise ValueError('启用真机需要确认 ENABLE')
                c.enable()
            elif action == 'move':
                c.move(data)
            elif action == 'mode':
                c.set_mode(data)
            elif action == 'waypoints':
                if c.trajectory:
                    raise ValueError('执行中不能修改路点')
                c.waypoints = c.validate_waypoints(data.get('waypoints'))
            elif action == 'trajectory':
                c.run_trajectory(data.get('speed'))
            else:
                return jsonify(error='未知接口'), 404
        return jsonify(success=True)

    @app.get('/api/waypoints')
    def waypoints():
        with c.lock:
            return jsonify(waypoints=c.waypoints)

    register_original_ui(app, c)

    @app.get('/api/scripts')
    def list_scripts():
        files = sorted((ROOT/'scripts').glob('*.py'))
        return jsonify(scripts=[{'name': p.name, 'file':p.name, 'supported':p.name in SUPPORTED, 'reason': '' if p.name in SUPPORTED else SCRIPT_LIMITATIONS.get(p.name, '该例程需要终端或额外设备，尚未适配网页执行；下方可查看源码。')} for p in files],
                       running=c.script_active, current=scripts.name, current_script=scripts.name if c.script_active else None, scripts_dir=str(ROOT/'scripts'), output=list(scripts.output))

    @app.get('/api/scripts/log')
    def script_log():
        return app.response_class('\n'.join(scripts.output), mimetype='text/plain')

    @app.get('/api/scripts/source')
    def source():
        name = request.args.get('name', '')
        if name not in {p.name for p in (ROOT/'scripts').glob('*.py')}:
            return jsonify(error='脚本不存在'), 404
        return jsonify(source=(ROOT/'scripts'/name).read_text())

    @app.post('/api/scripts/run')
    def run_script():
        data = request.get_json() or {}
        if live and data.get('confirm') != 'RUN':
            raise ValueError('运行真机脚本需要确认 RUN')
        with c.lock:
            scripts.start(data.get('name'), data.get('side', 'both'), f'http://127.0.0.1:{port}')
        return jsonify(success=True)

    @app.post('/api/scripts/stop')
    def stop_script():
        scripts.stop()
        return jsonify(success=True)

    @app.post('/api/scripts/input')
    def script_input():
        data = request.get_json() or {}
        with c.lock:
            if not c.script_active or scripts.process.poll() is not None:
                raise ValueError('没有运行中的脚本')
            text = str(data.get('text', ''))
            if len(text) > 200:
                raise ValueError('输入过长')
            scripts.process.stdin.write(text + '\n')
            scripts.process.stdin.flush()
        return jsonify(success=True)

    @app.post('/api/script-rpc')
    def script_rpc():
        with c.lock:
            if not scripts.token or not secrets.compare_digest(request.headers.get('X-Script-Token',''), scripts.token):
                return jsonify(error='脚本控制权已撤销'), 403
            data = request.get_json() or {}
            side = data.get('side')
            if side not in c.active or side not in c.script_sides:
                raise ValueError('机械臂未连接')
            if data.get('stop'):
                c.raw[side] = {}
                if live:
                    c.robots[side].set_stop()
            else:
                c.script_commands(side, data.get('commands', {}))
            states = [dict(position=float(c.q[side][i]), velocity=float(c.v[side][i]), torque=float(c.torque[side][i]), fault=0, time=time.time(), ID=i+1, mode=0) for i in range(7)]
            states.append(dict(position=c.grip[side], velocity=c.grip_velocity[side], torque=c.grip_torque[side], fault=0, time=time.time(), ID=8, mode=0))
            return jsonify(success=True, states=states)

    def cleanup():
        scripts.stop()
        c.running = False

    app.extensions['cleanup'] = cleanup
    if start_loop:
        threading.Thread(target=c.loop, daemon=True).start()
        atexit.register(cleanup)
    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='OpenArm 网页上位机，默认 Demo，不连接硬件')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--demo', action='store_true')
    group.add_argument('--live', action='store_true')
    parser.add_argument('--port', type=int, default=5001)
    args = parser.parse_args()
    app = create_app(live=args.live, port=args.port)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    logging.getLogger('werkzeug').setLevel(logging.WARNING)
    print(f'OpenArm {"LIVE（等待页面手动连接）" if args.live else "DEMO"}: http://localhost:{args.port}', flush=True)
    app.run(host='127.0.0.1', port=args.port, threaded=True, use_reloader=False)
