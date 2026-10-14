"""Adapt the unchanged Panthera panel contracts to the OpenArm controller."""
import time
import numpy as np
from flask import jsonify, request
from scipy.spatial.transform import Rotation
from model import SIDES, number, vector


def register_original_ui(app, c):
    def sides():
        return c.active or list(SIDES)

    def configuration():
        joints = []
        selected = sides()
        for side in selected:
            meta = c.model.metadata[side]
            for i, name in enumerate(meta['joint_names']):
                joints.append(dict(name=name, index=len(joints), min=meta['joint_limits']['lower'][i],
                                   max=meta['joint_limits']['upper'][i], sign=meta['joint_signs'][i], kind='joint', side=side))
        for side in selected:
            lim = c.model.metadata[side]['gripper_limits']
            joints.append(dict(name=side+'_gripper', index=len(joints), min=lim['lower'], max=lim['upper'], kind='gripper', side=side))
        return dict(robot_name='OpenArm HT A1 · ' + ' + '.join(selected), joints=joints,
                    demo_mode=not c.live, control_mode=mode(), end_effector_offset=0,
                    end_effector_link=c.model.metadata[selected[0]]['end_link'], arms=selected)

    def mode():
        return 'gravity_comp' if c.mode == 'gravity' else c.mode

    def flat(values):
        return [float(v) for s in sides() for v in values[s]]

    def waypoints():
        return [dict(positions=flat(p['positions']), duration=p['duration'], index=i)
                for i,p in enumerate(c.waypoints)]

    def split(values):
        selected = sides()
        if not isinstance(values, list) or len(values) != len(selected)*7:
            raise ValueError(f'需要 {len(selected)*7} 个关节目标')
        # Validate all arms before changing either arm.
        return {s:c.positions(s,values[i*7:(i+1)*7]) for i,s in enumerate(selected)}

    def move(data):
        c.require_control()
        if c.trajectory:
            raise ValueError('请先停止轨迹')
        targets = split(data['positions']) if 'positions' in data else {s:c.target[s].copy() for s in sides()}
        speed = number(data.get('velocity',c.speed), .02, min(1., *(min(c.model.metadata[s]['velocity_limits']) for s in sides())), '速度')
        grips = data.get('grippers', {})
        if 'gripper' in data and not grips:
            grips = {sides()[0]+'_gripper':data['gripper']}
        validated_grips = {s:number(grips.get(s+'_gripper',c.grip_target[s]), **c.grip_bounds(s)) for s in sides()}
        for s in sides():
            c.target[s] = targets[s]
            c.grip_target[s] = validated_grips[s]
        c.speed = speed
        c.log('网页关节目标已提交')

    @app.get('/api/arm_description_files')
    def description_files():
        root = c.model.path.parent.parent
        files = {str(p.relative_to(root)):'/model/'+str(p.relative_to(root))
                 for p in root.rglob('*') if p.is_file() and p.suffix.lower() in ('.urdf','.stl','.dae','.obj')}
        return jsonify(success=True,base_url='/model',files=files)

    @app.get('/api/legacy/config')
    def legacy_config():
        with c.lock:
            return jsonify(configuration())

    @app.get('/api/legacy/state')
    def legacy_state():
        with c.lock:
            snapshot = c.snapshot()
            fk = snapshot['fk'][sides()[0]]
            euler = Rotation.from_matrix(fk['rotation']).as_euler('xyz',degrees=True).tolist()
            wrench = None
            wrench_error = ''
            if c.connected and not c.fault and c.mode == 'impedance' and c.live:
                try:
                    wrench = c.model.estimate_wrench(c.q, c.torque, sides()[0])
                except Exception as error:
                    wrench_error = str(error)
            return jsonify(external_wrench=wrench, wrench_valid=wrench is not None,
                           wrench_source='joint_torque_estimate' if c.live else 'unavailable_in_demo',
                           wrench_frame='base_link', wrench_side=sides()[0], wrench_error=wrench_error,
                           positions=flat(c.q), velocities=flat(c.v), torques=flat(c.torque),
                           target_positions=flat(c.target), impedance_target=flat(c.target),
                           gripper_positions={s+'_gripper':c.grip[s] for s in sides()},
                           control_mode=mode(), ee_position=fk['position'], ee_euler=euler,
                           forward_kinematics=dict(position=fk['position'],euler=euler),
                           timestamp=snapshot['time'], waypoints=waypoints(),
                           trajectory_running=snapshot['trajectory_running'], trajectory_progress=c.progress,
                           connected=c.connected,enabled=c.enabled,fault=c.fault,
                           script_running=c.script_active, demo_mode=not c.live)

    @app.post('/api/legacy/<event>')
    def command(event):
        data = request.get_json(silent=True) or {}
        with c.lock:
            if event == 'move_all':
                move(data)
            elif event in ('move_joint','set_impedance_target'):
                if 'target' in data:
                    move(dict(positions=data['target']))
                else:
                    positions = flat(c.target)
                    index = int(data['joint'])
                    if not 0 <= index < len(positions):
                        raise ValueError('关节编号无效')
                    positions[index]=data['position']
                    move(dict(positions=positions))
            elif event == 'set_mode':
                c.set_mode(dict(mode='gravity' if data.get('mode')=='gravity_comp' else data.get('mode')))
            elif event == 'set_impedance_params':
                c.set_mode(dict(mode=c.mode, kp=data.get('K',c.kp.tolist()),kd=data.get('B',c.kd.tolist())))
            elif event in ('home','reset_all'):
                move(dict(positions=[0.]*(len(sides())*7),velocity=.25))
            elif event == 'add_waypoint':
                if c.trajectory or len(c.waypoints)>=6:
                    raise ValueError('轨迹执行中或路点已达到 6 个')
                positions = {s:c.q[s].tolist() for s in SIDES}
                positions.update({s:q.tolist() for s,q in split(data.get('positions')).items()})
                point = dict(name=f'Point {len(c.waypoints)+1}',positions=positions,grippers=c.grip.copy(),duration=data.get('duration',1))
                c.waypoints=c.validate_waypoints([*c.waypoints,point])
            elif event == 'clear_waypoints':
                if c.trajectory:
                    raise ValueError('执行中不能清空路点')
                c.waypoints=[]
            elif event == 'update_waypoint_duration':
                if c.trajectory:
                    raise ValueError('执行中不能修改路点')
                index=int(data['index'])
                if not 0<=index<len(c.waypoints):
                    raise ValueError('路点编号无效')
                c.waypoints[index]['duration']=number(data['duration'],.5,120,'段时长')
            elif event == 'run_trajectory':
                c.run_trajectory()
            elif event == 'go_to_waypoint':
                index=int(data.get('index',0))
                if not 0<=index<len(c.waypoints):
                    raise ValueError('路点编号无效')
                point=c.waypoints[index]
                if c.mode in ('gravity','gravity_friction'):
                    c.set_mode(dict(mode='position'))
                move(dict(positions=flat(point['positions']),grippers={s+'_gripper':point['grippers'][s] for s in sides()}))
            elif event == 'stop_trajectory':
                c.stop()
            else:
                raise ValueError('不支持的操作: '+event)
            return jsonify(success=True,mode=mode(),waypoints=waypoints())
