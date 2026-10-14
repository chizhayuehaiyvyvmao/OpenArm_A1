"""Offline contract and safety tests; no hardware SDK construction."""
import sys
from pathlib import Path
import time
from types import SimpleNamespace
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from app import create_app
from controller import Controller


class APIContract(unittest.TestCase):
    def setUp(self):
        self.app = create_app(start_loop=False)
        self.client = self.app.test_client()
        self.c = self.app.extensions['controller']
        self.headers = {'X-Control-Token': self.client.get('/api/config').json['token']}

    def post(self, path, data=None):
        return self.client.post('/api/' + path, json=data or {}, headers=self.headers)

    def test_demo_does_not_import_native(self):
        self.assertNotIn('hightorque_robot', sys.modules)
        self.assertFalse(self.c.enabled)
        self.assertEqual(self.client.get('/api/state').json['active'], ['left','right'])

    def test_auth_and_origin(self):
        self.assertEqual(self.client.post('/api/enable', json={}).status_code, 403)
        self.assertEqual(self.client.post('/api/enable', json={}, headers={**self.headers, 'Origin':'https://evil.example'}).status_code,403)
        self.assertEqual(self.client.get('/api/config', headers={'Host':'evil.example'}).status_code,403)

    def test_arm_dimensions_and_signs(self):
        for side, meta in self.c.model.metadata.items():
            self.assertEqual(len(meta['joint_names']),7)
            self.assertEqual(meta['joint_signs'],[1,-1,1,1,1,-1,1])
        self.assertEqual(len(self.c.model.joints),14)

    def test_enable_limits_and_stop(self):
        target = [-.2,-.2,.2,-.4,.2,.2,-.1]
        self.assertEqual(self.post('move',dict(side='left',positions=target)).status_code,400)
        self.assertEqual(self.post('enable').status_code,200)
        for bad in ([0]*6, [999]*7, [float('nan')]*7, [float('inf')]*7):
            self.assertEqual(self.post('move',dict(side='left',positions=bad)).status_code,400)
        self.assertEqual(self.post('move',dict(side='left',positions=target,speed=.5)).status_code,200)
        for _ in range(100):self.c.tick(.01)
        np.testing.assert_allclose(self.c.q['left'],target,atol=1e-6)
        self.assertEqual(self.post('stop').status_code,200)
        self.assertFalse(self.c.enabled)
        old = self.c.q['left'].copy()
        self.c.tick(.05)
        np.testing.assert_allclose(self.c.q['left'],old)

    def test_transaction_rejects_bad_gripper(self):
        self.post('enable')
        before=self.c.target['left'].copy()
        self.assertEqual(self.post('move',dict(side='left',positions=[.1,0,0,0,0,0,0],gripper=50)).status_code,400)
        np.testing.assert_array_equal(before,self.c.target['left'])

    def test_heartbeat_revokes_motion(self):
        self.post('enable')
        self.c.heartbeat = time.monotonic()-3
        self.c.tick(.01)
        self.assertFalse(self.c.enabled)

    def test_mode_switch_preserves_pose(self):
        self.c.q['left'][0]=.2
        self.post('enable')
        for mode in ('gravity','gravity_friction','impedance','position'):
            self.assertEqual(self.post('mode',dict(mode=mode)).status_code,200)
            self.assertEqual(self.c.target['left'][0],.2)

    def test_single_arm_routing(self):
        self.post('disconnect')
        self.post('connect',dict(side='right'))
        self.post('enable')
        self.assertEqual(self.post('move',dict(side='left',positions=[0]*7)).status_code,400)
        self.assertEqual(self.post('move',dict(side='right',positions=[0]*7)).status_code,200)

    def test_waypoint_speed_and_validation(self):
        self.post('enable')
        point=dict(name='P1',duration=.5,positions={'left':[1.,0,0,0,0,0,0], 'right':[0]*7})
        self.assertEqual(self.post('waypoints',dict(waypoints=[point])).status_code,200)
        self.assertEqual(self.post('trajectory').status_code,400)
        point['duration']=10
        self.post('waypoints',dict(waypoints=[point]))
        self.assertEqual(self.post('trajectory').status_code,200)
        self.assertEqual(self.post('move',dict(side='left',positions=[0]*7)).status_code,400)
        self.assertEqual(self.post('waypoints',dict(waypoints=[])).status_code,400)
        self.post('stop')
        self.assertIsNone(self.c.trajectory)

    def test_trajectory_feedback_completion(self):
        self.post('enable')
        self.post('waypoints',dict(waypoints=[dict(positions={s:[0]*7 for s in ('left','right')},duration=1)]))
        self.post('trajectory')
        self.c.trajectory['start']=time.monotonic()-1.5
        self.c.tick(.01)
        self.assertIsNone(self.c.trajectory)
        self.assertEqual(self.c.progress,1)

    def test_scripts_are_exclusive_and_paths_confined(self):
        self.assertEqual(self.client.get('/api/scripts/source?name=../../README.md').status_code,404)
        self.post('enable')
        self.assertEqual(self.post('scripts/run',dict(name='0_dual_robot_set_zero.py',side='both')).status_code,400)
        self.c.script_active=True
        self.assertEqual(self.post('move',dict(side='left',positions=[0]*7)).status_code,400)
        self.assertEqual(self.client.post('/api/script-rpc',json={'side':'left','commands':{}}).status_code,403)

    def test_raw_commands_validate_entire_batch(self):
        self.post('enable')
        self.c.script_active=True
        with self.assertRaises(ValueError):
            self.c.script_commands('left',{'0':['pos_vel_MAXtqe',.1,.2,10], '1':['pos_vel_MAXtqe',999,.2,10]})
        self.assertEqual(self.c.raw['left'],{})
        self.c.script_commands('left',{'0':['pos_vel_MAXtqe',.1,.2,10]})
        self.c.tick(.05)
        self.assertGreater(self.c.q['left'][0],0)
        self.post('stop')
        with self.assertRaises(ValueError):
            self.c.script_commands('left',{'0':['pos_vel_MAXtqe',.1,.2,10]})

    def test_live_start_is_passive(self):
        live = create_app(live=True,start_loop=False)
        client = live.test_client()
        headers={'X-Control-Token':client.get('/api/config').json['token']}
        self.assertFalse(client.get('/api/state').json['connected'])
        self.assertEqual(client.post('/api/connect',json={'side':'both'},headers=headers).status_code,400)
        self.assertEqual(client.post('/api/enable',json={},headers=headers).status_code,400)
        self.assertNotIn('hightorque_robot',sys.modules)

    def test_hardware_feedback_staleness_and_gripper(self):
        states = [SimpleNamespace(position=0.,velocity=.1,torque=.2,fault=0,time=12) for _ in range(8)]
        robot = SimpleNamespace(send_get_motor_state_cmd=lambda:None, motor_send_cmd=lambda:None,
                                get_current_state=lambda:states[:7], get_current_state_gripper=lambda:states[7])
        self.c.robots['left']=robot
        self.c.read_hardware()
        self.assertEqual(self.c.snapshot()['gripper_states']['left']['velocity'],.1)
        self.assertEqual(self.c.snapshot()['gripper_states']['left']['torque'],.2)
        self.c.feedback_seen[('left',7)]=time.monotonic()-1
        with self.assertRaisesRegex(ValueError,'M8'):
            self.c.read_hardware()

    def test_script_cannot_control_unselected_arm(self):
        self.c.enable()
        self.c.script_active=True
        self.c.script_sides=['left']
        with self.assertRaises(ValueError):
            self.c.script_commands('right',{'0':['pos_vel_MAXtqe',.1,.2,10]})

    def test_original_panel_contract(self):
        config=self.client.get('/api/legacy/config').json
        self.assertEqual(len(config['joints']),16)
        self.assertEqual(config['joints'][1]['sign'],-1)
        self.assertEqual([x['name'] for x in config['joints'][-2:]],['left_gripper','right_gripper'])
        state=self.client.get('/api/legacy/state').json
        self.assertEqual(len(state['positions']),14)
        self.assertEqual(set(state['gripper_positions']),{'left_gripper','right_gripper'})
        files=self.client.get('/api/arm_description_files').json
        self.assertTrue(files['success'])
        self.assertEqual(len([p for p in files['files'] if p.endswith('.urdf')]),1)

    def test_original_dual_command_is_atomic(self):
        self.post('enable')
        data=dict(positions=[.1,0,0,0,0,0,0]+[999]*7,grippers={'left_gripper':.2,'right_gripper':.3},velocity=.3)
        self.assertEqual(self.post('legacy/move_all',data).status_code,400)
        np.testing.assert_array_equal(self.c.target['left'],[0]*7)
        data['positions']=[.1,0,0,0,0,0,0]+[-.1,0,0,0,0,0,0]
        self.assertEqual(self.post('legacy/move_all',data).status_code,200)
        self.assertEqual(self.c.target['right'][0],-.1)
        self.assertEqual(self.c.grip_target['left'],.2)
        self.assertEqual(self.c.grip_target['right'],.3)

    def test_original_waypoints_contract(self):
        for _ in range(2):
            response=self.post('legacy/add_waypoint',dict(positions=[0]*14,duration=1))
            self.assertEqual(response.status_code,200)
        self.assertEqual(len(response.json['waypoints']),2)
        self.post('enable')
        self.assertEqual(self.post('legacy/run_trajectory').status_code,200)
        self.assertEqual(self.post('legacy/clear_waypoints').status_code,400)
        self.post('legacy/stop_trajectory')
        self.assertFalse(self.c.enabled)

    def test_plot_script_is_supported_and_unsupported_has_reason(self):
        scripts = {s['name']:s for s in self.client.get('/api/scripts').json['scripts']}
        self.assertTrue(scripts['3_sin_trajectory_control_with_plot.py']['supported'])
        for name in ('1_dual_Joint_PD_control.py', '1_dual_forward_kinematics_test.py',
                     '1_dual_inverse_kinematics_test.py', '2_dual_Jointimpendence_control_with_gra_fri_pd.py',
                     '3_gravity_compensation_with_fk.py'):
            self.assertTrue(scripts[name]['supported'], name)
        for name in ('3_interpolation_control_zeroVel.py', '3_interpolation_control_nozeroVel.py'):
            self.assertFalse(scripts[name]['supported'])
            self.assertIn('1 rad/s', scripts[name]['reason'])
        self.assertFalse(scripts['0_dual_robot_set_zero.py']['supported'])
        self.assertTrue(scripts['0_dual_robot_set_zero.py']['reason'])

    def test_wrench_api_marks_unavailable_and_recovers_errors(self):
        self.c.mode = 'impedance'
        self.assertIsNone(self.client.get('/api/legacy/state').json['external_wrench'])
        # Synthetic feedback only: never construct native hardware.
        self.c.live = True
        self.c.active = ['right']
        self.c.model.estimate_wrench = lambda q, tau, side: [1.,2.,3.,.1,.2,.3] if side == 'right' else None
        state = self.client.get('/api/legacy/state').json
        self.assertTrue(state['wrench_valid'])
        self.assertEqual(state['wrench_side'], 'right')
        self.assertEqual(state['external_wrench'][:3], [1.,2.,3.])
        def broken(*args): raise ValueError('model failure')
        self.c.model.estimate_wrench = broken
        state = self.client.get('/api/legacy/state').json
        self.assertFalse(state['wrench_valid'])
        self.assertEqual(state['wrench_error'], 'model failure')
        self.c.live = False

    def test_motor_jacobian_matches_finite_difference_both_arms(self):
        from scipy.spatial.transform import Rotation
        for side in ('left','right'):
            self.c.q[side] = np.array([.2, -.3 if side=='left' else .3, .25, -.6 if side=='left' else .6, .1, -.1, .2])
        for side in ('left','right'):
            gravity, jacobian = self.c.model.dynamics_at(self.c.q, side)
            base = self.c.model.fk(self.c.q)[side]
            for i in range(7):
                perturbed = {s:q.copy() for s,q in self.c.q.items()}
                perturbed[side][i] += 1e-7
                moved = self.c.model.fk(perturbed)[side]
                linear = (np.asarray(moved['position'])-base['position'])/1e-7
                angular = Rotation.from_matrix(np.asarray(moved['rotation']) @ np.asarray(base['rotation']).T).as_rotvec()/1e-7
                np.testing.assert_allclose(jacobian[:,i], np.r_[linear,angular], atol=1e-6)
            self.c.torque[side] = gravity.copy()
            np.testing.assert_allclose(self.c.model.estimate_wrench(self.c.q,self.c.torque,side),np.zeros(6),atol=1e-10)
            applied = np.array([3.,-2.,5.,.2,-.3,.1])
            self.c.torque[side] = gravity - jacobian.T @ applied
            # Independent augmented least-squares reference (DLS regularization).
            expected = np.linalg.lstsq(np.vstack([jacobian.T,.05*np.eye(6)]),
                                      np.r_[jacobian.T@applied,np.zeros(6)],rcond=None)[0]
            np.testing.assert_allclose(self.c.model.estimate_wrench(self.c.q,self.c.torque,side),expected,atol=1e-9)

    def test_fk_matches_pinocchio(self):
        import pinocchio as pin
        model=pin.buildModelFromUrdf(str(self.c.model.path))
        data=model.createData()
        q=np.zeros(model.nq)
        for s in ('left','right'):
            self.c.q[s]=np.array([.2, -.15 if s=='left' else .15, -.12, -.2 if s=='left' else .2, .13, .12, -.1])
            meta=self.c.model.metadata[s]
            for n,angle,sign in zip(meta['joint_names'],self.c.q[s],meta['joint_signs']):
                q[model.joints[model.getJointId(n)].idx_q]=angle*sign
        pin.forwardKinematics(model,data,q)
        pin.updateFramePlacements(model,data)
        fk=self.c.model.fk(self.c.q)
        for s in ('left','right'):
            frame=model.getFrameId(self.c.model.metadata[s]['end_link'],pin.BODY)
            np.testing.assert_allclose(fk[s]['position'],data.oMf[frame].translation,atol=1e-9)
            np.testing.assert_allclose(fk[s]['rotation'],data.oMf[frame].rotation,atol=1e-9)


if __name__ == '__main__':unittest.main()
