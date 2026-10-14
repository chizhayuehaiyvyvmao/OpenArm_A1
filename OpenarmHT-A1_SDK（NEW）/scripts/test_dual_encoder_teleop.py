"""离线测试：禁止构造真实 SDK，不连接串口、不驱动机械臂。"""
import contextlib
import importlib.util
import io
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import yaml

import dual_encoder_teleop as teleop


class FakeClock:
    def __init__(self):
        self.now = 1000.0

    def time(self):
        return self.now

    def sleep(self, seconds):
        assert seconds >= 0
        self.now += seconds


class FakeMotor:
    def __init__(self, name, motor_id, clock):
        self.name, self.motor_id = name, motor_id
        self.state = SimpleNamespace(position=0., velocity=0., torque=0., time=clock.time(), fault=0)
        self.version = SimpleNamespace(major=4, minor=13, patch=3)
        self.command = ("read",)

    def get_motor_name(self):
        return self.name

    def get_motor_id(self):
        return self.motor_id

    def get_version(self):
        return self.version

    def get_current_motor_state(self):
        return self.state

    def send_state_cmd(self):
        self.command = ("read",)

    def pos_vel_tqe_kp_kd(self, *args):
        self.command = ("mit", *args)

    def stop(self):
        self.command = ("stop",)


class FakeSDK:
    def __init__(self, clock):
        self.clock, self.frames = clock, []
        self.motors = [FakeMotor(teleop.channel_name(s, r, i), i, clock)
                       for s in teleop.SIDES for r in ("leader", "follower") for i in range(1, 9)]
        # 故意反序，确保不能靠全局 ID 排序或 SDK 列表顺序分组。
        self.motors.reverse()

    def get_motors(self):
        return self.motors

    def motor_send_cmd(self):
        self.frames.append({m.name: m.command for m in self.motors})
        for motor in self.motors:
            motor.state.time = self.clock.time()
            if motor.command[0] == "mit":
                motor.state.position = motor.command[1]


class DualTeleopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.settings = teleop.load_settings()
        cls.models = {s: teleop.ArmModel(cls.settings[s]) for s in teleop.SIDES}

    def setUp(self):
        self.clock = FakeClock()
        for name, value in (("time", self.clock.time), ("monotonic", self.clock.time), ("sleep", self.clock.sleep)):
            mock = patch.object(teleop.time, name, value)
            mock.start()
            self.addCleanup(mock.stop)
        no_hardware = patch.object(teleop.single.htr, "Robot", side_effect=AssertionError("真实硬件构造被测试禁止"))
        no_hardware.start()
        self.addCleanup(no_hardware.stop)
        self.sdk = FakeSDK(self.clock)
        self.stop = teleop.StopRequest()
        self.bus = teleop.Bus(self.sdk, self.settings, self.models, self.stop)

    def zeros(self, n=8):
        return {s: np.zeros(n) for s in teleop.SIDES}

    def test_routing_and_native_config_parse_without_robot(self):
        config = teleop.build_motor_config(self.settings)
        ports = config["robot"]["CANboard"]["No_1_CANboard"]["CANport"]
        self.assertEqual([self.settings[s]["leader_can"] for s in teleop.SIDES], [3, 4])
        self.assertEqual([self.settings[s]["follower_can"] for s in teleop.SIDES], [1, 2])
        for can, prefix in ((1, "follower_left"), (2, "follower_right"), (3, "leader_left"), (4, "leader_right")):
            self.assertEqual(ports[f"CANport_{can}"]["serial_id"], can)
            self.assertTrue(all(m["name"].startswith(prefix) for m in ports[f"CANport_{can}"]["motor"].values()))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motors.yaml"
            path.write_text(yaml.safe_dump(config))
            params = teleop.single.htr.parse_robot_params(str(path))
        board = params.CANboards["No_1_CANboard"]
        self.assertEqual(board.CANport_num, 4)
        self.assertEqual(sum(p.motor_num for p in board.CANports.values()), 32)

    def test_repeated_ids_and_reordered_motors_do_not_cross_arms(self):
        for side in teleop.SIDES:
            for role in ("leader", "follower"):
                group = self.bus.channels[side, role]
                self.assertEqual([m.motor_id for m in group], list(range(1, 9)))
                self.assertTrue(all(m.name.startswith(f"{role}_{side}") for m in group))
        self.sdk.motors[0].name = self.sdk.motors[1].name
        with self.assertRaises(RuntimeError):
            teleop.bind_channels(self.sdk, self.settings)

    def test_independent_references_and_preserved_right_signs(self):
        references = {"left": np.linspace(.1, .8, 8), "right": np.linspace(-.8, -.1, 8)}
        mapper = teleop.Mapper(self.settings, self.models, references)
        targets, _, _ = mapper.map(references, self.clock.now)
        for q in targets.values():
            np.testing.assert_allclose(q, 0)
        leader = {s: q.copy() for s, q in references.items()}
        leader["left"][[0, 7]] -= .10
        leader["right"][[2, 4]] += .08
        targets, velocities, _ = mapper.map(leader, self.clock.now + .01)
        np.testing.assert_allclose(targets["left"], [.1, 0, 0, 0, 0, 0, 0, .1])
        np.testing.assert_allclose(targets["right"], [0, 0, -.08, 0, -.08, 0, 0, 0])
        for vel in velocities.values():
            self.assertLessEqual(max(abs(vel)), 1)

    def test_per_side_limits_and_gripper_saturation_keep_other_arm(self):
        mapper = teleop.Mapper(self.settings, self.models, self.zeros())
        for n in range(1, 17):
            leaders = self.zeros()
            leaders["left"][1] = -.1 * n  # 左 J2 正方向很快触限。
            leaders["right"][1] = -.1 * n  # 右 J2 正方向有较大行程。
            leaders["right"][7] = .1 * n
            targets, _, limited = mapper.map(leaders, self.clock.now + n * .01)
        self.assertAlmostEqual(targets["left"][1], self.models["left"].upper[1] - teleop.single.JOINT_LIMIT_MARGIN)
        self.assertAlmostEqual(targets["right"][1], 1.6)
        self.assertAlmostEqual(targets["right"][7], -.2)
        self.assertIn(2, limited["left"])
        self.assertIn(8, limited["right"])

    def test_invalid_second_arm_is_atomic_and_counts_wrap_rejected(self):
        mapper = teleop.Mapper(self.settings, self.models, self.zeros())
        invalid = self.zeros()
        invalid["right"][2] = float("nan")
        with self.assertRaises(teleop.LeaderFault):
            mapper.map(invalid, self.clock.now)
        self.assertIsNone(mapper.last_sample)
        for q in mapper.previous_target.values():
            np.testing.assert_allclose(q, 0)
        invalid = self.zeros()
        invalid["right"][7] = 21.0
        with self.assertRaises(teleop.LeaderFault):
            mapper.map(invalid, self.clock.now)

    def test_single_send_contains_two_arms_and_read_only_leaders(self):
        targets = self.zeros()
        targets["left"][0], targets["right"][0] = -.12, .16
        targets["left"][7], targets["right"][7] = .2, .4
        self.bus.send(targets, self.zeros(7))
        self.assertEqual(len(self.sdk.frames), 1)
        frame = self.sdk.frames[0]
        for name, cmd in frame.items():
            self.assertEqual(cmd[0], "read" if name.startswith("leader_") else "mit")
        self.assertEqual(frame["follower_left_1"][1], -.12)
        self.assertEqual(frame["follower_right_1"][1], .16)
        self.assertEqual(frame["follower_left_8"][1], .2)
        self.assertEqual(frame["follower_right_8"][1], .4)

    def test_bad_second_target_does_not_queue_or_flush_first_arm(self):
        targets = self.zeros()
        targets["right"][1] = 100
        with self.assertRaises(teleop.FollowerFault):
            self.bus.send(targets, self.zeros(7))
        self.assertEqual(self.sdk.frames, [])
        self.assertTrue(all(m.command == ("read",) for m in self.sdk.motors))

    def test_stale_and_faulted_feedback_rejected_before_motion(self):
        motor = self.bus.channels["left", "leader"][3]
        motor.state.time -= 1
        with self.assertRaises(teleop.LeaderFault):
            self.bus.read("leader")
        self.bus.channels["right", "follower"][0].state.fault = 1
        with self.assertRaises(teleop.FollowerFault):
            self.bus.read("follower")
        self.assertEqual(self.sdk.frames, [])

    def test_simultaneous_home_and_reference_capture(self):
        for side in teleop.SIDES:
            self.bus.channels[side, "follower"][0].state.position = -.12 if side == "left" else .17
            self.bus.channels[side, "follower"][7].state.position = .3
            for i, motor in enumerate(self.bus.channels[side, "leader"]):
                motor.state.position = (i + 1) * (.05 if side == "left" else -.05)
        with contextlib.redirect_stdout(io.StringIO()):
            teleop.home_both(self.bus, "TEST")
            references = teleop.capture_references(self.bus)
        for side in teleop.SIDES:
            np.testing.assert_allclose(self.bus.read("follower")[side], 0, atol=1e-12)
            np.testing.assert_allclose(references[side], np.arange(1, 9) * (.05 if side == "left" else -.05))
        for frame in self.sdk.frames:
            self.assertEqual(sum(c[0] == "mit" for c in frame.values()), 16)
            self.assertEqual(sum(c[0] == "read" for c in frame.values()), 16)

    def test_one_leader_disconnect_returns_both_home_then_stops(self):
        with patch.object(teleop, "home_both") as home, patch.object(teleop, "capture_references", return_value=self.zeros()), \
                patch.object(teleop, "run_teleop", side_effect=teleop.LeaderFault("CAN3 stale")), \
                contextlib.redirect_stdout(io.StringIO()):
            result = teleop.execute(self.bus)
        self.assertEqual(result, 1)
        self.assertEqual([c.args[1] for c in home.call_args_list], ["START", "EXIT"])
        self.assertTrue(all(c[0] == ("read" if n.startswith("leader_") else "stop") for n, c in self.sdk.frames[-1].items()))

    def test_follower_fault_stops_both_without_unsafe_home(self):
        with patch.object(teleop, "home_both") as home, patch.object(teleop, "capture_references", return_value=self.zeros()), \
                patch.object(teleop, "run_teleop", side_effect=teleop.FollowerFault("CAN2 stale")), \
                contextlib.redirect_stdout(io.StringIO()):
            result = teleop.execute(self.bus)
        self.assertEqual(result, 1)
        self.assertEqual([c.args[1] for c in home.call_args_list], ["START"])
        self.assertEqual(sum(c[0] == "stop" for c in self.sdk.frames[-1].values()), 16)

    def test_first_interrupt_requests_home_second_allows_only_stop(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.stop.handle(2, None)
        self.assertTrue(self.stop.requested)
        self.assertFalse(self.stop.immediate)
        self.bus.send(self.zeros(), self.zeros(7))
        with contextlib.redirect_stdout(io.StringIO()):
            self.stop.handle(2, None)
        with self.assertRaises(teleop.single.ImmediateStop):
            self.bus.send(self.zeros(), self.zeros(7))
        self.bus.stop_followers()
        self.assertEqual(sum(c[0] == "stop" for c in self.sdk.frames[-1].values()), 16)

    def test_missing_one_encoder_aborts_before_any_motion(self):
        self.bus.channels["right", "leader"][7].state.position = 999
        with patch.object(teleop, "home_both") as home, contextlib.redirect_stdout(io.StringIO()):
            result = teleop.execute(self.bus)
        self.assertEqual(result, 1)
        home.assert_not_called()
        self.assertFalse(any(c[0] == "mit" for frame in self.sdk.frames for c in frame.values()))

    def test_entry_import_retains_separate_can3_zero_interface_without_connecting(self):
        entry = Path(teleop.__file__).with_name("5_teleop_control.py")
        spec = importlib.util.spec_from_file_location("test_teleop_entry", entry)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertIs(module.EncoderLeader, teleop.single.EncoderLeader)
        self.assertEqual(module.LEADER_CONFIG, teleop.single.LEADER_CONFIG)
        self.assertIs(module.main, teleop.main)

    def test_real_loop_maps_both_sides_each_frame_and_exits_on_first_interrupt(self):
        original_send = self.sdk.motor_send_cmd
        count = 0

        def send_and_move_leaders():
            nonlocal count
            original_send()
            count += 1
            self.bus.channels["left", "leader"][0].state.position = -.02 * count
            self.bus.channels["right", "leader"][0].state.position = .01 * count
            self.bus.channels["left", "leader"][7].state.position = -.01 * count
            self.bus.channels["right", "leader"][7].state.position = -.02 * count
            if count == 5:
                self.stop.handle(2, None)

        with patch.object(self.sdk, "motor_send_cmd", side_effect=send_and_move_leaders), contextlib.redirect_stdout(io.StringIO()):
            teleop.run_teleop(self.bus, self.zeros())
        self.assertEqual(len(self.sdk.frames), 5)
        last = self.sdk.frames[-1]
        self.assertAlmostEqual(last["follower_left_1"][1], .08)
        self.assertAlmostEqual(last["follower_right_1"][1], -.04)
        self.assertAlmostEqual(last["follower_left_8"][1], .04)
        self.assertAlmostEqual(last["follower_right_8"][1], .08)

    def test_first_interrupt_lifecycle_returns_both_home(self):
        with patch.object(teleop, "home_both") as home, patch.object(teleop, "capture_references", return_value=self.zeros()), \
                patch.object(teleop, "run_teleop", side_effect=lambda *args: self.stop.handle(2, None)), \
                contextlib.redirect_stdout(io.StringIO()):
            result = teleop.execute(self.bus)
        self.assertEqual(result, 0)
        self.assertEqual([c.args[1] for c in home.call_args_list], ["START", "EXIT"])
        self.assertEqual(sum(c[0] == "stop" for c in self.sdk.frames[-1].values()), 16)

    def test_second_interrupt_during_return_still_sends_stop(self):
        def home(_bus, label):
            if label == "EXIT":
                self.stop.handle(2, None)
                self.stop.check()

        with patch.object(teleop, "home_both", side_effect=home), patch.object(teleop, "capture_references", return_value=self.zeros()), \
                patch.object(teleop, "run_teleop", side_effect=lambda *args: self.stop.handle(2, None)), \
                contextlib.redirect_stdout(io.StringIO()):
            teleop.execute(self.bus)
        self.assertTrue(self.stop.immediate)
        self.assertEqual(sum(c[0] == "stop" for c in self.sdk.frames[-1].values()), 16)


if __name__ == "__main__":
    unittest.main()
