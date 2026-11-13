# SPDX-License-Identifier: MulanPSL-2.0
import math
import unittest

from aubo_arm.backend import RpcError
from aubo_arm.controller import Arm
from aubo_arm.kinematics import quaternion
from aubo_arm.sim import SimController

from helpers import CELL_JOINTS, Clock, settings

NAMES = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]


class Recorder:
    """Wrap a backend and remember every write."""

    def __init__(self, backend):
        self.backend = backend
        self.writes = []
        self.fail_next = None

    def call(self, method, params=()):
        return self.backend.call(method, params)

    def command(self, method, params):
        self.writes.append(method)
        if self.fail_next == method:
            self.fail_next = None
            raise RpcError(f"{method}: injected failure")
        self.backend.command(method, params)

    def close(self):
        pass


class ArmTest(unittest.TestCase):
    def make(self, **overrides):
        self.clock = Clock()
        s = settings(**overrides)
        self.sim = SimController(s, clock=self.clock)
        self.writer = Recorder(self.sim)
        arm = Arm(s, self.sim, self.writer, clock=self.clock, sleep=self.clock.advance)
        arm.connect()
        arm.activate()
        return arm

    def setUp(self):
        self.arm = self.make()

    def run_until_idle(self, arm=None, limit_s=60):
        arm = arm or self.arm
        for _ in range(int(limit_s / 0.1)):
            if arm.pending is None:
                return
            self.clock.advance(0.1)
            arm.refresh()
        self.fail("pending motion never resolved")

    def tcp(self):
        return self.arm.refresh().tcp

    def raise_tcp(self, dz):
        t = self.tcp()
        self.arm.command_pose([t[0], t[1], t[2] + dz], quaternion(t[3:]))
        return t[2] + dz

    def test_linear_pose_command_completes_from_feedback(self):
        z = self.raise_tcp(0.04)
        self.assertEqual(self.writer.writes, ["MotionControl.moveLine"])
        self.assertIsNotNone(self.arm.pending)
        self.run_until_idle()
        self.assertIn("completed", self.arm.last_result)
        self.assertAlmostEqual(self.tcp()[2], z, places=4)
        self.assertEqual(self.arm.fault, "")

    def test_joint_mode_uses_ik_then_move_joint(self):
        arm = self.make(cartesian_motion="joint")
        t = arm.refresh().tcp
        arm.command_pose([t[0] + 0.02, t[1], t[2]], quaternion(t[3:]))
        self.assertEqual(self.writer.writes, ["MotionControl.moveJoint"])
        self.run_until_idle(arm)
        self.assertLess(math.dist(arm.refresh().tcp[:3], (t[0] + 0.02, t[1], t[2])), 1e-3)

    def test_second_command_while_moving_is_refused(self):
        self.raise_tcp(0.1)
        self.clock.advance(0.3)
        with self.assertRaisesRegex(ValueError, "still in progress"):
            self.raise_tcp(0.01)
        self.assertEqual(self.writer.writes, ["MotionControl.moveLine"])

    def test_next_command_accepted_right_after_arrival(self):
        self.raise_tcp(0.02)
        self.clock.advance(30)  # motion done, but the monitor has not seen it yet
        self.raise_tcp(0.02)
        self.assertEqual(self.writer.writes, ["MotionControl.moveLine"] * 2)

    def test_targets_outside_the_workspace_or_step_are_refused(self):
        t = self.tcp()
        with self.assertRaisesRegex(ValueError, "workspace"):
            self.arm.command_pose([t[0], t[1], -0.05], quaternion(t[3:]))
        self.assertEqual(self.writer.writes, [])
        arm = self.make(max_tcp_step_m=0.1)
        t = arm.refresh().tcp
        with self.assertRaisesRegex(ValueError, "max_tcp_step_m"):
            arm.command_pose([t[0], t[1] + 0.15, t[2]], quaternion(t[3:]))
        self.assertEqual(self.writer.writes, [])

    def test_disabled_motion_refuses_every_command(self):
        arm = self.make(motion_enabled=False)
        t = arm.refresh().tcp
        with self.assertRaisesRegex(ValueError, "disabled"):
            arm.command_pose(t[:3], quaternion(t[3:]))
        with self.assertRaisesRegex(ValueError, "disabled"):
            arm.command_joints(["gripper"], [0.0])

    def test_gripper_close_and_open(self):
        self.arm.command_joints(["gripper"], [0.0])
        self.assertFalse(self.arm.refresh().gripper_open)
        self.arm.command_joints(["gripper"], [1.0])
        self.assertTrue(self.arm.refresh().gripper_open)
        self.assertEqual(self.writer.writes, ["IoControl.setStandardDigitalOutput"] * 2)

    def test_gripper_refused_while_the_arm_moves(self):
        self.raise_tcp(0.1)
        self.clock.advance(0.3)
        with self.assertRaises(ValueError):
            self.arm.command_joints(["gripper"], [0.0])
        self.assertTrue(self.arm.refresh().gripper_open)

    def test_mixed_joint_and_gripper_command_is_refused(self):
        with self.assertRaisesRegex(ValueError, "six arm joints or only the gripper"):
            self.arm.command_joints(NAMES[:3] + ["gripper"], [0, 0, 0, 0])

    def test_joint_command_moves_and_completes(self):
        target = [q + 0.03 for q in CELL_JOINTS]
        self.arm.command_joints(list(reversed(NAMES)), list(reversed(target)))
        self.run_until_idle()
        self.assertLess(max(abs(a - b) for a, b in zip(self.arm.refresh().positions, target)), 0.002)

    def test_halt_stops_the_line_and_latches_a_fault(self):
        self.raise_tcp(0.05)
        self.clock.advance(0.5)
        self.arm.halt("operator stop")
        self.assertEqual(self.writer.writes[-1], "MotionControl.stopLine")
        self.run_until_idle()
        self.assertIn("stopped", self.arm.last_result)
        with self.assertRaisesRegex(ValueError, "latched fault"):
            self.raise_tcp(0.01)
        self.arm.activate()
        self.raise_tcp(0.01)

    def test_timeout_halts_the_motion(self):
        arm = self.make(command_timeout_s=1.0)
        t = arm.refresh().tcp
        arm.command_pose([t[0], t[1], t[2] + 0.05], quaternion(t[3:]))
        self.clock.advance(1.5)
        arm.refresh()
        self.assertIn("command_timeout_s", arm.fault)
        self.assertEqual(self.writer.writes[-1], "MotionControl.stopLine")

    def test_motion_that_stops_short_is_a_fault(self):
        self.raise_tcp(0.05)
        self.clock.advance(0.5)
        self.sim.call("MotionControl.stopLine", [1.0, 2.0])  # stopped behind our back
        for _ in range(10):
            self.clock.advance(0.3)
            self.arm.refresh()
        self.assertIn("away from its target", self.arm.fault)

    def test_rejected_write_latches_a_fault(self):
        self.writer.fail_next = "MotionControl.moveLine"
        with self.assertRaises(RpcError):
            self.raise_tcp(0.02)
        self.assertIn("rejected or uncertain", self.arm.fault)

    def test_deactivate_stops_motion_and_refuses_commands(self):
        self.raise_tcp(0.05)
        self.clock.advance(0.5)
        with self.assertRaisesRegex(RpcError, "could not confirm"):
            self.arm.deactivate(timeout_s=0)
        self.assertEqual(self.writer.writes[-1], "MotionControl.stopLine")
        self.run_until_idle()
        with self.assertRaisesRegex(ValueError, "not active"):
            self.raise_tcp(0.01)

    def test_identity_mismatch_fails_connect(self):
        clock = Clock()
        sim = SimController(settings(), clock=clock)
        arm = Arm(settings(expected_model="aubo_i10H"), sim, sim, clock=clock)
        with self.assertRaisesRegex(ValueError, "does not match"):
            arm.connect()

    def test_tcp_offset_mismatch_fails_connect(self):
        clock = Clock()
        s = settings(expected_tcp_offset=[0, 0, 0.2, 0, 0, 0])
        arm = Arm(s, SimController(s, clock=clock), None, clock=clock)
        with self.assertRaisesRegex(ValueError, "TCP offset"):
            arm.connect()


    def test_command_waits_briefly_for_the_previous_motion(self):
        t = self.tcp()
        q = quaternion(t[3:])
        self.arm.command_pose([t[0], t[1], t[2] + 0.02], q)   # about 0.6 s of motion
        self.arm.command_pose([t[0], t[1], t[2] + 0.04], q)   # sent at once: waits, then goes
        self.assertEqual(self.writer.writes, ["MotionControl.moveLine"] * 2)

    def test_joint_path_leaving_the_workspace_is_refused(self):
        # Both ends are above z = 0.050 but the interpolation dips to 0.048.
        arm = self.make(workspace={"min": [-0.9, -0.2, 0.050], "max": [-0.2, 0.6, 0.5]})
        target = list(CELL_JOINTS)
        target[1] += 0.25
        target[2] += 0.25
        target[3] -= 0.5
        with self.assertRaisesRegex(ValueError, "sweep the TCP outside"):
            arm.command_joints(NAMES, target)
        self.assertEqual(self.writer.writes, [])

    def test_failed_stop_is_resent_and_never_reported_as_stopped(self):
        self.raise_tcp(0.05)
        self.clock.advance(0.5)
        self.writer.fail_next = "MotionControl.stopLine"
        with self.assertRaises(RpcError):
            self.arm.halt("operator stop")
        self.assertFalse(self.arm.pending.stop_sent)
        self.arm.refresh()             # the next sample sends the stop again
        self.assertEqual(self.writer.writes.count("MotionControl.stopLine"), 2)
        self.assertTrue(self.arm.pending.stop_sent)
        self.run_until_idle()
        self.assertIn("stopped", self.arm.last_result)

    def test_stop_that_is_never_accepted_keeps_the_motion_pending(self):
        self.raise_tcp(0.05)
        self.clock.advance(0.5)
        self.writer.command = lambda method, params: (_ for _ in ()).throw(RpcError("link down"))
        with self.assertRaisesRegex(RpcError, "could not confirm"):
            self.arm.deactivate(timeout_s=10)
        self.assertIsNotNone(self.arm.pending)
        self.assertIn("STOP NOT CONFIRMED", self.arm.fault)

    def test_activate_refuses_while_the_controller_moves(self):
        t = self.tcp()
        target = list(t)
        target[2] += 0.05
        self.sim.command("MotionControl.moveLine", [target, 0.2, 0.05, 0.0, 0.0])
        fresh = Arm(settings(), self.sim, self.writer, clock=self.clock)
        fresh.connect()
        with self.assertRaisesRegex(RuntimeError, "executing a motion"):
            fresh.activate()

    def test_configured_joint_limits_narrow_the_controller_limits(self):
        q6 = CELL_JOINTS[5]
        arm = self.make(joint_limits={"min": [-7] * 5 + [q6 - 0.01], "max": [7] * 6})
        target = list(CELL_JOINTS)
        target[5] -= 0.02
        with self.assertRaisesRegex(ValueError, "joint limits"):
            arm.command_joints(NAMES, target)
        target[5] += 0.03
        arm.command_joints(NAMES, target)
        self.assertEqual(self.writer.writes, ["MotionControl.moveJoint"])

    def test_low_payload_or_collision_level_refuses_motion(self):
        arm = self.make(min_payload_kg=2.0)
        with self.assertRaisesRegex(ValueError, "payload is 1.0 kg"):
            arm.command_joints(["gripper"], [0.0])
        arm = self.make(min_collision_level=7)
        with self.assertRaisesRegex(ValueError, "collision level is 6"):
            arm.command_joints(["gripper"], [0.0])
        self.assertEqual(self.writer.writes, [])
        self.make(min_payload_kg=0.5, min_collision_level=1).command_joints(["gripper"], [0.0])
        self.assertEqual(self.writer.writes, ["IoControl.setStandardDigitalOutput"])

    def test_line_from_outside_the_workspace_is_refused(self):
        arm = self.make(workspace={"min": [-0.9, -0.2, 0.06], "max": [-0.2, 0.6, 0.5]})
        t = arm.refresh().tcp              # z = 0.052, below the box
        with self.assertRaisesRegex(ValueError, "current TCP is outside"):
            arm.command_pose([t[0], t[1], 0.1], quaternion(t[3:]))


class SettingsTest(unittest.TestCase):
    def test_unknown_keys_and_speed_caps_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "unknown config keys"):
            settings(speeed=1)
        with self.assertRaisesRegex(ValueError, "linear_speed_m_s may not exceed"):
            settings(linear_speed_m_s=0.5)
        with self.assertRaisesRegex(ValueError, "workspace"):
            settings(workspace={"min": [0, 0, 0], "max": [0, 1, 1]})
        with self.assertRaisesRegex(ValueError, "host is required"):
            settings(transport="sdk")
        with self.assertRaisesRegex(ValueError, "joint_limits"):
            settings(joint_limits={"min": [0] * 6})
        with self.assertRaisesRegex(ValueError, "min_collision_level"):
            settings(min_collision_level=1.5)


if __name__ == "__main__":
    unittest.main()
