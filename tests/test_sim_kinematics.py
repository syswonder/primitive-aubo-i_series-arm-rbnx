# SPDX-License-Identifier: MulanPSL-2.0
import math
import unittest

from aubo_arm.backend import RpcError
from aubo_arm.kinematics import interpolate_pose, quaternion, rotation_angle, rpy
from aubo_arm.sim import (I5H_KINEMATICS, SimController, model_kinematics, solve_ik,
                          tcp_matrix)
from aubo_arm.kinematics import matrix_pose

from helpers import CELL_JOINTS, CELL_TCP, Clock, settings

OFFSET = (0.0, 0.0, 0.16, 0.0, 0.0, 0.0)


class KinematicsTest(unittest.TestCase):
    def test_rpy_quaternion_round_trip(self):
        for angles in ((3.1245, -0.0017, -1.8789), (0.3, -0.4, 2.5), (0, 0, 0)):
            back = rpy(quaternion(angles))
            self.assertLess(rotation_angle(quaternion(back), quaternion(angles)), 1e-9)

    def test_interpolation_hits_both_ends_and_the_midpoint(self):
        a, b = (0, 0, 0, 3.1, 0, -1.8), (0.1, -0.2, 0.3, 3.1, 0, -1.0)
        self.assertEqual(interpolate_pose(a, b, 0)[:3], (0, 0, 0))
        self.assertLess(math.dist(interpolate_pose(a, b, 1)[:3], b[:3]), 1e-12)
        mid = interpolate_pose(a, b, 0.5)
        self.assertLess(math.dist(mid[:3], (0.05, -0.1, 0.15)), 1e-12)


class SimModelTest(unittest.TestCase):
    def test_default_model_reproduces_the_measured_cell_tcp(self):
        tcp = matrix_pose(tcp_matrix(I5H_KINEMATICS, OFFSET, CELL_JOINTS))
        self.assertLess(math.dist(tcp[:3], CELL_TCP[:3]), 0.0015)
        self.assertLess(rotation_angle(quaternion(tcp[3:]), quaternion(CELL_TCP[3:])), 0.005)

    def test_ik_recovers_joints_for_a_nearby_pose(self):
        target = matrix_pose(tcp_matrix(I5H_KINEMATICS, OFFSET,
                                        [q + 0.05 for q in CELL_JOINTS]))
        q = solve_ik(I5H_KINEMATICS, OFFSET, CELL_JOINTS, target)
        self.assertIsNotNone(q)
        back = matrix_pose(tcp_matrix(I5H_KINEMATICS, OFFSET, q))
        self.assertLess(math.dist(back[:3], target[:3]), 1e-5)

    def test_ik_fails_for_an_unreachable_pose(self):
        self.assertIsNone(solve_ik(I5H_KINEMATICS, OFFSET, CELL_JOINTS,
                                   (3.0, 0.0, 0.0, 3.14, 0.0, 0.0)))

    def test_i10h_model_reproduces_a_measured_i10h_tcp(self):
        # Read from an aubo_i10H controller with a zero TCP offset.
        joints = (-1.5100662, 0.2243520, 2.3012281, 0.5044755, 1.5703981, -1.2273453)
        measured = (-0.1705413, -0.4942979, 0.4075662, -3.1412470, -0.0004589, 1.2895294)
        tcp = matrix_pose(tcp_matrix(model_kinematics("aubo_i10H"), (0,) * 6, joints))
        self.assertLess(math.dist(tcp[:3], measured[:3]), 0.0015)
        self.assertLess(rotation_angle(quaternion(tcp[3:]), quaternion(measured[3:])), 0.005)

    def test_model_kinematics_ignores_the_subtype(self):
        self.assertIs(model_kinematics("aubo_i5H_B0"), I5H_KINEMATICS)
        self.assertIsNone(model_kinematics("aubo_i16"))
        with self.assertRaisesRegex(ValueError, "set sim.kinematics"):
            settings(expected_model="aubo_i16")


class SimControllerTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.sim = SimController(settings(), clock=self.clock)

    def tcp(self):
        return self.sim.call("RobotState.getTcpPose")

    def test_move_line_follows_a_straight_line_and_finishes(self):
        start = self.tcp()
        target = list(start)
        target[2] += 0.03
        self.sim.command("MotionControl.moveLine", [target, 0.2, 0.05, 0.0, 0.0])
        self.assertNotEqual(self.sim.call("MotionControl.getExecId"), -1)
        self.clock.advance(0.4)
        mid = self.tcp()
        self.assertLess(math.dist(mid[:2], start[:2]), 1e-4)
        self.assertTrue(start[2] < mid[2] < target[2])
        self.clock.advance(10)
        self.assertEqual(self.sim.call("MotionControl.getExecId"), -1)
        self.assertLess(math.dist(self.tcp()[:3], target[:3]), 1e-5)
        self.assertEqual(max(abs(v) for v in self.sim.call("RobotState.getJointSpeeds")), 0.0)

    def test_move_line_refuses_an_unreachable_target(self):
        with self.assertRaises(RpcError):
            self.sim.command("MotionControl.moveLine", [[2.0, 0, 0.1, 3.14, 0, 0], 0.2, 0.05, 0, 0])

    def test_stop_freezes_the_arm(self):
        target = list(self.tcp())
        target[2] += 0.05
        self.sim.command("MotionControl.moveLine", [target, 0.2, 0.05, 0.0, 0.0])
        self.clock.advance(0.5)
        self.sim.command("MotionControl.stopLine", [1.0, 2.0])
        frozen = self.tcp()
        self.clock.advance(5)
        self.assertEqual(self.tcp(), frozen)
        self.assertEqual(self.sim.call("MotionControl.getExecId"), -1)

    def test_digital_output_starts_open_and_follows_writes(self):
        self.assertTrue(self.sim.call("IoControl.getStandardDigitalOutput", [5]))
        self.sim.command("IoControl.setStandardDigitalOutput", [5, False])
        self.assertFalse(self.sim.call("IoControl.getStandardDigitalOutput", [5]))

    def test_writes_are_refused_without_motion(self):
        sim = SimController(settings(motion_enabled=False), clock=self.clock)
        with self.assertRaises(RpcError):
            sim.call("IoControl.setStandardDigitalOutput", [5, False])


if __name__ == "__main__":
    unittest.main()
