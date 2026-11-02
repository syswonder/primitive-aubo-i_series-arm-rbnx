# SPDX-License-Identifier: MulanPSL-2.0
"""In-process stand-in for an AUBO controller, for offline rehearsal.

It answers the same allowlisted calls as the SDK backend, so the whole
Robonix chain (Atlas, primitive, skill, Executor) can run without the arm.
Kinematics use modified-DH parameters; motion follows a smooth time profile
(joint interpolation for ``moveJoint``, straight-line TCP for ``moveLine``)
and stops instantly on ``stopJoint``/``stopLine``. It models no dynamics,
collisions or gripper physics: a successful rehearsal says the software and
the configured poses are consistent, not that the cell is safe.
"""

from __future__ import annotations

import math
import threading
import time

from .backend import check_allowed
from .kinematics import (interpolate_pose, matrix_pose, multiply, pose_matrix,
                         quaternion, rotation_angle, vector)

# AUBO i5H calibration read from an i5H controller, with d1 fitted so that
# forward kinematics of the i5H_B0 cell's measured joints reproduces its
# measured TCP (residual under 0.5 mm).
I5H_KINEMATICS = {
    "a": (0.0, -0.00019999999494757503, 0.40896849998226387,
          0.37558639999385923, -0.00018259999342262745, -0.0005275000003166497),
    "alpha": (0.0, -1.570092956770912, 3.14322279359261, 3.1432716635805598,
              -1.5705868867979174, 1.572347926851597),
    "d": (0.09863, 0.12162379999679979, -0.00012379999679978937,
          0.00012379999679978937, 0.10258079999679466, 0.094),
    "theta": (math.pi, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0),
}
# AUBO i10H parameters as an i10H controller (ARCS 5.0) reports them, with d1
# fitted the same way to a measured joints/TCP pair of an i10H cell (residual
# 1.3 mm).
I10H_KINEMATICS = {
    "a": (0.0, 0.0, 0.647, 0.6005, 0.0, 0.0),
    "alpha": (0.0, -math.pi / 2, math.pi, math.pi, -math.pi / 2, math.pi / 2),
    "d": (0.184, 0.2013, 0.0, 0.0, 0.1025, 0.116),
    "theta": (math.pi, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0),
}
# Keyed by model without its subtype: aubo_i5H_B0 and aubo_i5H share one.
# Another model needs sim.kinematics, copied from the controller's
# RobotConfig.getKinematicsParam (logged by the primitive at start).
KINEMATICS = {"aubo_i5H": I5H_KINEMATICS, "aubo_i10H": I10H_KINEMATICS}


def model_kinematics(model):
    """Built-in parameters for a controller model string, or None."""
    return KINEMATICS.get("_".join(model.split("_")[:2]))


# The i5H_B0 cell's base joint sits at -3.40 rad, so the range is wider than
# +/-180 degrees; +/-360 degrees matches the i-series specification.
JOINT_LIMIT_RAD = math.radians(360.0)
DIGITAL_OUTPUTS = 16
ROTATION_SPEED_RAD_S = 0.5


def _mdh(a, alpha, d, theta):
    """Return the Tx(a) Rx(alpha) Rz(theta) Tz(d) homogeneous transform."""
    ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(theta), math.sin(theta)
    return [[ct, -st, 0.0, a], [ca * st, ca * ct, -sa, -d * sa],
            [sa * st, sa * ct, ca, d * ca], [0.0, 0.0, 0.0, 1.0]]


def tcp_matrix(params, tcp_offset, joints):
    """Forward kinematics: base-to-TCP transform for six joint angles."""
    t = pose_matrix((0, 0, 0, 0, 0, 0))
    for i in range(6):
        t = multiply(t, _mdh(params["a"][i], params["alpha"][i],
                             params["d"][i], params["theta"][i] + joints[i]))
    return multiply(t, pose_matrix(tcp_offset))


def _rotation_vector(m):
    """Axis-angle vector of a 3x3 rotation matrix (log map)."""
    cos_angle = max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1) / 2))
    angle = math.acos(cos_angle)
    axis = (m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1])
    if angle < 1e-9:
        return tuple(v / 2 for v in axis)
    scale = angle / (2 * math.sin(angle)) if angle < math.pi - 1e-6 else 0.0
    if scale == 0.0:
        # Half-turn: recover the axis from the symmetric part.
        axis = tuple(math.sqrt(max(0.0, (m[i][i] + 1) / 2)) for i in range(3))
        return tuple(angle * v for v in axis)
    return tuple(scale * v for v in axis)


def _twist(frm, to):
    """6-vector (translation, rotation vector) taking transform ``frm`` to ``to``."""
    dp = [to[i][3] - frm[i][3] for i in range(3)]
    r = [[sum(to[i][k] * frm[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    return dp + list(_rotation_vector(r))


def _solve(a, b):
    """Solve the square linear system ``a x = b`` by Gaussian elimination."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        m[col], m[pivot] = m[pivot], m[col]
        if abs(m[col][col]) < 1e-15:
            raise ArithmeticError("singular system")
        for r in range(col + 1, n):
            f = m[r][col] / m[col][col]
            for c in range(col, n + 1):
                m[r][c] -= f * m[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))) / m[r][r]
    return x


def solve_ik(params, tcp_offset, seed, target, iterations=100):
    """Damped least-squares IK from ``seed``; return joints or None.

    The solution stays on the seed's branch (no wrapping) and must converge
    to 1e-6 m / rad within the joint limits, like a controller IK that fails
    rather than jumping configurations.
    """
    q = list(vector(seed, 6, "seed"))
    goal = pose_matrix(target)
    damping, h = 1e-4, 1e-6
    for _ in range(iterations):
        current = tcp_matrix(params, tcp_offset, q)
        error = _twist(current, goal)
        if max(abs(v) for v in error) < 1e-6:
            if all(abs(v) <= JOINT_LIMIT_RAD for v in q):
                return tuple(q)
            return None
        jac = []
        for j in range(6):
            probe = q[:]
            probe[j] += h
            jac.append([v / h for v in _twist(current, tcp_matrix(params, tcp_offset, probe))])
        # jac holds columns; build J^T J + damping and J^T e.
        jtj = [[sum(jac[r][k] * jac[c][k] for k in range(6)) + (damping if r == c else 0.0)
                for c in range(6)] for r in range(6)]
        jte = [sum(jac[r][k] * error[k] for k in range(6)) for r in range(6)]
        try:
            dq = _solve(jtj, jte)
        except ArithmeticError:
            return None
        step = max(abs(v) for v in dq)
        if step > 0.2:
            dq = [v * 0.2 / step for v in dq]
        q = [a + b for a, b in zip(q, dq)]
    return None


def _smooth(u):
    """Smoothstep position profile and its derivative for u in [0, 1]."""
    u = max(0.0, min(1.0, u))
    return 3 * u * u - 2 * u ** 3, 6 * u * (1 - u)


class SimController:
    """Simulated controller answering ``call``/``command`` like SdkClient."""

    def __init__(self, settings, clock=time.monotonic):
        """Start at ``settings.sim.initial_joints`` with the gripper open."""
        sim = settings.sim
        self.motion = settings.motion_enabled
        self._clock = clock
        self._lock = threading.RLock()
        self._params = sim.kinematics or model_kinematics(settings.expected_model)
        self._tcp_offset = sim.tcp_offset
        self._model = settings.expected_model
        self._robot = settings.robot
        self._joints = list(sim.initial_joints)
        self._speeds = [0.0] * 6
        self._segment = None
        self._next_exec_id = 1
        self._outputs = [False] * DIGITAL_OUTPUTS
        if settings.gripper is not None:
            self._outputs[settings.gripper.do_index] = settings.gripper.open_value
        self._handlers = {
            "getRobotNames": lambda: [self._robot],
            "SystemInfo.getControlSoftwareFullVersion": lambda: "robonix-sim",
            "RuntimeMachine.getStatus": lambda: "Running",
            "RobotConfig.getRobotType": lambda: self._model,
            "RobotConfig.getJointMinPositions": lambda: [-JOINT_LIMIT_RAD] * 6,
            "RobotConfig.getJointMaxPositions": lambda: [JOINT_LIMIT_RAD] * 6,
            "RobotConfig.getTcpOffset": lambda: list(self._tcp_offset),
            "RobotConfig.getKinematicsParam": lambda *_: {k: list(v) for k, v in self._params.items()},
            "RobotState.getRobotModeType": lambda: "Running",
            "RobotState.getSafetyModeType": lambda: "Normal",
            "RobotState.getJointPositions": lambda: list(self._joints),
            "RobotState.getJointSpeeds": lambda: list(self._speeds),
            "RobotState.getTcpPose": lambda: list(self._tcp(self._joints)),
            "MotionControl.getExecId": lambda: -1 if self._segment is None else self._segment["id"],
            "MotionControl.getSpeedFraction": lambda: 1.0,
            "IoControl.getStandardDigitalOutputNum": lambda: DIGITAL_OUTPUTS,
            "IoControl.getStandardDigitalOutput": self._get_output,
            "IoControl.setStandardDigitalOutput": self._set_output,
            "RobotAlgorithm.forwardKinematics": lambda q: [list(self._tcp(q)), 0],
            "RobotAlgorithm.inverseKinematics": self._inverse,
            "MotionControl.moveJoint": self._move_joint,
            "MotionControl.moveLine": self._move_line,
            "MotionControl.stopJoint": lambda *_: self._stop(),
            "MotionControl.stopLine": lambda *_: self._stop(),
        }

    def _tcp(self, joints):
        """TCP pose for joint angles."""
        return matrix_pose(tcp_matrix(self._params, self._tcp_offset, vector(joints, 6, "joints")))

    def _inverse(self, seed, pose):
        """Controller-shaped IK reply: ``[joints, 0]`` or ``[seed, -1]``."""
        q = solve_ik(self._params, self._tcp_offset, seed, vector(pose, 6, "pose"))
        return [list(seed), -1] if q is None else [list(q), 0]

    def _get_output(self, index):
        """Read one standard digital output."""
        return self._outputs[index]

    def _set_output(self, index, value):
        """Write one standard digital output; return the zero success code."""
        if type(index) is not int or not 0 <= index < DIGITAL_OUTPUTS:
            return -1
        self._outputs[index] = bool(value)
        return 0

    def _start(self, kind, target, duration):
        """Begin a motion segment toward ``target`` over ``duration`` seconds."""
        self._segment = {"id": self._next_exec_id, "kind": kind, "target": target,
                         "start_joints": tuple(self._joints),
                         "start_tcp": self._tcp(self._joints),
                         "t0": self._clock(), "duration": max(0.1, duration)}
        self._next_exec_id += 1
        return 0

    def _move_joint(self, q, acceleration, speed, blend=0, duration=0):
        """Queue joint interpolation; refuse while moving or outside limits."""
        target = vector(q, 6, "joint target")
        if self._segment is not None or speed <= 0:
            return -1
        if any(abs(v) > JOINT_LIMIT_RAD for v in target):
            return -1
        span = max(abs(a - b) for a, b in zip(target, self._joints))
        return self._start("joint", target, 1.5 * span / speed)

    def _move_line(self, pose, acceleration, speed, blend=0, duration=0):
        """Queue a straight TCP line; refuse if any point on it has no IK."""
        target = vector(pose, 6, "pose target")
        if self._segment is not None or speed <= 0:
            return -1
        start = self._tcp(self._joints)
        seed = self._joints
        for i in range(1, 11):
            seed = solve_ik(self._params, self._tcp_offset, seed,
                            interpolate_pose(start, target, i / 10))
            if seed is None:
                return -1
        distance = math.dist(start[:3], target[:3])
        angle = rotation_angle(quaternion(start[3:]), quaternion(target[3:]))
        return self._start("line", target, 1.5 * max(distance / speed,
                                                     angle / ROTATION_SPEED_RAD_S))

    def _stop(self):
        """Stop at the current state (the simulator decelerates instantly)."""
        self._segment = None
        self._speeds = [0.0] * 6
        return 0

    def _advance(self):
        """Move the simulated arm to where the active segment is right now."""
        seg = self._segment
        if seg is None:
            return
        u = (self._clock() - seg["t0"]) / seg["duration"]
        s, ds = _smooth(u)
        if seg["kind"] == "joint":
            span = [b - a for a, b in zip(seg["start_joints"], seg["target"])]
            self._joints = [a + s * d for a, d in zip(seg["start_joints"], span)]
            self._speeds = [ds * d / seg["duration"] for d in span]
        else:
            pose = interpolate_pose(seg["start_tcp"], seg["target"], s)
            q = solve_ik(self._params, self._tcp_offset, self._joints, pose)
            if q is None:
                self._stop()
                return
            previous = self._joints
            self._joints = list(q)
            dt = max(1e-3, self._clock() - seg.get("t_last", seg["t0"]))
            self._speeds = [(a - b) / dt for a, b in zip(self._joints, previous)]
            seg["t_last"] = self._clock()
        if u >= 1:
            self._segment = None
            self._speeds = [0.0] * 6

    def call(self, method, params=()):
        """Answer one allowlisted call against the simulated state."""
        check_allowed(method, self.motion)
        with self._lock:
            self._advance()
            return self._handlers[method](*params)

    def command(self, method, params):
        """Run one write and require the zero success code."""
        from .backend import require_zero
        require_zero(method, self.call(method, params))

    def close(self):
        """Nothing to release."""
