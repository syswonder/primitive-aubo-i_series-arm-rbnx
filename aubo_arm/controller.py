# SPDX-License-Identifier: MulanPSL-2.0
"""Arm and gripper command logic, independent of ROS and of the backend.

One motion may be outstanding at a time. Completion is judged from measured
feedback only: an empty controller motion queue, stationary joints and the
target reached within tolerance. A fault latches until the primitive is
deactivated and activated again. No power, brake, runtime, safety, TCP or
payload change is ever made, and nothing is retried after an uncertain write
except the stop request itself, which is idempotent.

Locking: ``lock`` guards the shared state (sample, pending motion, fault) and
is held only briefly, so the feedback thread keeps running while a command is
validated. ``_command_lock`` serializes whole commands.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import logging
import math
import threading
import time

from .backend import RpcError
from .kinematics import (interpolate_pose, normalized_quaternion, quaternion,
                         rotation_angle, rpy, vector)

log = logging.getLogger("aubo_arm")

# A stop is confirmed only after the arm has been quiet this long, which is
# longer than the controller's start-up latency (its queue can read empty for
# a moment after a motion was accepted).
STOP_QUIET_S = 1.0
# How long a new command waits for the previous motion to finish settling.
SETTLE_WAIT_S = 1.5
# Samples along a joint-space motion checked against the workspace.
JOINT_PATH_POINTS = 20


@dataclass(frozen=True)
class Sample:
    """One feedback reading. The calls are sequential, not one atomic read."""

    positions: tuple
    speeds: tuple
    tcp: tuple
    mode: str
    safety: str
    exec_id: int
    gripper_open: bool | None
    began: float
    ended: float


@dataclass
class Pending:
    """The one outstanding motion and its completion bookkeeping."""

    kind: str                      # "joint" or "line"
    joints: tuple | None           # joint target, or the IK solution at the line's end
    tcp: tuple | None              # TCP target of a line
    issued: float                  # when the controller accepted it (or the send ended)
    settled: int = 0               # consecutive quiet samples at the target
    idle: int = 0                  # consecutive quiet samples away from the target
    stopping: bool = False         # a stop is wanted
    stop_sent: bool = False        # the controller accepted a stop request
    quiet_since: float | None = None


class Arm:
    """Validate, send and supervise commands for one AUBO robot."""

    def __init__(self, settings, reader, writer, clock=time.monotonic, sleep=time.sleep):
        """``reader`` serves feedback; ``writer`` also accepts allowlisted writes.

        Validation reads made while handling a command (IK, FK) go through the
        writer's connection so they never delay the feedback reads.
        """
        self.settings, self.reader, self.writer = settings, reader, writer
        self.clock, self.sleep = clock, sleep
        self.lock = threading.RLock()
        self._command_lock = threading.Lock()
        self.active = False
        self.fault = ""
        self.sample = None
        self.pending = None
        self.minimum = self.maximum = None
        self.tcp_offset = None
        self.last_result = "no command yet"

    # -- setup -------------------------------------------------------------

    def connect(self):
        """Check identity, limits, TCP and gripper output without changing anything.

        Raises ValueError or RpcError when the controller is not the configured
        robot. Logs the controller's kinematics parameters for reference.
        """
        s, r = self.settings, self.reader.call
        if s.robot not in r("getRobotNames"):
            raise ValueError(f"controller has no robot named {s.robot}")
        model = r("RobotConfig.getRobotType")
        if model != s.expected_model:
            raise ValueError(f"controller model {model!r} does not match {s.expected_model!r}")
        self.minimum = vector(r("RobotConfig.getJointMinPositions"), 6, "joint minimum")
        self.maximum = vector(r("RobotConfig.getJointMaxPositions"), 6, "joint maximum")
        if any(lo >= hi for lo, hi in zip(self.minimum, self.maximum)):
            raise ValueError("controller reported invalid joint limits")
        self.tcp_offset = vector(r("RobotConfig.getTcpOffset"), 6, "TCP offset")
        expected = s.expected_tcp_offset
        if expected is not None and max(abs(a - b) for a, b in zip(expected, self.tcp_offset)) > 1e-6:
            raise ValueError(f"controller TCP offset {self.tcp_offset} differs from "
                             f"expected_tcp_offset {expected}")
        if s.gripper is not None:
            count = r("IoControl.getStandardDigitalOutputNum")
            if type(count) is not int or not 0 <= s.gripper.do_index < count:
                raise ValueError(f"standard digital output {s.gripper.do_index} is unavailable")
        try:
            log.info("controller kinematics: %s", r("RobotConfig.getKinematicsParam", [True]))
        except Exception as exc:  # informational only
            log.warning("controller kinematics unavailable: %s", exc)
        log.info("connected to %s (%s), TCP offset %s", s.robot, model, list(self.tcp_offset))
        self.refresh()

    def activate(self):
        """Clear a latched fault and accept commands.

        Refuses while a motion of ours is unresolved, or while the controller
        itself is executing motion: something else owns the arm right now.
        """
        with self.lock:
            if self.pending is not None:
                raise RuntimeError("an earlier motion is unresolved; cannot activate")
            sample = self.refresh()
            if not self._quiet(sample):
                raise RuntimeError("the controller is executing a motion; cannot activate")
            self.fault = ""
            self.active = True

    # -- feedback ----------------------------------------------------------

    def refresh(self):
        """Read feedback, supervise the pending motion, and return the sample.

        A reading slower than ``max_sample_age_s`` or with malformed fields
        raises RpcError instead of being published.
        """
        s = self.settings
        began = self.clock()
        r = self.reader.call
        positions = vector(r("RobotState.getJointPositions"), 6, "joint positions")
        speeds = vector(r("RobotState.getJointSpeeds"), 6, "joint speeds")
        tcp = vector(r("RobotState.getTcpPose"), 6, "TCP pose")
        mode = r("RobotState.getRobotModeType")
        safety = r("RobotState.getSafetyModeType")
        exec_id = r("MotionControl.getExecId")
        gripper_open = None
        if s.gripper is not None:
            output = r("IoControl.getStandardDigitalOutput", [s.gripper.do_index])
            if type(output) is not bool:
                raise RpcError("controller returned a non-boolean digital output")
            gripper_open = output == s.gripper.open_value
        ended = self.clock()
        if not isinstance(mode, str) or not isinstance(safety, str):
            raise RpcError("controller returned an invalid robot or safety mode")
        if type(exec_id) is not int or exec_id < -1:
            raise RpcError("controller returned an invalid execution id")
        if ended - began > s.max_sample_age_s:
            raise RpcError("feedback acquisition exceeded max_sample_age_s")
        sample = Sample(positions, speeds, tcp, mode, safety, exec_id, gripper_open, began, ended)
        with self.lock:
            if self.sample is None or sample.began >= self.sample.began:
                self.sample = sample
                self._monitor(sample)
        return sample

    def _quiet(self, sample):
        """Controller queue empty and every joint below the stationary speed."""
        return (sample.exec_id == -1 and
                max(abs(v) for v in sample.speeds) < self.settings.stationary_speed_rad_s)

    def _at_target(self, pending, sample):
        """True when the sample is within tolerance of the pending target."""
        s = self.settings
        if pending.kind == "joint":
            return max(abs(a - b) for a, b in zip(sample.positions, pending.joints)) <= s.joint_tolerance_rad
        if math.dist(sample.tcp[:3], pending.tcp[:3]) > s.tcp_tolerance_m:
            return False
        angle = rotation_angle(quaternion(sample.tcp[3:]), quaternion(pending.tcp[3:]))
        return angle <= s.tcp_tolerance_rad

    def _monitor(self, sample):
        """Advance the pending motion's state from one sample (lock held).

        Two quiet samples at the target complete it; five quiet samples away
        from it (after a one-second start-up grace), a controller mode change,
        or ``command_timeout_s`` halt it with a latched fault. A stopping
        motion resolves only once a stop was accepted and the arm has stayed
        quiet for STOP_QUIET_S; a stop that failed is re-sent on every sample.
        """
        p = self.pending
        if p is None or sample.began < p.issued:
            return
        quiet = self._quiet(sample)
        if p.stopping:
            if not p.stop_sent:
                self._send_stop(p, raise_on_failure=False)
                return
            if not quiet:
                p.quiet_since = None
            elif p.quiet_since is None:
                p.quiet_since = sample.began
            elif sample.began - p.quiet_since >= STOP_QUIET_S:
                self.pending = None
                self.last_result = "stopped: feedback confirms a stationary arm"
                log.info(self.last_result)
            return
        if sample.mode != "Running" or sample.safety != "Normal":
            self.halt(f"controller left Running/Normal during motion ({sample.mode}/{sample.safety})",
                      raise_on_failure=False)
            return
        if self.clock() - p.issued > self.settings.command_timeout_s:
            self.halt("motion did not complete within command_timeout_s", raise_on_failure=False)
            return
        at_target = self._at_target(p, sample)
        if quiet and at_target:
            p.settled, p.idle = p.settled + 1, 0
        elif quiet and sample.began - p.issued > 1.0:
            p.settled, p.idle = 0, p.idle + 1
        else:
            p.settled = p.idle = 0
        if p.settled >= 2:
            self.pending = None
            self.last_result = f"completed: {p.kind} target reached"
            log.info(self.last_result)
        elif p.idle >= 5:
            self.halt("motion ended away from its target", raise_on_failure=False)

    # -- commands ----------------------------------------------------------

    @contextmanager
    def _command(self):
        """Serialize commands; give up rather than queue behind a slow one."""
        if not self._command_lock.acquire(timeout=SETTLE_WAIT_S + 1.0):
            raise ValueError("controller is busy; command was not queued")
        try:
            yield
        finally:
            self._command_lock.release()

    def _settle_previous(self):
        """Wait up to SETTLE_WAIT_S for the previous motion to finish, or refuse.

        The caller may have judged arrival slightly earlier than this
        primitive's stricter check, so a short wait avoids a spurious refusal.
        """
        for _ in range(int(SETTLE_WAIT_S / 0.05)):
            sample = self.refresh()
            with self.lock:
                p = self.pending
                if p is None:
                    return
                if not p.stopping and self._quiet(sample) and self._at_target(p, sample):
                    self.pending = None
                    self.last_result = f"completed: {p.kind} target reached"
                    return
                if p.stopping:
                    break
            self.sleep(0.05)
        raise ValueError("the previous motion is still in progress")

    def _ready(self):
        """Check every precondition for a write and return fresh feedback.

        Requires: motion enabled, active, no latched fault, no motion in
        progress, Running/Normal controller, the configured runtime state, a
        stationary arm with an empty queue, and an unchanged TCP offset.
        """
        s = self.settings
        if not s.motion_enabled:
            raise ValueError("motion is disabled in this deployment")
        if not self.active:
            raise ValueError("the arm primitive is not active")
        if self.fault:
            raise ValueError(f"latched fault ({self.fault}); deactivate and activate to clear it")
        self._settle_previous()
        sample = self.refresh()
        if sample.mode != "Running" or sample.safety != "Normal":
            raise ValueError(f"controller must be Running/Normal, not {sample.mode}/{sample.safety}")
        if s.required_runtime:
            runtime = self.writer.call("RuntimeMachine.getStatus")
            if runtime != s.required_runtime:
                raise ValueError(f"controller runtime is {runtime}, {s.required_runtime} is required")
        if not self._quiet(sample):
            raise ValueError("the arm must be stationary with an empty motion queue")
        offset = vector(self.writer.call("RobotConfig.getTcpOffset"), 6, "TCP offset")
        if max(abs(a - b) for a, b in zip(offset, self.tcp_offset)) > 1e-9:
            raise ValueError("the controller TCP offset changed; reactivate the primitive")
        return sample

    def _ik(self, seed, pose):
        """Controller IK near ``seed``; raise ValueError when it has no solution."""
        result = self.writer.call("RobotAlgorithm.inverseKinematics", [list(seed), list(pose)])
        if (not isinstance(result, (list, tuple)) or len(result) != 2
                or type(result[1]) is not int or result[1] != 0):
            raise ValueError(f"no inverse kinematics solution for {list(pose)}")
        return vector(result[0], 6, "IK solution")

    def _fk(self, joints):
        """Controller forward kinematics; raise ValueError when it fails."""
        result = self.writer.call("RobotAlgorithm.forwardKinematics", [list(joints)])
        if (not isinstance(result, (list, tuple)) or len(result) != 2
                or type(result[1]) is not int or result[1] != 0):
            raise ValueError("controller forward kinematics failed")
        return vector(result[0], 6, "FK pose")

    def _check_limits(self, joints):
        """Require the controller's joint limits."""
        if any(not lo <= q <= hi for q, lo, hi in zip(joints, self.minimum, self.maximum)):
            raise ValueError("a joint position is outside the controller joint limits")

    def _check_joint_path(self, start, target):
        """Validate a joint-space motion along its whole interpolation.

        Joint interpolation sweeps the TCP along a curve, so every sampled
        point (not just the end) must keep the TCP inside the workspace.
        """
        self._check_limits(target)
        if max(abs(a - b) for a, b in zip(target, start)) > self.settings.max_joint_step_rad:
            raise ValueError("target exceeds max_joint_step_rad from the current joints")
        for i in range(1, JOINT_PATH_POINTS + 1):
            q = [a + (b - a) * i / JOINT_PATH_POINTS for a, b in zip(start, target)]
            if not self.settings.in_workspace(self._fk(q)[:3]):
                raise ValueError("the joint motion would sweep the TCP outside the workspace")

    def _check_line(self, sample, target):
        """Validate a straight TCP line and return the joints at its end.

        The workspace is a box, so with both ends inside it the whole line is.
        IK at evenly spaced points must succeed within the joint limits and
        may not jump more than ``max_line_joint_jump_rad`` between points (a
        sign of a nearby singularity).
        """
        s = self.settings
        if not s.in_workspace(sample.tcp[:3]):
            raise ValueError("the current TCP is outside the workspace; recover on the pendant")
        seed = sample.positions
        for i in range(1, s.line_check_points + 1):
            q = self._ik(seed, interpolate_pose(sample.tcp, target, i / s.line_check_points))
            self._check_limits(q)
            if max(abs(a - b) for a, b in zip(q, seed)) > s.max_line_joint_jump_rad:
                raise ValueError("the straight line passes too close to a singularity")
            seed = q
        return seed

    def _issue(self, pending, method, params):
        """Send one motion once; start its clock when the controller replied."""
        with self.lock:
            self.pending = pending
            try:
                self.writer.command(method, params)
            except Exception as exc:
                pending.issued = self.clock()
                self.halt(f"{method} rejected or uncertain: {exc}", raise_on_failure=False)
                raise
            pending.issued = self.clock()
            self.last_result = f"{pending.kind} motion accepted; waiting for measured completion"
        log.info("%s sent", method)

    def command_pose(self, position, orientation):
        """Move the TCP to a base-frame pose (``linear`` or ``joint`` motion).

        The target must lie in the workspace box and within the step limits
        of the current TCP. Returns without motion when already there.
        """
        s = self.settings
        with self._command():
            xyz = vector(position, 3, "position")
            quat = normalized_quaternion(orientation)
            sample = self._ready()
            if not s.in_workspace(xyz):
                raise ValueError(f"target {list(xyz)} is outside the configured workspace")
            if math.dist(xyz, sample.tcp[:3]) > s.max_tcp_step_m:
                raise ValueError("target exceeds max_tcp_step_m from the current TCP")
            if rotation_angle(quat, quaternion(sample.tcp[3:])) > s.max_tcp_rotation_rad:
                raise ValueError("target exceeds max_tcp_rotation_rad from the current TCP")
            target = xyz + rpy(quat)
            if self._at_target(Pending("line", None, target, 0.0), sample):
                self.last_result = "already at the target; no motion sent"
                return
            if s.cartesian_motion == "linear":
                end = self._check_line(sample, target)
                self._issue(Pending("line", end, target, math.inf), "MotionControl.moveLine",
                            [list(target), s.linear_accel_m_s2, s.linear_speed_m_s, 0.0, 0.0])
            else:
                q = self._ik(sample.positions, target)
                self._check_joint_path(sample.positions, q)
                self._issue(Pending("joint", q, None, math.inf), "MotionControl.moveJoint",
                            [list(q), s.joint_accel_rad_s2, s.joint_speed_rad_s, 0.0, 0.0])

    def command_joints(self, names, positions, stamp_s=None):
        """Handle a JointState command naming the six arm joints or the gripper.

        Arm targets must respect controller limits and ``max_joint_step_rad``
        and keep the TCP inside the workspace along the whole motion. A
        gripper position of at least 0.5 opens it; below 0.5 closes it. Mixed
        commands are refused.
        """
        s = self.settings
        with self._command():
            if stamp_s is not None and not 0 <= time.time() - stamp_s <= 1.0:
                raise ValueError("command timestamp is stale or in the future")
            names = list(names)
            values = vector(positions, len(names), "positions") if names else ()
            if s.gripper is not None and names == [s.gripper.joint_name]:
                self._gripper(values[0] >= 0.5)
                return
            if len(names) != 6 or set(names) != set(s.joint_names):
                raise ValueError("name either all six arm joints or only the gripper joint")
            mapping = dict(zip(names, values))
            target = tuple(mapping[n] for n in s.joint_names)
            sample = self._ready()
            if max(abs(a - b) for a, b in zip(target, sample.positions)) <= s.joint_tolerance_rad:
                self.last_result = "already at the target; no motion sent"
                return
            self._check_joint_path(sample.positions, target)
            self._issue(Pending("joint", target, None, math.inf), "MotionControl.moveJoint",
                        [list(target), s.joint_accel_rad_s2, s.joint_speed_rad_s, 0.0, 0.0])

    def _gripper(self, open_):
        """Set the gripper output with the arm stationary (command lock held).

        A failed write latches a fault. The output level is not proof of the
        finger position: this gripper has no position sensor.
        """
        g = self.settings.gripper
        self._ready()
        value = g.open_value if open_ else not g.open_value
        with self.lock:
            try:
                self.writer.command("IoControl.setStandardDigitalOutput", [g.do_index, value])
            except Exception as exc:
                self.fault = f"gripper output write failed: {exc}"
                raise
        self.last_result = f"gripper output set to {'open' if open_ else 'closed'}"
        log.info(self.last_result)

    # -- stopping ----------------------------------------------------------

    def _send_stop(self, pending, raise_on_failure=True):
        """Send the stop for ``pending`` (lock held); record whether it was accepted."""
        s = self.settings
        if pending.kind == "line":
            method, params = "MotionControl.stopLine", [s.stop_accel_m_s2, s.stop_accel_rad_s2]
        else:
            method, params = "MotionControl.stopJoint", [s.stop_accel_rad_s2]
        try:
            self.writer.command(method, params)
        except Exception as exc:
            if "STOP NOT CONFIRMED" not in self.fault:
                self.fault += f"; STOP NOT CONFIRMED: {exc}"
            log.critical("%s failed: %s", method, exc)
            if raise_on_failure:
                raise RpcError(self.fault) from exc
            return
        pending.stop_sent, pending.quiet_since = True, None
        self.last_result = "stop accepted; waiting for feedback to confirm a stationary arm"

    def halt(self, reason, raise_on_failure=True):
        """Latch a fault and stop the owned motion, if one is outstanding.

        Until the controller accepts a stop, every call (and every feedback
        sample) sends it again. A failed stop raises RpcError unless
        ``raise_on_failure`` is false, and the motion stays pending, so
        nothing can claim the arm stopped.
        """
        with self.lock:
            if not self.fault:
                self.fault = str(reason)
            log.error("halt: %s", reason)
            p = self.pending
            if p is None or p.stop_sent:
                return
            p.stopping = True
            self._send_stop(p, raise_on_failure)

    def deactivate(self, timeout_s=5.0):
        """Refuse further commands, stop owned motion and wait for feedback.

        Raises RpcError when a stationary arm cannot be confirmed in time; the
        motion then stays pending and a later call tries again.
        """
        with self.lock:
            self.active = False
            if self.pending is not None:
                self.halt("deactivated during motion", raise_on_failure=False)
        deadline = self.clock() + timeout_s
        while self.pending is not None and self.clock() < deadline:
            try:
                self.refresh()
            except RpcError as exc:
                log.error("feedback during deactivation failed: %s", exc)
            self.sleep(0.05)
        if self.pending is not None:
            raise RpcError("deactivation could not confirm that the arm stopped")
