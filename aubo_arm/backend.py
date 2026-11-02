# SPDX-License-Identifier: MulanPSL-2.0
"""The controller calls this primitive may make, shared by every backend.

A backend exposes ``call(method, params)`` for reads, ``command(method,
params)`` for writes that must return the controller's zero success code, and
``close()``. Method names follow the ARCS interface layout
``<Interface>.<method>``. Nothing outside these sets is ever sent: in
particular no power, brake, runtime, safety, TCP or payload changes.
"""

from __future__ import annotations

READS = frozenset({
    "getRobotNames",
    "SystemInfo.getControlSoftwareFullVersion",
    "RuntimeMachine.getStatus",
    "RobotConfig.getRobotType",
    "RobotConfig.getJointMinPositions",
    "RobotConfig.getJointMaxPositions",
    "RobotConfig.getTcpOffset",
    "RobotConfig.getKinematicsParam",
    "RobotState.getRobotModeType",
    "RobotState.getSafetyModeType",
    "RobotState.getJointPositions",
    "RobotState.getJointSpeeds",
    "RobotState.getTcpPose",
    "MotionControl.getExecId",
    "MotionControl.getSpeedFraction",
    "IoControl.getStandardDigitalOutputNum",
    "IoControl.getStandardDigitalOutput",
    "RobotAlgorithm.forwardKinematics",
    "RobotAlgorithm.inverseKinematics",
})

WRITES = frozenset({
    "MotionControl.moveJoint",
    "MotionControl.moveLine",
    "MotionControl.stopJoint",
    "MotionControl.stopLine",
    "IoControl.setStandardDigitalOutput",
})

# Interfaces reached from the RPC client rather than from a robot interface.
CLIENT_INTERFACES = frozenset({"SystemInfo", "RuntimeMachine"})


class RpcError(RuntimeError):
    """Transport, protocol or controller error. A write may have executed."""


def check_allowed(method, motion):
    """Raise RpcError unless ``method`` is a read, or a write with motion on."""
    if method in READS:
        return
    if motion and method in WRITES:
        return
    raise RpcError(f"controller method is not permitted: {method}")


def require_zero(method, code):
    """Raise RpcError unless ``code`` is the integer zero success code."""
    if type(code) is not int or code != 0:
        raise RpcError(f"{method} rejected by controller: {code!r}")
