# SPDX-License-Identifier: MulanPSL-2.0
"""Pose conversions shared by the controller logic and the simulator.

Units are metres and radians. AUBO poses are ``[x, y, z, roll, pitch, yaw]``
with fixed-axis XYZ Euler angles (R = Rz(yaw) Ry(pitch) Rx(roll)); ROS poses
use ``(x, y, z, w)`` quaternions.
"""

from __future__ import annotations

import math


def vector(value, size, label):
    """Return ``value`` as a float tuple of ``size`` finite numbers.

    Booleans, strings and non-finite numbers are rejected instead of coerced,
    so a malformed controller reply or command can never become a target.
    """
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{label} must contain {size} numbers")
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError(f"{label} must contain only finite numbers")
    return tuple(float(v) for v in value)


def quaternion(rpy_angles):
    """Convert AUBO roll/pitch/yaw to a ROS ``(x, y, z, w)`` quaternion."""
    r, p, y = vector(rpy_angles, 3, "rpy")
    cr, cp, cy = (math.cos(v / 2) for v in (r, p, y))
    sr, sp, sy = (math.sin(v / 2) for v in (r, p, y))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def normalized_quaternion(value):
    """Return a unit quaternion; reject inputs whose norm is not close to one.

    A command with a badly scaled quaternion is more likely a bug than an
    intent, so it is refused rather than silently renormalized.
    """
    q = vector(value, 4, "quaternion")
    norm = math.sqrt(sum(v * v for v in q))
    if abs(norm - 1) > 0.001:
        raise ValueError("quaternion norm must be within 0.001 of one")
    return tuple(v / norm for v in q)


def rpy(value):
    """Convert a ROS ``(x, y, z, w)`` quaternion to AUBO roll/pitch/yaw."""
    x, y, z, w = normalized_quaternion(value)
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def rotation_angle(q1, q2):
    """Return the smallest rotation angle in radians between two quaternions."""
    a, b = normalized_quaternion(q1), normalized_quaternion(q2)
    dot = min(1.0, abs(sum(u * v for u, v in zip(a, b))))
    return 2 * math.acos(dot)


def rpy_matrix(rpy_angles):
    """Return the 3x3 rotation matrix for AUBO roll/pitch/yaw."""
    r, p, y = vector(rpy_angles, 3, "rpy")
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matrix_rpy(m):
    """Return AUBO roll/pitch/yaw for a 3x3 (or 4x4) rotation matrix."""
    return (math.atan2(m[2][1], m[2][2]),
            math.atan2(-m[2][0], math.hypot(m[0][0], m[1][0])),
            math.atan2(m[1][0], m[0][0]))


def pose_matrix(pose):
    """Return the 4x4 homogeneous transform of an AUBO pose."""
    p = vector(pose, 6, "pose")
    m = rpy_matrix(p[3:])
    return [m[0] + [p[0]], m[1] + [p[1]], m[2] + [p[2]], [0.0, 0.0, 0.0, 1.0]]


def matrix_pose(t):
    """Return the AUBO pose of a 4x4 homogeneous transform."""
    return (t[0][3], t[1][3], t[2][3]) + matrix_rpy(t)


def multiply(a, b):
    """Multiply two 4x4 matrices."""
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def slerp(q1, q2, s):
    """Spherical interpolation between unit quaternions at fraction ``s``."""
    a, b = list(normalized_quaternion(q1)), list(normalized_quaternion(q2))
    dot = sum(u * v for u, v in zip(a, b))
    if dot < 0:
        b, dot = [-v for v in b], -dot
    if dot > 0.9995:
        mixed = [u + s * (v - u) for u, v in zip(a, b)]
    else:
        theta = math.acos(dot)
        wa = math.sin((1 - s) * theta) / math.sin(theta)
        wb = math.sin(s * theta) / math.sin(theta)
        mixed = [wa * u + wb * v for u, v in zip(a, b)]
    norm = math.sqrt(sum(v * v for v in mixed))
    return tuple(v / norm for v in mixed)


def interpolate_pose(start, end, s):
    """Straight-line position and slerped orientation between two AUBO poses."""
    a, b = vector(start, 6, "start pose"), vector(end, 6, "end pose")
    xyz = tuple(u + s * (v - u) for u, v in zip(a[:3], b[:3]))
    return xyz + rpy(slerp(quaternion(a[3:]), quaternion(b[3:]), s))
