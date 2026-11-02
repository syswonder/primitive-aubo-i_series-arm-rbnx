# SPDX-License-Identifier: MulanPSL-2.0
"""Shared fixtures: a manual clock and a simulator-backed configuration."""

from __future__ import annotations

from aubo_arm.settings import Settings

# Joints measured on an i5H_B0 with the tool pointing down over a table.
CELL_JOINTS = [-3.4002364055434917, -0.3420394917580794, 1.7277259340962225,
               0.516106216993623, 1.5716363052154927, 0.04948903428245083]
# The controller-reported TCP at those joints (tool offset 0.16 m).
CELL_TCP = [-0.5241, 0.2641, 0.0522, 3.1245, -0.0017, -1.8789]


class Clock:
    """Monotonic clock that only moves when a test advances it."""

    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def config(**overrides):
    """A valid simulator configuration with motion enabled."""
    cfg = {
        "transport": "sim",
        "expected_model": "aubo_i5H_B0",
        "joint_names": ["shoulder_joint", "upperArm_joint", "foreArm_joint",
                        "wrist1_joint", "wrist2_joint", "wrist3_joint"],
        "workspace": {"min": [-0.9, -0.2, 0.0], "max": [-0.2, 0.6, 0.5]},
        "motion_enabled": True,
        "linear_speed_m_s": 0.05,
        "gripper": {"joint_name": "gripper", "do_index": 5, "open_value": True},
        "sim": {"initial_joints": CELL_JOINTS},
        "max_sample_age_s": 5.0,
    }
    cfg.update(overrides)
    return cfg


def settings(**overrides):
    return Settings.parse(config(**overrides))
