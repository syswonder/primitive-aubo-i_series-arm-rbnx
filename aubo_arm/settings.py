# SPDX-License-Identifier: MulanPSL-2.0
"""Deployment configuration for the AUBO arm primitive (see ``config.spec``)."""

from __future__ import annotations

from dataclasses import dataclass, fields
import math

from .kinematics import vector
from .sim import model_kinematics

# Upper bounds for a supervised demo cell. They are deliberately low; raise
# them only after the cell has been validated at the current values.
MAX_LINEAR_SPEED_M_S = 0.25
MAX_LINEAR_ACCEL_M_S2 = 1.0
MAX_JOINT_SPEED_RAD_S = 1.0
MAX_JOINT_ACCEL_RAD_S2 = 2.0


@dataclass(frozen=True)
class Gripper:
    """A gripper driven by one controller standard digital output."""

    joint_name: str = "gripper"
    do_index: int = 5
    open_value: bool = True

    @classmethod
    def parse(cls, cfg):
        """Build from a config mapping; reject unknown keys and wrong types."""
        cfg = dict(cfg)
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f"unknown gripper config keys: {sorted(unknown)}")
        gripper = cls(**cfg)
        if not isinstance(gripper.joint_name, str) or not gripper.joint_name:
            raise ValueError("gripper.joint_name must be a non-empty string")
        if type(gripper.do_index) is not int or gripper.do_index < 0:
            raise ValueError("gripper.do_index must be a non-negative integer")
        if type(gripper.open_value) is not bool:
            raise ValueError("gripper.open_value must be a boolean")
        return gripper


@dataclass(frozen=True)
class Sim:
    """Offline controller stand-in; see ``sim.py``."""

    initial_joints: tuple
    tcp_offset: tuple = (0.0, 0.0, 0.16, 0.0, 0.0, 0.0)
    kinematics: dict | None = None

    @classmethod
    def parse(cls, cfg):
        """Build from a config mapping; the initial joints are required."""
        cfg = dict(cfg)
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f"unknown sim config keys: {sorted(unknown)}")
        if "initial_joints" not in cfg:
            raise ValueError("sim.initial_joints is required")
        kinematics = cfg.get("kinematics")
        if kinematics is not None:
            if not isinstance(kinematics, dict):
                raise ValueError("sim.kinematics must be a mapping")
            kinematics = {k: vector(kinematics.get(k), 6, f"sim.kinematics.{k}")
                          for k in ("a", "alpha", "d", "theta")}
        return cls(vector(cfg["initial_joints"], 6, "sim.initial_joints"),
                   vector(cfg.get("tcp_offset", cls.tcp_offset), 6, "sim.tcp_offset"),
                   kinematics)


@dataclass(frozen=True)
class Settings:
    """Validated primitive configuration. Unknown keys are errors, not ignored."""

    transport: str
    expected_model: str
    joint_names: tuple
    workspace_min: tuple
    workspace_max: tuple
    host: str = ""
    port: int = 30004
    robot: str = "rob1"
    motion_enabled: bool = False
    base_frame: str = "base_link"
    topic_prefix: str = "/aubo_arm"
    poll_hz: float = 10.0
    rpc_timeout_s: float = 1.0
    max_sample_age_s: float = 0.5
    cartesian_motion: str = "linear"
    linear_speed_m_s: float = 0.05
    linear_accel_m_s2: float = 0.2
    joint_speed_rad_s: float = 0.3
    joint_accel_rad_s2: float = 0.5
    stop_accel_m_s2: float = 1.0
    stop_accel_rad_s2: float = 2.0
    command_timeout_s: float = 30.0
    joint_tolerance_rad: float = 0.002
    tcp_tolerance_m: float = 0.001
    tcp_tolerance_rad: float = 0.005
    stationary_speed_rad_s: float = 0.003
    max_tcp_step_m: float = 0.6
    max_tcp_rotation_rad: float = 1.6
    max_joint_step_rad: float = 0.5
    line_check_points: int = 10
    max_line_joint_jump_rad: float = 0.35
    required_runtime: str = "Running"
    expected_tcp_offset: tuple | None = None
    gripper: Gripper | None = None
    sim: Sim | None = None

    @classmethod
    def parse(cls, cfg):
        """Validate a deployment ``config:`` mapping into Settings.

        Nested ``workspace``, ``gripper`` and ``sim`` mappings are flattened or
        parsed into their own types. Anything unknown, mistyped, non-finite or
        above the demo speed caps raises ``ValueError``.
        """
        cfg = dict(cfg or {})
        workspace = cfg.pop("workspace", None)
        if not isinstance(workspace, dict) or set(workspace) != {"min", "max"}:
            raise ValueError("workspace must be a mapping with min and max")
        cfg["workspace_min"] = vector(workspace["min"], 3, "workspace.min")
        cfg["workspace_max"] = vector(workspace["max"], 3, "workspace.max")
        if cfg.get("gripper") is not None:
            cfg["gripper"] = Gripper.parse(cfg["gripper"])
        if cfg.get("sim") is not None:
            cfg["sim"] = Sim.parse(cfg["sim"])
        if cfg.get("expected_tcp_offset") is not None:
            cfg["expected_tcp_offset"] = vector(cfg["expected_tcp_offset"], 6,
                                                "expected_tcp_offset")
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f"unknown config keys: {sorted(unknown)}")
        if not isinstance(cfg.get("joint_names"), (list, tuple)):
            raise ValueError("joint_names must list six joint names")
        cfg["joint_names"] = tuple(cfg["joint_names"])
        settings = cls(**cfg)
        settings._validate()
        return settings

    def _validate(self):
        """Check cross-field rules after construction; raise ValueError."""
        names = self.joint_names
        if (len(names) != 6 or len(set(names)) != 6
                or any(not isinstance(n, str) or not n for n in names)):
            raise ValueError("six unique joint_names are required")
        if self.gripper is not None and self.gripper.joint_name in names:
            raise ValueError("gripper.joint_name must differ from the arm joints")
        if self.transport not in {"sdk", "sim"}:
            raise ValueError("transport must be sdk or sim")
        if self.transport == "sdk" and not (isinstance(self.host, str) and self.host):
            raise ValueError("host is required for the sdk transport")
        if self.transport == "sim" and self.sim is None:
            raise ValueError("the sim transport requires a sim mapping")
        if (self.transport == "sim" and self.sim.kinematics is None
                and model_kinematics(self.expected_model) is None):
            raise ValueError(f"no built-in kinematics for {self.expected_model}; set sim.kinematics")
        if type(self.port) is not int or not 0 < self.port < 65536:
            raise ValueError("port must be a TCP port number")
        if not isinstance(self.robot, str) or not self.robot.isidentifier():
            raise ValueError("robot must be a controller robot name such as rob1")
        if type(self.motion_enabled) is not bool:
            raise ValueError("motion_enabled must be a boolean")
        for label in ("expected_model", "base_frame", "topic_prefix"):
            if not isinstance(getattr(self, label), str) or not getattr(self, label):
                raise ValueError(f"{label} must be a non-empty string")
        if not self.topic_prefix.startswith("/") or self.topic_prefix.endswith("/"):
            raise ValueError("topic_prefix must be an absolute ROS name without a trailing /")
        if self.cartesian_motion not in {"linear", "joint"}:
            raise ValueError("cartesian_motion must be linear or joint")
        if not isinstance(self.required_runtime, str):
            raise ValueError("required_runtime must be a string ('' disables the check)")
        for f in fields(self):
            value = getattr(self, f.name)
            if f.name.endswith(("_s", "_s2", "_rad", "_m", "_hz")):
                if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                    raise ValueError(f"{f.name} must be finite and positive")
        if type(self.line_check_points) is not int or not 2 <= self.line_check_points <= 50:
            raise ValueError("line_check_points must be an integer from 2 to 50")
        if not 1 <= self.poll_hz <= 30:
            raise ValueError("poll_hz must be between 1 and 30")
        caps = (("linear_speed_m_s", MAX_LINEAR_SPEED_M_S),
                ("linear_accel_m_s2", MAX_LINEAR_ACCEL_M_S2),
                ("joint_speed_rad_s", MAX_JOINT_SPEED_RAD_S),
                ("joint_accel_rad_s2", MAX_JOINT_ACCEL_RAD_S2))
        for label, cap in caps:
            if getattr(self, label) > cap:
                raise ValueError(f"{label} may not exceed {cap}")
        if any(lo >= hi for lo, hi in zip(self.workspace_min, self.workspace_max)):
            raise ValueError("workspace.min must be below workspace.max on every axis")

    def topic(self, leaf):
        """Return the ROS topic for one contract leaf, e.g. ``joint_states``."""
        return f"{self.topic_prefix}/{leaf}"

    def in_workspace(self, xyz):
        """True when a base-frame point lies inside the configured box."""
        return all(lo <= v <= hi for v, lo, hi in
                   zip(xyz, self.workspace_min, self.workspace_max))
