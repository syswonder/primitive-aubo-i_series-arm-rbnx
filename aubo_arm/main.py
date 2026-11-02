# SPDX-License-Identifier: MulanPSL-2.0
"""Robonix entry point: the standard arm contracts over ROS 2 topics.

INIT validates configuration only. ACTIVATE connects to the controller (or the
simulator), checks its identity, publishes feedback and, when motion is
enabled, subscribes to the command topics. DEACTIVATE stops owned motion and
releases the controller connection.
"""

from __future__ import annotations

import logging
import os
import threading
import time

from robonix_api import Err, Ok, Primitive

from .backend import RpcError
from .controller import Arm
from .kinematics import quaternion
from .settings import Settings

JOINT_STATES = "robonix/primitive/arm/joint_states"
END_POSE = "robonix/primitive/arm/end_pose"
JOINT_COMMAND = "robonix/primitive/arm/joint_command"
POS_COMMAND = "robonix/primitive/arm/pos_command"

log = logging.getLogger("aubo_arm")
provider = Primitive(id="aubo_arm", namespace="robonix/primitive/arm")
settings = None
arm = None
backends = []
publishers = {}
subscriptions = {}
declared = set()
worker = None
stop_event = threading.Event()


def make_backends(s):
    """Return ``(reader, writer, owned)`` for the configured transport.

    The SDK transport opens separate read and write connections so feedback
    keeps flowing while a write is in flight; the simulator is one object.
    """
    if s.transport == "sim":
        from .sim import SimController
        sim = SimController(s)
        return sim, sim, [sim]
    from .sdk import SdkClient
    reader = SdkClient(s.host, s.port, s.robot, s.rpc_timeout_s, motion=False)
    try:
        writer = SdkClient(s.host, s.port, s.robot, s.rpc_timeout_s, motion=s.motion_enabled)
    except Exception:
        reader.close()
        raise
    return reader, writer, [reader, writer]


def close_backends():
    """Close every controller connection this process opened."""
    while backends:
        backend = backends.pop()
        try:
            backend.close()
        except Exception as exc:
            log.error("closing controller connection failed: %s", exc)


def on_joint_command(msg):
    """Validate the header, then hand the command to the arm; log rejections."""
    try:
        if msg.header.frame_id and msg.header.frame_id != settings.base_frame:
            raise ValueError(f"frame_id must be empty or {settings.base_frame}")
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stamp <= 0:
            raise ValueError("joint commands need a current header stamp")
        if len(msg.velocity) or len(msg.effort):
            raise ValueError("velocity and effort commands are not supported")
        arm.command_joints(list(msg.name), list(msg.position), stamp)
    except Exception as exc:
        log.error("joint_command rejected: %s", exc)


def on_pos_command(msg):
    """Hand a TCP target (base frame) to the arm; log rejections."""
    try:
        p, q = msg.position, msg.orientation
        arm.command_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
    except Exception as exc:
        log.error("pos_command rejected: %s", exc)


def publish(sample, clock):
    """Publish one sample as JointState (arm joints + gripper) and TCP Pose."""
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState

    js = JointState()
    js.header.stamp = clock.now().to_msg()
    js.header.frame_id = settings.base_frame
    js.name = list(settings.joint_names)
    js.position = list(sample.positions)
    js.velocity = list(sample.speeds)
    if settings.gripper is not None:
        js.name.append(settings.gripper.joint_name)
        js.position.append(1.0 if sample.gripper_open else 0.0)
        js.velocity.append(0.0)
    publishers[JOINT_STATES].publish(js)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = sample.tcp[:3]
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = quaternion(sample.tcp[3:])
    publishers[END_POSE].publish(pose)


def poll(clock):
    """Publish fresh feedback at ``poll_hz``; halt owned motion if it fails."""
    previous_error = ""
    while not stop_event.is_set():
        began = time.monotonic()
        try:
            publish(arm.refresh(), clock)
            previous_error = ""
        except Exception as exc:
            try:
                arm.halt(f"feedback unavailable: {exc}")
            except RpcError as stop_error:
                log.critical("%s", stop_error)
            if str(exc) != previous_error:
                log.error("feedback failed; commands need reactivation: %s", exc)
                previous_error = str(exc)
        stop_event.wait(max(0.0, 1 / settings.poll_hz - (time.monotonic() - began)))


def bind_ros():
    """Create publishers and (with motion) command subscriptions.

    Each entity is recorded as soon as it exists, so a retry after a partial
    failure never creates a second subscription (which would run every
    command twice).
    """
    from geometry_msgs.msg import Pose
    from rclpy.duration import Duration
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState

    for contract, leaf, msg_type in ((JOINT_STATES, "joint_states", JointState),
                                     (END_POSE, "end_pose", Pose)):
        if contract not in publishers:
            publishers[contract] = provider.create_publisher(
                contract, topic=settings.topic(leaf), msg_type=msg_type, qos="reliable")
    if not settings.motion_enabled:
        return
    # A command older than one second is dropped by DDS, never replayed.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE, lifespan=Duration(seconds=1))
    for contract, leaf, msg_type, callback in (
            (JOINT_COMMAND, "joint_command", JointState, on_joint_command),
            (POS_COMMAND, "pos_command", Pose, on_pos_command)):
        if contract not in subscriptions:
            subscriptions[contract] = provider.create_subscription(
                contract, topic=settings.topic(leaf), msg_type=msg_type,
                callback=callback, qos=qos, declare=False)
        if contract not in declared:
            # The command sink is this provider's own capability, so a failed
            # declaration must fail activation (raised to the caller).
            provider.declare_ros2_topic(contract, settings.topic(leaf), qos="reliable")
            declared.add(contract)


@provider.on_init
def init(cfg):
    """Validate the deployment config; do not touch the controller."""
    global settings
    try:
        parsed = Settings.parse(cfg)
        if (parsed.motion_enabled and parsed.transport == "sdk"
                and os.environ.get("AUBO_ENABLE_MOTION") != "1"):
            raise ValueError("motion on a real controller also requires AUBO_ENABLE_MOTION=1")
    except Exception as exc:
        return Err(f"invalid aubo_arm config: {exc}")
    settings = parsed
    log.info("configured %s transport, motion %s", settings.transport,
             "enabled" if settings.motion_enabled else "disabled")
    return Ok()


@provider.on_activate
def activate():
    """Connect, verify the controller, then start feedback and commands.

    Idempotent while active. Refused while a failed deactivation left the
    previous connection and feedback worker running: that motion must be
    resolved first (deactivate again once the arm is stopped).
    """
    global arm, worker
    from robonix_api.ros import RosBackend

    if arm is not None and arm.active:
        return Ok()
    if worker is not None or backends:
        return Err("a previous deactivation did not confirm the arm stopped; "
                   "deactivate again once it is stationary")
    try:
        reader, writer, owned = make_backends(settings)
        backends.extend(owned)
        candidate = Arm(settings, reader, writer)
        candidate.connect()
        candidate.activate()
        arm = candidate
        bind_ros()
        stop_event.clear()
        worker = threading.Thread(target=poll, args=(RosBackend.get().node.get_clock(),),
                                  name="aubo-arm-feedback", daemon=True)
        worker.start()
    except Exception as exc:
        log.error("activation failed: %s", exc)
        if arm is not None:
            arm.active = False
        if worker is None:
            close_backends()
        return Err(f"aubo_arm activation failed: {exc}")
    return Ok()


@provider.on_deactivate
def deactivate():
    """Stop owned motion, then stop the feedback worker and close the link.

    If the stop cannot be confirmed, feedback keeps running (so the motion
    stays supervised and observable) and the connection stays open; calling
    deactivate again retries.
    """
    global worker
    if arm is not None:
        try:
            arm.deactivate()
        except Exception as exc:
            log.error("deactivation failed: %s", exc)
            return Err(f"cannot confirm the arm stopped: {exc}")
    stop_event.set()
    if worker is not None:
        worker.join(timeout=5)
        if worker.is_alive():
            return Err("feedback worker did not stop")
        worker = None
    close_backends()
    return Ok()


@provider.on_shutdown
def shutdown():
    """Deactivate; the arm is never powered off or braked by this process."""
    return deactivate()


if __name__ == "__main__":
    provider.run()
