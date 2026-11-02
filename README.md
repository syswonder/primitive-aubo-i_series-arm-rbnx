# primitive-aubo-i_series-arm-rbnx

Robonix primitive for AUBO i-series arms on an ARCS controller, with an
optional gripper on one standard digital output. It implements the standard
arm contracts over ROS 2 topics; skills never see the AUBO SDK.

Checked controllers:

| Model (`RobotConfig.getRobotType`) | Controller software | Checked | Gripper |
|---|---|---|---|
| `aubo_i5H_B0` | 0.32.3 | sim rehearsal; reads on the real controller | digital output 5 |
| `aubo_i10H` (subtype `B0`) | 5.0.0-alpha.17 | every read this primitive makes, on the real controller | none on the controller |

| Contract | Topic | Direction |
|---|---|---|
| `robonix/primitive/arm/joint_states` | `<prefix>/joint_states` | out: 6 joints, plus `gripper` when configured |
| `robonix/primitive/arm/end_pose` | `<prefix>/end_pose` | out: TCP pose, base frame |
| `robonix/primitive/arm/pos_command` | `<prefix>/pos_command` | in: TCP target (moveLine) |
| `robonix/primitive/arm/joint_command` | `<prefix>/joint_command` | in: 6 joints, or `gripper` only |

Command topics exist only with `motion_enabled: true`.

## Behaviour

- INIT validates config only. ACTIVATE connects, checks the robot model, TCP
  offset and gripper output, logs the controller's kinematics parameters, and
  starts feedback at `poll_hz`.
- A command runs only when the controller is Running/Normal with the required
  runtime state, the arm is stationary with an empty motion queue, and no
  fault is latched. A command arriving while the previous motion is still
  settling waits up to 1.5 s, then is refused.
- Targets must be inside `workspace` and within the step limits. A straight
  line must start and end inside the box (so all of it is) and is checked by
  IK at `line_check_points` points. A joint-space motion is checked by forward
  kinematics at 20 points along its interpolation, since its TCP path curves.
- Completion is measured: empty queue, stationary joints, target within
  tolerance. Timeout, mode change, a motion ending short of its target, or
  lost feedback stops the motion (`stopLine`/`stopJoint`) and latches a fault.
- A stop counts only once the controller accepted it and the arm stayed quiet
  for 1 s; a failed stop request is re-sent on every feedback sample.
  Deactivation that cannot confirm a stop keeps feedback and the connection
  alive and returns an error; deactivate again to retry. Activation is
  idempotent and is refused while the controller is executing any motion.
- Nothing powers the arm on or off, releases brakes, starts the runtime, or
  changes TCP, payload, speed fraction or safety settings. Prepare the
  controller on the pendant first.

## Transports

- `sdk`: official `pyaubo_sdk` 0.26 over RPC 30004. Credentials come from the
  0600 file named by `AUBO_RPC_CREDENTIALS`; writes also need
  `AUBO_ENABLE_MOTION=1`.
- `sim`: an in-process simulator (modified-DH kinematics, smooth timed
  motion, instant stop, digital outputs) for offline rehearsal of the whole
  Robonix chain. It has built-in kinematics for `aubo_i5H*` and `aubo_i10H*`;
  other models need `sim.kinematics`. It models no dynamics, collisions or
  gripper physics.

See `config.spec` for every field.

## Notes per controller

- `required_runtime` defaults to `Running`. Both controllers above read
  `RuntimeMachine` Stopped when idle, and with the default every write is
  refused. The ARCS docs require the runtime only for blended motion, which
  this primitive never sends; confirm on your controller that RPC motion runs
  with the runtime Stopped before setting it to `""`.
- The controller's joint limits are what the primitive enforces. If a cell
  cannot reach part of that range (cabling, fixtures), narrow `workspace` and
  the step limits rather than relying on the controller to stop the arm.

## Tests

```bash
PYTHONPATH=. python3 -m unittest discover -s tests -v
```

The tests use the simulator and a fake SDK module; no ROS or arm is needed.
