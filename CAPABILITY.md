---
description: AUBO i-series arm; joint/TCP feedback, TCP and joint targets, optional digital-output gripper.
---

# AUBO arm (aubo_arm)

- `joint_states`: six arm joints, plus a `gripper` joint (1.0 open, 0.0
  closed) when the deployment has a digital-output gripper. The gripper value
  is the controller's output level; such a gripper has no finger position
  sensor.
- `end_pose`: measured TCP pose in the controller base frame.
- `pos_command`: move the TCP to a base-frame pose, in a straight line by
  default. Targets must lie inside the configured workspace box.
- `joint_command`: either all six arm joints, or only `gripper`
  (position >= 0.5 opens, < 0.5 closes). Mixed commands are refused.

One motion runs at a time; a command sent while the arm moves is refused and
logged. Watch `joint_states` / `end_pose` to see a target reached. Faults
latch until the primitive is deactivated and activated again.
