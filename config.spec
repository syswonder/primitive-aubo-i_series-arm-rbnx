# Instance config for the aubo_arm primitive (documentation, not a schema).
# Unknown keys are rejected at INIT.
config:
  transport: sdk                 # sdk (real controller) | sim (offline simulator)
  host: 192.168.1.10             # controller address; required for sdk
  port: 30004                    # ARCS RPC port
  robot: rob1
  expected_model: aubo_i5H_B0    # RobotConfig.getRobotType must match, e.g. aubo_i5H_B0, aubo_i10H
  expected_tcp_offset: [0, 0, 0.16, 0, 0, 0]   # optional; activation fails on mismatch
  joint_names: [shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint]
  base_frame: base_link
  topic_prefix: /aubo_arm        # topics: <prefix>/joint_states, end_pose, joint_command, pos_command

  # Commands are only subscribed when true. A real controller additionally
  # needs AUBO_ENABLE_MOTION=1 in the process environment.
  motion_enabled: false
  workspace:                     # required; every TCP target must lie in this base-frame box (m)
    min: [-0.9, -0.2, 0.0]
    max: [-0.2, 0.6, 0.5]
  cartesian_motion: linear       # pos_command as moveLine (linear) or IK + moveJoint (joint)
  linear_speed_m_s: 0.05         # cap 0.25
  linear_accel_m_s2: 0.2         # cap 1.0
  joint_speed_rad_s: 0.3         # cap 1.0
  joint_accel_rad_s2: 0.5        # cap 2.0
  max_tcp_step_m: 0.6            # per command, from the measured TCP
  max_tcp_rotation_rad: 1.6
  max_joint_step_rad: 0.5        # per joint, and the whole joint path must stay in the workspace
  command_timeout_s: 30
  required_runtime: Running      # RuntimeMachine state required for writes; "" skips the check
  joint_limits:                  # optional; narrows the controller's limits (rad), e.g. a wrist cable
    min: [-6.28, -6.28, -6.28, -6.28, -6.28, -1.80]
    max: [6.28, 6.28, 6.28, 6.28, 6.28, 6.28]
  min_payload_kg: 0              # writes refused while RobotConfig.getPayload is lower; 0 skips
  min_collision_level: 0         # writes refused while RobotConfig.getCollisionLevel is lower; 0 skips

  gripper:                       # omit for an arm without a digital-output gripper
    joint_name: gripper          # extra JointState joint: 1.0 open, 0.0 closed
    do_index: 5                  # controller standard digital output
    open_value: true             # output level that opens the gripper

  sim:                           # only for transport: sim
    initial_joints: [-3.4002, -0.3420, 1.7277, 0.5161, 1.5716, 0.0495]
    tcp_offset: [0, 0, 0.16, 0, 0, 0]
    # Built in for aubo_i5H* and aubo_i10H*; other models must set it from the
    # controller's RobotConfig.getKinematicsParam (logged at activation).
    # kinematics: {a: [...], alpha: [...], d: [...], theta: [...]}   # modified DH

# Environment:
#   AUBO_RPC_CREDENTIALS  path of a 0600 JSON file {"username": ..., "password": ...} (sdk)
#   AUBO_ENABLE_MOTION=1  required with motion_enabled on a real controller
