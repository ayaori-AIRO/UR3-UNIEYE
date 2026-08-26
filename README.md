# UR3 MoveIt 2 examples (ROS 2 Humble)

This package uses MoveIt's `moveit_msgs/action/MoveGroup` action. It does not
send `FollowJointTrajectory` goals itself. The installed UR MoveIt config maps
successful execution requests to `scaled_joint_trajectory_controller`.

The first example defaults to **plan only**. Add `--execute` only after checking
the target and planned path in RViz.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur3_moveit_examples --symlink-install
source install/setup.bash
```

## Preflight checks

With the driver and MoveIt running:

```bash
ros2 action list -t | grep move_action
ros2 action info /move_action
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Expected: `/move_action [moveit_msgs/action/MoveGroup]`, and an `active`
`scaled_joint_trajectory_controller`. If the `ros2 control` verb is unavailable,
install the matching Humble CLI package or inspect the controller manager topic;
RViz Execute already succeeding is also strong evidence of the mapping.

## Stage 1: plan, inspect, execute

First obtain a known-safe target quaternion from the current RViz pose display
or TF. Do not assume the example position is safe for your cell, tooling, table,
or workholding.

Plan only (replace `QX QY QZ QW`):

```bash
ros2 run ur3_moveit_examples move_to_pose \
  -0.40 0.10 0.35 QX QY QZ QW
```

Expected: `MoveIt succeeded`, a nonzero trajectory point count, and no physical
motion. A planning failure prints the symbolic MoveIt error (for example,
`NO_IK_SOLUTION`, `GOAL_IN_COLLISION`, or `PLANNING_FAILED`).

After inspecting the plan in RViz, clear people/objects from the cell, keep the
teach pendant and emergency stop reachable, use reduced/manual mode for the
first run, and execute the exact same target:

```bash
ros2 run ur3_moveit_examples move_to_pose \
  -0.40 0.10 0.35 QX QY QZ QW --execute \
  --velocity-scaling 0.1 --acceleration-scaling 0.1
```

Expected: MoveIt plans, forwards the trajectory through its configured
controller manager, waits for execution, and reports `SUCCESS`. Any planning or
control failure exits with status 1.

The target is the `tool0` pose expressed in `base_link`. Quaternion inputs are
normalized by the program. Position and orientation tolerances default to 5 mm
and 0.01 rad respectively.
