# Gripper Contract

The custom manipulation stack explicitly excludes the gripper from the 5-DoF kinematic chain (`lift`, `arm_extension`, `wrist_yaw`, `wrist_pitch`, `wrist_roll`).

## Why the Gripper is Excluded

The current Stretch URDF and driver model the gripper command values in arbitrary units (e.g., `[-100, 200]`), which do not map linearly to radians or meters like the other joints. Including the gripper in the uniform quintic trajectory generation and tracking error bounds would result in safety violations and unexpected behavior.

## Future Integration

When gripper support is required, it must be integrated out-of-band from the main 5-DoF executor.

1. Create a separate `GripperInterface` module.
2. Provide a mapping function from the logical gripper target to the physical `[-100, 200]` domain.
3. Use a separate `FollowJointTrajectory` or `GripperCommand` action server for the gripper.

Do not attempt to add `joint_gripper_finger_left` to the guarded executor's tracking array.
