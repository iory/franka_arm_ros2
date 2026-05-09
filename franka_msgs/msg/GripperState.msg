# Mirror of libfranka's franka::GripperState plus a header for skrobot
# / wait_until_update_all_joints style timestamp checks. Published at
# franka_gripper_node's state-publish rate (typically 30 Hz) on
# ~/<arm>_gripper/state. Useful for downstream code that needs to know
# whether the gripper has been homed (max_width > 0) without parsing
# the joint_states stream.
std_msgs/Header header
float64 width        # current finger separation [m]
float64 max_width    # finger-pair maximum after most recent homing [m] (0 = never homed in this libfranka session)
bool is_grasped      # libfranka thinks an object is currently grasped between the fingers
uint16 temperature   # gripper electronics temperature [degC]
