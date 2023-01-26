from wpimath import Pose3d
drivetrain_turn_gear_ratio = 80.4848
#TODO: convert from inches to meters
#robot constants
robot_length = 29
horizontal_boundary = 48
vertical_boundary = 78
min_elevator_height = 30
elevator_pivot_offset = -3.5
max_elevator_height = 59.5
pivot_point_height = 17
#elevator gear ratios
elevator_rotation_gear_ratio: float = 1
elevator_extend_gear_ratio: float = 1
#elevator zeroing constants
elevator_initial_roatation = 0
elevator_initial_height = 0

#Robot arm positions
cube_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)
arm_transport_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)
cone_top_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)

#Field intake positions (Relative to the field)
bottom_cone_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)
top_cone_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)

#Apriltag peg positions (Relative to the apriltag)
left_high_peg_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)
left_mid_peg_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)
left_low_peg_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)
right_high_peg_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)
right_mid_peg_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)
right_low_peg_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)