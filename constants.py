from wpimath.geometry import Pose3d, Rotation3d

def inches_to_meters(inches: float) -> float:
    return inches * 0.0254

drivetrain_turn_gear_ratio = 80.4848
#TODO: convert from inches to meters
#robot constants
robot_length = inches_to_meters(29)
horizontal_boundary = inches_to_meters(48)
vertical_boundary = inches_to_meters(78)
min_elevator_height = inches_to_meters(30)
elevator_pivot_offset = inches_to_meters(-3.5)
max_elevator_height = inches_to_meters(59.5)
pivot_point_height = inches_to_meters(17)
#elevator gear ratios
elevator_rotation_gear_ratio: float = 202.14 # to one
elevator_extend_gear_ratio: float = 6.33 # to one
#elevator zeroing constants
elevator_initial_roatation = 0
elevator_initial_height = 0

#general claw rotations
claw_horizontal_rotation: Rotation3d = Rotation3d(0, 90, 0)
claw_cone_intake_rotation: Rotation3d = Rotation3d(0, 100, 0)
claw_transport_rotation: Rotation3d = Rotation3d(0, 0, 0)
claw_cube_intake_rotation: Rotation3d = Rotation3d(0, 50, 0)


#Robot arm positions
cube_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_intake_rotation)
arm_transport_pose: Pose3d = Pose3d(0, 0, 0, claw_transport_rotation)
cone_top_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_intake_rotation)

#Field intake positions (Relative to the field)
bottom_cone_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_horizontal_rotation)
top_cone_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_horizontal_rotation)

#Apriltag peg positions (Relative to the apriltag)
left_high_peg_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)
left_mid_peg_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)
left_low_peg_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)
right_high_peg_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)
right_mid_peg_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)
right_low_peg_pose: Pose3d = Pose3d(0, 0, 0, 0, 0, 0)