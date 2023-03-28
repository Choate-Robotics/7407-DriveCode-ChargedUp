"""
Constant values
"""
import math

from robotpy_toolkit_7407.utils.units import hour, m, mile, rad, rev, s
from wpimath.geometry import Pose3d, Rotation3d, Transform3d

from config import field_length, field_width
from units.SI import (
    inches_to_meters,
    meters,
    meters_per_second,
    meters_per_second_squared,
    radians,
    radians_per_second,
    rotations,
    rotations_per_minute,
    rotations_per_minute_per_second,
)

# boundary dimension constants
# --------------------------------------------------------------
horizontal_boundary: meters = (
    28 * inches_to_meters
)  # the horizontal boundary is the distance from the pivot point (center of robot) to the\
# robots maximum extension limit in the x direction (one side of the robot)
vertical_boundary: meters = (
    78 * inches_to_meters
)  # the vertical boundary is the distance from the floor to the robots maximum extension limit in the y direction
# --------------------------------------------------------------

# boundary buffer constants
# --------------------------------------------------------------
bottom_boundary_buffer_gap: meters = (
    1 * inches_to_meters
)  # the buffer in between the bottom boundary
top_boundary_buffer_gap: meters = (
    0 * inches_to_meters
)  # the buffer in between the top boundary
side_boundary_buffer_gap: meters = (
    0 * inches_to_meters
)  # the buffer in between the side boundaries
# --------------------------------------------------------------

# shoulder constants
# --------------------------------------------------------------
shoulder_max_rotation: radians = math.radians(
    80
)  # the maximum rotation of the shoulder
shoulder_min_rotation: radians = math.radians(
    110
)  # the minimum rotation of the shoulder
shoulder_intake_up_max_rotation: radians = math.radians(
    90
)  # the maximum rotation of the shoulder when the intake is up
# --------------------------------------------------------------

# shoulder buffer constants
# --------------------------------------------------------------
shoulder_min_buffer_rotation: radians = math.radians(
    1
)  # the buffer in between the minimum rotation
shoulder_max_buffer_rotation: radians = math.radians(
    1
)  # the buffer in between the maximum rotation
# --------------------------------------------------------------

# elevator constants
# --------------------------------------------------------------
min_elevator_height: meters = (
    30 * inches_to_meters
)  # the minimum height of the elevator
elevator_pivot_offset: meters = (
    -2.5 * inches_to_meters
)  # offset from the pivot point to the center of the elevator
max_elevator_height: meters = (
    59.5 * inches_to_meters
)  # the maximum height of the elevator
max_elevator_height_delta: meters = (
    40 * inches_to_meters
)  # the maximum height of the elevator
pivot_point_height: meters = 17 * inches_to_meters  # the height of the pivot point
# --------------------------------------------------------------

# --------------------------------------------------------------
elevator_zero_length: meters = (min_elevator_height / 2) + (-elevator_pivot_offset)
# gets the length of the elevator above the pivot point using the offset and the min height
# --------------------------------------------------------------

# arm pose accuracy
# --------------------------------------------------------------
arm_pose_accuracy: float = 0.01  # the accuracy of the arm pose
# --------------------------------------------------------------

# claw constants
# --------------------------------------------------------------
claw_height: meters = 10 * inches_to_meters  # the height of the claw
claw_width: meters = 3 * inches_to_meters  # the width of the claw
claw_length_open: meters = (
    14 * inches_to_meters
)  # the length of the claw when it is open
claw_length_close: meters = (
    8 * inches_to_meters
)  # the length of the claw when it is closed
# --------------------------------------------------------------


# elevator gear ratios
# --------------------------------------------------------------
elevator_rotation_gear_ratio: rotations = 67.38  # to one
elevator_extend_gear_ratio: rotations = 6.33  # 6.33  # to one
elevator_length_per_rotation: meters = (
    1.736 * inches_to_meters
)  # the length of the elevator per rotation
wrist_gear_ratio: rotations = 80  # to one
# 24 rotations to max extension
stabilizer_magnitude: float = (
    2  # the magnitude of the rotation of the arm based on the tip of the robot
)
shoulder_max_velocity: rotations_per_minute = 25 * elevator_rotation_gear_ratio  # RPM
shoulder_max_acceleration: rotations_per_minute_per_second = (
    25 * elevator_rotation_gear_ratio
)  # RPM / S
shoulder_min_acceleration: rotations_per_minute_per_second = (
    5 * elevator_extend_gear_ratio
)  # RPM / S
# --------------------------------------------------------------


# Wrist soft mount
# --------------------------------------------------------------
wrist_max_rotation: radians = math.radians(90)  # the maximum rotation of the wrist
wrist_min_rotation: radians = math.radians(90)  # the minimum rotation of the wrist
# --------------------------------------------------------------

# elevator zeroing constants
# --------------------------------------------------------------
elevator_initial_rotation = (
    0  # the initial rotation of the elevator that it will zero to
)
elevator_initial_length = (
    0 * inches_to_meters
)  # the initial length of the elevator that it will zero to
# --------------------------------------------------------------

claw_motor_speed: float = 0.2

# general claw rotations
# --------------------------------------------------------------
claw_initial_rotation: Rotation3d = Rotation3d(0, 0, 0)
claw_horizontal_rotation: Rotation3d = Rotation3d(0, 90, 0)
claw_cone_intake_rotation: Rotation3d = Rotation3d(0, 30, 0)
claw_transport_rotation: Rotation3d = Rotation3d(0, 0, 0)
claw_cube_intake_rotation: Rotation3d = Rotation3d(0, 90, 0)
claw_cone_peg_rotation: Rotation3d = Rotation3d(0, 90, 0)
claw_cube_platform_rotation: Rotation3d = Rotation3d(0, 90, 0)
# --------------------------------------------------------------

# Robot arm positions
# --------------------------------------------------------------
zero_pose: Pose3d = Pose3d(0, 0, elevator_zero_length, claw_initial_rotation)
cube_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_intake_rotation)
arm_transport_pose: Pose3d = Pose3d(0, 0, 0, claw_transport_rotation)
cone_top_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_intake_rotation)
# --------------------------------------------------------------

# Field intake positions (Relative to the field)
# --------------------------------------------------------------
bottom_cone_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_horizontal_rotation)
top_cone_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_horizontal_rotation)
# --------------------------------------------------------------

# Apriltag positions (Relative to the apriltag)
# --------------------------------------------------------------
# CONE PEGS #
left_high_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
left_mid_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
left_low_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
right_high_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
right_mid_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
right_low_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)

# CUBE PLATFORMS #
center_low_platform_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_platform_rotation)
center_mid_platform_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_platform_rotation)
center_high_platform_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_platform_rotation)

# LOADING STATION #
double_loading_station_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_platform_rotation)
single_loading_station_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_platform_rotation)
# --------------------------------------------------------------

period = 0.03

# --- DRIVETRAIN ---
# drivetrain_turn_gear_ratio = ((8.16 * 4096)/(2*math.pi) * rev_sensor_unit / rad).asNumber()

drivetrain_turn_gear_ratio: rotations = 150 / 7  # 21.428
drivetrain_move_gear_ratio_as_rotations_per_meter = 21.148

drivetrain_move_gear_ratio: rotations_per_minute = (
    drivetrain_move_gear_ratio_as_rotations_per_meter * 60
)  # 20.64 * 62

track_width: meters = 0.60325
robot_length: meters = 0.7366

# TODO Maybe change these
drivetrain_max_vel: meters_per_second = (15 * mile / hour).asNumber(m / s)  # 15 11
drivetrain_max_target_accel: meters_per_second_squared = (
    1.5 * mile / hour
).asNumber(  # 10
    m / s
)
drivetrain_target_max_vel: meters_per_second = (2 * mile / hour).asNumber(m / s)  # 3
drivetrain_max_angular_vel: radians_per_second = (1 * rev / s).asNumber(rad / s)  # 5
drivetrain_max_climb_vel: meters_per_second = (5 * mile / hour).asNumber(m / s)

climber_out = False
# this sets up the operator controller to determine which state the climber is in at the start of the match :)

ApriltagPositionDictRed = {
    1: Pose3d(
        (field_length - inches_to_meters * 610.77),
        (field_width - inches_to_meters * 42.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, 0),
    ),
    2: Pose3d(
        (field_length - inches_to_meters * 610.77),
        (field_width - inches_to_meters * 108.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, 0),
    ),
    3: Pose3d(
        (field_length - inches_to_meters * 610.77),
        (field_width - inches_to_meters * 174.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, 0),
    ),
    4: Pose3d(
        (field_length - inches_to_meters * 636.96),
        (field_width - inches_to_meters * 265.74),
        (inches_to_meters * 27.38),
        Rotation3d(0.0, 0.0, 0),
    ),
    5: Pose3d(
        (field_length - inches_to_meters * 14.25),
        (field_width - inches_to_meters * 265.74),
        (inches_to_meters * 27.38),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    6: Pose3d(
        (field_length - inches_to_meters * 40.45),
        (field_width - inches_to_meters * 174.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    7: Pose3d(
        (field_length - inches_to_meters * 40.45),
        (field_width - inches_to_meters * 108.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    8: Pose3d(
        (field_length - inches_to_meters * 40.45),
        (field_width - inches_to_meters * 42.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
}

ApriltagPositionDictBlue = {
    1: Pose3d(
        (inches_to_meters * 610.77),
        (inches_to_meters * 42.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    2: Pose3d(
        (inches_to_meters * 610.77),
        (inches_to_meters * 108.19),
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    3: Pose3d(
        (inches_to_meters * 610.77),
        (inches_to_meters * 174.19),  # FIRST's diagram has a typo (it says 147.19)
        (inches_to_meters * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    4: Pose3d(
        (inches_to_meters * 636.96),
        (inches_to_meters * 265.74),
        (inches_to_meters * 27.38),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    5: Pose3d(
        (inches_to_meters * 14.25),
        (inches_to_meters * 265.74),
        (inches_to_meters * 27.38),
        Rotation3d(),
    ),
    6: Pose3d(
        (inches_to_meters * 40.45),
        (inches_to_meters * 174.19),  # FIRST's diagram has a typo (it says 147.19)
        (inches_to_meters * 18.22),
        Rotation3d(),
    ),
    7: Pose3d(
        (inches_to_meters * 40.45),
        (inches_to_meters * 108.19),
        (inches_to_meters * 18.22),
        Rotation3d(),
    ),
    8: Pose3d(
        (inches_to_meters * 40.45),
        (inches_to_meters * 42.19),
        (inches_to_meters * 18.22),
        Rotation3d(),
    ),
}

kCameras = {
    "Arducam_OV9281_USB_Camera": [
        Transform3d(
            Pose3d(),
            Pose3d(
                6.43 * inches_to_meters,
                -7 * inches_to_meters,
                22.5 * inches_to_meters,
                Rotation3d(0, 0, math.radians(180)),
            ),
        )
    ],
    "Arducam_OV9281_USB_Camera_2": [
        Transform3d(
            Pose3d(),
            Pose3d(
                -6.43 * inches_to_meters,
                -7 * inches_to_meters,
                22.5 * inches_to_meters,
                Rotation3d(0, 0, math.radians(0)),
            ),
        )
    ],
}

# Climber:

climber_motor_gear_ratio = 16
climber_pivot_rotations = 1.22 * climber_motor_gear_ratio
climber_unlatch_extension = 0.3 * climber_motor_gear_ratio
