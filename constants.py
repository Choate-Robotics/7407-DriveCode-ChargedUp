from wpimath.geometry import Pose3d, Rotation3d
from math import radians
def inches_to_meters(inches: float) -> float:
    return inches * 0.0254

import math

from robotpy_toolkit_7407.motors.rev_motors import (
    rev_sensor_accel_unit,
    rev_sensor_unit,
    rev_sensor_vel_unit,
)
from robotpy_toolkit_7407.utils.units import deg, ft, hour, inch, m, mile, rad, rev, s
from wpimath.geometry import Pose3d, Rotation3d, Transform3d

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
elevator_initial_roatation = radians(0)
elevator_initial_length = inches_to_meters(0)

#general claw rotations
claw_horizontal_rotation: Rotation3d = Rotation3d(0, 90, 0)
claw_cone_intake_rotation: Rotation3d = Rotation3d(0, 30, 0)
claw_transport_rotation: Rotation3d = Rotation3d(0, 0, 0)
claw_cube_intake_rotation: Rotation3d = Rotation3d(0, 50, 0)
claw_cone_peg_rotation: Rotation3d = Rotation3d(0, 90, 0)
claw_cube_platform_rotation: Rotation3d = Rotation3d(0, 90, 0)

#Robot arm positions
cube_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_intake_rotation)
arm_transport_pose: Pose3d = Pose3d(0, 0, 0, claw_transport_rotation)
cone_top_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_intake_rotation)

#Field intake positions (Relative to the field)
bottom_cone_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_horizontal_rotation)
top_cone_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_horizontal_rotation)

#Apriltag peg positions (Relative to the apriltag)
left_high_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
left_mid_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
left_low_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
right_high_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
right_mid_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
right_low_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
center_low_platform_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_platform_rotation)
center_mid_platform_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_platform_rotation)
center_high_platform_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_platform_rotation)


period = 0.03

# --- DRIVETRAIN ---
# drivetrain_turn_gear_ratio = ((8.16 * 4096)/(2*math.pi) * rev_sensor_unit / rad).asNumber()

drivetrain_turn_gear_ratio = 21.45
drivetrain_move_gear_ratio = (((20.64 * 4096) / 2*math.pi) * rev_sensor_unit / rad).asNumber()
drivetrain_move_gear_ratio_as_rotations_per_meter = 20.64


track_width = (24.2 * inch).asNumber(m)

# TODO Maybe change these
drivetrain_max_vel = (10 * mile / hour).asNumber(m / s)
drivetrain_target_max_vel = (7 * mile / hour).asNumber(m / s)
drivetrain_max_angular_vel = (2 * rev / s).asNumber(rad / s)
drivetrain_max_climb_vel = (2 * mile / hour).asNumber(m / s)

# Change this later on we don't really need these conversions
kCentimetersPerInch = 2.54
"""centimeters / inch"""

kCentimetersPerMeter = 100
"""centimeters / meter"""

kMetersPerInch = kCentimetersPerInch / kCentimetersPerMeter
"""meters / inch"""

# Thanks to 1757 + 6328 for providing field constants
kApriltagPositionDict = {
    # this is gathered from a mapping of the field, derived with positions and ZYX order rotations converted to Quaternions in bulk in a spreadsheet
    1: Pose3d(
        (kMetersPerInch * 610.77),
        (kMetersPerInch * 42.19),
        (kMetersPerInch * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    2: Pose3d(
        (kMetersPerInch * 610.77),
        (kMetersPerInch * 108.19),
        (kMetersPerInch * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    3: Pose3d(
        (kMetersPerInch * 610.77),
        (kMetersPerInch * 174.19),  # FIRST's diagram has a typo (it says 147.19)
        (kMetersPerInch * 18.22),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    4: Pose3d(
        (kMetersPerInch * 636.96),
        (kMetersPerInch * 265.74),
        (kMetersPerInch * 27.38),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    5: Pose3d(
        (kMetersPerInch * 14.25),
        (kMetersPerInch * 265.74),
        (kMetersPerInch * 27.38),
        Rotation3d(),
    ),
    6: Pose3d(
        (kMetersPerInch * 40.45),
        (kMetersPerInch * 174.19),  # FIRST's diagram has a typo (it says 147.19)
        (kMetersPerInch * 18.22),
        Rotation3d(),
    ),
    7: Pose3d(
        (kMetersPerInch * 40.45),
        (kMetersPerInch * 108.19),
        (kMetersPerInch * 18.22),
        Rotation3d(),
    ),
    8: Pose3d(
        (kMetersPerInch * 40.45),
        (kMetersPerInch * 42.19),
        (kMetersPerInch * 18.22),
        Rotation3d(),
    ),
}

kCameras = {
    "Global_Shutter_Camera": Transform3d(Pose3d(), Pose3d(0, 0, 0, Rotation3d()))
}
