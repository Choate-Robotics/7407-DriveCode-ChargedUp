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

#robot constants
robot_length = inches_to_meters(29) #the length of the robot

#boundry dimension constants
#--------------------------------------------------------------
horizontal_boundary = inches_to_meters(48) #the horizontal boundry is the distance from the pivot point (center of robot) to the robots maximun extension limit in the x direction (one side of the robot)
vertical_boundary = inches_to_meters(78) #the vertical boundry is the distance from the floor to the robots maximun extension limit in the y direction 
#--------------------------------------------------------------

#boundry buffer constants
#--------------------------------------------------------------
bottom_boundry_buffer_gap = inches_to_meters(1) #the buffer in between the bottom boundry
top_boundry_buffer_gap = inches_to_meters(0) #the buffer in between the top boundry
side_boundry_buffer_gap = inches_to_meters(0) #the buffer in between the side boundries
#--------------------------------------------------------------

#shoulder constants
#--------------------------------------------------------------
shoulder_max_rotation = math.radians(135) #the maximum rotation of the shoulder
shoulder_min_rotation = math.radians(135) #the minimum rotation of the shoulder
shoulder_intake_up_max_rotation = math.radians(90) #the maximum rotation of the shoulder when the intake is up
#--------------------------------------------------------------

#shoulder buffer constants
#--------------------------------------------------------------
shoulder_min_buffer_rotation = math.radians(1) #the buffer in between the minimun rotation
shoulder_max_buffer_rotation = math.radians(1) #the buffer in between the maximum rotation
#--------------------------------------------------------------

#elevator constants
#--------------------------------------------------------------
min_elevator_height = inches_to_meters(30) #the minimum height of the elevator
elevator_pivot_offset = inches_to_meters(-2.5) #offset from the pivot point to the center of the elevator
max_elevator_height = inches_to_meters(59.5) #the maximum height of the elevator
pivot_point_height = inches_to_meters(17) #the height of the pivot point
#--------------------------------------------------------------

#DONT CHANGE UNLESS YOU KNOW WHAT YOU ARE DOING/CALL SEBASTIAN PLUNKETT
#--------------------------------------------------------------
elevator_zero_length: float = (min_elevator_height / 2) + (-elevator_pivot_offset) 
#gets the length of the elevator above the pivot point using the offset and the min height
#--------------------------------------------------------------

#arm pose accuracy
#--------------------------------------------------------------
arm_pose_accuracy: float = 0.01; #the accuracy of the arm pose
#--------------------------------------------------------------

#claw constants
#--------------------------------------------------------------
claw_height = inches_to_meters(10) #the height of the claw
claw_width = inches_to_meters(3) #the width of the claw
claw_length_open = inches_to_meters(14) #the length of the claw when it is open
claw_length_close = inches_to_meters(7) #the length of the claw when it is closed
#--------------------------------------------------------------


#elevator gear ratios
#--------------------------------------------------------------
elevator_rotation_gear_ratio: float = 202.14 # to one
elevator_extend_gear_ratio: float = 6.33 # to one
stabalizer_magnitude: float = 2 #the magnitude of the rotation of the arm based on the tip of the robot
#--------------------------------------------------------------

#elevator zeroing constants
#--------------------------------------------------------------
elevator_initial_roatation = radians(0) #the initial rotation of the elevator that it will zero too
elevator_initial_length = inches_to_meters(0) #the initial length of the elevator that it will zero too
#--------------------------------------------------------------


#general claw rotations
#--------------------------------------------------------------
claw_initial_rotation: Rotation3d = Rotation3d(0, 0, 0)
claw_horizontal_rotation: Rotation3d = Rotation3d(0, 90, 0)
claw_cone_intake_rotation: Rotation3d = Rotation3d(0, 30, 0)
claw_transport_rotation: Rotation3d = Rotation3d(0, 0, 0)
claw_cube_intake_rotation: Rotation3d = Rotation3d(0, 90, 0)
claw_cone_peg_rotation: Rotation3d = Rotation3d(0, 90, 0)
claw_cube_platform_rotation: Rotation3d = Rotation3d(0, 90, 0)
#--------------------------------------------------------------

#Robot arm positions
#--------------------------------------------------------------
zero_pose: Pose3d = Pose3d(0, 0, pivot_point_height + elevator_zero_length, claw_initial_rotation)
cube_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_intake_rotation)
arm_transport_pose: Pose3d = Pose3d(0, 0, 0, claw_transport_rotation)
cone_top_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_intake_rotation)
#--------------------------------------------------------------

#Field intake positions (Relative to the field)
#--------------------------------------------------------------
bottom_cone_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_horizontal_rotation)
top_cone_intake_arm_pose: Pose3d = Pose3d(0, 0, 0, claw_horizontal_rotation)
#--------------------------------------------------------------

#Apriltag positions (Relative to the apriltag)
#--------------------------------------------------------------
## CONE PEGS ##
left_high_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
left_mid_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
left_low_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
right_high_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
right_mid_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)
right_low_peg_pose: Pose3d = Pose3d(0, 0, 0, claw_cone_peg_rotation)

## CUBE PLATFORMS ##
center_low_platform_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_platform_rotation)
center_mid_platform_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_platform_rotation)
center_high_platform_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_platform_rotation)

## LOADING STATION ##
double_loading_station_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_platform_rotation)
single_loading_station_pose: Pose3d = Pose3d(0, 0, 0, claw_cube_platform_rotation)
#--------------------------------------------------------------

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
