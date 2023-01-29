"""
Constant values
"""
import math

from robotpy_toolkit_7407.motors.rev_motors import rev_sensor_unit
from robotpy_toolkit_7407.utils.units import hour, m, mile, rad, rev, s
from wpimath.geometry import Pose3d, Rotation3d, Transform3d

period = 0.03

# --- DRIVETRAIN ---
# drivetrain_turn_gear_ratio = ((8.16 * 4096)/(2*math.pi) * rev_sensor_unit / rad).asNumber()

drivetrain_turn_gear_ratio = 21.45
drivetrain_move_gear_ratio = (
    ((20.64 * 4096) / 2 * math.pi) * rev_sensor_unit / rad
).asNumber()
drivetrain_move_gear_ratio_as_rotations_per_meter = 20.64

# track_width = (24.2 * inch).asNumber(m)
track_width = 0.60325
robot_length = 0.7366

# TODO Maybe change these
drivetrain_max_vel = (10 * mile / hour).asNumber(m / s)
drivetrain_target_max_vel = (7 * mile / hour).asNumber(m / s)
drivetrain_max_angular_vel = (2 * rev / s).asNumber(rad / s)
drivetrain_max_climb_vel = (2 * mile / hour).asNumber(m / s)

kCentimetersPerInch = 2.54

kCentimetersPerMeter = 100

kMetersPerInch = kCentimetersPerInch / kCentimetersPerMeter

kApriltagPositionDict = {
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
    "Global_Shutter_Camera": [
        Transform3d(
            Pose3d(), Pose3d(0.7366 / 2, 0, 0, Rotation3d(0, 0, math.radians(0)))
        )
    ]
}
