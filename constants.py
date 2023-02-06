"""
Constant values
"""
import math

from wpimath.geometry import Pose3d, Rotation3d, Transform3d

from units.SI import (
    rotations,
    rotations_per_minute_per_meter,
    meters,
    meters_per_second,
    miles_per_hour_to_meters_per_second,
    radians_per_second,
    rotations_per_second__to__radians_per_second,
    inches_to_meters,
)

period = 0.03

# --- DRIVETRAIN ---
drivetrain_turn_gear_ratio: rotations = 21.45
drivetrain_move_gear_ratio_as_rotations_per_meter: rotations = 20.64

drivetrain_move_gear_ratio: rotations_per_minute_per_meter = 20.64 * 60

track_width: meters = 0.60325
robot_length: meters = 0.7366

# TODO Maybe change these
drivetrain_max_vel: meters_per_second = 3 * miles_per_hour_to_meters_per_second
drivetrain_target_max_vel: meters_per_second = 3 * miles_per_hour_to_meters_per_second
drivetrain_max_angular_vel: radians_per_second = (
    1.5 * rotations_per_second__to__radians_per_second
)
drivetrain_max_climb_vel: meters_per_second = 2 * miles_per_hour_to_meters_per_second

ApriltagPositionDict = {
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
    "Global_Shutter_Camera": [
        Transform3d(
            Pose3d(), Pose3d(-0.7366 / 2, 0, 0, Rotation3d(0, 0, math.radians(180)))
        )
    ]
}
