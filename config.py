import math
from dataclasses import dataclass

from wpimath.geometry import Pose2d, Translation2d

import units.SI
from units.SI import (
    inches_to_meters,
    meters,
    meters_per_second,
    meters_per_second_squared,
    radians,
)

# Climber Configurations:
# Needs to be verified

climber_motor_id = 7

compressor = 31
climber_forwardChannel = 11
climber_reverseChannel = 10

latch_forwardChannel = 8
latch_reverseChannel = 9

current_scoring_position = "None"

intake_inverted = False

grabber_target_angle = 0
grabber_disable_intake = False

blue_team: bool = False
drivetrain_reversed: bool = False
driver_centric: bool = True

field_length = 651.25 * inches_to_meters
field_width = 315.5 * inches_to_meters
scoring_width = 216.2 * inches_to_meters  # 5.44

drivetrain_scoring_velocity = 0.5
drivetrain_scoring_angular_velocity = 1
drivetrain_routing_velocity = 2
drivetrain_routing_acceleration = 1
drivetrain_routing_angular_velocity = 3


@dataclass
class TargetData:
    arm_angle: radians
    arm_length: meters
    wrist_angle: radians
    target_pose: Pose2d | None
    target_waypoints: list[Translation2d] = None
    intake_enabled: bool = False
    intake_reversed: bool = False
    intake_off: bool = False

    claw_picking: bool = False
    cube_picking: bool = False
    cone_picking: bool = False
    double_station_picking: bool = False

    claw_scoring: bool = False
    arm_scoring: bool = False
    arm_reversed: bool = False

    claw_wait: bool = False

    max_velocity: meters_per_second = None
    max_acceleration: meters_per_second_squared = None
    max_angular_velocity: meters_per_second = None

    grabber_no_grab: bool = False

    low_scoring: bool = False
    arm_angle_opposite: float | None = None
    arm_length_opposite: float | None = None
    wrist_angle_opposite: float | None = None

    no_intake: bool = False


elevator_motor_extend_id = 17
elevator_secondary_rotation_motor_id = 1
elevator_main_rotation_motor_id = 2
elevator_brake_id = 3
# converted to radians in subsystems/elevator.py

pneumatics_control_module = 31

claw_motor_speed: float = 0.6

# Intake
intake_motor_id = 11
intake_piston_forwardChannel = 4
intake_piston_reverseChannel = 5

default_intake_speed = 0.40

kRobotVisionPoseWeight = 0.1
# Dummy data
claw_motor_extend_id = 0

blue_scoring_positions = [
    Pose2d(1.46, 4.99, 0),  # Cone
    Pose2d(1.46, 4.43, 0),  # Cube
    Pose2d(1.46, 3.87, 0),  # Cone
    Pose2d(1.46, 3.31, 0),  # Cone
    Pose2d(1.46, 2.75, 0),  # Cube
    Pose2d(1.46, 2.19, 0),  # Cone
    Pose2d(1.46, 1.63, 0),  # Cone
    Pose2d(1.46, 1.07, 0),  # Cube
    Pose2d(1.46, 0.51, 0),  # Cone
]

red_scoring_positions = [
    Pose2d(1.46, field_width - 0.54, 0),  # Cone
    Pose2d(1.46, field_width - 1.66, 0),  # Cube
    Pose2d(1.46, field_width - 1.10, 0),  # Cone
    Pose2d(1.46, field_width - 2.22, 0),  # Cone
    Pose2d(1.46, field_width - 2.78, 0),  # Cube
    Pose2d(1.46, field_width - 3.34, 0),  # Cone
    Pose2d(1.46, field_width - 3.90, 0),  # Cone
    Pose2d(1.46, field_width - 4.46, 0),  # Cube
    Pose2d(1.46, field_width - 5.02, 0),  # Cone
]

# SCORING LOCATIONS
scoring_locations: dict[str, TargetData] = {
    "low": TargetData(
        target_pose=None,
        arm_angle=math.radians(-98),
        arm_length=0,
        wrist_angle=math.radians(0),
        arm_angle_opposite=math.radians(50),
        arm_length_opposite=0.2,
        wrist_angle_opposite=math.radians(90),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=False,
        cone_picking=False,
        low_scoring=True,
        arm_scoring=True,
    ),
    "middle": TargetData(
        target_pose=Pose2d(1.55, 1.55, 0),  # 2.43 .94
        target_waypoints=[Translation2d(1.81, 1.55)],
        arm_angle=math.radians(-53.78),
        arm_length=0.55,
        wrist_angle=math.radians(-27.09),
        intake_enabled=False,
        claw_scoring=True,
        claw_picking=False,
        arm_scoring=True,
        max_velocity=1,
        max_acceleration=0.5,
        max_angular_velocity=1,
        claw_wait=True,
    ),
    "high": TargetData(
        target_pose=None,
        arm_angle=math.radians(-49.7),
        arm_length=1.01,
        wrist_angle=math.radians(25),
        intake_enabled=False,
        claw_scoring=True,
        claw_picking=False,
        arm_scoring=True,
        max_velocity=1,
        max_acceleration=0.5,
        max_angular_velocity=1,
        claw_wait=True,
    ),
    "high_auto_back": TargetData(
        target_pose=None,
        arm_angle=math.radians(-49.7),
        arm_length=1.01,
        wrist_angle=math.radians(-25),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=False,
        arm_scoring=False,
        max_velocity=1,
        max_acceleration=0.5,
        max_angular_velocity=1,
        claw_wait=True,
    ),
    "high_auto_back_intake": TargetData(
        target_pose=None,
        arm_angle=math.radians(-49.7),
        arm_length=1.01,
        wrist_angle=math.radians(-25),
        intake_enabled=True,
        claw_scoring=False,
        claw_picking=False,
        arm_scoring=False,
        max_velocity=1,
        max_acceleration=0.5,
        max_angular_velocity=1,
        claw_wait=True,
    ),
    "high_auto_back_cube": TargetData(
        target_pose=None,
        arm_angle=math.radians(-49.7),
        arm_length=1.01,
        wrist_angle=math.radians(-30),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=False,
        arm_scoring=False,
        max_velocity=1,
        max_acceleration=0.5,
        max_angular_velocity=1,
    ),
    "mid_auto_back_cube": TargetData(
        target_pose=None,
        arm_angle=math.radians(-64),
        arm_length=0.55,
        wrist_angle=math.radians(-24.09),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=False,
        arm_scoring=False,
        max_velocity=1,
        max_acceleration=0.5,
        max_angular_velocity=1,
    ),
    "middle_auto_back": TargetData(
        target_pose=None,
        arm_angle=math.radians(-64),
        arm_length=0.55,
        wrist_angle=math.radians(-24.09),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=False,
        arm_scoring=False,
        max_velocity=1,
        max_acceleration=0.5,
        max_angular_velocity=1,
        claw_wait=False,
    ),
    "middle_auto_front": TargetData(
        target_pose=None,
        arm_angle=math.radians(46.78),
        arm_length=0.55,
        wrist_angle=math.radians(27.09),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=False,
        arm_scoring=False,
        max_velocity=1,
        max_acceleration=0.5,
        max_angular_velocity=1,
        claw_wait=False,
    ),
    "pickup": TargetData(
        target_pose=None,
        arm_angle=math.radians(-108),
        arm_length=0.199,
        wrist_angle=math.radians(-10),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=True,
        cone_picking=True,
    ),
    "double_station": TargetData(
        target_pose=Pose2d(16, 7.51, math.radians(0)),
        arm_angle=math.radians(32.84),
        arm_length=0.3220,  # .322 for comp
        wrist_angle=math.radians(54.63),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=True,
        arm_scoring=True,
        arm_reversed=True,
        double_station_picking=True,
    ),
    "cube_intake": TargetData(
        target_pose=None,
        arm_angle=math.radians(74.5),
        arm_length=2.5 * units.SI.inches_to_meters,
        wrist_angle=math.radians(100),
        intake_enabled=True,
        claw_scoring=False,
        claw_picking=True,
        cube_picking=True,
    ),
    "cube_intake_off": TargetData(
        target_pose=None,
        arm_angle=math.radians(74.5),
        arm_length=0 * units.SI.inches_to_meters,
        wrist_angle=math.radians(100),
        intake_enabled=True,
        claw_scoring=False,
        claw_picking=True,
        cube_picking=True,
        intake_off=True,
    ),
    "cube_intake_no_grab": TargetData(
        target_pose=None,
        arm_angle=math.radians(0),
        arm_length=0 * units.SI.inches_to_meters,
        wrist_angle=math.radians(0),
        intake_enabled=True,
        claw_scoring=False,
        claw_picking=False,
        cube_picking=False,
        grabber_no_grab=False,
        no_intake=False,
    ),
    "cube_intake_auto_but_slightly_higher": TargetData(
        target_pose=None,
        arm_angle=math.radians(54.5),
        arm_length=0 * units.SI.inches_to_meters,
        wrist_angle=math.radians(100),
        intake_enabled=True,
        claw_scoring=False,
        claw_picking=True,
    ),
    "cube_intake_auto": TargetData(
        target_pose=None,
        arm_angle=math.radians(74.5),
        arm_length=1 * units.SI.inches_to_meters,
        wrist_angle=math.radians(100),
        intake_enabled=True,
        claw_scoring=False,
        claw_picking=True,
    ),
    "cube_intake_auto_2": TargetData(
        target_pose=None,
        arm_angle=math.radians(74.5),
        arm_length=0 * units.SI.inches_to_meters,
        wrist_angle=math.radians(100),
        intake_enabled=True,
        claw_scoring=False,
        claw_picking=True,
        intake_off=True,
    ),
    "standard": TargetData(
        target_pose=None,
        arm_angle=math.radians(0),
        arm_length=0,
        wrist_angle=math.radians(0),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=False,
    ),
    "standard_pickup": TargetData(
        target_pose=None,
        arm_angle=math.radians(0),
        arm_length=0,
        wrist_angle=math.radians(0),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=False,
        claw_wait=True,
    ),
    "eject": TargetData(
        target_pose=None,
        arm_angle=math.radians(-45),
        arm_length=0,
        wrist_angle=math.radians(0),
        intake_enabled=True,
        claw_scoring=False,
        claw_picking=False,
        claw_wait=False,
        intake_reversed=True,
    ),
}
