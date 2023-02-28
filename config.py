import math
from dataclasses import dataclass

from wpimath.geometry import Pose2d, Translation2d

from units.SI import meters, meters_per_second, meters_per_second_squared, radians

red_team: bool = False

drivetrain_scoring_velocity = 1
drivetrain_scoring_angular_velocity = 1
drivetrain_routing_velocity = 2
drivetrain_routing_acceleration = 1
drivetrain_routing_angular_velocity = 3
current_scoring_location = "double_station"


@dataclass
class TargetData:
    arm_angle: radians
    arm_length: meters
    wrist_angle: radians
    target_pose: Pose2d | None
    target_waypoints: list[Translation2d] = None
    intake_enabled: bool = False
    intake_reversed: bool = False

    claw_picking: bool = False
    cube_picking: bool = False

    claw_scoring: bool = False
    arm_scoring: bool = False

    claw_wait: bool = False

    max_velocity: meters_per_second = None
    max_acceleration: meters_per_second_squared = None
    max_angular_velocity: meters_per_second = None


elevator_motor_extend_id = 17
elevator_secondary_rotation_motor_id = 1
elevator_main_rotation_motor_id = 2
elevator_brake_id = 3
# converted to radians in subsystems/elevator.py

pneumatics_control_module = 31

claw_motor_speed: float = 0.7

# Intake
intake_motor_id = 11
intake_piston_forwardChannel = 4
intake_piston_reverseChannel = 5

default_intake_speed = 0.5

kRobotVisionPoseWeight = 0.1
# Dummy data
claw_motor_extend_id = 0

# SCORING LOCATIONS
scoring_locations: dict[str, TargetData] = {
    "low": TargetData(
        target_pose=Pose2d(0, 0, 0),
        arm_angle=math.radians(0),
        arm_length=0,
        wrist_angle=math.radians(0),
        intake_enabled=False,
        claw_scoring=True,
        claw_picking=False,
        arm_scoring=True,
    ),
    "middle": TargetData(
        target_pose=Pose2d(1.55, 1.55, 0),  # 2.43 .94
        target_waypoints=[Translation2d(1.81, 1.55)],
        arm_angle=math.radians(-46.78),
        arm_length=0.55,
        wrist_angle=math.radians(-27.09),
        intake_enabled=False,
        claw_scoring=True,
        claw_picking=False,
        arm_scoring=True,
        max_velocity=1,
        max_acceleration=0.5,
        max_angular_velocity=1,
    ),
    "high": TargetData(
        target_pose=None,
        arm_angle=math.radians(-49.7),
        arm_length=1.03,
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
        arm_length=1.03,
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
    "pickup": TargetData(
        target_pose=None,
        arm_angle=math.radians(-108),
        arm_length=0.199,
        wrist_angle=math.radians(0),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=True,
    ),
    "double_station": TargetData(
        target_pose=Pose2d(16, 7.51, math.radians(0)),
        arm_angle=math.radians(23.66),
        arm_length=0.29,
        wrist_angle=math.radians(83.37),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=True,
    ),
    "cube_intake": TargetData(
        target_pose=None,
        arm_angle=math.radians(76.5),
        arm_length=0,
        wrist_angle=math.radians(94.8),
        intake_enabled=True,
        claw_scoring=False,
        claw_picking=True,
        cube_picking=True,
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
