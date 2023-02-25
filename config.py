import math
from dataclasses import dataclass

from wpimath.geometry import Pose2d, Translation2d

from units.SI import meters, meters_per_second, meters_per_second_squared, radians


@dataclass
class TargetData:
    arm_angle: radians
    arm_length: meters
    wrist_angle: radians
    target_pose: Pose2d | None
    target_waypoints: list[Translation2d] | None = None
    intake_enabled: bool = False
    intake_reversed: bool = False
    claw_picking: bool = False
    claw_scoring: bool = False
    arm_scoring: bool = False

    claw_wait: bool = False

    max_velocity: meters_per_second = None
    max_acceleration: meters_per_second_squared = None
    max_angular_velocity: meters_per_second = None


red_team: bool = False

elevator_motor_extend_id = 17
elevator_secondary_rotation_motor_id = 1
elevator_main_rotation_motor_id = 2
elevator_brake_id = 3
# converted to radians in subsystems/elevator.py

pneumatics_control_module = 31

claw_motor_speed: float = 0.4

# Intake
intake_motor_id = 11
intake_piston_forwardChannel = 4
intake_piston_reverseChannel = 5

default_intake_speed = 0.5

kRobotVisionPoseWeight = 0.1
# Dummy data
claw_motor_extend_id = 0

# SCORING LOCATIONS
scoring_locations = {
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
        arm_angle=math.radians(-44.78),
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
        arm_angle=math.radians(-47.7),
        arm_length=1.04,
        wrist_angle=math.radians(-18.61),
        intake_enabled=False,
        claw_scoring=True,
        claw_picking=False,
        arm_scoring=True,
        max_velocity=1,
        max_acceleration=0.5,
        max_angular_velocity=1,
    ),
    "pickup": TargetData(
        target_pose=None,
        arm_angle=math.radians(-100),
        arm_length=0.099,
        wrist_angle=math.radians(-50.53),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=True,
    ),
    "double_station": TargetData(
        target_pose=Pose2d(1, 1, 0),
        arm_angle=math.radians(-44.78),
        arm_length=0.55,
        wrist_angle=math.radians(-27.09),
        intake_enabled=False,
        claw_scoring=False,
        claw_picking=True,
    ),
    "cube_intake": TargetData(
        target_pose=None,
        arm_angle=math.radians(67.5),
        arm_length=0,
        wrist_angle=math.radians(94.8),
        intake_enabled=True,
        claw_scoring=False,
        claw_picking=True,
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
