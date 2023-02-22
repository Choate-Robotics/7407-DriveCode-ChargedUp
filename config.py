import math
from dataclasses import dataclass

from wpimath.geometry import Pose2d

from units.SI import meters, meters_per_second, meters_per_second_squared, radians


@dataclass
class TargetData:
    target_pose: Pose2d | None
    arm_angle: radians
    arm_length: meters
    wrist_angle: radians
    intake_enabled: bool = False
    claw_picking: bool = False
    claw_scoring: bool = False
    arm_scoring: bool = False

    claw_wait_time: float = 0

    max_velocity: meters_per_second = None
    max_acceleration: meters_per_second_squared = None


red_team: bool = True

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
        target_pose=Pose2d(10, 10, 0),
        arm_angle=math.radians(-44.78),
        arm_length=0.55,
        wrist_angle=math.radians(-27.09),
        intake_enabled=False,
        claw_scoring=True,
        claw_picking=False,
        arm_scoring=True,
    ),
    "high": TargetData(
        target_pose=Pose2d(1, 1, 0),
        arm_angle=math.radians(-47.7),
        arm_length=1.04,
        wrist_angle=math.radians(-18.61),
        intake_enabled=False,
        claw_scoring=True,
        claw_picking=False,
        arm_scoring=True,
    ),
    "pickup": TargetData(
        target_pose=None,
        arm_angle=math.radians(-100),
        arm_length=0.099,
        wrist_angle=math.radians(-20.53),
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
        claw_wait_time=1,
    ),
}
