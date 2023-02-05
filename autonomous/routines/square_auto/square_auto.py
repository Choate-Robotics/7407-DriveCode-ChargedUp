import math

from commands2 import SequentialCommandGroup, InstantCommand
from wpimath.geometry import Pose2d

import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.utils.custom_pathing import FollowPathCustom, RotateInPlace
from autonomous.utils.trajectory import CustomTrajectory
from robot_systems import Robot
from utils import logger

max_vel = 1
max_accel = 2

field_length = 2.896 - constants.robot_length
field_width = 2.629 - constants.robot_length

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(0, 0, math.radians(90)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(field_length, 0, math.radians(90)),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=0,
        end_velocity=max_vel,
    ),
    period=constants.period,
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(field_length, 0, math.radians(90)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(field_length, field_width, math.radians(180)),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=max_vel,
        end_velocity=max_vel,
    ),
    period=constants.period,
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(field_length, field_width, math.radians(180)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(0, field_width, math.radians(270)),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=max_vel,
        end_velocity=max_vel,
    ),
    period=constants.period,
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(0, field_width, math.radians(270)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(0, 0, math.radians(0)),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=max_vel,
        end_velocity=max_vel,
    ),
    period=constants.period,
)

path_5 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(0, 0, math.radians(0)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(0, field_width, math.radians(0)),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=max_vel,
        end_velocity=max_vel,
    ),
    period=constants.period,
)

path_6 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(0, field_width, math.radians(0)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(field_length, field_width, math.radians(270)),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=max_vel,
        end_velocity=max_vel,
    ),
    period=constants.period,
)

path_7 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(field_length, field_width, math.radians(270)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(field_length, 0, math.radians(180)),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=max_vel,
        end_velocity=max_vel,
    ),
    period=constants.period,
)

path_8 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(field_length, 0, math.radians(180)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(0, 0, math.radians(90)),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=max_vel,
        end_velocity=max_vel,
    ),
    period=constants.period,
)

auto = SequentialCommandGroup(
    InstantCommand(lambda: logger.debug("AUTONOMOUS", "Starting autonomous")),
    path_1,
    path_2,
    path_3,
    path_4,
    path_5,
    path_6,
    path_7,
    path_8,
    RotateInPlace(
        subsystem=Robot.drivetrain,
        theta_f=math.radians(0),
        period=constants.period,
        threshold=math.radians(0.6),
        max_angular_vel=math.radians(90),
    ),
)

routine = AutoRoutine(Pose2d(0, 0, math.radians(270)), auto)
