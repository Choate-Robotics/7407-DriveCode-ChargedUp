import math

from commands2 import InstantCommand, SequentialCommandGroup
from wpimath.geometry import Pose2d

import constants
from autonomous.auto_routine import AutoRoutine
from command.autonomous.custom_pathing import FollowPathCustom, RotateInPlace
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot
from units.SI import meters, meters_per_second, meters_per_second_squared
from utils import logger

max_vel: meters_per_second = 2
max_accel: meters_per_second_squared = 4

initial_x: meters = 6.6
initial_y: meters = 1.2

field_length: meters = (2.896 - constants.robot_length) * 2
field_width: meters = (2.629 - constants.robot_length) * 2

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(initial_x, initial_y, math.radians(0)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(initial_x + field_length, initial_y, math.radians(90)),
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
        start_pose=Pose2d(initial_x + field_length, initial_y, math.radians(90)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(
            initial_x + field_length, initial_y + field_width, math.radians(180)
        ),
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
        start_pose=Pose2d(
            initial_x + field_length, initial_y + field_width, math.radians(180)
        ),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(initial_x, initial_y + field_width, math.radians(270)),
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
        start_pose=Pose2d(initial_x, initial_y + field_width, math.radians(270)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(initial_x, initial_y, math.radians(0)),
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
        start_pose=Pose2d(initial_x, initial_y, math.radians(0)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(initial_x, initial_y + field_width, math.radians(0)),
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
        start_pose=Pose2d(initial_x, initial_y + field_width, math.radians(0)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(
            initial_x + field_length, initial_y + field_width, math.radians(270)
        ),
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
        start_pose=Pose2d(
            initial_x + field_length, initial_y + field_width, math.radians(270)
        ),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(initial_x + field_length, initial_y, math.radians(180)),
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
        start_pose=Pose2d(initial_x + field_length, initial_y, math.radians(180)),
        waypoints=[
            # Translation2d(.5 * (2.896 - constants.robot_length), .5 * (2.629 - constants.robot_length)),
        ],
        end_pose=Pose2d(initial_x, initial_y, math.radians(90)),
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

routine = AutoRoutine(Pose2d(initial_x, initial_y, math.radians(0)), auto)
