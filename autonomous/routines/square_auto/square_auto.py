import math

from commands2 import SequentialCommandGroup, WaitCommand
from wpimath.geometry import Pose2d

import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.utils.custom_pathing import FollowPathCustom, RotateInPlace
from autonomous.utils.trajectory import CustomTrajectory
from robot_systems import Robot

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(0, 0, math.radians(0)),
        waypoints=[
            # Translation2d(.2, .2),
            # Translation2d(.3, .3),
            # Translation2d(1, 0),
        ],
        end_pose=Pose2d(1, 0, math.radians(90)),
        max_velocity=1,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
    ),
    period=constants.period,
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(1, 0, math.radians(90)),
        waypoints=[],
        end_pose=Pose2d(1, 1, math.radians(180)),
        max_velocity=1,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
    ),
    period=constants.period,
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(1, 1, math.radians(180)),
        waypoints=[],
        end_pose=Pose2d(0, 1, math.radians(270)),
        max_velocity=1,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
    ),
    period=constants.period,
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(0, 1, math.radians(270)),
        waypoints=[],
        end_pose=Pose2d(0, 0, math.radians(0)),
        max_velocity=1,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
    ),
    period=constants.period,
)

path_5 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(0, 0, math.radians(0)),
        waypoints=[],
        end_pose=Pose2d(0, 1, math.radians(0)),
        max_velocity=1,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
    ),
    period=constants.period,
)

path_6 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(0, 1, math.radians(0)),
        waypoints=[],
        end_pose=Pose2d(1, 1, math.radians(270)),
        max_velocity=1,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
    ),
    period=constants.period,
)

path_7 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(1, 1, math.radians(270)),
        waypoints=[],
        end_pose=Pose2d(1, 0, math.radians(180)),
        max_velocity=1,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
    ),
    period=constants.period,
)

path_8 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(1, 0, math.radians(180)),
        waypoints=[],
        end_pose=Pose2d(0, 0, math.radians(90)),
        max_velocity=1,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
    ),
    period=constants.period,
)

auto = SequentialCommandGroup(
    path_1,
    WaitCommand(0.1),
    path_2,
    WaitCommand(0.1),
    path_3,
    WaitCommand(0.1),
    path_4,
    WaitCommand(0.1),
    path_5,
    WaitCommand(0.1),
    path_6,
    WaitCommand(0.1),
    path_7,
    WaitCommand(0.1),
    path_8,
    WaitCommand(0.1),
    RotateInPlace(
        subsystem=Robot.drivetrain,
        theta_f=math.radians(0),
        period=constants.period,
        threshold=math.radians(0.6),
        max_angular_vel=math.radians(90),
    ),
)

routine = AutoRoutine(Pose2d(0, 0, math.radians(0)), auto)
