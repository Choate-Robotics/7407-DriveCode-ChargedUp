import math

from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from wpimath.geometry import Pose2d

import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.custom_pathing import FollowPathCustom
from autonomous.trajectory import CustomTrajectory
from robot_systems import Robot

trajectory_1 = CustomTrajectory(
    start_pose=Pose2d(0, 0, math.radians(0)),
    waypoints=[
        # Translation2d(.2, .2),
        # Translation2d(.3, .3),
        # Translation2d(.4, .4),
    ],
    end_pose=Pose2d(.5, .5, math.radians(90)),
    max_velocity=.5,
    max_accel=1,
    start_velocity=0,
    end_velocity=0,
)

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=trajectory_1,
    period=constants.period,
)

trajectory_2 = CustomTrajectory(
    start_pose=Pose2d(.5, .5, math.radians(90)),
    waypoints=[
        # Translation2d(.4, .4),
        # Translation2d(.3, .3),
        # Translation2d(.2, .2),
    ],
    end_pose=Pose2d(0, 0, math.radians(0)),
    max_velocity=.5,
    max_accel=1,
    start_velocity=0,
    end_velocity=0,
)

path_2 = FollowPathCustom(
    Robot.drivetrain,
    trajectory_2,
    constants.period,
)

auto = SequentialCommandGroup(
    InstantCommand(lambda: print("POSE START: ", Robot.drivetrain.odometry.getPose())),
    path_1,
    InstantCommand(lambda: print("POSE MIDDLE: ", Robot.drivetrain.odometry.getPose())),
    WaitCommand(2),
    path_2,
    InstantCommand(lambda: print("POSE END: ", Robot.drivetrain.odometry.getPose())),
)

routine = AutoRoutine(Pose2d(0, 0, math.radians(0)), auto)
