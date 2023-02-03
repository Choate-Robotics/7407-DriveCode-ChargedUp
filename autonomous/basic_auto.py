import math

from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from wpimath.geometry import Pose2d, Translation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, TrajectoryUtil

import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.custom_pathing import FollowPathCustom
from robot_systems import Robot

from autonomous.combine_trajectory_json import combine_trajectory_json

routines = ["routine_1", "routine_2"]

combine_trajectory_json(routines)

# assuming this file is for routine 1
trajectory_1 = TrajectoryUtil.fromPathweaverJson("./autonomous/routines/routine_1/auto_routine.json")

path_1 = FollowPathCustom(
    Robot.drivetrain,
    trajectory_1,
    math.radians(0),
    constants.period,
)

trajectory_config_2 = TrajectoryConfig(1, 2)
trajectory_config_2.setStartVelocity(0)
trajectory_config_2.setEndVelocity(0)

trajectory_2 = TrajectoryGenerator.generateTrajectory(
    start=Pose2d(2, 2, math.radians(0)),
    interiorWaypoints=[
        Translation2d(1.5, 1.5),
        Translation2d(1, 1),
        Translation2d(0.5, 0.5),
    ],
    end=Pose2d(0, 0, 0),
    config=trajectory_config_2,
)

path_2 = FollowPathCustom(
    Robot.drivetrain,
    trajectory_2,
    math.radians(0),
    constants.period,
)

auto = SequentialCommandGroup(
    InstantCommand(lambda: print(Robot.drivetrain.odometry.getPose())),
    path_1,
    InstantCommand(lambda: print(Robot.drivetrain.odometry.getPose())),
    WaitCommand(2),
    path_2,
    InstantCommand(lambda: print(Robot.drivetrain.odometry.getPose())),
)

routine = AutoRoutine(Pose2d(0, 0, math.radians(0)), auto)