import math
import json
import tempfile

from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from wpimath.geometry import Pose2d, Translation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, TrajectoryUtil

import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.custom_pathing import FollowPathCustom
from robot_systems import Robot

from autonomous.trajectory import combine_trajectory_json, get_trajectories

# combines all trajectories in each routine into one auto_routine.json file
routines = ["routine_1", "routine_2"]
combine_trajectory_json(routines)

# assuming this file is for routine_1, store all trajectories in a dict
trajectories = get_trajectories("routine_1")

path_1 = FollowPathCustom(
    Robot.drivetrain,
    trajectories["trajectory_1"],
    math.radians(0),
    constants.period,
)

path_2 = FollowPathCustom(
    Robot.drivetrain,
    trajectories["trajectory_2"],
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
