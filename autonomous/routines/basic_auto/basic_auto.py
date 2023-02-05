import math

from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from wpimath.geometry import Pose2d

import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.utils.custom_pathing import FollowPathCustom
from autonomous.utils.path_planner import generate_trajectories
from robot_systems import Robot

max_vel = 1
max_accel = 2

configs = {
    "trajectory_1": {
        "max_vel": max_vel,
        "max_accel": max_accel,
        "start_vel": 0,
        "end_vel": max_vel
    },
    "trajectory_2": {
        "max_vel": max_vel,
        "max_accel": max_accel,
        "start_vel": max_vel,
        "end_vel": max_vel
    },
    "trajectory_3": {
        "max_vel": max_vel,
        "max_accel": max_accel,
        "start_vel": max_vel,
        "end_vel": max_vel
    },
    "trajectory_4": {
        "max_vel": max_vel,
        "max_accel": max_accel,
        "start_vel": max_vel,
        "end_vel": 0
    }
}

trajectories = generate_trajectories(configs)

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=trajectories["trajectory_1"],
    period=constants.period,
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=trajectories["trajectory_2"],
    period=constants.period,
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=trajectories["trajectory_3"],
    period=constants.period,
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=trajectories["trajectory_4"],
    period=constants.period,
)

auto = SequentialCommandGroup(
    path_1,
    path_2,
    path_3,
    path_4
)

routine = AutoRoutine(Pose2d(0, 0, math.radians(270)), auto)