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
        "end_vel": 0
    }
}

trajectories = generate_trajectories(configs)

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=trajectories["trajectory_1"],
    period=constants.period,
)

auto = SequentialCommandGroup(
    path_1
)

routine = AutoRoutine(Pose2d(0, 0, math.radians(270)), auto)