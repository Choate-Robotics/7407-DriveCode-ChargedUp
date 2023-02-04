import math

from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from wpimath.geometry import Pose2d

import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.utils.custom_pathing import FollowPathCustom
from autonomous.utils.path_planner import generate_trajectories
from robot_systems import Robot

trajectories = generate_trajectories(1, 2)

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=trajectories[0],
    period=constants.period,
)

auto = SequentialCommandGroup(
    path_1
)

routine = AutoRoutine(Pose2d(0, 0, math.radians(0)), auto)