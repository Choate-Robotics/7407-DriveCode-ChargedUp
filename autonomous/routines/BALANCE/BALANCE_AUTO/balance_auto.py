import math

from commands2 import (
    InstantCommand,
    ParallelDeadlineGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
    WaitCommand,
)
from wpimath.geometry import Pose2d

import command
import config
from autonomous.auto_routine import AutoRoutine
from command.autonomous.custom_pathing import FollowPathCustom
from robot_systems import Robot, Sensors
from units.SI import meters, meters_per_second, meters_per_second_squared, radians

max_vel: meters_per_second = 2
max_accel: meters_per_second_squared = 4

initial_x: meters = 1.55
initial_y: meters = 2.18
initial_theta: radians = math.radians(0)

auto = SequentialCommandGroup(
    command.DrivetrainZero(Robot.drivetrain),
    command.DrivetrainDock(Robot.drivetrain, False),
    command.DrivetrainEngage(Robot.drivetrain, False)
)

routine = AutoRoutine(Pose2d(initial_x, initial_y, initial_theta), auto)


