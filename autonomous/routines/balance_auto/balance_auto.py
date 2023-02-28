import math

from commands2 import InstantCommand, SequentialCommandGroup
from wpimath.geometry import Pose2d

import constants
from autonomous.auto_routine import AutoRoutine
from command.autonomous.custom_pathing import DriveOverChargeStation, FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot
from units.SI import meters_per_second, meters_per_second_squared
from utils import logger

max_vel: meters_per_second = 1
max_accel: meters_per_second_squared = 2

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(14.92, 3.35, math.radians(0)),
        waypoints=[],
        end_pose=Pose2d(12.23, 3.37, math.radians(0)),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=0,
        end_velocity=max_vel,
    ),
    period=constants.period,
)

auto = SequentialCommandGroup(
    InstantCommand(lambda: logger.debug("AUTONOMOUS", "Starting autonomous")),
    DriveOverChargeStation(Robot.drivetrain, -1, 0, 0, -10),
)

routine = AutoRoutine(Pose2d(14.92, 3.35, math.radians(0)), auto)
