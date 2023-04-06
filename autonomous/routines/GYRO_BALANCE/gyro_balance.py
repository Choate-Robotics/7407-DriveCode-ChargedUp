import math

from commands2 import (
    InstantCommand,
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
    command.ZeroElevator(Robot.arm),
    command.ZeroShoulder(Robot.arm),
    command.ZeroWrist(Robot.grabber),
    ParallelRaceGroup(
        command.autonomous.custom_pathing.GyroBalance(
            Robot.drivetrain,
            vx=2,  # Initial velocity of drivetrain while balancing (m/s)
        ),
        command.TargetAuto(
            Robot.arm,
            Robot.grabber,
            Robot.intake,
            Sensors.odometry,
            target=config.scoring_locations["standard"],
        ).generate(),
        WaitCommand(7),
    ),
    InstantCommand(lambda: Robot.drivetrain.set_robot_centric((0, 0), 0)),
    InstantCommand(lambda: Robot.drivetrain.x_mode()),
)

routine = AutoRoutine(Pose2d(initial_x, initial_y, initial_theta), auto)
