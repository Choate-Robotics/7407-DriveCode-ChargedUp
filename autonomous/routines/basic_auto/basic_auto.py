import math

from commands2 import (
    InstantCommand,
    ParallelCommandGroup,
    ParallelDeadlineGroup,
    SequentialCommandGroup,
    WaitCommand,
)
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d

import command
import config
import constants
from autonomous.auto_routine import AutoRoutine
from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot, Sensors
from units.SI import meters, meters_per_second, meters_per_second_squared

max_vel: meters_per_second = 2
max_accel: meters_per_second_squared = 4

initial_x: meters = 1.55
initial_y: meters = 2.18

field_length: meters = (2.896 - constants.robot_length) * 2
field_width: meters = (2.629 - constants.robot_length) * 2

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(initial_x, initial_y, math.radians(0)),
        waypoints=[],
        end_pose=Pose2d(initial_x + 2, initial_y, math.radians(0)),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=0,
        end_velocity=max_vel,
    ),
    period=constants.period,
)

auto = SequentialCommandGroup(
    command.ZeroElevator(Robot.arm),
    command.ZeroShoulder(Robot.arm),
    command.ZeroWrist(Robot.grabber),
    InstantCommand(lambda: SmartDashboard.putBoolean("AUTO", False)),
    InstantCommand(lambda: SmartDashboard.putBoolean("GRABBER", False)),
    InstantCommand(lambda: SmartDashboard.putBoolean("CLAW", False)),
    InstantCommand(lambda: SmartDashboard.putBoolean("BAL", False)),
    InstantCommand(lambda: SmartDashboard.putBoolean("AUTO", True)),
    ParallelDeadlineGroup(
        deadline=WaitCommand(1.15),
        commands=[
            command.TargetAuto(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["high_auto_back"],
            ).generate()
        ],
    ),
    InstantCommand(lambda: SmartDashboard.putBoolean("GRABBER", True)),
    InstantCommand(lambda: SmartDashboard.putBoolean("CLAW", True)),
    InstantCommand(lambda: Robot.grabber.open_claw()),
    WaitCommand(0.3),
    ParallelCommandGroup(
        command.autonomous.custom_pathing.AutoBalance(
            Robot.drivetrain, 1, 0, 0
        ).andThen(InstantCommand(lambda: SmartDashboard.putBoolean("BAL", True))),
        command.TargetAuto(
            Robot.arm,
            Robot.grabber,
            Robot.intake,
            Sensors.odometry,
            target=config.scoring_locations["middle_auto_back"],
        ).generate(),
    ),
    InstantCommand(lambda: SmartDashboard.putBoolean("AUTO", False)),
)

routine = AutoRoutine(Pose2d(initial_x, initial_y, math.radians(0)), auto)
