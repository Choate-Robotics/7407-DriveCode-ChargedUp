import math

from commands2 import (
    InstantCommand,
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
        start_pose=Pose2d(4.88, 2.38, math.radians(0)),
        waypoints=[],
        end_pose=Pose2d(6.32, 2.5, math.radians(0)),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=0,
        end_velocity=max_vel,
    ),
    period=constants.period,
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(6.32, 2.5, math.radians(0)),
        waypoints=[],
        end_pose=Pose2d(4.88, 2.38, math.radians(0)),
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
        deadline=WaitCommand(1.4),
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
    ParallelDeadlineGroup(
        deadline=command.autonomous.custom_pathing.DriveOverChargeStation(
            Robot.drivetrain, 1, 0, 0, times_before_stop=2
        ).andThen(InstantCommand(lambda: SmartDashboard.putBoolean("BAL", True))),
        commands=[
            command.TargetAuto(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["middle_auto_back"],
            ).generate()
        ],
    ),
    ParallelDeadlineGroup(
        deadline=WaitCommand(0.5),
        commands=[
            command.TargetAuto(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["standard"],
            ).generate()
        ],
    ),
    ParallelDeadlineGroup(
        deadline=WaitCommand(1.5),
        commands=[
            path_1,
            command.TargetAuto(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["cube_intake"],
            ).generate(),
        ],
    ),
    InstantCommand(lambda: Robot.grabber.close_claw()),
    ParallelDeadlineGroup(
        deadline=WaitCommand(1.5),
        commands=[
            path_2,
            command.TargetAuto(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["middle_auto_front"],
            ).generate(),
        ],
    ),
    command.autonomous.custom_pathing.AutoBalance(
        Robot.drivetrain, -1, 0, 0, times_before_stop=1
    ),
    InstantCommand(lambda: SmartDashboard.putBoolean("AUTO", False)),
)

routine = AutoRoutine(Pose2d(initial_x, initial_y, math.radians(0)), auto)
