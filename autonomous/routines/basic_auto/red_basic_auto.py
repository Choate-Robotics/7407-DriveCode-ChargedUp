import math

from commands2 import (
    InstantCommand,
    ParallelDeadlineGroup,
    SequentialCommandGroup,
    WaitCommand,
)
from wpimath.geometry import Pose2d

import command
import config
import constants
from autonomous.auto_routine import AutoRoutine
from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot, Sensors
from units.SI import meters, meters_per_second, meters_per_second_squared, radians

max_vel: meters_per_second = 2
max_accel: meters_per_second_squared = 4

initial_x: meters = 1.55
initial_y: meters = config.field_width - 2.18
initial_theta: radians = math.radians(0)

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(4.88, config.field_width - 2.38, math.radians(0)),
        waypoints=[],
        end_pose=Pose2d(6.32, config.field_width - 2.5, math.radians(0)),
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
        start_pose=Pose2d(6.32, config.field_width - 2.5, math.radians(0)),
        waypoints=[],
        end_pose=Pose2d(4.88, config.field_width - 2.38, math.radians(0)),
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
    InstantCommand(lambda: Robot.grabber.open_claw()),
    WaitCommand(0.3),
    ParallelDeadlineGroup(
        deadline=command.autonomous.custom_pathing.DriveOverChargeStation(
            Robot.drivetrain, 1, 0, 0, times_before_stop=2
        ),
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
    command.IntakeDisable(Robot.intake),
    command.autonomous.custom_pathing.AutoBalance(
        Robot.drivetrain, -1, 0, 0, times_before_stop=1
    ),
)

routine = AutoRoutine(Pose2d(initial_x, initial_y, initial_theta), auto)
