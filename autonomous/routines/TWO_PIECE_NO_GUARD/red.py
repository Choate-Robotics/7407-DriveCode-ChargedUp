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
from autonomous.routines.TWO_PIECE_NO_GUARD.base_coords import (
    base_initial_coords,
    base_path_1,
    base_path_2,
)
from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot, Sensors
from units.SI import meters, meters_per_second, meters_per_second_squared, radians

max_vel: meters_per_second = 2
max_accel: meters_per_second_squared = 4

initial_x: meters = base_initial_coords[0] + 0
initial_y: meters = config.field_width - base_initial_coords[1] + 0
initial_theta: radians = base_initial_coords[2] + 0

path_1_end_x: meters = base_path_1[1][0] + 0
path_1_end_y: meters = config.field_width - base_path_1[1][1] + 0
path_1_end_theta: radians = base_path_1[1][2] + 0

path_2_end_x: meters = base_path_2[1][0] + 0
path_2_end_y: meters = config.field_width - base_path_2[1][1] + 0
path_2_end_theta: radians = base_path_2[1][2] + 0

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(initial_x, initial_y, math.radians(0)),
        waypoints=[],
        end_pose=Pose2d(path_1_end_x, path_1_end_y, path_1_end_theta),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=0,
        end_velocity=0,
    ),
    period=constants.period,
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(path_1_end_x, path_1_end_y, path_1_end_theta),
        waypoints=[],
        end_pose=Pose2d(path_2_end_x, path_2_end_y, path_2_end_theta),
        max_velocity=1.5,
        max_accel=1,
        start_velocity=0,
        end_velocity=0,
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
    command.SetGrabber(Robot.grabber, wrist_angle=math.radians(-25), claw_active=False),
    InstantCommand(lambda: Robot.grabber.open_claw()),
    WaitCommand(0.3),
    # InstantCommand(lambda: Robot.grabber.close_claw()),
    InstantCommand(lambda: Robot.grabber.open_claw()),
    ParallelDeadlineGroup(
        deadline=WaitCommand(3.5),
        commands=[
            InstantCommand(lambda: Robot.grabber.open_claw()),
            path_1,
            command.TargetAuto(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["cube_intake_auto"],
            ).generate(),
        ],
    ),
    InstantCommand(lambda: Robot.grabber.close_claw()),
    WaitCommand(0.5),
    ParallelDeadlineGroup(
        deadline=WaitCommand(5),
        commands=[
            path_2,
            command.TargetAuto(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["standard"],
            ).generate(),
        ],
    ),
    command.IntakeDisable(Robot.intake),
    WaitCommand(0.3),
    ParallelDeadlineGroup(
        deadline=WaitCommand(1.5),
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
        deadline=WaitCommand(1.5),
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
)

routine = AutoRoutine(
    Pose2d(initial_x, initial_y, initial_theta), auto, blue_team=False
)