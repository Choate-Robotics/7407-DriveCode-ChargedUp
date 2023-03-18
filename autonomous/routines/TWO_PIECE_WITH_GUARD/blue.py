import math

from commands2 import (
    InstantCommand,
    ParallelDeadlineGroup,
    SequentialCommandGroup,
    WaitCommand,
)
from wpimath.geometry import Pose2d, Translation2d

import command
import config
import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.routines.TWO_PIECE_WITH_GUARD.base_coords import (
    base_initial_coords,
    base_path_1,
    base_path_2,
)
from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot, Sensors
from units.SI import meters, meters_per_second, meters_per_second_squared, radians

max_vel: meters_per_second = 4
max_accel: meters_per_second_squared = 3

initial_x: meters = base_initial_coords[0] + 0
initial_y: meters = base_initial_coords[1] + 0
initial_theta: radians = base_initial_coords[2] + 0

path_1_end_x: meters = base_path_1[1][0] + 0
path_1_end_y: meters = base_path_1[1][1] + 0
path_1_end_theta: radians = base_path_1[1][2] + 0

path_2_end_x: meters = base_path_2[1][0] + 0
path_2_end_y: meters = base_path_2[1][1] + 0
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
        waypoints=[Translation2d(3.92, 0.92)],
        end_pose=Pose2d(path_2_end_x, path_2_end_y, path_2_end_theta),
        max_velocity=2.7,
        max_accel=1.7,
        start_velocity=0,
        end_velocity=0,
    ),
    period=constants.period,
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(path_2_end_x, path_2_end_y, path_2_end_theta),
        waypoints=[Translation2d(5.12, 1.22), Translation2d(4.96, 2.3)],
        end_pose=Pose2d(6.32, 2.3, 0),
        max_velocity=2.7,
        max_accel=1.7,
        start_velocity=0,
        end_velocity=0,
    ),
    period=constants.period,
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(6.32, 2.3, 0),
        waypoints=[Translation2d(5.12, 1.22), Translation2d(3.92, 0.92)],
        end_pose=Pose2d(path_2_end_x, path_2_end_y, path_2_end_theta),
        max_velocity=3.6,
        max_accel=2.5,
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
        deadline=WaitCommand(0.8),
        commands=[
            command.TargetAuto(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["high_auto_back_intake"],
            ).generate()
        ],
    ),
    command.SetGrabber(Robot.grabber, wrist_angle=math.radians(-25), claw_active=False),
    InstantCommand(lambda: Robot.grabber.open_claw()),
    WaitCommand(0.1),
    ParallelDeadlineGroup(
        deadline=SequentialCommandGroup(path_1, WaitCommand(0)),
        commands=[
            SequentialCommandGroup(
                command.TargetAuto(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["cube_intake_auto"],
                ).generate(),
            )
        ],
    ),
    InstantCommand(lambda: Robot.grabber.close_claw()),
    WaitCommand(0.1),
    ParallelDeadlineGroup(
        deadline=path_2,
        commands=[
            command.TargetAuto(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["high_auto_back_cube"],
            ).generate(),
        ],
    ),
    command.IntakeEnable(Robot.intake),
    InstantCommand(lambda: Robot.grabber.open_claw()),
    InstantCommand(lambda: Robot.grabber.set_output(-0.3)),
    WaitCommand(0.3),
    ParallelDeadlineGroup(
        deadline=SequentialCommandGroup(path_3, WaitCommand(0)),
        commands=[
            SequentialCommandGroup(
                command.TargetAuto(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["cube_intake_auto"],
                ).generate(),
            )
        ],
    ),
    InstantCommand(lambda: Robot.grabber.close_claw()),
    WaitCommand(0.1),
    ParallelDeadlineGroup(
        deadline=path_4,
        commands=[
            SequentialCommandGroup(
                WaitCommand(1),
                command.TargetAuto(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["mid_auto_back_cube"],
                ).generate(),
            )
        ],
    ),
    InstantCommand(lambda: Robot.grabber.open_claw()),
    InstantCommand(lambda: Robot.grabber.set_output(-0.3)),
    WaitCommand(0.3),
    command.TargetAuto(
        Robot.arm,
        Robot.grabber,
        Robot.intake,
        Sensors.odometry,
        target=config.scoring_locations["standard"],
    ).generate(),
)

routine = AutoRoutine(Pose2d(initial_x, initial_y, initial_theta), auto)
