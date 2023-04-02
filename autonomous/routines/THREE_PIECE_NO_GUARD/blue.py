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
from autonomous.routines.THREE_PIECE_NO_GUARD.blue_base_coords import (
    base_initial_coords,
    base_path_1,
    base_path_2,
    base_path_3,
    base_path_4,
    blue_team,
)
from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot, Sensors
from units.SI import meters_per_second, meters_per_second_squared

max_vel: meters_per_second = 4
max_accel: meters_per_second_squared = 3

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*base_path_1[0]),
        waypoints=[Translation2d(*x) for x in base_path_2[1]],
        end_pose=Pose2d(*base_path_1[2]),
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
        start_pose=Pose2d(*base_path_2[0]),
        waypoints=[Translation2d(*x) for x in base_path_2[1]],
        end_pose=Pose2d(*base_path_2[2]),
        max_velocity=3,
        max_accel=2,
        start_velocity=0,
        end_velocity=0,
    ),
    period=constants.period,
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*base_path_3[0]),
        waypoints=[Translation2d(*x) for x in base_path_3[1]],
        end_pose=Pose2d(*base_path_3[2]),
        max_velocity=3,
        max_accel=1.7,
        start_velocity=0,
        end_velocity=0,
    ),
    period=constants.period,
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*base_path_4[0]),
        waypoints=[Translation2d(*x) for x in base_path_4[1]],
        end_pose=Pose2d(*base_path_4[2]),
        max_velocity=3.5,
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
    command.IntakeEnable(Robot.intake),
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
            command.TargetAuto(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["cube_intake_auto"],
            ).generate(),
        ],
    ),
    ParallelDeadlineGroup(
        deadline=path_2,
        commands=[
            SequentialCommandGroup(
                WaitCommand(0.5),
                InstantCommand(lambda: Robot.grabber.close_claw()),
            ),
            SequentialCommandGroup(
                WaitCommand(1.3),
                command.TargetAuto(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["high_auto_back_cube"],
                ).generate(),
            ),
        ],
    ),
    InstantCommand(lambda: Robot.grabber.set_output(-0.3)),
    InstantCommand(lambda: Robot.grabber.open_claw()),
    WaitCommand(0.25),
    ParallelDeadlineGroup(
        deadline=SequentialCommandGroup(path_3, WaitCommand(0)),
        commands=[
            SequentialCommandGroup(
                ParallelDeadlineGroup(
                    deadline=WaitCommand(0.8),
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
                command.IntakeEnable(Robot.intake),
                WaitCommand(0.7),
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
                WaitCommand(1.6),
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
    InstantCommand(lambda: Robot.grabber.set_output(-0.3)),
    InstantCommand(lambda: Robot.grabber.open_claw()),
    WaitCommand(0.35),
    command.TargetAuto(
        Robot.arm,
        Robot.grabber,
        Robot.intake,
        Sensors.odometry,
        target=config.scoring_locations["standard"],
    ).generate(),
)

routine = AutoRoutine(Pose2d(*base_initial_coords), auto, blue_team=blue_team)
