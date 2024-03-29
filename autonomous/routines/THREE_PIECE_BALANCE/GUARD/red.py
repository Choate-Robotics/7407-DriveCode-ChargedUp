import math

from commands2 import (
    InstantCommand,
    ParallelCommandGroup,
    ParallelDeadlineGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
    WaitCommand,
)
from wpimath.geometry import Pose2d, Translation2d

import command
import config
import constants
from autonomous.auto_routine import AutoRoutine
from autonomous.routines.THREE_PIECE_BALANCE.GUARD.coords.red import (
    blue_team,
    come_back_with_first_cube,
    come_back_with_second_cube,
    go_get_first_cube,
    go_get_second_cube,
    initial,
)
from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Robot, Sensors
from units.SI import meters_per_second, meters_per_second_squared

max_vel: meters_per_second = 4.5
max_accel: meters_per_second_squared = 3.3

path_1 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*go_get_first_cube[0]),
        waypoints=[Translation2d(*x) for x in go_get_first_cube[1]],
        end_pose=Pose2d(*go_get_first_cube[2]),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=constants.period,
)

path_2 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*come_back_with_first_cube[0]),
        waypoints=[Translation2d(*x) for x in come_back_with_first_cube[1]],
        end_pose=Pose2d(*come_back_with_first_cube[2]),
        max_velocity=3.7,
        max_accel=2.3,
        start_velocity=0,
        end_velocity=0,
        rev=True,
    ),
    period=constants.period,
)

path_3 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*go_get_second_cube[0]),
        waypoints=[Translation2d(*x) for x in go_get_second_cube[1]],
        end_pose=Pose2d(*go_get_second_cube[2]),
        max_velocity=4,
        max_accel=2.5,
        start_velocity=0,
        end_velocity=0,
        rev=False,
    ),
    period=constants.period,
)

path_4 = FollowPathCustom(
    subsystem=Robot.drivetrain,
    trajectory=CustomTrajectory(
        start_pose=Pose2d(*come_back_with_second_cube[0]),
        waypoints=[Translation2d(*x) for x in come_back_with_second_cube[1]],
        end_pose=Pose2d(*come_back_with_second_cube[2]),
        max_velocity=3.6,
        max_accel=2.6,
        start_velocity=0,
        end_velocity=0,
        rev=True,
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
        deadline=path_1,
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
        deadline=SequentialCommandGroup(path_2, WaitCommand(0.5)),
        commands=[
            SequentialCommandGroup(
                WaitCommand(0.5),
                InstantCommand(lambda: Robot.grabber.disengage_claw()),
                InstantCommand(lambda: Robot.grabber.set_output(0)),
            ),
            SequentialCommandGroup(
                WaitCommand(2.1),
                ParallelCommandGroup(
                    command.SetArm(
                        Robot.arm,
                        config.scoring_locations["high_auto_back_cube"].arm_length,
                        config.scoring_locations["high_auto_back_cube"].arm_angle,
                    ),
                    SequentialCommandGroup(
                        WaitCommand(0.3),
                        command.SetGrabber(
                            Robot.grabber,
                            config.scoring_locations["high_auto_back_cube"].wrist_angle,
                            False,
                        ),
                    ),
                ),
            ),
        ],
    ),
    InstantCommand(lambda: Robot.grabber.set_output(-0.3)),
    InstantCommand(lambda: Robot.grabber.open_claw()),
    WaitCommand(0.25),
    ParallelDeadlineGroup(
        deadline=path_3,
        commands=[
            SequentialCommandGroup(
                ParallelDeadlineGroup(
                    deadline=WaitCommand(1.7),
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
                command.TargetAuto(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations[
                        "cube_intake_auto_but_slightly_higher"
                    ],
                ).generate(),
            )
        ],
    ),
    ParallelDeadlineGroup(
        deadline=SequentialCommandGroup(
            ParallelRaceGroup(
                command.autonomous.custom_pathing.GyroBalance(
                    Robot.drivetrain,
                    vx=2,  # Initial velocity of drivetrain while balancing (m/s)
                    rever=True,
                ),
                WaitCommand(5),
            )
        ),
        commands=[
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.intake.intake_motor.set_raw_output(0)),
                InstantCommand(lambda: Robot.intake.intake_piston.retract()),
                WaitCommand(0.4),
                InstantCommand(lambda: Robot.intake.intake_piston.extend()),
                InstantCommand(lambda: Robot.intake.intake_motor.set_raw_output(0.1)),
                WaitCommand(1),
                InstantCommand(lambda: Robot.intake.intake_motor.set_raw_output(0.1)),
                InstantCommand(lambda: Robot.grabber.disengage_claw()),
                InstantCommand(lambda: Robot.grabber.set_output(0)),
            ),
            SequentialCommandGroup(
                WaitCommand(0.6),
                ParallelDeadlineGroup(
                    deadline=WaitCommand(1.2),
                    commands=[
                        command.TargetAuto(
                            Robot.arm,
                            Robot.grabber,
                            Robot.intake,
                            Sensors.odometry,
                            target=config.scoring_locations["cube_intake_auto_2"],
                        ).generate()
                    ],
                ),
                ParallelDeadlineGroup(
                    deadline=WaitCommand(2),
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
            ),
        ],
    ),
    ParallelCommandGroup(
        command.TargetAuto(
            Robot.arm,
            Robot.grabber,
            Robot.intake,
            Sensors.odometry,
            target=config.scoring_locations["high_auto_back"],
        ).generate(),
        SequentialCommandGroup(
            WaitCommand(1), InstantCommand(lambda: Robot.grabber.open_claw())
        ),
    ),
)

routine = AutoRoutine(Pose2d(*initial), auto, blue_team=blue_team)
