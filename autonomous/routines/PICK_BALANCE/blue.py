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
from autonomous.routines.PICK_BALANCE.blue_base_coords import (
    base_initial_coords,
    base_path_1,
    base_path_2,
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
        max_velocity=2.5,
        max_accel=1.5,
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
        deadline=path_1,
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
    ParallelDeadlineGroup(
        deadline=path_2,
        commands=[
            command.TargetAuto(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["standard"],
            ).generate(),
        ],
    ),
    WaitCommand(0.25),
    ParallelDeadlineGroup(
        deadline=command.autonomous.custom_pathing.AutoBalance(
            Robot.drivetrain,
            vx=-2,  # Initial velocity of drivetrain while balancing (m/s)
            vx2=-0.8,  # Final velocity of drivetrain while balancing (m/s)
            omega=0,
            times_before_stop=1,
            gyro_threshold_2=0.195,  # Threshold for reducing speed of drivetrain (pitch in radians)
        ),
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
    # The reason this is same sign vel is that in the auto balance code the drivetrain is set to negative
    InstantCommand(lambda: Robot.drivetrain.set_robot_centric((-0.8, 0), 0)),
    WaitCommand(
        0.7
    ),  # TUNE THIS AT SE MASS (HOW LONG TO MOVE BACKWARDS FOR AFTER TIPPING)
    InstantCommand(lambda: Robot.drivetrain.set_robot_centric((0, 0), 0)),
    InstantCommand(lambda: Robot.drivetrain.x_mode()),
)

routine = AutoRoutine(Pose2d(*base_initial_coords), auto, blue_team=blue_team)
