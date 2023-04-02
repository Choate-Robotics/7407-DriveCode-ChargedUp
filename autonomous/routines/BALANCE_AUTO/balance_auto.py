import math

from commands2 import (
    InstantCommand,
    ParallelDeadlineGroup,
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
    command.SetGrabber(Robot.grabber, wrist_angle=math.radians(-25), claw_active=False),
    InstantCommand(lambda: Robot.grabber.open_claw()),
    WaitCommand(0.3),
    ParallelRaceGroup(
        command.autonomous.custom_pathing.AutoBalanceNew(
            Robot.drivetrain,
            vx=2,  # Initial velocity of drivetrain while balancing (m/s)
            vx2=0.6,  # Final velocity of drivetrain while balancing (m/s)
            omega=0,
            times_before_stop=1,
            gyro_threshold_2=0.17,  # Threshold for reducing speed of drivetrain (pitch in radians)
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
    # The reason this is same sign vel is that in the auto balance code the drivetrain is set to negative
    # InstantCommand(lambda: Robot.drivetrain.set_robot_centric((0.8, 0), 0)),
    # WaitCommand(
    #     1.2
    # ),  # TUNE THIS AT SE MASS (HOW LONG TO MOVE BACKWARDS FOR AFTER TIPPING)
    InstantCommand(lambda: Robot.drivetrain.set_robot_centric((0, 0), 0)),
    InstantCommand(lambda: Robot.drivetrain.x_mode()),
)

routine = AutoRoutine(Pose2d(initial_x, initial_y, initial_theta), auto)
