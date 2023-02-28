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
import constants
from autonomous.auto_routine import AutoRoutine
from command.autonomous.custom_pathing import FollowPathCustom
from command.autonomous.trajectory import CustomTrajectory
from config import TargetData
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
        end_pose=Pose2d(
            initial_x + field_length, initial_y + field_width, math.radians(90)
        ),
        max_velocity=max_vel,
        max_accel=max_accel,
        start_velocity=0,
        end_velocity=max_vel,
    ),
    period=constants.period,
)

auto = SequentialCommandGroup(
    InstantCommand(lambda: SmartDashboard.putBoolean("AUTO", False)),
    InstantCommand(lambda: SmartDashboard.putBoolean("GRABBER", False)),
    InstantCommand(lambda: SmartDashboard.putBoolean("CLAW", False)),
    InstantCommand(lambda: SmartDashboard.putBoolean("AUTO", True)),
    ParallelDeadlineGroup(
        deadline=WaitCommand(2),
        commands=[
            command.TargetAuto(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                pose=Pose2d(initial_x, initial_y, 0),
                target=TargetData(
                    target_pose=None,
                    arm_angle=math.radians(-49.7),
                    arm_length=1.03,
                    wrist_angle=math.radians(25),
                    intake_enabled=False,
                    claw_scoring=True,
                    claw_picking=False,
                    arm_scoring=True,
                    max_velocity=1,
                    max_acceleration=0.5,
                    max_angular_velocity=1,
                    claw_wait=True,
                ),
            ).generate()
        ],
    ),
    InstantCommand(lambda: SmartDashboard.putBoolean("GRABBER", True)),
    command.SetGrabber(Robot.grabber, wrist_angle=math.radians(-50), claw_active=False),
    InstantCommand(lambda: SmartDashboard.putBoolean("CLAW", True)),
    InstantCommand(lambda: Robot.grabber.open_claw()),
)

routine = AutoRoutine(Pose2d(initial_x, initial_y, math.radians(0)), auto)
