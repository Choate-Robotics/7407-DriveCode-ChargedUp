import math
import time

import commands2
from commands2 import InstantCommand, SequentialCommandGroup, WaitCommand
from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveDrivetrain
from robotpy_toolkit_7407.utils.math import bounded_angle_diff, rotate_vector
from robotpy_toolkit_7407.utils.units import radians
from wpilib import SmartDashboard
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians

from command.autonomous.trajectory import CustomTrajectory
from robot_systems import Sensors
from subsystem import Drivetrain


class AutoBalance(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain, vx, vy, omega, gyro_threshold):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.vx = vx
        self.vy = vy
        self.omega = omega
        self.gyro_threshold = gyro_threshold
        self.times_tipped = 0
        self.currently_tipped = False

    def initialize(self) -> None:
        ...

    def execute(self) -> None:
        self.subsystem.set_driver_centric((-self.vx, -self.vy), self.omega)

    def isFinished(self) -> bool:
        if self.subsystem.gyro._gyro.getRawGyro()[1][1] < self.gyro_threshold:
            if not self.currently_tipped:
                self.times_tipped += 1
            self.currently_tipped = True
        else:
            self.currently_tipped = False

        return self.times_tipped > 1

    def end(self, interrupted: bool = False) -> None:
        if not interrupted:
            commands2.CommandScheduler.getInstance().schedule(
                command=SequentialCommandGroup(
                    InstantCommand(
                        lambda: self.subsystem.set_driver_centric((0.5, 0), 0)
                    ),
                    WaitCommand(0.3),
                    InstantCommand(
                        lambda: self.subsystem.set_driver_centric((0, 0), 0)
                    ),
                )
            )


class FollowPathCustom(SubsystemCommand[SwerveDrivetrain]):
    """
    Follows a path using a holonomic drive controller.

    :param subsystem: The subsystem to run this command on
    :type subsystem: SwerveDrivetrain
    :param trajectory: The trajectory to follow
    :type trajectory: CustomTrajectory
    :param period: The period of the controller, defaults to 0.02
    :type period: float, optional
    """

    def __init__(
        self,
        subsystem: SwerveDrivetrain,
        trajectory: CustomTrajectory,
        period: float = 0.02,
    ):
        super().__init__(subsystem)
        self.trajectory: Trajectory = trajectory.trajectory
        self.controller = HolonomicDriveController(
            PIDController(1, 0, 0, period),
            PIDController(1, 0, 0, period),
            ProfiledPIDControllerRadians(
                8,  # 7.3
                0,
                0.05,  # .005
                TrapezoidProfileRadians.Constraints(
                    subsystem.max_angular_vel, subsystem.max_angular_vel / 0.001  # .001
                ),
                period,
            ),
        )
        self.start_time = 0
        self.t = 0
        self.duration = self.trajectory.totalTime()
        self.theta_i: float | None = None
        self.theta_f: float | None = None
        self.theta_diff: float | None = None
        self.omega: float | None = None
        self.end_pose: Pose2d = trajectory.end_pose
        self.finished: bool = False

    def initialize(self) -> None:
        self.start_time = time.perf_counter()
        self.theta_i = Sensors.odometry.getPose().rotation().radians()
        self.theta_f = self.end_pose.rotation().radians()
        self.theta_diff = bounded_angle_diff(self.theta_i, self.theta_f)
        self.omega = self.theta_diff / self.duration
        self.finished = False

    def execute(self) -> None:
        self.t = time.perf_counter() - self.start_time
        if self.t > self.duration:
            self.t = self.duration
            self.finished = True
        goal = self.trajectory.sample(self.t)
        goal_theta = self.theta_i + self.omega * self.t
        speeds = self.controller.calculate(
            Sensors.odometry.getPose(), goal, Rotation2d(goal_theta)
        )

        vx, vy = rotate_vector(
            speeds.vx, speeds.vy, Sensors.odometry.getPose().rotation().radians()
        )

        self.subsystem.set_driver_centric((-vx, -vy), speeds.omega)

    def isFinished(self) -> bool:
        return self.finished

    def end(self, interrupted: bool) -> None:
        self.subsystem.set_driver_centric((0, 0), 0)
        SmartDashboard.putString("POSE", str(self.subsystem.odometry.getPose()))
        SmartDashboard.putString(
            "POSD", str(Sensors.odometry.getPose().rotation().degrees())
        )

    def runsWhenDisabled(self) -> bool:
        return False


class RotateInPlace(SubsystemCommand[SwerveDrivetrain]):
    """
    Rotates the robot in place.

    :param subsystem: The subsystem to run this command on
    :type subsystem: SwerveDrivetrain
    :param theta_f: The final angle in radians
    :type theta_f: radians
    :param threshold: The angle threshold for the controller, defaults to .1
    :type threshold: radians, optional
    :param period: The period of the controller, defaults to 0.02
    :type period: seconds, optional
    """

    def __init__(
        self,
        subsystem: SwerveDrivetrain,
        theta_f: radians,
        threshold: float = math.radians(1),
        max_angular_vel: float | None = None,
        period: float = 0.02,
    ):
        super().__init__(subsystem)

        max_angular_vel = max_angular_vel or subsystem.max_angular_vel

        self.controller = HolonomicDriveController(
            PIDController(1, 0, 0, period),
            PIDController(1, 0, 0, period),
            ProfiledPIDControllerRadians(
                4,
                0,
                0,
                TrapezoidProfileRadians.Constraints(
                    max_angular_vel, max_angular_vel / 0.001
                ),
                period,
            ),
        )
        self.theta_f = theta_f
        self.threshold = threshold

        self.theta_i: float | None = None
        self.theta_diff: float | None = None

    def initialize(self) -> None:
        print("DESIRED FINAL THETA: ", math.degrees(self.theta_f))
        self.theta_i = Sensors.odometry.getPose().rotation().radians()
        self.theta_diff = bounded_angle_diff(self.theta_i, self.theta_f)

    def execute(self) -> None:
        goal = Sensors.odometry.getPose()
        speeds = self.controller.calculate(goal, goal, 0, Rotation2d(self.theta_f))
        self.subsystem.set_driver_centric((0, 0), speeds.omega)

    def end(self, interrupted: bool) -> None:
        print("ENDED ROTATE")
        self.subsystem.set_driver_centric((0, 0), 0)

    def isFinished(self) -> bool:
        return (
            abs(Sensors.odometry.getPose().rotation().radians() - self.theta_f)
            < self.threshold
        )

    def runsWhenDisabled(self) -> bool:
        return False
