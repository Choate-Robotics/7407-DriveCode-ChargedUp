import time

from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveDrivetrain
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.math import bounded_angle_diff, rotate_vector
from robotpy_toolkit_7407.utils.units import radians
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.geometry import Rotation2d
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians


class FollowPathCustom(SubsystemCommand[SwerveDrivetrain]):
    def __init__(
        self,
        subsystem: SwerveDrivetrain,
        trajectory: Trajectory,
        theta_f: radians,
        period: float = 0.02,
    ):
        super().__init__(subsystem)
        self.trajectory = trajectory
        self.controller = HolonomicDriveController(
            PIDController(1, 0, 0, period),
            PIDController(1, 0, 0, period),
            ProfiledPIDControllerRadians(
                4,
                0,
                0,
                TrapezoidProfileRadians.Constraints(
                    subsystem.max_angular_vel, subsystem.max_angular_vel / 0.01
                ),
                period,
            ),
        )
        self.start_time = 0
        self.t = 0
        self.duration = trajectory.totalTime()
        self.theta_f = theta_f
        self.theta_i: float | None = None
        self.theta_diff: float | None = None
        self.omega: float | None = None
        self.finished: bool = False

    def initialize(self) -> None:
        self.start_time = time.perf_counter()
        self.theta_i = self.subsystem.odometry.getPose().rotation().radians()
        self.theta_diff = bounded_angle_diff(self.theta_i, self.theta_f)
        self.omega = self.theta_diff / self.duration
        self.finished = False

        self.mod = 0

    def execute(self) -> None:
        if self.mod % 5 == 0:
            print(self.subsystem.odometry.getPose())
        self.mod += 1

        self.t = time.perf_counter() - self.start_time
        if self.t > self.duration:
            self.t = self.duration
            self.finished = True
        goal = self.trajectory.sample(self.t)
        goal_theta = self.theta_i + self.omega * self.t
        speeds = self.controller.calculate(
            self.subsystem.odometry.getPose(), goal, Rotation2d(goal_theta)
        )

        vx, vy = rotate_vector(
            speeds.vx, speeds.vy, self.subsystem.odometry.getPose().rotation().radians()
        )

        print(f"vx: {vx}, vy: {vy}, omega: {speeds.omega}")

        self.subsystem.set_driver_centric((-vx, vy), speeds.omega)

    def end(self, interrupted: bool) -> None:
        print("ENDED")
        self.subsystem.set_driver_centric((0, 0), 0)

    def isFinished(self) -> bool:
        return self.finished

    def runsWhenDisabled(self) -> bool:
        return False


class RotateInPlace(SubsystemCommand[SwerveDrivetrain]):
    def __init__(
        self,
        subsystem: SwerveDrivetrain,
        theta_f: radians,
        duration: float = 0.5,
        period: float = 0.02,
    ):
        super().__init__(subsystem)
        self.controller = HolonomicDriveController(
            PIDController(1, 0, 0, period),
            PIDController(1, 0, 0, period),
            ProfiledPIDControllerRadians(
                4,
                0,
                0,
                TrapezoidProfileRadians.Constraints(
                    subsystem.max_angular_vel, subsystem.max_angular_vel / 0.01
                ),
                period,
            ),
        )
        self.start_time = 0
        self.t = 0
        self.duration = duration
        self.theta_f = theta_f
        self.theta_i: float | None = None
        self.theta_diff: float | None = None
        self.omega: float | None = None
        self.finished = False

    def initialize(self) -> None:
        logger.info(f"rotating")
        self.start_time = time.perf_counter()
        self.theta_i = self.subsystem.odometry.getPose().rotation().radians()
        self.theta_diff = bounded_angle_diff(self.theta_i, self.theta_f)
        self.omega = self.theta_diff / self.duration
        self.finished = False

    def execute(self) -> None:
        self.t = time.perf_counter() - self.start_time
        if self.t > self.duration:
            self.t = self.duration
            self.finished = True
        goal = self.subsystem.odometry.getPose()
        goal_theta = self.theta_i + self.omega * self.t
        speeds = self.controller.calculate(goal, goal, 0, Rotation2d(goal_theta))
        self.subsystem.set_driver_centric((0, 0), speeds.omega)

    def end(self, interrupted: bool) -> None:
        self.subsystem.set_driver_centric((0, 0), 0)

    def isFinished(self) -> bool:
        return self.finished

    def runsWhenDisabled(self) -> bool:
        return False
