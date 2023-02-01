import logging

from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveDrivetrain
from robotpy_toolkit_7407.utils.math import bounded_angle_diff, rotate_vector
import config
from subsystem import Drivetrain
# from robotpy_toolkit_7407.utils.units import radians
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.geometry import Pose2d
from wpimath.geometry import Rotation2d
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians




def curve_abs(x):
    return x**2.4


def curve(x):
    if x < 0:
        return -curve_abs(-x)
    return curve_abs(x)


class DriveSwerveCustom(SubsystemCommand[Drivetrain]):
    driver_centric = True
    driver_centric_reversed = False
    gravitate = False
    def __init__(
        self,
        subsystem: Drivetrain,
        pose_f: Pose2d,
        period: float = 0.02,
    ):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.controller = HolonomicDriveController(
            PIDController(1, 0, 0, period),
            PIDController(1, 0, 0, period),
            ProfiledPIDControllerRadians(
                4,
                0,
                0,
                TrapezoidProfileRadians.Constraints(
                    self.subsystem.max_angular_vel, self.subsystem.max_angular_vel / 0.01
                ),
                period,
            ),
        )
        self.start_time = 0
        self.pose_f = pose_f
        self.theta_i: float | None = None
        self.theta_diff: float | None = None
        self.omega: float | None = None
        self.finished: bool = False

    def initialize(self) -> None:
        pass

    def execute(self) -> None:

        dx, dy, d_theta = (
            self.subsystem.axis_dx.value * (-1 if config.red_team else 1),
            self.subsystem.axis_dy.value * (-1 if config.red_team else 1),
            -self.subsystem.axis_rotation.value,
        )

        if self.gravitate:
            speeds = self.controller.calculate(self.subsystem.odometry.getPose(), self.pose_f)#, Rotation2d(goal_theta))
            dx,dy = rotate_vector(speeds.vx, speeds.vy, self.subsystem.odometry.getPose().rotation().radians())
        

        if abs(d_theta) < 0.15:
            d_theta = 0

        dx = curve(dx)
        dy = curve(dy)
        d_theta = curve(d_theta)

        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel

        if DriveSwerveCustom.driver_centric:
            self.subsystem.set_driver_centric(
                (-dy, dx), d_theta * self.subsystem.max_angular_vel
            )
        elif DriveSwerveCustom.driver_centric_reversed:
            self.subsystem.set_driver_centric(
                (dy, -dx), d_theta * self.subsystem.max_angular_vel
            )
        else:
            self.subsystem.set_robot_centric(
                (dx, dy), d_theta * self.subsystem.max_angular_vel
            )

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_front_left.set(0, 0)
        self.subsystem.n_front_right.set(0, 0)
        self.subsystem.n_back_left.set(0, 0)
        self.subsystem.n_back_right.set(0, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class DrivetrainZero(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        self.subsystem.n_front_left.zero()
        self.subsystem.n_front_right.zero()
        self.subsystem.n_back_left.zero()
        self.subsystem.n_back_right.zero()

    def execute(self) -> None:
        ...

    def isFinished(self) -> bool:
        ...
        return True

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_front_left.m_move.set_sensor_position(0)
        self.subsystem.n_front_right.m_move.set_sensor_position(0)
        self.subsystem.n_back_left.m_move.set_sensor_position(0)
        self.subsystem.n_back_right.m_move.set_sensor_position(0)

        logging.info(
            "Successfully rezeroed swerve pods."
        )
        ...
