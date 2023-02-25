import logging
import math

import commands2
from commands2 import SequentialCommandGroup
from robotpy_toolkit_7407.command import SubsystemCommand

import autonomous.utils.custom_pathing
import command
import config
import constants
from sensors import FieldOdometry
from subsystem import Drivetrain


def curve_abs(x):
    return x**2.4


def curve(x):
    if x < 0:
        return -curve_abs(-x)
    return curve_abs(x)


class DriveSwerveCustom(SubsystemCommand[Drivetrain]):
    driver_centric = True
    driver_centric_reversed = False

    def initialize(self) -> None:
        print("STARTED DRIVE SWERVE")
        pass

    def execute(self) -> None:

        dx, dy, d_theta = (
            self.subsystem.axis_dx.value * (-1 if config.red_team else 1),
            self.subsystem.axis_dy.value * (-1 if config.red_team else 1),
            -self.subsystem.axis_rotation.value,
        )

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

        logging.info("Successfully re-zeroed swerve pods.")
        ...


class DrivetrainScore(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain, odometry: FieldOdometry):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.odometry = odometry

    def initialize(self) -> None:
        current_theta = self.odometry.getPose().rotation().degrees()

        if -90 < current_theta < 90:
            desired_theta = 0
        else:
            desired_theta = math.radians(180)

        self.subsystem.max_vel = config.drivetrain_scoring_velocity
        self.subsystem.max_angular_vel = config.drivetrain_scoring_angular_velocity

        commands2.CommandScheduler.getInstance().schedule(
            SequentialCommandGroup(
                autonomous.utils.custom_pathing.RotateInPlaceTeleOp(
                    self.subsystem,
                    theta_f=desired_theta,
                    max_angular_vel=config.drivetrain_scoring_angular_velocity,
                ),
                command.DriveSwerveCustom(self.subsystem),
            )
        )

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        ...


class DrivetrainRegular(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain, odometry: FieldOdometry):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.odometry = odometry

    def initialize(self) -> None:
        self.subsystem.max_vel = constants.drivetrain_max_vel
        self.subsystem.max_angular_vel = constants.drivetrain_max_angular_vel

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        commands2.CommandScheduler.getInstance().schedule(
            command.DriveSwerveCustom(self.subsystem)
        )
