import logging
import math

import commands2
from commands2 import SequentialCommandGroup
from robotpy_toolkit_7407.command import SubsystemCommand
from wpimath.geometry import Pose2d

import command
import command.autonomous.custom_pathing
import config
import constants
from command.autonomous import CustomTrajectory
from sensors import FieldOdometry
from subsystem import Drivetrain


def curve_abs(x):
    return x ** 2


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
            self.subsystem.axis_dx.value
            * (-1 if config.red_team else 1)
            * (-1 if config.drivetrain_reversed else 1),
            self.subsystem.axis_dy.value
            * (-1 if config.red_team else 1)
            * (-1 if config.drivetrain_reversed else 1),
            -self.subsystem.axis_rotation.value,
        )

        if abs(d_theta) < 0.15:
            d_theta = 0

        dx = curve(dx)
        dy = curve(dy)
        d_theta = curve(d_theta)

        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel

        if config.driver_centric:
            self.subsystem.set_driver_centric(
                (-dy, dx), d_theta * self.subsystem.max_angular_vel
            )
        elif self.driver_centric_reversed:
            self.subsystem.set_driver_centric(
                (dy, -dx), d_theta * self.subsystem.max_angular_vel
            )
        else:
            self.subsystem.set_robot_centric(
                (dy, -dx), d_theta * self.subsystem.max_angular_vel
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


class DrivetrainRoute(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain, odometry: FieldOdometry):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.odometry = odometry
        self.drive_on = True

    def initialize(self) -> None:
        self.odometry.vision_on = False
        current_pose = self.odometry.getPose()
        if self.drive_on and config.current_scoring_location != "":
            try:
                desired_target = config.scoring_locations[
                    config.current_scoring_location
                ]
                current_pose = Pose2d(
                    current_pose.x,
                    current_pose.y,
                    desired_target.target_pose.rotation().radians(),
                )
                trajectory = CustomTrajectory(
                    current_pose,
                    desired_target.target_waypoints
                    if desired_target.target_waypoints is not None
                    else [],
                    desired_target.target_pose,
                    max_velocity=config.drivetrain_routing_velocity,
                    max_accel=config.drivetrain_routing_acceleration,
                    start_velocity=0,
                    end_velocity=0,
                )

                commands2.CommandScheduler.getInstance().schedule(
                    command.autonomous.custom_pathing.RotateInPlace(
                        self.subsystem,
                        desired_target.target_pose.rotation().radians(),
                        threshold=math.radians(4),
                        max_angular_vel=config.drivetrain_routing_angular_velocity,
                    ).andThen(
                        command.autonomous.custom_pathing.FollowPathCustom(
                            self.subsystem, trajectory
                        ).andThen(DrivetrainScoreBack(self.subsystem, self.odometry))
                    )
                )
            except:
                print("COULD NOT GENERATE TRAJECTORY")
                commands2.CommandScheduler.getInstance().schedule(
                    DrivetrainScoreBack(self.subsystem, self.odometry)
                )

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        ...


class DrivetrainScoreBack(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain, odometry: FieldOdometry):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.odometry = odometry

    def initialize(self) -> None:
        config.driver_centric = False
        self.odometry.vision_on = False
        current_theta = self.odometry.getPose().rotation().degrees()

        if -90 < current_theta < 90:
            desired_theta = 0
        else:
            desired_theta = math.radians(180)

        self.subsystem.max_vel = config.drivetrain_scoring_velocity
        self.subsystem.max_angular_vel = config.drivetrain_scoring_angular_velocity

        commands2.CommandScheduler.getInstance().schedule(
            SequentialCommandGroup(
                # command.autonomous.custom_pathing.RotateInPlace(
                #     self.subsystem,
                #     threshold=math.radians(6),
                #     theta_f=desired_theta,
                #     max_angular_vel=config.drivetrain_scoring_angular_velocity,
                # ),
                command.DriveSwerveCustom(self.subsystem),
            )
        )

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        ...


class DrivetrainScoreFront(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain, odometry: FieldOdometry):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.odometry = odometry

    def initialize(self) -> None:
        # config.driver_centric = False
        config.drivetrain_reversed = True

        self.odometry.vision_on = False
        current_theta = self.odometry.getPose().rotation().degrees()

        if -90 < current_theta < 90:
            desired_theta = 0
        else:
            desired_theta = math.radians(180)

        self.subsystem.max_vel = config.drivetrain_scoring_velocity
        self.subsystem.max_angular_vel = config.drivetrain_scoring_angular_velocity

        commands2.CommandScheduler.getInstance().schedule(
            SequentialCommandGroup(
                # command.autonomous.custom_pathing.RotateInPlace(
                #     self.subsystem,
                #     threshold=math.radians(6),
                #     theta_f=desired_theta,
                #     max_angular_vel=config.drivetrain_scoring_angular_velocity,
                # ),
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
        # config.driver_centric = True
        config.drivetrain_reversed = False

        self.odometry.vision_on = True
        self.subsystem.max_vel = constants.drivetrain_max_vel
        self.subsystem.max_angular_vel = constants.drivetrain_max_angular_vel

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        commands2.CommandScheduler.getInstance().schedule(
            command.DriveSwerveCustom(self.subsystem)
        )
