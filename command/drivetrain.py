import logging
import math

import commands2
from commands2 import SequentialCommandGroup
from robotpy_toolkit_7407.command import SubsystemCommand
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.filter import SlewRateLimiter
import command
import command.autonomous.custom_pathing
import config
import constants
from command.autonomous import CustomTrajectory
from robot_systems import Sensors
from sensors import FieldOdometry
from subsystem import Drivetrain


def curve_abs(x):
    return x**2


def curve(x):
    if x < 0:
        return -curve_abs(-x)
    return curve_abs(x)


class DriveSwerveCustom(SubsystemCommand[Drivetrain]):
    driver_centric = True
    driver_centric_reversed = False
    period = constants.period
    angular_pid: PIDController = PIDController(4, 0, 0.05)
    target_angle = None

    def initialize(self) -> None:
        self.angular_pid.setSetpoint(0)
        self.target_angle = Sensors.gyro.get_robot_heading() % (math.pi * 2)
        self.target_angle = math.atan2(
            math.sin(self.target_angle), math.cos(self.target_angle)
        )
        self.ramp_limit_x = SlewRateLimiter(constants.drivetrain_max_accel_tele, -constants.drivetrain_max_accel_tele, 0.0)
        self.ramp_limit_y = SlewRateLimiter(constants.drivetrain_max_accel_tele, -constants.drivetrain_max_accel_tele, 0.0)
    def execute(self) -> None:
        
        #might be better to add acceleration after scaling if its non-linear
                
        dx, dy, d_theta = (
            self.subsystem.axis_dx.value * (-1 if config.drivetrain_reversed else 1),
            self.subsystem.axis_dy.value * (-1 if config.drivetrain_reversed else 1),
            -self.subsystem.axis_rotation.value,
        )
        
        if abs(d_theta) < 0.11:
            d_theta = 0
            

        print("dx", dx)
        dx = curve(dx)
        dy = curve(dy)
        d_theta = curve(d_theta)

        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel
        d_theta *= self.subsystem.max_angular_vel

        # if dx == 0 and dy == 0 and d_theta == 0:
        #     self.subsystem.n_front_left.zero()
        #     self.subsystem.n_front_right.zero()
        #     self.subsystem.n_back_left.zero()
        #     self.subsystem.n_back_right.zero()
        #
        #     self.subsystem.n_front_left.set_motor_angle(0)
        #     self.subsystem.n_front_right.set_motor_angle(0)
        #     self.subsystem.n_back_left.set_motor_angle(0)
        #     self.subsystem.n_back_right.set_motor_angle(0)


        if constants.drivetrain_accel:
            dx_scale = dx
            dy_scale = dy

            dx = self.ramp_limit_x.calculate(dx)
            dy = self.ramp_limit_y.calculate(dy)
        
            ## deceleration
            if abs(dx) > abs(dx_scale):
                dx = self.ramp_limit_x.reset(dx_scale)
            
            if abs(dy) > abs(dy_scale):
                dx = self.ramp_limit_y.reset(dy_scale)


        if config.driver_centric:
            self.subsystem.set_driver_centric((-dy, dx), d_theta)
        elif self.driver_centric_reversed:
            self.subsystem.set_driver_centric((dy, -dx), d_theta)
        else:
            self.subsystem.set_robot_centric((dy, -dx), d_theta)

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_front_left.set_motor_velocity(0)
        self.subsystem.n_front_right.set_motor_velocity(0)
        self.subsystem.n_back_left.set_motor_velocity(0)
        self.subsystem.n_back_right.set_motor_velocity(0)
        self.ramp_limit_x.reset(0)
        self.ramp_limit_y.reset(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class DrivetrainZero(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        print("ZEROING DRIVETRAIN")
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
        logging.info("Successfully re-zeroed swerve pods.")
        ...


class DriveSwerveSlowed(SubsystemCommand[Drivetrain]):
    driver_centric = True
    driver_centric_reversed = False
    angular_pid = None
    target_angle = Rotation2d(0)

    def initialize(self) -> None:
        print("STARTED DRIVE SWERVE SLOW")
        self.angular_pid: PIDController = PIDController(2, 0, 0.05)
        self.angular_pid.setSetpoint(0)

        gyro_angle = self.subsystem.gyro.get_robot_heading() % (math.pi * 2)
        gyro_angle = math.degrees(
            math.atan2(math.sin(gyro_angle), math.cos(gyro_angle))
        )

        if -90 < gyro_angle < 90:
            self.target_angle = Rotation2d(0)
        else:
            self.target_angle = Rotation2d(math.pi)

    def execute(self) -> None:
        current_angle = Sensors.odometry.getPose().rotation()
        relative = Pose2d(0, 0, self.target_angle).relativeTo(
            Pose2d(0, 0, current_angle)
        )

        angular_vel = self.angular_pid.calculate(abs(relative.rotation().radians())) * (
            -1 if relative.rotation().radians() > 0 else 1
        )

        dx, dy, d_theta = (
            self.subsystem.axis_dx.value * (-1 if config.drivetrain_reversed else 1),
            self.subsystem.axis_dy.value * (-1 if config.drivetrain_reversed else 1),
            angular_vel,
        )

        dx = curve(dx)
        dy = curve(dy)

        dx *= config.drivetrain_scoring_velocity * 2.5
        dy *= -config.drivetrain_scoring_velocity * 2.5
        d_theta = min(config.drivetrain_scoring_angular_velocity, d_theta)

        controller_d_theta = -self.subsystem.axis_rotation.value

        if abs(controller_d_theta) > 0.15:
            d_theta = (
                curve(controller_d_theta) * config.drivetrain_scoring_angular_velocity
            )

        self.subsystem.set_driver_centric((-dy, dx), d_theta)

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_front_left.set_motor_velocity(0)
        self.subsystem.n_front_right.set_motor_velocity(0)
        self.subsystem.n_back_left.set_motor_velocity(0)
        self.subsystem.n_back_right.set_motor_velocity(0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


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
                        ).andThen(DrivetrainScoreSlow(self.subsystem, self.odometry))
                    )
                )
            except:
                print("COULD NOT GENERATE TRAJECTORY")
                commands2.CommandScheduler.getInstance().schedule(
                    DrivetrainScoreSlow(self.subsystem, self.odometry)
                )

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        ...


class DrivetrainScoreSlow(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        commands2.CommandScheduler.getInstance().schedule(
            command.DriveSwerveSlowed(self.subsystem)
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
        config.drivetrain_reversed = False

        commands2.CommandScheduler.getInstance().schedule(
            command.DriveSwerveCustom(self.subsystem)
        )

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        ...
