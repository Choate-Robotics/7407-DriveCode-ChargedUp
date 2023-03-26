import math
import time

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
    def __init__(
        self,
        subsystem: Drivetrain,
        vx,
        vx2,
        omega,
        gyro_threshold=math.radians(3),
        gyro_threshold_2=0.195,
        times_before_stop=1,
    ):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.vx = vx
        self.vx2 = vx2
        self.omega = omega
        self.gyro_threshold = gyro_threshold
        self.gyro_threshold_2 = gyro_threshold_2
        self.times_zeroed = 0
        self.currently_zeroed = 0
        self.times_before_stop = times_before_stop
        self.reduced_speed = False

    def initialize(self) -> None:
        self.times_zeroed = 0
        self.currently_zeroed = 0
        self.reduced_speed = False
        SmartDashboard.putBoolean("REDUCED SPEED", False)
        ...

    def execute(self) -> None:
        SmartDashboard.putBoolean("REDUCED SPEED", self.reduced_speed)

        if (
            self.times_zeroed > 0
            and self.currently_zeroed == 0
            and abs(self.subsystem.gyro.get_robot_pitch()) > self.gyro_threshold_2
            and not self.reduced_speed
        ):
            self.reduced_speed = True

        if self.reduced_speed:
            self.subsystem.set_driver_centric((-self.vx2, 0), self.omega)
        else:
            self.subsystem.set_driver_centric((-self.vx, 0), self.omega)

    def isFinished(self) -> bool:
        pitch = self.subsystem.gyro.get_robot_pitch()
        if abs(pitch) < self.gyro_threshold:
            if self.currently_zeroed == 1:
                self.times_zeroed += 1
                self.currently_zeroed += 1
            else:
                self.currently_zeroed += 1
        else:
            self.currently_zeroed = 0

        return self.times_zeroed > self.times_before_stop

    def end(self, interrupted: bool = False) -> None:
        if not interrupted:
            self.subsystem.set_driver_centric((0, 0), 0)
        ...


class DriveOverChargeStation(SubsystemCommand[Drivetrain]):
    def __init__(
        self,
        subsystem: Drivetrain,
        vx,
        vy,
        omega,
        gyro_threshold=math.radians(5),
        times_before_stop=2,
    ):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.vx = vx
        self.vy = vy
        self.omega = omega
        self.gyro_threshold = gyro_threshold
        self.times_zeroed = 0
        self.currently_zeroed = 0
        self.times_before_stop = times_before_stop

    def initialize(self) -> None:
        self.times_zeroed = 0
        self.currently_zeroed = 0
        ...

    def execute(self) -> None:
        self.subsystem.set_driver_centric((-self.vx, -self.vy), self.omega)

    def isFinished(self) -> bool:
        pitch = self.subsystem.gyro.get_robot_pitch()
        if abs(pitch) < self.gyro_threshold:
            if self.currently_zeroed == 1:
                self.times_zeroed += 1
                self.currently_zeroed += 1
                print("TIMES ZEROED: ", self.times_zeroed)
            else:
                self.currently_zeroed += 1
        else:
            print("RESET CURRENTLY ZEROED")
            self.currently_zeroed = 0

        return self.times_zeroed > self.times_before_stop

    def end(self, interrupted: bool = False) -> None:
        if not interrupted:
            self.subsystem.set_driver_centric((0, 0), 0)
        ...


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
                7.3,  # 7.3
                0,
                0.01,  # .005
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

        relative = self.end_pose.relativeTo(Sensors.odometry.getPose())

        if (
            abs(relative.x) < 0.05
            and abs(relative.y < 0.05)
            and abs(relative.rotation().degrees()) < 3
            or self.t > self.duration
        ):
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


class CustomRouting(SubsystemCommand[SwerveDrivetrain]):
    """
    Follows a path using a holonomic drive controller.

    :param subsystem: The subsystem to run this command on
    :type subsystem: SwerveDrivetrain
    :param target: The Pose to drive the drivetrain to
    :type target: Pose2d
    :param period: The period of the controller, defaults to 0.02
    :type period: float, optional
    """

    def __init__(
        self,
        subsystem: SwerveDrivetrain,
        target: Pose2d,
        min_horizontal_vel: float | None = 1,
        min_vertical_vel: float | None = 1,
        min_angular_vel: float | None = 0.2,
    ):
        super().__init__(subsystem)
        self.target: Pose2d = target
        self.min_horizontal_vel = min_horizontal_vel
        self.min_vertical_vel = min_vertical_vel
        self.min_angular_vel = min_angular_vel
        self.end_pose: Pose2d = target
        self.horizontal_finished: bool = False
        self.vertical_finished: bool = False
        self.angular_finished: bool = False

        self.horizontal_pid: PIDController | None = None
        self.vertical_pid: PIDController | None = None
        self.angular_pid: PIDController | None = None

    def initialize(self) -> None:
        self.horizontal_finished: bool = False
        self.vertical_finished: bool = False
        self.angular_finished: bool = False

        self.horizontal_pid = PIDController(4, 0, 0.01)
        self.vertical_pid = PIDController(4, 0, 0.01)
        self.angular_pid = PIDController(2, 0, 0.05)

        self.horizontal_pid.setSetpoint(0)
        self.vertical_pid.setSetpoint(0)
        self.angular_pid.setSetpoint(0)

    def execute(self) -> None:
        current_pose = Sensors.odometry.getPose()
        relative = self.end_pose.relativeTo(current_pose)

        if abs(relative.x) < 0.01:
            self.horizontal_finished = True
        else:
            self.horizontal_finished = False
        if abs(relative.y) < 0.01:
            self.vertical_finished = True
        else:
            self.vertical_finished = False
        if abs(relative.rotation().degrees()) < 5:
            self.angular_finished = True
        else:
            self.angular_finished = False

        horizontal_vel = (
            self.horizontal_pid.calculate(abs(relative.x))
            * (1 if relative.x > 0 else -1)
            * (1 if self.target.rotation().radians() == 0 else -1)
        )
        vertical_vel = (
            self.vertical_pid.calculate(abs(relative.y))
            * (1 if relative.y > 0 else -1)
            * (1 if self.target.rotation().radians() == 0 else -1)
        )
        angular_vel = self.angular_pid.calculate(abs(relative.rotation().radians())) * (
            -1 if relative.rotation().radians() > 0 else 1
        )

        self.subsystem.set_driver_centric(
            (
                horizontal_vel if not self.horizontal_finished else 0,
                vertical_vel if not self.vertical_finished else 0,
            ),
            angular_vel,
        )

    def isFinished(self) -> bool:
        print(self.horizontal_finished, self.vertical_finished, self.angular_finished)
        return (
            self.horizontal_finished
            and self.vertical_finished
            and self.angular_finished
        )

    def end(self, interrupted: bool) -> None:
        print("FINISHED RUNNING CUSTOM ROUTE")
        self.subsystem.set_driver_centric((0, 0), 0)
        SmartDashboard.putString("POSE", str(self.subsystem.odometry.getPose()))
        SmartDashboard.putString(
            "POSD", str(Sensors.odometry.getPose().rotation().degrees())
        )

    def runsWhenDisabled(self) -> bool:
        return False


class CustomRouting(SubsystemCommand[SwerveDrivetrain]):
    """
    Follows a path using a holonomic drive controller.

    :param subsystem: The subsystem to run this command on
    :type subsystem: SwerveDrivetrain
    :param target: The Pose to drive the drivetrain to
    :type target: Pose2d
    :param period: The period of the controller, defaults to 0.02
    :type period: float, optional
    """

    def __init__(
        self,
        subsystem: SwerveDrivetrain,
        target: Pose2d,
        min_horizontal_vel: float | None = 1,
        min_vertical_vel: float | None = 1,
        min_angular_vel: float | None = 0.2,
    ):
        super().__init__(subsystem)
        self.target: Pose2d = target
        self.min_horizontal_vel = min_horizontal_vel
        self.min_vertical_vel = min_vertical_vel
        self.min_angular_vel = min_angular_vel
        self.end_pose: Pose2d = target
        self.horizontal_finished: bool = False
        self.vertical_finished: bool = False
        self.angular_finished: bool = False

        self.horizontal_pid: PIDController | None = None
        self.vertical_pid: PIDController | None = None
        self.angular_pid: PIDController | None = None

    def initialize(self) -> None:
        self.horizontal_finished: bool = False
        self.vertical_finished: bool = False
        self.angular_finished: bool = False

        self.horizontal_pid = PIDController(4, 0, 0.01)
        self.vertical_pid = PIDController(4, 0, 0.01)
        self.angular_pid = PIDController(2, 0, 0.05)

        self.horizontal_pid.setSetpoint(0)
        self.vertical_pid.setSetpoint(0)
        self.angular_pid.setSetpoint(0)

    def execute(self) -> None:
        current_pose = Sensors.odometry.getPose()
        relative = self.end_pose.relativeTo(current_pose)

        if abs(relative.x) < 0.03:
            self.horizontal_finished = True
        else:
            self.horizontal_finished = False
        if abs(relative.y) < 0.03:
            self.vertical_finished = True
        else:
            self.vertical_finished = False
        if abs(relative.rotation().degrees()) < 5:
            self.angular_finished = True
        else:
            self.angular_finished = False

        horizontal_vel = (
            self.horizontal_pid.calculate(abs(relative.x))
            * (1 if relative.x > 0 else -1)
            * (1 if self.target.rotation().radians() == 0 else -1)
        )
        vertical_vel = (
            self.vertical_pid.calculate(abs(relative.y))
            * (1 if relative.y > 0 else -1)
            * (1 if self.target.rotation().radians() == 0 else -1)
        )
        angular_vel = self.angular_pid.calculate(abs(relative.rotation().radians())) * (
            -1 if relative.rotation().radians() > 0 else 1
        )

        self.subsystem.set_driver_centric(
            (
                horizontal_vel if not self.horizontal_finished else 0,
                vertical_vel if not self.vertical_finished else 0,
            ),
            angular_vel,
        )

    def isFinished(self) -> bool:
        print(self.horizontal_finished, self.vertical_finished, self.angular_finished)
        return (
            self.horizontal_finished
            and self.vertical_finished
            and self.angular_finished
        )

    def end(self, interrupted: bool) -> None:
        print("FINISHED RUNNING CUSTOM ROUTE")
        self.subsystem.set_driver_centric((0, 0), 0)
        SmartDashboard.putString("POSE", str(self.subsystem.odometry.getPose()))
        SmartDashboard.putString(
            "POSD", str(Sensors.odometry.getPose().rotation().degrees())
        )

    def runsWhenDisabled(self) -> bool:
        return False
