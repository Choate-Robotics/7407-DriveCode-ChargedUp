import math

import commands2
import wpilib
from wpilib import SmartDashboard
import constants
from oi.keymap import Keymap
import command
from autonomous import routine
from oi.OI import OI
from robot_systems import Robot, Sensors, Pneumatics
from sensors import FieldOdometry
from utils import logger


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface'
        Robot.Arm.init()
        # for subsystem in Robot:
        #     subsystem.init()
        OI.init()
        OI.map_controls()
        period = 0.03
        commands2.CommandScheduler.getInstance().setPeriod(period)
        Pneumatics.compressor.enableAnalog(90, 120)
        #self.gyro = PigeonIMUGyro_Wrapper(10)
        # Target is .46272 meters above ground

        # Sensors.limelight_front = Limelight(
        #     cam_height=0, cam_angle=0, robot_ip="10.74.07.2"
        # )
        # Sensors.limelight_controller = LimelightController([Sensors.limelight_front])

        Robot.drivetrain.init()

        #SmartDashboard.init()
        #Sensors.pv_controller = PV_Cameras()

        Sensors.odometry = FieldOdometry(Robot.drivetrain, None)
        Sensors.gyro = Robot.drivetrain.gyro

        # self.start_limelight_pose = Sensors.limelight_controller.get_estimated_robot_pose()[0].toPose2d()
        # self.start_robot_pose = Sensors.odometry.get_robot_pose()

    def robotPeriodic(self):
        # Robot.drivetrain.logger_periodic()
        Sensors.odometry.update()
        SmartDashboard.putString("ODOM", str(Robot.drivetrain.odometry.getPose()))
        SmartDashboard.putString("FDOM", str(Sensors.odometry.getPose()))
        SmartDashboard.putString(
            "EDOM", str(Robot.drivetrain.odometry_estimator.getEstimatedPosition())
        )
        try:
            SmartDashboard.putString(
                "PHOTON", str(Sensors.pv_controller.get_estimated_robot_pose())
            )
            SmartDashboard.putString(
                "PHOTON ANGLE",
                str(
                    Sensors.pv_controller.get_estimated_robot_pose()[0][0]
                    .rotation()
                    .toRotation2d()
                    .degrees()
                ),
            )
        except Exception:
            pass

        pose = Robot.drivetrain.odometry_estimator.getEstimatedPosition()
        pose2 = Sensors.odometry.getPose()

        SmartDashboard.putNumberArray(
            "RobotPoseAdvantage", [pose.X(), pose.Y(), pose.rotation().radians()]
        )

        SmartDashboard.putNumberArray(
            "RobotPoseOrig", [pose2.X(), pose2.Y(), pose2.rotation().radians()]
        )

        try:
            pv_pose = Sensors.pv_controller.get_estimated_robot_pose()
            SmartDashboard.putNumberArray(
                "PVPoseAdvantage",
                [
                    pv_pose[0][0].toPose2d().X(),
                    pv_pose[0][0].toPose2d().Y(),
                    pv_pose[0][0].rotation().toRotation2d().radians(),
                ],
            )
        except Exception:
            pass

        commands2.CommandScheduler.getInstance().run()

        SmartDashboard.putString(
            "N00", str(Robot.drivetrain.n_front_left.encoder.getAbsolutePosition())
        )
        SmartDashboard.putString(
            "N01", str(Robot.drivetrain.n_front_right.encoder.getAbsolutePosition())
        )
        SmartDashboard.putString(
            "N10", str(Robot.drivetrain.n_back_left.encoder.getAbsolutePosition())
        )
        SmartDashboard.putString(
            "N11", str(Robot.drivetrain.n_back_right.encoder.getAbsolutePosition())
        )

        SmartDashboard.putString(
            "N00_m",
            str(math.degrees(Robot.drivetrain.n_front_left.get_turn_motor_angle())),
        )
        SmartDashboard.putString(
            "N01_m",
            str(math.degrees(Robot.drivetrain.n_front_right.get_turn_motor_angle())),
        )
        SmartDashboard.putString(
            "N10_m",
            str(math.degrees(Robot.drivetrain.n_back_left.get_turn_motor_angle())),
        )
        SmartDashboard.putString(
            "N11_m",
            str(math.degrees(Robot.drivetrain.n_back_right.get_turn_motor_angle())),
        )

        SmartDashboard.putString(
            "N00_drive",
            str(
                math.degrees(Robot.drivetrain.n_front_left.m_move.get_sensor_position())
            ),
        )
        SmartDashboard.putString(
            "N01_drive",
            str(
                math.degrees(
                    Robot.drivetrain.n_front_right.m_move.get_sensor_position()
                )
            ),
        )
        SmartDashboard.putString(
            "N10_drive",
            str(
                math.degrees(Robot.drivetrain.n_back_left.m_move.get_sensor_position())
            ),
        )
        SmartDashboard.putString(
            "N11_drive",
            str(
                math.degrees(Robot.drivetrain.n_back_right.m_move.get_sensor_position())
            ),
        )

        SmartDashboard.putNumber(
            "gyro_angle: ", math.degrees(Robot.drivetrain.gyro.get_robot_heading())
        )

    def teleopInit(self):

        commands2.CommandScheduler.getInstance().schedule(command.ZeroArm(Robot.Arm).andThen(command.manualMovement(Robot.Arm)))

        logger.debug("TELEOP", "Teleop Initialized")
        commands2.CommandScheduler.getInstance().schedule(
            command.DriveSwerveCustom(Robot.drivetrain)
        )

    def teleopPeriodic(self):
        pass


    def autonomousInit(self):
        routine.run()

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(_Robot)
