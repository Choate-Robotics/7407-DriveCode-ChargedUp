import math

import commands2
import wpilib
from wpilib import SmartDashboard

import command
from autonomous import routine
from oi.OI import OI
from robot_systems import Robot, Sensors
from sensors import FieldOdometry
from utils import logger


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface'
        OI.init()
        OI.map_controls()
        period = 0.03
        commands2.CommandScheduler.getInstance().setPeriod(period)

        Robot.drivetrain.init()

        SmartDashboard.init()
        # Sensors.pv_controller = PV_Cameras()

        Sensors.odometry = FieldOdometry(Robot.drivetrain, None)

    def robotPeriodic(self):
        Sensors.odometry.update()

        try:
            pose = Robot.drivetrain.odometry_estimator.getEstimatedPosition()

            SmartDashboard.putNumberArray(
                "RobotPoseAdvantage", [pose.X(), pose.Y(), pose.rotation().radians()]
            )
        except Exception:
            pass

        try:
            pose2 = Sensors.odometry.getPose()

            SmartDashboard.putNumberArray(
                "RobotPoseOrig", [pose2.X(), pose2.Y(), pose2.rotation().radians()]
            )
        except Exception:
            pass

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

        SmartDashboard.putNumber(
            "gyro_angle: ", math.degrees(Robot.drivetrain.gyro.get_robot_heading())
        )

        commands2.CommandScheduler.getInstance().run()

    def teleopInit(self):
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
