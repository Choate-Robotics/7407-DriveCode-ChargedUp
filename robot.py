import math

import commands2
import wpilib
from wpilib import SmartDashboard

import command
from autonomous import routine
from oi.OI import OI
from robot_systems import Pneumatics, Robot, Sensors
from sensors import FieldOdometry
from utils import logger


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        period = 0.03
        commands2.CommandScheduler.getInstance().setPeriod(period)
        Pneumatics.compressor.enableAnalog(90, 120)

        Robot.arm.init()
        Robot.drivetrain.init()
        Robot.intake.init()
        Robot.grabber.init()

        # Sensors.pv_controller = PV_Cameras()

        Sensors.odometry = FieldOdometry(Robot.drivetrain, None)
        Sensors.gyro = Robot.drivetrain.gyro

        OI.init()
        OI.map_controls()

    def robotPeriodic(self):
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

        SmartDashboard.putNumber(
            "gyro_angle: ", math.degrees(Robot.drivetrain.gyro.get_robot_heading())
        )

        commands2.CommandScheduler.getInstance().run()

        SmartDashboard.putNumber(
            "SHOULDER ANGLE: ", math.degrees(Robot.arm.get_rotation())
        )
        SmartDashboard.putNumber(
            "WRIST ANGLE: ", math.degrees(Robot.grabber.get_angle())
        )
        SmartDashboard.putNumber("SHOULDER DIST: ", Robot.arm.get_length())

    def teleopInit(self):
        logger.debug("TELEOP", "Teleop Initialized")
        commands2.CommandScheduler.getInstance().schedule(
            command.ZeroElevator(Robot.arm).andThen(
                command.SetArm(Robot.arm, distance=0, shoulder_angle=math.radians(0))
            )
        )
        commands2.CommandScheduler.getInstance().schedule(
            command.ZeroWrist(Robot.grabber)
        )
        commands2.CommandScheduler.getInstance().schedule(
            command.DriveSwerveCustom(Robot.drivetrain)
        )

        Robot.arm.enable_brake()

    def teleopPeriodic(self):
        SmartDashboard.putBoolean(
            "Limit Switch", Robot.arm.elevator_bottom_sensor.get()
        )
        # print("Limit Switch: ", Robot.arm.elevator_bottom_sensor.get())

        # print(Robot.arm.elevator_bottom_sensor.get())
        # print(Pneumatics.compressor.getPressure())
        # print("I THINK I'm AT: ", math.degrees(Robot.Arm.get_rotation()))
        # Robot.Arm.disable_brake()
        ...

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
