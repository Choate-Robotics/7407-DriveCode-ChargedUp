import commands2
import wpilib
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
from robotpy_toolkit_7407.sensors.limelight import Limelight, LimelightController
from wpilib import SmartDashboard

import command
import constants
from oi.OI import OI
from robot_systems import Robot, Sensors
from sensors import FieldOdometry, PV_Cameras


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface
        OI.init()
        OI.map_controls()
        period = 0.03
        commands2.CommandScheduler.getInstance().setPeriod(period)

        Sensors.limelight_front = Limelight(
            cam_height=0, cam_angle=0, robot_ip="10.74.07.2"
        )
        # Sensors.limelight_controller = LimelightController([Sensors.limelight_front])

        Robot.drivetrain.init()

        SmartDashboard.init()
        # Sensors.pv_controller = PV_Cameras()

        # Sensors.odometry = FieldOdometry(Robot.drivetrain, Sensors.pv_controller)

        # self.start_limelight_pose = Sensors.limelight_controller.get_estimated_robot_pose()[0].toPose2d()
        # self.start_robot_pose = Sensors.odometry.get_robot_pose()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        # limelight_bot_pose = Sensors.limelight_controller.get_estimated_robot_pose()
        # pv_bot_pose = Sensors.pv_controller.get_estimated_robot_pose()
        # current_robot_pose = Sensors.odometry.update()
        # SmartDashboard.putString("ODOMETRY POSE", str(current_robot_pose))
        # SmartDashboard.putString(
        #     "DRIVETRAIN POSE", str(Robot.drivetrain.odometry.getPose())
        # )

        # print("Wheel Position: ", Robot.drivetrain.n_00.m_turn.get_sensor_position())
        # print(
        #     "Absolute Position: ", Robot.drivetrain.n_00.encoder.getAbsolutePosition()
        # )

    def teleopInit(self):
        commands2.CommandScheduler.getInstance().schedule(
            command.DrivetrainZero(Robot.drivetrain).andThen(
                command.DriveSwerveCustom(Robot.drivetrain)
            )
        )
        pass

    def teleopPeriodic(self):
        SmartDashboard.putString(
            "N00", str(Robot.drivetrain.n_00.encoder.getAbsolutePosition())
        )
        SmartDashboard.putString(
            "N01", str(Robot.drivetrain.n_01.encoder.getAbsolutePosition())
        )
        SmartDashboard.putString(
            "N10", str(Robot.drivetrain.n_10.encoder.getAbsolutePosition())
        )
        SmartDashboard.putString(
            "N11", str(Robot.drivetrain.n_11.encoder.getAbsolutePosition())
        )

        SmartDashboard.putString(
            "N00_m", str(Robot.drivetrain.n_00.get_turn_motor_angle())
        )
        SmartDashboard.putString(
            "N01_m", str(Robot.drivetrain.n_01.get_turn_motor_angle())
        )
        SmartDashboard.putString(
            "N10_m", str(Robot.drivetrain.n_10.get_turn_motor_angle())
        )
        SmartDashboard.putString(
            "N11_m", str(Robot.drivetrain.n_11.get_turn_motor_angle())
        )

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(_Robot)
    # Robot.robotInit(Robot())
