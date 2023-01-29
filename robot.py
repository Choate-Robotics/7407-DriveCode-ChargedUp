import math

import commands2
import wpilib
from wpilib import SmartDashboard

import command
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

        # Sensors.limelight_front = Limelight(
        #     cam_height=0, cam_angle=0, robot_ip="10.74.07.2"
        # )
        # Sensors.limelight_controller = LimelightController([Sensors.limelight_front])

        Robot.drivetrain.init()
        # Robot.drivetrain.n_front_left.m_move.set_sensor_position(0)
        # Robot.drivetrain.n_front_right.m_move.set_sensor_position(0)
        # Robot.drivetrain.n_back_left.m_move.set_sensor_position(0)
        # Robot.drivetrain.n_back_right.m_move.set_sensor_position(0)

        SmartDashboard.init()
        Sensors.pv_controller = PV_Cameras()

        Sensors.odometry = FieldOdometry(Robot.drivetrain, Sensors.pv_controller)

        # self.start_limelight_pose = Sensors.limelight_controller.get_estimated_robot_pose()[0].toPose2d()
        # self.start_robot_pose = Sensors.odometry.get_robot_pose()

    def robotPeriodic(self):
        Robot.drivetrain.logger_periodic()
        Sensors.odometry.update()
        SmartDashboard.putString("ODOM", str(Robot.drivetrain.odometry.getPose()))
        SmartDashboard.putString("FDOM", str(Sensors.odometry.get_robot_pose()))
        SmartDashboard.putString(
            "EDOM", str(Robot.drivetrain.odometry_estimator.getEstimatedPosition())
        )
        try:
            SmartDashboard.putString(
                "PHOTON", str(Sensors.pv_controller.get_estimated_robot_pose())
            )
        except Exception:
            pass

        pose = Robot.drivetrain.odometry_estimator.getEstimatedPosition()
        pv_pose = Sensors.pv_controller.get_estimated_robot_pose()

        SmartDashboard.putNumberArray(
            "RobotPoseAdvantage", [pose.X(), pose.Y(), pose.rotation().radians()]
        )

        try:
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
            "n_front_left: ",
            Robot.drivetrain.n_front_left.m_move.get_sensor_position()
            * (-1 if Robot.drivetrain.n_front_left.drive_reversed else 1),
        )
        SmartDashboard.putNumber(
            "n_front_right: ",
            Robot.drivetrain.n_front_right.m_move.get_sensor_position()
            * (-1 if Robot.drivetrain.n_front_right.drive_reversed else 1),
        )
        SmartDashboard.putNumber(
            "n_back_left: ",
            Robot.drivetrain.n_back_left.m_move.get_sensor_position()
            * (-1 if Robot.drivetrain.n_back_left.drive_reversed else 1),
        )
        SmartDashboard.putNumber(
            "n_back_right: ",
            Robot.drivetrain.n_back_right.m_move.get_sensor_position()
            * (-1 if Robot.drivetrain.n_back_right.drive_reversed else 1),
        )

        SmartDashboard.putNumber(
            "gyro_angle: ", math.degrees(Robot.drivetrain.gyro.get_robot_heading())
        )

    def teleopInit(self):
        commands2.CommandScheduler.getInstance().schedule(
            command.DrivetrainZero(Robot.drivetrain).andThen(
                command.DriveSwerveCustom(Robot.drivetrain)
            )
        )

    def teleopPeriodic(self):
        pass

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
