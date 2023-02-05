import math

import commands2
import wpilib
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
from robotpy_toolkit_7407.sensors.limelight import Limelight
from oi.OI import OI
from robot_systems import Pneumatics, Sensors, Robot
from robotpy_toolkit_7407.sensors.limelight import Limelight, LimelightController
from wpilib import SmartDashboard

import command
from oi.OI import OI
from robot_systems import Robot, Sensors
from sensors import FieldOdometry, PV_Cameras

from autonomous import routine


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface
        OI.init()
        OI.map_controls()
        period = 0.03
        commands2.CommandScheduler.getInstance().setPeriod(period)
        Pneumatics.compressor.enableAnalog(90, 120)
        self.gyro = PigeonIMUGyro_Wrapper(10)
        # Target is .46272 meters above ground

        # Sensors.limelight_front = Limelight(
        #     cam_height=0, cam_angle=0, robot_ip="10.74.07.2"
        # )
        # Sensors.limelight_controller = LimelightController([Sensors.limelight_front])

        Robot.drivetrain.init()
        Robot.Elevator.init()
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
            SmartDashboard.putString(
                "PHOTON ANGLE", str(Sensors.pv_controller.get_estimated_robot_pose()[0][0].rotation().toRotation2d().degrees())
            )
        except Exception:
            pass

        pose = Robot.drivetrain.odometry_estimator.getEstimatedPosition()
        pose2 = Sensors.odometry.get_robot_pose()
        pv_pose = Sensors.pv_controller.get_estimated_robot_pose()

        SmartDashboard.putNumberArray(
            "RobotPoseAdvantage", [pose.X(), pose.Y(), pose.rotation().radians()]
        )

        SmartDashboard.putNumberArray(
            "RobotPoseOrig", [pose2.X(), pose2.Y(), pose2.rotation().radians()]
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
        # # botpose = self.limelight.get_bot_pose(round_to=2)
        # # if botpose:
        # #     print(botpose)
        # #     SmartDashboard.putString("botpose_x", str(botpose[0]))
        # #     SmartDashboard.putString("botpose_y", str(botpose[1]))
        # #     SmartDashboard.putString("botpose_z", str(botpose[2]))

        # SmartDashboard.putString(
        #     "N00", str(Robot.drivetrain.n_front_left.encoder.getAbsolutePosition())
        # )
        # SmartDashboard.putString(
        #     "N01", str(Robot.drivetrain.n_front_right.encoder.getAbsolutePosition())
        # )
        # SmartDashboard.putString(
        #     "N10", str(Robot.drivetrain.n_back_left.encoder.getAbsolutePosition())
        # )
        # SmartDashboard.putString(
        #     "N11", str(Robot.drivetrain.n_back_right.encoder.getAbsolutePosition())
        # )

        # SmartDashboard.putString(
        #     "N00_m",
        #     str(math.degrees(Robot.drivetrain.n_front_left.get_turn_motor_angle())),
        # )
        # SmartDashboard.putString(
        #     "N01_m",
        #     str(math.degrees(Robot.drivetrain.n_front_right.get_turn_motor_angle())),
        # )
        # SmartDashboard.putString(
        #     "N10_m",
        #     str(math.degrees(Robot.drivetrain.n_back_left.get_turn_motor_angle())),
        # )
        # SmartDashboard.putString(
        #     "N11_m",
        #     str(math.degrees(Robot.drivetrain.n_back_right.get_turn_motor_angle())),
        # )

        # SmartDashboard.putString(
        #     "N00_drive",
        #     str(
        #         math.degrees(Robot.drivetrain.n_front_left.m_move.get_sensor_position())
        #     ),
        # )
        # SmartDashboard.putString(
        #     "N01_drive",
        #     str(
        #         math.degrees(
        #             Robot.drivetrain.n_front_right.m_move.get_sensor_position()
        #         )
        #     ),
        # )
        # SmartDashboard.putString(
        #     "N10_drive",
        #     str(
        #         math.degrees(Robot.drivetrain.n_back_left.m_move.get_sensor_position())
        #     ),
        # )
        # SmartDashboard.putString(
        #     "N11_drive",
        #     str(
        #         math.degrees(Robot.drivetrain.n_back_right.m_move.get_sensor_position())
        #     ),
        # )

        # SmartDashboard.putNumber(
        #     "n_front_left: ",
        #     Robot.drivetrain.n_front_left.m_move.get_sensor_position()
        #     * (-1 if Robot.drivetrain.n_front_left.drive_reversed else 1),
        # )
        # SmartDashboard.putNumber(
        #     "n_front_right: ",
        #     Robot.drivetrain.n_front_right.m_move.get_sensor_position()
        #     * (-1 if Robot.drivetrain.n_front_right.drive_reversed else 1),
        # )
        # SmartDashboard.putNumber(
        #     "n_back_left: ",
        #     Robot.drivetrain.n_back_left.m_move.get_sensor_position()
        #     * (-1 if Robot.drivetrain.n_back_left.drive_reversed else 1),
        # )
        # SmartDashboard.putNumber(
        #     "n_back_right: ",
        #     Robot.drivetrain.n_back_right.m_move.get_sensor_position()
        #     * (-1 if Robot.drivetrain.n_back_right.drive_reversed else 1),
        # )

        SmartDashboard.putNumber(
            "gyro_angle: ", math.degrees(Robot.drivetrain.gyro.get_robot_heading())
        )

    def teleopInit(self):
        #Robot.Elevator.main_rotation_motor.set_sensor_position(0)
        #commands2.InstantCommand(command.DrivetrainZero(Robot.drivetrain))
        #commands2.InstantCommand(command.ZeroArm(Robot.Elevator))
        #SmartDashboard.putData("pose", Robot.Elevator.get_pose())
        #Robot.Elevator.zero_elevator_rotation()
        commands2.CommandScheduler.getInstance().schedule(
            command.DrivetrainZero(Robot.drivetrain).andThen(
                command.DriveSwerveCustom(Robot.drivetrain)
            )
        )
        commands2.CommandScheduler.getInstance().schedule(command.ZeroArm(Robot.Elevator))
        #commands2.CommandScheduler.getInstance().schedule(command.ArmPose(Robot.Elevator))


    def teleopPeriodic(self):
        pass
        
        #Robot.Elevator.main_rotation_motor.set_raw_output(.2)

    def autonomousInit(self):
        routine.run()
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
