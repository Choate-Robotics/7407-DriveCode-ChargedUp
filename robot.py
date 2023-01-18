import commands2
import wpilib
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
from robotpy_toolkit_7407.sensors.limelight import Limelight, LimelightController

import command
from oi.OI import OI
from wpilib import SmartDashboard

from robot_systems import Robot, Sensors
from sensors import FieldOdometry


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface
        OI.init()
        OI.map_controls()
        period = .03
        commands2.CommandScheduler.getInstance().setPeriod(period)

        self.gyro = PigeonIMUGyro_Wrapper(10)
        # Target is .46272 meters above ground

        self.gyro.reset_angle()

        Sensors.limelight_front = Limelight(cam_height=0, cam_angle=0, robot_ip="10.74.07.2")
        Sensors.limelight_controller = LimelightController([Sensors.limelight_front])

        Robot.drivetrain.init()

        SmartDashboard.init()

        Sensors.odometry = FieldOdometry(Robot.drivetrain, Sensors.limelight_controller)

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        bot_pose = Sensors.limelight_front.get_bot_pose(round_to=2)
        if bot_pose:
            SmartDashboard.putString("bot_pose_x", str(bot_pose[0]))
            SmartDashboard.putString("bot_pose_y", str(bot_pose[1]))
            SmartDashboard.putString("bot_pose_z", str(bot_pose[2]))
            print("LIMELIGHT POSE: ", bot_pose)

        Sensors.odometry.update()
        # print(Sensors.odometry.get_robot_pose())

    # Initialize subsystems

    # Pneumatics

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
