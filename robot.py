import commands2
import wpilib
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
from robotpy_toolkit_7407.sensors.limelight import Limelight
from robotpy_toolkit_7407 import Subsystem
from oi.OI import OI
from wpilib import SmartDashboard
import command
import constants
from robot_systems import Robot

class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface
        OI.init()
        OI.map_controls()
    
        commands2.CommandScheduler.getInstance().setPeriod(constants.period)

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    # Initialize subsystems

    # Pneumatics

    def teleopInit(self):
        commands2.CommandScheduler.getInstance().schedule(
            command.DrivetrainZero(Robot.drivetrain)
        )
        pass

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
