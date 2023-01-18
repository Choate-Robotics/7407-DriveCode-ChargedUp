import commands2
import wpilib
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
from robotpy_toolkit_7407.sensors.limelight import Limelight
from oi.OI import OI
from wpilib import SmartDashboard
import command
from robot_systems import Robot

class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface
        OI.init()
        OI.map_controls()
        # period = .03
        # commands2.CommandScheduler.getInstance().setPeriod(period)

        # self.gyro = PigeonIMUGyro_Wrapper(10)
        # # Target is .46272 meters above ground

        # self.gyro.reset_angle()

        # self.limelight = Limelight(cam_height=0, cam_angle=0, robot_ip="10.74.07.2")
        # SmartDashboard.init()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        # botpose = self.limelight.get_bot_pose(round_to=2)
        # if botpose:
        #     print(botpose)
        #     SmartDashboard.putString("botpose_x", str(botpose[0]))
        #     SmartDashboard.putString("botpose_y", str(botpose[1]))
        #     SmartDashboard.putString("botpose_z", str(botpose[2]))

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
    # Robot.robotInit(Robot())
