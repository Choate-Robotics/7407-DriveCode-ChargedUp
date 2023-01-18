import commands2
import wpilib
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
from robotpy_toolkit_7407.sensors.limelight import Limelight
from oi.OI import OI
from wpilib import SmartDashboard

from robot_systems import Robot, Sensors
from sensors import FieldOdometry, PV_Cameras


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
        #self.limelight = Limelight(cam_height=0, cam_angle=0, robot_ip="10.74.07.2")
        #SmartDashboard.init()
        self.pv = PV_Cameras()
        # self.pv.init()

        Sensors.odometry = FieldOdometry(Robot.drivetrain)

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        botposes = self.pv.getPoses()
        if botposes:
            print(botposes[0])
            SmartDashboard.putString("botpose_x", str(botposes[0]))
            # SmartDashboard.putString("botpose_y", str(botposes[0][1]))
            # SmartDashboard.putString("botpose_z", str(botposes[0][2]))
        else:
            print("Botpose not found")

        print(botposes)

    # Initialize subsystems

    # Pneumatics

    def teleopInit(self):
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
