import commands2
import wpilib
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
from robotpy_toolkit_7407.sensors.limelight import Limelight
from oi.OI import OI
from wpilib import SmartDashboard
from networktables import NetworkTables
from robot_systems import Robot, Sensors
from sensors import FieldOdometry, PV_Cameras
from wpimath.geometry import (
    Pose3d,
    Pose2d,
)


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface
        OI.init()
        OI.map_controls()
        #NetworkTables.initialize()
        period = .03
        commands2.CommandScheduler.getInstance().setPeriod(period)

        self.gyro = PigeonIMUGyro_Wrapper(10)
        # Target is .46272 meters above ground

        self.gyro.reset_angle()
        #self.limelight = Limelight(cam_height=0, cam_angle=0, robot_ip="10.74.07.2")
        #SmartDashboard.init()
        self.pv = PV_Cameras()
        self.testPose = Pose2d(8.689, 10.168, -0.22)

        #self.robotPoseArray = [testPose.X(), testPose.Y(), testPose.rotation().radians()]
        # self.pv.init()

        Sensors.odometry = FieldOdometry(Robot.drivetrain)

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        botposes = self.pv.getPoses()
        if botposes:
            print(botposes[0])
            #SmartDashboard.putString("botpose", str(botposes[0]))
            botpose = botposes[0][0].toPose2d()
            SmartDashboard.putNumberArray("botpose", [botpose.X(), botpose.Y(), botpose.rotation().radians()])
            # SmartDashboard.putString("botpose_y", str(botposes[0][1]))
            # SmartDashboard.putString("botpose_z", str(botposes[0][2]))
        # else:
        #     print("Botpose not found")
        #     #SmartDashboard.putString("botpose", "bruh")
        #     print(self.testPose.X(), self.testPose.Y(), self.testPose.rotation().radians())
        #     SmartDashboard.putNumberArray("botpose", [self.testPose.X(), self.testPose.Y(), self.testPose.rotation().radians()])

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
