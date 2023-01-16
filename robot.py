import math

import networktables
import ntcore

import commands2
import wpilib
from photonvision import PhotonTrackedTarget
from robotpy_apriltag import AprilTagFieldLayout, AprilTag
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
from robotpy_toolkit_7407.sensors.limelight import Limelight

from wpimath.geometry import Pose3d, Rotation3d, Translation3d, Transform3d
from oi.OI import OI

from robotpy_toolkit_7407.sensors.photonvision import PhotonOdometry, PhotonTarget, PhotonCamera

from wpilib import SmartDashboard

from networktables import NetworkTables


class Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface
        OI.init()
        OI.map_controls()
        period = .03
        commands2.CommandScheduler.getInstance().setPeriod(period)

        self.gyro = PigeonIMUGyro_Wrapper(10)

        self.gyro.reset_angle()

        NetworkTables.initialize(server=f"10.74.07.2")
        self.limelight = NetworkTables.getTable("limelight")
        SmartDashboard.init()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        botpose = self.limelight.getValue("botpose", None)
        if botpose:
            botpose = [round(x, 2) for x in botpose]
            print(botpose)
            SmartDashboard.putString("botpose_x", str(botpose[0]))
            SmartDashboard.putString("botpose_y", str(botpose[1]))
            SmartDashboard.putString("botpose_z", str(botpose[2]))

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
    wpilib.run(Robot)
    # Robot.robotInit(Robot())
