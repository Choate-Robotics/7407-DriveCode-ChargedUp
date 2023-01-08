import commands2
import ctre
import wpilib
from wpimath.geometry import Pose3d, Translation2d, Rotation2d, Translation3d, Rotation3d

import command
import config
import constants
import robot_systems
import sensors
import subsystem
import utils
from oi.OI import OI

from robotpy_toolkit_7407.sensors.photonvision import PhotonCamera, PhotonOdometry


class Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Initialize Operator Interface
        OI.init()
        OI.map_controls()
        period = .03
        commands2.CommandScheduler.getInstance().setPeriod(period)

        self.cam = PhotonCamera("hello", Pose3d(Translation3d(0, 1, 2), Rotation3d(roll=1, pitch=2, yaw=3)), height=1, pitch=1)

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        self.cam.refresh()
        print(self.cam.latest_target)

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
