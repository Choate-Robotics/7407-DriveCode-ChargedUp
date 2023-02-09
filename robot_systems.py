import wpilib
from robotpy_toolkit_7407.sensors.limelight import Limelight, LimelightController

import subsystem
from sensors import FieldOdometry, PV_Cameras


class Robot:
    Arm = subsystem.Arm()
    drivetrain = subsystem.Drivetrain()
    #pass


class Pneumatics:
    compressor = wpilib.Compressor(1, wpilib.PneumaticsModuleType.REVPH)

    @classmethod
    def get_compressor(cls):
        return cls.compressor.enabled(), cls.compressor.getCurrent()

class Sensors:
    odometry: FieldOdometry
    limelight_front: Limelight
    limelight_controller: LimelightController
    pv_controller: PV_Cameras
