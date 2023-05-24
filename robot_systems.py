import wpilib
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveGyro

import subsystem
from sensors import FieldOdometry, PV_Cameras
from leds import ALEDS

class Robot:
    arm = subsystem.Arm()
    drivetrain = subsystem.Drivetrain()
    climber = subsystem.Climber()
    intake = subsystem.Intake()
    grabber = subsystem.Grabber()


class Pneumatics:
    compressor = wpilib.Compressor(31, wpilib.PneumaticsModuleType.REVPH)

    @classmethod
    def get_compressor(cls):
        return cls.compressor.enabled(), cls.compressor.getCurrent()


class Sensors:
    pv_controller: PV_Cameras = None
    odometry: FieldOdometry = FieldOdometry(Robot.drivetrain, None)
    gyro: SwerveGyro
    
class Leds:
    Default = ALEDS(0, 60)
