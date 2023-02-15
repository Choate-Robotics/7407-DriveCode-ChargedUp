import wpilib
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveGyro

import subsystem
from sensors import FieldOdometry, PV_Cameras


class Robot:
    arm = subsystem.Arm()
    drivetrain = subsystem.Drivetrain()
    intake = subsystem.Intake()


class Pneumatics:
    compressor = wpilib.Compressor(31, wpilib.PneumaticsModuleType.REVPH)

    @classmethod
    def get_compressor(cls):
        return cls.compressor.enabled(), cls.compressor.getCurrent()


class Sensors:
    odometry: FieldOdometry
    pv_controller: PV_Cameras
    gyro: SwerveGyro
