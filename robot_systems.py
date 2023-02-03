from robotpy_toolkit_7407.sensors.limelight import Limelight, LimelightController

import subsystem
from sensors import FieldOdometry, PV_Cameras


class Robot:
    drivetrain = subsystem.Drivetrain()
    climber =  subsystem.Climber()
    pass


class Pneumatics:
    pass


class Sensors:
    odometry: FieldOdometry
    limelight_front: Limelight
    limelight_controller: LimelightController
    pv_controller: PV_Cameras
