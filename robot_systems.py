import subsystem
from sensors import FieldOdometry, PV_Cameras
from robotpy_toolkit_7407.sensors.limelight import LimelightController, Limelight


class Robot:
    drivetrain = subsystem.Drivetrain()
    pass


class Pneumatics:
    pass


class Sensors:
    odometry: FieldOdometry
    limelight_front: Limelight
    limelight_controller: LimelightController
    pv_controller: PV_Cameras
