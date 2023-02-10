import subsystem
from sensors import FieldOdometry, PV_Cameras


class Robot:
    drivetrain = subsystem.Drivetrain()
    pass


class Pneumatics:
    pass


class Sensors:
    odometry: FieldOdometry
    pv_controller: PV_Cameras
