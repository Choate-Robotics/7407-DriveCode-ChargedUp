import subsystem
import sensors


class Robot:
    drivetrain = subsystem.Drivetrain()
    pass


class Pneumatics:
    pass


class Sensors:
    odometry: sensors.FieldOdometry
    pass
