from subsystem import Drivetrain, Claw
import sensors

class Robot:
    drivetrain = Drivetrain()
    claw = Claw()

class Pneumatics:
    pass

class Sensors:
    IR_Sensor: sensors.IR_Sensor
