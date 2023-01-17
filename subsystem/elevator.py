import wpilib
from robotpy_toolkit_7407 import subsystem
from robotpy_toolkit_7407.motors.revmotor import SparkMax
from robotpy_toolkit_7407.utils.units import meters

import constants

class Elevator(Subsystem):
    extendMotor: SparkMax = SparkMax(0)#put the right CAN ID here
    rotationMotorRight: SparkMax = SparkMax(0, inverted = True)#put the right CAN ID here
    rotationMotorLeft: SparkMax = SparkMax(0, inverted = False)#put the right CAN ID here
    initialized: bool = False
    def init(self):
        self.motors.init()

    def set_length(self, h: meters):
        pass

    def get_length(self):
        pass

    def zero_elevator(self):
        pass

    def set_speed(self):
        pass

    def bottomed_out(self):
        pass

    def full_extend(self):
        pass

    
    
    
    