import math
import config
import constants
import wpilib
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax
from robotpy_toolkit_7407.pneumatics.pistons import SingleSolenoidPiston

class Claw(Subsystem):
    
    def __init__(self, claw_motor: SparkMax):
        """
        Constructor 

        Args:
            claw_motor (SparkMax): The motor that controls the claw. Pass with the correct
            can_id and turn/drive config. 
        """
        self.claw_motor = claw_motor
        self.claw_close_piston: SingleSolenoidPiston = SingleSolenoidPiston(1, wpilib.PneumaticsModuleType.REVPH, 4)
        self.claw_motor_initialized: bool = False
        self.claw_compressed: bool = False

    def init(self):
        self.claw_motor.init()
        self.zero()
        self.claw_motor_initialized = True

    def set_angle(self, pos: float):
        """
        Set the angle of the claw

        Args:
            pos (float): Angle in radians (0, 2pi)
        """
        self.claw_motor.set_target_position(
            (pos / (math.pi * 2)) * constants.claw_gear_ratio  # 80 rotations per 360 degrees
        )   

    def get_angle(self):
        # Return through bore encode value 
        return self.claw_motor.get_sensor_position() / constants.claw_gear_ratio


    def zero(self):
        self.claw_motor.set_sensor_position(0)
        self.set_angle(0)

    # def set claw motor output (speed)
    def set_claw_ouput(self, speed: float):
        """
        Set the output of the claw motor

        Args:
            output (float): Speed from -1 to 1
        """
        self.claw_motor.set_raw_output(speed)
        self.raw_output = speed

    def open_claw(self):
        # Set distance forward (closes claw)
        self.claw_close_piston.extend()
        self.set_claw_ouput(config.claw_motor_speed)
        self.claw_compressed = True

    def close_claw(self):
        self.claw_close_piston.retract()
        self.set_claw_ouput(0)
        self.claw_compressed = False