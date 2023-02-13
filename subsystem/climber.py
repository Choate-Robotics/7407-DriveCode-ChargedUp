import constants
import config
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.pneumatics.pistons import DoubleSolenoidPiston
from dataclasses import dataclass
from ctre import CANCoder


import constants
from oi.keymap import Keymap
from robotpy_toolkit_7407.utils.units import radians
import math


@dataclass
class Climber(Subsystem):
    climber_motor: SparkMax
    l_piston: DoubleSolenoidPiston
    r_piston: DoubleSolenoidPiston
    
    climber_active: bool
    pivot_threshold: float


    def init(self):
        super().init()
        self.climber_motor = SparkMax(config.climber_motor_id)
        self.climber_motor.init()

        self.l_piston = DoubleSolenoidPiston(config.l_piston_module, config.l_piston_forwardChannel, config.l_piston_reverseChannel)
        self.r_piston = DoubleSolenoidPiston(config.r_piston_module, config.r_piston_forwardChannel, config.r_piston_reverseChannel)

        self.climber_active = False
        self.pivot_threshold = config.climber_pivot_threshold
        self.pivot_speed = config.climber_pivot_speed

    def climber_deploy(self):
        self.l_piston.extend()
        self.climber_active = True
    
    def climber_disable(self):
        self.r_piston.retract()
        self.climber_active = False


    def zero(self):
        self.climber_motor.set_sensor_position(0)

    def set_motor_angle(self, pos: radians):
        self.climber_motor.set_target_position(
            (pos / (2 * math.pi)) * constants.climber_motor_gear_ratio
        )
    
    def get_angle(self):
        return self.climber_motor.get_sensor_position()