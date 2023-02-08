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
    encoder: CANCoder
    
    climber_active: bool
    absolute_encoder_zeroed_pos: float


    def init(self):
        super().init()
        self.climber_motor = SparkMax(config.climber_motor_id)
        self.climber_motor.init()

        self.l_piston = DoubleSolenoidPiston(config.l_piston_module, config.l_piston_forwardChannel, config.l_piston_reverseChannel)
        self.r_piston = DoubleSolenoidPiston(config.r_piston_module, config.r_piston_forwardChannel, config.r_piston_reverseChannel)

        self.encoder = CANCoder(config.climber_encoder_id)

        self.climber_active = False
        absolute_encoder_zeroed_pos = config.climber_motor_zero_pos

    def climber_deploy(self):
        self.l_piston.extend()
        climber_active = True
    
    def climber_disable(self):
        self.r_piston.retract()
        climber_active = False


    def zero(self):
        target_pos_rad = (
                math.radians(self.encoder.getAbsolutePosition())
                - self.absolute_encoder_zeroed_pos
        )

        self.climber_motor.set_sensor_position(
            target_pos_rad * constants.climber_motor_gear_ratio / (2 * math.pi)
        )
        self.set_motor_angle(0)

    def set_motor_angle(self, pos: radians):
        self.climber_motor.set_target_position(
            (pos / (2 * math.pi)) * constants.climber_motor_gear_ratio
        )
    
    def pivot(self):
        target_pos_rad = (
                math.radians(self.encoder.getAbsolutePosition())
                - self.absolute_encoder_pivot_pos
        )
        self.climber_motor.set_sensor_position(
            target_pos_rad * constants.climber_motor_gear_ratio / (2 * math.pi)
        )
        self.set_motor_angle(target_pos_rad)
