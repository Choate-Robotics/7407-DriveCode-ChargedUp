import wpilib
import constants
import config
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.pneumatics.pistons import DoubleSolenoidPiston
from dataclasses import dataclass


import constants
from oi.keymap import Keymap



@dataclass
class Climber(Subsystem):
    climber_motor: SparkMax
    l_piston: DoubleSolenoidPiston
    r_piston: DoubleSolenoidPiston
    

    def init(self):
        super().init()
        self.climber_motor = SparkMax(config.climber_motor_id)
        self.climber_motor.init()

        self.l_piston = DoubleSolenoidPiston(config.l_piston_module, config.l_piston_forwardChannel, config.l_piston_reverseChannel)
        self.r_piston = DoubleSolenoidPiston(config.r_piston_module, config.r_piston_forwardChannel, config.r_piston_reverseChannel)

    def zero(self):
        current_pos_rad = (
                math.radians(self.encoder.getAbsolutePosition())
                - self.absolute_encoder_zeroed_pos
        )

        self.m_turn.set_sensor_position(
            current_pos_rad * constants.drivetrain_turn_gear_ratio / (2 * math.pi)
        )
        self.set_motor_angle(current_pos_rad)

    def raw_output(self, power):
        self.m_move.set_raw_output(power)
