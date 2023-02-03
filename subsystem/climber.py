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
    
    climber_deployed: bool

    def init(self):
        super().init()
        self.climber_motor = SparkMax(config.climber_motor_id)
        self.climber_motor.init()

        self.l_piston = DoubleSolenoidPiston(config.l_piston_module, config.l_piston_forwardChannel, config.l_piston_reverseChannel)
        self.r_piston = DoubleSolenoidPiston(config.r_piston_module, config.r_piston_forwardChannel, config.r_piston_reverseChannel)

        self.climber_deployed = False

    def climber_active(self):
        self.l_piston.extend
        climber_deployed = True
    
    def climber_disable(self):
        self.r_piston.retract
        climber_deployed = False
    