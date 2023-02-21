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

CLIMBER_CONFIG = SparkMaxConfig(0.01, 0, 0.01, output_range=(-.3, .3), idle_mode=SparkMaxConfig.idle_mode.kBrake)

@dataclass
class Climber(Subsystem):
    pneumatics: DoubleSolenoidPiston
    latch: DoubleSolenoidPiston
    climber_motor: SparkMax = SparkMax(config.climber_motor_id, config=CLIMBER_CONFIG)
    climber_active: bool = False
    latch_enabled: bool = False


    def init(self):
        self.climber_motor.init()
        self.climber_motor.motor.setOpenLoopRampRate(.2)
        self.pneumatics = DoubleSolenoidPiston(config.compressor, config.climber_forwardChannel, config.climber_reverseChannel)
        self.latch = DoubleSolenoidPiston(config.compressor, config.latch_forwardChannel, config.latch_reverseChannel)

        self.climber_active = False
        self.pivot_threshold = constants.climber_pivot_threshold
        self.pivot_speed = constants.climber_pivot_speed

    def climber_deploy(self):
        self.pneumatics.extend()
        self.climber_active = True
    
    def climber_disable(self):
        self.pneumatics.retract()
        self.climber_active = False
        
    def latch_enable(self):
        self.latch.extend()
        self.latch_enabled = True
        
    def latch_disable(self):
        self.latch.retract()
        self.latch_enabled = False

    def zero(self):
        self.climber_motor.set_sensor_position(0)

    def set_motor_angle(self, pos: radians):
        self.climber_motor.set_target_position(
            (pos / (2 * math.pi)) * constants.climber_motor_gear_ratio
        )
    
    def get_angle(self):
        return self.climber_motor.get_sensor_position()