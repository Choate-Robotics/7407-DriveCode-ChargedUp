import constants
import config
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.pneumatics.pistons import DoubleSolenoidPiston
from dataclasses import dataclass
from ctre import CANCoder
from wpilib import SmartDashboard

import constants
from oi.keymap import Keymap
from robotpy_toolkit_7407.utils.units import radians
import math
import rev

CLIMBER_CONFIG = SparkMaxConfig(
    0.008, 0, .0001, 0.0012, (-0.25, 0.25), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)

@dataclass
class Climber(Subsystem):
    
    pneumatics: DoubleSolenoidPiston
    latch: DoubleSolenoidPiston
    climber_motor: SparkMax = SparkMax(config.climber_motor_id, config=CLIMBER_CONFIG, inverted=False)
    climber_active: bool = False
    latch_enabled: bool = False
    pivoted: bool = False
    
    def __init__(self):
        super().__init__()
        self.pneumatics = DoubleSolenoidPiston(config.compressor, config.climber_forwardChannel, config.climber_reverseChannel)
        self.latch = DoubleSolenoidPiston(config.compressor, config.latch_forwardChannel, config.latch_reverseChannel)


    def init(self):
        self.climber_motor.init()
        self.climber_motor.set_sensor_position(0)
        self.pivot_speed = constants.climber_pivot_speed

    def climber_deploy(self):
        self.pneumatics.extend()
        self.climber_active = True
    
    def climber_disable(self):
        self.pneumatics.retract()
        self.climber_active = False
        
    def climber_reset(self):
        if SmartDashboard.getBoolean("Can Reset Climber", False):
            self.climber_disable()
        
    def latch_enable(self):
        self.latch.extend()
        self.latch_enabled = True
        
    def latch_disable(self):
        self.latch.retract()
        self.latch_enabled = False

    def zero(self):
        self.climber_motor.set_sensor_position(0)

    def set_motor(self, rotations):
        self.climber_motor.set_target_position(
            rotations * constants.climber_motor_gear_ratio
        )
            
    def pivot(self):
        if self.climber_active == True and self.pivoted == False:
            self.pivoted = True
            self.set_motor(constants.climber_pivot_rotations)
            self.latch_enable()
            
    def unpivot(self):
        if self.climber_active == True and self.pivoted == True:
            self.pivoted = False
            self.set_motor(0)
            self.latch_disable()

    def get_motor(self):
        return self.climber_motor.get_sensor_position() / constants.climber_motor_gear_ratio
    
    def is_climbed(self):
        return self.get_motor() * constants.climber_motor_gear_ratio < constants.climber_pivot_rotations + 1 and self.get_motor() * constants.climber_motor_gear_ratio > constants.climber_pivot_rotations - 1