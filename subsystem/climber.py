from dataclasses import dataclass

import rev
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.pneumatics.pistons import DoubleSolenoidPiston

import config
import constants

CLIMBER_CONFIG = SparkMaxConfig(
    0.425, 0, 0.0001, 0.0012, (-1, 1), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)


@dataclass
class Climber(Subsystem):
    pneumatics: DoubleSolenoidPiston
    latch: DoubleSolenoidPiston
    climber_motor: SparkMax = SparkMax(
        config.climber_motor_id, config=CLIMBER_CONFIG, inverted=False
    )
    climber_active: bool = False
    latch_enabled: bool = False
    pivoted: bool = False

    def __init__(self):
        super().__init__()
        self.pneumatics = DoubleSolenoidPiston(
            config.compressor,
            config.climber_forwardChannel,
            config.climber_reverseChannel,
        )
        self.latch = DoubleSolenoidPiston(
            config.compressor, config.latch_forwardChannel, config.latch_reverseChannel
        )

    def init(self):
        self.climber_motor.init()
        self.climber_motor.set_sensor_position(0)

    def climber_deploy(self):
        self.pneumatics.extend()
        self.climber_active = True

    def climber_disable(self):
        self.pneumatics.retract()
        self.climber_active = False

    def climber_reset(self):
        self.climber_disable()

    def latch_enable(self):
        self.latch.extend()
        self.latch_enabled = True

    def latch_disable(self):
        self.latch.retract()
        self.latch_enabled = False

    def zero(self):
        self.climber_motor.set_sensor_position(0)

    def set_motor_rotations(self, rotations):
        self.climber_motor.set_target_position(rotations)

    def set_shaft_rotations(self, rotations):
        self.climber_motor.set_target_position(
            rotations * constants.climber_motor_gear_ratio
        )

    def pivot(self):
        if self.climber_active == True and self.pivoted == False:
            self.pivoted = True
            self.latch_enable()
            self.set_motor_rotations(constants.climber_pivot_rotations)

    def unpivot(self):
        if self.climber_active == True and self.pivoted == True:
            self.pivoted = False
            self.latch_disable()
            self.set_motor_rotations(0)

    def get_motor_rotations(self):
        return self.climber_motor.get_sensor_position()

    def get_shaft_rotations(self):
        return (
            self.climber_motor.get_sensor_position()
            / constants.climber_motor_gear_ratio
        )

    def is_climbed(self):
        return abs(self.get_motor_rotations() - constants.climber_pivot_rotations) < 1

    def is_at_rotation(self, target):
        print(abs(self.climber_motor.get_sensor_position() - target))
        return abs(self.climber_motor.get_sensor_position() - target) < 0.3
