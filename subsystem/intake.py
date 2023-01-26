import wpilib
import constants
import config
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.pneumatics.pistons import DoubleSolenoidPiston
from dataclasses import dataclass

@dataclass
class Intake(Subsystem):
    intake_motor: SparkMax
    intake_piston: DoubleSolenoidPiston

    intake_speed: float
    intake_down: bool

    def init(self):
        self.intake_motor = SparkMax(config.intake_motor_id)
        self.intake_motor.init()

        self.intake_piston = DoubleSolenoidPiston(config.intake_piston_module, config.intake_piston_forwardChannel, config.intake_piston_reverseChannel)

        self.intake_down = False
        self.intake_speed = constants.intake_speed
    
    def intake_enable(self):
        self.intake_piston.extend()
        self.intake_down = True
        self.intake_motor.set_raw_output(self.intake_speed)
    
    def intake_disable(self):
        self.intake_piston.retract()
        self.intake_down = False
        self.intake_motor.set_raw_output(0)
        