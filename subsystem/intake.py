from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import SparkMax
from robotpy_toolkit_7407.pneumatics.pistons import DoubleSolenoidPiston

import config


class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.intake_motor = SparkMax(config.intake_motor_id)

        self.intake_piston = DoubleSolenoidPiston(
            config.pneumatics_control_module,
            config.intake_piston_forwardChannel,
            config.intake_piston_reverseChannel,
        )

        self.intake_active = False
        self.intake_speed = config.default_intake_speed

    def init(self):
        self.intake_motor.init()

    def intake_enable(self):
        self.intake_piston.extend()
        self.intake_active = True
        self.intake_motor.set_raw_output(self.intake_speed)

    def intake_disable(self):
        self.intake_piston.retract()
        self.intake_active = False
        self.intake_motor.set_raw_output(0)
