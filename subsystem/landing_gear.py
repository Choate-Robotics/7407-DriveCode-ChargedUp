import rev
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import SparkMax, SparkMaxConfig

LANDING_CONFIG_BRAKE_ON = SparkMaxConfig(
    0.2, 0, 0.003, 0.00015, (-0.5, 0.5), rev.CANSparkMax.IdleMode.kBrake
)
LANDING_CONFIG_BRAKE_OFF = SparkMaxConfig(
    0.2, 0, 0.003, 0.00015, (-0.5, 0.5), rev.CANSparkMax.IdleMode.kCoast
)


class LandingGear(Subsystem):
    motor_extend: SparkMax = SparkMax(8, config=LANDING_CONFIG_BRAKE_ON)

    def init(self):
        self.motor_extend.init()
        self.motor_extend.set_sensor_position(0)

    def deploy(self):
        self.motor_extend.set_raw_output(0.2)

    def release(self):
        self.motor_extend.set_raw_output(0)
        self.motor_extend._set_config(LANDING_CONFIG_BRAKE_OFF)
