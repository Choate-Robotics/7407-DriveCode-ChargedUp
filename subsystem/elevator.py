import rev
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors import SparkMax, SparkMaxConfig

import config
import constants
from units.SI import meters

ELEVATOR_CONFIG = SparkMaxConfig(
    1.5, 0, 0.004, 0.00017, (-1, 1), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)


class Elevator(Subsystem):
    motor_extend: SparkMax = SparkMax(
        config.elevator_motor_extend_id, config=ELEVATOR_CONFIG, inverted=True
    )

    extension_override: bool = False
    disable_extension: bool = False

    def __init__(self):
        super().__init__()

        self.distance_sensor = None
        self.elevator_top_sensor = None
        self.elevator_bottom_sensor = None
        self.length = None

    def init(self):
        self.motor_extend.init()

        self.elevator_bottom_sensor = self.motor_extend.motor.getReverseLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen
        )

        self.elevator_top_sensor = self.motor_extend.motor.getForwardLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen
        )

    def is_at_length(self, length) -> bool:
        length_threshold = 0.05
        return abs(length - self.get_length()) < length_threshold

    def set_length(self, distance: meters) -> None:
        """
        Sets the length of the elevator to the given meters
        """
        if not self.disable_extension:
            length = 1 / constants.elevator_length_per_rotation
            self.motor_extend.set_target_position(length)

    def get_length(self) -> float:  # returns arm extension
        """Gets the length of the elevator in meters"""
        return self.motor_extend.get_sensor_position() / (
            1 / constants.elevator_length_per_rotation
        )
