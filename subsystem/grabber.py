import math

import rev
import wpilib
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.utils.units import radians

import config
import constants

WRIST_CONFIG = SparkMaxConfig(
    0.05, 0, 0.004, 0.00017, idle_mode=rev.CANSparkMax.IdleMode.kBrake
)


class Grabber(Subsystem):
    wrist: SparkMax = SparkMax(18, inverted=False, config=WRIST_CONFIG)

    claw_motor: SparkMax = SparkMax(12, inverted=False)
    claw_grabber: wpilib.DoubleSolenoid = wpilib.DoubleSolenoid(
        config.pneumatics_control_module, wpilib.PneumaticsModuleType.REVPH, 0, 1
    )

    claw_motor_initialized: bool = False
    claw_compressed: bool = False
    claw_open: bool = False

    wrist_abs_encoder = None

    distance_sensor: rev.AnalogInput = None

    def init(self):
        self.wrist.init()
        self.claw_motor.init()
        self.claw_motor_initialized = True
        self.distance_sensor = self.claw_motor.motor.getAnalog()

        self.wrist_abs_encoder = self.wrist.motor.getAbsoluteEncoder(
            rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )

        self.wrist.motor.setSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kForward,
            self.wrist_angle_to_motor_rotations(constants.wrist_max_rotation),
        )

        self.wrist.motor.setSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kReverse,
            -self.wrist_angle_to_motor_rotations(constants.wrist_min_rotation),
        )

        self.zero_wrist()

    def get_detected(self):
        return 0.5 < self.distance_sensor.getVoltage() < 0.9

    def get_detected_farther_away(self):
        return 0.4 < self.distance_sensor.getVoltage() < 0.9

    def set_angle(self, pos: float):
        """
        Set the angle of the claw

        Args:
            pos (float): Angle in radians (0, 2pi)
        """
        self.wrist.set_target_position(
            (pos / (math.pi * 2)) * constants.wrist_gear_ratio
        )

    def get_angle(self):
        """
        Get the angle of the claw
        :return: Angle in radians (0, 2pi)
        :rtype: float
        """
        return (
            (self.wrist.get_sensor_position() / constants.wrist_gear_ratio)
            * math.pi
            * 2
        )

    def set_output(self, output: float):
        """
        Set the output of the claw motor

        Args:
            output (float): Speed from -1 to 1
        """
        self.claw_motor.set_raw_output(output)

    def open_claw(self):
        self.claw_grabber.set(wpilib.DoubleSolenoid.Value.kForward)
        self.claw_open = True

    def close_claw(self):
        self.claw_grabber.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.claw_open = False

    def engage_claw(self):
        # Set distance forward (closes claw)
        self.open_claw()
        print("Setting claw speed.")
        self.set_output(config.claw_motor_speed)

    def disengage_claw(self):
        self.close_claw()
        self.claw_motor.set_target_velocity(0)

    def is_at_angle(self, angle: radians):
        wrist_threshold = math.radians(2)
        return abs(self.get_angle() - angle) < wrist_threshold

    def zero_wrist(self):
        """Sets the shoulder to the zero position (no extension)"""
        abs_encoder_position: float = self.wrist_abs_encoder.getPosition()
        if abs_encoder_position > 0.5:
            abs_encoder_position = -(1 - abs_encoder_position)
        encoder_difference: float = abs_encoder_position - 0
        motor_position: float = encoder_difference * constants.wrist_gear_ratio
        self.wrist.set_sensor_position(-motor_position)
        self.wrist.set_target_position(-motor_position)

    @staticmethod
    def wrist_angle_to_motor_rotations(angle: radians):
        """returns the scale of angle in radians to the shoulder motor rotations

        Args:
            angle (float): the angle in radians

        Returns:
            float: a float of the rotations
        """
        return (angle / (2 * math.pi)) * constants.wrist_gear_ratio

    def stop(self):
        self.wrist.set_raw_output(0)
