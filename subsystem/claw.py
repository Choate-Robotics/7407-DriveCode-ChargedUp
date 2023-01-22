import math
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.sensors.limit_switches.limit_switch import LimitSwitch
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveDrivetrain
import config
import ctre
import constants
import wpilib

class Claw(Subsystem):

    claw_motor: SparkMax = SparkMax(config.claw_motor_extend_id)
    claw_close_piston: wpilib.DoubleSolenoid = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 4, 5)
    claw_motor_initialized: bool = False

    def init(self):
        self.claw_motor.init()
        self.zero()
        self.claw_motor_initialized = True

    def set_angle(self, pos: float):
        """
        Set the angle of the claw

        Args:
            pos (float): Angle in radians (0, 2pi)
        """
        self.claw_motor.set_target_position(
            (pos / (math.pi * 2)) * constants.claw_gear_ratio  # 80 rotations per 360 degrees
        )   

    def get_angle(self):
        # Return through bore encode value 
        return self.claw_motor.get_sensor_position() / constants.claw_gear_ratio


    def zero(self):
        self.claw_motor.set_sensor_position(0)
        self.set_angle(0)

    # def set claw motor speed
    def set_claw_speed(self, speed: float):
        """
        Set the speed of the claw motor

        Args:
            speed (float): Speed from -1 to 1
        """
        self.claw_motor.set_raw_output(speed)
        self.raw_output = speed

    def open_claw(self):
        # Set distance forward (closes claw)
        self.claw_close_piston.set(wpilib.DoubleSolenoid.Value.kForward)
        self.set_claw_speed(constants.claw_motor_speed)
        self.is_claw_compressed = True

    def close_claw(self):
        self.claw_close_piston.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.set_claw_speed(0)
        self.is_claw_compressed = False