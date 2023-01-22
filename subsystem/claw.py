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
    initialized: bool = False

    def __init__(self, spark1: SparkMax, spark2: SparkMax, encoder: ctre.CANCoder):

        # Intake motor
        self.drive = spark1
        # Wrist motor
        self.turn = spark2
        # Encoder value 0-42 -- through bore encoder
        self.encoder = encoder
        # Speed variable 
        self.raw_output = None
        # Claw compress settings
        self.is_claw_compressed = False

    def init(self):
        # Initialize motor
        self.claw_motor.init()
        # Zero motor
        self.zero()
        # Initialized as True
        self.initialized = True
        # Initializing pneumatic cylinder
        self.cylinder = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, 4, 5)

    def set_angle(self, pos: float):
        # Pos is value from 0-2pi
        # 80 Rotations = 360 degrees
        self.claw_motor.set_target_position(
            (pos / (math.pi * 2)) * constants.claw_gear_ratio
        )   

    def get_angle(self):
        # Return through bore encode value 
        return self.claw_motor.get_sensor_position() / constants.claw_gear_ratio

    def zero(self):
        # Setting sensor to zero
        self.claw_motor.set_sensor_position(0)
        # Passing 0 will ouput 0
        self.set_angle(0)

    # def set claw motor speed
    def set_claw_speed(self, speed: float):
        # Float from -1 to 1
        # No need to add gear ratio atm since it is just setting the raw output
        self.claw_motor.set_raw_output(speed)
        # Changing speed var, not too sure abt this, lmk if u can confirm
        self.raw_output = speed
        # Doing this instead of set_target_velocity as per 2021 code

    # def get claw motor speed
    # Being solved by datastore in self.speed

    def open_claw(self):
        # Enable pneumatic cylinder
        # Set distance forward (closes claw)
        self.cylinder.set(wpilib.DoubleSolenoid.Value.kForward)
        # Setting speed
        self.set_claw_speed(constants.claw_motor_speed)
        # Update bool
        self.is_claw_compressed = True

    def close_claw(self):
        self.cylinder.set(wpilib.DoubleSolenoid.Value.kReverse)
        # Setting speed to 0
        self.set_claw_speed(0)
        # Update bool
        self.is_claw_compressed = False