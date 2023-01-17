import math
import constants
import ctre
import rev
import wpilib
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNode, SwerveDrivetrain

from robotpy_toolkit_7407.utils.units import (
    deg,
    meters,
    meters_per_second,
    rad,
    radians,
    radians_per_second,
    s,
)

TURN_CONFIG = SparkMaxConfig(
    0.2, 0, 0.003, 0.00015, (-0.5, 0.5), rev.CANSparkMax.IdleMode.kBrake
)
MOVE_CONFIG = SparkMaxConfig(
    0.00005, 0, 0.0004, 0.00017, idle_mode=rev.CANSparkMax.IdleMode.kBrake
)

class SparkMaxSwerveNode(SwerveNode):
    m_move: SparkMax
    m_turn: SparkMax
    encoder: ctre.CANCoder
    encoder_zeroed_absolute_pos: float = 0
    drive_reversed: bool = False
    turn_reversed: bool = False

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()

    # make the turn motor set their sensor 0 to current horizontal thingy
    def zero(self):
        diff_rad = self.encoder_zeroed_absolute_pos - self.encoder.getAbsolutePosition()
        
        current_position_rad = self.get_current_motor_angle()

        real_position = current_position_rad - diff_rad
        self.m_turn.set_sensor_position(
            (real_position / (2 * math.pi) * constants.drivetrain_turn_gear_ratio)
        )

    # reposition the wheels
    def set_motor_angle(self, pos: radians):
        if self.turn_reversed:
            pos *= -1
        self.m_turn.set_target_position(
            (pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio
        )

    def direct_set_motor_angle(self, pos: radians):
        self.m_turn.set_target_position(
            (pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio
        )

    def get_current_motor_angle(self) -> radians:
        return self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio

    # rotate the wheel so the robot moves
    def set_motor_velocity(self, vel: meters_per_second):
        if self.drive_reversed:
            vel *= -1
        self.m_move.set_target_velocity(vel * constants.drivetrain_move_gear_ratio)

    def get_motor_velocity(self) -> radians_per_second:
        return self.m_move.get_sensor_velocity() / constants.drivetrain_move_gear_ratio

class Drivetrain(SwerveDrivetrain):
    n_00 = SparkMaxSwerveNode(
        SparkMax(7, config=MOVE_CONFIG),
        SparkMax(8, config=TURN_CONFIG),
        wpilib.AnalogEncoder(1),
        
    )
    n_01 = SparkMaxSwerveNode(
        SparkMax(1, config=MOVE_CONFIG),
        SparkMax(2, config=TURN_CONFIG),
        wpilib.AnalogEncoder(0),
    )

    n_10 = SparkMaxSwerveNode(
        SparkMax(5, config=MOVE_CONFIG),
        SparkMax(6, config=TURN_CONFIG),
        wpilib.AnalogEncoder(2),
        
    )
    n_11 = SparkMaxSwerveNode(
        SparkMax(3, config=MOVE_CONFIG),
        SparkMax(4, config=TURN_CONFIG),
        wpilib.AnalogEncoder(3),
    )



