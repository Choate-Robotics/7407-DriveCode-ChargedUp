import math
import constants
import ctre
from ctre import CANCoder, Pigeon2
import rev
import wpilib
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNode, SwerveDrivetrain, SwerveGyro
from oi.keymap import Keymap
from units.SI import *

# from robotpy_toolkit_7407.utils.units import (
#     deg,
#     meters,
#     meters_per_second,
#     rad,
#     radians,
#     radians_per_second,
#     s,
# )

TURN_CONFIG = SparkMaxConfig(
    0.2, 0, 0.003, 0.00015, (-0.5, 0.5), rev.CANSparkMax.IdleMode.kBrake
)
MOVE_CONFIG = SparkMaxConfig(
    0.00005, 0, 0.0004, 0.00017, idle_mode=rev.CANSparkMax.IdleMode.kBrake
)


class SparkMaxSwerveNode(SwerveNode):
    def __init__(self, m_move: SparkMax, m_turn: SparkMax, encoder: wpilib.AnalogEncoder,
                 encoder_zeroed_absolute_pos: float = 0, drive_reversed: bool = False, turn_reversed: bool = False):
        self.m_move = m_move
        self.m_turn = m_turn
        self.encoder = encoder
        self.encoder_zeroed_absolute_pos = encoder_zeroed_absolute_pos
        self.drive_reversed = drive_reversed
        self.turn_reversed = turn_reversed

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()

    # make the turn motor set their sensor 0 to current horizontal thingy
    def zero(self):
        current_pos_rad = self.encoder.getAbsolutePosition()
        zeroed_pos_rad = self.encoder_zeroed_absolute_pos
        new_pos_rad = current_pos_rad - zeroed_pos_rad

        new_pos_rotations: rotation_motor_encoder_units = new_pos_rad * radians_to_rotations * swerve_pod_rotations__to__rotation_motor_encoder_units

        self.m_turn.set_sensor_position(new_pos_rotations)

    # reposition the wheels
    def set_motor_angle(self, pos: radians):
        if self.turn_reversed:
            pos *= -1

        target_position: rotation_motor_encoder_units = pos * radians_to_rotations * swerve_pod_rotations__to__rotation_motor_encoder_units

        self.m_turn.set_target_position(
            target_position
        )

    def direct_set_motor_angle(self, pos: radians):
        target_position: rotation_motor_encoder_units = pos * radians_to_rotations * swerve_pod_rotations__to__rotation_motor_encoder_units

        self.m_turn.set_target_position(
            target_position
        )

    def get_current_motor_angle(self) -> radians:
        current_motor_angle_radians: radians = self.m_turn.get_sensor_position() * rotation_motor_encoder_units__to__swerve_pod_rotations * rotations_to_radians
        return current_motor_angle_radians

    # rotate the wheel so the robot moves
    def set_motor_velocity(self, vel: meters_per_second):
        if self.drive_reversed:
            vel *= -1

        target_velocity: drive_motor_encoder_units = vel * meters__to__drive_motor_encoder_units
        self.m_move.set_target_velocity(target_velocity)

    def get_motor_velocity(self) -> radians_per_second:
        sensor_velocity: radians_per_second = self.m_move.get_sensor_velocity() * rotation_motor_encoder_units__to__swerve_pod_rotations * rotations_to_radians
        return sensor_velocity


class Drivetrain(SwerveDrivetrain):
    n_00 = SparkMaxSwerveNode(
        SparkMax(1, config=MOVE_CONFIG),
        SparkMax(2, config=TURN_CONFIG),
        # CANCoder(1),
        wpilib.AnalogEncoder(0),
        encoder_zeroed_absolute_pos=0.578,
        turn_reversed=True,
    )
    n_01 = SparkMaxSwerveNode(
        SparkMax(3, config=MOVE_CONFIG),
        SparkMax(4, config=TURN_CONFIG),
        # CANCoder(0),
        wpilib.AnalogEncoder(3),
        encoder_zeroed_absolute_pos=0.414,
        turn_reversed=True,
        drive_reversed=True,
    )

    n_10 = SparkMaxSwerveNode(
        SparkMax(5, config=MOVE_CONFIG),
        SparkMax(6, config=TURN_CONFIG),
        # CANCoder(2),
        wpilib.AnalogEncoder(2),
        encoder_zeroed_absolute_pos=0.58,
        turn_reversed=True,
    )
    n_11 = SparkMaxSwerveNode(
        SparkMax(7, config=MOVE_CONFIG),
        SparkMax(8, config=TURN_CONFIG),
        # CANCoder(3),
        wpilib.AnalogEncoder(1),
        encoder_zeroed_absolute_pos=0.990,
        turn_reversed=True,
        drive_reversed=True,

    )

    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    track_width: meters = constants.track_width
    max_vel: meters_per_second = constants.drivetrain_max_vel
    max_angular_vel: radians_per_second = constants.drivetrain_max_angular_vel
    deadzone_velocity: meters_per_second = 0.01
    deadzone_angular_velocity: radians_per_second = 5 * degrees_per_second__to__radians_per_second
