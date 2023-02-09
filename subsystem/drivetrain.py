import math
from dataclasses import dataclass

import rev
from ctre import CANCoder
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.subsystem_templates.drivetrain import (
    SwerveDrivetrain,
    SwerveGyro,
    SwerveNode,
)
from wpimath.geometry import Pose2d

import constants
from oi.keymap import Keymap
from robot_systems import Sensors
from units.SI import (
    meters,
    meters_per_second,
    radians,
    radians_per_second,
    radians_to_rotations,
    rotations_to_radians,
)

TURN_CONFIG = SparkMaxConfig(
    0.2, 0, 0.003, 0.00015, (-0.5, 0.5), rev.CANSparkMax.IdleMode.kBrake
)
MOVE_CONFIG = SparkMaxConfig(
    0.00005, 0, 0.0004, 0.00017, idle_mode=rev.CANSparkMax.IdleMode.kBrake
)


@dataclass
class SparkMaxSwerveNode(SwerveNode):
    m_move: SparkMax
    m_turn: SparkMax
    encoder: CANCoder
    absolute_encoder_zeroed_pos: radians = 0

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()
        self.zero()

    def zero(self):
        current_pos: radians = (
            math.radians(self.encoder.getAbsolutePosition())
            - self.absolute_encoder_zeroed_pos
        )

        self.m_turn.set_sensor_position(
            current_pos * radians_to_rotations * constants.drivetrain_turn_gear_ratio
        )
        self.set_motor_angle(current_pos)
        self.m_move.set_sensor_position(0)
        self.m_move.set_target_position(0)

    def raw_output(self, power):
        self.m_move.set_raw_output(power)

    def set_motor_angle(self, pos: radians):
        self.m_turn.set_target_position(
            pos * radians_to_rotations * constants.drivetrain_turn_gear_ratio
        )

    def direct_set_motor_angle(self, pos: radians):
        self.m_turn.set_target_position(
            pos * radians_to_rotations * constants.drivetrain_turn_gear_ratio
        )

    def set_motor_velocity(self, vel: meters_per_second):
        self.m_move.set_target_velocity(vel * constants.drivetrain_move_gear_ratio)

    def get_motor_velocity(self) -> radians_per_second:
        return (
            self.m_move.get_sensor_velocity()
            / constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        )

    def get_drive_motor_traveled_distance(self) -> meters:
        sensor_position = -1 * self.m_move.get_sensor_position()

        return (
            sensor_position
            / constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        )

    def get_turn_motor_angle(self) -> radians:
        return (
            self.m_turn.get_sensor_position()
            * rotations_to_radians
            / constants.drivetrain_turn_gear_ratio
        )


class Drivetrain(SwerveDrivetrain):
    n_front_left = SparkMaxSwerveNode(
        SparkMax(16, config=MOVE_CONFIG),
        SparkMax(15, config=TURN_CONFIG),
        CANCoder(24),
        absolute_encoder_zeroed_pos=math.radians(354.023 + 270 - 360),
    )
    n_front_right = SparkMaxSwerveNode(
        SparkMax(14, config=MOVE_CONFIG),
        SparkMax(13, config=TURN_CONFIG),
        CANCoder(23),
        absolute_encoder_zeroed_pos=math.radians(13.535 + 270),
    )
    n_back_left = SparkMaxSwerveNode(
        SparkMax(3, config=MOVE_CONFIG),
        SparkMax(4, config=TURN_CONFIG),
        CANCoder(21),
        absolute_encoder_zeroed_pos=math.radians(42.539 + 270),
    )
    n_back_right = SparkMaxSwerveNode(
        SparkMax(5, config=MOVE_CONFIG),
        SparkMax(6, config=TURN_CONFIG),
        CANCoder(22),
        absolute_encoder_zeroed_pos=math.radians(48.603 + 270),
    )

    gyro: SwerveGyro = Sensors.gyro
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    track_width: meters = constants.track_width
    max_vel: meters_per_second = constants.drivetrain_max_vel
    max_angular_vel: radians_per_second = constants.drivetrain_max_angular_vel
    deadzone_velocity: meters_per_second = 0.01
    deadzone_angular_velocity: radians_per_second = math.radians(5)
    start_angle: radians = 0
    start_pose: Pose2d = Pose2d(
        0.0254 * (40.45 + 17.625) + constants.track_width / 2,
        0.0254 * 42.19,
        math.radians(start_angle),
    )
    gyro_start_angle: radians = start_angle
    gyro_offset: radians = math.radians(0)
