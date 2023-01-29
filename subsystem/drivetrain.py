import logging
import math
from dataclasses import dataclass

import rev
from ctre import CANCoder
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
from robotpy_toolkit_7407.subsystem_templates.drivetrain import (
    SwerveDrivetrain,
    SwerveNode,
)
from robotpy_toolkit_7407.utils.units import (
    meters,
    meters_per_second,
    radians,
    radians_per_second,
)
from wpimath.geometry import Pose2d

import constants
from oi.keymap import Keymap

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
    drive_reversed: bool = False
    turn_reversed: bool = False

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()

    def zero(self):
        current_pos_rad = (
            math.radians(self.encoder.getAbsolutePosition())
            - self.absolute_encoder_zeroed_pos
        )

        self.m_turn.set_sensor_position(
            current_pos_rad * constants.drivetrain_turn_gear_ratio / (2 * math.pi)
        )
        self.set_motor_angle(current_pos_rad)

    def raw_output(self, power):
        self.m_move.set_raw_output(power)

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
        return (
            (self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio)
            * 2
            * math.pi
        )

    # rotate the wheel so the robot moves
    def set_motor_velocity(self, vel: meters_per_second):
        if self.drive_reversed:
            vel *= -1
        self.m_move.set_target_velocity(vel * constants.drivetrain_move_gear_ratio)

    def get_motor_velocity(self) -> radians_per_second:
        return self.m_move.get_sensor_velocity() / constants.drivetrain_move_gear_ratio

    def get_drive_motor_traveled_distance(self) -> meters:
        sensor_position = -1 * self.m_move.get_sensor_position()  # TEST
        if self.drive_reversed:
            sensor_position *= -1

        return (
            sensor_position
            / constants.drivetrain_move_gear_ratio_as_rotations_per_meter
        )

    def get_turn_motor_angle(self) -> radians:
        return (
            (self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio)
            * 2
            * math.pi
        )


class Drivetrain(SwerveDrivetrain):
    n_front_left = SparkMaxSwerveNode(
        SparkMax(16, config=MOVE_CONFIG),
        SparkMax(15, config=TURN_CONFIG),
        CANCoder(24),
        absolute_encoder_zeroed_pos=math.radians(354.023 - 180 + 270),  # +90
        drive_reversed=True,
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

    gyro = PigeonIMUGyro_Wrapper(20)
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    track_width: meters = constants.track_width
    max_vel: meters_per_second = constants.drivetrain_max_vel
    max_angular_vel: radians_per_second = constants.drivetrain_max_angular_vel
    deadzone_velocity: meters_per_second = 0.01
    deadzone_angular_velocity: radians_per_second = math.radians(5)
    start_pose: Pose2d = Pose2d(0, 0, math.radians(270))
    gyro_start_angle = 270
    gyro_offset = math.radians(0)

    def logger_periodic(self):
        logging.info(f"GYRO: {math.degrees(self.gyro.get_robot_heading())}")
        logging.info(f"ODOM: {self.odometry_estimator.getEstimatedPosition().rotation().degrees()}")
