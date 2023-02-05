import math
from dataclasses import dataclass

import rev
from ctre import CANCoder
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNode, SwerveDrivetrain
from wpimath.geometry import Pose2d
from oi.keymap import Keymap

from units.SI import *

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
    start_dist: meters = 0

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()
        self.start_dist: meters = self.m_move.get_sensor_position() * drive_motor_encoder_units__to__meters

    def zero(self):
        current_pos_rad: radians = math.radians(self.encoder.getAbsolutePosition())
        zeroed_pos_rad: radians = self.absolute_encoder_zeroed_pos
        new_pos_rad: radians = current_pos_rad - zeroed_pos_rad

        new_pos_rotations: rotation_motor_encoder_units = new_pos_rad * radians_to_rotations * swerve_pod_rotations__to__rotation_motor_encoder_units

        self.m_turn.set_sensor_position(new_pos_rotations)

    def raw_output(self, power: float):
        self.m_move.set_raw_output(power)

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

    def set_motor_velocity(self, vel: meters_per_second):
        if self.drive_reversed:
            vel *= -1

        target_velocity: drive_motor_encoder_units = vel * meters__to__drive_motor_encoder_units
        self.m_move.set_target_velocity(target_velocity)

    def get_motor_velocity(self) -> radians_per_second:
        sensor_velocity: radians_per_second = self.m_move.get_sensor_velocity() * rotation_motor_encoder_units__to__swerve_pod_rotations * rotations_to_radians
        return sensor_velocity

    def get_drive_motor_traveled_distance(self) -> meters:
        drive_motor_traveled_distance: meters = (
                                                            self.m_move.get_sensor_position() * drive_motor_encoder_units__to__meters) - self.start_dist
        return drive_motor_traveled_distance

    def get_turn_motor_angle(self) -> radians:
        turn_motor_angle: radians = self.m_turn.get_sensor_position() * rotation_motor_encoder_units__to__swerve_pod_rotations * rotations_to_radians
        return turn_motor_angle


class Drivetrain(SwerveDrivetrain):
    n_00 = SparkMaxSwerveNode(
        SparkMax(16, config=MOVE_CONFIG),
        SparkMax(15, config=TURN_CONFIG),
        CANCoder(24),
        absolute_encoder_zeroed_pos=354.023
    )
    n_01 = SparkMaxSwerveNode(
        SparkMax(3, config=MOVE_CONFIG),
        SparkMax(4, config=TURN_CONFIG),
        CANCoder(21),
        absolute_encoder_zeroed_pos=42.539
    )
    n_10 = SparkMaxSwerveNode(
        SparkMax(14, config=MOVE_CONFIG),
        SparkMax(13, config=TURN_CONFIG),
        CANCoder(23),
        absolute_encoder_zeroed_pos=13.535
    )
    n_11 = SparkMaxSwerveNode(
        SparkMax(5, config=MOVE_CONFIG),
        SparkMax(6, config=TURN_CONFIG),
        CANCoder(22),
        absolute_encoder_zeroed_pos=48.603
    )

    gyro = PigeonIMUGyro_Wrapper(0)
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    track_width: meters = constants.track_width
    max_vel: meters_per_second = constants.drivetrain_max_vel
    max_angular_vel: radians_per_second = constants.drivetrain_max_angular_vel
    deadzone_velocity: meters_per_second = 0.01
    deadzone_angular_velocity: radians_per_second = math.radians(5)
    start_pose: Pose2d = Pose2d(0, 0, 0)
