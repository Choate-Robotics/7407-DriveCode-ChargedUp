import math

from robotpy_toolkit_7407.utils.units import meters
from wpimath.geometry import Pose2d
from ctre import CANCoder
import rev
import wpilib
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNode, SwerveDrivetrain, SwerveGyro
from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper

from dataclasses import dataclass

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
    encoder_zeroed_absolute_pos: float = 0
    drive_reversed: bool = False
    turn_reversed: bool = False

    def init(self):
        super().init()
        self.m_move.init()
        self.m_turn.init()

    # make the turn motor set their sensor 0 to current horizontal thingy
    def zero(self):
        diff = self.encoder_zeroed_absolute_pos - self.encoder.getAbsolutePosition()
        diff_rad = diff * 2 * math.pi

        current_position_rad = self.get_current_motor_angle()

        real_position = current_position_rad - diff_rad
        self.m_turn.set_sensor_position(
            (real_position / (2 * math.pi) * constants.drivetrain_turn_gear_ratio)
        )

    # reposition the wheels
    def set_motor_angle(self, pos: float):
        if self.turn_reversed:
            pos *= -1
        self.m_turn.set_target_position(
            (pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio
        )

    def direct_set_motor_angle(self, pos: float):
        self.m_turn.set_target_position(
            (pos / (2 * math.pi)) * constants.drivetrain_turn_gear_ratio
        )

    def get_current_motor_angle(self) -> float:
        return self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio

    # rotate the wheel so the robot moves
    def set_motor_velocity(self, vel: float):
        if self.drive_reversed:
            vel *= -1
        self.m_move.set_target_velocity(vel * constants.drivetrain_move_gear_ratio)

    def get_motor_velocity(self) -> float:
        return self.m_move.get_sensor_velocity() / constants.drivetrain_move_gear_ratio

    def get_drive_motor_traveled_distance(self) -> float:  # in meters
        return self.m_move.get_sensor_position() / constants.drivetrain_move_gear_ratio

    def get_turn_motor_angle(self) -> float:  # Radians
        return self.m_turn.get_sensor_position() / constants.drivetrain_turn_gear_ratio


class Drivetrain(SwerveDrivetrain):
    n_00 = SparkMaxSwerveNode(
        SparkMax(7, config=MOVE_CONFIG),
        SparkMax(8, config=TURN_CONFIG),
        wpilib.AnalogEncoder(1),
        encoder_zeroed_absolute_pos=0.990,
        turn_reversed=True,
        drive_reversed=True,
    )
    n_01 = SparkMaxSwerveNode(
        SparkMax(1, config=MOVE_CONFIG),
        SparkMax(2, config=TURN_CONFIG),
        wpilib.AnalogEncoder(0),
        encoder_zeroed_absolute_pos=0.578,
        turn_reversed=True,
    )
    n_10 = SparkMaxSwerveNode(
        SparkMax(5, config=MOVE_CONFIG),
        SparkMax(6, config=TURN_CONFIG),
        wpilib.AnalogEncoder(2),
        encoder_zeroed_absolute_pos=0.58,
        turn_reversed=True,
    )
    n_11 = SparkMaxSwerveNode(
        SparkMax(3, config=MOVE_CONFIG),
        SparkMax(4, config=TURN_CONFIG),
        wpilib.AnalogEncoder(3),
        encoder_zeroed_absolute_pos=0.414,
        turn_reversed=True,
        drive_reversed=True,
    )

    gyro = PigeonIMUGyro_Wrapper(13)
    axis_dx = Keymap.Drivetrain.DRIVE_X_AXIS
    axis_dy = Keymap.Drivetrain.DRIVE_Y_AXIS
    axis_rotation = Keymap.Drivetrain.DRIVE_ROTATION_AXIS
    axis_y2 = Keymap.Drivetrain.DRIVE_Y2_AXIS
    track_width: float = constants.track_width
    max_vel: float = constants.drivetrain_max_vel
    max_angular_vel: float = constants.drivetrain_max_angular_vel
    deadzone_velocity: float = 0.01
    deadzone_angular_velocity: float = math.radians(5)
    start_pose: Pose2d = Pose2d(0, 0, 0)
