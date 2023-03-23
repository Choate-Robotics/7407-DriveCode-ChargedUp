import math

import rev
import wpilib
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.sensors.limit_switches import MagneticLimitSwitch
from wpimath.geometry import Pose3d

import config
import constants
from units.SI import meters, radians

# importing packages

SHOULDER_CONFIG = SparkMaxConfig(
    0.006, 0, 1, 0.2, (-0.5, 0.5), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)
ELEVATOR_CONFIG = SparkMaxConfig(
    1.5, 0, 0.004, 0.00017, (-1, 1), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)


class Arm(Subsystem):
    motor_extend: SparkMax = SparkMax(
        config.elevator_motor_extend_id, config=ELEVATOR_CONFIG, inverted=True
    )
    arm_rotation_motor: SparkMax = SparkMax(
        config.elevator_main_rotation_motor_id, inverted=True, config=SHOULDER_CONFIG
    )
    arm_rotation_follower_motor: SparkMax = SparkMax(
        config.elevator_secondary_rotation_motor_id,
        inverted=True,
        config=SHOULDER_CONFIG,
    )
    brake: wpilib.DoubleSolenoid = wpilib.DoubleSolenoid(
        31, wpilib.PneumaticsModuleType.REVPH, 2, 3
    )

    initialized: bool = False
    brake_enabled: bool = False
    pose: Pose3d
    rotation_override: bool = False  # if true, the rotation limits will be ignored
    extension_override: bool = False  # if true, the extension limits will be ignored
    brake_override: bool = False  # if true, the brake will not be enabled or disabled
    disable_rotation: bool = False
    intake_up: bool = True

    def __init__(self):
        super().__init__()

        self.angle = None
        self.length = None
        self.distance_sensor = None
        self.abs_encoder = None
        self.elevator_top_sensor = None
        self.elevator_bottom_sensor: MagneticLimitSwitch | None = None

    def init(self):  # initializing motors
        """initializes the motors"""
        self.motor_extend.init()
        self.arm_rotation_motor.init()
        self.arm_rotation_follower_motor.init()
        self.arm_rotation_follower_motor.motor.follow(
            self.arm_rotation_motor.motor, True
        )

        self.elevator_bottom_sensor = MagneticLimitSwitch(5)

        self.elevator_top_sensor = self.motor_extend.motor.getForwardLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen
        )

        self.abs_encoder = self.arm_rotation_follower_motor.motor.getAbsoluteEncoder(
            rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )
        # assuem start zero,
        self.motor_extend.set_sensor_position(0)

    def enable_brake(self) -> bool:
        """enables the brake"""
        if self.brake_override:
            return False
        else:
            self.brake_enabled = True
            self.brake.set(wpilib.DoubleSolenoid.Value.kForward)
            return True

    def disable_brake(self) -> bool:
        """disables the brake"""
        if self.brake_override:
            return False
        else:
            self.brake_enabled = False
            self.brake.set(wpilib.DoubleSolenoid.Value.kReverse)
            return True

    def enable_brake_override(self) -> None:
        """enables the brake override"""
        self.brake_override = True

    def disable_brake_override(self) -> None:
        """disables the brake override"""
        self.brake_override = False

    def enable_rotation_override(self) -> None:
        """enables the rotation override"""
        self.rotation_override = True

    def disable_rotation_override(self) -> None:
        """disables the rotation override"""
        self.rotation_override = False

    def get_pose(self) -> Pose3d:
        """Gets the pose of the arm"""
        return self.pose

    def set_pose(self, pose: Pose3d) -> None:
        """sets the pose of the arm"""
        self.pose = pose

    def is_at_length(self, length) -> bool:
        length_threshold = 0.05
        return abs(length - self.get_length()) < length_threshold

    def is_at_shoulder_rotation(self, angle):
        rotation_threshold = math.radians(2)
        return abs(angle - self.get_rotation()) < rotation_threshold

    def is_at_position(self, length, angle) -> bool:
        """checks if the arm is at the desired position"""
        return self.is_at_length(length) and self.is_at_shoulder_rotation(angle)

    @staticmethod
    def shoulder_angle_to_motor_rotations(angle: float):
        """returns the scale of angle in radians to the shoulder motor rotations

        Args:
            angle (float): the angle in radians

        Returns:
            float: a float of the rotations
        """
        return (angle * constants.elevator_rotation_gear_ratio) / (2 * math.pi)

    @staticmethod
    def abs_encoder_to_rad(encoder_value: float):
        """converts the encoder value to radians"""
        return encoder_value * (2 * math.pi)

    def stop(self) -> None:
        """stops the elevator"""
        self.motor_extend.set_raw_output(0)
        self.arm_rotation_motor.set_raw_output(0)

    def hard_stop(self) -> None:
        """stops the elevator and enables the brake"""
        self.stop()
        self.enable_brake()

    def set_length(self, dist: meters) -> None:  # set arm extension
        """Sets the length of the elevator to the given meters, returns true if the elevator is at the given meters, float of the meters the elevator is at if it is not at the given meters"""
        length = 1 / constants.elevator_length_per_rotation
        self.motor_extend.set_target_position(length)

    def shoulder_rotation_limits(self, angle: radians) -> bool:
        """Returns if the given radians are within the soft limits of the shoulder"""
        # returns if the angle is within the soft limits
        if self.rotation_override:
            return True
        else:
            return (
                constants.shoulder_max_rotation
                >= angle
                >= -constants.shoulder_min_rotation
            )

    def set_rotation(self, angle: radians):  # set arm rotation
        """Sets the rotation of the shoulder to the given radians, returns true if the shoulder is at the given radians, float of the radians the shoulder is at if it is not at the given radians"""

        # print("RADIANS: " + str(radians))
        def set_arm_angle(aligned_angle: radians):
            rotations = (
                aligned_angle / (2 * math.pi)
            ) * constants.elevator_rotation_gear_ratio
            self.arm_rotation_motor.set_target_position(rotations)
            # self.rotation_PID.setReference(1, rev.CANSparkMax.ControlType.kVelocity)

        # if the rotation is within the soft limits, set the rotation to the given angle
        if not self.disable_rotation:
            set_arm_angle(angle)
            return True
        return False

    def get_length(self) -> float:  # returns arm extension
        """Gets the length of the elevator in meters"""
        return self.motor_extend.get_sensor_position() / (
            1 / constants.elevator_length_per_rotation
        )

    def get_rotation(self) -> float:
        """Gets the rotation of the shoulder in radians"""
        return (
            self.arm_rotation_motor.get_sensor_position()
            / constants.elevator_rotation_gear_ratio
        ) * (2 * math.pi)

    def get_rotation_radians_abs(self) -> float:
        """Gets the rotation of the shoulder in rotations from the absolute encoder"""
        return self.abs_encoder_to_rad(self.abs_encoder.getPosition())

    def get_rotation_abs(self) -> float:
        """Gets the rotation of the shoulder in rotations from the absolute encoder"""
        return self.abs_encoder.getPosition()

    def zero_elevator_length(self) -> None:
        """Sets the elevator to the zero position (no rotation)"""
        self.motor_extend.set_target_position(
            self.motor_extend.get_sensor_position() + 0.005
        )

    def zero_elevator_rotation(self) -> None:
        """Sets the shoulder to the zero position (no extension)"""
        abs_encoder_position: float = self.abs_encoder.getPosition()

        if abs_encoder_position > 0.5:
            abs_encoder_position = -(1 - abs_encoder_position)

        encoder_difference: float = (
            abs_encoder_position - constants.elevator_initial_rotation
        )

        motor_position: float = (
            encoder_difference * constants.elevator_rotation_gear_ratio
        )

        self.arm_rotation_motor.set_sensor_position(motor_position)
        self.arm_rotation_motor.set_target_position(motor_position)

        # self.arm_rotation_motor.motor.setSoftLimit(
        #     rev.CANSparkMax.SoftLimitDirection.kForward,
        #     self.shoulder_angle_to_motor_rotations(constants.shoulder_max_rotation),
        # )
        #
        # self.arm_rotation_motor.motor.setSoftLimit(
        #     rev.CANSparkMax.SoftLimitDirection.kReverse,
        #     -self.shoulder_angle_to_motor_rotations(constants.shoulder_min_rotation),
        # )

    def extend_max_elevator(self) -> None:
        """Sets the elevator to the max position (no rotation)"""
        self.set_length(constants.max_elevator_height_delta)

    def update_pose(self) -> None:
        """Updates the pose of the arm using the encoder values and rotation of the elevator"""
        self.angle = self.get_rotation()
        self.length = self.get_length()
