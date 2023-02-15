import math

import rev
import wpilib
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from wpimath.geometry import Pose3d

import config
import constants
from units.SI import radians

# importing packages

SHOULDER_CONFIG = SparkMaxConfig(
    0.6, 0, 1, 0.2, (-0.3, 0.3), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)
ELEVATOR_CONFIG = SparkMaxConfig(
    1, 0, 0.004, 0.00017, (-0.5, 0.5), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)

WRIST_CONFIG = SparkMaxConfig(
    0.05, 0, 0.004, 0.00017, idle_mode=rev.CANSparkMax.IdleMode.kBrake
)


class Arm(Subsystem):  # elevator class
    # Elevator Motors
    motor_extend: SparkMax = SparkMax(
        config.elevator_motor_extend_id, config=ELEVATOR_CONFIG
    )  # motor that extends the arm
    # Shoulder Motors
    secondary_rotation_motor: SparkMax = SparkMax(
        config.elevator_secondary_rotation_motor_id,
        inverted=True,
        config=SHOULDER_CONFIG,
    )  # motor that rotates the arm
    main_rotation_motor: SparkMax = SparkMax(
        config.elevator_main_rotation_motor_id, inverted=True, config=SHOULDER_CONFIG
    )  # motor that rotates the arm
    # main_rotation_motor.__pid_controller.setSmartMotionMaxAccel(2 * constants.elevator_rotation_gear_ratio, 0)
    # Brake
    brake: wpilib.DoubleSolenoid = wpilib.DoubleSolenoid(
        31, wpilib.PneumaticsModuleType.REVPH, 14, 15
    )  # brake that holds the arm in place
    # Wrist and Claw
    wrist: SparkMax = SparkMax(18, inverted=False, config=WRIST_CONFIG)
    claw_motor: SparkMax = SparkMax(12, inverted=False)
    claw_grabber: wpilib.DoubleSolenoid = wpilib.DoubleSolenoid(
        config.pneumatics_control_module, wpilib.PneumaticsModuleType.REVPH, 0, 1
    )
    initialized: bool = False
    brake_enabled: bool = False
    pose: Pose3d
    rotation_override: bool = False  # if true, the rotation limits will be ignored
    extension_override: bool = False  # if true, the extension limits will be ignored
    brake_override: bool = False  # if true, the brake will not be enabled or disabled
    disable_extension: bool = False
    disable_rotation: bool = False
    intake_up: bool = True
    claw_motor_initialized: bool = False
    claw_compressed: bool = False
    claw_open: bool = False

    def __init__(self):
        super().__init__()
        self.angle = None
        self.length = None
        self.distance_sensor = None
        self.abs_encoder = None
        self.elevator_top_sensor = None
        self.elevator_bottom_sensor = None
        self.pose = None

    def init(self):  # initializing motors
        """initializes the motors"""
        self.motor_extend.init()
        self.claw_motor.init()
        self.distance_sensor = self.claw_motor.motor.getAnalog()
        self.claw_motor_initialized = True
        self.secondary_rotation_motor.init()
        self.main_rotation_motor.init()
        self.rotation_PID = self.main_rotation_motor.pid_controller
        self.elevator_bottom_sensor = self.motor_extend.motor.getReverseLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen
        )
        self.elevator_top_sensor = self.motor_extend.motor.getForwardLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen
        )
        self.abs_encoder = self.secondary_rotation_motor.motor.getAbsoluteEncoder(
            rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )
        self.secondary_rotation_motor.motor.follow(self.main_rotation_motor.motor, True)
        self.wrist.init()
        self.wrist_abs = self.wrist.motor.getAbsoluteEncoder(
            rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )
        self.main_rotation_motor.motor.setSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kForward,
            self.shoulder_angle_to_motor_rotations(constants.shoulder_max_rotation),
        )
        self.main_rotation_motor.motor.setSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kReverse,
            self.shoulder_angle_to_motor_rotations(constants.shoulder_min_rotation),
        )
        self.wrist.motor.setSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kForward,
            self.wrist_angle_to_motor_rotations(constants.wrist_max_rotation),
        )
        self.wrist.motor.setSoftLimit(
            rev.CANSparkMax.SoftLimitDirection.kReverse,
            -self.wrist_angle_to_motor_rotations(constants.wrist_min_rotation),
        )
        self.enable_brake()
        self.rotation_PID.setSmartMotionMaxVelocity(constants.shoulder_max_velocity)
        self.rotation_PID.setSmartMotionMaxAccel(5)
        self.rotation_PID.setIZone(0.0)
        self.rotation_PID.setSmartMotionAllowedClosedLoopError(.1)
        self.rotation_PID.setSmartMotionMinOutputVelocity(0.0)
        self.zero_elevator_rotation()
        self.zero_wrist()

    def set_angle_wrist(self, pos: float):
        """
        Set the angle of the claw

        Args:
            pos (float): Angle in radians (0, 2pi)
        """
        self.wrist.set_target_position(
            # 80 rotations per 360 degrees
            (pos / (math.pi * 2))
            * constants.wrist_gear_ratio
        )

    def get_angle_wrist(self):
        # Return through bore encode value
        return self.wrist.get_sensor_position() / constants.wrist_gear_ratio

    def zero_wrist(self):
        """Sets the shoulder to the zero position (no extension)"""
        self.wrist.set_sensor_position(0)
        # print(self.main_rotation_motor.get_sensor_position())
        # get Absolute encoder position
        abs_encoder_position: float = self.wrist_abs.getPosition()
        # print("INTERN ABS:" + str(abs_encoder_position))
        # calculate the difference between the current position and the zero position
        if abs_encoder_position > 0.5:
            abs_encoder_position = -(1 - abs_encoder_position)
        encoder_difference: float = abs_encoder_position - 0
        # print("INTERN ENC: " + str(encoder_difference))
        # convert absolute position ratio to motor rotations and gearbox ratio
        motor_position: float = encoder_difference * constants.wrist_gear_ratio
        # print("ELEVATOR RATIO: " + str(constants.elevator_rotation_gear_ratio))
        # print( "MOTOR POSITION W/GEAR RATIO:" + str(motor_position))
        # set motor position to the difference
        self.wrist.set_sensor_position(-motor_position)
        # run the motor to the zero position
        self.wrist.set_target_position(0)

    # def set claw motor output (speed)
    def set_claw_output(self, speed: float):
        """
        Set the output of the claw motor

        Args:
            output (float): Speed from -1 to 1
        """
        self.claw_motor.set_raw_output(speed)
        self.raw_output = speed

    def open_claw(self):
        self.claw_grabber.set(wpilib.DoubleSolenoid.Value.kForward)
        self.claw_open = True

    def close_claw(self):
        self.claw_grabber.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.claw_open = False

    def engage_claw(self):
        # Set distance forward (closes claw)
        self.open_claw()
        self.set_claw_output(config.claw_motor_speed)

    def disengage_claw(self):
        self.close_claw()
        self.set_claw_output(0)

    def enable_brake(self) -> bool:
        """enables the brake"""
        if self.brake_override:
            return False
        else:
            self.brake_enabled = True
            self.brake.set(wpilib.DoubleSolenoid.Value.kReverse)
            return True

    def disable_brake(self) -> bool:
        """disables the brake"""
        if self.brake_override:
            return False
        else:
            self.brake_enabled = False
            self.brake.set(wpilib.DoubleSolenoid.Value.kOff)
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

    def is_at_position(self, length, angle, angle_wrist) -> bool:
        """checks if the arm is at the desired position"""  # TODO: ADD A THRESHOLD FOR THIS CHECK!!
        return (
            length == self.get_length()
            and angle == self.get_rotation()
            and angle_wrist == self.get_angle_wrist()
        )

    def is_at_length(self, length: float) -> bool:
        """checks if the arm is at the desired length"""
        return length == self.get_length()

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
    def wrist_angle_to_motor_rotations(angle: radians):
        """returns the scale of angle in radians to the shoulder motor rotations

        Args:
            angle (float): the angle in radians

        Returns:
            float: a float of the rotations
        """
        return (angle / (2 * math.pi)) * constants.wrist_gear_ratio

    @staticmethod
    def abs_encoder_to_rad(encoder_value: float):
        """converts the encoder value to radians"""
        return encoder_value * (2 * math.pi)

    def stop(self) -> None:
        """stops the elevator"""
        self.motor_extend.set_raw_output(0)
        self.main_rotation_motor.set_raw_output(0)
        self.wrist.set_raw_output(0)
        self.claw_motor.set_raw_output(0)

    def hard_stop(self) -> None:
        """stops the elevator and enables the brake"""
        self.stop()
        self.enable_brake()

    def set_length(self, meters) -> None:  # set arm extension
        """Sets the length of the elevator to the given meters, returns true if the elevator is at the given meters, float of the meters the elevator is at if it is not at the given meters"""
        print(meters)
        if not self.disable_extension:
            length_ratio = self.boundary_box(self.get_rotation())
            # lol = min(length_ratio * constants.max_elevator_height, meters)
            lol = meters
            print(lol)
            length = lol * (1 / constants.elevator_length_per_rotation)
            print(length)
            self.motor_extend.set_target_position(length)
            self.rotation_PID.setSmartMotionMaxAccel(.01)

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
        def set(angle: radians):
            rotations = (angle / (2 * math.pi)) * constants.elevator_rotation_gear_ratio
            self.main_rotation_motor.set_target_position(rotations)
            # self.rotation_PID.setReference(1, rev.CANSparkMax.ControlType.kVelocity)

        # if the rotation is within the soft limits, set the rotation to the given angle
        if not self.disable_rotation:
            if self.shoulder_rotation_limits(angle):
                # print("In LIMITS")
                set(angle)
                return True
            else:
                # print("NOT IN LIMITS")
                if angle > 0:
                    y = constants.shoulder_max_rotation
                else:
                    y = -constants.shoulder_min_rotation
                set(y)
                return y

    def get_length(self) -> float:  # returns arm extension
        """Gets the length of the elevator in meters"""
        return self.motor_extend.get_sensor_position() / (
            1 / constants.elevator_length_per_rotation
        )
        # return (self.motor_extend.get_sensor_position() * constants.elevator_extend_gear_ratio * constants.max_elevator_height_delta) + constants.elevator_zero_length

    def get_rotation(self) -> float:  # returns arm rotation in radians
        """Gets the rotation of the shoulder in radians"""
        return (
            self.main_rotation_motor.get_sensor_position()
            / constants.elevator_rotation_gear_ratio
        ) * (2 * math.pi)

    def get_rotation_radians_abs(self) -> float:
        """Gets the rotation of the shoulder in rotations from the absolute encoder"""
        return self.abs_encoder_to_rad(self.abs_encoder.getPosition())

    def get_rotation_abs(self) -> float:
        """Gets the rotation of the shoulder in rotations from the absolute encoder"""
        return self.abs_encoder.getPosition()

    # brings elevator to zero position (no extension, no rotation)
    def zero_elevator_length(self) -> None:
        """Sets the elevator to the zero position (no rotation)"""
        self.motor_extend.set_target_position(
            self.motor_extend.get_sensor_position() - 0.005
        )

    def zero_elevator_rotation(self) -> None:
        """Sets the shoulder to the zero position (no extension)"""
        self.main_rotation_motor.set_sensor_position(0)
        # print(self.main_rotation_motor.get_sensor_position())
        # get Absolute encoder position
        abs_encoder_position: float = self.abs_encoder.getPosition()
        # print("INTERN ABS:" + str(abs_encoder_position))
        # calculate the difference between the current position and the zero position
        if abs_encoder_position > 0.5:
            abs_encoder_position = -(1 - abs_encoder_position)
        encoder_difference: float = (
            abs_encoder_position - constants.elevator_initial_rotation
        )
        # print("INTERN ENC: " + str(encoder_difference))
        # convert absolute position ratio to motor rotations and gearbox ratio
        motor_position: float = (
            encoder_difference * constants.elevator_rotation_gear_ratio
        )
        # print("ELEVATOR RATIO: " + str(constants.elevator_rotation_gear_ratio))
        # print( "MOTOR POSITION W/GEAR RATIO:" + str(motor_position))
        # set motor position to the difference
        self.main_rotation_motor.set_sensor_position(motor_position)
        # run the motor to the zero position
        self.main_rotation_motor.set_target_position(0)

    def extend_max_elevator(self) -> None:
        """Sets the elevator to the max position (no rotation)"""
        print("EXTEND MAX")
        self.set_length(constants.max_elevator_height_delta)

    def update_pose(self) -> None:
        """Updates the pose of the arm using the encoder values and rotation of the elevator"""
        angle = self.get_rotation()
        print("POSE ANGLE: " + str(angle))
        length = self.get_length()
        print("POSE LENGTH: " + str(length))
        self.length = length
        self.angle = angle
