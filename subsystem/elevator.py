import math

import rev
import wpilib
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from wpimath.geometry import Pose3d, Rotation3d

import config
import constants

# importing packages

# TODO: check conversions

SHOLDER_CONFIG = SparkMaxConfig(
    0.1, 0, 0.003, 0.0002, (-0.3, 0.3), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)
ELEVATOR_CONFIG = SparkMaxConfig(
    0.00005, 0, 0.0004, 0.00017, idle_mode=rev.CANSparkMax.IdleMode.kBrake
)


class Elevator(Subsystem):  # elevator class
    motor_extend: SparkMax = SparkMax(
        config.elevator_motor_extend_id, config=ELEVATOR_CONFIG)  # motor that extends the arm
    secondary_rotation_motor: SparkMax = SparkMax(
        config.elevator_secondary_rotation_motor_id, inverted=True, config=SHOLDER_CONFIG)  # motor that rotates the arm
    main_rotation_motor: SparkMax = SparkMax(
        config.elevator_main_rotation_motor_id, inverted=True, config=SHOLDER_CONFIG)  # motor that rotates the arm
    brake: wpilib.DoubleSolenoid = wpilib.DoubleSolenoid(
        1, wpilib.PneumaticsModuleType.REVPH, config.elevator_enable_brake_id,
        config.elevator_disable_brake_id)  # brake that holds the arm in place
    initialized: bool = False
    brake_enabled: bool = False
    pose: Pose3d
    # elevator_length                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               h: float # the length of the elevator in meters
    rotation_override: bool = False  # if true, the rotation limits will be ignored
    extension_override: bool = False  # if true, the extension limits will be ignored
    brake_override: bool = False  # if true, the brake will not be enabled or disabled
    intake_up: bool = True

    def __init__(self):
        super().__init__()
        self.abs_encoder = None
        self.elevator_top_sensor = None
        self.elevator_bottom_sensor = None

    def enable_brake(self):
        """enables the brake"""
        if self.brake_override:
            return False
        else:
            self.brake_enabled = True
            self.brake.set(self.brake.Value.kForward)
            return True

    def disable_brake(self):
        """disables the brake"""
        if self.brake_override:
            return False
        else:
            self.brake_enabled = False
            self.brake.set(self.brake.Value.kReverse)
            return True

    def enable_brake_override(self):
        """enables the brake override"""
        self.brake_override = True

    def disable_brake_override(self):
        """disables the brake override"""
        self.brake_override = False

    def enable_rotation_override(self):
        """enables the rotation override"""
        self.rotation_override = True

    def disable_rotation_override(self):
        """disables the rotation override"""
        self.rotation_override = False

    def get_pose(self):
        """Gets the pose of the arm"""
        return self.pose

    def set_pose(self, pose: Pose3d):
        """sets the pose of the arm"""
        self.pose = pose

    def is_at_position(self, position: Pose3d):  # NEED TO EDIT THIS TO USE MOTOR ANGLE AND POSITIONS INSTEAD OF POSE!!
        """checks if the arm is at the desired position"""
        return (self.get_pose() < Pose3d(position.X - constants.arm_pose_accuracy),
                position.Y - constants.arm_pose_accuracy,
                position.Rotation - constants.arm_pose_accuracy) and (
                   self.get_pose() > Pose3d(position.X + constants.arm_pose_accuracy),
                   position.Y + constants.arm_pose_accuracy,
                   position.Rotation + constants.arm_pose_accuracy)

    def shoulder_angle_to_motor_rotations(self, angle: float):
        """returns the scale of angle in radians to the shoulder motor rotations

        Args:
            angle (float): the angle in radians

        Returns:
            float: a float of the rotations
        """
        return (angle * constants.elevator_rotation_gear_ratio) / (2 * math.pi)

    def abs_encoder_to_rad(self, encoder_value: float):
        """converts the encoder value to radians"""
        return encoder_value * (2 * math.pi)

    def init(self):  # initializing motors
        """initializes the motors"""
        self.motor_extend.init()
        self.secondary_rotation_motor.init()
        self.main_rotation_motor.init()
        self.elevator_bottom_sensor = self.motor_extend._motor.getReverseLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        self.elevator_top_sensor = self.motor_extend._motor.getForwardLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        self.abs_encoder = self.secondary_rotation_motor._motor.getAbsoluteEncoder(
            rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.secondary_rotation_motor._motor.follow(
            self.main_rotation_motor._motor, True)
        # if not self.intake_up:
        #     self.main_rotation_motor._motor.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.angle_to_shoulder_rotations(constants.shoulder_max_rotation) )
        #     self.main_rotation_motor._motor.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.angle_to_shoulder_rotations(constants.shoulder_min_rotation) )
        # else:
        #     self.main_rotation_motor._motor.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.angle_to_shoulder_rotations(constants.shoulder_intake_up_max_rotation) )
        #     self.main_rotation_motor._motor.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.angle_to_shoulder_rotations(constants.shoulder_min_rotation) )
        self.main_rotation_motor._motor.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward,
                                                     self.shoulder_angle_to_motor_rotations(
                                                         constants.shoulder_max_rotation))
        self.main_rotation_motor._motor.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse,
                                                     self.shoulder_angle_to_motor_rotations(
                                                         constants.shoulder_min_rotation))

        self.enable_brake()

    def stop(self):
        """stops the elevator"""
        self.motor_extend.set_raw_output(0)
        self.main_rotation_motor.set_raw_output(0)

    def hard_stop(self):
        """stops the elevator and enables the brake"""
        self.stop()
        self.enable_brake()

    def boundary_box(self, angle: float) -> float:  # TESTED
        """returns a float of the percentage of the max elevator height allowed at the given angle (limited by the robot extension limits)"""
        if self.extension_override:
            # if the extension override is enabled, return 1, which will maximize the max elevator height
            return 1
        else:
            RNangle = angle
            angle = abs(angle)

            def get_length_ratio(theta, adjacent):
                # Gets the length of the angle using cosine
                max_length = adjacent / math.cos(theta)
                # if the length of the angle is greater than the max_elevator_height, return one
                # else give a percentage of the height of the current line and the max_elevator_height
                # protects against any irregular numbers higher than the max elevator height by always
                # defaulting to 1
                if abs(max_length) > constants.max_elevator_height:
                    return 1
                else:
                    res = abs(max_length / constants.max_elevator_height)
                    return res  # where is the hype at

            # Finds the length of each the dimensions for the boundary box using the constants
            top_vertical_boundary_from_pivot = constants.vertical_boundary - constants.pivot_point_height - constants.top_boundary_buffer_gap  # positive z
            bottom_vertical_boundary_from_pivot = constants.pivot_point_height - constants.bottom_boundary_buffer_gap  # negative z
            horizontal_boundary_length_from_pivot = 0.5 * (
                constants.robot_length) + constants.horizontal_boundary - constants.side_boundary_buffer_gap  # positive x

            # the max angle of the boundary dimsensions
            # the top boundry max angle is the angle of the top boundry and horizontal boundry length from the pivot point
            top_boundary_max_angle = math.atan2(top_vertical_boundary_from_pivot, horizontal_boundary_length_from_pivot)
            # the bottom boundry max angle is the angle of the bottom boundry and horizontal boundry length from the pivot point
            bottom_boundary_max_angle = math.atan2(bottom_vertical_boundary_from_pivot,
                                                   horizontal_boundary_length_from_pivot) + math.radians(90)
            # the side boundry angle can be inferred from the top and bottom boundry angles

            # since we can only extend past our bumper in one direction at a time, if the intakes are down, we limit the angle to the side with the intake
            if not self.intake_up:
                back_horizontal_boundary_length_from_pivot = -0.5 * (
                    constants.robot_length) - constants.side_boundary_buffer_gap
                back_horizontal_boundry_max_angle = math.atan2(top_vertical_boundary_from_pivot,
                                                               back_horizontal_boundary_length_from_pivot)
                if (RNangle < back_horizontal_boundry_max_angle):
                    # if the angle is on the other side of the robot, return 0
                    return 0

            # assuming the zero is vertical...
            # if the angle is less than the top boundry max angle, return the length ratio of the angle and the top boundry
            if (angle < top_boundary_max_angle):
                return get_length_ratio(angle, top_vertical_boundary_from_pivot)
            # if the angle is between the top boundry max angle and the bottom boundry max angle, return the length ratio of the angle and the horizontal boundry
            elif (angle > top_boundary_max_angle and angle < bottom_boundary_max_angle):
                return get_length_ratio(90 - angle, horizontal_boundary_length_from_pivot)
            # if the angle is greater than the bottom boundry max angle, return the length ratio of the angle and the bottom boundry
            elif (angle > bottom_boundary_max_angle):
                return get_length_ratio(180 - angle, bottom_vertical_boundary_from_pivot)
            else:
                return 1

    def set_length(self, meters):  # set arm extension
        """Sets the length of the elevator to the given meters, returns true if the elevator is at the given meters, float of the meters the elevator is at if it is not at the given meters"""

        def set(length):
            start_length = (self.motor_extend.get_sensor_position() * constants.elevator_extend_gear_ratio) \
                           / constants.max_elevator_height
            fin_length = (length / constants.max_elevator_height) * constants.elevator_extend_gear_ratio
            delta_length = fin_length - start_length
            self.motor_extend.set_target_position(delta_length)

        length_ratio = self.boundary_box(self.abs_encoder * (2 * math.pi))
        if length_ratio * constants.max_elevator_height > meters:
            set(meters)
            return True
        else:
            set(length_ratio * constants.max_elevator_height)
            return length_ratio * constants.max_elevator_height

    def shoulder_rotation_limits(self, radians: float):
        """Returns if the given radians are within the soft limits of the shoulder"""
        # returns if the angle is within the soft limits
        if self.rotation_override:
            return True
        else:
            if self.intake_up:
                return constants.shoulder_intake_up_max_rotation > radians > -constants.shoulder_min_rotation
            else:
                return constants.shoulder_max_rotation > radians > -constants.shoulder_min_rotation

    def set_rotation(self, radians: float):  # set arm rotation
        """Sets the rotation of the shoulder to the given radians, returns true if the shoulder is at the given radians, float of the radians the shoulder is at if it is not at the given radians"""

        def set(radians: float):
            rotations = (radians / (2 * math.pi)) * constants.elevator_rotation_gear_ratio
            self.main_rotation_motor.set_target_position(rotations)

        # if the rotation is within the soft limits, set the rotation to the given angle
        if self.soft_limits(radians):
            set(radians)
            return True
        else:
            if self.intake_up:
                if radians > 0:
                    y = constants.shoulder_intake_up_max_rotation
                else:
                    y = -constants.shoulder_min_rotation
                set(y)
                return y
            else:
                if radians > 0:
                    y = constants.shoulder_max_rotation
                else:
                    y = -constants.shoulder_min_rotation
                set(y)
                return y

    def get_length(self):  # returns arm extension
        """Gets the length of the elevator in meters"""
        return (self.motor_extend.get_sensor_position() * constants.elevator_extend_gear_ratio) \
               / constants.max_elevator_height

    def get_rotation(self):  # returns arm rotation in radians
        """Gets the rotation of the shoulder in radians"""
        return (self.main_rotation_motor.get_sensor_position() * constants.elevator_rotation_gear_ratio) * (2 * math.pi)

    def get_rotation_radians_abs(self):
        """Gets the rotation of the shoulder in rotations from the absolute encoder"""
        return self.abs_encoder_to_rad(self.abs_encoder.getPosition())

    def get_rotation_abs(self):
        """Gets the rotation of the shoulder in rotations from the absolute encoder"""
        return self.abs_encoder.getPosition()

    # brings elevator to zero position (no extension, no rotation)
    def zero_elevator_length(self):
        """Sets the elevator to the zero position (no rotation)"""
        self.set_length(self.get_length() - 0.005)

    def zero_elevator_rotation(self):
        """Sets the shoulder to the zero position (no extension)"""
        self.main_rotation_motor.set_sensor_position(0)
        # print(self.main_rotation_motor.get_sensor_position())
        # get Absolute encoder position
        abs_encoder_position: float = self.abs_encoder.getPosition()
        # print("INTERN ABS:" + str(abs_encoder_position))
        # calculate the difference between the current position and the zero position
        if abs_encoder_position > .5:
            abs_encoder_position = -(1 - abs_encoder_position)
        encoder_difference: float = abs_encoder_position - constants.elevator_initial_rotation
        # print("INTERN ENC: " + str(encoder_difference))
        # convert absolute position ratio to motor rotations and gearbox ratio
        motor_position: float = encoder_difference * constants.elevator_rotation_gear_ratio
        # print("ELEVATOR RATIO: " + str(constants.elevator_rotation_gear_ratio))
        # print( "MOTOR POSITION W/GEAR RATIO:" + str(motor_position))
        # set motor position to the difference
        self.main_rotation_motor.set_sensor_position(motor_position)
        # run the motor to the zero position
        self.main_rotation_motor.set_target_position(0)

    def extend_max_elevator(self):
        """Sets the elevator to the max position (no rotation)"""
        self.set_length(self.boundary_box(self.get_rotation()) *
                        constants.max_elevator_height)

    def update_pose(self):
        """Updates the pose of the arm using the encoder values and rotation of the elevator"""
        # updates the pose of the arm using the encoder values and rotation of the elevator
        angle = self.get_rotation()
        length = self.get_length()
        x = length * math.cos(angle)
        z = length * math.sin(angle)
        self.pose = Pose3d(x, 0, z + constants.pivot_point_height, Rotation3d(0, 0, 0))
