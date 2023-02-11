import math

import rev
import wpilib
import wpimath
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from wpimath.geometry import Pose3d, Rotation3d
import config
import constants
from wpimath import trajectory, controller

# importing packages

SHOULDER_CONFIG = SparkMaxConfig(
    0.1, 0, 0.003, 0.0002, (-0.25, 0.25), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)
ELEVATOR_CONFIG = SparkMaxConfig(
    1, 0, 0.0004, 0.00017, (-1, 1), idle_mode=rev.CANSparkMax.IdleMode.kBrake
)


WRIST_CONFIG = SparkMaxConfig(
    0.05, 0, 0.004, 0.00017, idle_mode=rev.CANSparkMax.IdleMode.kBrake
)

shoulder_constraints = trajectory.TrapezoidProfile.Constraints(2, 0.25)

shoulder_pid = controller.ProfiledPIDController(0.1, 0, .003, shoulder_constraints)

shoulder_pid.calculate()



class Arm(Subsystem):  # elevator class
    # Elevator Motors
    motor_extend: SparkMax = SparkMax(
        config.elevator_motor_extend_id, config=ELEVATOR_CONFIG)  # motor that extends the arm
    # Shoulder Motors
    secondary_rotation_motor: SparkMax = SparkMax(
        config.elevator_secondary_rotation_motor_id, inverted=True, config=SHOULDER_CONFIG)  # motor that rotates the arm
    main_rotation_motor: SparkMax = SparkMax(
        config.elevator_main_rotation_motor_id, inverted=True, config=SHOULDER_CONFIG)  # motor that rotates the arm
    #main_rotation_motor.__pid_controller.setSmartMotionMaxAccel(2 * constants.elevator_rotation_gear_ratio, 0)
    # Brake
    brake: wpilib.Solenoid = wpilib.Solenoid(
        1, wpilib.PneumaticsModuleType.REVPH, config.elevator_brake_id)  # brake that holds the arm in place
    # Wrist and Claw 
    wrist: SparkMax = SparkMax(18, inverted=True, config=WRIST_CONFIG)
    claw_motor: SparkMax = SparkMax(12, inverted=False)
    claw_grabber: wpilib.DoubleSolenoid = wpilib.DoubleSolenoid(
        1, wpilib.PneumaticsModuleType.REVPH, 4, 5)
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
        self.abs_encoder = None
        self.elevator_top_sensor = None
        self.elevator_bottom_sensor = None
        self.pose = None


    def set_angle_wrist(self, pos: float):
        """
        Set the angle of the claw

        Args:
            pos (float): Angle in radians (0, 2pi)
        """
        self.wrist.set_target_position(
            # 80 rotations per 360 degrees
            (pos / (math.pi * 2)) * constants.wrist_gear_ratio
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
        if abs_encoder_position > .5:
            abs_encoder_position = -(1 - abs_encoder_position)
        encoder_difference: float = abs_encoder_position - 0
        # print("INTERN ENC: " + str(encoder_difference))
        # convert absolute position ratio to motor rotations and gearbox ratio
        motor_position: float = encoder_difference * \
            constants.wrist_gear_ratio
        # print("ELEVATOR RATIO: " + str(constants.elevator_rotation_gear_ratio))
        # print( "MOTOR POSITION W/GEAR RATIO:" + str(motor_position))
        # set motor position to the difference
        self.wrist.set_sensor_position(motor_position)
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
        self.set_claw_output(constants.claw_motor_speed)

    def disengage_claw(self):
        self.close_claw()
        self.set_claw_output(0)

    def enable_brake(self) -> bool:
        """enables the brake"""
        if self.brake_override:
            return False
        else:
            self.brake_enabled = True
            self.brake.set(True)
            return True

    def disable_brake(self) -> bool:
        """disables the brake"""
        if self.brake_override:
            return False
        else:
            self.brake_enabled = False
            self.brake.set(False)
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
        """checks if the arm is at the desired position"""
        return length == self.get_length() and angle == self.get_rotation() and angle_wrist == self.get_angle_wrist()

    def is_at_length(self, length: float) -> bool:
        """checks if the arm is at the desired length"""
        return length == self.get_length()

    def shoulder_angle_to_motor_rotations(self, angle: float):
        """returns the scale of angle in radians to the shoulder motor rotations

        Args:
            angle (float): the angle in radians

        Returns:
            float: a float of the rotations
        """
        return (angle * constants.elevator_rotation_gear_ratio) / (2 * math.pi)

    def wrist_angle_to_motor_rotations(self, radians: float):
        """returns the scale of angle in radians to the shoulder motor rotations

        Args:
            angle (float): the angle in radians

        Returns:
            float: a float of the rotations
        """
        return (radians / (2 * math.pi)) * constants.wrist_gear_ratio

    def init(self):  # initializing motors
        """initializes the motors"""
        self.motor_extend.init()
        self.claw_motor.init()
        self.distance_sensor = self.claw_motor.motor.getAnalog()
        self.claw_motor_initialized = True
        self.secondary_rotation_motor.init()
        self.main_rotation_motor.init()
        self.elevator_bottom_sensor = self.motor_extend.motor.getReverseLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        self.elevator_top_sensor = self.motor_extend.motor.getForwardLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        self.abs_encoder = self.secondary_rotation_motor.motor.getAbsoluteEncoder(
            rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.secondary_rotation_motor.motor.follow(
            self.main_rotation_motor.motor, True)
        self.wrist.init()
        self.wrist_abs = self.wrist.motor.getAbsoluteEncoder(
            rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.main_rotation_motor.motor.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.shoulder_angle_to_motor_rotations(constants.shoulder_max_rotation))
        self.main_rotation_motor.motor.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.shoulder_angle_to_motor_rotations(constants.shoulder_min_rotation))
        self.wrist.motor.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.wrist_angle_to_motor_rotations(constants.wrist_max_rotation))
        self.wrist.motor.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, -self.wrist_angle_to_motor_rotations(constants.wrist_min_rotation))
        self.enable_brake()

    def abs_encoder_to_rad(self, encoder_value: float):
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
                if theta == 0:
                    v = adjacent / constants.max_elevator_height
                    return 1 if v > 1 else v

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
            top_vertical_boundary_from_pivot = constants.vertical_boundary - \
                constants.pivot_point_height - constants.top_boundary_buffer_gap  # positive z
            bottom_vertical_boundary_from_pivot = constants.pivot_point_height - \
                constants.bottom_boundary_buffer_gap  # negative z
            horizontal_boundary_length_from_pivot = 0.5 * (
                constants.robot_length) + constants.horizontal_boundary - constants.side_boundary_buffer_gap  # positive x

            # the max angle of the boundary dimsensions
            # the top boundry max angle is the angle of the top boundry and horizontal boundry length from the pivot point
            top_boundary_max_angle = math.atan2(
                top_vertical_boundary_from_pivot, horizontal_boundary_length_from_pivot)
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
                return get_length_ratio(math.radians(90) - angle, horizontal_boundary_length_from_pivot)
            # if the angle is greater than the bottom boundry max angle, return the length ratio of the angle and the bottom boundry
            elif (angle > bottom_boundary_max_angle):
                return get_length_ratio(math.radians(180) - angle, bottom_vertical_boundary_from_pivot)
            else:
                return 1

    def set_length(self, meters) -> None:  # set arm extension
        """Sets the length of the elevator to the given meters, returns true if the elevator is at the given meters, float of the meters the elevator is at if it is not at the given meters"""
        print(meters)
        if not self.disable_extension:
            length_ratio = self.boundary_box(
                self.get_rotation())
            #lol = min(length_ratio * constants.max_elevator_height, meters)
            lol = meters
            print(lol)
            length = lol * ( 1/constants.elevator_length_per_rotation)
            print(length)
            self.motor_extend.set_target_position(length)

    def shoulder_rotation_limits(self, radians: float) -> bool:
        """Returns if the given radians are within the soft limits of the shoulder"""
        # returns if the angle is within the soft limits
        if self.rotation_override:
            return True
        else:
            return radians <= constants.shoulder_max_rotation and radians >= -constants.shoulder_min_rotation

    def set_rotation(self, radians: float):  # set arm rotation
        """Sets the rotation of the shoulder to the given radians, returns true if the shoulder is at the given radians, float of the radians the shoulder is at if it is not at the given radians"""
        # print("RADIANS: " + str(radians))
        def set(radians: float):
            rotations = (radians / (2 * math.pi)) * \
                constants.elevator_rotation_gear_ratio
            self.main_rotation_motor.set_target_position(rotations)

        # if the rotation is within the soft limits, set the rotation to the given angle
        if not self.disable_rotation:
            if self.shoulder_rotation_limits(radians):
                # print("In LIMITS")
                set(radians)
                return True
            else:
                # print("NOT IN LIMITS")
                if radians > 0:
                    y = constants.shoulder_max_rotation
                else:
                    y = -constants.shoulder_min_rotation
                set(y)
                return y

    def get_length(self) -> float:  # returns arm extension
        """Gets the length of the elevator in meters"""
        return (self.motor_extend.get_sensor_position() / ( 1/constants.elevator_length_per_rotation))
        #return (self.motor_extend.get_sensor_position() * constants.elevator_extend_gear_ratio * constants.max_elevator_height_delta) + constants.elevator_zero_length

    def get_rotation(self) -> float:  # returns arm rotation in radians
        """Gets the rotation of the shoulder in radians"""
        return (self.main_rotation_motor.get_sensor_position() / constants.elevator_rotation_gear_ratio) * (2 * math.pi)

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
            self.motor_extend.get_sensor_position() - 0.005)

    def zero_elevator_rotation(self) -> None:
        """Sets the shoulder to the zero position (no extension)"""
        self.main_rotation_motor.set_sensor_position(0)
        # print(self.main_rotation_motor.get_sensor_position())
        # get Absolute encoder position
        abs_encoder_position: float = self.abs_encoder.getPosition()
        # print("INTERN ABS:" + str(abs_encoder_position))
        # calculate the difference between the current position and the zero position
        if abs_encoder_position > .5:
            abs_encoder_position = -(1 - abs_encoder_position)
        encoder_difference: float = abs_encoder_position - \
            constants.elevator_initial_rotation
        # print("INTERN ENC: " + str(encoder_difference))
        # convert absolute position ratio to motor rotations and gearbox ratio
        motor_position: float = encoder_difference * \
            constants.elevator_rotation_gear_ratio
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
        # updates the pose of the arm using the encoder values and rotation of the elevator
        angle = self.get_rotation()
        print("POSE ANGLE: " + str(angle))
        length = self.get_length()
        print("POSE LENGTH: " + str(length))
        self.length = length
        self.angle = angle