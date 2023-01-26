import wpilib
import rev
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax
from robotpy_toolkit_7407.utils.units import meters
from robotpy_toolkit_7407.sensors.limit_switches.limit_switch import LimitSwitch
import math
import constants
import config
from wpimath import Pose3d
# importing packages

#TODO: check conversions

class Elevator(Subsystem):  # elevator class
    motor_extend: SparkMax = SparkMax(
        config.elevator_motor_extend_id)  # motor that extends the arm
    right_rotation_motor: SparkMax = SparkMax(
        config.elevator_right_rotation_motor_id, inverted=True)  # motor that rotates the arm
    left_rotation_motor: SparkMax = SparkMax(
        config.elevator_left_rotation_motor_id, inverted=False)  # motor that rotates the arm
    brake: wpilib.Solenoid = wpilib.Solenoid(
        1, wpilib.PneumaticsModuleType.REVPH, config.elevator_brake_id)  # brake that holds the arm in place
    initialized: bool = False
    elevator_bottom_sensor = rev.CANSparkMax(config.elevator_left_rotation_motor_id, rev.MotorType.kBrushless).getReverseLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
    elevator_top_sensor = rev.CANSparkMax(config.elevator_left_rotation_motor_id, rev.MotorType.kBrushless).getForwardLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
    turn_sensor = rev.CANSparkMax(config.elevator_left_rotation_motor_id, rev.MotorType.kBrushless).getAnalog()  # senses the rotation of the elevator
    brake_enabled: bool = False
    pose = Pose3d(0, 0, constants.pivot_point_height, 0, 0, 0)
    elevator_length: float
    rotation_overide: bool = False # if true, the rotation limits will be ignored
    extension_overide: bool = False # if true, the extension limits will be ignored
    brake_overide: bool = False # if true, the brake will not be enabled or disabled

    def enable_brake(self):
        if self.brake_overide:
            return
        else:
            self.brake_enabled = True
            self.brake.set(True)

    def disable_brake(self):
        if self.brake_overide:
            return
        else:
            self.brake_enabled = False
            self.brake.set(False)

    def get_pose(self):
        # returns the pose of the arm
        return self.pose

    def set_pose(self, pose: Pose3d):
        self.pose = pose

    def is_at_position(self, position: Pose3d):
        if (self.get_pose() == position):
            return True
        else:
            return False

    def init(self):  # initializing motors
        self.motor_extend.init()
        self.right_rotation_motor.init()
        self.left_rotation_motor.init()
        self.right_rotation_motor.follow(
            config.elevator_left_rotation_motor_id, True)
        self.enable_brake()

    def stop(self):
        self.motor_extend.set_raw_output(0)
        self.left_rotation_motor.set_raw_output(0)

    def boundary_box(self, radians: float):

        if self.extension_overide:
            return 1
        else:

            degree = abs(radians)

            def get_length_ratio(angle, adjacent):
                # Gets the length of the angle using cosine
                max_length = adjacent / math.cos(angle)
                # if the length of the angle is greater than the max_elevator_height, return one
                # else give a percentage of the height of the current line and the max_elevator_height
                # protects against any irregular numbers heigher than the max elevator height by always
                # defaulting to 1
                if max_length > constants.max_elevator_height:
                    return 1
                else:
                    return max_length / constants.max_elevator_height  # where is the hype at

            # Finds the length of each the dimensions for the boundry box using the constants
            top_box_height = constants.vertical_boundary - constants.pivot_point_height
            bottom_box_height = constants.pivot_point_height
            horizontal_length = 0.5(constants.robot_length) + \
                constants.horizontal_boundary

            # finds the angle on each side where the length is equal to the max_elevator_height
            top_box_angle = math.acos(top_box_height/constants.max_elevator_height)
            side_box_angle = math.acos(
                horizontal_length/constants.max_elevator_height)
            bottom_box_angle = math.acos(
                bottom_box_height/constants.max_elevator_height)

            # if the degree is within the minimun range of each of these boundry sides

            # assuming the top is zero...

            # if the degree is within the max_elevator_extension and facing the top
            if degree < top_box_angle:
                return get_length_ratio(degree, top_box_angle)
            # if the degree is within the max_elevator_extension and facing the sides
            elif (degree) > side_box_angle and degree < side_box_angle+(90 - side_box_angle):
                return get_length_ratio(90 - degree, side_box_angle)
            # if the degree is within the max_elevator_extension and facing the bottom
            elif (degree) > bottom_box_angle:
                return get_length_ratio(180 - degree, bottom_box_angle)

    def set_length(self, length: meters):  # set arm extension
        if self.boundary_box(length) > self.elevator_length:
            self.motor_extend.set_target_position(
                length * constants.elevator_extend_gear_ratio)
            self.elevator_length = self.motor_extend.get_sensor_position() / constants.elevator_extend_gear_ratio
            return True
        else:
            step: int
            if length > 0: step = -1
            else: step = 1
            for i in range(length, 0, step):
                if self.boundary_box(i) > self.elevator_length:
                    self.motor_extend.set_target_position(
                        i * constants.elevator_extend_gear_ratio)
                    self.motor_extend.get_sensor_position() / constants.elevator_extend_gear_ratio
                    return i

    def soft_limits(self, radians: float):
        # returns if the degree is within the soft limits
        if self.rotation_overide:
            return True
        else:
            return radians < math.radians(config.elevator_max_rotation) and radians > math.radians(config.elevator_min_rotation)

    def set_rotation(self, radians: float):  # set arm rotation
        # if the rotation is within the soft limits, set the rotation to the given degree
        if self.soft_limits(radians):
            self.left_rotation_motor.set_target_position(
                radians * constants.elevator_rotation_gear_ratio)
            # returns True
            return True
        else:
            # if the rotation is not within the soft limits, set the rotation to the closest soft limit
            step: int
            if radians > 0: step = -1
            else: step = 1
            for i in range(radians, 0, step):
                if self.soft_limits(i):
                    self.left_rotation_motor.set_target_position(
                        i * constants.elevator_rotation_gear_ratio)
                    # returns the closest soft limit
                    return i

    def get_length(self):  # returns arm extension
        return self.motor_extend.get_sensor_position() / constants.elevator_extend_gear_ratio

    def get_rotation(self):  # returns arm rotation
        return self.left_rotation_motor.get_sensor_position() / constants.elevator_rotation_gear_ratio

    def zero_elevator(self):  # brings elevator to zero position (no extension, no rotation)
        self.set_length(self.get_length()-0.005)
        self.set_rotation(self.get_rotation()-0.005)

    def extend_max_elevator(self):
        self.set_length(config.elevator_max_height)

    def update_pose(self):
        # updates the pose of the arm using the encoder values and rotation of the elevator
        angle = self.get_rotation()
        length = self.get_length()
        x = length * math.cos(angle)
        height = length * math.sin(angle)
        self.pose = Pose3d(x, 0, height + constants.pivot_point_height, 0, 0, 0)



