import wpilib
import rev
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig
from robotpy_toolkit_7407.utils.units import meters
from robotpy_toolkit_7407.sensors.limit_switches.limit_switch import LimitSwitch
import math
import constants
import config
from wpimath.geometry import Pose3d
# importing packages

# TODO: check conversions

SHOLDER_CONFIG = SparkMaxConfig(
    0.2, 0, 0.003, 0.00015, (-0.5, 0.5), idle_mode=rev.CANSparkMax.IdleMode.kBrake
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
        config.elevator_main_rotation_motor_id, inverted=False, config=SHOLDER_CONFIG)  # motor that rotates the arm
    brake: wpilib.Solenoid = wpilib.DoubleSolenoid(
        1, wpilib.PneumaticsModuleType.REVPH, config.elevator_enable_brake_id, config.elevator_disable_brake_id)  # brake that holds the arm in place
    initialized: bool = False
    brake_enabled: bool = False
    pose: Pose3d
    #elevator_length: float # the length of the elevator in meters
    rotation_overide: bool = False  # if true, the rotation limits will be ignored
    extension_overide: bool = False  # if true, the extension limits will be ignored
    brake_overide: bool = False  # if true, the brake will not be enabled or disabled
    intake_up: bool = True

    def enable_brake(self):
        if self.brake_overide:
            return False
        else:
            self.brake_enabled = True
            self.brake.set(True)
            return True

    def disable_brake(self):
        if self.brake_overide:
            return False
        else:
            self.brake_enabled = False
            self.brake.set(False)
            return True

    def enable_brake_overide(self):
        self.brake_overide = True
    
    def disable_brake_overide(self):
        self.brake_overide = False
        
    def enable_rotation_overide(self):
        self.rotation_overide = True
        
    def disable_rotation_overide(self):
        self.rotation_overide = False

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

    def abs_encoder_to_rad(self, encoder_value: float):
        # converts the encoder value to radians
        return encoder_value * (2 * math.pi) 
    
    def init(self):  # initializing motors
        self.motor_extend.init()
        self.secondary_rotation_motor.init()
        self.main_rotation_motor.init()
        self.elevator_bottom_sensor = self.motor_extend._motor.getReverseLimitSwitch(
        rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        self.elevator_top_sensor = self.motor_extend._motor.getForwardLimitSwitch(
        rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        self.turn_sensor: SparkMax._motor.getAbsoluteEncoder = self.motor_extend._motor.getAbsoluteEncoder(rev._rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle) 
        self.secondary_rotation_motor._motor.follow(
            config.elevator_main_rotation_motor_id, True)
        self.enable_brake()

    def stop(self):
        self.motor_extend.set_raw_output(0)
        self.main_rotation_motor.set_raw_output(0)

    def hard_stop(self):
        self.stop()
        self.enable_brake()

    def boundary_box(self, angle: float): #TESTED AND WORKING

        if self.extension_overide:
            return 1
        else:
            RNangle = angle
            angle = abs(angle)

            def get_length_ratio(angle, adjacent):
                # Gets the length of the angle using cosine
                max_length = adjacent / math.cos(angle)
                # if the length of the angle is greater than the max_elevator_height, return one
                # else give a percentage of the height of the current line and the max_elevator_height
                # protects against any irregular numbers heigher than the max elevator height by always
                # defaulting to 1
                if abs(max_length) > constants.max_elevator_height:
                    return 1
                else:
                    res = abs(max_length / constants.max_elevator_height)
                    return res  # where is the hype at

            # Finds the length of each the dimensions for the boundry box using the constants
            top_vertical_boundry_from_pivot = constants.vertical_boundary - constants.pivot_point_height - constants.top_boundry_buffer_gap
            bottom_vertical_boundry_from_pivot = constants.pivot_point_height - constants.bottom_boundry_buffer_gap
            horizontal_boundry_length_from_pivot = 0.5 * (constants.robot_length) + constants.horizontal_boundary - constants.side_boundry_buffer_gap
            
            top_boundry_max_angle = math.atan2(top_vertical_boundry_from_pivot, horizontal_boundry_length_from_pivot)
            bottom_boundry_max_angle = math.atan2(bottom_vertical_boundry_from_pivot, horizontal_boundry_length_from_pivot) + math.radians(90)
            # if the angle is within the minimun range of each of these boundry sides
            if not self.intake_up:
                back_horizontal_boundry_length_from_pivot = -0.5 * (constants.robot_length) - constants.side_boundry_buffer_gap
                back_horizontal_boundry_max_angle = math.atan2(top_vertical_boundry_from_pivot, back_horizontal_boundry_length_from_pivot)
                if(RNangle < back_horizontal_boundry_max_angle):
                    return 0
            # assuming the top is zero...
            if(angle < top_boundry_max_angle):
                return get_length_ratio(angle, top_vertical_boundry_from_pivot)
            elif(angle > top_boundry_max_angle and angle < bottom_boundry_max_angle):
                return get_length_ratio(angle, horizontal_boundry_length_from_pivot)
            elif (angle > bottom_boundry_max_angle):
                return get_length_ratio(angle, bottom_vertical_boundry_from_pivot)
            else:
                return 1

    def set_length(self, length: meters):  # set arm extension 
        def set(length):
            start_length = self.motor_extend.get_sensor_position()
            fin_length = (length / constants.max_elevator_height) * constants.elevator_extend_gear_ratio
            delta_length = fin_length - start_length
            self.motor_extend.set_target_position(delta_length)
        length_ratio = self.boundary_box(self.turn_sensor * (2 * math.pi))
        if length_ratio * constants.max_elevator_height > length:
            set(length)
            return True
        else:
            set(length_ratio * constants.max_elevator_height)
            return length_ratio * constants.max_elevator_height

    def shoulder_rotation_limits(self, radians: float):
        # returns if the angle is within the soft limits
        if self.rotation_overide:
            return True
        else:
            if self.intake_up:
                return radians < constants.shoulder_intake_up_max_rotation and radians > -constants.shoulder_min_rotation
            else:
                return radians < constants.shoulder_max_rotation and radians > -constants.shoulder_min_rotation

    def set_rotation(self, radians: float):  # set arm rotation
        
        def set(radians: float):
            rotations = radians / (2 * math.pi)
            current_rotation = self.main_rotation_motor.get_sensor_position()
            final_rotation = rotations - current_rotation
            self.main_rotation_motor.set_target_position(final_rotation)
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
        return self.motor_extend.get_sensor_position() / constants.elevator_extend_gear_ratio

    def get_rotation(self):  # returns arm rotation in radians
        return self.main_rotation_motor.get_sensor_position() * (2 * math.pi)

    def get_rotation_abs(self): 
        return self.abs_encoder_to_rad(self.turn_sensor.get_rotation())
    # brings elevator to zero position (no extension, no rotation)
    def zero_elevator_length(self):
        self.set_length(self.get_length()-0.005)

    def zero_elevator_rotation(self):
        #get Absolute encoder position
        abs_encoder_position: float = self.turn_sensor.get_position()
        #calculate the difference between the current position and the zero position
        encoder_difference: float = abs_encoder_position - constants.elevator_zero_rotation
        #convert absolute position ratio to motor rotations and gearbox ratio
        motor_position: float = encoder_difference * constants.elevator_rotation_gear_ratio
        #set motor position to the difference
        self.main_rotation_motor.set_sensor_position(motor_position)
        #run the motor to the zero position
        #self.main_rotation_motor.set_target_position(0)
        #https://codedocs.revrobotics.com/java/com/revrobotics/relativeencoder
        self.set_rotation(0)
    
    def extend_max_elevator(self):
        self.set_length(self.boundary_box(self.get_rotation()) *
                        constants.max_elevator_height)

    def update_pose(self):
        # updates the pose of the arm using the encoder values and rotation of the elevator
        angle = self.get_rotation()
        length = self.get_length()
        fn = math.modf(length)[0] * (2 * math.pi)
        x = fn * math.cos(angle)
        height = length * math.sin(angle)
        self.pose = Pose3d(
            x, 0, height + constants.pivot_point_height, 0, 0, 0)
