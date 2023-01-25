import wpilib
import rev
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.rev_motors import SparkMax
from robotpy_toolkit_7407.utils.units import meters
from robotpy_toolkit_7407.sensors.limit_switches.limit_switch import LimitSwitch
import math
import constants
import config
#importing packages


class Elevator(Subsystem): #elevator class
    motor_extend: SparkMax = SparkMax(config.elevator_motor_extend_id)#motor that extends the arm
    right_rotation_motor: SparkMax = SparkMax(config.elevator_right_rotation_motor_id, inverted = True) #motor that rotates the arm
    left_rotation_motor: SparkMax = SparkMax(config.elevator_left_rotation_motor_id, inverted = False) #motor that rotates the arm
    brake: wpilib.Solenoid = wpilib.Solenoid(1, wpilib.PneumaticsModuleType.REVPH, config.elevator_brake_id) #brake that holds the arm in place
    initialized: bool = False
    extend_sensor = LimitSwitch(9) #senses when extension is at zero
    turn_sensor = rev.CANSparkMax(config.elevator_left_rotation_motor_id, rev.MotorType.kBrushless).getAbsoluteEncoder(rev._rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle) #senses the rotation of the elevator



    def init(self): #initializing motors
        self.motor_extend.init()
        self.right_rotation_motor.init()
        self.left_rotation_motor.init()
        self.right_rotation_motor.follow(config.elevator_left_rotation_motor_id, True)
        self.brake.set(True)
        
    
    def set_length(self, h: meters): #set arm extension
        self.motor_extend.set_target_position(h * constants.elevator_gear_ratio)

    def set_rotation(self, h: meters): #set arm rotation
        self.right_rotation_motor.set_target_position(h * constants.elevator_gear_ratio)
        self.left_rotation_motor.set_target_position(h* constants.elevator_gear_ratio)

    def get_length(self): #returns arm extension
        return self.motor_extend.get_sensor_position() / constants.elevator_gear_ratio
    
    def get_rotation(self, h: meters): #returns arm rotation
        return self.left_rotation_motor.get_sensor_position() / constants.elevator_gear_ratio
    
    def enable_brake(self):
        self.brake.set(True)

    def disable_brake(self):
        self.brake.set(False)

    def zero_elevator(self): #brings elevator to zero position (no extension, no rotation)
        self.set_length(self.get_length()-0.005)
        self.set_rotation(self.get_rotation()-0.005)
    
    
    def extend_max_elevator(self):
        self.set_height(config.elevator_max_height)

    
    def boundary_box(self, degree):

        degree = abs(degree)

        def get_length_ratio(angle, adjacent):
            #Gets the length of the angle using cosine
            max_length = adjacent / math.cos(angle)
            #if the length of the angle is greater than the max_elevator_height, return one
            #else give a percentage of the height of the current line and the max_elevator_height
            #protects against any irregular numbers heigher than the max elevator height by always 
            #defaulting to 1
            if max_length > constants.max_elevator_height:
                return 1
            else:
                return max_length / constants.max_elevator_height #where is the hype at 

        #Finds the length of each the dimensions for the boundry box using the constants
        top_box_height = constants.vertical_boundary - constants.pivot_point_height
        bottom_box_height = constants.pivot_point_height
        horizontal_length = 0.5(constants.robot_length) + constants.horizontal_boundary


        #finds the angle on each side where the length is equal to the max_elevator_height
        top_box_angle = math.acos(top_box_height/constants.max_elevator_height)
        side_box_angle = math.acos(horizontal_length/constants.max_elevator_height)
        bottom_box_angle = math.acos(bottom_box_height/constants.max_elevator_height)
        
        #if the degree is within the minimun range of each of these boundry sides

        # assuming the top is zero...

        #if the degree is within the max_elevator_extension and facing the top
        if degree < top_box_angle:
            return get_length_ratio(degree, top_box_angle)
        #if the degree is within the max_elevator_extension and facing the sides
        elif (degree) > side_box_angle and degree < side_box_angle+(90 - side_box_angle):
            return get_length_ratio( 90- degree, side_box_angle)
        #if the degree is within the max_elevator_extension and facing the bottom
        elif (degree) > bottom_box_angle:
            return get_length_ratio( 180 - degree, bottom_box_angle)

        