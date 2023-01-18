import wpilib
from robotpy_toolkit_7407 import Subsystem
from robotpy_toolkit_7407.motors.revmotor import SparkMax
from robotpy_toolkit_7407.utils.units import meters
from robotpy_toolkit_7407.sensors.limit_switches.limit_switch import LimitSwitch

import constants
import config
#importing packages


class Elevator(Subsystem): #elevator class
    extendMotor: SparkMax = SparkMax(config.elevator_motor_extend_id)#motor that extends the arm
    rotationMotorRight: SparkMax = SparkMax(config.elevator_motor_turn_right_id, inverted = True) #motor that rotates the arm
    rotationMotorLeft: SparkMax = SparkMax(config.elevator_motor_turn_left_id, inverted = False) #motor that rotates the arm
    initialized: bool = False
    extend_sensor = LimitSwitch(9) #senses when extension is at zero
    turn_sensor = LimitSwitch(8) #senses when turn is at zero

    def init(self): #initializing motors
        self.extendMotor.init()
        self.rotationMotorRight.init()
        self.rotationMotorLeft.init()
        
    
    def set_length(self, h: meters): #set arm extension
        self.extendMotor.set_target_position(h * constants.elevator_gear_ratio)

    def set_rotation(self, h: meters): #set arm rotation
        self.rotationMotorRight.set_target_position(h * constants.elevator_gear_ratio)
        self.rotationMotorLeft.set_target_position(h* constants.elevator_gear_ratio)

    def get_length(self): #returns arm extension
        return self.extendMotors.get_sensor_position() / constants.elevator_gear_ratio
    
    def get_rotation(self, h: meters): #returns arm rotation
        return self.rotationMotorLeft.get_sensor_position() / constants.elevator_gear_ratio
    

    def zero_elevator(self): #brings elevator to zero position (no extension, no rotation)
        while(self.extend_sensor.get_value() == False):
            h = self.get_length()
            h -= 0.005
            self.set_length(h)
        #We didnt know where we were before, so after we have reached the zero position we tell the motor sensor this is zero. Like Zeroing the CNC machine.
        self.extendMotor.set_target_position(0)
        self.extendMotor.set_sensor_position(0)

        while(self.turn_sensor.get_value() == False):
            h = self.get_length()
            h -= 0.005
            self.set_rotation(h)
         # reset motor's sensor position to 0 
        self.rotationMotorRight.set_target_position(0)
        self.rotationMotorRight.set_sensor_position(0)
        self.rotationMotorLeft.set_target_position(0)
        self.rotationMotorLeft.set_sensor_position(0)
    
    
    def extend_max_elevator(self):
        self.set_height(1)
    '''def set_speed(self):
     #motor_cfg = SparkMax(motion_cruise_velocity=1500*Sparkmax_motors.k_sensor_vel_to_rad_per_sec,motion_acceleration=5000*Sparkmax_motors.k_sensor_accel_to_rad_per_sec_sq)

    for m in self.extendMotors.motors:
            m._set_config(motor_cfg)'''
    

    

    
    
    
    