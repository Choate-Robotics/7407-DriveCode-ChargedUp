import wpilib
from robotpy_toolkit_7407.command import SubsystemCommand

import config
import constants
from subsystem import Climber
from robot_systems import Robot, Sensors
import math


class ClimberZero(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        Robot.climber.zero()
        Robot.climber.climber_disable()
    
    def execute(self):
        pass

    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted=True):
        pass


'''
    TODO: Determine if we even need this command
'''
class ClimberEnable(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem
    
    def initialize(self):
        Robot.climber.climber_deploy()

    def execute(self):
        pass
    
    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted=False):
        pass


class ClimberPivot(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem
    
    def initialize(self):
        # self.turn_reversed = Robot.climber.pivot_threshold < Robot.climber.get_angle()
        Robot.climber.climber_motor.set_raw_output(Robot.climber.pivot_speed)

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return Sensors.gyro.get_robot_pitch() >= Robot.climber.pivot_threshold
    
    def end(self, interrupted=False):
        Robot.climber.climber_motor.set_raw_output(0)
        Robot.climber.climber_motor.brake.set(True)



# class ClimberDisable(SubsystemCommand[Climber]):
#     def __init__(self, subsystem: Climber):
#         super().__init__(subsystem)
#         self.subsystem = subsystem
    
#     def initialize(self):
#         Robot.climber.climber_disable()
    
#     def execute(self):
#         pass
    
#     def isFinished(self) -> bool:
#         return False
    
#     def end(self, interrupted=False):
#         pass
