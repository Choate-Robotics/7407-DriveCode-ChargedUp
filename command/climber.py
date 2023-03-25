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



class OperatorControl(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem
    
    def initialize(self):
        if not Robot.climber.climber_active:
            Robot.climber.climber_deploy()
        else:
            Robot.climber.climber_disable()

    def execute(self):
        pass
    
    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted=False):
        pass


class ClimberDeploy(SubsystemCommand[Climber]):
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

class ClimberRetract(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem
    
    def initialize(self):
        Robot.climber.climber_disable()

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
        Robot.climber.pivot()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted=False):
        pass

class ClimberUnpivot(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem
    
    def initialize(self):
        # self.turn_reversed = Robot.climber.pivot_threshold < Robot.climber.get_angle()
        Robot.climber.unpivot()
    def execute(self):
        pass

    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted=False):
        pass
    
    
class ClimberToZero(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem
    
    def initialize(self):
        # self.turn_reversed = Robot.climber.pivot_threshold < Robot.climber.get_angle()
        Robot.climber.latch_disable()
        Robot.climber.climber_motor.set_target_position(0)

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted=False):
        Robot.climber.latch_enable()



