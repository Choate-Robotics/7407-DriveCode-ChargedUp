import wpilib
from robotpy_toolkit_7407.command import SubsystemCommand

import config
import constants
from subsystem import Climber
from robot_systems import Robot


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
        Robot.climber.pivot()
        # Robot.climber.climber_motor.brake

    def execute(self):
        pass
    
    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted=False):
        pass



class ClimberDisable(SubsystemCommand[Climber]):
    def __init__(self, subsystem: Climber):
        super().__init__(subsystem)
        self.subsystem = subsystem
    
    def initialize(self):
        Robot.climber.climber_disable()
    
    def execute(self):
        pass
    
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted=False):
        pass
