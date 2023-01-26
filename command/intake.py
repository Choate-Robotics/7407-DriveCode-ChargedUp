import wpilib
from robotpy_toolkit_7407.command import SubsystemCommand

import config
import constants
from subsystem import Intake
from robot_systems import Robot

class IntakeEnable(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem
    
    def execute(self):
        Robot.intake.intake_enable()
    
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted=False):
        pass

class IntakeDisable(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem
    
    def execute(self):
        Robot.intake.intake_disable()
    
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted=False):
        pass