from robotpy_toolkit_7407.command import SubsystemCommand
import commands2
from robot_systems import Robot
from subsystem import Intake
from commands2 import InstantCommand, SequentialCommandGroup
import command
class IntakeEnable(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem
        
    def initialize(self):
        Robot.intake.intake_enable()
    def execute(self):
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted=False):
        # InstantCommand(command.ZeroArm(Robot.Arm))


class IntakeDisable(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        Robot.intake.intake_disable()

    def execute(self):
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted=False):
        pass
