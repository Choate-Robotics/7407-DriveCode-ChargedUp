from robotpy_toolkit_7407.command import SubsystemCommand, BasicCommand

from robot_systems import Robot
from subsystem import Intake

class TestCommand(BasicCommand):
    def __init__(self):
        super().__init__()

    def initialize(self) -> None:
        print("INTIAL")

    def execute(self):
        print("HI")
        raise TypeError
    
    def isFinished(self) -> bool:
        return False
    
    def end(self):
        print("DONE")

    def runsWhenDisabled(self) -> bool:
        return True


class IntakeEnable(SubsystemCommand[Intake]):
    def __init__(self, subsystem: Intake):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        print("hi")
        # Robot.intake.intake_enable()

    def execute(self):
        print("HI")
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted=False):
        pass


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
