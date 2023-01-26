from robotpy_toolkit_7407.command import SubsystemCommand

from subsystem import Claw

class ClawCommand(SubsystemCommand[Claw]):

    def __init__(self, subsystem: Claw):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:   
        self.subsystem.zero()

    def execute(self) -> None:
        self.isFinished()

    def isFinished(self) -> bool:
        return round(self.subsystem.get_angle()) == 0
    
    def end(self, interrupted: bool) -> None:
        pass
        
