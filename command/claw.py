from robotpy_toolkit_7407.command import SubsystemCommand

from subsystem import Claw

class ClawCommand(SubsystemCommand[Claw]):

    def __init__(self, subsystem: Claw):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.subsystem.init()
        self.subsystem.zero()
        
