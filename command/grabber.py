from robotpy_toolkit_7407 import SubsystemCommand

from subsystem import Grabber
from units.SI import radians


class SetGrabber(SubsystemCommand[Grabber]):
    def __init__(self, subsystem: Grabber, wrist_angle: radians, claw_active: bool):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.wrist_angle = wrist_angle
        self.claw_active = claw_active

    def initialize(self) -> None:
        self.subsystem.set_angle(self.wrist_angle)
        if self.claw_active:
            self.subsystem.engage_claw()
        else:
            self.subsystem.disengage_claw()

    def execute(self) -> None:
        ...

    def isFinished(self) -> bool:
        return self.subsystem.is_at_angle(self.wrist_angle)

    def end(self, interrupted: bool) -> None:
        ...
