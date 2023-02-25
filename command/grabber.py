from robotpy_toolkit_7407 import SubsystemCommand
from wpilib import SmartDashboard

import utils
from subsystem import Grabber
from units.SI import radians


class SetGrabber(SubsystemCommand[Grabber]):
    def __init__(self, subsystem: Grabber, wrist_angle: radians, claw_active: bool):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.wrist_angle = wrist_angle
        self.claw_active = claw_active
        self.wait_for_arm = False

    def initialize(self) -> None:
        print("RUNNING GRABBER")
        self.subsystem.set_angle(self.wrist_angle)
        if self.claw_active:
            self.subsystem.engage_claw()
        else:
            self.subsystem.disengage_claw()

    def execute(self) -> None:
        SmartDashboard.putNumber("WRIST_CURRENT", self.subsystem.get_angle())

        SmartDashboard.putNumber("WRIST_TARGET", self.wrist_angle)

        SmartDashboard.putNumber(
            "WRIST_ERROR", self.wrist_angle - self.subsystem.get_angle()
        )
        ...

    def isFinished(self) -> bool:
        return self.subsystem.is_at_angle(self.wrist_angle)

    def end(self, interrupted: bool) -> None:
        print("FINISHED GRABBER SET")
        ...


class ZeroWrist(SubsystemCommand[Grabber]):
    def __init__(self, subsystem: Grabber):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.zero_wrist()

    def execute(self):
        ...

    def isFinished(self):
        return True

    def end(self, interrupted=False):
        utils.logger.debug("Wrist", "Wrist Successfully Zeroed.")
        print("WRIST ZEROED")
