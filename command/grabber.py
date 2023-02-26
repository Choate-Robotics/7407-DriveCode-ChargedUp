from robotpy_toolkit_7407 import SubsystemCommand
from wpilib import SmartDashboard

import utils
from subsystem import Grabber, Intake
from units.SI import radians


class SetGrabberIntake(SubsystemCommand[Grabber]):
    def __init__(
        self,
        subsystem: Grabber,
        intake: Intake,
        wrist_angle: radians,
        claw_active: bool,
        intake_active: bool,
        intake_reversed: bool,
        auto_claw: bool = False,
    ):
        super().__init__(subsystem)
        super().addRequirements(self.intake)
        self.subsystem = subsystem
        self.intake = intake
        self.wrist_angle = wrist_angle
        self.claw_active = claw_active
        self.intake_active = intake_active
        self.intake_reversed = intake_reversed
        self.wait_for_arm = False
        self.auto_claw = auto_claw
        self.finished = not self.auto_claw

    def initialize(self) -> None:
        self.subsystem.set_angle(self.wrist_angle)
        if self.claw_active:
            self.subsystem.engage_claw()
        else:
            self.subsystem.disengage_claw()

        if self.intake_active:
            self.intake.intake_enable(intake_reversed=self.intake_reversed)
        else:
            self.intake.intake_disable()

    def execute(self) -> None:
        if self.subsystem.get_detected_farther_away() and self.auto_claw:
            self.intake.intake_motor.set_raw_output(0)

        if self.subsystem.get_detected() and self.auto_claw:
            self.subsystem.disengage_claw()
            self.finished = True

    def isFinished(self) -> bool:
        return self.subsystem.is_at_angle(self.wrist_angle) and self.finished


class SetGrabber(SubsystemCommand[Grabber]):
    def __init__(
        self,
        subsystem: Grabber,
        intake: Intake,
        wrist_angle: radians,
        claw_active: bool,
        auto_claw: bool = False,
    ):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.intake = intake
        self.wrist_angle = wrist_angle
        self.claw_active = claw_active
        self.wait_for_arm = False
        self.auto_claw = auto_claw
        self.finished = not self.auto_claw

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

        if self.subsystem.get_detected_farther_away() and self.auto_claw:
            self.intake.intake_motor.set_raw_output(0)

        if self.subsystem.get_detected() and self.auto_claw:
            self.subsystem.disengage_claw()
            self.finished = True
        ...

    def isFinished(self) -> bool:
        return self.subsystem.is_at_angle(self.wrist_angle) and self.finished

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
