from robotpy_toolkit_7407 import SubsystemCommand

from subsystem import Grabber
from units.SI import radians


class SetGrabber(SubsystemCommand[Grabber]):
    def __init__(
            self,
            subsystem: Grabber,
            wrist_angle: radians,
            claw_active: bool,
            auto_claw: bool = False,
            auto_cube: bool = False,
            auto_cone: bool = False,
            auto_double: bool = False,
            threshold: float | None = None,
    ):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.wrist_angle = wrist_angle
        self.claw_active = claw_active
        self.auto_claw = auto_claw
        self.auto_cube = auto_cube
        self.auto_cone = auto_cone
        self.auto_double = auto_double
        self.threshold = threshold

        self.finished = not auto_claw

    def initialize(self) -> None:
        self.subsystem.set_angle(self.wrist_angle)
        if self.claw_active:
            self.subsystem.engage_claw()
        else:
            self.subsystem.disengage_claw()

    def execute(self) -> None:
        if self.auto_claw:
            if self.auto_cube and self.subsystem.get_cube_detected():
                self.subsystem.disengage_claw()
                self.finished = True
            elif self.auto_cone and self.subsystem.get_cone_detected():
                self.subsystem.disengage_claw()
                self.finished = True
            elif self.auto_double and self.subsystem.get_double_station_detected():
                self.subsystem.disengage_claw()
                self.finished = True
        ...

    def isFinished(self) -> bool:
        if self.threshold is not None:
            self.subsystem.is_at_angle(
                self.wrist_angle, threshold=self.threshold
            ) and self.finished
        else:
            return self.subsystem.is_at_angle(self.wrist_angle) and self.finished

    def end(self, interrupted: bool) -> None:
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
        ...
