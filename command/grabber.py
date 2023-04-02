import wpilib
from robotpy_toolkit_7407 import SubsystemCommand

import config
from oi.keymap import Controllers
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
        no_grab: bool = False,
        threshold: float | None = None,
        finish: bool = True,
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

        self.no_grab = no_grab

        self.finished = not auto_claw
        self.finish = finish

    def initialize(self) -> None:
        self.subsystem.set_angle(self.wrist_angle)
        config.grabber_target_angle = self.wrist_angle
        if self.claw_active:
            self.subsystem.engage_claw_cube()
        else:
            self.subsystem.claw_motor.set_raw_output(0.05)

    def execute(self) -> None:
        self.subsystem.set_angle(config.grabber_target_angle)

        if self.auto_claw:
            if (
                self.auto_cube
                and self.no_grab
                and self.subsystem.get_no_grab_cube_detected()
            ):
                self.finished = True
                Controllers.OPERATOR_CONTROLLER.setRumble(
                    wpilib.Joystick.RumbleType.kBothRumble, 0.3
                )
            elif (
                self.auto_cube
                and not self.no_grab
                and self.subsystem.get_cube_detected()
            ):
                self.subsystem.disengage_claw()
                self.finished = True
                Controllers.OPERATOR_CONTROLLER.setRumble(
                    wpilib.Joystick.RumbleType.kBothRumble, 0.3
                )
                config.grabber_disable_intake = True
            elif (
                self.auto_cone
                and not self.no_grab
                and self.subsystem.get_cone_detected()
            ):
                self.subsystem.disengage_claw()
                self.finished = True
                Controllers.OPERATOR_CONTROLLER.setRumble(
                    wpilib.Joystick.RumbleType.kBothRumble, 0.3
                )
            elif (
                self.auto_double
                and not self.no_grab
                and self.subsystem.get_double_station_detected()
            ):
                self.subsystem.disengage_claw()
                self.finished = True
        ...

    def isFinished(self) -> bool:
        if self.threshold is not None:
            self.subsystem.is_at_angle(
                self.wrist_angle, threshold=self.threshold
            ) and self.finished and self.finish
        else:
            return (
                self.subsystem.is_at_angle(self.wrist_angle)
                and self.finished
                and self.finish
            )

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
