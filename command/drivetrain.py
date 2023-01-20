from robotpy_toolkit_7407.command import SubsystemCommand

from subsystem import Drivetrain


def curve_abs(x):
    return x**2.4

def curve(x):
    if x < 0:
        return -curve_abs(-x)
    return curve_abs(x)

class DrivetrainZero(SubsystemCommand[Drivetrain]):
    def __init__(self, subsystem: Drivetrain):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def zero(self):
        self.subsystem.n_00.zero()
        self.subsystem.n_01.zero()
        self.subsystem.n_10.zero()
        self.subsystem.n_11.zero()

        self.subsystem.n_00.set_motor_angle(0)
        self.subsystem.n_01.set_motor_angle(0)
        self.subsystem.n_10.set_motor_angle(0)
        self.subsystem.n_11.set_motor_angle(0)

    def zero_success(self):
        threshold = 0.02

        success = True

        for i in [
            self.subsystem.n_00,
            self.subsystem.n_01,
            self.subsystem.n_10,
            self.subsystem.n_11,
        ]:
            if not (
                abs(i.encoder.getAbsolutePosition() - i.encoder_zeroed_absolute_pos)
                < threshold
            ):
                success = False

        return success

    def initialize(self) -> None:
        ...

    def execute(self) -> None:
        self.zero()

    def isFinished(self) -> bool:
        return self.zero_success()

    def end(self, interrupted: bool) -> None:
        ...