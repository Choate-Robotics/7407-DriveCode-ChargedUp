from robotpy_toolkit_7407.command import SubsystemCommand

from subsystem import Drivetrain


def curve_abs(x):
    return x**2.4


def curve(x):
    if x < 0:
        return -curve_abs(-x)
    return curve_abs(x)


class DriveSwerveCustom(SubsystemCommand[Drivetrain]):
    driver_centric = True
    driver_centric_reversed = False

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        dx, dy, d_theta = (
            self.subsystem.axis_dx.value,
            self.subsystem.axis_dy.value,
            -self.subsystem.axis_rotation.value,
        )

        if abs(d_theta) < 0.15:
            d_theta = 0

        dx = curve(dx)
        dy = curve(dy)
        d_theta = curve(d_theta)

        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel

        if DriveSwerveCustom.driver_centric:
            self.subsystem.set_driver_centric(
                (-dy, dx), d_theta * self.subsystem.max_angular_vel
            )
        elif DriveSwerveCustom.driver_centric_reversed:
            self.subsystem.set_driver_centric(
                (dy, -dx), d_theta * self.subsystem.max_angular_vel
            )
        else:
            self.subsystem.set_robot_centric(
                (dx, dy), d_theta * self.subsystem.max_angular_vel
            )

    def end(self, interrupted: bool) -> None:
        self.subsystem.n_00.set(0, 0)
        self.subsystem.n_01.set(0, 0)
        self.subsystem.n_10.set(0, 0)
        self.subsystem.n_11.set(0, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


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
        self.subsystem.gyro.reset_angle()
        return self.zero_success()

    def end(self, interrupted: bool) -> None:
        # for i in [self.subsystem.n_00, self.subsystem.n_01, self.subsystem.n_10, self.subsystem.n_11]:
        #     i.m_turn.set_sensor_position(0)
        ...
