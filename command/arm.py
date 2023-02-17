import math
import time

import commands2
import rev
from commands2 import InstantCommand, SequentialCommandGroup, WaitCommand
from robotpy_toolkit_7407.command import SubsystemCommand
from wpimath._controls._controls.trajectory import TrapezoidProfileRadians
from wpimath.controller import ProfiledPIDControllerRadians

import constants
import utils
from robot_systems import Robot, Sensors
from subsystem import Arm
from units.SI import meters, radians


class ZeroElevator(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.motor_extend.set_sensor_position(0)

    def execute(self):
        # self.subsystem.zero_elevator_length()
        ...

    def isFinished(self):
        # return (
        #     self.subsystem.elevator_bottom_sensor.get()
        # )
        return True

    def end(self, interrupted=False):
        utils.logger.debug("Elevator", "Elevator Successfully Zeroed.")


class ZeroShoulder(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.disable_brake()
        self.subsystem.zero_elevator_rotation()

    def execute(self):
        ...

    def isFinished(self):
        # return not self.subsystem.elevator_bottom_sensor and \
        # self.subsystem.main_rotation_motor.get_sensor_position() == 0
        return abs(self.subsystem.arm_rotation_motor.get_sensor_position()) < 0.1

    def end(self, interrupted=False):
        utils.logger.debug("Shoulder", "Shoulder Successfully Zeroed.")
        if not interrupted:
            self.subsystem.enable_brake()
            self.subsystem.stop()


class ZeroWrist(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.zero_wrist()

    def execute(self):
        ...

    def isFinished(self):
        return abs(self.subsystem.wrist.get_sensor_position()) < 0.1

    def end(self, interrupted=False):
        utils.logger.debug("Wrist", "Wrist Successfully Zeroed.")


ZeroArm = lambda: commands2.SequentialCommandGroup(
    commands2.InstantCommand(lambda: Robot.Arm.disengage_claw()),
    ZeroElevator(Robot.Arm),
    ZeroShoulder(Robot.Arm),
    ZeroWrist(Robot.Arm),
)


class ManualMovement(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.rotate: float = 0
        self.claw_rotate: float = 0
        self.extend: float

    def initialize(self) -> None:
        self.subsystem.rotation_PID.setReference(
            10, rev.CANSparkMax.ControlType.kSmartMotion
        )

    def execute(self) -> None:
        ...
        # if abs(Keymap.Arm.ELEVATOR_ROTATION_AXIS.value) < 0.1:
        #     self.rotate = 0
        # else:
        #     self.rotate = Keymap.Arm.ELEVATOR_ROTATION_AXIS.value
        # self.subsystem.set_rotation(self.rotate * (2 * math.pi))
        # if abs(Keymap.Arm.CLAW_ROTATION_AXIS.value) < 0.1:
        #     self.claw_rotate = 0
        # else:
        #     self.claw_rotate = Keymap.Arm.CLAW_ROTATION_AXIS.value
        # self.subsystem.set_angle_wrist(self.claw_rotate * (2 * math.pi))
        # if abs(Keymap.Arm.ELEVATOR_EXTENSION_AXIS.value) < .05:
        #     self.extend = 0
        # else:
        #     self.elevator = Keymap.Arm.ELEVATOR_EXTENSION_AXIS.value
        # self.subsystem.set_angle_wrist(self.extend * (2 * math.pi))

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        pass


class ArmAssistedRobotStabilizer(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm):
        super().__init__(subsystem)
        self.pitch: float = 0
        self.elevator_rotation: float = 0

    def initialize(self) -> None:
        self.subsystem.disable_brake()

    def execute(self) -> None:
        self.pitch: float = Sensors.gyro.get_robot_pitch()
        self.elevator_rotation: float = self.pitch * constants.stabilizer_magnitude
        self.subsystem.set_rotation(self.elevator_rotation)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        if not interrupted:
            self.subsystem.enable_brake()
            self.subsystem.stop()


class HardStop(SubsystemCommand[Arm]):
    def initialize(self) -> None:
        self.subsystem.enable_brake()

    def execute(self) -> None:
        self.subsystem.stop()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted=False) -> None:
        if not interrupted:
            self.subsystem.enable_brake()
            self.subsystem.stop()


class EngageClaw(SubsystemCommand[Arm]):
    def initialize(self) -> None:
        self.subsystem.engage_claw()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted=False) -> None:
        ...


class DisengageClaw(SubsystemCommand[Arm]):
    def initialize(self) -> None:
        self.subsystem.disengage_claw()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted=False) -> None:
        ...


class CubeIntakeExtend(SubsystemCommand[Arm]):
    def initialize(self) -> None:
        self.subsystem.set_rotation(math.radians(45))
        self.subsystem.set_angle_wrist(math.radians(45))
        self.subsystem.engage_claw()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted=False) -> None:
        pass


class CubeIntakeRetract(SubsystemCommand[Arm]):
    def initialize(self) -> None:
        self.subsystem.set_rotation(math.radians(0))
        self.subsystem.set_angle_wrist(math.radians(0))
        self.subsystem.disengage_claw()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted=False) -> None:
        pass


class SetArm(SubsystemCommand[Arm]):
    def __init__(
        self,
        subsystem: Arm,
        distance: meters,
        shoulder_angle: radians,
        wrist_angle: radians,
        claw_active: bool = False,
    ):
        super().__init__(subsystem)
        self.distance = distance
        self.shoulder_angle = shoulder_angle
        self.wrist_angle = wrist_angle
        self.claw_active = claw_active

        self.arm_controller: ProfiledPIDControllerRadians | None = None

        self.start_time = None
        self.theta_i = 0
        self.theta_f = 0
        self.theta_diff = 0
        self.threshold = math.radians(1)

        self.desired_time = 3

    def initialize(self):
        if self.shoulder_angle > self.subsystem.get_rotation():
            commands2.CommandScheduler.getInstance().schedule(
                SequentialCommandGroup(
                    WaitCommand(0.5),
                    InstantCommand(
                        lambda: self.subsystem.set_angle_wrist(self.wrist_angle)
                    ),
                    InstantCommand(lambda: self.subsystem.set_length(self.distance)),
                )
            )
        else:
            commands2.CommandScheduler.getInstance().schedule(
                SequentialCommandGroup(
                    InstantCommand(
                        lambda: self.subsystem.set_angle_wrist(self.wrist_angle)
                    ),
                    InstantCommand(lambda: self.subsystem.set_length(self.distance)),
                )
            )

        if self.claw_active:
            self.subsystem.engage_claw()

        self.arm_controller = ProfiledPIDControllerRadians(
            1,
            0,
            0,
            TrapezoidProfileRadians.Constraints(0.2, 0.05),
        )

        self.start_time = time.perf_counter()
        self.theta_i = self.subsystem.get_rotation()
        self.theta_f = self.shoulder_angle

    def execute(self) -> None:
        self.subsystem.update_pose()
        current_time = time.perf_counter() - self.start_time
        current_theta = self.subsystem.get_rotation()

        maximum_power = 0.2

        self.subsystem.arm_rotation_motor.set_raw_output(
            min(
                maximum_power,
                self.arm_controller.calculate(
                    current_theta,
                    self.theta_f,
                ),
            )
        )

    def isFinished(self) -> bool:
        return self.subsystem.is_at_position(
            self.distance, self.shoulder_angle, self.wrist_angle
        )

    def end(self, interrupted: bool) -> None:
        self.subsystem.arm_rotation_motor.set_target_velocity(0)
        self.subsystem.disengage_claw()
        # self.subsystem.set_angle_wrist(math.radians(0))
        # self.subsystem.set_rotation(math.radians(0))
        # self.subsystem.set_angle_wrist(math.radians(0))
