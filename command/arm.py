import math

from robotpy_toolkit_7407.command import SubsystemCommand

import constants
import utils
from oi.keymap import Keymap
from robot_systems import Sensors
from subsystem import Arm
from units.SI import meters, radians


class ZeroArm(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.disable_brake()
        self.subsystem.zero_elevator_rotation()
        self.subsystem.zero_wrist()
        self.subsystem.motor_extend.set_sensor_position(
            0
        )  # This is because the limit switches are not in the robot yet

    def execute(self):
        # self.subsystem.zero_elevator_length()
        # print(round(self.subsystem.main_rotation_motor.get_sensor_position()))
        ...

    def isFinished(self):
        # return not self.subsystem.elevator_bottom_sensor and \
        # self.subsystem.main_rotation_motor.get_sensor_position() == 0
        return (
                round(self.subsystem.main_rotation_motor.get_sensor_position()) == 0
                and round(self.subsystem.wrist.get_sensor_position()) == 0
        )

    def end(self, interrupted=False):
        utils.logger.debug("ARM", "Arm Successfully Zeroed.")
        if not interrupted:
            self.subsystem.enable_brake()
            self.subsystem.stop()


class ManualMovement(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.rotate: float = 0
        self.claw_rotate: float = 0
        self.extend: float

    def initialize(self) -> None:
        ...

    def execute(self) -> None:
        if abs(Keymap.Arm.ELEVATOR_ROTATION_AXIS.value) < 0.05:
            self.rotate = 0
        else:
            self.rotate = Keymap.Arm.ELEVATOR_ROTATION_AXIS.value
        self.subsystem.set_rotation(self.rotate * (2 * math.pi))
        if abs(Keymap.Arm.CLAW_ROTATION_AXIS.value) < 0.05:
            self.claw_rotate = 0
        else:
            self.claw_rotate = Keymap.Arm.CLAW_ROTATION_AXIS.value
        self.subsystem.set_angle_wrist(self.claw_rotate * (2 * math.pi))
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


class CubeIntake(SubsystemCommand[Arm]):
    def initialize(self) -> None:
        self.subsystem.set_rotation(math.radians(90))
        self.subsystem.set_angle_wrist(math.radians(90))
        self.subsystem.engage_claw()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted=False) -> None:
        self.subsystem.set_rotation(math.radians(0))
        self.subsystem.set_angle_wrist(math.radians(0))


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

    def initialize(self):
        self.subsystem.set_rotation(self.shoulder_angle)
        self.subsystem.set_angle_wrist(self.wrist_angle)
        self.subsystem.set_length(self.distance)
        if self.claw_active:
            self.subsystem.engage_claw()

    def execute(self) -> None:
        self.subsystem.update_pose()

    def isFinished(self) -> bool:
        return self.subsystem.is_at_position(
            self.distance, self.shoulder_angle, self.wrist_angle
        )

    def end(self, interrupted: bool) -> None:
        self.subsystem.disengage_claw()
        self.subsystem.set_angle_wrist(math.radians(0))
        self.subsystem.set_rotation(math.radians(0))
        self.subsystem.set_angle_wrist(math.radians(0))
