import math
import time

import rev
from robotpy_toolkit_7407.command import SubsystemCommand
from wpilib import SmartDashboard
from wpimath.controller import ArmFeedforward, PIDController

import constants
import utils
from robot_systems import Sensors
from subsystem import Arm, Grabber
from units.SI import meters, radians


class ZeroElevator(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        ...
        self.subsystem.motor_extend.set_sensor_position(0)

    def execute(self):
        self.subsystem.zero_elevator_length()
        print("ZEROING")
        ...

    def isFinished(self):
        return self.subsystem.elevator_bottom_sensor.get()

    def end(self, interrupted=False):
        self.subsystem.motor_extend.set_sensor_position(0)
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


class ZeroWrist(SubsystemCommand[Grabber]):
    def __init__(self, subsystem: Grabber):
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


class setShoulderRotation(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm, shoulder_angle):
        super().__init__(subsystem)
        self.shoulder_angle = shoulder_angle
        self.arm_controller: PIDController | None = None
        self.arm_ff: ArmFeedforward | None = None
        self.start_time = None
        self.theta_i = 0
        self.theta_f = 0
        self.theta_diff = 0
        self.threshold = math.radians(1)
        self.desired_time = 3

    def initialize(self) -> None:
        self.arm_controller = PIDController(1, 0, 0.03)

        self.arm_ff = ArmFeedforward(kG=0.045, kS=0, kV=0, kA=0)  # perfect dont touch

        self.start_time = time.perf_counter()
        self.theta_i = self.subsystem.get_rotation()
        self.theta_f = self.shoulder_angle

        if not self.subsystem.is_at_shoulder_rotation(self.shoulder_angle):
            self.subsystem.disable_brake()

    def execute(self) -> None:
        current_theta = self.subsystem.get_rotation()

        maximum_power = 0.5 * self.subsystem.arm_rotation_motor.motor.getBusVoltage()
        # print(math.pi/2 - self.theta_f)
        # print("Actual Theta:", current_theta)
        # print("Theta:", math.pi/2 - current_theta)

        feed_forward = -self.arm_ff.calculate(
            angle=(math.pi / 2 - current_theta), velocity=0.1, acceleration=0
        )
        # print("Voltage", desired_voltage)
        pid_voltage = -(
            self.arm_controller.calculate(
                math.pi / 2 - current_theta,  # sets correct origin
                self.theta_f,
            )
        )
        # if abs(pid_voltage) < .0005:
        #     pid_voltage = 0
        # print(self.subsystem.is_at_shoulder_rotation(math.pi/2 - self.shoulder_angle))
        if self.subsystem.is_at_shoulder_rotation(math.pi / 2 - self.shoulder_angle):
            pid_voltage = 0
        desired_voltage = (
            feed_forward + pid_voltage
        ) * self.subsystem.arm_rotation_motor.motor.getBusVoltage()
        # print(self.shoulder_angle)
        print("SHOULDER: ", self.shoulder_angle)
        print("CURRENT: ", current_theta)

        SmartDashboard.putNumber("PID_Voltage", pid_voltage)
        self.subsystem.arm_rotation_motor.pid_controller.setReference(
            min(maximum_power, abs(desired_voltage))
            * (1 if desired_voltage > 0 else -1),
            rev.CANSparkMax.ControlType.kVoltage,
            pidSlot=1,
        )

    def isFinished(self) -> bool:
        return self.subsystem.is_at_shoulder_rotation(self.shoulder_angle)

    def end(self, interrupted: bool) -> None:
        pass


class SetArm(SubsystemCommand[Arm]):
    def __init__(
        self,
        subsystem: Arm,
        distance: meters,
        shoulder_angle: radians,
    ):
        super().__init__(subsystem)
        self.real_desired = shoulder_angle
        self.distance = distance
        self.shoulder_angle = (math.pi / 2) - shoulder_angle

        self.arm_controller: PIDController | None = None
        self.arm_ff: ArmFeedforward | None = None
        self.start_time = None
        self.theta_i = 0
        self.theta_f = 0
        self.theta_diff = 0
        self.threshold = math.radians(5)

        self.desired_time = 3

    def initialize(self):
        self.arm_controller = PIDController(1, 0, 0.03)

        self.arm_ff = ArmFeedforward(kG=0.045, kS=0, kV=0, kA=0)  # perfect don't touch

        self.start_time = time.perf_counter()
        self.theta_i = self.subsystem.get_rotation()
        self.theta_f = self.shoulder_angle

        if not self.subsystem.is_at_position(self.distance, self.shoulder_angle):
            self.subsystem.disable_brake()

    def execute(self) -> None:
        SmartDashboard.putNumber("ARM_CURRENT", self.subsystem.get_rotation())

        SmartDashboard.putNumber("ARM_TARGET", self.real_desired)

        SmartDashboard.putNumber(
            "ARM_ERROR", self.real_desired - self.subsystem.get_rotation()
        )

        # print("RUNNING")
        self.subsystem.update_pose()
        current_theta = self.subsystem.get_rotation()

        maximum_power = 0.8 * self.subsystem.arm_rotation_motor.motor.getBusVoltage()

        feed_forward = -self.arm_ff.calculate(
            angle=(math.pi / 2 - current_theta), velocity=0.1, acceleration=0
        )
        pid_voltage = -(
            self.arm_controller.calculate(
                math.pi / 2 - current_theta,
                self.theta_f,
            )
        )

        if abs(self.subsystem.get_rotation() - self.real_desired) < self.threshold:
            pid_voltage = 0
        else:
            print(self.subsystem.get_rotation(), self.real_desired)

        desired_voltage = (
            feed_forward + pid_voltage
        ) * self.subsystem.arm_rotation_motor.motor.getBusVoltage()
        print(desired_voltage)
        SmartDashboard.putNumber("PID_Voltage", pid_voltage)
        self.subsystem.arm_rotation_motor.pid_controller.setReference(
            min(maximum_power, abs(desired_voltage))
            * (1 if desired_voltage > 0 else -1),
            rev.CANSparkMax.ControlType.kVoltage,
            1,
        )

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        if not interrupted:
            self.subsystem.enable_brake()
            self.subsystem.arm_rotation_motor.pid_controller.setReference(
                0, rev.CANSparkMax.ControlType.kVoltage, 0
            )
