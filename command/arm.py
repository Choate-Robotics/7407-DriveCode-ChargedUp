import math
import time

import rev
from robotpy_toolkit_7407.command import SubsystemCommand
from wpilib import SmartDashboard
from wpimath.controller import ArmFeedforward, PIDController

import constants
import utils
from subsystem import Arm
from units.SI import meters, radians


class ZeroElevator(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        ...
        self.subsystem.motor_extend.set_sensor_position(0)

    def execute(self):
        self.subsystem.motor_extend.set_raw_output(-0.05)
        print("ZEROING")
        ...

    def isFinished(self):
        return self.subsystem.elevator_bottom_sensor.get()

    def end(self, interrupted=False):
        self.subsystem.motor_extend.set_sensor_position(0)
        print("Zeroed MEOW")
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
        return True

    def end(self, interrupted=False):
        print("SHOULDER ZEROED")
        utils.logger.debug("Shoulder", "Shoulder Successfully Zeroed.")
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
        self.threshold = math.radians(2.5)

        self.desired_time = 3

    def initialize(self):
        print("RUNNING ARM SET")
        self.arm_controller = PIDController(1, 0, 0.03)

        self.arm_ff = ArmFeedforward(kG=0.045, kS=0, kV=0, kA=0)  # perfect don't touch

        self.start_time = time.perf_counter()
        self.theta_i = self.subsystem.get_rotation()
        self.theta_f = self.shoulder_angle

        if not self.subsystem.is_at_position(self.distance, self.shoulder_angle):
            self.subsystem.disable_brake()

        self.subsystem.set_length(self.distance)

    def execute(self) -> None:
        SmartDashboard.putNumber(
            "ARM_CURRENT", math.degrees(self.subsystem.get_rotation())
        )

        SmartDashboard.putNumber("ARM_TARGET", math.degrees(self.real_desired))

        SmartDashboard.putNumber(
            "ARM_ERROR", math.degrees(self.real_desired - self.subsystem.get_rotation())
        )

        SmartDashboard.putNumber("DIST_CURRENT", self.subsystem.get_length())
        SmartDashboard.putNumber("DIST_TARGET", self.distance)
        SmartDashboard.putNumber(
            "DIST_ERROR", self.subsystem.get_length() - self.distance
        )

        # print("RUNNING")
        self.subsystem.update_pose()
        current_theta = self.subsystem.get_rotation()

        maximum_power = 0.1 * self.subsystem.arm_rotation_motor.motor.getBusVoltage()

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

        desired_voltage = (
            feed_forward + pid_voltage
        ) * self.subsystem.arm_rotation_motor.motor.getBusVoltage()
        SmartDashboard.putNumber("PID_Voltage", pid_voltage)
        self.subsystem.arm_rotation_motor.pid_controller.setOutputRange(
            -0.2, 0.2, slotID=1
        )
        self.subsystem.arm_rotation_motor.pid_controller.setReference(
            min(maximum_power, abs(desired_voltage))
            * (1 if desired_voltage > 0 else -1),
            rev.CANSparkMax.ControlType.kVoltage,
            1,
        )

        if self.subsystem.get_length() < 0.2 * constants.max_elevator_height_delta:
            self.subsystem.motor_extend.pid_controller.setOutputRange(-0.1, 1)
        else:
            self.subsystem.motor_extend.pid_controller.setOutputRange(-1, 1)

    def isFinished(self) -> bool:
        return (
            abs(self.subsystem.get_rotation() - self.real_desired) < math.radians(4)
            and abs(self.subsystem.get_length() - self.distance) < 0.05
        )

    def end(self, interrupted: bool) -> None:
        if not interrupted:
            print("FINISHED ARM SET")
            self.subsystem.enable_brake()
            self.subsystem.arm_rotation_motor.pid_controller.setReference(
                0, rev.CANSparkMax.ControlType.kVoltage, 0
            )
        else:
            print("FINISHED ARM SET BUT INTERRUPTED")
