import math
import time

import rev
from robotpy_toolkit_7407.command import SubsystemCommand
from wpilib import SmartDashboard
from wpimath._controls._controls.controller import ProfiledPIDControllerRadians
from wpimath._controls._controls.trajectory import TrapezoidProfileRadians
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

    def execute(self):
        self.subsystem.motor_extend.set_raw_output(-0.05)
        print("ZEROING")
        ...

    def isFinished(self):
        return self.subsystem.elevator_bottom_sensor.get()

    def end(self, interrupted=False):
        self.subsystem.motor_extend.set_sensor_position(0)
        print("Elevator successfully zeroed.")
        utils.logger.debug("Elevator", "Elevator Successfully Zeroed.")


class ZeroShoulder(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        print("ZEROING SHOULDER")
        self.subsystem.disable_brake()
        self.subsystem.zero_elevator_rotation()

    def execute(self):
        ...

    def isFinished(self):
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
        self.threshold = math.radians(0.5)

        self.elevator_controller = None
        self.arm_controller_profiled = None
        self.arm_ff_constant = None
        self.elevator_ff = None

        self.desired_time = 3

    def initialize(self):

        elevator_p = SmartDashboard.getNumber("ELEVATOR_P_VALUE", 0)
        self.arm_controller = PIDController(6, 0, 0.1)
        self.elevator_controller = PIDController(0.9, 0, 0.0)
        self.arm_controller_profiled = ProfiledPIDControllerRadians(
            3,
            0,
            0,
            TrapezoidProfileRadians.Constraints(math.radians(45), math.radians(45)),
        )

        self.arm_controller_profiled.setGoal(self.shoulder_angle)
        self.arm_ff_constant = 0.25
        # self.arm_ff = ArmFeedforward(kG=0.045, kS=0, kV=0, kA=0)  # perfect don't touch
        self.elevator_ff = ArmFeedforward(
            kG=0.145, kS=0, kV=0, kA=0
        )  # perfect don't touch

        self.start_time = time.perf_counter()
        self.theta_i = self.subsystem.get_rotation()
        self.theta_f = self.shoulder_angle

        if not self.subsystem.is_at_position(self.distance, self.shoulder_angle):
            self.subsystem.disable_brake()

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
        current_length = self.subsystem.motor_extend.get_sensor_position()

        # ------------ ARM ------------

        arm_maximum_power = 8

        arm_feed_forward = -math.sin(current_theta) * self.arm_ff_constant

        arm_pid_output = -(
            self.arm_controller.calculate(
                math.pi / 2 - current_theta,
                self.theta_f,
            )
        )

        if abs(self.subsystem.get_rotation() - self.real_desired) < self.threshold:
            pid_voltage = 0

        arm_desired_voltage = arm_feed_forward + arm_pid_output
        SmartDashboard.putNumber("ARM_Voltage", arm_desired_voltage)
        SmartDashboard.putNumber("ARM_PID_Voltage", arm_pid_output)
        self.subsystem.arm_rotation_motor.pid_controller.setReference(
            min(arm_maximum_power, abs(arm_desired_voltage))
            * (1 if arm_desired_voltage > 0 else -1),
            rev.CANSparkMax.ControlType.kVoltage,
            1,
        )
        # ^^------------ ARM ------------^^

        # ------------ ELEVATOR ------------

        elevator_maximum_power = 8

        elevator_feed_forward = math.sin(math.pi / 2 - current_theta) * 1.1

        # --PID STUFF--
        meters_ceiling = min(self.distance, constants.max_elevator_height_delta)
        calculated_motor_rotations = meters_ceiling * (
            1 / constants.elevator_length_per_rotation
        )
        elevator_pid_output = self.elevator_controller.calculate(
            current_length, calculated_motor_rotations
        )
        # --PID STUFF--

        SmartDashboard.putNumber("Current_motor_length", current_length)
        SmartDashboard.putNumber("calculated_motor_length", calculated_motor_rotations)
        SmartDashboard.putNumber("PID_Voltage", elevator_pid_output)

        if abs(current_length - calculated_motor_rotations) < 0.1:
            elevator_pid_output = 0

        SmartDashboard.putNumber("PID_Voltage", elevator_pid_output)

        elevator_desired_voltage = elevator_feed_forward + elevator_pid_output

        SmartDashboard.putNumber("ELEVATOR_Voltage", elevator_desired_voltage)
        test_voltage = SmartDashboard.getNumber("Test_ELEVATOR_Voltage", 0)

        # SmartDashboard.putNumber("")

        self.subsystem.motor_extend.pid_controller.setReference(
            min(elevator_maximum_power, abs(elevator_desired_voltage))
            * (1 if elevator_desired_voltage > 0 else -1),
            rev.CANSparkMax.ControlType.kVoltage,
            pidSlot=1,
        )

        # ------------ ELEVATOR ------------^^

        if self.subsystem.get_length() < 0.2 * constants.max_elevator_height_delta:
            self.subsystem.motor_extend.pid_controller.setOutputRange(
                min=-0.2, max=1, slotID=1
            )
        else:
            self.subsystem.motor_extend.pid_controller.setOutputRange(
                min=-1, max=1, slotID=1
            )

    def isFinished(self) -> bool:
        # return (
        #         abs(self.subsystem.get_rotation() - self.real_desired) < self.threshold
        #         and abs(self.subsystem.get_length() - self.distance) < 0.05
        # )
        return False

    def end(self, interrupted: bool) -> None:
        if not interrupted:
            print("FINISHED ARM SET")
            self.subsystem.enable_brake()
            self.subsystem.arm_rotation_motor.pid_controller.setReference(
                0, rev.CANSparkMax.ControlType.kVoltage, pidSlot=0
            )
        else:
            print("FINISHED ARM SET BUT INTERRUPTED")
