import math

import rev
from robotpy_toolkit_7407 import SubsystemCommand
from wpimath._controls._controls.controller import ArmFeedforward, PIDController

import constants
import utils
from robot_systems import Robot
from subsystem import Elevator
from units.SI import meters


class ZeroElevator(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: Elevator):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        ...

    def execute(self):
        self.subsystem.motor_extend.set_raw_output(-0.05)
        ...

    def isFinished(self):
        return self.subsystem.elevator_bottom_sensor.get()

    def end(self, interrupted=False):
        self.subsystem.motor_extend.set_sensor_position(0)
        print("Elevator successfully zeroed.")
        utils.logger.debug("Elevator", "Elevator Successfully Zeroed.")


class SetElevator(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: Elevator, distance: meters):
        super().__init__(subsystem)
        self.distance = distance
        self.elevator_controller = None
        self.elevator_ff = None

    def initialize(self) -> None:
        self.elevator_controller = PIDController(0.9, 0, 0.0)

        self.elevator_ff = ArmFeedforward(kG=0.145, kS=0, kV=0, kA=0)

    def execute(self) -> None:
        current_length_rotations = self.subsystem.motor_extend.get_sensor_position()

        elevator_maximum_power = 8
        current_theta = Robot.arm.get_rotation()

        elevator_feed_forward = math.sin(math.pi / 2 - current_theta) * 1.1

        meters_ceiling = min(self.distance, constants.max_elevator_height_delta)
        calculated_motor_rotations = meters_ceiling * (
            1 / constants.elevator_length_per_rotation
        )
        elevator_pid_output = self.elevator_controller.calculate(
            current_length_rotations, calculated_motor_rotations
        )

        if (
            abs(self.subsystem.get_rotation() - self.real_desired) > math.radians(25)
            and elevator_pid_output > 0.0
        ):
            elevator_pid_output = 0

        if (
            abs(current_length_rotations - calculated_motor_rotations)
            * constants.elevator_length_per_rotation
        ) < 0.03:
            elevator_pid_output = 0

        elevator_desired_voltage = elevator_feed_forward + elevator_pid_output

        self.subsystem.motor_extend.pid_controller.setReference(
            min(elevator_maximum_power, abs(elevator_desired_voltage))
            * (1 if elevator_desired_voltage > 0 else -1),
            rev.CANSparkMax.ControlType.kVoltage,
            pidSlot=1,
        )

        if self.subsystem.get_length() < 0.2 * constants.max_elevator_height_delta:
            self.subsystem.motor_extend.pid_controller.setOutputRange(
                min=-0.2, max=1, slotID=1
            )
        else:
            self.subsystem.motor_extend.pid_controller.setOutputRange(
                min=-1, max=1, slotID=1
            )
