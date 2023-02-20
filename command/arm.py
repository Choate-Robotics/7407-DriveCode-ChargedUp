import math
import time

import rev
from robotpy_toolkit_7407.command import SubsystemCommand
from wpilib import SmartDashboard
from wpimath.controller import ArmFeedforward, PIDController,ProfiledPIDControllerRadians, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfileRadians, TrapezoidProfile

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


# class setShoulderRotation(SubsystemCommand[Arm]):
#     def __init__(self, subsystem: Arm, shoulder_angle):
#         super().__init__(subsystem)
#         self.shoulder_angle = (math.pi / 2) - shoulder_angle
#         self.arm_controller: PIDController | None = None
#         self.arm_ff: ArmFeedforward | None = None
#         self.start_time = None
#         self.theta_i = 0
#         self.theta_f = 0
#         self.theta_diff = 0
#         self.threshold = math.radians(1)
#         self.desired_time = 3

#     def initialize(self) -> None:
#         self.arm_controller = PIDController(1, 0, 0.03)

#         self.arm_ff = ArmFeedforward(kG=0.045, kS=0, kV=0, kA=0)  # perfect dont touch

#         self.start_time = time.perf_counter()
#         self.theta_i = self.subsystem.get_rotation()
#         self.theta_f = self.shoulder_angle

#         if not self.subsystem.is_at_shoulder_rotation(self.shoulder_angle):
#             self.subsystem.disable_brake()

#     def execute(self) -> None:
#         current_theta = self.subsystem.get_rotation()

#         maximum_power = 0.5 * self.subsystem.arm_rotation_motor.motor.getBusVoltage()
#         # print(math.pi/2 - self.theta_f)
#         # print("Actual Theta:", current_theta)
#         # print("Theta:", math.pi/2 - current_theta)

#         feed_forward = -self.arm_ff.calculate(
#             angle=(math.pi / 2 - current_theta), velocity=0.1, acceleration=0
#         )
#         # print("Voltage", desired_voltage)
#         pid_voltage = -(
#             self.arm_controller.calculate(
#                 math.pi / 2 - current_theta,  # sets correct origin
#                 self.theta_f,
#             )
#         )
#         # if abs(pid_voltage) < .0005:
#         #     pid_voltage = 0
#         # print(self.subsystem.is_at_shoulder_rotation(math.pi/2 - self.shoulder_angle))
#         if self.subsystem.is_at_shoulder_rotation(math.pi / 2 - self.shoulder_angle):
#             pid_voltage = 0
#         desired_voltage = (
#                                   feed_forward + pid_voltage
#                           ) * self.subsystem.arm_rotation_motor.motor.getBusVoltage()
#         # print(self.shoulder_angle)
#         print("SHOULDER: ", self.shoulder_angle)
#         print("CURRENT: ", current_theta)

#         SmartDashboard.putNumber("PID_Voltage", pid_voltage)
#         self.subsystem.arm_rotation_motor.pid_controller.setReference(
#             min(maximum_power, abs(desired_voltage))
#             * (1 if desired_voltage > 0 else -1),
#             rev.CANSparkMax.ControlType.kVoltage,
#             pidSlot=1,
#         )

#     def isFinished(self) -> bool:
#         return self.subsystem.is_at_shoulder_rotation(self.shoulder_angle)

#     def end(self, interrupted: bool) -> None:
#         pass


class setElevator(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm, length):
        super().__init__(subsystem)
        self.length = length

    def initialize(self) -> None:
        self.subsystem.set_length(self.length)

    def execute(self) -> None:
        if self.subsystem.get_length() < 0.2 * constants.max_elevator_height_delta:
            self.subsystem.motor_extend.pid_controller.setOutputRange(-0.1, 1)
        else:
            self.subsystem.motor_extend.pid_controller.setOutputRange(-1, 1)

    def isFinished(self) -> bool:
        return self.subsystem.is_at_length(self.length)

    def end(self, interrupted: bool) -> None:
        pass


class SetGrabber(SubsystemCommand[Grabber]):
    def __init__(self, subsystem: Grabber, wrist_angle: radians, claw_active: bool):
        super().__init__(subsystem)
        self.subsystem = subsystem
        self.wrist_angle = wrist_angle
        self.claw_active = claw_active

    def initialize(self) -> None:
        self.subsystem.set_angle(self.wrist_angle)
        if self.claw_active:
            self.subsystem.engage_claw()
        else:
            self.subsystem.disengage_claw()

    def execute(self) -> None:
        ...

    def isFinished(self) -> bool:
        return self.subsystem.is_at_angle(self.wrist_angle)

    def end(self, interrupted: bool) -> None:
        ...


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

        self.desired_time = 3

    def initialize(self):

        elevator_p = SmartDashboard.getNumber("ELEVATOR_P_VALUE",0)
        self.arm_controller = PIDController(10, 0, 0.1)
        self.elevator_controller = PIDController(0.9, 0, 0.0)
        self.arm_controller_profiled = ProfiledPIDControllerRadians(3,0,0, TrapezoidProfileRadians.Constraints(math.radians(45),math.radians(45)))
        
        self.arm_controller_profiled.setGoal(self.shoulder_angle)
        self.arm_ff_constant = 0.25
        #self.arm_ff = ArmFeedforward(kG=0.045, kS=0, kV=0, kA=0)  # perfect don't touch
        self.elevator_ff = ArmFeedforward(kG=0.145, kS=0, kV=0, kA=0)  # perfect don't touch

        self.start_time = time.perf_counter()
        self.theta_i = self.subsystem.get_rotation()
        self.theta_f = self.shoulder_angle


        if not self.subsystem.is_at_position(self.distance, self.shoulder_angle):
            self.subsystem.disable_brake()

        #self.subsystem.set_length(self.distance)

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
        self.subsystem.update_pose()
        current_theta = self.subsystem.get_rotation()
        current_length = self.subsystem.motor_extend.get_sensor_position()

        # ------------ ARM ------------

        arm_maximum_power = 6

        arm_feed_forward = -math.sin(current_theta) * self.arm_ff_constant

        # arm_pid_output = -(
        #     self.arm_controller.calculate(
        #         math.pi / 2 - current_theta,
        #         self.theta_f,
        #     )
        # )

        arm_pid_output = -( #PROFILED
            self.arm_controller_profiled.calculate(math.pi / 2 - current_theta)
        )


        if abs(self.subsystem.get_rotation() - self.real_desired) < self.threshold:
             arm_pid_output = 0

        
        arm_desired_voltage = arm_feed_forward #+ arm_pid_output
                          #) * self.subsystem.arm_rotation_motor.motor.getBusVoltage()
        SmartDashboard.putNumber("ARM_Voltage", arm_desired_voltage)
        SmartDashboard.putNumber("ARM_PID_Voltage", arm_pid_output)
        # self.subsystem.arm_rotation_motor.pid_controller.setReference(
        #     min(arm_maximum_power, abs(arm_desired_voltage))
        #     * (1 if arm_desired_voltage > 0 else -1),
        #     rev.CANSparkMax.ControlType.kVoltage,
        #     1,
        # )
        # ^^------------ ARM ------------^^



        # ------------ ELEVATOR ------------

        elevator_maximum_power = 8

        elevator_feed_forward = math.sin(math.pi/2 - current_theta) * 1.1


        #--PID STUFF--
        meters_ceiling = min(self.distance, constants.max_elevator_height_delta)
        calculated_motor_rotations = meters_ceiling * (1 / constants.elevator_length_per_rotation)
        elevator_pid_output = self.elevator_controller.calculate(current_length,calculated_motor_rotations)
        #--PID STUFF--

        SmartDashboard.putNumber("Current_motor_length", current_length)
        SmartDashboard.putNumber("calculated_motor_length", calculated_motor_rotations)
        SmartDashboard.putNumber("PID_Voltage", elevator_pid_output)

        if abs(current_length - calculated_motor_rotations) < 0.1:
            elevator_pid_output = 0

        SmartDashboard.putNumber("PID_Voltage", elevator_pid_output)

        elevator_desired_voltage = elevator_feed_forward #+ elevator_pid_output

        SmartDashboard.putNumber("ELEVATOR_Voltage", elevator_desired_voltage)
        test_voltage = SmartDashboard.getNumber("Test_ELEVATOR_Voltage",0)



        #SmartDashboard.putNumber("")

        # self.subsystem.motor_extend.pid_controller.setReference(
        #     min(elevator_maximum_power, abs(elevator_desired_voltage))
        #     * (1 if elevator_desired_voltage > 0 else -1),
        #     rev.CANSparkMax.ControlType.kVoltage,
        #     pidSlot=1,
        # )

        # ------------ ELEVATOR ------------^^


        if self.subsystem.get_length() < 0.2 * constants.max_elevator_height_delta:
            self.subsystem.motor_extend.pid_controller.setOutputRange(min=-0.2, max=1, slotID=1)
        else:
            self.subsystem.motor_extend.pid_controller.setOutputRange(min=-1, max=1, slotID=1)

    def isFinished(self) -> bool:
        return False
        # return (
        #         abs(self.subsystem.get_rotation() - self.real_desired) < math.radians(4)
        #         and abs(self.subsystem.get_length() - self.distance) < 0.05
        # )

    def end(self, interrupted: bool) -> None:
        if not interrupted:
            print("FINISHED ARM SET")
            self.subsystem.enable_brake()
            self.subsystem.arm_rotation_motor.pid_controller.setReference(
                0, rev.CANSparkMax.ControlType.kVoltage, 0
            )
            self.subsystem.motor_extend.pid_controller.setReference(
                0, rev.CANSparkMax.ControlType.kVoltage, pidSlot=1
            )
        else:
            print("FINISHED ARM SET BUT INTERRUPTED")
