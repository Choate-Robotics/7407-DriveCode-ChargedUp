import math

import commands2
from commands2 import (
    InstantCommand,
    ParallelCommandGroup,
    SequentialCommandGroup,
)
from robotpy_toolkit_7407 import SubsystemCommand

import command
import config
from config import TargetData
from robot_systems import Sensors
from sensors import FieldOdometry
from subsystem import Arm, Grabber, Intake


class TargetAuto:
    def __init__(
        self,
        arm: Arm,
        grabber: Grabber,
        intake: Intake,
        field_odometry: FieldOdometry,
        target: TargetData,
        grabber_back_first=False,
    ):
        self.arm = arm
        self.grabber = grabber
        self.intake = intake
        self.field_odometry = field_odometry

        self.target = target

        self.arm_on = True
        self.intake_on = True

        self.grabber_back_first = grabber_back_first

        self.arm_sequence: SequentialCommandGroup | None = None
        self.intake_command: InstantCommand | None = None

        self.command = None

    def generate(self) -> ParallelCommandGroup:
        if self.target.intake_enabled and self.intake_on:
            self.intake_command = command.IntakeEnable(
                self.intake, intake_reversed=self.target.intake_reversed
            )
        elif self.intake_on:
            self.intake_command = command.IntakeDisable(self.intake)
        else:
            self.intake_command = command.IntakeDisable(self.intake)

        if self.target.claw_wait:
            if self.target.claw_picking and self.arm_on:
                self.arm_sequence = SequentialCommandGroup(
                    command.SetArm(
                        self.arm, self.target.arm_length, self.target.arm_angle
                    ),
                    command.SetGrabber(self.grabber, self.target.wrist_angle, True),
                )
            elif self.target.claw_scoring and self.arm_on:
                self.arm_sequence = ParallelCommandGroup(
                    command.SetArm(
                        self.arm, self.target.arm_length, self.target.arm_angle
                    ),
                    SequentialCommandGroup(
                        command.SetGrabber(
                            self.grabber,
                            math.radians(70)
                            * (-1 if self.target.wrist_angle > 0 else 1),
                            False,
                        ),
                        command.SetGrabber(
                            self.grabber,
                            math.radians(25)
                            * (-1 if self.target.wrist_angle > 0 else 1),
                            False,
                        ),
                    ),
                )
            elif self.arm_on:
                self.arm_sequence = SequentialCommandGroup(
                    command.SetArm(
                        self.arm, self.target.arm_length, self.target.arm_angle
                    ),
                    command.SetGrabber(self.grabber, self.target.wrist_angle, False),
                )
            else:
                self.arm_sequence = InstantCommand(lambda: None)
        else:
            if self.target.claw_picking and self.arm_on:
                self.arm_sequence = ParallelCommandGroup(
                    command.SetArm(
                        self.arm, self.target.arm_length, self.target.arm_angle
                    ),
                    command.SetGrabber(
                        self.grabber,
                        self.target.wrist_angle,
                        True,
                        auto_claw=True,
                        auto_cube=self.target.cube_picking,
                    ),
                )
            elif self.target.claw_scoring and self.arm_on:
                self.arm_sequence = SequentialCommandGroup(
                    ParallelCommandGroup(
                        command.SetArm(
                            self.arm, self.target.arm_length, self.target.arm_angle
                        ),
                        command.SetGrabber(
                            self.grabber, self.target.wrist_angle, False
                        ),
                    ),
                    InstantCommand(self.grabber.open_claw()),
                )
            elif self.arm_on:
                self.arm_sequence = ParallelCommandGroup(
                    command.SetArm(
                        self.arm, self.target.arm_length, self.target.arm_angle
                    ),
                    command.SetGrabber(self.grabber, self.target.wrist_angle, False),
                )
            else:
                self.arm_sequence = InstantCommand(lambda: None)

        return ParallelCommandGroup(self.arm_sequence, self.intake_command)


class Target(SubsystemCommand[Arm]):
    def __init__(
        self,
        arm: Arm,
        grabber: Grabber,
        intake: Intake,
        field_odometry: FieldOdometry,
        target: TargetData,
        grabber_back_first=False,
    ):
        super().__init__(arm)
        super().addRequirements(grabber)
        super().addRequirements(intake)

        self.arm = arm
        self.grabber = grabber
        self.intake = intake
        self.field_odometry = field_odometry

        self.target = target

        self.finished = False
        self.arm_on = True
        self.intake_on = True

        self.grabber_back_first = grabber_back_first

        self.arm_sequence: SequentialCommandGroup | None = None
        self.intake_command: InstantCommand | None = None

    def finish(self) -> None:
        self.finished = True

    def initialize(self) -> None:
        if self.target.arm_scoring:
            gyro_angle = (math.degrees(Sensors.gyro.get_robot_heading()) % 360) - 180

            if not (-90 < gyro_angle < 90):
                self.target.arm_angle = abs(self.target.arm_angle) * (
                    1 if config.red_team else -1
                )
                self.target.wrist_angle = abs(self.target.wrist_angle) * (
                    1 if config.red_team else -1
                )
            else:
                self.target.arm_angle = (
                    -1 * abs(self.target.arm_angle) * (1 if config.red_team else -1)
                )
                self.target.wrist_angle = (
                    -1 * abs(self.target.wrist_angle) * (1 if config.red_team else -1)
                )

        if self.target.intake_enabled and self.intake_on:
            self.intake_command = command.IntakeEnable(
                self.intake, intake_reversed=self.target.intake_reversed
            )
        elif self.intake_on:
            self.intake_command = command.IntakeDisable(self.intake)
        else:
            self.intake_command = InstantCommand(lambda: None)

        if self.target.claw_wait:
            if self.target.claw_picking and self.arm_on:
                self.arm_sequence = SequentialCommandGroup(
                    command.SetArm(
                        self.arm, self.target.arm_length, self.target.arm_angle
                    ),
                    command.SetGrabber(self.grabber, self.target.wrist_angle, True),
                )
            elif self.target.claw_scoring and self.arm_on:
                self.arm_sequence = ParallelCommandGroup(
                    command.SetArm(
                        self.arm, self.target.arm_length, self.target.arm_angle
                    ),
                    SequentialCommandGroup(
                        command.SetGrabber(
                            self.grabber,
                            math.radians(90)
                            * (-1 if self.target.wrist_angle > 0 else 1),
                            False,
                        ),
                        command.SetGrabber(
                            self.grabber,
                            math.radians(25)
                            * (-1 if self.target.wrist_angle > 0 else 1),
                            False,
                        ),
                    ),
                )
            elif self.arm_on:
                self.arm_sequence = SequentialCommandGroup(
                    command.SetArm(
                        self.arm, self.target.arm_length, self.target.arm_angle
                    ),
                    command.SetGrabber(self.grabber, self.target.wrist_angle, False),
                )
            else:
                self.arm_sequence = InstantCommand(lambda: None)
        else:
            if self.target.claw_picking and self.arm_on:
                self.arm_sequence = ParallelCommandGroup(
                    command.SetArm(
                        self.arm, self.target.arm_length, self.target.arm_angle
                    ),
                    command.SetGrabber(
                        self.grabber,
                        self.target.wrist_angle,
                        True,
                        auto_claw=True,
                        auto_cube=self.target.cube_picking,
                    ),
                )
            elif self.target.claw_scoring and self.arm_on:
                self.arm_sequence = SequentialCommandGroup(
                    ParallelCommandGroup(
                        command.SetArm(
                            self.arm, self.target.arm_length, self.target.arm_angle
                        ),
                        command.SetGrabber(
                            self.grabber, self.target.wrist_angle, False
                        ),
                    ),
                    InstantCommand(self.grabber.open_claw()),
                )
            elif self.arm_on:
                self.arm_sequence = ParallelCommandGroup(
                    command.SetArm(
                        self.arm, self.target.arm_length, self.target.arm_angle
                    ),
                    command.SetGrabber(self.grabber, self.target.wrist_angle, False),
                )
            else:
                self.arm_sequence = InstantCommand(lambda: None)

        commands2.CommandScheduler.getInstance().schedule(
            SequentialCommandGroup(
                ParallelCommandGroup(self.arm_sequence, self.intake_command),
                InstantCommand(lambda: self.finish()),
            )
        )

    def execute(self) -> None:
        ...

    def isFinished(self) -> bool:
        ...
        return self.finished

    def end(self, interrupted: bool = False) -> None:
        ...
