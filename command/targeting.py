import commands2
from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
)
from robotpy_toolkit_7407 import SubsystemCommand

import command
import config
from config import TargetData
from robot_systems import Sensors
from sensors import FieldOdometry
from subsystem import Arm, Grabber, Intake


class Target(SubsystemCommand[Arm]):
    def __init__(
        self,
        arm: Arm,
        grabber: Grabber,
        intake: Intake,
        field_odometry: FieldOdometry,
        target: TargetData,
    ):
        super().__init__(arm)
        super().addRequirements(grabber)
        super().addRequirements(intake)

        self.arm = arm
        self.grabber = grabber
        self.intake = intake
        self.field_odometry = field_odometry

        self.target = target

        self.arm_on = True
        self.intake_on = True
        self.grabber_on = True

        self.arm_sequence: SequentialCommandGroup | None = None
        self.intake_command: InstantCommand | None = None

    def initialize(self) -> None:
        print("STARTING TARGETING COMMAND")
        if self.arm_on:
            if self.target.arm_scoring:
                gyro_angle = Sensors.odometry.getPose().rotation().degrees()

                if -90 < gyro_angle < 90:
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
                        -1
                        * abs(self.target.wrist_angle)
                        * (1 if config.red_team else -1)
                    )

                commands2.CommandScheduler.getInstance().schedule(
                    command.SetArm(
                        self.arm, self.target.arm_length, self.target.arm_angle
                    ),
                )

        if self.grabber_on:
            commands2.CommandScheduler.getInstance().schedule(
                command.SetGrabberIntake(
                    subsystem=self.grabber,
                    intake=self.intake,
                    wrist_angle=self.target.wrist_angle,
                    claw_active=self.target.claw_picking,
                    intake_active=self.target.intake_enabled
                    if self.intake_on
                    else False,
                    intake_reversed=self.target.intake_reversed,
                    auto_claw=self.target.claw_picking,
                )
            )

    def execute(self) -> None:
        ...

    def isFinished(self) -> bool:
        ...
        return False

    def end(self, interrupted: bool = False) -> None:
        ...
