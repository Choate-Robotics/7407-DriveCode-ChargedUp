import commands2
from commands2 import (
    InstantCommand,
    ParallelCommandGroup,
    SequentialCommandGroup,
)
from robotpy_toolkit_7407 import SubsystemCommand

import command
import config
import constants
from config import TargetData
from robot_systems import Robot, Sensors
from sensors import FieldOdometry
from subsystem import Arm, Drivetrain, Grabber, Intake


class Target(SubsystemCommand[Arm]):
    def __init__(
        self,
        arm: Arm,
        grabber: Grabber,
        intake: Intake,
        drivetrain: Drivetrain,
        field_odometry: FieldOdometry,
        target: TargetData,
    ):
        super().__init__(arm)
        super().addRequirements(grabber)
        super().addRequirements(intake)
        super().addRequirements(drivetrain)

        self.arm = arm
        self.grabber = grabber
        self.intake = intake
        self.drivetrain = drivetrain
        self.field_odometry = field_odometry

        self.target = target

        self.finished = False
        self.arm_on = True
        self.intake_on = True

        self.arm_sequence: SequentialCommandGroup | None = None
        self.intake_command: InstantCommand | None = None

    def finish(self) -> None:
        print("FINISHED TARGETING SEQUENCE")
        self.finished = True

    def initialize(self) -> None:
        Robot.drivetrain.max_vel = (
            self.target.max_velocity or constants.drivetrain_max_vel
        )
        Robot.drivetrain.max_target_accel = (
            self.target.max_acceleration or constants.drivetrain_max_target_accel
        )
        Robot.drivetrain.max_angular_vel = (
            self.target.max_angular_velocity or constants.drivetrain_max_angular_vel
        )

        commands2.CommandScheduler.getInstance().schedule(
            command.DriveSwerveCustom(self.drivetrain)
        )

        print("STARTING TARGETING COMMAND")
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
                self.arm_sequence = SequentialCommandGroup(
                    command.SetArm(
                        self.arm, self.target.arm_length, self.target.arm_angle
                    ),
                    command.SetGrabber(self.grabber, self.target.wrist_angle, False),
                    InstantCommand(self.grabber.open_claw()),
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
                    command.SetGrabber(self.grabber, self.target.wrist_angle, True),
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
