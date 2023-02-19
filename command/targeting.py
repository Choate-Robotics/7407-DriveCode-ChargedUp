from dataclasses import dataclass

import commands2
from commands2 import (
    InstantCommand,
    ParallelCommandGroup,
    SequentialCommandGroup,
)
from robotpy_toolkit_7407 import SubsystemCommand
from wpimath.geometry import Pose2d

import command
import utils
from autonomous.utils.custom_pathing import FollowPathCustom, RotateInPlace
from autonomous.utils.trajectory import CustomTrajectory
from sensors import FieldOdometry
from subsystem import Arm, Drivetrain, Grabber, Intake
from units.SI import meters, meters_per_second, meters_per_second_squared, radians


@dataclass
class TargetData:
    target_pose: Pose2d | None
    arm_angle: radians
    arm_length: meters
    wrist_angle: radians
    intake_enabled: bool = False
    claw_picking: bool = False
    claw_scoring: bool = False

    max_velocity: meters_per_second = None
    max_acceleration: meters_per_second_squared = None


class Target(SubsystemCommand[Arm]):
    def __init__(
        self,
        arm: Arm,
        grabber: Grabber,
        intake: Intake,
        field_odometry: FieldOdometry,
        target: TargetData,
        drivetrain: Drivetrain | None = None,
    ):
        super().__init__(arm)
        super().addRequirements(drivetrain)
        super().addRequirements(grabber)
        self.drivetrain = drivetrain
        self.arm = arm
        self.grabber = grabber
        self.intake = intake
        self.field_odometry = field_odometry

        self.target = target

        self.trajectory: CustomTrajectory | None = None

        self.finished = False
        self.drive_on = False

        self.arm_sequence: SequentialCommandGroup | None = None
        self.intake_command: InstantCommand | None = None

    def finish(self) -> None:
        self.finished = True

    def initialize(self) -> None:
        if self.target.intake_enabled:
            self.intake_command = InstantCommand(
                lambda: self.intake.set_intake_enabled(True)
            )
        else:
            self.intake_command = InstantCommand(
                lambda: self.intake.set_intake_enabled(False)
            )

        initial_pose = self.field_odometry.getPose()

        if self.target.target_pose and self.drivetrain:
            try:
                self.trajectory = CustomTrajectory(
                    initial_pose,
                    [],
                    self.target.target_pose,
                    self.target.max_velocity or self.drivetrain.max_vel,
                    self.target.max_acceleration or self.drivetrain.max_target_accel,
                    0,
                    0,
                )
            except RuntimeError:
                utils.logger.debug("TARGETING", "Failed to generate trajectory.")
                print("Failed to generate trajectory.")
                self.drive_on = False
        else:
            self.drive_on = False

        if self.target.claw_picking:
            self.arm_sequence = ParallelCommandGroup(
                command.SetArm(self.arm, self.target.arm_length, self.target.arm_angle),
                command.SetGrabber(self.grabber, self.target.wrist_angle, True),
            )
        elif self.target.claw_scoring:
            self.arm_sequence = SequentialCommandGroup(
                ParallelCommandGroup(
                    command.SetArm(
                        self.arm, self.target.arm_length, self.target.arm_angle
                    ),
                    command.SetGrabber(self.grabber, self.target.wrist_angle, False),
                ),
                InstantCommand(self.grabber.open_claw()),
            )
        else:
            self.arm_sequence = ParallelCommandGroup(
                command.SetArm(self.arm, self.target.arm_length, self.target.arm_angle),
                command.SetGrabber(self.grabber, self.target.wrist_angle, False),
            )

        if self.drive_on:
            commands2.CommandScheduler.getInstance().schedule(
                SequentialCommandGroup(
                    ParallelCommandGroup(
                        SequentialCommandGroup(
                            FollowPathCustom(self.drivetrain, self.trajectory),
                            RotateInPlace(
                                self.drivetrain, self.pose.rotation().radians()
                            ),
                        ),
                        self.arm_sequence,
                        self.intake_command,
                    ),
                    InstantCommand(lambda: self.finish()),
                ),
            )
        else:
            commands2.CommandScheduler.getInstance().schedule(
                SequentialCommandGroup(
                    ParallelCommandGroup(self.arm_sequence, self.intake_command),
                    InstantCommand(lambda: self.finish()),
                ),
            )

    def execute(self) -> None:
        ...

    def isFinished(self) -> bool:
        ...
        return self.finished

    def end(self, interrupted: bool = False) -> None:
        ...
