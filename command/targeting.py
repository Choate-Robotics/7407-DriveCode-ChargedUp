import commands2
import wpilib
from commands2 import (
    InstantCommand,
    ParallelCommandGroup,
    SequentialCommandGroup,
)
from robotpy_toolkit_7407 import SubsystemCommand

import command
import config
from autonomous.utils.custom_pathing import FollowPathCustom, RotateInPlaceTeleOp
from autonomous.utils.trajectory import CustomTrajectory
from config import TargetData
from robot_systems import Sensors
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

        self.drivetrain = drivetrain
        self.arm = arm
        self.grabber = grabber
        self.intake = intake
        self.field_odometry = field_odometry

        self.target = target

        self.trajectory_initial: CustomTrajectory | None = None
        self.trajectory_final: CustomTrajectory | None = None

        self.finished = False
        self.arm_on = True
        self.intake_on = True
        self.drive_on = True

        self.arm_sequence: SequentialCommandGroup | None = None
        self.intake_command: InstantCommand | None = None

    def finish(self) -> None:
        print("FINISHED TARGETING SEQUENCE")
        self.finished = True

    def initialize(self) -> None:
        self.drive_on = False

        print("STARTING TARGETING COMMAND")
        if self.target.arm_scoring:
            if self.target.target_pose_initial and self.drive_on:
                gyro_angle = self.target.target_pose_initial.rotation().degrees()
            else:
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

        if self.target.target_pose_initial is not None and self.drive_on:
            print("!!!TESTING TRAJECTORY")
            initial_pose = self.field_odometry.getPose()
            try:
                self.trajectory_initial = CustomTrajectory(
                    initial_pose,
                    [],
                    self.target.target_pose_initial,
                    self.target.max_velocity or self.drivetrain.max_vel,
                    self.target.max_acceleration or self.drivetrain.max_target_accel,
                    0,
                    0,
                )
                if self.trajectory_final is not None:
                    self.trajectory_final = CustomTrajectory(
                        initial_pose,
                        [],
                        self.target.target_pose_final,
                        self.target.max_velocity or self.drivetrain.max_vel,
                        self.target.max_acceleration
                        or self.drivetrain.max_target_accel,
                        0,
                        0,
                    )
                print("SUCCESSFULLY MADE TRAJECTORY!!!!!!!!!!!!")
            except Exception as e:
                print("Failed to generate trajectory!!!!!!!!!!!!!!")
                print(e)
                self.trajectory_initial = None
                self.drive_on = False
        else:
            self.drive_on = False

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

        if self.drive_on:
            commands2.CommandScheduler.getInstance().schedule(
                SequentialCommandGroup(
                    FollowPathCustom(self.drivetrain, self.trajectory_initial),
                    RotateInPlaceTeleOp(
                        self.drivetrain,
                        self.target.target_pose_initial.rotation().radians(),
                    ),
                    commands2.ConditionalCommand(
                        FollowPathCustom(
                            self.drivetrain, self.trajectory_final
                        ).andThen(
                            InstantCommand(
                                lambda: commands2.CommandScheduler.getInstance().schedule(
                                    self.drivetrain
                                )
                            )
                        ),
                        InstantCommand(lambda: None),
                        lambda: self.trajectory_final is not None,
                    ),
                ),
            )
            commands2.CommandScheduler.getInstance().schedule(
                SequentialCommandGroup(
                    ParallelCommandGroup(
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
                )
            )

        wpilib.SmartDashboard.putBoolean("DRIVE ON", self.drive_on)

    def execute(self) -> None:
        ...

    def isFinished(self) -> bool:
        ...
        return False

    def end(self, interrupted: bool = False) -> None:
        commands2.CommandScheduler.getInstance().schedule(
            command.DriveSwerveCustom(self.drivetrain)
        )
        ...
