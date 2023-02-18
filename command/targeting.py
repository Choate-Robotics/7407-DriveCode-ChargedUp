import commands2
from commands2 import InstantCommand, ParallelCommandGroup, SequentialCommandGroup
from robotpy_toolkit_7407 import SubsystemCommand
from wpimath.geometry import Pose2d

import command
import utils
from autonomous.utils.custom_pathing import FollowPathCustom, RotateInPlace
from autonomous.utils.trajectory import CustomTrajectory
from sensors import FieldOdometry
from subsystem import Arm, Drivetrain, Grabber
from units.SI import meters, radians


class Target(SubsystemCommand[Drivetrain]):
    def __init__(
        self,
        drivetrain: Drivetrain,
        arm: Arm,
        grabber: Grabber,
        field_odometry: FieldOdometry,
        pose: Pose2d,
        arm_angle: radians,
        arm_length: meters,
        wrist_angle: radians,
        wrist_enabled: bool,
    ):
        super().__init__(drivetrain)
        super().addRequirements(arm)
        super().addRequirements(grabber)
        self.drivetrain = drivetrain
        self.arm = arm
        self.grabber = grabber
        self.field_odometry = field_odometry

        self.pose = pose
        self.arm_angle = arm_angle
        self.arm_length = arm_length
        self.wrist_angle = wrist_angle
        self.wrist_enabled = wrist_enabled

        self.trajectory: CustomTrajectory | None = None

        self.finished = False
        self.drive_on = False

    def finish(self) -> None:
        self.finished = True

    def initialize(self) -> None:
        current_pose = self.field_odometry.getPose()

        try:
            self.trajectory = CustomTrajectory(
                current_pose,
                [],
                self.pose,
                self.drivetrain.max_vel,
                self.drivetrain.max_accel,
                0,
                0,
            )
        except RuntimeError:
            utils.logger.debug("TARGETING", "Failed to generate trajectory.")
            print("Failed to generate trajectory.")
            self.finished = True

        if not self.finished and self.drive_on:
            commands2.CommandScheduler.getInstance().schedule(
                SequentialCommandGroup(
                    ParallelCommandGroup(
                        SequentialCommandGroup(
                            FollowPathCustom(self.drivetrain, self.trajectory),
                            RotateInPlace(
                                self.drivetrain, self.pose.rotation().radians()
                            ),
                        ),
                        SequentialCommandGroup(
                            command.SetArm(self.arm, self.arm_length, self.arm_angle)
                        ),
                        command.SetGrabber(
                            self.grabber, self.wrist_angle, self.wrist_enabled
                        ),
                    ),
                    InstantCommand(lambda: self.finish()),
                ),
            )
        else:
            commands2.CommandScheduler.getInstance().schedule(
                SequentialCommandGroup(
                    ParallelCommandGroup(
                        command.SetArm(self.arm, self.arm_length, self.arm_angle),
                        command.SetGrabber(
                            self.grabber, self.wrist_angle, self.wrist_enabled
                        ),
                    ),
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
