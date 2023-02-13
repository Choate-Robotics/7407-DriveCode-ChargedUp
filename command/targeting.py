import commands2
from commands2 import InstantCommand, ParallelCommandGroup, SequentialCommandGroup
from robotpy_toolkit_7407 import SubsystemCommand
from wpimath.geometry import Pose2d

from autonomous.utils.custom_pathing import FollowPathCustom, RotateInPlace
from autonomous.utils.trajectory import CustomTrajectory
from command import DriveSwerveCustom
from sensors import FieldOdometry
from subsystem import Arm, Drivetrain
from units.SI import meters, radians


class Target(SubsystemCommand[Drivetrain, Arm]):
    def __init__(
        self,
        drivetrain: Drivetrain,
        arm: Arm,
        field_odometry: FieldOdometry,
        pose: Pose2d,
        arm_angle: radians,
        arm_length: meters,
        wrist_angle: radians,
    ):
        """
        Command for autonomously targeting the entire robot.

        :param drivetrain: Drivetrain subsystem.
        :type drivetrain: Drivetrain
        :param arm: Arm subsystem.
        :type arm: Arm
        :param field_odometry: Field odometry.
        :type field_odometry: FieldOdometry
        :param pose: Target pose.
        :type pose: Pose2d
        :param arm_angle: Target arm angle.
        :type arm_angle: radians
        :param arm_length: Target arm length.
        :type arm_length: meters
        :param wrist_angle: Target wrist angle.
        :type wrist_angle: radians
        """
        super().__init__(drivetrain)
        super().addRequirements(arm)
        self.drivetrain = drivetrain
        self.arm = arm
        self.field_odometry = field_odometry

        self.pose = pose
        self.arm_angle = arm_angle
        self.arm_length = arm_length
        self.wrist_angle = wrist_angle

        self.trajectory: CustomTrajectory | None = None

        self.finished = False

    def finish(self) -> None:
        self.finished = True

    def initialize(self) -> None:
        current_pose = self.field_odometry.getPose()
        self.trajectory = CustomTrajectory(
            current_pose,
            [],
            self.pose,
            self.drivetrain.max_vel,
            self.drivetrain.max_accel,
            0,
            0,
        )
        commands2.CommandScheduler.getInstance().schedule(
            SequentialCommandGroup(
                ParallelCommandGroup(
                    SequentialCommandGroup(
                        FollowPathCustom(self.drivetrain, self.trajectory),
                        RotateInPlace(self.drivetrain, self.pose.rotation().radians()),
                    ),
                    InstantCommand(lambda: self.arm.set_rotation(self.arm_angle)),
                    InstantCommand(lambda: self.arm.set_length(self.arm_length)),
                    InstantCommand(lambda: self.arm.set_wrist(self.wrist_angle)),
                ),
                InstantCommand(lambda: self.arm.disengage_claw()),
                InstantCommand(lambda: self.finish()),
            ),
        )

    def execute(self) -> None:
        ...

    def isFinished(self) -> bool:
        ...
        return self.finished

    def end(self, interrupted: bool = False) -> None:
        commands2.CommandScheduler.schedule(
            DriveSwerveCustom(self.drivetrain),
        )
        ...
