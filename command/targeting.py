import math

import commands2
import wpilib
from commands2 import (
    InstantCommand,
    ParallelCommandGroup,
    SequentialCommandGroup,
)
from robotpy_toolkit_7407.command import BasicCommand, SubsystemCommand
from wpimath.geometry import Pose2d

import command
import config
from command.autonomous import CustomRouting, RotateInPlace
from config import TargetData
from robot_systems import Robot, Sensors
from sensors import FieldOdometry
from subsystem import Arm, Grabber, Intake


class TargetDrivetrain(BasicCommand):
    def __init__(
        self,
        field_odometry: FieldOdometry,
    ):
        super().__init__()
        self.field_odometry = field_odometry
        self.target_list = None

    def initialize(self):
        self.target_list = (
            config.blue_scoring_positions
            if config.blue_team
            else config.red_scoring_positions
        )

        gyro_angle = Sensors.gyro.get_robot_heading() % (math.pi * 2)
        gyro_angle = math.degrees(
            math.atan2(math.sin(gyro_angle), math.cos(gyro_angle))
        )

        if -90 < gyro_angle < 90:
            target_angle = 0
        else:
            target_angle = math.pi

        if config.current_scoring_position != "None":
            target = self.target_list[config.current_scoring_position]
            target = Pose2d(target.x, target.y, target_angle)
        else:
            current_pose = self.field_odometry.getPose()
            target = min(
                self.target_list,
                key=lambda x: x.translation().distance(current_pose.translation()),
            )
            target = Pose2d(target.x, target.y, target_angle)

        wpilib.SmartDashboard.putString("TARGET POSE", str(target))

        # Generate a command to follow the trajectory
        commands2.CommandScheduler.getInstance().schedule(
            SequentialCommandGroup(
                RotateInPlace(
                    Robot.drivetrain,
                    target_angle,
                    threshold=math.radians(4),
                    max_angular_vel=config.drivetrain_routing_angular_velocity,
                ),
                CustomRouting(
                    subsystem=Robot.drivetrain,
                    target=target,
                ),
                RotateInPlace(
                    Robot.drivetrain,
                    target_angle,
                    threshold=math.radians(2),
                    max_angular_vel=config.drivetrain_routing_angular_velocity,
                ),
                InstantCommand(
                    lambda: commands2.CommandScheduler.getInstance().schedule(
                        command.DriveSwerveSlowed(
                            Robot.drivetrain,
                        )
                    )
                ),
                # InstantCommand(
                #     lambda: commands2.CommandScheduler.getInstance().schedule(
                #         command.DriveSwerveCustom(
                #             Robot.drivetrain,
                #         )
                #     )
                # ),
            )
        )

    def isFinished(self) -> bool:
        return True


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
        if self.target.intake_enabled and self.intake_on and not self.target.intake_off:
            self.intake_command = command.IntakeEnable(
                self.intake, intake_reversed=self.target.intake_reversed
            )
        elif self.intake_on and self.target.intake_off:
            self.intake_command = SequentialCommandGroup(
                InstantCommand(lambda: self.intake.intake_piston.extend()),
                InstantCommand(lambda: self.intake.intake_motor.set_raw_output(0)),
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
                            finish=False,
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
        self.intake_on = not self.target.no_intake

        arm_length = self.target.arm_length
        arm_angle = self.target.arm_angle
        wrist_angle = self.target.wrist_angle

        if self.target.arm_scoring:
            gyro_angle = Sensors.gyro.get_robot_heading() % (math.pi * 2)
            gyro_angle = math.degrees(
                math.atan2(math.sin(gyro_angle), math.cos(gyro_angle))
            )

            if -90 < gyro_angle < 90:
                arm_angle = abs(self.target.arm_angle) * -1
                wrist_angle = abs(self.target.wrist_angle) * -1
            else:
                arm_angle = abs(self.target.arm_angle)
                wrist_angle = abs(self.target.wrist_angle)

            if self.target.arm_reversed:
                arm_angle = -1 * arm_angle
                wrist_angle = -1 * wrist_angle

            if self.target.low_scoring:
                if -90 < gyro_angle < 90:
                    arm_length = self.target.arm_length
                    arm_angle = self.target.arm_angle
                    wrist_angle = self.target.wrist_angle
                else:
                    arm_length = self.target.arm_length_opposite
                    arm_angle = self.target.arm_angle_opposite
                    wrist_angle = self.target.wrist_angle_opposite

        if self.target.intake_enabled and self.intake_on and not self.target.intake_off:
            self.intake_command = command.IntakeEnable(
                self.intake, intake_reversed=self.target.intake_reversed
            )
        elif self.intake_on and self.target.intake_off:
            self.intake_command = SequentialCommandGroup(
                InstantCommand(lambda: self.intake.intake_motor.set_raw_output(0)),
                InstantCommand(lambda: self.intake.intake_piston.extend()),
            )
        elif self.intake_on:
            self.intake_command = command.IntakeDisable(self.intake)
        else:
            self.intake_command = InstantCommand(lambda: None)

        if self.target.claw_wait:
            if self.target.claw_picking and self.arm_on:
                self.arm_sequence = SequentialCommandGroup(
                    command.SetArm(self.arm, arm_length, arm_angle),
                    command.SetGrabber(
                        self.grabber,
                        wrist_angle,
                        True,
                        finish=False,
                        no_grab=self.target.grabber_no_grab,
                    ),
                )
            elif self.target.claw_scoring and self.arm_on:
                self.arm_sequence = ParallelCommandGroup(
                    command.SetArm(self.arm, arm_length, arm_angle),
                    SequentialCommandGroup(
                        command.SetGrabber(
                            self.grabber,
                            math.radians(90) * (-1 if wrist_angle > 0 else 1),
                            False,
                        ),
                        command.SetGrabber(
                            self.grabber,
                            math.radians(25) * (-1 if wrist_angle > 0 else 1),
                            False,
                            finish=False,
                        ),
                    ),
                )
            elif self.arm_on:
                self.arm_sequence = SequentialCommandGroup(
                    command.SetArm(self.arm, arm_length, arm_angle),
                    command.SetGrabber(
                        self.grabber,
                        wrist_angle,
                        False,
                        finish=False,
                        no_grab=self.target.grabber_no_grab,
                    ),
                )
            else:
                self.arm_sequence = InstantCommand(lambda: None)
        else:
            if self.target.claw_picking and self.arm_on:
                self.arm_sequence = ParallelCommandGroup(
                    command.SetArm(self.arm, arm_length, arm_angle),
                    command.SetGrabber(
                        self.grabber,
                        wrist_angle,
                        True,
                        auto_claw=True,
                        auto_cube=self.target.cube_picking,
                        auto_cone=self.target.cone_picking,
                        auto_double=self.target.double_station_picking,
                        no_grab=self.target.grabber_no_grab,
                        finish=False,
                    ),
                )
            elif self.target.claw_scoring and self.arm_on:
                self.arm_sequence = SequentialCommandGroup(
                    ParallelCommandGroup(
                        command.SetArm(self.arm, arm_length, arm_angle),
                        command.SetGrabber(
                            self.grabber,
                            wrist_angle,
                            False,
                            finish=False,
                            no_grab=self.target.grabber_no_grab,
                        ),
                    ),
                    InstantCommand(self.grabber.open_claw()),
                )
            elif self.arm_on:
                self.arm_sequence = ParallelCommandGroup(
                    command.SetArm(self.arm, arm_length, arm_angle),
                    command.SetGrabber(
                        self.grabber,
                        wrist_angle,
                        False,
                        finish=False,
                        no_grab=self.target.grabber_no_grab,
                    ),
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
