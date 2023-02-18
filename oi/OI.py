import math

import commands2
from commands2 import InstantCommand
from robotpy_toolkit_7407.utils import logger
from wpimath.geometry import Pose2d

import command
from oi.keymap import Keymap
from robot_systems import Robot, Sensors

logger.info("Hi, I'm OI!")


class OI:
    @staticmethod
    def init() -> None:
        pass

    @staticmethod
    def map_controls():
        logger.info("Mapping controls...")

        # Keymap.Arm.REZERO_ELEVATOR.whenPressed(command.ZeroArm())

        # Keymap.Arm.ENGAGE_CLAW().whenPressed(engageClaw()).whenReleased(disEngageClaw())

        # Keymap.Intake.INTAKE_ENABLE.whenPressed(
        #     command.SetArm(Robot.Arm, 0, math.radians(0), math.radians(0), False)
        #     # command.IntakeEnable(Robot.intake).andThen(
        #     #     WaitCommand(0).andThen(command.SetArm(Robot.Arm, 0, math.radians(45), math.radians(45), False))
        #     # )
        # )

        Keymap.Intake.INTAKE_ENABLE.whenPressed(
            InstantCommand(lambda: Robot.intake.intake_enable())
        )

        Keymap.Intake.INTAKE_ENABLE.whenReleased(
            InstantCommand(lambda: Robot.intake.intake_disable())
        )

        Keymap.Claw.ENGAGE_CLAW.whenPressed(
            command.Target(
                Robot.drivetrain,
                Robot.arm,
                Robot.grabber,
                Sensors.odometry,
                pose=Pose2d(8.4, 2.3, 0),
                arm_angle=math.radians(45),
                arm_length=0,
                wrist_angle=math.radians(-45),
                wrist_enabled=True,
            )
        )

        Keymap.Claw.ENGAGE_CLAW.whenReleased(
            InstantCommand(
                lambda: commands2.CommandScheduler.getInstance().schedule(
                    commands=[
                        command.DriveSwerveCustom(Robot.drivetrain),
                        command.SetArm(Robot.arm, 0, 0),
                        command.SetGrabber(Robot.grabber, 0, False),
                        command.IntakeDisable(Robot.intake),
                    ]
                )
            )
        )

        # Keymap.Intake.INTAKE_ENABLE.whenReleased(
        #     command.SetArm(Robot.Arm, 0, 0, 0).andThen(
        #         WaitCommand(0).andThen(command.IntakeDisable(Robot.intake))
        #     )
        # )
        # Keymap.Intake.INTAKE_ENABLE.whenPressed(
        #     ParallelCommandGroup(
        #         command.SetArm(Robot.arm, distance=0, shoulder_angle=math.radians(45)),
        #         command.SetGrabber(Robot.grabber, wrist_angle=math.radians(-45), claw_active=True)
        #     )
        # )
        #
        # Keymap.Intake.INTAKE_ENABLE.whenReleased(
        #     ParallelCommandGroup(
        #         command.SetArm(Robot.arm, distance=0, shoulder_angle=math.radians(0)),
        #         command.SetGrabber(Robot.grabber, wrist_angle=math.radians(0), claw_active=False)
        #     )
        # )
        #
        # Keymap.Claw.ENGAGE_CLAW.onTrue(InstantCommand(lambda: Robot.arm.engage_claw()))
        # Keymap.Claw.ENGAGE_CLAW.onFalse(
        #     InstantCommand(lambda: Robot.arm.disengage_claw())
        #
        # )
