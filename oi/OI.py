import math

import commands2
from commands2 import (
    InstantCommand,
    ParallelCommandGroup,
    SequentialCommandGroup,
    WaitCommand,
)
from robotpy_toolkit_7407.utils import logger

import command
import config
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
        # Keymap.Intake.INTAKE_ENABLE.whenReleased(
        #     ParallelCommandGroup(
        #         command.SetArm(Robot.arm, distance=0, shoulder_angle=math.radians(0)),
        #         command.SetGrabber(Robot.grabber, wrist_angle=math.radians(0), claw_active=False)
        #     )
        # )

        # Keymap.Intake.INTAKE_ENABLE.whenPressed(
        #     command.setElevator(Robot.arm, .3)
        # )
        #
        # Keymap.Intake.INTAKE_ENABLE.whenReleased(
        #     command.setElevator(Robot.arm, 0)
        # )

        # Keymap.Claw.ENGAGE_CLAW.onTrue(InstantCommand(lambda: Robot.arm.engage_claw()))
        # Keymap.Claw.ENGAGE_CLAW.onFalse(
        #     InstantCommand(lambda: Robot.arm.disengage_claw())
        #
        # )

        # STAND ALONE SCORING/PLACING COMMANDS

        Keymap.Intake.PICK_UP_ARM.whenPressed(
            ParallelCommandGroup(
                command.SetArm(
                    Robot.arm,
                    distance=0,
                    shoulder_angle=math.radians(-30),  # .099, -100
                ),
                command.SetGrabber(
                    Robot.grabber, wrist_angle=math.radians(-20.53), claw_active=True
                ),
            )
        )
        #
        Keymap.Intake.PICK_UP_ARM.whenReleased(
            ParallelCommandGroup(
                command.SetArm(Robot.arm, distance=0, shoulder_angle=math.radians(0)),
                command.SetGrabber(
                    Robot.grabber, wrist_angle=math.radians(0), claw_active=False
                ),
            )
        )
        #
        # Keymap.Intake.DROP_OFF_ARM.whenPressed(
        #     ParallelCommandGroup(
        #         command.SetArm(
        #             Robot.arm, distance=0.55, shoulder_angle=math.radians(-44.78)
        #         ),
        #         command.SetGrabber(
        #             Robot.grabber, wrist_angle=math.radians(-27.09), claw_active=False
        #         ),
        #     )
        # )
        #
        # Keymap.Intake.DROP_OFF_ARM.whenReleased(
        #     ParallelCommandGroup(
        #         command.SetArm(Robot.arm, distance=0, shoulder_angle=math.radians(0)),
        #         command.SetGrabber(
        #             Robot.grabber, wrist_angle=math.radians(0), claw_active=False
        #         ),
        #     )
        # )
        #
        # Keymap.Intake.GRABBER_PICK.whenPressed(
        #     InstantCommand(lambda: Robot.grabber.engage_claw())
        # )
        #
        # Keymap.Intake.GRABBER_PICK.whenReleased(
        #     SequentialCommandGroup(
        #         InstantCommand(lambda: Robot.grabber.disengage_claw())
        #     )
        # )
        #
        # Keymap.Intake.GRABBER_SCORE.whenPressed(
        #     InstantCommand(lambda: Robot.grabber.open_claw())
        # )
        #
        # Keymap.Intake.GRABBER_SCORE.whenReleased(
        #     InstantCommand(lambda: Robot.grabber.disengage_claw())
        # )

        # DRIVETRAIN
        Keymap.Drivetrain.RESET_GYRO.whenPressed(
            InstantCommand(lambda: Robot.drivetrain.gyro.reset_angle(0))
        )

        Keymap.Drivetrain.RESET_ODOMETRY.whenPressed(
            InstantCommand(
                lambda: Robot.drivetrain.reset_odometry(Robot.drivetrain.start_pose)
            )
        )

        # TARGETING
        Keymap.Targeting.TARGETING_PICKUP.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                drivetrain=Robot.drivetrain,
                target=config.scoring_locations["pickup"],
            )
        )
        #
        Keymap.Targeting.TARGETING_PICKUP.whenReleased(
            SequentialCommandGroup(
                InstantCommand(
                    lambda: commands2.CommandScheduler.getInstance().schedule(
                        command.DriveSwerveCustom(Robot.drivetrain)
                    )
                ),
                InstantCommand(lambda: Robot.grabber.close_claw()),
                WaitCommand(0.2),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    drivetrain=None,
                    target=config.scoring_locations["standard"],
                ),
            )
        )

        Keymap.Targeting.TARGETING_SCORING.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                drivetrain=Robot.drivetrain,
                target=config.scoring_locations["middle"],
            )
        )

        Keymap.Targeting.TARGETING_PICKUP.whenReleased(
            SequentialCommandGroup(
                command.DriveSwerveCustom(Robot.drivetrain),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    drivetrain=None,
                    target=config.scoring_locations["standard"],
                ),
            )
        )
