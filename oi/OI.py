from commands2 import InstantCommand
from robotpy_toolkit_7407.utils import logger

import command
from oi.keymap import Keymap
from robot_systems import Robot

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

        Keymap.Intake.INTAKE_ENABLE.whenPressed(command.IntakeEnable(Robot.intake))
        # Keymap.Intake.INTAKE_ENABLE.onFalse(command.IntakeDisable(Robot.intake))
        # Keymap.Claw.ENGAGE_CLAW.onTrue(InstantCommand(lambda: Robot.arm.engage_claw()))
        # Keymap.Claw.ENGAGE_CLAW.onFalse(
            # InstantCommand(lambda: Robot.arm.disengage_claw())
        # )

        Keymap.Drivetrain.RESET_GYRO.onTrue(InstantCommand(lambda: command.TestCommand()))

        Keymap.Scoring.CONFIRM.whenPressed(InstantCommand(lambda: print("CONFIRM")))
        Keymap.Scoring.DELETE.whenPressed(InstantCommand(lambda: print("DELETE")))

        Keymap.Scoring.LEFT_GRID.whenPressed(InstantCommand(lambda: print("LEFT GRID")))
        Keymap.Scoring.MIDDLE_GRID.onTrue(InstantCommand(lambda: print("MIDDLE GRID")))
        Keymap.Scoring.RIGHT_GRID.onTrue(InstantCommand(lambda: print("RIGHT GRID")))

        Keymap.Scoring.TOP_LEFT.onTrue(InstantCommand(lambda: print("TOP LEFT")))
        Keymap.Scoring.TOP_MIDDLE.onTrue(InstantCommand(lambda: print("TOP MIDDLE")))
        Keymap.Scoring.TOP_RIGHT.onTrue(InstantCommand(lambda: print("TOP RIGHT")))

        Keymap.Scoring.MIDDLE_LEFT.onTrue(InstantCommand(lambda: print("MIDDLE LEFT")))
        Keymap.Scoring.MIDDLE_MIDDLE.onTrue(InstantCommand(lambda: print("MIDDLE MIDDLE")))
        Keymap.Scoring.MIDDLE_RIGHT.onTrue(InstantCommand(lambda: print("MIDDLE RIGHT")))
        
        Keymap.Scoring.BOTTOM_LEFT.onTrue(InstantCommand(lambda: print("BOTTOM LEFT")))
        Keymap.Scoring.BOTTOM_MIDDLE.onTrue(InstantCommand(lambda: print("BOTTOM MIDDLE")))
        Keymap.Scoring.BOTTOM_RIGHT.onTrue(InstantCommand(lambda: print("BOTTOM RIGHT")))
