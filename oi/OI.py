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
        Keymap.Intake.INTAKE_ENABLE.onFalse(command.IntakeDisable(Robot.intake))
        Keymap.Claw.ENGAGE_CLAW.onTrue(InstantCommand(lambda: Robot.arm.engage_claw()))
        Keymap.Claw.ENGAGE_CLAW.onFalse(
            InstantCommand(lambda: Robot.arm.disengage_claw())
        )

        # Grid
        Keymap.Scoring.LEFT_GRID.onTrue()
        Keymap.Scoring.MIDDLE_GRID.onTrue()
        Keymap.Scoring.RIGHT_GRID.onTrue()

        # Position in grid
        Keymap.Scoring.TOP_LEFT.onTrue()
        Keymap.Scoring.TOP_MIDDLE.onTrue()
        Keymap.Scoring.TOP_RIGHT.onTrue()

        Keymap.Scoring.MIDDLE_LEFT.onTrue()
        Keymap.Scoring.MIDDLE_MIDDLE.onTrue()
        Keymap.Scoring.MIDDLE_RIGHT.onTrue()

        Keymap.Scoring.BOTTOM_LEFT.onTrue()
        Keymap.Scoring.BOTTOM_MIDDLE.onTrue()
        Keymap.Scoring.BOTTOM_RIGHT.onTrue()

        # Confirm and delete
        Keymap.Scoring.CONFIRM.onTrue()
        Keymap.Scoring.DELETE.onTrue()
