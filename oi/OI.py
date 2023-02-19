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

        Keymap.Scoring.CONFIRM.whenPressed(InstantCommand(lambda: print("CONFIRM pressed")))
