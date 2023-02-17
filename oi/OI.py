from commands2 import InstantCommand
from robotpy_toolkit_7407.utils import logger

import command
import config
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

        def update_scoring_position(key: str, changes_to=""):
            changing_to = ""
            if key == "CONFIRM":
                # call scoring command
                config.scoring_position = ""
            elif key == "DELETE":
                config.scoring_position = ""

            elif key == "LG":
                changing_to = "1"
            elif key == "MG":
                changing_to = "2"
            elif key == "RG":
                changing_to = "3"
                
            elif key == "TL":
                changing_to = "7"
            elif key == "TM":
                changing_to = "8"
            elif key == "TR":
                changing_to = "9"
            elif key == "ML":
                changing_to = "4"
            elif key == "MM":
                changing_to = "5"
            elif key == "MR":
                changing_to = "6"
            elif key == "BL":
                changing_to = "1"
            elif key == "BM":
                changing_to = "2"
            elif key == "BR":
                changing_to = "3"
            
            if changes_to == "g":
                scoring_position_list = list(config.scoring_position)
                scoring_position_list[0] = changing_to
                config.scoring_position = ''.join(scoring_position_list)
            elif changing_to == "p":
                scoring_position_list = list(config.scoring_position)
                scoring_position_list[1] = changing_to
                config.scoring_position = ''.join(scoring_position_list)

        # Grid
        Keymap.Scoring.LEFT_GRID.onTrue(InstantCommand(update_scoring_position("LG", "g")))
        Keymap.Scoring.MIDDLE_GRID.onTrue(InstantCommand(update_scoring_position("MG", "g")))
        Keymap.Scoring.RIGHT_GRID.onTrue(InstantCommand(update_scoring_position("RG", "g")))

        # Position in grid
        Keymap.Scoring.TOP_LEFT.onTrue(InstantCommand(update_scoring_position("TL", "p")))
        Keymap.Scoring.TOP_MIDDLE.onTrue(InstantCommand(update_scoring_position("TM", "p")))
        Keymap.Scoring.TOP_RIGHT.onTrue(InstantCommand(update_scoring_position("TR", "p")))

        Keymap.Scoring.MIDDLE_LEFT.onTrue(InstantCommand(update_scoring_position("ML", "p")))
        Keymap.Scoring.MIDDLE_MIDDLE.onTrue(InstantCommand(update_scoring_position("MM", "p")))
        Keymap.Scoring.MIDDLE_RIGHT.onTrue(InstantCommand(update_scoring_position("MR", "p")))

        Keymap.Scoring.BOTTOM_LEFT.onTrue(InstantCommand(update_scoring_position("BL", "p")))
        Keymap.Scoring.BOTTOM_MIDDLE.onTrue(InstantCommand(update_scoring_position("BM", "p")))
        Keymap.Scoring.BOTTOM_RIGHT.onTrue(InstantCommand(update_scoring_position("BR", "p")))

        # Confirm and delete
        Keymap.Scoring.CONFIRM.onTrue(InstantCommand(update_scoring_position("CONFIRM")))
        Keymap.Scoring.DELETE.onTrue(InstantCommand(update_scoring_position("DELETE")))
