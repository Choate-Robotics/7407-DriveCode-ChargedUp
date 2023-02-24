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

        def valid_current_scoring_position(pos: str):
            grid = "123"
            position = "123456789"
            if len(pos) == 2 and grid.find(pos[0]) >= 0 and position.find(pos[1]) >= 0:
                return True
            return False

        def edit_current_scoring_position(key: str, changing: str):
            scoring_pos = list(config.current_scoring_position)

            code = {
                "LG": 1,
                "MG": 2,
                "RG": 3,
                "TL": 7,
                "TM": 8,
                "TR": 9,
                "ML": 4,
                "MM": 5,
                "MR": 6,
                "BL": 1,
                "BM": 2,
                "BR": 3
            }

            if changing == "g":
                scoring_pos[0] = code[key]
            elif changing == "p":
                scoring_pos[1] = code[key]
                
            config.current_scoring_position = ''.join(scoring_pos)

            if key == "DELETE":
                config.current_scoring_position = ""
            elif key == "CONFIRM":
                if valid_current_scoring_position(config.current_scoring_position):
                    config.real_scoring_position = config.current_scoring_position
                config.current_scoring_position = ""
            
            print("CURRENT SCORING POSITION: ", config.current_scoring_position)
            print("REAL SCORING POSITION: ", config.real_scoring_position)

        Keymap.Scoring.CONFIRM.onTrue(InstantCommand(edit_current_scoring_position("CONFIRM", "")))
        Keymap.Scoring.DELETE.onTrue(InstantCommand(edit_current_scoring_position("DELETE", "")))

        Keymap.Scoring.LEFT_GRID.onTrue(InstantCommand(edit_current_scoring_position("LG", "g")))
        Keymap.Scoring.MIDDLE_GRID.onTrue(InstantCommand(edit_current_scoring_position("MG", "g")))
        Keymap.Scoring.RIGHT_GRID.onTrue(InstantCommand(edit_current_scoring_position("RG", "g")))

        Keymap.Scoring.TOP_LEFT.onTrue(InstantCommand(edit_current_scoring_position("TL", "p")))
        Keymap.Scoring.TOP_MIDDLE.onTrue(InstantCommand(edit_current_scoring_position("TM", "p")))
        Keymap.Scoring.TOP_RIGHT.onTrue(InstantCommand(edit_current_scoring_position("TR", "p")))

        Keymap.Scoring.MIDDLE_LEFT.onTrue(InstantCommand(edit_current_scoring_position("ML", "p")))
        Keymap.Scoring.MIDDLE_MIDDLE.onTrue(InstantCommand(edit_current_scoring_position("MM", "p")))
        Keymap.Scoring.MIDDLE_RIGHT.onTrue(InstantCommand(edit_current_scoring_position("MR", "p")))
        
        Keymap.Scoring.BOTTOM_LEFT.onTrue(InstantCommand(edit_current_scoring_position("BL", "p")))
        Keymap.Scoring.BOTTOM_MIDDLE.onTrue(InstantCommand(edit_current_scoring_position("BM", "p")))
        Keymap.Scoring.BOTTOM_RIGHT.onTrue(InstantCommand(edit_current_scoring_position("BR", "p")))
