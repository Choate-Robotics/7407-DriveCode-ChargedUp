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

        def edit_current_scoring_position(key: str, changing=2):  # 0 = grid, 1 = position, 2 = confirm
            def valid_current_scoring_position(pos: str):
                grid = "ABC"
                position = "LMR"
                return len(pos) == 2 and pos[0] in grid and pos[1] in position

            if key == "CONFIRM":
                if valid_current_scoring_position(config.current_scoring_position):
                    config.real_scoring_position = config.current_scoring_position
            else:
                scoring_pos = list(config.current_scoring_position)
                scoring_pos[changing] = key
                config.current_scoring_position = ''.join(scoring_pos)

        Keymap.Scoring.CONFIRM.onTrue(InstantCommand(lambda: edit_current_scoring_position("CONFIRM")))

        Keymap.Scoring.LEFT_GRID.onTrue(InstantCommand(lambda: edit_current_scoring_position("A", 0)))
        Keymap.Scoring.MIDDLE_GRID.onTrue(InstantCommand(lambda: edit_current_scoring_position("B", 0)))
        Keymap.Scoring.RIGHT_GRID.onTrue(InstantCommand(lambda: edit_current_scoring_position("C", 0)))

        Keymap.Scoring.LEFT.onTrue(InstantCommand(lambda: edit_current_scoring_position("L", 1)))
        Keymap.Scoring.MIDDLE.onTrue(InstantCommand(lambda: edit_current_scoring_position("M", 1)))
        Keymap.Scoring.RIGHT.onTrue(InstantCommand(lambda: edit_current_scoring_position("R", 1)))
