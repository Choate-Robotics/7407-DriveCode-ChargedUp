from robotpy_toolkit_7407.utils import logger
from oi.keymap import Keymap
import command
import constants
from robot_systems import Robot
logger.info("Hi, I'm OI!")


class OI:
    @staticmethod
    def init() -> None:
        logger.info("Initializing OI...")

    @staticmethod
    def map_controls():
        logger.info("Mapping controls...")
        
        Keymap.Arm.ELEVATOR_ROTATION_AXIS.whenPressed(command.SetAngle(Robot.elevator, Keymap.Arm.ELEVATOR_ROTATION_AXIS.value() * constants.shoulder_max_rotation))
    
    
