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
        
        #Keymap.Arm.REZERO_ELEVATOR.whenPressed(command.ZeroArm())
        
        #Keymap.Arm.ENGAGE_CLAW().whileHeld(command.EngageClaw()).whenReleased(command.DisengageClaw())

    
