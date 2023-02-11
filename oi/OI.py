from robotpy_toolkit_7407.utils import logger
from oi.keymap import Keymap
import command
import constants
from robot_systems import Robot
logger.info("Hi, I'm OI!")



class OI:
    @staticmethod
    def init() -> None:
        pass

    @staticmethod
    def map_controls():
        logger.info("Mapping controls...")
        
        def engageClaw():
            Robot.Arm.engage_claw()
        
        def disEngageClaw():
            Robot.Arm.disengage_claw()

        #Keymap.Arm.REZERO_ELEVATOR.whenPressed(command.ZeroArm())
        
        #Keymap.Arm.ENGAGE_CLAW().whenPressed(engageClaw()).whenReleased(disEngageClaw())

    
