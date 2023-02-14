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

        def engageClaw():
            Robot.arm.engage_claw()

        def disEngageClaw():
            Robot.arm.disengage_claw()

        # Keymap.Arm.REZERO_ELEVATOR.whenPressed(command.ZeroArm())

        # Keymap.Arm.ENGAGE_CLAW().whenPressed(engageClaw()).whenReleased(disEngageClaw())

        Keymap.Arm.ARM_BRAKE.whenPressed(InstantCommand(Robot.arm.enable_brake()))
        Keymap.Arm.ARM_BRAKE.whenReleased(InstantCommand(Robot.arm.disable_brake()))

        Keymap.Intake.INTAKE_ENABLE.whenPressed(command.IntakeEnable(Robot.intake))
        Keymap.Claw.ENGAGE_CLAW.whenPressed(InstantCommand(Robot.arm.engage_claw()))
        Keymap.Claw.ENGAGE_CLAW.whenReleased(InstantCommand(Robot.arm.disengage_claw()))
