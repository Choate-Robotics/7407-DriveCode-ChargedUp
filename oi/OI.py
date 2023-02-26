from commands2 import (
    InstantCommand,
)
from robotpy_toolkit_7407.utils import logger

import command
import config
from oi.keymap import Keymap
from robot_systems import Robot, Sensors

logger.info("Hi, I'm OI!")


class OI:
    @staticmethod
    def init() -> None:
        pass

    @staticmethod
    def map_controls():
        logger.info("Mapping controls...")

        # DRIVETRAIN
        Keymap.Drivetrain.RESET_GYRO.whenPressed(
            InstantCommand(
                lambda: Robot.drivetrain.gyro.reset_angle(Robot.drivetrain.start_angle)
            )
        )

        Keymap.Drivetrain.RESET_ODOMETRY.whenPressed(
            InstantCommand(
                lambda: Robot.drivetrain.reset_odometry(Robot.drivetrain.start_pose)
            )
        )

        Keymap.Drivetrain.LIMIT_VELOCITY.whenPressed(
            command.DrivetrainScore(Robot.drivetrain, Sensors.odometry)
        ).whenReleased(command.DrivetrainRegular(Robot.drivetrain, Sensors.odometry))

        # TARGETING
        Keymap.Targeting.TARGETING_PICKUP.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["pickup"],
            )
        )

        Keymap.Targeting.TARGETING_PICKUP.whenReleased(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["standard_pickup"],
            ),
        )

        Keymap.Targeting.TARGETING_MIDDLE.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["middle"],
            )
        )

        Keymap.Targeting.TARGETING_MIDDLE.whenReleased(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["standard"],
            ),
        )

        Keymap.Targeting.TARGETING_HIGH.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["high"],
            )
        )

        Keymap.Targeting.TARGETING_HIGH.whenReleased(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["standard"],
            ),
        )

        Keymap.Targeting.TARGETING_DOUBLE_STATION.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["double_station"],
            )
        )

        Keymap.Targeting.TARGETING_DOUBLE_STATION.whenReleased(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["standard"],
            ),
        )

        Keymap.Targeting.TARGETING_CUBE_INTAKE.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["cube_intake"],
            )
        )

        Keymap.Targeting.TARGETING_CUBE_INTAKE.whenReleased(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["standard_pickup"],
            ),
        )

        Keymap.Targeting.TARGETING_EJECT.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["eject"],
            )
        )

        Keymap.Targeting.TARGETING_EJECT.whenReleased(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["standard"],
            ),
        )

        Keymap.Claw.OPEN_CLAW.whenPressed(
            InstantCommand(lambda: Robot.grabber.open_claw())
        )

        Keymap.Claw.OPEN_CLAW.whenReleased(
            InstantCommand(lambda: Robot.grabber.close_claw())
        )
