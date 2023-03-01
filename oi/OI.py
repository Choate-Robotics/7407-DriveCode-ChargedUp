import math

from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
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

        Keymap.Drivetrain.X_MODE.whenPressed(
            InstantCommand(lambda: Robot.drivetrain.x_mode())
        )

        Keymap.Drivetrain.ROUTE.whenPressed(
            command.DrivetrainRoute(Robot.drivetrain, Sensors.odometry)
        ).whenReleased(command.DrivetrainRegular(Robot.drivetrain, Sensors.odometry))

        Keymap.Drivetrain.LIMIT.whenPressed(
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
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.disengage_claw()),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["standard_pickup"],
                ),
            )
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
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.disengage_claw()),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["standard"],
                ),
            )
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

        Keymap.Claw.DROP_CLAW.whenPressed(
            command.SetGrabber(
                Robot.grabber,
                wrist_angle=math.radians(25)
                * (-1 if (config.scoring_locations["high"].arm_angle > 0) else 1),
                claw_active=False,
            )
        ).whenReleased(
            command.SetGrabber(
                Robot.grabber,
                wrist_angle=math.radians(25)
                * (1 if (config.scoring_locations["high"].arm_angle > 0) else -1),
                claw_active=False,
            )
        )

        Keymap.Targeting.TARGETING_HIGH.whenReleased(
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.disengage_claw()),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["standard"],
                ),
            )
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
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.disengage_claw()),
                command.SetGrabber(Robot.grabber, 0, False),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["standard"],
                ),
            )
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
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.disengage_claw()),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["standard_pickup"],
                ),
            )
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
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.disengage_claw()),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["standard"],
                ),
            )
        )

        Keymap.Claw.OPEN_CLAW_DRIVER.whenPressed(
            InstantCommand(lambda: Robot.grabber.open_claw()).andThen(
                lambda: print("DONE")
            )
        )

        Keymap.Claw.OPEN_CLAW_OPERATOR.whenPressed(
            InstantCommand(lambda: Robot.grabber.open_claw()).andThen(
                lambda: print("DONE")
            )
        ).whenReleased(
            InstantCommand(lambda: Robot.grabber.close_claw()).andThen(
                lambda: print("DONE")
            )
        )

        Keymap.Claw.RUN_CLAW_UP.whenPressed(
            InstantCommand(lambda: Robot.grabber.set_output(-0.3))
        ).whenReleased(InstantCommand(lambda: Robot.grabber.set_output(0)))

        Keymap.Claw.RUN_CLAW_DOWN.whenPressed(
            InstantCommand(lambda: Robot.grabber.set_output(0.3))
        ).whenReleased(InstantCommand(lambda: Robot.grabber.set_output(0)))
