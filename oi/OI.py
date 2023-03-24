import math

import wpilib
from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    WaitCommand,
)
from robotpy_toolkit_7407.utils import logger

import command
import config
from oi.keymap import Controllers, Keymap
from robot_systems import Robot, Sensors
import constants

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
        def deploy():
            InstantCommand(command.ClimberDeploy(Robot.climber))
        

        Keymap.Drivetrain.SLOW_FORWARD.whenPressed(
            command.DrivetrainScoreBack(Robot.drivetrain, Sensors.odometry)
        ).whenReleased(command.DrivetrainRegular(Robot.drivetrain, Sensors.odometry))

        Keymap.Drivetrain.SLOW_REVERSE.whenPressed(
            command.DrivetrainScoreFront(Robot.drivetrain, Sensors.odometry)
        ).whenReleased(command.DrivetrainRegular(Robot.drivetrain, Sensors.odometry))
        
        Keymap.Climber.DEPLOY.whenPressed(
            command.ClimberDeploy(Robot.climber)
        )
        
        def unpivot():
            if Robot.climber.climber_active == False or Robot.climber.pivoted == False:
                return
            else:
                print("UnPivoting")
                InstantCommand(command.ClimberUnpivot(Robot.climber))
        
        Keymap.Climber.UNCLIMB.whenPressed(
            command.ClimberUnpivot(Robot.climber)
        )
        
        def pivot():
            if Robot.climber.climber_active == False:
                return
            else:
                print("Pivoting")
                command.ClimberPivot(Robot.climber)
        
        Keymap.Climber.CLIMB.whenPressed(
            command.ClimberPivot(Robot.climber)
        )
        

        # Keymap.Drivetrain.LANDING_GEAR_RIGHT.whenPressed(
        #     SequentialCommandGroup(
        #         InstantCommand(lambda: Robot.landing_gear.deploy()),
        #         WaitCommand(0.5),
        #         InstantCommand(lambda: Robot.landing_gear.release()),
        #     )
        # )

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
                InstantCommand(
                    lambda: Controllers.OPERATOR_CONTROLLER.setRumble(
                        wpilib.Joystick.RumbleType.kBothRumble, 0
                    )
                ),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["standard_pickup"],
                ),
            )
        )

        Keymap.Targeting.TARGETING_LOW.whenPressed(
            command.Target(
                Robot.arm,
                Robot.grabber,
                Robot.intake,
                Sensors.odometry,
                target=config.scoring_locations["low"],
            )
        ).whenReleased(
            SequentialCommandGroup(
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["standard"],
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

        def gyro_angle_calc():
            gyro_angle = Sensors.gyro.get_robot_heading() % (math.pi * 2)
            gyro_angle = math.degrees(
                math.atan2(math.sin(gyro_angle), math.cos(gyro_angle))
            )

            if -90 < gyro_angle < 90:
                return 1
            else:
                return -1

        def set_claw_angle(angle):
            config.grabber_target_angle = angle

        Keymap.Claw.DROP_CLAW.whenPressed(
            InstantCommand(
                lambda: set_claw_angle(math.radians(25) * gyro_angle_calc() * -1)
            )
        ).whenReleased(
            InstantCommand(lambda: set_claw_angle(math.radians(25) * gyro_angle_calc()))
        )

        Keymap.Targeting.TARGETING_HIGH.whenReleased(
            SequentialCommandGroup(
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
                WaitCommand(0.3),
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
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.engage_claw()),
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    target=config.scoring_locations["cube_intake"],
                ),
            )
        )

        Keymap.Targeting.TARGETING_CUBE_INTAKE.whenReleased(
            SequentialCommandGroup(
                InstantCommand(lambda: Robot.grabber.disengage_claw()),
                InstantCommand(
                    lambda: Controllers.OPERATOR_CONTROLLER.setRumble(
                        wpilib.Joystick.RumbleType.kBothRumble, 0
                    )
                ),
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

        # Keymap.Targeting.ZERO_ARM.whenPressed(
        #     InstantCommand(lambda: Robot.arm.zero_elevator_rotation())
        # )
