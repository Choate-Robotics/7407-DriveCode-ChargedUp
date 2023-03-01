import math

import commands2
import wpilib
from commands2 import InstantCommand
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d

import autonomous
import command
import config
from autonomous.auto_routine import AutoRoutine
from oi.OI import OI
from robot_systems import Pneumatics, Robot, Sensors
from sensors import FieldOdometry, PV_Cameras
from utils import logger


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()
        self.auto_selection: wpilib.SendableChooser | None = None

    def robotInit(self):
        period = 0.03
        commands2.CommandScheduler.getInstance().setPeriod(period)
        Pneumatics.compressor.enableAnalog(90, 120)

        Robot.arm.init()
        Robot.drivetrain.init()
        Robot.intake.init()
        Robot.grabber.init()

        Sensors.pv_controller = PV_Cameras()
        Sensors.odometry = FieldOdometry(Robot.drivetrain, Sensors.pv_controller)
        Sensors.gyro = Robot.drivetrain.gyro

        SmartDashboard.putNumber("ELEVATOR_Voltage", 0)
        SmartDashboard.putNumber("Test_ELEVATOR_Voltage", 0)
        SmartDashboard.getNumber("ELEVATOR_P_VALUE", 0)
        SmartDashboard.putNumber("ARM_Voltage", 0)

        OI.init()
        OI.map_controls()

        self.auto_selection = wpilib.SendableChooser()
        self.auto_selection.setDefaultOption("Basic Auto", autonomous.BasicAuto)
        self.auto_selection.addOption(
            "Do Nothing", AutoRoutine(Pose2d(0, 0, 0), InstantCommand(lambda: None))
        )
        self.auto_selection.addOption("Square Auto", autonomous.SquareAuto)
        self.auto_selection.addOption("Balance Auto", autonomous.BalanceAuto)
        self.auto_selection.addOption("Cone Cube Right", autonomous.ConeCubeRight)
        self.auto_selection.addOption("Cone Cube Left", autonomous.ConeCubeLeft)

        wpilib.SmartDashboard.putData("Auto Mode", self.auto_selection)

    def robotPeriodic(self):
        SmartDashboard.putNumber("PITCH", Robot.drivetrain.gyro.get_robot_pitch())
        # SmartDashboard.putNumber("ARM_REAL", math.degrees(Robot.arm.get_rotation()))
        #
        # Sensors.odometry.update()
        # SmartDashboard.putString("ODOM", str(Robot.drivetrain.odometry.getPose()))
        # SmartDashboard.putString("FDOM", str(Sensors.odometry.getPose()))
        # SmartDashboard.putNumber("Current_length_meters", Robot.arm.get_length())
        # SmartDashboard.putString(
        #     "EDOM", str(Robot.drivetrain.odometry_estimator.getEstimatedPosition())
        # )
        # try:
        #     SmartDashboard.putString(
        #         "PHOTON", str(Sensors.pv_controller.get_estimated_robot_pose())
        #     )
        #     SmartDashboard.putString(
        #         "PHOTON ANGLE",
        #         str(
        #             Sensors.pv_controller.get_estimated_robot_pose()[0][0]
        #             .rotation()
        #             .toRotation2d()
        #             .degrees()
        #         ),
        #     )
        # except Exception:
        #     pass
        #
        Sensors.odometry.update()
        pose = Robot.drivetrain.odometry_estimator.getEstimatedPosition()
        # pose2 = Sensors.odometry.getPose()
        #
        SmartDashboard.putNumberArray(
            "RobotPoseAdvantage", [pose.X(), pose.Y(), pose.rotation().radians()]
        )
        #
        # SmartDashboard.putNumberArray(
        #     "RobotPoseOrig", [pose2.X(), pose2.Y(), pose2.rotation().radians()]
        # )
        #
        try:
            pv_pose = Sensors.pv_controller.get_estimated_robot_pose()
            SmartDashboard.putNumberArray(
                "PVPoseAdvantage",
                [
                    pv_pose[0][0].toPose2d().X(),
                    pv_pose[0][0].toPose2d().Y(),
                    pv_pose[0][0].rotation().toRotation2d().radians(),
                ],
            )
        except Exception:
            pass
        #
        # SmartDashboard.putNumber(
        #     "gyro_angle: ", math.degrees(Robot.drivetrain.gyro.get_robot_heading())
        # )
        #
        SmartDashboard.putNumber(
            "SHOULDER ANGLE: ", math.degrees(Robot.arm.get_rotation())
        )
        SmartDashboard.putNumber(
            "WRIST ANGLE: ", math.degrees(Robot.grabber.get_angle())
        )
        SmartDashboard.putNumber("SHOULDER DIST: ", Robot.arm.get_length())

        try:
            commands2.CommandScheduler.getInstance().run()
        except Exception as e:
            ...

    def teleopInit(self):
        logger.debug("TELEOP", "Teleop Initialized")
        commands2.CommandScheduler.getInstance().schedule(
            command.DriveSwerveCustom(Robot.drivetrain)
        )
        Robot.arm.arm_rotation_motor.pid_controller.setOutputRange(-0.2, 0.2, slotID=1)
        commands2.CommandScheduler.getInstance().schedule(
            command.ZeroElevator(Robot.arm).andThen(
                command.ZeroShoulder(Robot.arm).andThen(
                    command.ZeroWrist(Robot.grabber).andThen(
                        command.Target(
                            Robot.arm,
                            Robot.grabber,
                            Robot.intake,
                            Sensors.odometry,
                            config.scoring_locations["standard"],
                        )
                    )
                )
            )
        )

        Robot.arm.enable_brake()

    def teleopPeriodic(self):
        # SmartDashboard.putBoolean(
        #     "Limit Switch", Robot.arm.elevator_bottom_sensor.get()
        # )
        # print("Limit Switch: ", Robot.arm.elevator_bottom_sensor.get())

        # SmartDashboard.putBoolean("ELEVATOR_BOTTOM_SENSOR", Robot.arm.elevator_bottom_sensor.get())
        # print(Pneumatics.compressor.getPressure())
        # print("I THINK I'm AT: ", math.degrees(Robot.Arm.get_rotation()))
        # Robot.Arm.disable_brake()
        ...

    def autonomousInit(self):
        Robot.arm.arm_rotation_motor.pid_controller.setOutputRange(-0.2, 0.2, slotID=1)
        self.auto_selection.getSelected().run()

    def autonomousPeriodic(self):
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(_Robot)
