import math

import commands2
import wpilib
from commands2 import InstantCommand, SequentialCommandGroup
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d

import autonomous
import command
import config
import constants
from autonomous.auto_routine import AutoRoutine
from oi.OI import OI
from robot_systems import Pneumatics, Robot, Sensors
from sensors import FieldOdometry, PV_Cameras
from utils import logger


class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()
        self.teleop_zero: wpilib.SendableChooser | None = None
        self.pv_selection: wpilib.SendableChooser | None = None
        self.auto_selection: wpilib.SendableChooser | None = None

    def robotInit(self):
        period = 0.05
        commands2.CommandScheduler.getInstance().setPeriod(period)
        Pneumatics.compressor.enableAnalog(90, 120)

        Robot.intake.init()
        Robot.climber.init()
        Robot.drivetrain.init()
        Robot.arm.init()
        Robot.grabber.init()

        for i in range(10):
            Robot.intake.intake_motor.motor.setInverted(True)

        for i in range(10):
            Robot.climber.climber_motor.motor.setInverted(False)

        for i in range(10):
            Robot.arm.motor_extend.motor.setInverted(True)

        Sensors.pv_controller = None
        Sensors.odometry = FieldOdometry(Robot.drivetrain, None)
        Sensors.gyro = Robot.drivetrain.gyro

        # SmartDashboard.putNumber("ELEVATOR_Voltage", 0)
        # SmartDashboard.putNumber("Test_ELEVATOR_Voltage", 0)
        # SmartDashboard.getNumber("ELEVATOR_P_VALUE", 0)
        # SmartDashboard.putNumber("ARM_Voltage", 0)

        OI.init()
        OI.map_controls()

        self.pv_selection = wpilib.SendableChooser()
        self.pv_selection.setDefaultOption("On", "on")
        self.pv_selection.addOption("Off", "off")
        wpilib.SmartDashboard.putData("Photonvision", self.pv_selection)

        self.auto_selection = wpilib.SendableChooser()

        self.auto_selection.setDefaultOption("BALANCE", autonomous.balance_auto)

        self.auto_selection.addOption(
            "3 NO GUARD BLUE", autonomous.blue_three_piece_no_guard
        )

        self.auto_selection.addOption(
            "3 NO GUARD RED", autonomous.red_three_piece_no_guard
        )

        self.auto_selection.addOption(
            "3 WITH GUARD BLUE", autonomous.blue_three_piece_with_guard
        )

        self.auto_selection.addOption(
            "3 WITH GUARD RED", autonomous.red_three_piece_with_guard
        )

        self.auto_selection.addOption(
            "2 NO GUARD BLUE", autonomous.blue_two_piece_no_guard
        )

        self.auto_selection.addOption(
            "2 NO GUARD RED", autonomous.red_two_piece_no_guard
        )

        self.auto_selection.addOption(
            "2 WITH GUARD BLUE", autonomous.blue_two_piece_with_guard
        )

        self.auto_selection.addOption(
            "2 WITH GUARD RED", autonomous.red_two_piece_with_guard
        )

        self.auto_selection.addOption("NEW_BALANCE", autonomous.new_balance)

        self.auto_selection.addOption(
            "Do Nothing", AutoRoutine(Pose2d(0, 0, 0), InstantCommand(lambda: None))
        )

        wpilib.SmartDashboard.putData("Auto Mode", self.auto_selection)

        self.teleop_zero = wpilib.SendableChooser()
        self.teleop_zero.setDefaultOption("Off", "off")
        self.teleop_zero.addOption("On", "on")
        wpilib.SmartDashboard.putData("Teleop Zero", self.teleop_zero)

        Robot.drivetrain.n_front_left.initial_zero()
        Robot.drivetrain.n_front_right.initial_zero()
        Robot.drivetrain.n_back_left.initial_zero()
        Robot.drivetrain.n_back_right.initial_zero()

    def robotPeriodic(self):

        SmartDashboard.putBoolean("Climbed", Robot.climber.is_climbed())
        SmartDashboard.putNumber(
            "Robot Roll", math.degrees(Sensors.gyro.get_robot_roll())
        )
        SmartDashboard.putNumber(
            "Robot Pitch", math.degrees(Sensors.gyro.get_robot_pitch())
        )
        SmartDashboard.putNumber(
            "Robot Yaw", math.degrees(Sensors.gyro.get_robot_heading())
        )
        # SmartDashboard.putNumber("PITCH", Robot.drivetrain.gyro.get_robot_pitch())
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
        SmartDashboard.putNumber(
            "ELEVATOR CURRENT", Robot.arm.motor_extend.motor.getOutputCurrent()
        )
        SmartDashboard.putNumber(
            "ELEVATOR BUS VOLTAGE", Robot.arm.motor_extend.motor.getBusVoltage()
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

        # SmartDashboard.putNumber(
        #     "FRONT LEFT VEL", Robot.drivetrain.n_front_left.get_motor_velocity()
        # )
        # SmartDashboard.putNumber(
        #     "FRONT RIGHT VEL", Robot.drivetrain.n_front_right.get_motor_velocity()
        # )
        # SmartDashboard.putNumber(
        #     "BACK LEFT VEL", Robot.drivetrain.n_back_left.get_motor_velocity()
        # )
        # SmartDashboard.putNumber(
        #     "BACK RIGHT VEL", Robot.drivetrain.n_back_right.get_motor_velocity()
        # )

        # SmartDashboard.putNumber(
        #     "FRONT LEFT INTERNAL",
        #     Robot.drivetrain.n_front_left.m_turn.get_sensor_position(),
        # )
        # SmartDashboard.putNumber(
        #     "FRONT RIGHT INTERNAL",
        #     Robot.drivetrain.n_front_right.m_turn.get_sensor_position(),
        # )
        # SmartDashboard.putNumber(
        #     "BACK LEFT INTERNAL",
        #     Robot.drivetrain.n_back_left.m_turn.get_sensor_position(),
        # )
        # SmartDashboard.putNumber(
        #     "BACK RIGHT INTERNAL",
        #     Robot.drivetrain.n_back_right.m_turn.get_sensor_position(),
        # )

        SmartDashboard.putNumber(
            "FRONT LEFT", Robot.drivetrain.n_front_left.encoder.getAbsolutePosition()
        )
        SmartDashboard.putNumber(
            "FRONT RIGHT", Robot.drivetrain.n_front_right.encoder.getAbsolutePosition()
        )
        SmartDashboard.putNumber(
            "BACK LEFT", Robot.drivetrain.n_back_left.encoder.getAbsolutePosition()
        )
        SmartDashboard.putNumber(
            "BACK RIGHT", Robot.drivetrain.n_back_right.encoder.getAbsolutePosition()
        )

        # SmartDashboard.putNumber(
        #     "Gyro Pitch", Robot.drivetrain.gyro.get_robot_pitch()
        # )

        # SmartDashboard.putString(
        #     "Raw Gyro", str(Robot.drivetrain.gyro._gyro.getRawGyro())
        # )

        try:
            commands2.CommandScheduler.getInstance().run()
        except Exception as e:
            print(e)

    def teleopInit(self):
        # Robot.drivetrain.n_front_left.zero()
        # Robot.drivetrain.n_front_right.zero()
        # Robot.drivetrain.n_back_left.zero()
        # Robot.drivetrain.n_back_right.zero()

        # Robot.drivetrain.n_front_left.m_turn.set_sensor_position(0)
        # Robot.drivetrain.n_front_right.m_turn.set_sensor_position(0)
        # Robot.drivetrain.n_back_left.m_turn.set_sensor_position(0)
        # Robot.drivetrain.n_back_right.m_turn.set_sensor_position(0)

        # Robot.drivetrain.n_front_left.m_turn.set_target_position(constants.drivetrain_turn_gear_ratio * 100)
        # Robot.drivetrain.n_front_right.m_turn.set_target_position(constants.drivetrain_turn_gear_ratio * 100)
        # Robot.drivetrain.n_back_left.m_turn.set_target_position(constants.drivetrain_turn_gear_ratio * 100)
        # Robot.drivetrain.n_back_right.m_turn.set_target_position(constants.drivetrain_turn_gear_ratio * 100)

        Robot.climber.climber_disable()
        Robot.climber.latch_enable()
        Robot.climber.climber_motor.set_sensor_position(0)
        Robot.climber.set_motor_rotations(0)

        logger.debug("TELEOP", "Teleop Initialized")

        config.blue_team = self.auto_selection.getSelected().blue_team

        commands2.CommandScheduler.getInstance().schedule(
            command.DriveSwerveCustom(Robot.drivetrain)
        )

        Robot.arm.arm_rotation_motor.pid_controller.setOutputRange(-0.2, 0.2, slotID=1)

        if self.teleop_zero.getSelected() == "on":
            commands2.CommandScheduler.getInstance().schedule(
                SequentialCommandGroup(
                    command.ZeroElevator(Robot.arm),
                    command.ZeroShoulder(Robot.arm),
                    command.ZeroWrist(Robot.grabber),
                    command.Target(
                        Robot.arm,
                        Robot.grabber,
                        Robot.intake,
                        Sensors.odometry,
                        config.scoring_locations["standard"],
                    ),
                )
            )
            Robot.drivetrain.n_front_left.zero()
            Robot.drivetrain.n_front_right.zero()
            Robot.drivetrain.n_back_left.zero()
            Robot.drivetrain.n_back_right.zero()
        else:
            commands2.CommandScheduler.getInstance().schedule(
                command.Target(
                    Robot.arm,
                    Robot.grabber,
                    Robot.intake,
                    Sensors.odometry,
                    config.scoring_locations["standard"],
                )
            )

        if self.pv_selection.getSelected() == "on":
            Sensors.pv_controller = PV_Cameras(
                constants.ApriltagPositionDictBlue
                if config.blue_team
                else constants.ApriltagPositionDictRed
            )
            Sensors.odometry = FieldOdometry(Robot.drivetrain, Sensors.pv_controller)
        else:
            Sensors.pv_controller = None
            Sensors.odometry = FieldOdometry(Robot.drivetrain, None)

        # self.iters = 0

    def teleopPeriodic(self):
        if config.grabber_disable_intake:
            Robot.intake.intake_motor.set_raw_output(0)
            config.grabber_disable_intake = False
        # reported = math.degrees(Robot.drivetrain.n_front_left.get_current_motor_angle())
        # actual = (
        #     Robot.drivetrain.n_front_left.encoder.getAbsolutePosition()
        #     - Robot.drivetrain.n_front_left.absolute_encoder_zeroed_pos
        # )
        #
        # SmartDashboard.putNumber("n_front_left_error", reported - actual)
        #
        # reported = math.degrees(
        #     Robot.drivetrain.n_front_right.get_current_motor_angle()
        # )
        # actual = (
        #     Robot.drivetrain.n_front_right.encoder.getAbsolutePosition()
        #     - Robot.drivetrain.n_front_right.absolute_encoder_zeroed_pos
        # )
        #
        # SmartDashboard.putNumber("n_front_right_error", reported - actual)
        #
        # reported = math.degrees(Robot.drivetrain.n_back_left.get_current_motor_angle())
        # actual = (
        #     Robot.drivetrain.n_back_left.encoder.getAbsolutePosition()
        #     - Robot.drivetrain.n_back_left.absolute_encoder_zeroed_pos
        # )
        #
        # SmartDashboard.putNumber("n_back_left_error", reported - actual)
        #
        # reported = math.degrees(Robot.drivetrain.n_back_right.get_current_motor_angle())
        # actual = (
        #     Robot.drivetrain.n_back_right.encoder.getAbsolutePosition()
        #     - Robot.drivetrain.n_back_right.absolute_encoder_zeroed_pos
        # )
        #
        # SmartDashboard.putNumber("n_back_right_error", reported - actual)
        #
        # # self.iters += 1
        # # if self.iters % 5 == 0:
        # #     Robot.drivetrain.n_front_left.zero()
        # #     Robot.drivetrain.n_front_right.zero()
        # #     Robot.drivetrain.n_back_left.zero()
        # #     Robot.drivetrain.n_back_right.zero()
        #
        # SmartDashboard.putString(
        #     "Scoring Position", str(config.current_scoring_position)
        # )
        #
        # SmartDashboard.putNumber(
        #     "FRONT LEFT", Robot.drivetrain.n_front_left.encoder.getAbsolutePosition()
        # )
        # SmartDashboard.putNumber(
        #     "FRONT RIGHT", Robot.drivetrain.n_front_right.encoder.getAbsolutePosition()
        # )
        # SmartDashboard.putNumber(
        #     "BACK LEFT", Robot.drivetrain.n_back_left.encoder.getAbsolutePosition()
        # )
        # SmartDashboard.putNumber(
        #     "BACK RIGHT", Robot.drivetrain.n_back_right.encoder.getAbsolutePosition()
        # )
        ...

    def autonomousInit(self):
        Robot.drivetrain.n_front_left.zero()
        Robot.drivetrain.n_front_right.zero()
        Robot.drivetrain.n_back_left.zero()
        Robot.drivetrain.n_back_right.zero()

        Robot.climber.climber_disable()
        Robot.climber.latch_enable()
        Robot.climber.climber_motor.set_sensor_position(0)
        Robot.climber.set_motor_rotations(0)

        config.blue_team = self.auto_selection.getSelected().blue_team
        if self.pv_selection.getSelected() == "on":
            Sensors.pv_controller = PV_Cameras(
                constants.ApriltagPositionDictBlue
                if config.blue_team
                else constants.ApriltagPositionDictRed
            )
            Sensors.odometry = FieldOdometry(Robot.drivetrain, Sensors.pv_controller)
        else:
            Sensors.pv_controller = None
            Sensors.odometry = FieldOdometry(Robot.drivetrain, None)
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
