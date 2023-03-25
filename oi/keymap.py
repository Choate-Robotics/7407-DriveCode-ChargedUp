import commands2.button
import wpilib
from robotpy_toolkit_7407.oi import (
    JoystickAxis,
    XBoxController,
)
from robotpy_toolkit_7407.oi.joysticks import Joysticks

controllerDRIVER = XBoxController
controllerOPERATOR = XBoxController


class Controllers:
    DRIVER = 0
    OPERATOR = 1

    DRIVER_CONTROLLER = wpilib.Joystick(0)
    OPERATOR_CONTROLLER = wpilib.Joystick(1)


class Keymap:
    class Drivetrain:
        DRIVE_X_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[0])
        DRIVE_Y_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[1])
        DRIVE_ROTATION_AXIS = JoystickAxis(
            Controllers.DRIVER, controllerDRIVER.R_JOY[0]
        )
        RESET_GYRO = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.B
        )
        RESET_ODOMETRY = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.Y
        )
        SLOW_REVERSE = commands2.button.Button(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.LT) > 0.5
        )
        SLOW_FORWARD = commands2.button.Button(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.RT) > 0.5
        )
        X_MODE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.X
        )

        AUTO_ROUTE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.RB
        )

    class Claw:
        OPEN_CLAW_DRIVER = commands2.button.JoystickButton(
            Controllers.DRIVER_CONTROLLER, controllerDRIVER.A
        )

        OPEN_CLAW_OPERATOR = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.A
        )

        DROP_CLAW = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.B
        )

        RUN_CLAW_UP = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 0
        )

        RUN_CLAW_DOWN = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 180
        )

    class Targeting:
        TARGETING_PICKUP = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.LT)
            > 0.5
        )

        TARGETING_DOUBLE_STATION = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.LB
        )

        TARGETING_LOW = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.START
        )

        TARGETING_MIDDLE = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.RT)
            > 0.5
        )

        TARGETING_HIGH = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.RB
        )

        TARGETING_CUBE_INTAKE = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.X
        )

        TARGETING_EJECT = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.Y
        )

        ZERO_ARM = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.SELECT
        )
