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
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.A
        )
        RESET_ODOMETRY = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerOPERATOR.X
        )

    class Claw:
        OPEN_CLAW = GRABBER = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.A
        )

    class Targeting:
        TARGETING_PICKUP = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.LT)
            > 0.8
        )

        TARGETING_DOUBLE_STATION = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.LB
        )

        TARGETING_MIDDLE = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getRawAxis(-controllerOPERATOR.RT)
            > 0.8
        )

        TARGETING_HIGH = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.RB
        )
