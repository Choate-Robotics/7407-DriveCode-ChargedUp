import commands2.button
import wpilib
from robotpy_toolkit_7407.oi import (
    JoystickAxis,
    XBoxController,
)
from robotpy_toolkit_7407.oi.joysticks import Joysticks

controllerDRIVER = XBoxController
controllerOPERATOR = XBoxController
controllerNUMPAD = XBoxController


class Controllers:
    DRIVER = 0
    OPERATOR = 1
    NUMPAD = 2

    DRIVER_CONTROLLER = wpilib.Joystick(0)
    OPERATOR_CONTROLLER = wpilib.Joystick(1)
    NUMPAD_CONTROLLER = wpilib.Joystick(2)


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
        # SLOW_REVERSE = commands2.button.Button(
        #     lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.LT) > 0.5
        # )
        SLOW_FORWARD = commands2.button.Button(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.RT) > 0.5
        )
        X_MODE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.X
        )

        AUTO_ROUTE = commands2.button.Button(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerDRIVER.LT) > 0.5
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

        TARGETING_CUBE_INTAKE_CLAW = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.X
        )

        TARGETING_CUBE_INTAKE = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.Y
        )

        TARGETING_EJECT_INTAKE = commands2.button.JoystickButton(
            Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.SELECT
        )

        # ZERO_ARM = commands2.button.JoystickButton(
        #     Controllers.OPERATOR_CONTROLLER, controllerOPERATOR.SELECT
        # )

    class Climber:
        DEPLOY = commands2.button.JoystickButton(
            Controllers.DRIVER_CONTROLLER, controllerDRIVER.SELECT
        )

        RESET = commands2.button.JoystickButton(
            Controllers.DRIVER_CONTROLLER, controllerDRIVER.START
        )

        UNCLIMB = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 90
        )

        CLIMB = commands2.button.Button(
            lambda: Controllers.OPERATOR_CONTROLLER.getPOV() == 270
        )

    class Debug:
        INVERT_ELEVATOR = commands2.button.Button(
            lambda: Controllers.DRIVER_CONTROLLER.getPOV() == 180
        )

    class Scoring:
        ONE = commands2.button.JoystickButton(
            Controllers.NUMPAD_CONTROLLER, controllerNUMPAD.A
        )

        TWO = commands2.button.JoystickButton(
            Controllers.NUMPAD_CONTROLLER, controllerNUMPAD.B
        )

        THREE = commands2.button.JoystickButton(
            Controllers.NUMPAD_CONTROLLER, controllerNUMPAD.X
        )

        FOUR = commands2.button.JoystickButton(
            Controllers.NUMPAD_CONTROLLER, controllerNUMPAD.Y
        )

        FIVE = commands2.button.JoystickButton(
            Controllers.NUMPAD_CONTROLLER, controllerNUMPAD.LB
        )

        SIX = commands2.button.JoystickButton(
            Controllers.NUMPAD_CONTROLLER, controllerNUMPAD.RB
        )

        SEVEN = commands2.button.JoystickButton(
            Controllers.NUMPAD_CONTROLLER, controllerNUMPAD.SELECT
        )

        EIGHT = commands2.button.JoystickButton(
            Controllers.NUMPAD_CONTROLLER, controllerNUMPAD.START
        )

        NINE = commands2.button.JoystickButton(Controllers.NUMPAD_CONTROLLER, 9)

        DEL = commands2.button.JoystickButton(Controllers.NUMPAD_CONTROLLER, 10)

        DEL_DRIVER = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.LB
        )
