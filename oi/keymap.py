from robotpy_toolkit_7407.oi import (
    XBoxController,
    LogitechController,
    JoystickAxis,
    DefaultButton,
)

controllerDRIVER = XBoxController
controllerOPERATOR = XBoxController

class Controllers:
    DRIVER = 0
    OPERATOR = 1


class Keymap:
    class Drivetrain:
        DRIVE_X_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.L_JOY[0])
        DRIVE_Y_AXIS = JoystickAxis(Controllers.OPERATOR, controllerOPERATOR.L_JOY[1])
        DRIVE_ROTATION_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.R_JOY[0])
        DRIVE_Y2_AXIS = JoystickAxis(Controllers.DRIVER, controllerDRIVER.R_JOY[1])
