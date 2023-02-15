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
        REZERO_MOTORS = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerOPERATOR.X
        )

    class Arm:
        ELEVATOR_ROTATION_AXIS = JoystickAxis(
            Controllers.OPERATOR, controllerOPERATOR.L_JOY[0]
        )
        ELEVATOR_EXTENSION_AXIS = JoystickAxis(
            Controllers.OPERATOR, controllerOPERATOR.L_JOY[1]
        )
        CLAW_ROTATION_AXIS = JoystickAxis(
            Controllers.OPERATOR, controllerOPERATOR.R_JOY[0]
        )
        REZERO_ELEVATOR = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.A
        )
        EXTEND_ELEVATOR_MAX = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.B
        )
        RETRACT_ELEVATOR_MIN = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.X
        )

        ARM_BRAKE = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.OPERATOR], controllerOPERATOR.Y
        )

    class Intake:
        INTAKE_ENABLE = commands2.button.Button(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerOPERATOR.LT)
            > 0.8
        )

        

    class Claw:
        ENGAGE_CLAW = commands2.button.Button(
            lambda: Controllers.DRIVER_CONTROLLER.getRawAxis(-controllerOPERATOR.RT)
            > 0.8
        )
        
        ENGAGE_DOUBLE_STATION = Joysticks.joysticks[Controllers.DRIVER], controllerOPERATOR.X
        
