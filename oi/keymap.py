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
            Joysticks.joysticks[Controllers.DRIVER], controllerDRIVER.A
        )
        REZERO_MOTORS = commands2.button.JoystickButton(
            Joysticks.joysticks[Controllers.DRIVER], controllerOPERATOR.B
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
    
    class Scoring:
        CONFIRM = commands2.button.Button( # 0, Right Thumb/Stick
            lambda: Controllers.NUMPAD_CONTROLLER.getRawAxis(10)
            > 0.8
        )

        DELETE = commands2.button.Button( # Dot, Left Trigger
            lambda: Controllers.NUMPAD_CONTROLLER.getRawAxis(controllerNUMPAD.LT)
            > 0.8
        )

        LEFT_GRID = commands2.button.Button( # NUM, Axis X Min (Left)
            lambda: Controllers.NUMPAD_CONTROLLER.getRawAxis(controllerNUMPAD.L_JOY[0])
            < -0.8
        )

        MIDDLE_GRID = commands2.button.Button( # /, Axis Y Max (Left)
            lambda: Controllers.NUMPAD_CONTROLLER.getRawAxis(controllerNUMPAD.L_JOY[1])
            > 0.8
        )

        RIGHT_GRID = commands2.button.Button( # *, Axis Y Min (Left)
            lambda: Controllers.NUMPAD_CONTROLLER.getRawAxis(controllerNUMPAD.L_JOY[1])
            < -0.8
        )

        TOP_LEFT = commands2.button.JoystickButton( # 7, Back
            Joysticks.joysticks[Controllers.NUMPAD],
            controllerNUMPAD.SELECT
        )

        TOP_MIDDLE = commands2.button.JoystickButton( # 8, Start
            Joysticks.joysticks[Controllers.NUMPAD],
            controllerNUMPAD.START
        )

        TOP_RIGHT = commands2.button.Button( # 9, Left Thumb/Stick
            lambda: Controllers.NUMPAD_CONTROLLER.getRawAxis(9)
            > 0.8
        )

        MIDDLE_LEFT = commands2.button.JoystickButton( # 4, Y
            Joysticks.joysticks[Controllers.NUMPAD], controllerNUMPAD.Y
        )
        
        MIDDLE_MIDDLE = commands2.button.Button( # 5, Left Bumper
            lambda: Controllers.NUMPAD_CONTROLLER.getRawAxis(controllerNUMPAD.LB)
            > 0.8
        )

        MIDDLE_RIGHT = commands2.button.Button( # 6, Right Bumper
            lambda: Controllers.NUMPAD_CONTROLLER.getRawAxis(controllerNUMPAD.RB)
            > 0.8
        )

        BOTTOM_LEFT = commands2.button.Button( # 1, A
            Joysticks.joysticks[Controllers.NUMPAD], 
            controllerNUMPAD.A
        )

        BOTTOM_MIDDLE = commands2.button.Button( # 2, B
            Joysticks.joysticks[Controllers.NUMPAD], 
            controllerNUMPAD.B
        )

        BOTTOM_RIGHT = commands2.button.Button( # 3, X
            Joysticks.joysticks[Controllers.NUMPAD], 
            controllerNUMPAD.X
        )