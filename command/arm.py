from robotpy_toolkit_7407.command import SubsystemCommand, T
import math

from robotpy_toolkit_7407.command import SubsystemCommand, T
from wpilib import SmartDashboard
from wpimath.geometry import Pose3d, Pose2d, Rotation3d

import constants
from robot_systems import Robot, Sensors
from subsystem import Arm
from oi.keymap import Keymap


class ArmPose(SubsystemCommand[Arm]):
    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.subsystem.update_pose()

    def IsFinished(self) -> bool:
        pass

    def end(self, interrupted: bool) -> None:
        pass


class SetArmPositionRobot(SubsystemCommand[Arm]):

    def __init__(self, subsystem: T, target: Pose3d):
        super().__init__(subsystem)
        self.target: Pose3d = target
        self.subsystem = subsystem
        self.position = self.subsystem.get_pose()
        self.x_distance = self.target.X - self.position.X
        self.y_distance = self.target.Y - self.position.Y
        # gets the distance
        self.distance = math.sqrt(
            (self.x_distance ** 2) + (self.y_distance ** 2))
        self.length = abs(self.distance)
        # gets the angle
        self.angle = math.radians(
            90) - (math.atan2(self.y_distance, self.x_distance))
        
    def __init__(self, subsystem: T, length: float, angle: float):
        self.angle = angle
        self.length = length

    def initialize(self):
        # add from pose of robot the height to get arm height
        self.subsystem.disable_brake()
        if not self.subsystem.set_length(self.length):
            print("Reached Boundry Limits")
        if not self.subsystem.set_rotation(self.angle):
            print("Reached Soft Limits")

    def execute(self):
        self.update_pose()

    def isFinished(self):
        return self.subsystem.is_at_position(self.position)

    def end(self, interrupted):
        if not interrupted:
            self.subsystem.enable_brake()
            self.subsystem.stop()



class PauseMovement(SubsystemCommand[Arm]):

    def initialize(self) -> None:
        self.position = self.subsystem.get_pose()
        self.subsystem.enable_brake()

    def execute(self) -> None:
        self.subsystem.stop()

    def isFinished(self) -> bool:
        pass

    def end(self, interrupted: bool) -> None:
        if not interrupted:
            self.subsystem.enable_brake()
            self.subsystem.stop()


class SetArmPositionField(SubsystemCommand[Arm]):
    """
    Sets the arm to a position in the field or relative to the april tag id

    if no april tag id is given, the position is relative to the field

    Args:
        SubsystemCommand (Elevator): Elevator Subsystem
        Target (Pose3d): Target Position in the field or relative to the april tag id
        april_tag_id (int, optional): April Tag ID. Defaults to None.
    """

    def __init__(self, subsystem: T, target: Pose3d, april_tag_id: int = None):
        super().__init__(subsystem)
        self.target: Pose3d = target
        self.april_tag_id = april_tag_id

    def initialize(self):
        if self.april_tag_id is not None:
            self.tag = constants.kApriltagPositionDict[self.april_tag_id]
            self.target.transformBy(self.tag)
        self.subsystem.disable_brake()

    def execute(self):
        self.robot_pose: Pose2d = Sensors.odometry.get_robot_pose()
        Robot.claw_rotation = 0
        self.rotation = Rotation3d(
            0, Robot.claw.get_rotation, self.robot_pose.rotation)
        self.position = Pose3d(self.robot_pose.translation(), self.rotation)
        # add from pose of robot the height to get arm height
        self.slope = (self.target.Y - self.position.Y) / (self.target.X - self.position.X)
        self.length = abs(self.slope)
        self.x_distance = self.target.X - self.position.X
        self.y_distance = self.target.Y - self.position.Y
        self.angle = math.radians(
            90) - (math.atan2(self.y_distance, self.x_distance))

        if not self.subsystem.set_length(self.length):
            print("Reached Boundry Limits")
        if not self.subsystem.set_rotation(self.angle):
            print("Reached Soft Limits")
        self.subsystem.update_pose()

    def isFinished(self):
        pass

    def end(self, interrupted):
        if not interrupted:
            self.subsystem.enable_brake()
            self.subsystem.stop()


class ZeroArm(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm):
        super().__init__(subsystem)
        self.subsystem = subsystem

    def initialize(self):
        self.subsystem.disable_brake()
        # self.subsystem.motor_extend.set_target_position(constants.elevator_initial_height)
        # self.subsystem.left_rotation_motor.set_target_position(constants.elevator_initial_roatation)
        self.subsystem.zero_elevator_rotation()
        self.subsystem.zero_wrist()
        self.subsystem.motor_extend.set_sensor_position(0)
    def execute(self):
        # pass
        self.subsystem.zero_elevator_length()

    def isFinished(self):
        #return self.subsystem.elevator_bottom_sensor == False and self.subsystem.main_rotation_motor.get_sensor_position() == 0
        return round(self.subsystem.main_rotation_motor.get_sensor_position(), 2) == 0 and round(self.subsystem.wrist.get_sensor_position(), 2) == 0 

    def end(self, interrupted):
        if not interrupted:
            self.subsystem.enable_brake()
            self.subsystem.stop()
        self.subsystem.set_pose(constants.zero_pose)


class SetAngle(SubsystemCommand[Arm]):
    def __init__(self, subsystem: T, radians):
        super().__init__(subsystem)
        self.radians = radians

    def initialize(self):
        self.subsystem.disable_brake()
        self.subsystem.set_rotation(self.radians)

    def execute(self):
        self.subsystem.update_pose()

    def isFinished(self):
        return self.subsystem.get_rotation() == self.radians

    def end(self, interrupted):
        if not interrupted:
            self.subsystem.enable_brake()
            self.subsystem.stop()


class SetLength(SubsystemCommand[Arm]):
    def __init__(self, subsystem: T, length):
        super().__init__(subsystem, length)
        self.length = length

    def initialize(self) -> None:
        self.subsystem.set_length(self.length)

    def execute(self) -> None:
        self.subsystem.update_pose()

    def isFinished(self) -> bool:
        return self.subsystem.is_at_length(self.length)

    def end(self, interrupted: bool) -> None:
        if not interrupted:
            self.subsystem.stop()


class PrintArmPoseTerminal(SubsystemCommand[Arm]):
    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        print(self.subsystem.get_pose())
        self.subsystem.update_pose()

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        pass

class manualMovement(SubsystemCommand[Arm]):
    def __init__(self, subsystem: Arm):
        super().__init__(subsystem)
    
    def initialize(self) -> None:
        # self.subsystem.motor_extend.set_sensor_position(-22)
        # self.subsystem.motor_extend.set_target_position(0)
        pass
    
    def execute(self) -> None:
        rotate: float
        if abs(Keymap.Arm.ELEVATOR_ROTATION_AXIS.value) < .03:
            rotate = 0
        else:
            rotate += Keymap.Arm.ELEVATOR_ROTATION_AXIS.value
        self.subsystem.set_rotation(rotate * (2 * math.pi))
        claw_rotate: float
        if abs(Keymap.Arm.CLAW_ROTATION_AXIS.value) < .03:
            claw_rotate = 0
        else:
            claw_rotate += Keymap.Arm.CLAW_ROTATION_AXIS.value
        self.subsystem.set_angle_wrist(claw_rotate * (2 * math.pi))
        if abs(Keymap.Arm.ELEVATOR_EXTENSION_AXIS.value) < .03:
            extend = 0
        else:
            extend += Keymap.Arm.ELEVATOR_EXTENSION_AXIS.value
        self.subsystem.set_length(extend / constants.max_elevator_height)
        self.subsystem.update_pose()
        #print(Keymap.Arm.ELEVATOR_ROTATION_AXIS.value)
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool):
        pass
    
class PrintArmPoseDashboard(SubsystemCommand[Arm]):
    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        SmartDashboard.putData("Arm Pose", self.subsystem.get_pose())
        self.subsystem.update_pose()

    def isFinished(self) -> bool:
        pass

    def end(self, interrupted: bool) -> None:
        pass


class ArmAssistedRobotStabalizer(SubsystemCommand[Arm]):

    def __init__(self, subsystem: T):
        super().__init__(subsystem)

    def initialize(self) -> None:
        self.subsystem.disable_brake()

    def execute(self) -> None:
        self.gyro_roll: float
        self.elevator_rotation: float = self.gyro_roll * constants.stabilizer_magnitude
        self.subsystem.set_rotation(self.elevator_rotation)

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        if not interrupted:
            self.subsystem.enable_brake()
            self.subsystem.stop()


class HardStop(SubsystemCommand[Arm]):

    def initialize(self) -> None:
        self.position = self.subsystem.get_pose()
        self.subsystem.enable_brake()

    def execute(self) -> None:
        self.subsystem.stop()

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        if not interrupted:
            self.subsystem.enable_brake()
            self.subsystem.stop()
            
class EngageClaw(SubsystemCommand[Arm]):

    def initialize(self) -> None:
        self.subsystem.engage_claw()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        pass
    

class DisengageClaw(SubsystemCommand[Arm]):

    def initialize(self) -> None:
        self.subsystem.disengage_claw()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        pass
    
    
class CubeIntake(SubsystemCommand[Arm]):

    def initialize(self) -> None:
        self.subsystem.set_rotation(math.radians(90))
        self.subsystem.set_angle_wrist(math.radians(90))
        self.subsystem.engage_claw()
    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.subsystem.set_rotation(math.radians(0))
        self.subsystem.set_angle_wrist(math.radians(0))
