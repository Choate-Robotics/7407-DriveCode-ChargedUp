from commands2 import InstantCommand, ParallelCommandGroup, ConditionalCommand, SequentialCommandGroup, WaitCommand
from robotpy_toolkit_7407.command import SubsystemCommand, T
from robotpy_toolkit_7407.utils.units import inch, meters, rev
from wpimath.geometry import Pose3d, Translation3d, Pose2d, Rotation3d
import math
import constants
from robot_systems import Robot, Sensors
from subsystem import Elevator

class SetArmPositionRobot(SubsystemCommand[Elevator]):

    def __init__(self, subsystem: T, target: Pose3d):
        super().__init__(subsystem)
        self.target: Pose3d = target
        self.subsystem = subsystem
        self.position = self.subsystem.get_pose()
        self.x_distance = self.target.X - self.position.X
        self.y_distance = self.target.Y - self.position.Y
        self.slope = (self.target.Y - self.position.Y) / (self.target.X - self.position.X)
        self.length = abs(self.slope)
        #gets the angle
        self.angle = math.radians(90) - (math.atan2(self.y_distance, self.x_distance))

    def initialize(self):
        #add from pose of robot the height to get arm height
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
        if(interrupted):
            self.subsystem.stop()
        self.subsystem.enable_brake()

class pauseMovement(SubsystemCommand[Elevator]):

    def initialize(self) -> None:
        self.position = self.subsystem.get_pose()
        self.subsystem.enable_brake()
        
    def execute(self) -> None:
        self.subsystem.stop()

    def isFinished(self) -> bool:
        pass

    def end(self, interrupted: bool) -> None:
        self.subsystem.disable_brake()

class SetArmPositionField(SubsystemCommand[Elevator]):

    def __init__(self, subsystem: T, target: Pose3d):
        super().__init__(subsystem)
        self.target = target

    def initialize(self):
        pass

    def execute(self):
        self.robot_pose: Pose2d = Sensors.odometry.get_robot_pose()
        Robot.claw_rotation = 0
        self.rotation = Rotation3d(0, Robot.claw.get_rotation, self.robot_pose.rotation)
        self.position = Pose3d(self.robot_pose.translation(), self.rotation)
        #add from pose of robot the height to get arm height
        self.slope = (self.target.Y - self.position.Y) / (self.target.X - self.position.X)
        self.length = abs(self.slope)
        self.x_distance = self.target.X - self.position.X
        self.y_distance = self.target.Y - self.position.Y
        self.angle = math.radians(90) - (math.atan2(self.y_distance, self.x_distance))
        
        if not self.subsystem.set_length(self.length):
            print("Reached Boundry Limits")
        if not self.subsystem.set_rotation(self.angle):
            print("Reached Soft Limits")
        self.subsystem.update_pose()
    def isFinished(self):
        pass

    def end(self, interrupted):
        self.subsystem.stop()

class ZeroArm(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: T):
        super().__init__(subsystem)
    
    def initialize(self):
        self.subsystem.disable_brake()
        # self.subsystem.motor_extend.set_target_position(constants.elevator_initial_height)
        # self.subsystem.left_rotation_motor.set_target_position(constants.elevator_initial_roatation)
        self.subsystem.zero_elevator_rotation()
    def execute(self):
        self.subsystem.zero_elevator_length()

    def isFinished(self):
        return self.extend_sensor.get_value() == False and self.turn_sensor.get_value() == 0

    def end(self, interrupted):
        self.subsystem.enable_brake
        self.subsystem.zero_pose()

class SetRotation(SubsystemCommand[Elevator]):
    def __init__(self, subsystem: T, rotations):
        super().__init__(subsystem)
        self.rotations = rotations
    def initialize(self):
        self.subsystem.disable_brake()
        self.rotation = self.subsystem.get_rotation()
        self.target = self.rotation + self.target
        self.subsystem.set_rotation(self.rotations)
    
    def execute(self):
        self.subsystem.update_pose()

    def isFinished(self):
        return self.subsystem.get_rotation() == self.target

    def end(self, interrupted):
        self.subsystem.set_rotation(self.rotation)

class SetLength(SubsystemCommand[Elevator]):
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
        if interrupted:
            self.subsystem.stop()
            
class printArmPose(SubsystemCommand[Elevator]):
    def initialize(self) -> None:
        pass
    def execute(self) -> None:
        print(self.subsystem.get_pose())
        self.subsystem.update_pose()
        
class armAssistedRobotStabalizer(SubsystemCommand[Elevator]):
    
    def __init__(self, subsystem: T):
        super().__init__(subsystem)
    
    def initialize(self) -> None:
        pass
    def execute(self) -> None:
        self.gyro = Robot.drivetrain.gyro.get_rotation()
        
        
        
    def isFinished(self) -> bool:
        pass
    def end(self, interrupted: bool) -> None:
        pass
    
