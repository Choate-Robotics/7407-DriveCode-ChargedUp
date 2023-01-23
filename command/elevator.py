from commands2 import InstantCommand, ParallelCommandGroup, ConditionalCommand, SequentialCommandGroup, WaitCommand
from robotpy_toolkit_7407.command import SubsystemCommand, T
from robotpy_toolkit_7407.utils.units import inch, meters, rev
from wpimath.geometry import Pose3d, Translation3d, Pose2d

import constants
from robot_systems import Robot
from subsystem import Elevator

class setPosition(SubsystemCommand[Elevator]):

    def __init__(self, subsystem: T, target: Pose3d):
        super().__init__(subsystem)
        self.target = target

    def initialize(self):
        self.subsystem.disable_brake()
        #get pose of robot
        self.robot_pose: Pose2d
        self.position = Pose3d(self.robot_pose.X, self.robot_pose.Y, constants.arm_height, self.robot_pose.rotation)
        #add from pose of robot the height to get arm height
        

        #LETS TALK TO WANG ABOUT THIS

    def execute(self):
        pass


    def isFinished(self):
        return self.subsystem.is_at_position(self.position)

    def end(self, interrupted):
        self.subsystem.set_position(self.position)
