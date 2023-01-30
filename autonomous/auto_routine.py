from dataclasses import dataclass
from subsystem.drivetrain import Drivetrain

import commands2
from commands2 import CommandBase
from wpimath.geometry import Pose2d

from robot_systems import Robot
import command


@dataclass
class AutoRoutine:
    initial_robot_pose: Pose2d
    command: CommandBase
    subsystem: Drivetrain = Robot.drivetrain

    def run(self):
        self.subsystem.n_front_left.zero()
        self.subsystem.n_front_right.zero()
        self.subsystem.n_back_left.zero()
        self.subsystem.n_back_right.zero()
        self.subsystem.n_front_left.m_move.set_sensor_position(0)
        self.subsystem.n_front_right.m_move.set_sensor_position(0)
        self.subsystem.n_back_left.m_move.set_sensor_position(0)
        self.subsystem.n_back_right.m_move.set_sensor_position(0)
        self.subsystem.gyro.reset_angle(self.initial_robot_pose.rotation().degrees())

        self.subsystem.odometry.resetPosition(
            self.initial_robot_pose.rotation(),
            self.initial_robot_pose,
            self.subsystem.n_front_left.get_node_position(),
            self.subsystem.n_front_right.get_node_position(),
            self.subsystem.n_back_left.get_node_position(),
            self.subsystem.n_back_right.get_node_position()
        )

        commands2.CommandScheduler.getInstance().schedule(self.command)
