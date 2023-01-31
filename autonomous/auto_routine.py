import logging
from dataclasses import dataclass

import commands2
from commands2 import CommandBase
from wpimath.geometry import Pose2d

from robot_systems import Robot


@dataclass
class AutoRoutine:
    initial_robot_pose: Pose2d
    command: CommandBase

    def log(self):
        logging.info(Robot.drivetrain.odometry.getPose())

    def run(self):
        """
        Runs the autonomous routine
        """
        Robot.drivetrain.node_positions = (
            Robot.drivetrain.n_front_left.get_node_position(),
            Robot.drivetrain.n_front_right.get_node_position(),
            Robot.drivetrain.n_back_left.get_node_position(),
            Robot.drivetrain.n_back_right.get_node_position()
        )

        Robot.drivetrain.odometry.resetPosition(
            Robot.drivetrain.get_heading(),
            self.initial_robot_pose,
            *Robot.drivetrain.node_positions
        )

        commands2.CommandScheduler.getInstance().schedule(
            self.command
        )
