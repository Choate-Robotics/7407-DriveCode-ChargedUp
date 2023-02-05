import json
import os
import inspect
import math
from wpimath.geometry import Pose2d, Translation2d
from autonomous.utils.trajectory import CustomTrajectory

def generate_trajectories(configs: dict):
    folder_path = os.path.dirname(inspect.stack()[1].filename) + "/trajectories"

    trajectories = os.listdir(folder_path)

    output_trajectories = []

    for trajectory in trajectories:
        file_path = os.path.join(folder_path, trajectory)

        with open(file_path, "r") as file:
            file_contents = file.read()

        waypoints = json.loads(file_contents)["waypoints"]

        start = waypoints[0]
        end = waypoints[len(waypoints) - 1]

        start_pose = Pose2d(
            start["anchorPoint"]["x"], 
            start["anchorPoint"]["y"], 
            math.radians(start["holonomicAngle"])
        )

        end_pose = Pose2d(
            end["anchorPoint"]["x"], 
            end["anchorPoint"]["y"], 
            math.radians(end["holonomicAngle"])
        )

        interior_waypoints = []

        for waypoint in waypoints[1:-1]:
            coord = waypoint["anchorPoint"]
            interior_waypoints.append(Translation2d(coord["x"], coord["y"]))

        output_trajectories.append(CustomTrajectory(
            start_pose,
            interior_waypoints,
            end_pose,
            configs[trajectory.split(".")[0]][0], # max vel
            configs[trajectory.split(".")[0]][1] # max accel
        ))
        
    return output_trajectories