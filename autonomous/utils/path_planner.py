import json
import os
import tempfile

from wpimath.trajectory import TrajectoryUtil

auto_folder_path = "./autonomous/routines/"


# combines trajectory files into one auto_routine.json file
def combine_trajectory_json(routines):
    for routine in routines:
        output = {}
        folder_path = os.path.join(auto_folder_path, routine)

        path_number = 1

        for file in os.listdir(folder_path):
            # ignore auto_routine.json when finding trajectories
            if file == "auto_routine.json":
                continue

            file_path = os.path.join(folder_path, file)

            with open(file_path, "r") as json_file:
                path = json.load(json_file)
                output["trajectory_" + str(path_number)] = path

            path_number += 1

        with open(
            auto_folder_path + routine + "/auto_routine.json", "w"
        ) as auto_routine:
            json.dump(output, auto_routine, indent=4)


# converts json to trajectories
def get_trajectories(routine):
    trajectories = {}

    with open("./autonomous/routines/" + routine + "/auto_routine.json", "r") as file:
        auto_routine = json.load(file)

    for key, value in auto_routine.items():
        with tempfile.NamedTemporaryFile(mode="w", delete=False) as temp:
            temp.write(json.dumps(value))
            trajectories[key] = TrajectoryUtil.fromPathweaverJson(temp.name)

    return trajectories
