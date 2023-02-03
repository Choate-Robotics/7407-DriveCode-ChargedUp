import os
import json

auto_folder_path = "./autonomous/routines/"

def generate_routine(routines):
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

        with open(auto_folder_path + routine + "/auto_routine.json", "w") as auto_routine:
            json.dump(output, auto_routine, indent=4)