import config
from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = True

base_initial_coords: coord = (1.5, config.scoring_width - 0.57, 0)

base_path_1: path = (
    base_initial_coords,
    [],
    (6.55, config.scoring_width - 1.06, 0),
)

base_path_2: path = (
    base_path_1[2],
    [(3.92, config.scoring_width - 0.82)],
    (1.87, config.scoring_width - 1.07, 0),
)

base_path_3: path = (
    base_path_2[2],
    [],
    (2.1, config.scoring_width - 2.1, 0),
)
