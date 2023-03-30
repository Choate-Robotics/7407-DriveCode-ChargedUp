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
    (6.55, config.scoring_width - 1, 0),
)

base_path_2: path = (
    base_path_1[2],
    [(3.92, config.scoring_width - 0.82)],
    (1.87, config.scoring_width - 1.12, 0),
)

base_path_3: path = (
    base_path_2[2],
    [
        (3.35, config.scoring_width - 0.57),
        (4.8, config.scoring_width - 0.53),
        (5.6, config.scoring_width - 0.68),
    ],
    (6.75, config.scoring_width - 2.1, 0.91),
)

base_path_4: path = (
    base_path_3[2],
    [
        (5.6, config.scoring_width - 0.68),
        (4.8, config.scoring_width - 0.53),
        (3.92, config.scoring_width - 0.57),
    ],
    (1.76, config.scoring_width - 1.08, 0),
)
