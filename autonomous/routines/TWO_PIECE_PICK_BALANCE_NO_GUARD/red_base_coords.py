import config
from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = False

base_initial_coords: coord = (
    1.5,
    config.field_width - (config.scoring_width - 0.57),
    0,
)

base_path_1: path = (
    base_initial_coords,
    [],
    (6.55, config.field_width - (config.scoring_width - 1.06), 0),
)

base_path_2: path = (
    base_path_1[2],
    [(3.92, config.field_width - (config.scoring_width - 0.82))],
    (1.87, config.field_width - (config.scoring_width - 1.07), 0),
)

base_path_3: path = (
    base_path_2[2],
    [
        (3.35, config.field_width - (config.scoring_width - 0.68)),
        (4, config.field_width - (config.scoring_width - 0.65)),
        (4.8, config.field_width - (config.scoring_width - 0.8)),
        (5.7, config.field_width - (config.scoring_width - 2.1)),
    ],
    (6.75, config.field_width - (config.scoring_width - 2.15), 0),
)
