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
    (6.68, config.field_width - (config.scoring_width - 1), 0),
)

base_path_2: path = (
    base_path_1[2],
    [(3.92, config.field_width - (config.scoring_width - 0.82))],
    (1.65, config.field_width - (config.scoring_width - 1.12), 0),
)

base_path_3: path = (
    base_path_2[2],
    [
        (3.35, config.field_width - (config.scoring_width - 0.67)),
        (4.8, config.field_width - (config.scoring_width - 0.63)),
        (5.6, config.field_width - (config.scoring_width - 0.67)),
    ],
    (6.9, config.field_width - (config.scoring_width - 2.4), 1),
)

base_path_4: path = (
    base_path_3[2],
    [
        (5.6, config.field_width - (config.scoring_width - 0.67)),
        (4.8, config.field_width - (config.scoring_width - 0.63)),
        (3.92, config.field_width - (config.scoring_width - 0.67)),
    ],
    (1.6, config.field_width - (config.scoring_width - 1.08), 0),
)
