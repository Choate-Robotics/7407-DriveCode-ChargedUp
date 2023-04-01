import config
from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = False

base_initial_coords: coord = (1.5, config.field_width - 0.57, 0)

base_path_1: path = (
    base_initial_coords,
    [],
    (6.55, config.field_width - 1, 0),
)

base_path_2: path = (
    base_path_1[2],
    [(3.92, config.field_width - 0.82)],
    (1.87, config.field_width - 1.12, 0),
)

base_path_3: path = (
    base_path_2[2],
    [
        (3.35, config.field_width - 0.85),
        (4.8, config.field_width - 0.8),
        (5.6, config.field_width - 0.82),
    ],
    (6.75, config.field_width - 2.2, 1),
)

base_path_4: path = (
    base_path_3[2],
    [
        (5.6, config.field_width - 0.7),
        (4.8, config.field_width - 0.62),
        (3.92, config.field_width - 0.66),
    ],
    (1.76, config.field_width - 1.08, 0),
)
