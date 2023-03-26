import config
from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_base_initial_coords: coord = (1.5, 0.57, 0)

blue_base_path_1: path = (
    blue_base_initial_coords,
    [],
    (6.55, 1, 0),
)

blue_base_path_2: path = (
    blue_base_path_1[2],
    [(3.92, 0.82)],
    (1.87, 1.12, 0),
)

blue_base_path_3: path = (
    blue_base_path_2[2],
    [(3.35, 0.57), (4.8, 0.53), (5.6, 0.68)],
    (6.75, 2.1, 0.91),
)

blue_base_path_4: path = (
    blue_base_path_3[2],
    [(5.6, 0.68), (4.8, 0.53), (3.92, 0.57)],
    (1.76, 1.08, 0),
)

red_base_initial_coords: coord = (1.5, config.field_width - 0.57, 0)

red_base_path_1: path = (
    red_base_initial_coords,
    [],
    (6.6, config.field_width - 1, 0),
)

red_base_path_2: path = (
    red_base_path_1[2],
    [(3.92, config.field_width - 0.82)],
    (1.67, config.field_width - 1.12, 0),
)

red_base_path_3: path = (
    red_base_path_2[2],
    [
        (3.35, config.field_width - 0.6),
        (4.8, config.field_width - 0.57),
        (5.6, config.field_width - 0.68),
    ],
    (7, config.field_width - 2.1, -0.91),
)

red_base_path_4: path = (
    red_base_path_3[2],
    [
        (5.6, config.field_width - 0.68),
        (4.8, config.field_width - 0.57),
        (3.92, config.field_width - 0.6),
    ],
    (1.67, config.field_width - 1.08, 0),
)
