from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

base_initial_coords: coord = (1.5, 0.57, 0)

base_path_1: path = (
    (base_initial_coords[0], base_initial_coords[1], base_initial_coords[2]),
    (6.55, 1, 0),
)

base_path_2: path = (
    base_path_1[1],
    (1.87, 1.12, 0),
)

base_path_3: path = (
    base_path_2[1],
    (6.6, 2.1, 0.91),
)

base_path_4: path = (
    base_path_3[1],
    (1.76, 1.12, 0),
)
