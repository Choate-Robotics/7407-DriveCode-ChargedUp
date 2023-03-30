from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = True

base_initial_coords: coord = (1.5, 0.57, 0)

base_path_1: path = (
    base_initial_coords,
    [],
    (6.55, 1, 0),
)

base_path_2: path = (
    base_path_1[2],
    [(3.92, 0.82)],
    (1.87, 1.12, 0),
)

base_path_3: path = (
    base_path_2[2],
    [(3.35, 0.66), (4.8, 0.62), (5.6, 0.7)],
    (6.75, 2.1, 0.91),
)

base_path_4: path = (
    base_path_3[2],
    [(5.6, 0.7), (4.8, 0.62), (3.92, 0.66)],
    (1.76, 1.08, 0),
)
