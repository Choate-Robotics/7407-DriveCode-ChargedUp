from units.SI import meters, radians

coord = (meters, meters, radians)
waypoints = [(meters, meters)]
path = (coord, waypoints, coord)

blue_team = True

initial: coord = (1.55, 0.51, 0)  # .65

go_get_first_cube: path = (
    initial,
    [],
    (initial[0] + 5.38, initial[1] + 0.43, 0),
)

come_back_with_first_cube: path = (
    go_get_first_cube[2],
    [(initial[0] + 2.42, initial[1] + 0.25)],
    (initial[0] + 0.05, initial[1] + 0.55, 0),
)

go_get_second_cube: path = (
    come_back_with_first_cube[2],
    [
        (initial[0] + 1.85, initial[1] + 0.27),
        (initial[0] + 3.3, initial[1] + 0.24),
        (initial[0] + 3.9, initial[1] + 0.68),
        (initial[0] + 4.3, initial[1] + 2.33),
    ],
    (initial[0] + 5.5, initial[1] + 2.36, 0),
)

come_back_with_second_cube: path = (
    go_get_second_cube[2],
    [
        (initial[0] + 4.4, initial[1] + 0.9),
        (initial[0] + 3.3, initial[1] + 0.2),
        (initial[0] + 1.85, initial[1] + 0.27),
    ],
    (initial[0] + 0, initial[1] + 0.55, 0),
)
