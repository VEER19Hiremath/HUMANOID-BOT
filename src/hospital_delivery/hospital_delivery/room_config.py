import math

ROOM_COORDS = {
    # maps/area_30x40.yaml: 40 x 30 ft, robot starts in the MIDDLE (home)
    # facing along the 40 ft side (+x); +y is its left. Rooms sit where they
    # were on the old lab map (scripts/make_area_map.py has the walls).
    "home": (0.00, 0.00),
    "room1": (5.00, 3.00),    # right upper side room
    "room2": (-1.50, -2.00),  # lower hall, left of centre
    "room3": (-1.50, 1.50),   # upper hall, left of centre
    "room4": (-5.10, -3.20),  # left lower side room
    "room5": (-5.10, 3.00),   # left upper side room
}


def rooms_from_map(yaml_path):
    """Rooms listed in a map yaml as "# room: name x y" lines, or None.

    Drawn maps (scripts/make_area_map.py) carry their own room positions, so
    switching maps can't leave the robot driving to another map's rooms.
    """
    rooms = {}
    try:
        with open(yaml_path) as f:
            for line in f:
                parts = line.split()
                if len(parts) == 5 and parts[:2] == ['#', 'room:']:
                    rooms[parts[2]] = (float(parts[3]), float(parts[4]))
    except (OSError, ValueError):
        return None
    return rooms or None


def room_heading(name, xy):
    """Heading to arrive at a room with: facing home (the open centre).

    The robot drives forward only, so it leaves a room the way it faces. A
    room reached facing outwards (towards a corner) left no space for the
    turn-around loop and every next trip failed (floor 2026-10-01). Home is
    reached facing +x, the way the robot starts.
    """
    if name == "home":
        return 0.0
    x, y = xy
    return math.atan2(-y, -x)


def goal_yaw(start_xy, goal_xy):
    """Heading for a room goal: the bearing from where the robot is.

    The planner (Hybrid-A*) must reach the goal's exact heading and the base
    can't turn on the spot, so a fixed heading forced turn-arounds inside
    small rooms. Arriving along the straight line never needs one.
    """
    dx, dy = goal_xy[0] - start_xy[0], goal_xy[1] - start_xy[1]
    if math.hypot(dx, dy) < 0.3:
        return 0.0
    return math.atan2(dy, dx)

ROOM_ALIASES = {
    "room1": [
        "room one", "room 1", "room1", "go to room one", "go to room 1",
        "go room one", "go room 1", "go to the one", "go to one",
        "gotta one", "got a one", "got to one", "room one please", "room 1 please",
    ],
    "room2": [
        "room two", "room 2", "room2", "go to room two", "go to room 2",
        "go room two", "go room 2", "go to the two", "go to two",
        "gotta two", "got to two", "to room two", "to room 2",
        # vosk often hears "go to room two" as "go to toronto". A bare
        # "toronto" came from background noise, so it isn't a command.
        "go to toronto",
        "room two please", "room 2 please",
    ],
    "room3": [
        "room three", "room 3", "room3", "go to room three", "go to room 3",
        "go room three", "go room 3", "go to the three", "go to three",
        "gotta three", "room three please", "room 3 please",
    ],
    "room4": [
        "room four", "room 4", "room4", "go to room four", "go to room 4",
        "go room four", "go room 4", "go to the four", "go to four",
        "room four please", "room 4 please",
    ],
    "room5": [
        "room five", "room 5", "room5", "go to room five", "go to room 5",
        "go room five", "go room 5", "go to the five", "go to five",
        "room five please", "room 5 please",
    ],
    "home": [
        "reception", "go home", "return home", "go to home", "go to reception",
        "return to home", "go to start", "go to reception",
    ],
}
