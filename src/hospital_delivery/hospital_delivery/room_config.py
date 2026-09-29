# Fitted to the lab map (~30 x 40 ft → ~9.1 m x 12.2 m usable floor).
# Map image is 12.2 m x 8.55 m at 0.05 m/px; goals stay inside free space.
ROOM_COORDS = {
    "room1": (4.20, 2.50),
    "room2": (-1.65, -1.19),
    "room3": (-1.65, 1.36),
    "room4": (-4.50, -2.40),
    "room5": (-4.80, 2.50),
    "home": (1.60, 1.26),
}

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
        # vosk often hears "room two" as toronto
        "go to toronto", "toronto",
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
