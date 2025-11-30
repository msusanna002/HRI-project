# scene_objects.py

# Individual references (optional)
puzzle_outline = None
viewpoint = None

# # Viewpoint fields
# vp_pos_field = None
# vp_rot_field = None

# Red objects
square_red = None
para_1_red = None

# Dictionaries for grouping by color
red_objects = {}
blue_objects = {}

def get_node_or_warn(robot, def_name):
    """Fetch a Webots node by DEF name, warn if missing."""
    node = robot.getFromDef(def_name)
    if node is None:
        print(f"[scene_objects] ERROR: Could not find node with DEF '{def_name}'")
    return node


def init_scene_objects(robot):
    """
    Initializes scene objects and automatically categorizes them
    into red_objects or blue_objects if their variable name contains
    'red' or 'blue'.
    """
    global square_red, para_1_red, puzzle_outline
    global red_objects, blue_objects

    # Define all objects here once
    names = {
        "puzzle_outline": "PUZZLE_OUTLINE",
        "square_red": "SQUARE_RED",
        "para_1_red": "PARALELLOGRAM_2_RED",
        "viewpoint": "USER_CAMERA"
    }

    red_objects = {}
    blue_objects = {}

    # Initialize objects and auto-sort by name
    for attr_name, def_name in names.items():
        node = get_node_or_warn(robot, def_name)

        # Set global variable (square_red, para_1_red, etc.)
        globals()[attr_name] = node

        # Auto-categorize: checks name like "square_red"
        lname = attr_name.lower()
        if "red" in lname:
            red_objects[attr_name] = node
        elif "blue" in lname:
            blue_objects[attr_name] = node
