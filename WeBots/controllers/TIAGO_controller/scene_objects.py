# scene_objects.py


# Individual references (optional)
puzzle_outline = None
viewpoint = None
current_object = None

# Red pieces
small_triangle_red = None
para_1_red = None
para_2_red = None
big_triangle_1_red = None
big_triangle_2_red = None
square_red = None

# Blue pieces
small_triangle_blue = None
para_1_blue = None
para_2_blue = None
big_triangle_1_blue = None
big_triangle_2_blue = None
square_blue = None

# target Positions
small_triangle_target_pos = [0.62, -0.67, 0.52]
para_2_target_pos = [0.225, -0.56, 0.52]
para_1_target_pos = [0.225, -0.79, 0.52]
big_triangle_1_target_pos = [0.535, -0.64, 0.52]
big_triangle_2_target_pos = [0.461796, -0.717877, 0.52]
square_target_pos = [0.301374, -0.677455, 0.52]

# PROTO instance itself (TangramPiecesTarget)
tangram_target_proto = None

# Dictionaries for grouping by color
red_objects = {}
blue_objects = {}
all_pieces = {}

def get_node_or_warn(robot, def_name: str):
    """
    Fetch a Webots node by DEF name in the global scene tree.
    Warn if missing.
    """
    node = robot.getFromDef(def_name)
    if node is None:
        print(f"[scene_objects] ERROR: Could not find node with DEF '{def_name}'")
    return node

def get_proto_node_or_warn(proto_node, def_name: str):
    """
    Fetch a node defined INSIDE A PROTO by its DEF name, using getFromProtoDef.
    Warn if missing.
    """
    if proto_node is None:
        print(
            f"[scene_objects] ERROR: Cannot search for internal DEF '{def_name}' "
            f"because proto_node is None"
        )
        return None

    node = proto_node.getFromProtoDef(def_name)
    if node is None:
        print(
            f"[scene_objects] ERROR: Could not find internal PROTO node with DEF '{def_name}'"
        )
    return node

def init_scene_objects(robot):
    """
    Initializes scene objects and automatically categorizes them
    into red_objects, blue_objects, and target_objects.
    """

    global puzzle_outline, viewpoint
    global small_triangle_red, para_1_red, para_2_red
    global big_triangle_1_red, big_triangle_2_red, square_red
    global small_triangle_blue, para_1_blue, para_2_blue
    global big_triangle_1_blue, big_triangle_2_blue, square_blue
    global tangram_target_proto
    global red_objects, blue_objects, all_pieces

    # map scene-level DEF names to variable names
    world_def_map = {
        "puzzle_outline": "PUZZLE_OUTLINE",
        "viewpoint": "USER_CAMERA",
        # Red pieces
        "small_triangle_red": "SMALL_TRIANGLE_RED",
        "para_1_red": "PARALELLOGRAM_1_RED",
        "para_2_red": "PARALELLOGRAM_2_RED",
        "big_triangle_1_red": "BIG_TRIANGLE_1_RED",
        "big_triangle_2_red": "BIG_TRIANGLE_2_RED",
        "square_red": "SQUARE_RED",
        # Blue pieces
        "small_triangle_blue": "SMALL_TRIANGLE_BLUE",
        "para_1_blue": "PARALELLOGRAM_2_BLUE",
        "para_2_blue": "PARALELLOGRAM_1_BLUE",
        "big_triangle_1_blue": "BIG_TRIANGLE_1_BLUE",
        "big_triangle_2_blue": "BIG_TRIANGLE_2_BLUE",
        "square_blue": "SQUARE_BLUE",
    }

    # Fetch all world-level nodes and auto-group red/blue
    for attr_name, def_name in world_def_map.items():
        node = get_node_or_warn(robot, def_name)
        globals()[attr_name] = node  # set each individual node

        # categorize into red/blue based on name
        lname = attr_name.lower()
        if "red" in lname:
            red_objects[attr_name] = node
            all_pieces[attr_name] = node
        elif "blue" in lname:
            blue_objects[attr_name] = node
            all_pieces[attr_name] = node
