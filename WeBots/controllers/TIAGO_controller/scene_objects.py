# scene_objects.py

from controller import Supervisor

# Individual references (optional)
puzzle_outline = None
viewpoint = None

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

# Target pieces (inside TangramPiecesTarget PROTO)
small_triangle_target = None
para_1_target = None
para_2_target = None
big_triangle_1_target = None
big_triangle_2_target = None
square_target = None

# PROTO instance itself (TangramPiecesTarget)
tangram_target_proto = None

# Dictionaries for grouping by color
red_objects = {}
blue_objects = {}
target_objects = {}


def get_node_or_warn(robot: Supervisor, def_name: str):
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


def init_scene_objects(robot: Supervisor):
    """
    Initializes scene objects and automatically categorizes them
    into red_objects, blue_objects, and target_objects.
    """

    global puzzle_outline, viewpoint
    global small_triangle_red, para_1_red, para_2_red
    global big_triangle_1_red, big_triangle_2_red, square_red
    global small_triangle_blue, para_1_blue, para_2_blue
    global big_triangle_1_blue, big_triangle_2_blue, square_blue
    global small_triangle_target, para_1_target, para_2_target
    global big_triangle_1_target, big_triangle_2_target, square_target
    global tangram_target_proto
    global red_objects, blue_objects, target_objects

    # 1) World-level nodes (not inside PROTOs)
    world_def_map = {
        "puzzle_outline": "PUZZLE_OUTLINE",
        "viewpoint": "USER_CAMERA",
        # Red pieces
        "small_triangle_red": "SMALL_TRIANGLE_RED",
        "para_1_red": "PARALELLOGRAM_2_RED",
        "para_2_red": "PARALELLOGRAM_1_RED",
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
        # Tangram target PROTO instance
        "tangram_target_proto": "TANGRAM_TARGET",  # DEF in the world file
    }

    red_objects = {}
    blue_objects = {}
    target_objects = {}

    # Fetch all world-level nodes and auto-group red/blue
    for attr_name, def_name in world_def_map.items():
        node = get_node_or_warn(robot, def_name)
        globals()[attr_name] = node  # set e.g. small_triangle_red, square_blue, etc.

        lname = attr_name.lower()
        if "red" in lname:
            red_objects[attr_name] = node
        elif "blue" in lname:
            blue_objects[attr_name] = node

    # 2) Internal target nodes inside the TangramPiecesTarget PROTO instance
    if tangram_target_proto is None:
        print(
            "[scene_objects] WARNING: 'tangram_target_proto' is None. "
            "Internal target pieces will not be initialized."
        )
        return  # nothing more we can do for target pieces

    # Map variables -> internal DEF names inside TangramPiecesTarget
    target_def_map = {
        "small_triangle_target": "SMALL_TRIANGLE_TARGET",
        "para_1_target": "PARALELLOGRAM_1_TARGET",
        "para_2_target": "PARALELLOGRAM_2_TARGET",
        "big_triangle_1_target": "BIG_TRIANGLE_1_TARGET",
        "big_triangle_2_target": "BIG_TRIANGLE_2_TARGET",
        "square_target": "SQUARE_TARGET",
    }

    for attr_name, internal_def in target_def_map.items():
        node = get_proto_node_or_warn(tangram_target_proto, internal_def)
        globals()[attr_name] = node
        target_objects[attr_name] = node

    # Optional debug print
    print("[scene_objects] Initialized:")
    print(f"  Red objects:   {list(red_objects.keys())}")
    print(f"  Blue objects:  {list(blue_objects.keys())}")
    print(f"  Target objects:{list(target_objects.keys())}")
