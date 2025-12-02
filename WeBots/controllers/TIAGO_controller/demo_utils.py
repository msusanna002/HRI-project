# e.g. in keyboard_utils.py or demo_utils.py

from arm_motion_utils import arm_movement
import scene_objects as scene


def _move_arm_over_group(piece_dict, robot_node, group_name):
    """
    Helper: loops over all nodes in piece_dict and calls arm_movement on each.

    piece_dict: dict[str, Node] from scene_objects (red_objects, blue_objects, target_objects)
    x_offset, y_offset: desired grip offset in robot frame
    robot_node: Node of the robot (Supervisor.getFromDef("TIAGO") or similar)
    group_name: string label for debug prints ("red", "blue", "target")
    """
    if not piece_dict:
        print(f"[arm_demo] WARNING: No pieces in {group_name} group.")
        return

    # Sort keys for deterministic order
    for name in sorted(piece_dict.keys()):
        node = piece_dict[name]
        if node is None:
            print(f"[arm_demo] WARNING: Node '{name}' in {group_name} group is None, skipping.")
            continue

        print(f"[arm_demo] Moving arm to {group_name} piece '{name}'")
        arm_movement(node, robot_node)

def _move_arm_over_group_back_and_forth(piece_dict1, piece_dict2, robot_node,
                                        group_name1, group_name2):
    """
    Helper: move arm back and forth between two groups.

    piece_dict1, piece_dict2: dict[str, Node]
    robot_node: Node of the robot (Supervisor.getFromDef("TIAGO") or similar)
    group_name1, group_name2: string labels for debug prints ("red", "blue", etc.)
    """

    if not piece_dict1 and not piece_dict2:
        print(f"[arm_demo] WARNING: No pieces in {group_name1} or {group_name2} groups.")
        return

    if not piece_dict1:
        print(f"[arm_demo] WARNING: No pieces in {group_name1} group.")
        return

    if not piece_dict2:
        print(f"[arm_demo] WARNING: No pieces in {group_name2} group.")
        return

    # Deterministic order in each group
    names1 = sorted(piece_dict1.keys())
    names2 = sorted(piece_dict2.keys())

    # Only iterate up to the shorter list length
    limit = min(len(names1), len(names2))
    if limit == 0:
        print(f"[arm_demo] WARNING: No overlapping indices for {group_name1} and {group_name2}.")
        return

    for i in range(limit):
        name1 = names1[i]
        name2 = names2[i]

        node1 = piece_dict1.get(name1)
        node2 = piece_dict2.get(name2)

        if node1 is None or node2 is None:
            print(f"[arm_demo] WARNING: Node '{name1}' in {group_name1} "
                  f"or '{name2}' in {group_name2} is None, skipping.")
            continue

        print(f"[arm_demo] Moving arm to {group_name1} piece '{name1}' "
              f"then {group_name2} piece '{name2}'")
        arm_movement(node1, robot_node)
        arm_movement(node2, robot_node) 


def move_arm_to_all_pieces(robot_node, ):
    """
    Move the arm sequentially to:
      1. all red pieces
      2. all blue pieces
      3. all target pieces

    Uses the dictionaries initialized in scene_objects.init_scene_objects(robot).
    """
    # Make sure scene objects were initialized
    # (optional but helpful)
    if scene.red_objects is None or scene.blue_objects is None:
        print("[arm_demo] ERROR: scene_objects.init_scene_objects(robot) was not called.")
        return

    print("[arm_demo] Visiting all RED pieces...")
    _move_arm_over_group(scene.red_objects, robot_node, "red")

    print("[arm_demo] Visiting all BLUE pieces...")
    _move_arm_over_group(scene.blue_objects, robot_node, "blue")

    print("[arm_demo] Visiting all TARGET pieces...")
    _move_arm_over_group(scene.target_objects, robot_node, "target")

from arm_motion_utils import MovePieceTask  # if this code is in another file
import scene_objects as scene

class MoveAllPiecesTask:
    """
    Non-blocking task that moves ALL tangram pieces to fixed target positions
    using MovePieceTask internally.
    """

    def __init__(self, robotNode, piece_target_pairs,
                 label="MoveAllPiecesTask"):
        self.robotNode = robotNode
        self.piece_target_pairs = piece_target_pairs
        self.label = label

        self.index = 0
        self.current_subtask = None
        self.done = False

        if not piece_target_pairs:
            print(f"[{label}] WARNING: No piece-target pairs; finishing immediately.")
            self.done = True

    def step(self):
        if self.done:
            return

        # If no current task, start the next one
        if self.current_subtask is None:
            if self.index >= len(self.piece_target_pairs):
                print(f"[{self.label}] All pieces processed.")
                self.done = True
                return

            piece_node, target_pos = self.piece_target_pairs[self.index]

            if piece_node is None:
                print(f"[{self.label}] WARNING: missing piece node, skipping index {self.index}")
                self.index += 1
                return

            piece_def = piece_node.getDef()
            print(f"[{self.label}] Moving {piece_def} → {target_pos}")

            # NOTE: second argument is a VECTOR, not a node
            self.current_subtask = MovePieceTask(
                piece_node,
                target_pos,     # world-space xyz vector
                self.robotNode
            )
            return

        # Advance current subtask
        self.current_subtask.step()

        if self.current_subtask.done:
            piece_node, target_pos = self.piece_target_pairs[self.index]
            print(f"[{self.label}] Finished {piece_node.getDef()}")
            self.current_subtask = None
            self.index += 1

def create_move_all_pieces_task(robot_node):
    """
    Creates a MoveAllPiecesTask that moves every tangram piece
    to its fixed target WORLD coordinates from scene_objects.py
    """

    # Build ordered list of (piece_node, target_vector)
    piece_target_pairs = [

        # RED pieces
        (scene.small_triangle_red, scene.small_triangle_target_pos),
        (scene.para_1_red,         scene.para_1_target_pos),
        (scene.para_2_red,         scene.para_2_target_pos),
        (scene.big_triangle_1_red, scene.big_triangle_1_target_pos),
        (scene.big_triangle_2_red, scene.big_triangle_2_target_pos),
        (scene.square_red,         scene.square_target_pos),

        # BLUE pieces
        (scene.small_triangle_blue, scene.small_triangle_target_pos),
        (scene.para_1_blue,         scene.para_1_target_pos),
        (scene.para_2_blue,         scene.para_2_target_pos),
        (scene.big_triangle_1_blue, scene.big_triangle_1_target_pos),
        (scene.big_triangle_2_blue, scene.big_triangle_2_target_pos),
        (scene.square_blue,         scene.square_target_pos),
    ]

    # Filter missing nodes
    filtered = []
    for piece, pos in piece_target_pairs:
        if piece is None:
            print("[create_move_all_pieces_task] Missing piece node → skipping.")
            continue
        filtered.append((piece, pos))

    if not filtered:
        print("[create_move_all_pieces_task] ERROR: no valid piece-target pairs!")
        return None

    return MoveAllPiecesTask(robot_node, filtered)