# e.g. in keyboard_utils.py or demo_utils.py


from task_utils import MovePieceTask
import scene_objects as scene

class MoveAllPiecesTask:
    """
    Non-blocking task that moves ALL tangram pieces to fixed target positions
    using MovePieceTask internally.
    """

    def __init__(self, robotNode, piece_target_pairs,
                 label="MoveAllPiecesTask"):
        
        self.robotNode = robotNode # the TIAGO robot node
        self.piece_target_pairs = piece_target_pairs # list of (piece_node, target_vector) tuples
        self.label = label # task label for logging

        self.index = 0 # current index in piece_target_pairs
        self.current_subtask = None # current MovePieceTask
        self.done = False # overall task completion flag

        if not piece_target_pairs:
            print(f"[{label}] WARNING: No piece-target pairs; finishing immediately.")
            self.done = True

    def step(self):
        # If already done, nothing to do
        if self.done:
            return

        # If no current task, start the next one
        if self.current_subtask is None:
            if self.index >= len(self.piece_target_pairs):
                print(f"[{self.label}] All pieces processed.")
                self.done = True
                return

            # Start next subtask
            piece_node, target_pos = self.piece_target_pairs[self.index]

            if piece_node is None:
                print(f"[{self.label}] WARNING: missing piece node, skipping index {self.index}")
                self.index += 1
                return

            #logging
            piece_def = piece_node.getDef()
            print(f"[{self.label}] Moving {piece_def} → {target_pos}")

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