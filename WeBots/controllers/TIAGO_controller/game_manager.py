import requests
#from gaze_algo.gazetracking import LOG
import json
#from gaze_algo.logger import get_logger
from task_utils import MovePieceTask
import scene_objects as scene
import arm_motion_utils as arm_utils
import random
import sys, os
GAZE_ALGO_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "gaze_algo")
)
sys.path.append(GAZE_ALGO_PATH)
import gazetracking
from logger import get_logger
LOG = get_logger(__name__)
GAZE_SERVER_URL = "http://127.0.0.1:8000/evaluate_move" 


class GameManager:
    def __init__(self, robot_node, gaze_duration):
        self.robot_node = robot_node
        self.finished = False

        self.state = "choose_next_piece"  # directs the game flow
        self.current_piece = None #the piece currently being placed or removed
        self.was_correct_move = None #result of gaze tracking for last move

        self.current_arm_task = None #the current arm movement task
        self.gaze_running = False #is gaze tracking currently running
        self.gaze_start_time = 0.0 #time when gaze tracking started
        self.gaze_duration = gaze_duration #duration of gaze tracking in seconds
        self.placed_pieces = [] #list of pieces (nodes) that have been correctly placed
        self.current_step = 0 #current step in gaze tracking logging

    def get_scattered_pieces(self):
        scattered = []

        #find all pieces that are not yet placed
        for name, node in scene.all_pieces.items():
            if node is None:
                continue

            #use isPlaced field to check if piece is placed
            field = node.getField("isPlaced")

            #Check field type to avoid errors
            if field is None:
                is_placed = False
            else:
                try:
                    is_placed = field.getSFBool()
                except Exception as e:
                    print("[GameManager] WARNING: isPlaced on", name,
                        "is not SFBool, defaulting to False:", e)
                    is_placed = False

            #if not placed, add to scattered list
            if not is_placed:
                scattered.append(node)

        return scattered
    
    def filter_out_placed_types(self, pieces):
        unplaced = []

        #for each piece check against all placed pieces if type is already placed
        for node in pieces:
            is_placed = False
            for placed in self.placed_pieces:
                #compare type by checking node type name
                if node.getTypeName() == placed.getTypeName():
                    is_placed = True
                    break
            # if type not yet placed, add to unplaced list
            if not is_placed:
                unplaced.append(node)

        return unplaced


    def get_next_piece(self):
        print("Getting next piece")

        possible_pieces = []

        #get all scattered pieces
        scattered_pieces = self.get_scattered_pieces()

        #filter out pieces of types that have already been placed
        possible_pieces = self.filter_out_placed_types(scattered_pieces)

        #return None if there are no possible pieces left, ie. puzzle is complete
        if len(possible_pieces) == 0:
            return None

        #randomly choose one of the possible pieces
        i = random.randint(0, len(possible_pieces) - 1)
        print("random choice index:", i)

        return possible_pieces[i]

    
    def LLM_get_next_piece(self):
        """
        Ask the local llm agent (llm_agent.py as a server or module) for a suggested piece.
        We will try to call a local HTTP LLM bridge at http://127.0.0.1:8001/recommend_piece.
        If that fails, fall back to the built-in get_next_piece() heuristic.
        """
        try:
            payload = {
                "board_state": {
                    "placed": [p.getDef() for p in self.placed_pieces]
                },
                "available": [n.getDef() for n in self.get_scattered_pieces()]
            }
            resp = requests.post("http://127.0.0.1:8001/recommend_piece", json=payload, timeout=20.0)
            resp.raise_for_status()
            data = resp.json()
            name = data.get("recommended_piece")
            if name:
                # convert DEF string -> node object
                node = scene.all_pieces.get(name)
                if node:
                    LOG.info("LLM suggested piece: %s", name)
                    return node
        except Exception as e:
            LOG.warning("LLM HTTP call failed: %s — falling back to heuristic", e)

        # fallback:
        LOG.info("LLM fallback: using local get_next_piece()")
        return self.get_next_piece()

    
    def gather_gaze_data(self):
        """
        Call external gaze server (FastAPI) which will run gaze tracking for self.gaze_duration.
        The gaze server returns JSON: { "decision": "Positive" | "Negative" | "NoData", "metrics": {...} }
        Return True for Positive, False otherwise.
        """
        try:
            payload = {
                "move_id": f"step_{len(self.placed_pieces)}",
                "piece_id": self.current_piece.getDef() if self.current_piece is not None else None,
                "board_state": {
                    "placed": [p.getDef() for p in self.placed_pieces]
                },
                "gaze_duration": float(self.gaze_duration)
            }
            LOG.info("Calling gaze server %s with %s", GAZE_SERVER_URL, payload)
            r = requests.post(GAZE_SERVER_URL, json=payload, timeout=self.gaze_duration + 15)
            r.raise_for_status()
            data = r.json()
            decision = data.get("decision", "NoData")
            LOG.info("Gaze server returned: %s", decision)
            return True if decision == "Positive" else False
        except Exception as e:
            LOG.exception("Error calling gaze server: %s", e)
            return False

    def step(self):
        """Call this once per simulation tick from the main Webots loop."""

        if self.finished:
            return

        # If an arm task is running, step it
        if self.current_arm_task is not None:
            self.current_arm_task.step()
            if self.current_arm_task.done:
                self.current_arm_task = None
            else:
                return   # wait until task finishes

        # If gaze tracking is running
        elif self.gaze_running and self.state == "gaze_track":
            now = arm_utils.robot.getTime()
            if now - self.gaze_start_time >= self.gaze_duration:
                print("Finished gaze tracking")
                self.was_correct_move = self.gather_gaze_data()
                print("Gaze tracking result:", self.was_correct_move)
                self.current_step = 0
                self.gaze_running = False
                if self.was_correct_move:
                    print("Correct move!")
                    self.current_piece.getField("isPlaced").setSFBool(True)
                    self.placed_pieces.append(self.current_piece)
                    self.state = "choose_next_piece"
                else:
                    print("Incorrect move, try again.")
                    self.state = "retry"
            else:
                steps = 5
                gaze_time_step = self.gaze_duration / steps
                if now - self.gaze_start_time >= gaze_time_step * self.current_step:
                    if self.current_step == 0:
                        print("Gaze tracking running...")
                        print(steps - self.current_step, "/", steps)
                        self.current_step += 1

            return 

        # State: Decide what to do next 
        elif self.state == "choose_next_piece":
            # get next piece to place
            self.current_piece = self.get_next_piece()
            
            # if there are no valid pieces left, we are done
            if self.current_piece is None:
                self.finished = True
                print("Puzzle completed!")
                return
            
            # update current object for visualization
            scene.current_object = self.current_piece

            print("Placing:", self.current_piece.getDef())
            # move the piece in target position
            self.current_arm_task = MovePieceTask(
                self.current_piece,
                self.current_piece.getField("targetPos").getSFVec3f(),
                self.robot_node
            )
            self.state = "wait_arm_done"

        # State: retrying after incorrect move
        elif self.state == "retry":
            print("Removing piece:", self.current_piece.getDef())

            # move piece back to scattered position
            self.current_arm_task = MovePieceTask(
                self.current_piece,
                self.current_piece.getField("scatteredPos").getSFVec3f(),
                self.robot_node
            )
            self.state = "LLM_choose_next_piece"

        # State: choose next piece via LLM
        elif self.state == "LLM_choose_next_piece":
            #get next piece from LLM
            self.current_piece = self.LLM_get_next_piece()

            # update current object for visualization
            scene.current_object = self.current_piece

            print("Placing:", self.current_piece.getDef())

            # place the piece in target position
            self.current_arm_task = MovePieceTask(
                self.current_piece,
                self.current_piece.getField("targetPos").getSFVec3f(),
                self.robot_node
            )
            self.state = "wait_arm_done"

        elif self.state == "wait_arm_done":
            # arm done automatically triggers next logic in step()
            self.state = "gaze_track"
            self.gaze_start_time = arm_utils.robot.getTime()
            self.gaze_running = True
