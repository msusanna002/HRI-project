from task_utils import MovePieceTask
import scene_objects as scene
import arm_motion_utils as arm_utils
import random

class GameManager:
    def __init__(self, robot_node, gaze_duration):
        self.robot_node = robot_node
        self.finished = False

        self.state = "choose_next_piece"  
        self.current_piece = None
        self.last_piece = None
        self.was_correct_move = None

        self.current_arm_task = None
        self.gaze_running = False
        self.gaze_start_time = 0.0
        self.gaze_duration = gaze_duration
        self.placed_pieces = []
        self.current_step = 0

    def get_scattered_pieces(self):
        scattered = []
        for name, node in scene.all_pieces.items():
            if node is None:
                continue
            field = node.getField("isPlaced")
            if field is None:
                is_placed = False
            else:
                try:
                    is_placed = field.getSFBool()
                except Exception as e:
                    print("[GameManager] WARNING: isPlaced on", name,
                        "is not SFBool, defaulting to False:", e)
                    is_placed = False

            if not is_placed:
                scattered.append(node)
        return scattered
    
    def filter_out_placed_types(self, pieces):
        unplaced = []
        for node in pieces:
            is_placed = False
            for placed in self.placed_pieces:
                if node.getTypeName() == placed.getTypeName():
                    is_placed = True
                    break
            if not is_placed:
                unplaced.append(node)
        return unplaced


    def get_next_piece(self):
        print("Getting next piece")

        possible_pieces = []

        scattered_pieces = self.get_scattered_pieces()
        possible_pieces = self.filter_out_placed_types(scattered_pieces)

        print("Possible pieces to place:", [p.getDef() for p in possible_pieces])

        #return None if there are no possible pieces left, ie. puzzle is complete
        if len(possible_pieces) == 0:
            return None

        #randomly choose one of the possible pieces
        i = random.randint(0, len(possible_pieces) - 1)
        print("random choice index:", i)

        return possible_pieces[i]

    
    #TODO: LLM integration
    def LLM_get_next_piece(self):
        next_piece = self.get_next_piece()
        return next_piece
    
    #TODO: Gaze data gathering integration
    def gather_gaze_data(self):
        scene.current_object = scene.viewpoint
        random_value = random.randint(0, 1)
        if random_value == 0:
            return True
        else:
            return False 


    def all_pieces_placed(self):
        return all(p.is_placed for p in scene.all_pieces)

    def step(self):
        """Call this once per simulation tick from the main Webots loop."""

        if self.finished:
            return

        # 1. If an arm task is running, step it
        if self.current_arm_task is not None:
            self.current_arm_task.step()
            if self.current_arm_task.done:
                self.current_arm_task = None
            else:
                return   # wait until task finishes

        # 2. If gaze tracking is running
        elif self.gaze_running and self.state == "gaze_track":
            
            #placeholder for gaze data gathering
            self.was_correct_move = self.gather_gaze_data()

            #printing log
            now = arm_utils.robot.getTime()
            delta = now - self.gaze_start_time
            steps = 5
            gaze_time_step = self.gaze_duration / steps
            if delta >= gaze_time_step * self.current_step:
                if self.current_step == 0:
                    print("Gaze tracking running...")
                print (steps - self.current_step ,"/", steps )
                self.current_step += 1
            
            if now - self.gaze_start_time >= self.gaze_duration:
                print("Finished gaze tracking")
                print("Gaze tracking result: ", self.was_correct_move)
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
                return

        # 3. State machine
        elif self.state == "choose_next_piece":
            self.current_piece = self.get_next_piece()
            
            if self.current_piece is None:
                self.finished = True
                print("Puzzle completed!")
                return
            scene.current_object = self.current_piece

            print("Placing:", self.current_piece.getDef())

            self.current_arm_task = MovePieceTask(
                self.current_piece,
                self.current_piece.getField("targetPos").getSFVec3f(),
                self.robot_node
            )
            self.state = "wait_arm_done"

        elif self.state == "retry":
            print("Removing piece:", self.current_piece.getDef())
            self.current_arm_task = MovePieceTask(
                self.current_piece,
                self.current_piece.getField("scatteredPos").getSFVec3f(),
                self.robot_node
            )
            self.state = "LLM_choose_next_piece"

        elif self.state == "LLM_choose_next_piece":
            self.current_piece = self.LLM_get_next_piece()
            scene.current_object = self.current_piece

            print("Placing:", self.current_piece.getDef())

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
