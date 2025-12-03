from task_utils import MovePieceTask, gather_gaze_data
import scene_objects as scene
import arm_motion_utils as arm_utils
import random

class GameManager:
    def __init__(self, robot_node):
        self.robot_node = robot_node
        self.finished = False

        self.state = "choose_next_piece"  
        self.current_piece = None
        self.last_piece = None
        self.was_correct_move = None

        self.current_arm_task = None
        self.gaze_running = False
        self.gaze_start_time = 0.0
        self.gaze_duration = 2.0
        self.placed_pieces = []
        self.current_step = 0



    def get_next_piece(self):
        print("Getting next piece")

        possible_pieces = []

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
                skip = False
                for placed in self.placed_pieces:
                    if node.getTypeName() == placed.getTypeName():
                        skip = True
                        break

                if skip:
                    continue  # do not append this node

                possible_pieces.append(node)

        print("Possible pieces to place:", [p.getDef() for p in possible_pieces])

        if not possible_pieces:
            return None

        i = random.randint(0, len(possible_pieces) - 1)
        print("random choice index:", i)

        return possible_pieces[i]


    
    #Placeholder for LLM integration
    def LLM_get_next_piece(self):
        next_piece = self.get_next_piece()
        return next_piece

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
            now = arm_utils.robot.getTime()
            #placeholder for gaze data gathering
            self.was_correct_move = gather_gaze_data()
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
