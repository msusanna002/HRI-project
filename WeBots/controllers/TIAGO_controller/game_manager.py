from task_utils import MovePieceTask
import scene_objects as scene
import arm_motion_utils as arm_utils
import random

random.seed(42)

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
            return True 

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
            
            #get error detection decision from gaze data gathering
            #OBS: is called every step during gaze tracking
            self.was_correct_move = self.gather_gaze_data()

            #printing log
            now = arm_utils.robot.getTime() #current time
            delta = now - self.gaze_start_time #time elapsed since gaze started
            steps = 5 #number of log steps during gaze duration
            gaze_time_step = self.gaze_duration / steps #time per log step

            #print if it's time for next log step
            if delta >= gaze_time_step * self.current_step:
                if self.current_step == 0:
                    print("Gaze tracking running...")
                print (steps - self.current_step ,"/", steps )
                self.current_step += 1
            
            #when gaze duration is over, process result
            if delta >= self.gaze_duration:
                print("Finished gaze tracking")
                print("Gaze tracking result: ", self.was_correct_move)

                #reset gaze variables for next use
                self.current_step = 0
                self.gaze_running = False

                #if the move was correct, mark piece as placed and add to placed pieces array
                if self.was_correct_move:
                    print("Correct move!")
                    self.current_piece.getField("isPlaced").setSFBool(True)
                    self.placed_pieces.append(self.current_piece)
                    self.state = "choose_next_piece"
                #if the move was incorrect, retry (remove piece and choose next via LLM)
                else:
                    print("Incorrect move, try again.")
                    self.state = "retry"
            else:
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
