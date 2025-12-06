#How to Set Up and Run the HRI Tangram Gaze‑LLM System
This guide explains how to install dependencies, configure Webots, start the gaze tracking + LLM servers, and launch the robot simulation.

##1. Create and Activate Virtual Environment
.\.venv\Scripts\activate
Activates the project's Python virtual environment so all dependencies install locally instead of globally.

📦 2. Install IKPy (robot inverse kinematics library)
C:\Users\HM\AppData\Local\Programs\Python\Python311\python.exe -m pip install git+https://github.com/alters-mit/ikpy.git#egg=ikpy
This installs IKPy directly from the MIT-alters GitHub repository.
Webots requires IKPy for inverse kinematics when controlling the robot arm.

📦 3. Install All Remaining Dependencies
Option A — install packages individually:
C:\Users\HM\AppData\Local\Programs\Python\Python311\python.exe -m pip install fastapi uvicorn requests opencv-python mediapipe numpy ikpy
Option B — install automatically from requirements.txt:
C:\Users\HM\AppData\Local\Programs\Python\Python311\python.exe -m pip install -r requirements.txt
This installs:

FastAPI → for the gaze evaluation server

Uvicorn → runs the FastAPI server

Requests → used by Webots to call the servers

OpenCV + Mediapipe → for webcam gaze tracking

NumPy → mathematical operations

IKPy → robot inverse kinematics

🤖 4. Tell Webots to Use Your Virtual Environment Python
$env:WEBOTS_PYTHON = (Resolve-Path .\.venv\Scripts\python.exe).Path
This ensures Webots runs controllers using the exact same venv Python where your libraries are installed.

🟩 5. Launch Webots
& "C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe"
This opens the Webots simulator where the Tangram robot environment will run.

🎯 6. Calibrate Gaze Tracking
python calibrategaze.py
During calibration, follow the yellow dots for 2 seconds each.
This step lets Mediapipe learn your eye geometry for accurate gaze detection.

🧠 7. Start the Local LLM Agent
python llm_agent.py
This runs the LLM recommendation server at:

http://127.0.0.1:8001/recommend_piece
Webots calls this to get the next puzzle piece suggestion.

🔍 8. Start the Gaze Evaluation Server (FastAPI)
Open a second terminal, activate the venv, then run:

cd WeBots/gaze_algo
.\.venv\Scripts\python.exe -m uvicorn gaze_server:app --host 127.0.0.1 --port 8000 --log-level info
This creates the gaze evaluation endpoint:

http://127.0.0.1:8000/evaluate_move
Webots continuously sends:

piece ID

gaze duration

board state

The server returns Positive or Negative to guide robot behavior.

▶️ 9. Start Webots Simulation
With:

llm_agent.py running

gaze_server running

Webots open

Click Run inside Webots.

The system will now:

Track your gaze in real-time

Evaluate each robot move

Provide LLM-generated piece suggestions

Remove or place pieces according to your gaze feedback
