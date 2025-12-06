# gaze_server.py
import time
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from gazetracking import GazeTracker  # your existing module
from logger import get_logger

LOG = get_logger(__name__)
app = FastAPI()

# configure these to your screen size and camera calibration
SCREEN_W = 2560
SCREEN_H = 1664
DEFAULT_GAZE_WINDOW = 5.0

tracker = None

class GazeRequest(BaseModel):
    move_id: str | None = None
    piece_id: str | None = None
    board_state: dict | None = None
    gaze_duration: float | None = None

@app.on_event("startup")
def startup_event():
    global tracker
    LOG.info("Starting gaze_server startup")
    # initialize GazeTracker but instruct it to not show windows by default
    tracker = GazeTracker(SCREEN_W, SCREEN_H, calib_path="gaze_calib.npz")
    LOG.info("GazeTracker initialized")

@app.post("/evaluate_move")
def evaluate_move(req: GazeRequest):
    """
    Called by Webots GameManager. Runs gaze tracking for the requested duration and returns decision.
    Response example:
    { "decision":"Positive", "metrics": {...} }
    """
    global tracker
    if tracker is None:
        raise HTTPException(status_code=500, detail="GazeTracker not initialized")

    duration = float(req.gaze_duration or DEFAULT_GAZE_WINDOW)
    LOG.info("Received evaluate_move: piece=%s duration=%.2fs", req.piece_id, duration)

    # collect samples in blocking call (this uses camera)
    try:
        samples = tracker.collect_window(duration, show=False)
    except Exception as e:
        LOG.exception("collect_window failed: %s", e)
        raise HTTPException(status_code=500, detail=str(e))

    # compute metrics using your compute_metrics/decide functions (assume in gazewithpoint or gazeutils)
    # Minimal inline metric: positive if more board AOI than robot AOI (you can replace with compute_metrics)
    from gazewithpoint import compute_metrics, decide  # your script earlier
    metrics = compute_metrics(samples)
    decision = decide(metrics)
    LOG.info("Metrics computed: final_score=%.3f decision=%s", metrics.get("final_score", 0.0), decision)

    # Return JSON
    return {"decision": decision, "metrics": metrics}

if __name__ == "__main__":
    LOG.info("Starting gaze_server on http://0.0.0.0:8000")
    uvicorn.run("gaze_server:app", host="0.0.0.0", port=8000, reload=False)
