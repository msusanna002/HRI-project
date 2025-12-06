# gazewithpoint.py
"""
Computes metrics and decision from a list of FrameSample objects (from gazeutils.FrameSample).
This file contains compute_metrics(samples) -> dict and decide(metrics) -> "Positive"/"Negative"/"NoData"
"""

from gazeutils import FrameSample
import numpy as np

# PARAMETERS (tweak)
BLINK_THRESH = 3  # >3 blinks in window -> negative
SHIFT_THRESH = 4  # >4 AOI shifts -> negative

# weights - the importance of each metrics
W_BLINK = 0.2
W_SHIFT = 0.4
W_DWELL = 0.3
W_RATIO = 0.1

def compute_metrics(samples):
    if not samples:
        return None

    blink_count = sum(1 for s in samples if s.blink)
    aoi_seq = [s.aoi for s in samples]
    shifts = sum(1 for i in range(1, len(aoi_seq)) if aoi_seq[i] != aoi_seq[i-1])

    # approximate dwell time per AOI using equal dt per sample
    dt = (samples[-1].time - samples[0].time) / max(1, len(samples))
    dwell = {"Robot":0.0, "Puzzle":0.0, "Scattered":0.0, "Outside":0.0}
    for s in samples:
        lbl = s.aoi if s.aoi in dwell else "Outside"
        dwell[lbl] += dt

    total_dwell = sum(dwell.values()) if sum(dwell.values()) > 0 else 1.0
    prop_robot = dwell["Robot"] / total_dwell
    prop_board = dwell["Puzzle"] / total_dwell

    # ratio logic (Robot / Board)
    if dwell["Puzzle"] == 0:
        ratio_score = 0
    elif dwell["Robot"] == 0:
        ratio_score = +1
    elif dwell["Robot"] > dwell["Puzzle"]:
        ratio_score = -1
    elif dwell["Puzzle"] > dwell["Robot"]:
        ratio_score = +1
    else:
        ratio_score = 0

    blink_score = +1 if blink_count <= BLINK_THRESH else -1
    shift_score = -1 if shifts > SHIFT_THRESH else +1

    pos_dwell = dwell["Puzzle"] + dwell["Outside"]
    neg_dwell = dwell["Robot"] + dwell["Scattered"]
    if pos_dwell > neg_dwell:
        dwell_score = +1
    elif neg_dwell > pos_dwell:
        dwell_score = -1
    else:
        dwell_score = 0
    print(dwell["Outside"],dwell["Puzzle"],dwell["Robot"],dwell["Scattered"])
    print(blink_score,shift_score,dwell_score,ratio_score)
    final_score = (
        W_BLINK * blink_score +
        W_SHIFT * shift_score +
        W_DWELL * dwell_score +
        W_RATIO * ratio_score
    )

    metrics = {
        "blink_count": blink_count,
        "shifts": shifts,
        "dwell": dwell,
        "prop_robot": prop_robot,
        "prop_board": prop_board,
        "ratio_score": ratio_score,
        "blink_score": blink_score,
        "shift_score": shift_score,
        "dwell_score": dwell_score,
        "final_score": final_score
    }
    return metrics

def decide(metrics):
    if metrics is None:
        return "NoData"
    return "Positive" if metrics["final_score"] >= 0 else "Negative"
