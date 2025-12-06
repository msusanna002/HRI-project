# calibrategaze.py
import cv2
import time
import numpy as np
import mediapipe as mp

from gazeutils import GazeCalibrator, estimate_gaze_features
from logger import get_logger

LOG = get_logger(__name__)

# -------------------------------------------------------------------
# CONFIG – set this as each individual's actual screen resolution
# -------------------------------------------------------------------
SCREEN_W = 2560  
SCREEN_H = 1664    

PRE_WAIT_SECONDS = 0.8
COLLECT_SECONDS = 2.0

# -------------------------------------------------------------------
# Mediapipe FaceMesh
# -------------------------------------------------------------------
mp_face_mesh = mp.solutions.face_mesh

def make_calibration_points(screen_w: int, screen_h: int):
    """
    9‑point grid:
       (15%, 15%) (50%, 15%) (85%, 15%)
       (15%, 50%) (50%, 50%) (85%, 50%)
       (15%, 85%) (50%, 85%) (85%, 85%)
    """
    xs = [0.15, 0.5, 0.85]
    ys = [0.15, 0.5, 0.85]
    pts = []
    for vy in ys:
        for vx in xs:
            px = int(vx * screen_w)
            py = int(vy * screen_h)
            pts.append((px, py))
    return pts


def draw_calibration_screen(img_w, img_h, dot_pos, point_idx, total_points):
    """
    Create a black background with a single yellow dot at dot_pos
    and some instruction text. So It is easier for the user to calibrate on their own.
    """
    canvas = np.zeros((img_h, img_w, 3), dtype=np.uint8)

    # Yellow dot
    cv2.circle(canvas, dot_pos, 15, (0, 255, 255), -1)

    # Text
    msg1 = f"Point {point_idx} / {total_points}"
    msg2 = "Look at the yellow dot and keep your head still."
    msg3 = "Press ESC to abort."

    cv2.putText(canvas, msg1, (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(canvas, msg2, (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(canvas, msg3, (30, img_h - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    return canvas


def main():
    LOG.info("Starting 9‑point gaze calibration")

    # 1. Open camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        LOG.error("Cannot open webcam (index 0).")
        return

    # 2. Setup calibration window
    win_name = "Gaze Calibration"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    # Make it full screen so it matches the webots screen during testing and run
    try:
        cv2.setWindowProperty(
            win_name,
            cv2.WND_PROP_FULLSCREEN,
            cv2.WINDOW_FULLSCREEN
        )
    except Exception:
        # Fallback: just resize to screen size
        cv2.resizeWindow(win_name, SCREEN_W, SCREEN_H)

    # 3. Building 9 calibration points for this screen size
    points = make_calibration_points(SCREEN_W, SCREEN_H)
    LOG.info("Calibration points: %s", points)

    face_mesh = mp_face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )

    all_features = []  # list of (horiz, vert)
    all_targets = []   # list of (screen_x, screen_y)

    try:
        for idx, (px, py) in enumerate(points, start=1):
            LOG.info("Point %d / %d: (%d, %d) — look at the yellow dot.",
                     idx, len(points), px, py)

            # Show static screen with point so user can move gaze
            canvas = draw_calibration_screen(SCREEN_W, SCREEN_H,
                                             (px, py), idx, len(points))
            cv2.imshow(win_name, canvas)
            start_show = time.time()
            while time.time() - start_show < PRE_WAIT_SECONDS:
                if cv2.waitKey(1) & 0xFF == 27:  # ESC
                    LOG.info("Calibration aborted by user (during PRE_WAIT).")
                    return

            LOG.info("Collecting samples for point: (%d, %d)", px, py)

            # Collect frames for COLLECT_SECONDS
            collect_end = time.time() + COLLECT_SECONDS
            point_features = 0
            while time.time() < collect_end:
                ret, frame = cap.read()
                if not ret:
                    continue

                # process face
                h, w, _ = frame.shape
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = face_mesh.process(rgb)

                if results.multi_face_landmarks:
                    fl = results.multi_face_landmarks[0]
                    horiz, vert = estimate_gaze_features(fl, w, h)
                    all_features.append((horiz, vert))
                    all_targets.append((px, py))
                    point_features += 1

                # show both: calibration screen + small camera view in corner
                cam_small = cv2.resize(frame, (260, 180))
                canvas = draw_calibration_screen(SCREEN_W, SCREEN_H,
                                                 (px, py), idx, len(points))
                # put camera in bottom‑right corner
                ch, cw, _ = cam_small.shape
                canvas[SCREEN_H - ch - 10:SCREEN_H - 10,
                       SCREEN_W - cw - 10:SCREEN_W - 10] = cam_small

                cv2.imshow(win_name, canvas)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC
                    LOG.info("Calibration aborted by user (during collection).")
                    return

            LOG.info("Collected %d feature samples for point %d",
                     point_features, idx)

        if not all_features:
            LOG.error("No samples collected at all. Calibration failed.")
            return

        # 4. Fit calibration matrix
        calib = GazeCalibrator()
        LOG.info("Fitting calibration matrix on %d samples",
                 len(all_features))
        calib.fit(all_features, all_targets)

        if calib.M is None:
            LOG.error("Calibration matrix M is None. Fit failed.")
            return

        # 5. Save to gaze_calib.npz
        np.savez("gaze_calib.npz", M=calib.M)
        LOG.info("Calibration done. Saved gaze_calib.npz with M=%s", calib.M)

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
