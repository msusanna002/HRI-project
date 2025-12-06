# gazetracking.py
import cv2
import time
import mediapipe as mp
import numpy as np
from gazeutils import estimate_gaze_features, GazeCalibrator, FrameSample, get_AOI, detect_blink_from_landmarks
import logger

LOG = logger.get_logger("gazetracking")
mp_face_mesh = mp.solutions.face_mesh

class GazeTracker:
    def __init__(self, screen_w: int, screen_h: int, calib_path: str = "gaze_calib.npz", show_window: bool = False, cam_index: int = 0):
        self.screen_w = screen_w
        self.screen_h = screen_h
        self.cap = None
        self.show_window = bool(show_window)
        self.cam_index = cam_index

        self.face_mesh = mp_face_mesh.FaceMesh(
            max_num_faces=1, refine_landmarks=True,
            min_detection_confidence=0.5, min_tracking_confidence=0.5)

        self.calib = GazeCalibrator()
        try:
            data = np.load(calib_path)
            self.calib.M = data["M"]
            LOG.info("Loaded calibration from %s", calib_path)
        except Exception as e:
            LOG.warning("Calibration not found or failed to load: %s", e)
            self.calib.M = None

    def start_camera(self):
        if self.cap is None:
            LOG.info("Opening camera index %s", self.cam_index)
            self.cap = cv2.VideoCapture(self.cam_index)
            if not self.cap.isOpened():
                raise RuntimeError("Cannot open webcam")

    def stop_camera(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                LOG.exception("Error releasing camera")
            self.cap = None
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

    def collect_window(self, window_seconds: float, show: bool = True):
        """Capture frames for window_seconds and return list[FrameSample].
        If show=True, display:
        - camera with AOI text
        - a separate "AOI debug" screen showing the predicted (sx, sy)
        """
        self.start_camera()
        samples = []
        end_t = time.time() + window_seconds
        if show:
            cv2.namedWindow("GazeTracker", cv2.WINDOW_NORMAL)
            cv2.namedWindow("AOI debug", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("AOI debug", self.screen_w, self.screen_h)
        while time.time() < end_t:
            ret, frame = self.cap.read()
            if not ret:
                continue
            h, w, _ = frame.shape
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.face_mesh.process(rgb)
            aoi = "Outside"
            blink = False
            sx = None
            sy = None
            if results.multi_face_landmarks:
                fl = results.multi_face_landmarks[0]
                horiz, vert = estimate_gaze_features(fl, w, h)
                if self.calib and self.calib.M is not None:
                    sx, sy = self.calib.predict(horiz, vert)
                else:
                    sx = horiz * self.screen_w
                    sy = vert * self.screen_h
                aoi = get_AOI(sx, sy, self.screen_w, self.screen_h)
                blink = detect_blink_from_landmarks(fl, w, h)
            samples.append(FrameSample(time.time(), aoi, blink, sx, sy))
            if show:
                disp = frame.copy()
                txt = f"AOI:{aoi} Blink:{int(blink)}"
                if sx is not None and sy is not None:
                    txt += f" X:{int(sx)} Y:{int(sy)}"
                cv2.putText(
                    disp, txt, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                    )
                cv2.imshow("GazeTracker", disp)
                debug = np.zeros((self.screen_h, self.screen_w, 3), dtype=np.uint8)
                cx = self.screen_w // 2
                cy = self.screen_h // 2
                cv2.line(debug, (cx, 0), (cx, self.screen_h), (80, 80, 80), 1)
                cv2.line(debug, (0, cy), (self.screen_w, cy), (80, 80, 80), 1)
                cv2.putText(debug, "Scattered", (30, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(debug, "Robot", (cx + 30, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(debug, "Puzzle", (cx + 30, cy + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                if sx is not None and sy is not None:
                    if aoi == "Outside":
                        color = (0, 0, 255)   # red
                    elif aoi == "Scattered":
                        color = (255, 255, 0) # cyan
                    elif aoi == "Robot":
                        color = (0, 255, 0)   # green
                    elif aoi == "Puzzle":
                        color = (255, 0, 0)   # blue
                    else:
                        color = (255, 255, 255)
                    ix = int(sx)
                    iy = int(sy)
                    if 0 <= ix < self.screen_w and 0 <= iy < self.screen_h:
                        cv2.circle(debug, (ix, iy), 8, color, -1)
                    else:
                        cv2.putText(debug, "OFF‑SCREEN", (30, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                cv2.putText(debug, f"AOI: {aoi}", (30, self.screen_h - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("AOI debug", debug)
                if cv2.waitKey(1) & 0xFF == 27: 
                    break

        return samples


    def __del__(self):
        try:
            self.stop_camera()
        except Exception:
            pass
