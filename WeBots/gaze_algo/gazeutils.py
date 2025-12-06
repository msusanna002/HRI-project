# gazeutils.py
import numpy as np
from dataclasses import dataclass

# MediaPipe indices (FaceMesh)
LEFT_EYE_OUTER = 33
LEFT_EYE_INNER = 133
RIGHT_EYE_OUTER = 362
RIGHT_EYE_INNER = 263
LEFT_IRIS_CENTER = 468
RIGHT_IRIS_CENTER = 473

LEFT_EYE_EAR_IDX = [33, 160, 158, 133, 153, 144]
RIGHT_EYE_EAR_IDX = [362, 385, 387, 263, 373, 380]

@dataclass
class FrameSample:
    time: float
    aoi: str
    blink: bool
    screen_x: float | None = None
    screen_y: float | None = None

def landmark_to_xy(landmark, w, h):
    return int(landmark.x * w), int(landmark.y * h)

def estimate_gaze_features(face_landmarks, img_w, img_h):
    lm = face_landmarks.landmark

    lx_o = np.array(landmark_to_xy(lm[LEFT_EYE_OUTER], img_w, img_h), dtype=np.float32)
    lx_i = np.array(landmark_to_xy(lm[LEFT_EYE_INNER], img_w, img_h), dtype=np.float32)
    rx_o = np.array(landmark_to_xy(lm[RIGHT_EYE_OUTER], img_w, img_h), dtype=np.float32)
    rx_i = np.array(landmark_to_xy(lm[RIGHT_EYE_INNER], img_w, img_h), dtype=np.float32)

    l_ir = np.array(landmark_to_xy(lm[LEFT_IRIS_CENTER], img_w, img_h), dtype=np.float32)
    r_ir = np.array(landmark_to_xy(lm[RIGHT_IRIS_CENTER], img_w, img_h), dtype=np.float32)

    def norm_pos(outer, inner, iris):
        eye_vec = inner - outer
        eye_len = np.linalg.norm(eye_vec)
        if eye_len < 1e-6:
            return 0.5
        proj = np.dot(iris - outer, eye_vec / eye_len)
        return float(np.clip(proj / eye_len, 0.0, 1.0))

    l_h = norm_pos(lx_o, lx_i, l_ir)
    r_h = norm_pos(rx_o, rx_i, r_ir)
    horiz = (l_h + r_h) / 2.0

    l_y = l_ir[1] / img_h
    r_y = r_ir[1] / img_h
    vert = float(np.clip((l_y + r_y) / 2.0, 0.0, 1.0))

    return horiz, vert

class GazeCalibrator:
    def __init__(self):
        self.M = None

    def fit(self, features, targets):
        X, Y = [], []
        for (h, v), (sx, sy) in zip(features, targets):
            X.append([h, v, 1.0])
            Y.append([sx, sy])
        X = np.array(X, dtype=np.float32)
        Y = np.array(Y, dtype=np.float32)
        M, _, _, _ = np.linalg.lstsq(X, Y, rcond=None)
        self.M = M

    def predict(self, h, v):
        if self.M is None:
            return None
        vec = np.array([h, v, 1.0], dtype=np.float32)
        sx, sy = vec @ self.M
        return float(sx), float(sy)

def get_AOI(x, y, W, H, buffer=0):
    if x is None or y is None:
        return "Outside"
    if x < 0 or y < 0 or x > W or y > H:
        return "Outside"
    left_max = W/2 - buffer
    right_min = W/2 + buffer
    top_max = H/2 - buffer
    bot_min = H/2 + buffer
    if x <= left_max:
        return "Scattered"
    if x >= right_min and y <= top_max:
        return "Robot"
    if x >= right_min and y >= bot_min:
        return "Puzzle"
    if x < W/2:
        return "Scattered"
    return "Robot" if y < H/2 else "Puzzle"

def eye_aspect_ratio(pts):
    if pts.shape[0] < 6:
        return 1.0
    A = np.linalg.norm(pts[1] - pts[5])
    B = np.linalg.norm(pts[2] - pts[4])
    C = np.linalg.norm(pts[0] - pts[3])
    if C < 1e-6:
        return 1.0
    return (A + B) / (2.0 * C)

def detect_blink_from_landmarks(face_landmarks, img_w, img_h, ear_thresh=0.21):
    lm = face_landmarks.landmark
    def get_eye(idx_list):
        pts = []
        for i in idx_list:
            x, y = landmark_to_xy(lm[i], img_w, img_h)
            pts.append([x, y])
        return np.array(pts, dtype=np.float32)
    left = get_eye(LEFT_EYE_EAR_IDX)
    right = get_eye(RIGHT_EYE_EAR_IDX)
    ear = (eye_aspect_ratio(left) + eye_aspect_ratio(right)) / 2.0
    return ear < ear_thresh
