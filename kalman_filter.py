import numpy as np
import time

class KalmanFilter2D:
    def __init__(self):
        self.x = np.zeros((4, 1))          # [cx, cy, vx, vy]
        self.P = np.eye(4) * 1000
        self.H = np.array([[1,0,0,0],[0,1,0,0]], dtype=float)
        self.R = np.eye(2) * 15
        self.Q = np.diag([1.0, 1.0, 10.0, 10.0])
        self.last_time = None              # FIX 1: None dulu, bukan time.time()
        self.initialized = False

    def predict(self):
        now = time.time()
        if self.last_time is None:         # skip predict pertama
            self.last_time = now
            return
        dt = now - self.last_time
        dt = min(dt, 0.1)                  # clamp dt, jaga dari lag spike
        self.last_time = now

        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, cx, cy):
        z = np.array([[cx], [cy]])
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S) # FIX 2: solve, bukan inv
        y = z - self.H @ self.x
        self.x = self.x + K @ y

        # FIX 3: Joseph form — numerically stable
        I_KH = np.eye(4) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

        return float(self.x[0]), float(self.x[1])

    def predict_only(self):
        """Panggil saat objek tidak terdeteksi (occlusion)"""
        self.predict()
        return float(self.x[0]), float(self.x[1])