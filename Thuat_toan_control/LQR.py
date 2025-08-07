import numpy as np
from scipy.linalg import solve_continuous_are

class LQR:
    def __init__(self, A, B, Q, R):
        """
        A, B: Ma trận trạng thái và điều khiển của hệ thống tuyến tính liên tục
        Q: Ma trận trọng số trạng thái
        R: Ma trận trọng số điều khiển
        """
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R

        # Giải phương trình Riccati đại số liên tục
        self.P = solve_continuous_are(A, B, Q, R)

        # Tính ma trận phản hồi trạng thái K
        self.K = np.linalg.inv(R) @ B.T @ self.P

    def get_gain(self):
        """Trả về ma trận K"""
        return self.K

    def compute_control(self, x):
        """
        Tính toán tín hiệu điều khiển u = -Kx
        """
        u = -self.K @ x
        return u
