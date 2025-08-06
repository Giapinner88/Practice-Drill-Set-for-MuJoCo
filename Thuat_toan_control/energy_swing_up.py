import numpy as np

class Swing_up:
    def __init__(self, m, l, g, k, kx, kv):
        self.m = m
        self.l = l      # 1/2 chiều dài con lắc
        self.g = g
        self.k = k      # hệ số điều khiển năng lượng
        self.kx = kx    # hệ số phản hồi vị trí
        self.kv = kv    # hệ số phản hồi vận tốc

    def energy(self,  x, x_dot, theta, theta_dot):
        E = 0.5 * self.m * (self.l**2) * (theta_dot**2) + self.m * self.g * self.l * (1 - np.cos(theta))
        E_end = self.m * self.g * self.l
        u = self.k * (E_end - E) * np.sign(theta_dot * np.cos(theta))
        u -= self.kx * x + self.kv * x_dot
        return u
