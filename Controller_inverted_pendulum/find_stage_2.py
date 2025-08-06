import mujoco
import mujoco.viewer
import numpy as np
import math
import time

# Load mô hình từ XML
model = mujoco.MjModel.from_xml_path("Con_lac_nguoc.xml")
data = mujoco.MjData(model)

# Thông số ban đầu (có thể thay đổi để test)
x = 0                    # Vị trí xe 
x_dot = 0                # Tốc độ xe (m/s)
theta = math.radians(20) # Góc ban đâu (rad) (<= 20 độ)
theta_dot = 0            # Tốc độ góc (rad/s)

# Tham số mục tiêu (có thể thay đổi để test)
x_ref = 0.0
theta_ref = 0.0

# Thông số PID Ki
ki_x = 0      
ki_theta = 0

# Ghi thông số vào Mujoco
data.qpos[0] = x
data.qvel[0] = x_dot
data.qpos[1] = theta
data.qvel[1] = theta_dot

# Hàm PID cơ bản
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative
    
# Đánh giá một bộ PID
def evaluate_pid(kp_x, kd_x, kp_theta, kd_theta, x, x_dot, theta, theta_dot, x_ref, theta_ref, sim_time=20.0):
    pid1 = PIDController(kp_x, ki_x , kd_x)
    pid2 = PIDController(kp_theta, ki_theta, kd_theta)

    data = mujoco.MjData(model)

    data.qpos[0] = x
    data.qvel[0] = x_dot
    data.qpos[1] = theta
    data.qvel[1] = theta_dot


    steps = int(sim_time / model.opt.timestep)
    for _ in range(steps):
        # Đọc dữ liệu trạng thái
        x = data.qpos[0]
        x_dot = data.qvel[0]
        theta = data.qpos[1]
        theta_dot = data.qvel[1]

        # Tính sai số
        error_x = x_ref - x
        error_theta = theta_ref - theta

        u1 = pid1.compute(error_x, model.opt.timestep)
        u2 = pid2.compute(error_theta, model.opt.timestep)

        u = -np.clip(u1 + u2, -1.0, 1.0)

        data.ctrl[0] = u
        mujoco.mj_step(model, data)
        if abs(theta) > np.pi: 
            return False
    return True

# Thử các bộ hệ số PID
def auto_tune_pid():
    best = None
    for kp_x in [0.5, 10, 20, 50, 100, 150]:
        for kd_x in [ 0.1, 0.5, 1, 5, 10, 20]:
            for kp_theta in [ 0.5, 10, 20, 50, 100, 150]:
                for kd_theta in [ 0.1, 0.5, 1, 5, 10, 20]:
                    print(f"Đang thử PID1(kp={kp_x}, kd={kd_x}) PID2(kp={kp_theta}, kd={kd_theta})...")
                    if evaluate_pid(kp_x, kd_x, kp_theta, kd_theta, x, x_dot, theta, theta_dot, x_ref, theta_ref):
                        print("Giữ được 20s!")
                        return kp_x, kd_x, kp_theta, kd_theta
    return None

# Tự động tìm hệ số phù hợp
result = auto_tune_pid()
if result is None:
    print(" Không tìm được bộ PID giữ được 10s.")
    exit()

kp_x, kd_x, kp_theta, kd_theta = result
print(f"Dùng bộ tốt nhất: PID1(kp={kp_x}, kd={kd_x}) PID2(kp={kp_theta}, kd={kd_theta}).")