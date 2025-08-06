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

# Thông số PID điều khiển vị trí (Có thể thay đổi để test)
kp_x = 0.5
kd_x = 0.5
ki_x = 0                  # Thường để 0

# Thông số PID điều khiển góc (Có thể thay đổi để test)
kp_theta = 10
kd_theta = 0.5
ki_theta = 0                  # Thường để 0

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
    
# Tạo bộ điều khiển PID
pid1 = PIDController(kp_x, ki_x, kd_x)
pid2 = PIDController(kp_theta, ki_theta, kd_theta)

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    time.sleep(2)
    while viewer.is_running():
        step_start = time.time()

        # Đọc dữ liệu trạng thái
        x = data.qpos[0]
        x_dot = data.qvel[0]
        theta = data.qpos[1]
        theta_dot = data.qvel[1]

        # Tính sai số
        error_x = x_ref - x
        error_theta = theta_ref - theta

        # Tính đầu ra PID
        u1 = pid1.compute(error_x, model.opt.timestep)
        u2 = pid2.compute(error_theta, model.opt.timestep)
        u = u1 + u2

        # Lực 
        u = -np.clip(u, -1.0, 1.0)

        # Gán điều khiển vào actuator 
        data.ctrl[0] = u

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))