import mujoco
import mujoco.viewer
import numpy as np
import time
import math

# Load mô hình từ XML
model = mujoco.MjModel.from_xml_path("Con_lac_nguoc.xml")
data = mujoco.MjData(model)

# Trạng thái ban đầu
data.qpos[0] = 0
data.qpos[1] = math.radians(-180)
data.qvel[0] = 0
data.qvel[1] = 0

# Tham số mục tiêu 
x_ref = 0.0
theta_ref = 0.0

# Thông số PID điều khiển vị trí
kp_x = 0.5
kd_x = 5
ki_x = 0                  # Thường để 0

# Thông số PID điều khiển góc
kp_theta = 50
kd_theta = 5
ki_theta = 0                  # Thường để 0

# Thông số vật lý
pole_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pole")
geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cpole")

m = model.body_mass[pole_id]        # khối lượng con lắc (kg)
l = model.geom_size[geom_id][1]     # chiều dài con lắc (m)
g = 9.81                            # m/s^2

# Thông số điều khiển
k = 10
kx = 10
kv = 10

# Ngưỡng để chuyển sang PID
goc_PID = math.radians(10)
use_pid = False

# === Giai đoạn 1 ====
# Điều khiển swing-up
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
        return np.clip(u, -1.0, 1.0)
    
# Tạo bộ điều khiển swing-up
swing_up = Swing_up(m, l, g, k, kx, kv)

# === Giai đoạn 2 ===
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

# Mô phỏng với Viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    time.sleep(2)
    while viewer.is_running():
        current_time = time.time() - start
        step_start = time.time()
        if current_time > 100:
            break

        # Lấy trạng thái hiện tại
        x = data.qpos[0]
        x_dot = data.qvel[0]
        theta = data.qpos[1]
        theta = ((theta + np.pi) % (2 * np.pi)) - np.pi
        theta_dot = data.qvel[1]

        # Kiểm tra điều kiện chuyển sang PID
        if abs(theta) <= goc_PID:
            use_pid = True

        # Điều khiển
        if not use_pid:
            u = swing_up.energy(x, x_dot, theta, theta_dot)
        else:
            error_x = x_ref - x
            error_theta = theta_ref - theta
            u1 = pid1.compute(error_x, model.opt.timestep)
            u2 = pid2.compute(error_theta, model.opt.timestep)
            u = -(u1 + u2)
            u = np.clip(u, -1.0, 1.0)

        # In log theo thời gian
        print(f"[{current_time:6.2f}s] x={x:+.3f}, x_dot={x_dot:+.3f}, "
              f"theta={theta:+.3f} rad, theta_dot={theta_dot:+.3f} rad/s, "
              f"u={u:+.3f}, mode={'PID' if use_pid else 'Swing-up'}")

        # Gửi điều khiển
        data.ctrl[0] = u

        # Bước mô phỏng
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))