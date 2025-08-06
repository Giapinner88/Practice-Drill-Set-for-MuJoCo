import mujoco
import mujoco.viewer
import numpy as np
import time
import math

# Load mô hình từ XML
model = mujoco.MjModel.from_xml_path("Con_lac_nguoc.xml")
data = mujoco.MjData(model)

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

# Trạng thái ban đầu
data.qpos[0] = 0
data.qpos[1] = math.radians(-180)
data.qvel[0] = 0
data.qvel[1] = 0

# Ngưỡng để chuyển sang PID
goc_PID = math.radians(10)

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
    
# Mô phỏng với Viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    time.sleep(2)
    while viewer.is_running():
        step_start = time.time()

        # Lấy trạng thái hiện tại
        x = data.qpos[0]
        x_dot = data.qvel[0]
        theta = data.qpos[1]
        theta = ((theta + np.pi) % (2 * np.pi)) - np.pi
        theta_dot = data.qvel[1]

        current_time = time.time() - start

        # Kiểm tra điều kiện chuyển sang PID
        if abs(theta) <= goc_PID:
            print("-------")
            print(theta)
            print("--------")

        # Điều khiển
        u = swing_up.energy(x, x_dot, theta, theta_dot)

        # In log theo thời gian
        print(f"[{current_time:6.2f}s] x={x:+.3f}, x_dot={x_dot:+.3f}, "
              f"theta={theta:+.3f} rad, theta_dot={theta_dot:+.3f} rad/s.")

        # Gửi điều khiển
        data.ctrl[0] = u

        # Bước mô phỏng
        mujoco.mj_step(model, data)
        viewer.sync()

        # Dừng sau 20 giây
        if current_time > 100:
            break

        # Đảm bảo tốc độ thời gian thực
        time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))