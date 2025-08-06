import mujoco
import numpy as np
import math
import itertools

# Load mô hình
model = mujoco.MjModel.from_xml_path("Con_lac_nguoc.xml")
data = mujoco.MjData(model)

# Thông số vật lý
pole_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pole")
geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cpole")
m = model.body_mass[pole_id]
l = model.geom_size[geom_id][1]
g = 9.81

# Reset trạng thái ban đầu
def reset(data):
    data.qpos[:] = [0.0, math.radians(-180)]  # xe ở giữa, cánh tay hướng xuống
    data.qvel[:] = [0.0, 0.0]
    mujoco.mj_forward(model, data)

# Hàm điều khiển swing-up theo năng lượng
def energy_swing_up(theta, theta_dot, x, x_dot, k, kx, kv):
    E = 0.5 * m * (l**2) * (theta_dot**2) + m * g * l * (1 - np.cos(theta))
    E_desired = m * g * l
    E_error = E_desired - E
    u = k * E_error * np.sign(theta_dot * np.cos(theta))
    u -= kx * x + kv * x_dot
    return np.clip(u, -1.0, 1.0)

# Hàm đánh giá hiệu suất toàn diện
def evaluate_gain(k, kx, kv, duration=5.0):
    reset(data)
    total_error = 0
    steps = int(duration / model.opt.timestep)
    
    for _ in range(steps):
        x = data.qpos[0]
        x_dot = data.qvel[0]
        theta = ((data.qpos[1] + np.pi) % (2 * np.pi)) - np.pi
        theta_dot = data.qvel[1]

        u = energy_swing_up(theta, theta_dot, x, x_dot, k, kx, kv)
        data.ctrl[0] = u
        mujoco.mj_step(model, data)

        # Tính lỗi toàn cục (tổng trọng số các lỗi)
        error = (
            1.0 * abs(theta) +        # lỗi góc
            0.3 * abs(theta_dot) +    # lỗi vận tốc góc
            0.2 * abs(x) +            # lỗi vị trí xe
            0.2 * abs(x_dot)          # lỗi vận tốc xe
        )
        total_error += error

    return total_error

# Dải giá trị gain để tìm kiếm
k_values = np.arange(10, 31, 1)
kx_values = np.arange(10, 31, 1)
kv_values = np.arange(10, 31, 1)

# Tìm bộ gain tốt nhất
best_score = float('inf')
best_gains = None

for k, kx, kv in itertools.product(k_values, kx_values, kv_values):
    score = evaluate_gain(k, kx, kv)
    print(f"Test: k={k}, kx={kx}, kv={kv} => Score={score:.2f}")
    if score < best_score:
        best_score = score
        best_gains = (k, kx, kv)

# In kết quả tốt nhất
print("\n✅ Best Gains Found:")
print(f"  k={best_gains[0]}, kx={best_gains[1]}, kv={best_gains[2]}, score={best_score:.2f}")
