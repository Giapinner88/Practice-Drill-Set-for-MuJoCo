import mujoco
import numpy as np
import scipy.linalg as la

model = mujoco.MjModel.from_xml_path("Kinematics & Actuation/12_Cartpole/model.xml")
data = mujoco.MjData(model)

# Đặt hệ thống vào điểm cân bằng ngược (Upright)
data.qpos[0] = 0.0      # Vị trí xe
data.qpos[1] = np.pi    # Góc con lắc (pi = hướng lên)
data.qvel[:] = 0.0
data.ctrl[:] = 0.0
mujoco.mj_forward(model, data)

original_integrator = model.opt.integrator

model.opt.integrator = 0 # Sử dụng Euler để tính Jacobian chính xác hơn tại điểm cân bằng

# Sử dụng mjd_transitionFD để trích xuất Jacobian A và B
nv = model.nv
nu = model.nu
A_mj = np.zeros((2*nv, 2*nv))
B_mj = np.zeros((2*nv, nu))

# Epsilon, flg_centered=True để sai phân trung tâm (chính xác hơn)
mujoco.mjd_transitionFD(model, data, 1e-6, True, A_mj, B_mj, None, None)

model.opt.integrator = original_integrator # Khôi phục lại integrator ban đầu

print("Ma trận A (MuJoCo):")
print(np.round(A_mj, 3))
print("\nMa trận B (MuJoCo):")
print(np.round(B_mj, 3))

# mjd_transitionFD trả về x_{k+1} = A x_k + B u_k (hệ rời rạc theo timestep mô phỏng)
# Vì vậy cần dùng DARE để thiết kế LQR rời rạc, không dùng ARE liên tục.
# Với lực 10N (underactuated), giảm Q để LQR ít aggressive, cho phép swing-up tiếp tục
Q = np.diag([2.0, 30.0, 1.0, 4.0])
R = np.diag([0.5])

# Kiểm tra nhanh tính điều khiển được
ctrb = np.hstack([B_mj, A_mj @ B_mj, A_mj @ A_mj @ B_mj, A_mj @ A_mj @ A_mj @ B_mj])
ctrb_rank = np.linalg.matrix_rank(ctrb)
print(f"\nControllability rank: {ctrb_rank}/{A_mj.shape[0]}")

P = la.solve_discrete_are(A_mj, B_mj, Q, R)
K = np.linalg.solve(B_mj.T @ P @ B_mj + R, B_mj.T @ P @ A_mj)

print("\nMa trận K (LQR Gain):")
print(K)
# Lưu K để dùng cho file mô phỏng
np.save("lqr_gain.npy", K)