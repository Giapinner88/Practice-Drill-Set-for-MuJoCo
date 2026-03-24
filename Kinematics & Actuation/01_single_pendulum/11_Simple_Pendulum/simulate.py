import mujoco
import mujoco.viewer
import time
import numpy as np
import matplotlib.pyplot as plt

def main():
    model = mujoco.MjModel.from_xml_path("Kinematics & Actuation/01_single_pendulum/11_Simple_Pendulum/model.xml")
    data = mujoco.MjData(model)

    # 1. Khởi tạo: Cấp một nhiễu nhỏ (nudge) để thoát khỏi điểm kỳ dị tĩnh
    data.qpos[0] = 0.1  # Lệch 0.1 rad so với vị trí cân bằng bền
    data.qvel[0] = 0.0  
    mujoco.mj_forward(model, data)

    # --- THÔNG SỐ TOÁN HỌC & ĐIỀU KHIỂN ---
    m = 1.0     # Khối lượng
    l = 1.0     # Chiều dài
    g = 9.81    # Gia tốc trọng trường
    b = 0.01    # Damping từ XML
    
    # Năng lượng tại quỹ đạo đồng tà (homoclinic orbit) - Vị trí lộn ngược
    E_desired = m * g * l
    
    k = 0.5     # Hệ số khuếch đại (Gain) bơm năng lượng
    
    # Cấu trúc log dữ liệu
    log_theta, log_theta_dot, log_E, log_time = [], [], [], []
    sim_duration = 30.0  # Chạy 30s để đủ thời gian văng lên đỉnh

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        
        while viewer.is_running() and data.time < sim_duration:
            step_start = time.time()
            
            # Trích xuất trạng thái
            theta = data.sensor("sens_theta").data[0]
            theta_dot = data.sensor("sens_theta_dot").data[0]
            
            # 2. Tính toán Năng lượng hệ thống
            E = 0.5 * m * (l**2) * (theta_dot**2) - m * g * l * np.cos(theta)
            E_tilde = E - E_desired
            
            # 3. Luật điều khiển định hình năng lượng (có bù damping)
            tau = -k * theta_dot * E_tilde + b * theta_dot
            
            # Giới hạn Underactuated
            data.ctrl[0] = np.clip(tau, -3.0, 3.0)
            
            # Logging
            log_theta.append(theta)
            log_theta_dot.append(theta_dot)
            log_E.append(E)
            log_time.append(data.time)
            
            mujoco.mj_step(model, data)
            viewer.sync()

            time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))

    # --- VẼ BIỂU ĐỒ KHẢO SÁT CHỨNG MINH LÝ THUYẾT ---
    plt.figure(figsize=(12, 5))
    
    # Biểu đồ 1: Phase Portrait
    plt.subplot(1, 2, 1)
    plt.plot(log_theta, log_theta_dot, 'b-', linewidth=1)
    plt.plot(log_theta[0], log_theta_dot[0], 'ro', label='Start')
    plt.axhline(0, color='k', linewidth=0.5)
    plt.title('Phase Portrait (Swing-up Trajectory)')
    plt.xlabel('Theta (rad)')
    plt.ylabel('Theta_dot (rad/s)')
    plt.grid(True)
    plt.legend()

    # Biểu đồ 2: Energy Tracking
    plt.subplot(1, 2, 2)
    plt.plot(log_time, log_E, 'g-', label='Current Energy E(t)')
    plt.axhline(E_desired, color='r', linestyle='--', label='Desired Energy (mgl)')
    plt.title('Energy Pumping Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Energy (J)')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()