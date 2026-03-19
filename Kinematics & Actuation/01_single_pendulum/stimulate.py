import mujoco
import mujoco.viewer
import time
import numpy as np

def main():
    model = mujoco.MjModel.from_xml_path("Kinematics & Actuation/01_single_pendulum/model.xml")
    data = mujoco.MjData(model)

    # Khởi tạo vị trí ban đầu: Nghiêng 1 góc 1.0 rad
    data.qpos[0] = 1.0  
    data.qvel[0] = 0.0  
    mujoco.mj_forward(model, data)

    # --- THÔNG SỐ BỘ ĐIỀU KHIỂN PD ---
    kp = 5.0  # Độ cứng lò xo ảo
    kd = 0.5  # Độ cản nhớt ảo
    theta_ref = 0.0  # Góc mục tiêu (cân bằng bền)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        
        while viewer.is_running():
            step_start = time.time()
            
            # 1. Đọc trạng thái (Sensor Reading)
            theta = data.sensor("sens_theta").data[0]
            theta_dot = data.sensor("sens_theta_dot").data[0]
            
            # 2. Tính toán luật điều khiển PD (Control Law)
            error = theta - theta_ref
            tau = -kp * error - kd * theta_dot
            
            # 3. Áp dụng giới hạn lực (Saturation)
            # MuJoCo đã có ctrlrange="-3.0 3.0" trong XML, 
            # nhưng việc clip ở phần mềm giúp thuật toán an toàn hơn trước khi gửi xuống firmware thực tế.
            tau = np.clip(tau, -3.0, 3.0)
            
            # 4. Xuất tín hiệu ra Actuator
            data.ctrl[0] = tau
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()