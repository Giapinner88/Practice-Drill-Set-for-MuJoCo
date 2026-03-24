import mujoco
import mujoco.viewer
import time

# 1. Khởi tạo đối tượng Model từ file XML
# mjModel chứa các thông số tĩnh (khối lượng, kích thước hình học, cấu trúc cây động học)
model = mujoco.MjModel.from_xml_path('Rigid Body Fundamentals/08_Camera/model.xml')

# 2. Khởi tạo đối tượng Data
# mjData chứa các trạng thái động thay đổi theo thời gian (vị trí qpos, vận tốc qvel, lực tiếp xúc)
data = mujoco.MjData(model)

# Simulation loop
step = 0
# 3. Khởi chạy Viewer tương tác
print("Khởi chạy môi trường mô phỏng. Nhấn ESC tại cửa sổ viewer để thoát.")
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():

        # Điều khiển box qua lại (oscillate)
        data.ctrl[0] = 0.5 if (step // 500) % 2 == 0 else -0.5
    
        # Hoặc dùng sine wave mượt hơn:
        # data.ctrl[0] = np.sin(step * 0.01)

        # Tiến lên một bước thời gian mô phỏng (mặc định là 0.002s)
        mujoco.mj_step(model, data)
        
        # Cập nhật trạng thái đồ họa lên Viewer
        viewer.sync()

        step += 1
        
        # Giữ vòng lặp chạy theo thời gian thực (real-time)
        time.sleep(model.opt.timestep)