import mujoco
import mujoco.viewer
import time

# 1. Khởi tạo đối tượng Model từ file XML
# mjModel chứa các thông số tĩnh (khối lượng, kích thước hình học, cấu trúc cây động học)
model = mujoco.MjModel.from_xml_path('Rigid Body Fundamentals/05_MeshLoading/model.xml')

# 2. Khởi tạo đối tượng Data
# mjData chứa các trạng thái động thay đổi theo thời gian (vị trí qpos, vận tốc qvel, lực tiếp xúc)
data = mujoco.MjData(model)

# 3. Khởi chạy Viewer tương tác
print("Khởi chạy môi trường mô phỏng. Nhấn ESC tại cửa sổ viewer để thoát.")
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # Tiến lên một bước thời gian mô phỏng (mặc định là 0.002s)
        mujoco.mj_step(model, data)
        
        # Cập nhật trạng thái đồ họa lên Viewer
        viewer.sync()
        
        # Giữ vòng lặp chạy theo thời gian thực (real-time)
        time.sleep(model.opt.timestep)