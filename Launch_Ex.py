import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path("07_FramenOrientation/model.xml")
data = mujoco.MjData(model)

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    mujoco.mj_resetData(model, data)

    start = time.time()
    while viewer.is_running():
        step_start = time.time()

        mujoco.mj_step(model, data)
        viewer.sync()

        # Đảm bảo đúng tốc độ thực tế
        time.sleep(max(0, 0.01 - (time.time() - step_start)))