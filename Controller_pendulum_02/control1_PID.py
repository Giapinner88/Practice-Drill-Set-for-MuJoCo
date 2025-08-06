import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import time

model_path = "arm1_2D_PID.xml"

# Load XML
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

Use_PID = True

# PID controller (có thể thay đổi)
Kp = 30
Ki = 5
Kd = 2

# Góc mục tiêu (có thể thay đổi)
target_angle = np.deg2rad(90)  # 90 độ

def quy_dao(t):
    omega = np.pi  * 5
    joint = np.sin(omega * t)
    return joint 

def pid_controller(setpoint, measured_value, pre_error, integral, dt, Kp, Ki, Kd):
    error = setpoint - measured_value
    proportional = error
    integral += error*dt
    derivative = (error - pre_error)/dt
    control_value = Kp*proportional + Ki*integral + Kd*derivative
    return control_value, error, integral

def bieu_do1(time_log, qpos_log, setpoint_log, use_pid):
    plt.figure(figsize=(10, 5))
    plt.plot(time_log, qpos_log, label='Thực tế khớp 1', color='blue')
    plt.plot(time_log, setpoint_log, label='Mong muốn khớp 1', color='orange', linestyle='--')
    plt.xlabel("Thời gian (s)")
    plt.ylabel("Góc (độ)")
    plt.grid(True)
    plt.legend()

    mode = "PID" if use_pid else "No PID"
    plt.suptitle(f"Kết quả mô phỏng - Chế độ điều khiển: {mode}", fontsize=16, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()

# Viewer (giao diện trực quan)
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    pre_error = 0
    integral = 0

    # Log
    qpos_log = []
    setpoint_log=[]
    time_log = []

    while viewer.is_running():
        current_time = time.time() - start_time
        dt = model.opt.timestep

        measured_value = data.qpos[0]
        setpoint = quy_dao(current_time)

        # Compute control signals
        control_value = []
        if Use_PID:
            ctrl, err, integral = pid_controller(setpoint, measured_value, pre_error, integral, dt, Kp, Ki, Kd)
            pre_error = err
            control_value.append(ctrl)
        else:
            control_value.append(setpoint)

        # Write log
        qpos_log.append(np.rad2deg(measured_value))   # thực tế
        setpoint_log.append(np.rad2deg(setpoint))     # mong muốn
        time_log.append(current_time)


        # Aplly control signals
        data.ctrl[0] = control_value[0]

        mujoco.mj_step(model, data)
        viewer.sync()

        if current_time >= 5:
            bieu_do1(time_log, qpos_log, setpoint_log, Use_PID)
            break