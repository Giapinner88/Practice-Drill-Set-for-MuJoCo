import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import time

model_path = "arm_2D_PID.xml"

# Load XML
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# PID controller
Use_PID = False
Kp = 50 #(default=50.0)
Ki = 0.1 #(default=0.1)
Kd = 1 #(default=1.0)

def quy_dao1(t):
    c = 3.14
    omega = np.pi  
    joint1 = 3.14 + c * np.sin(omega * t)
    joint2 = 4 + c * np.cos(omega * t)
    return joint1, joint2

def quy_dao2(t):
    c = 0.5
    omega = np.pi * 5
    joint1 = c * np.sin(omega * t)
    joint2 = c * np.cos(omega * t)
    return joint1, joint2

def pid_controller(setpoint, measured_value, pre_error, integral, dt, Kp, Ki, Kd):
    error = setpoint - measured_value
    proportional = error
    integral += error*dt
    derivative = (error - pre_error)/dt
    control_value = Kp*proportional + Ki*integral + Kd*derivative
    return control_value, error, integral

def bieu_do1(time_log, qpos_log1, qpos_log2, setpoint_log1, setpoint_log2, use_pid):
    plt.figure(figsize=(12, 8))

    # Joint 1
    plt.subplot(2, 1, 1)
    plt.plot(time_log, qpos_log1, label='Thực tế khớp 1', color='blue')
    plt.plot(time_log, setpoint_log1, label='Mong muốn khớp 1', color='orange', linestyle='--')
    plt.title("Joint 1 - Vị trí")
    plt.ylabel("Vị trí (độ)")
    plt.grid(True)
    plt.legend()

    # Joint 2
    plt.subplot(2, 1, 2)
    plt.plot(time_log, qpos_log2, label='Thực tế khớp 2', color='green')
    plt.plot(time_log, setpoint_log2, label='Mong muốn khớp 2', color='red', linestyle='--')
    plt.title("Joint 2 - Vị trí")
    plt.xlabel("Thời gian (s)")
    plt.ylabel("Vị trí (độ)")
    plt.grid(True)
    plt.legend()

    mode = "PID" if use_pid else "No PID"
    plt.suptitle(f"Kết quả mô phỏng - Chế độ điều khiển: {mode}", fontsize=16, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()

# Run simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    pre_error = [0, 0]
    integral = [0, 0]

    # Log
    time_log = []
    qpos_log1 = []
    qpos_log2 = []
    setpoint_log1 = []
    setpoint_log2 = []

    while viewer.is_running():
        current_time = time.time() - start_time
        dt = model.opt.timestep

        # Joint pos ban đầu pos
        measured_value = [data.qpos[0], data.qpos[1]]
        setpoint = quy_dao2(current_time)

        # Compute control signals
        control_value = []
        if Use_PID:
            for i in range(2):
                ctrl, err, integral[i] = pid_controller(setpoint[i], measured_value[i], pre_error[i], integral[i], dt, Kp, Ki, Kd)
                pre_error[i] = err
                control_value.append(ctrl)
        else:
            control_value = list(setpoint)
        
        # Write log
        time_log.append(current_time)
        qpos_log1.append(np.rad2deg(measured_value[0]))
        qpos_log2.append(np.rad2deg(measured_value[1]))
        setpoint_log1.append(np.rad2deg(setpoint[0]))
        setpoint_log2.append(np.rad2deg(setpoint[1]))

        # Aplly control signals
        data.ctrl[0] = control_value[0]
        data.ctrl[1] = control_value[1]

        # Simulation
        mujoco.mj_step(model, data)
        viewer.sync()

        if current_time >= 2:
            bieu_do1(time_log, qpos_log1, qpos_log2, setpoint_log1, setpoint_log2, Use_PID)
            break