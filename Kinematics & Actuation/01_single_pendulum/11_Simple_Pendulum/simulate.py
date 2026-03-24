import mujoco
import mujoco.viewer
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def main():
    model = mujoco.MjModel.from_xml_path(
        "Kinematics & Actuation/01_single_pendulum/11_Simple_Pendulum/model.xml"
    )
    data = mujoco.MjData(model)

    # --- Trạng thái ban đầu ---
    data.qpos[0] = 0.1   # Nhiễu nhỏ để thoát điểm kỳ dị
    data.qvel[0] = 0.0
    mujoco.mj_forward(model, data)

    # --- Thông số vật lý & điều khiển ---
    m    = 1.0
    l    = 1.0
    g    = 9.81
    b    = 0.01      # Damping (khớp với XML)
    k    = 0.5       # Gain bơm năng lượng
    tmax = 3.0       # Giới hạn torque (N·m)

    E_desired = m * g * l   # Năng lượng tại homoclinic orbit

    sim_duration = 30.0

    # --- Log ---
    log_theta     = []
    log_theta_dot = []
    log_E         = []
    log_time      = []

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and data.time < sim_duration:
            step_start = time.time()

            theta     = data.sensor("sens_theta").data[0]
            theta_dot = data.sensor("sens_theta_dot").data[0]

            # Năng lượng cơ học
            E       = 0.5 * m * (l**2) * theta_dot**2 - m * g * l * np.cos(theta)
            E_tilde = E - E_desired

            # Luật điều khiển định hình năng lượng (+ bù damping)
            tau = -k * theta_dot * E_tilde + b * theta_dot
            data.ctrl[0] = np.clip(tau, -tmax, tmax)

            log_theta.append(theta)
            log_theta_dot.append(theta_dot)
            log_E.append(E)
            log_time.append(data.time)

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))

    # --- Vẽ biểu đồ ---
    _plot(log_theta, log_theta_dot, log_E, log_time, E_desired, k)


def _plot(log_theta, log_theta_dot, log_E, log_time, E_desired, k):
    isDark = False   # Đổi thành True nếu dùng dark background

    BG   = "#ffffff" if not isDark else "#1a1a18"
    AX   = "#f5f4f0" if not isDark else "#222220"
    GR   = "#eeece8" if not isDark else "#2a2a28"
    TXT  = "#3d3d3a" if not isDark else "#c2c0b6"
    MTXT = "#73726c" if not isDark else "#88877f"

    C_TRAJ  = "#378ADD"
    C_START = "#E24B4A"
    C_END   = "#1D9E75"
    C_HCLI  = "#EF9F27"
    C_E     = "#1D9E75"
    C_EREF  = "#E24B4A"

    fig = plt.figure(figsize=(12, 5), facecolor=BG)
    gs  = gridspec.GridSpec(1, 2, figure=fig, wspace=0.35)

    # ── Biểu đồ 1: Phase Portrait ──────────────────────────────────────
    ax1 = fig.add_subplot(gs[0])
    ax1.set_facecolor(AX)
    for spine in ax1.spines.values():
        spine.set_edgecolor(GR)

    ax1.plot(log_theta, log_theta_dot,
             color=C_TRAJ, linewidth=1.0, alpha=0.85, label="Trajectory")

    ax1.plot(log_theta[0], log_theta_dot[0],
             "o", color=C_START, markersize=7, zorder=5, label="Start")
    ax1.plot(log_theta[-1], log_theta_dot[-1],
             "o", color=C_END,   markersize=6, zorder=5, label="End")

    # Homoclinic saddle points ±π
    for hp in [-np.pi, np.pi]:
        ax1.plot(hp, 0, "o",
                 markerfacecolor="none", markeredgecolor=C_HCLI,
                 markeredgewidth=1.8, markersize=9, zorder=6,
                 label="Homoclinic ±π" if hp == np.pi else "")

    ax1.axhline(0, color=MTXT, linewidth=0.5, linestyle="--", alpha=0.5)
    ax1.axvline(0, color=MTXT, linewidth=0.5, linestyle="--", alpha=0.5)

    ax1.set_xlabel("θ (rad)",   color=TXT, fontsize=11)
    ax1.set_ylabel("θ̇ (rad/s)", color=TXT, fontsize=11)
    ax1.set_title(f"Phase portrait — swing-up (k={k})",
                  color=TXT, fontsize=12, fontweight="normal", pad=10)
    ax1.tick_params(colors=MTXT, labelsize=9)
    ax1.grid(True, color=GR, linewidth=0.7)

    leg1 = ax1.legend(fontsize=9, framealpha=0.85,
                      facecolor=BG, edgecolor=GR, labelcolor=TXT)

    # ── Biểu đồ 2: Energy Pumping ──────────────────────────────────────
    ax2 = fig.add_subplot(gs[1])
    ax2.set_facecolor(AX)
    for spine in ax2.spines.values():
        spine.set_edgecolor(GR)

    ax2.plot(log_time, log_E,
             color=C_E, linewidth=1.2, label="E(t)")
    ax2.axhline(E_desired,
                color=C_EREF, linewidth=1.2, linestyle="--",
                label=f"E_desired = mgl = {E_desired:.2f} J")

    ax2.set_xlabel("t (s)",   color=TXT, fontsize=11)
    ax2.set_ylabel("E (J)",   color=TXT, fontsize=11)
    ax2.set_title("Energy pumping over time",
                  color=TXT, fontsize=12, fontweight="normal", pad=10)
    ax2.tick_params(colors=MTXT, labelsize=9)
    ax2.grid(True, color=GR, linewidth=0.7)

    leg2 = ax2.legend(fontsize=9, framealpha=0.85,
                      facecolor=BG, edgecolor=GR, labelcolor=TXT)

    plt.savefig("swingup_analysis.png", dpi=150, bbox_inches="tight",
                facecolor=BG)
    plt.show()


if __name__ == "__main__":
    main()