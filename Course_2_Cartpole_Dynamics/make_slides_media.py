"""
Sinh hình ảnh và video minh hoạ cho slide buổi 2.

KHÔNG phải tài liệu cho học viên — đây là công cụ soạn bài. Chạy một lần,
lấy file trong media/ bỏ vào slide.

    python make_slides_media.py            # sinh tất cả
    python make_slides_media.py gravity    # chỉ một mục
    python make_slides_media.py --list     # xem danh sách

Yêu cầu: mujoco, numpy, matplotlib, và ffmpeg trong PATH (để xuất .mp4).
Không có ffmpeg thì script vẫn chạy, chỉ bỏ qua phần video.
"""

import os
import re
import shutil
import subprocess
import sys
from typing import NamedTuple

os.environ.setdefault("MUJOCO_GL", "egl")   # render không cần cửa sổ

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "media")
BASE_XML = open(os.path.join(HERE, "model.xml"), encoding="utf-8").read()

W, H, FPS = 1280, 720, 60
HAS_FFMPEG = shutil.which("ffmpeg") is not None

# Bảng màu dùng chung cho mọi biểu đồ, để slide nhìn nhất quán.
COLORS = ["#2d6cdf", "#e2703a", "#3fa34d", "#9b59b6", "#c0392b"]

plt.rcParams.update({
    "figure.facecolor": "white", "axes.facecolor": "white",
    "font.size": 13, "axes.titlesize": 16, "axes.labelsize": 13,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.alpha": 0.25, "lines.linewidth": 2.2,
})


# --------------------------------------------------------------------------
# Hạ tầng chung
# --------------------------------------------------------------------------
def sub(xml, pattern, repl, what):
    """Thay thế bằng regex, và BÁO LỖI nếu không khớp.

    Không có kiểm tra này thì một phép thay thế hụt sẽ âm thầm cho ra hình
    minh hoạ trong đó mọi đường trùng khít nhau - trông vẫn "chạy được"
    nhưng nội dung thì sai hoàn toàn.
    """
    out, n = re.subn(pattern, repl, xml, flags=re.S)
    if n != 1:
        raise RuntimeError(
            f"variant({what}): khớp {n} chỗ, cần đúng 1. "
            f"model.xml đã đổi? Cập nhật lại pattern trong variant().")
    return out


def variant(**kw):
    """Tạo biến thể XML từ model.xml gốc. Mỗi tham số là một phép thay thế."""
    xml = BASE_XML
    if "gravity" in kw:
        xml = sub(xml, r'gravity="0 0 -9\.81"',
                  f'gravity="{kw["gravity"]}"', "gravity")
    if "timestep" in kw:
        xml = sub(xml, r'timestep="0\.002"',
                  f'timestep="{kw["timestep"]}"', "timestep")
    if "pole_mass" in kw:
        # Bám vào tên geom rồi mới tới mass, để không đụng nhầm mass của cart.
        xml = sub(xml, r'(name="pole_geom".*?)mass="0\.1"',
                  rf'\g<1>mass="{kw["pole_mass"]}"', "pole_mass")
    if "pole_damping" in kw:
        xml = sub(xml, r'(name="pole_hinge"[^>]*?)damping="0\.01"',
                  rf'\g<1>damping="{kw["pole_damping"]}"', "pole_damping")
    if "pole_len" in kw:
        xml = sub(xml, r'fromto="0 0 0  0 0 0\.6"',
                  f'fromto="0 0 0  0 0 {kw["pole_len"]}"', "pole_len")
    if kw.get("free_rail"):
        xml = sub(xml, r'limited="true" range="-1 1"',
                  'limited="false"', "free_rail")
    if "qpos" in kw:
        xml = sub(xml, r'qpos="0 0\.2"', f'qpos="{kw["qpos"]}"', "qpos")
    return xml


class Roll(NamedTuple):
    t: np.ndarray
    x: np.ndarray
    th: np.ndarray
    energy: np.ndarray
    frames: list


def rollout(xml, duration, ctrl=0.0, record=False):
    """Chạy mô phỏng, trả về quỹ đạo (và frame video nếu record=True)."""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)

    t, x, th, energy = [], [], [], []
    frames = []
    renderer = mujoco.Renderer(model, H, W) if record else None
    next_frame = 0.0

    while data.time < duration:
        t.append(data.time)
        x.append(data.qpos[0])
        th.append(data.qpos[1])
        # Năng lượng cơ học (động + thế). Cần bật enableflags energy trong
        # <option>, nếu không mảng này toàn số 0.
        energy.append(data.energy[0] + data.energy[1])

        if record and data.time >= next_frame:
            mujoco.mj_forward(model, data)   # cập nhật hình học trước khi vẽ
            renderer.update_scene(data, camera="cam_side")
            frames.append(renderer.render())
            next_frame += 1.0 / FPS

        data.ctrl[0] = ctrl(data.time) if callable(ctrl) else ctrl
        mujoco.mj_step(model, data)

    if renderer is not None:
        renderer.close()
    return Roll(np.array(t), np.array(x), np.array(th),
                np.array(energy), frames)


def wrap(a):
    """Quy góc về [-pi, pi] để biểu đồ không nhảy vọt khi pole quay hết vòng."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def save_fig(fig, name):
    path = os.path.join(OUT, f"{name}.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  ✓ {os.path.relpath(path, HERE)}")


def save_video(frames, name):
    """Ghi video qua pipe sang ffmpeg — không cần thư viện encode nào."""
    if not HAS_FFMPEG:
        print(f"  ⚠ bỏ qua {name}.mp4 (không tìm thấy ffmpeg)")
        return
    path = os.path.join(OUT, f"{name}.mp4")
    cmd = ["ffmpeg", "-y", "-loglevel", "error",
           "-f", "rawvideo", "-pix_fmt", "rgb24",
           "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
           "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", path]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for f in frames:
        proc.stdin.write(f.astype(np.uint8).tobytes())
    proc.stdin.close()
    proc.wait()
    print(f"  ✓ {os.path.relpath(path, HERE)}  ({len(frames)} frames)")


def snapshot(xml, name, at=(0.0,), ctrl=0.0, labels=None):
    """Chụp một dãy ảnh tĩnh tại các mốc thời gian — dùng làm hình chuỗi."""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    renderer = mujoco.Renderer(model, H, W)

    shots, targets = [], sorted(at)
    i = 0
    while i < len(targets):
        if data.time >= targets[i]:
            mujoco.mj_forward(model, data)   # nếu thiếu, khung t=0 sẽ đen
            renderer.update_scene(data, camera="cam_side")
            shots.append(renderer.render())
            i += 1
            continue
        data.ctrl[0] = ctrl(data.time) if callable(ctrl) else ctrl
        mujoco.mj_step(model, data)
    renderer.close()

    fig, axes = plt.subplots(1, len(shots), figsize=(5 * len(shots), 3.2))
    axes = np.atleast_1d(axes)
    for ax, img, tt in zip(axes, shots, targets):
        ax.imshow(img)
        ax.set_title(f"t = {tt:.2f} s")
        ax.axis("off")
    if labels:
        fig.suptitle(labels, fontsize=17, y=1.04)
    fig.tight_layout()
    save_fig(fig, name)


# --------------------------------------------------------------------------
# Các mục media
# --------------------------------------------------------------------------
def m_overview():
    """Video mở đầu: cart-pole tự đổ, dùng cho slide giới thiệu hệ."""
    print("[overview] cart-pole tự đổ từ vị trí gần thẳng đứng")
    save_video(rollout(variant(), 6.0, record=True).frames, "00_overview")


def m_coupling():
    """Coupling: đẩy cart, pole phản ứng dù không có actuator nào."""
    print("[coupling] lực chỉ tác động lên cart, pole vẫn chuyển động")
    # Giữ giới hạn ray để cart không chạy khỏi khung hình trong video.
    # Xung lực ngắn sang phải rồi thả: đủ để pole giật lại thấy rõ, nhưng
    # không mạnh tới mức pole quay hết vòng (khi đó đường theta bị wrap và
    # biểu đồ đầy vạch dựng đứng, mất hết ý nghĩa minh hoạ).
    # Pole bắt đầu TREO XUỐNG (theta = pi) - vị trí cân bằng BỀN. Nếu để
    # nó thẳng đứng như keyframe mặc định, chỉ cần đẩy nhẹ là pole đổ hẳn
    # rồi quay tít, đường theta bị wrap và biểu đồ mất sạch ý nghĩa.
    # Treo xuống thì pole chỉ lắc quanh vị trí nghỉ - coupling lộ ra rõ.
    xml = variant(qpos="0 3.14159")
    PUSH, T = 2.0, 0.35
    push = lambda t: PUSH if t < T else 0.0

    save_video(rollout(xml, 4.0, ctrl=push, record=True).frames, "01_coupling")

    r = rollout(xml, 4.0, ctrl=push)
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for a in ax:
        a.axvspan(0, T, color="0.87", zorder=0)
    ax[0].plot(r.t, r.x, color=COLORS[0])
    ax[0].set_ylabel("cart x  (m)")
    ax[0].set_title(f"Lực {PUSH:g} N chỉ tác động lên CART (vùng xám) "
                    "— pole KHÔNG có actuator")
    ax[0].text(T, ax[0].get_ylim()[1] * 0.9, "  thả lực", fontsize=11,
               color="0.4", va="top")
    # Đo độ lệch khỏi vị trí treo, để 0 là "pole đứng yên".
    ax[1].plot(r.t, r.th - np.pi, color=COLORS[1])
    ax[1].axhline(0, color="0.6", lw=1)
    ax[1].set_ylabel("pole lệch khỏi\nvị trí treo  (rad)")
    ax[1].set_xlabel("thời gian (s)")
    ax[1].set_title("…nhưng pole vẫn lắc: đó là coupling", fontsize=13)
    fig.tight_layout()
    save_fig(fig, "01_coupling")


def m_gravity():
    """Thí nghiệm 1: gravity."""
    print("[gravity] -9.81 / -1.62 / 0")
    cases = [("Trái Đất  g = 9.81", "0 0 -9.81"),
             ("Mặt Trăng  g = 1.62", "0 0 -1.62"),
             ("Không trọng lực  g = 0", "0 0 0")]

    fig, ax = plt.subplots(figsize=(9, 5))
    for (lab, g), c in zip(cases, COLORS):
        # 0.75 s: dừng trước khi pole (ở g Trái Đất) quay qua pi và bị wrap.
        r = rollout(variant(gravity=g), 0.75)
        ax.plot(r.t, r.th, label=lab, color=c)
    ax.axhline(0.2, color="0.6", lw=1, ls=":")
    ax.text(0.75, 0.2, " góc ban đầu", va="center", color="0.4", fontsize=11)
    ax.set_xlabel("thời gian (s)")
    ax.set_ylabel("pole θ  (rad)")
    ax.set_title("g = 0 → pole đứng yên: số hạng g(q) biến mất")
    ax.legend()
    save_fig(fig, "02_gravity")

    save_video(rollout(variant(gravity="0 0 0"), 3.0, record=True).frames,
               "02_gravity_zero")


def m_mass():
    """Thí nghiệm 2: khối lượng pole ảnh hưởng gia tốc cart."""
    print("[mass] pole 0.05 / 0.2 / 1.0 kg, cùng lực đẩy 5 N")
    fig, ax = plt.subplots(figsize=(9, 5))
    for m, c in zip(["0.05", "0.2", "1.0"], COLORS):
        r = rollout(variant(pole_mass=m, free_rail=True), 0.6, ctrl=5.0)
        ax.plot(r.t, r.x, label=f"pole {m} kg", color=c)
    ax.set_xlabel("thời gian (s)")
    ax.set_ylabel("cart x  (m)")
    ax.set_title("Cùng lực 5 N — pole càng nặng, cart đi càng chậm")
    ax.legend()
    save_fig(fig, "03_mass")


def m_inertia():
    """Geometry đổi inertia: pole dài gấp đôi đổ chậm hơn hẳn."""
    print("[inertia] pole 0.6 m vs 1.2 m, cùng khối lượng")
    fig, ax = plt.subplots(figsize=(9, 5))
    rows = []
    for L, c in zip(["0.6", "1.2"], COLORS):
        xml = variant(pole_len=L)
        # Dừng ở 0.75 s: pole ngắn đổ tới gần pi lúc ~0.8 s, để lâu hơn thì
        # đường bị wrap thành vạch dựng đứng và che mất thông điệp.
        r = rollout(xml, 0.75)
        ax.plot(r.t, r.th, label=f"pole dài {L} m", color=c)

        model = mujoco.MjModel.from_xml_string(xml)
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pole")
        rows.append((L, model.body_inertia[bid][0]))

    ax.set_xlabel("thời gian (s)")
    ax.set_ylabel("pole θ  (rad)")
    ax.set_title("Cùng mass 0.1 kg — pole dài hơn đổ CHẬM hơn")
    ax.axhline(np.pi / 2, color="0.6", lw=1, ls=":")
    ax.text(0.02, np.pi / 2, " pole nằm ngang", va="bottom", color="0.4",
            fontsize=11)
    sub = " · ".join(f"L={L} m → I={I:.2e}" for L, I in rows)
    ax.text(0.5, -0.22, sub, transform=ax.transAxes, ha="center",
            fontsize=12, color="0.35")
    ax.legend()
    save_fig(fig, "04_inertia")


def m_damping():
    """Thí nghiệm 3: damping, gồm cả chế độ overdamped."""
    print("[damping] 0 / 0.01 / 0.1 / 1.0, thả từ phương ngang")
    # Vẽ |tốc độ góc| chứ không phải góc. Pole quay qua +-pi liên tục nên
    # đường theta bị wrap thành các vạch dựng đứng, nhìn như nhiễu và che
    # mất thông điệp. Biên độ tốc độ góc tắt dần thì thấy ngay bằng mắt.
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for d, c in zip(["0", "0.01", "0.1", "1.0"], COLORS):
        r = rollout(variant(pole_damping=d, qpos="0 1.5708"), 10.0)
        ax.plot(r.t, np.abs(np.gradient(r.th, r.t)), color=c,
                label=f"damping = {d}")

    ax.set_xlabel("thời gian (s)")
    ax.set_ylabel("|tốc độ góc| của pole  (rad/s)")
    ax.set_title("damping = 0 dao động mãi mãi; damping càng lớn càng tắt nhanh")
    ax.annotate("damping = 1.0: không dao động,\npole BÒ chậm xuống (overdamped)",
                xy=(2.2, 0.22), xytext=(3.6, 2.2), fontsize=12, color="0.25",
                arrowprops=dict(arrowstyle="->", color="0.45"))
    ax.legend()
    save_fig(fig, "05_damping")

    for d in ("0", "1.0"):
        r = rollout(variant(pole_damping=d, qpos="0 1.5708"), 6.0, record=True)
        save_video(r.frames, f"05_damping_{d.replace('.', '_')}")


def m_ctrlrange():
    """Thí nghiệm 4: ctrlrange kẹp lệnh điều khiển."""
    print("[ctrlrange] ctrl 1 / 5 / 10 / 50 N (ctrlrange = ±10)")
    fig, ax = plt.subplots(figsize=(9, 5))
    for u, c in zip([1.0, 5.0, 10.0, 50.0], COLORS):
        r = rollout(variant(free_rail=True), 0.4, ctrl=u)
        style = "--" if u == 50.0 else "-"
        ax.plot(r.t, r.x, style, label=f"ctrl = {u:g} N", color=c)
    ax.set_xlabel("thời gian (s)")
    ax.set_ylabel("cart x  (m)")
    ax.set_title("ctrl = 50 TRÙNG KHÍT ctrl = 10 — ctrlrange đã kẹp lệnh lại")
    ax.legend()
    save_fig(fig, "06_ctrlrange")


def m_timestep():
    """Thí nghiệm 5: timestep và sai số tích luỹ."""
    print("[timestep] 0.0005 … 0.1 so với tham chiếu 0.00005")

    # Chỉ lấy 0.5 s: đủ để sai số lộ rõ, mà pole CHƯA quay qua ±pi. Nếu để
    # dài hơn, đường cong sẽ nhảy dựng đứng lúc wrap và trông như lỗi mô
    # phỏng; tệ hơn, sai số đo tại một thời điểm sau khi pole lật vòng sẽ
    # phụ thuộc pha, cho ra dt=0.1 "chính xác hơn" dt=0.05.
    T = 0.5
    ref = rollout(variant(timestep="0.00005", free_rail=True), T, ctrl=8.0)
    ref_t, ref_th = ref.t, ref.th

    fig, ax = plt.subplots(1, 2, figsize=(14, 5))
    ax[0].plot(ref_t, ref_th, color="0.25", lw=3.5,
               label="tham chiếu dt = 0.00005", zorder=10)

    dts, errs = [], []
    for dt, c in zip(["0.0005", "0.002", "0.01", "0.05", "0.1"], COLORS):
        r = rollout(variant(timestep=dt, free_rail=True), T, ctrl=8.0)
        t, th = r.t, r.th
        ax[0].plot(t, th, label=f"dt = {dt}", color=c, alpha=0.9)
        # Sai số RMS trên TOÀN quỹ đạo, nội suy về lưới thời gian tham
        # chiếu. Đo cả đường thay vì một điểm cuối nên không phụ thuộc pha.
        dts.append(float(dt))
        errs.append(float(np.sqrt(np.mean(
            (np.interp(ref_t, t, th) - ref_th) ** 2))))

    ax[0].set_xlabel("thời gian (s)")
    ax[0].set_ylabel("pole θ  (rad)")
    ax[0].set_title("dt càng lớn, quỹ đạo càng lệch khỏi tham chiếu")
    ax[0].legend(fontsize=10)

    ax[1].loglog(dts, errs, "o-", color=COLORS[0], markersize=9)
    for d, e in zip(dts, errs):
        ax[1].annotate(f"{e:.4f}", (d, e), textcoords="offset points",
                       xytext=(8, -4), fontsize=11)
    ax[1].set_xlabel("timestep (s)")
    ax[1].set_ylabel("sai số RMS của θ  (rad)")
    ax[1].set_title("Sai số tăng đơn điệu theo dt — thang log")
    ax[1].grid(True, which="both", alpha=0.25)
    fig.tight_layout()
    save_fig(fig, "07_timestep")


def m_broken():
    """Ảnh so sánh model đúng và model lỗi, cho phần debug."""
    print("[broken] so sánh model.xml và model_broken.xml")
    broken = open(os.path.join(HERE, "model_broken.xml"), encoding="utf-8").read()

    fig, axes = plt.subplots(2, 3, figsize=(15, 7))
    for row, (xml, title) in enumerate(
            [(BASE_XML, "model.xml — ĐÚNG"),
             (broken, "model_broken.xml — SAI")]):
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        renderer = mujoco.Renderer(model, H, W)

        # Bật hiển thị joint + khối tâm, đúng thứ học viên sẽ bật trong viewer.
        opt = mujoco.MjvOption()
        opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
        opt.flags[mujoco.mjtVisFlag.mjVIS_COM] = True

        for col, tt in enumerate([0.0, 0.5, 1.5]):
            while data.time < tt:
                data.ctrl[0] = 4.0
                mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)   # nếu thiếu, khung t=0 sẽ đen
            renderer.update_scene(data, camera="cam_side", scene_option=opt)
            axes[row, col].imshow(renderer.render())
            axes[row, col].set_xticks([])
            axes[row, col].set_yticks([])
            axes[row, col].set_title(f"t = {tt:.1f} s")
            if col == 0:
                # Đặt nhãn hàng ở trục y thay vì nhồi vào set_title, nếu
                # không nó sẽ đè lên hàng hình phía trên.
                axes[row, col].set_ylabel(title, fontsize=15,
                                          fontweight="bold", labelpad=12)
        renderer.close()
    fig.suptitle("Bật Joint + Com để nhìn ra lỗi trục và lỗi vị trí", fontsize=17)
    fig.tight_layout()
    save_fig(fig, "08_broken_vs_correct")


def m_states():
    """Chuỗi ảnh pole đổ — dùng cho slide giải thích qpos/qvel."""
    print("[states] chuỗi ảnh pole đổ theo thời gian")
    snapshot(variant(), "09_state_sequence",
             at=(0.0, 0.3, 0.5, 0.7),
             labels="qpos[1] tăng dần khi pole đổ")


ITEMS = {
    "overview": m_overview, "coupling": m_coupling, "gravity": m_gravity,
    "mass": m_mass, "inertia": m_inertia, "damping": m_damping,
    "ctrlrange": m_ctrlrange, "timestep": m_timestep,
    "broken": m_broken, "states": m_states,
}

if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("-")]

    if "--list" in sys.argv:
        print("Các mục có thể sinh:")
        for k, fn in ITEMS.items():
            print(f"  {k:10s} {fn.__doc__.splitlines()[0]}")
        sys.exit(0)

    os.makedirs(OUT, exist_ok=True)
    if not HAS_FFMPEG:
        print("⚠ Không tìm thấy ffmpeg — chỉ sinh ảnh PNG, bỏ qua video.\n")

    for key in (args or ITEMS):
        if key not in ITEMS:
            print(f"Không có mục '{key}'. Chạy --list để xem danh sách.")
            sys.exit(1)
        ITEMS[key]()

    print(f"\nXong. File nằm trong: {os.path.relpath(OUT, HERE)}/")
