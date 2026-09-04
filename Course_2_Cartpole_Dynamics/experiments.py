"""
Course 2 - Năm thí nghiệm của phần 95-110 phút, chạy tự động.

Mỗi thí nghiệm đổi ĐÚNG MỘT tham số rồi in ra hệ quả bằng số, để so sánh
được thay vì chỉ "nhìn thấy khang khác". Model gốc không bị sửa: ta nạp
XML thành chuỗi rồi thay tham số bằng regex trước khi biên dịch.

Chạy:  python experiments.py
       python experiments.py 3      # chỉ chạy thí nghiệm số 3
"""

import math
import os
import re
import sys

import mujoco

XML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.xml")
with open(XML_PATH, encoding="utf-8") as f:
    BASE_XML = f.read()


def run(xml, duration=3.0, ctrl=0.0, free_rail=False):
    """Chạy mô phỏng không đồ hoạ, trả về state cuối và vài chỉ số dẫn xuất.

    free_rail=True gỡ giới hạn ray. Cần thiết khi ta muốn đo phản ứng của
    cart: nếu cart đâm vào biên +-1 m thì mọi cấu hình đều cho x = 1.0 và
    phép so sánh trở nên vô nghĩa.
    """
    if free_rail:
        xml = xml.replace('limited="true" range="-1 1"', 'limited="false"')

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)

    peak = 0.0
    tail = []          # |ω| trong 1 s cuối, để đo dao động CÒN LẠI
    while data.time < duration:
        data.ctrl[0] = ctrl
        mujoco.mj_step(model, data)
        # |θ| đo theo góc lệch khỏi phương thẳng đứng, quy về [-pi, pi].
        # Không quy chuẩn thì pole quay vài vòng sẽ cho |θ|max = 18 rad,
        # một con số không nói lên biên độ dao động.
        peak = max(peak, abs(wrap(data.qpos[1])))
        if data.time > duration - 1.0:
            tail.append(abs(data.qvel[1]))

    # ω tức thời ở đúng thời điểm cuối là con số may rủi: nó phụ thuộc pha
    # dao động. Trung bình |ω| trên 1 s cuối mới cho biết hệ còn đung đưa
    # hay đã đứng yên.
    return {
        "x": data.qpos[0], "v": data.qvel[0],
        "theta": wrap(data.qpos[1]), "omega": data.qvel[1],
        "peak_theta": peak,
        "tail_speed": sum(tail) / len(tail) if tail else 0.0,
    }


def wrap(angle):
    """Quy góc về khoảng [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def show(label, r):
    print(f"  {label:<14} x={r['x']:+7.4f} m   θ={r['theta']:+7.4f} rad   "
          f"|θ|max={r['peak_theta']:6.4f}   ω={r['omega']:+8.4f} rad/s")


def header(n, title, question):
    print(f"\n{'=' * 72}\nTHÍ NGHIỆM {n} — {title}\n{'=' * 72}")
    print(f"Câu hỏi: {question}\n")


# --- 1. Gravity ------------------------------------------------------------
def exp_gravity():
    header(1, "GRAVITY", "Thành phần nào của dynamics biến mất khi g = 0?")
    for g in ("0 0 -9.81", "0 0 -1.62", "0 0 0"):
        xml = BASE_XML.replace('gravity="0 0 -9.81"', f'gravity="{g}"')
        show(f"g = {g}", run(xml, duration=0.5))
    print("\n  g = 0: pole giữ nguyên góc lệch, không có moment kéo nó xuống.")
    print("  Số hạng g(q) trong M(q)q̈ + C(q,q̇)q̇ + g(q) = Bu bằng 0.")


# --- 2. Pole mass ----------------------------------------------------------
def exp_mass():
    header(2, "POLE MASS", "Pole nặng hơn thì cart phản ứng thế nào?")
    for m in ("0.05", "0.2", "1.0"):
        xml = re.sub(r'(name="pole_geom".*?)mass="0\.1"', rf'\g<1>mass="{m}"',
                     BASE_XML, flags=re.S)
        # free_rail: nếu để giới hạn +-1 m thì cả ba đều đâm biên, x giống
        # hệt nhau và không so sánh được gì.
        show(f"mass = {m} kg", run(xml, duration=0.5, ctrl=5.0, free_rail=True))
    print("\n  Cùng lực 5 N trong 0.5 s: pole càng nặng, cart đi được càng")
    print("  ít - lực phải gia tốc tổng khối lượng cart + pole (F = ma).")


# --- 3. Damping ------------------------------------------------------------
def exp_damping():
    header(3, "DAMPING", "Damping ảnh hưởng dao động của pole ra sao?")
    # Thả pole từ gần phương NGANG (θ = pi/2) rồi để nó đung đưa quanh
    # phương thẳng đứng hướng xuống. Đây là dao động thật, đo tắt dần mới
    # có nghĩa - khác với thả từ 0.2 rad ở đỉnh, nơi pole lật hẳn một vòng.
    for d in ("0", "0.01", "0.1", "1.0"):
        xml = BASE_XML.replace(
            'name="pole_hinge" type="hinge" axis="0 1 0" damping="0.01"',
            f'name="pole_hinge" type="hinge" axis="0 1 0" damping="{d}"')
        xml = xml.replace('qpos="0 0.2"', 'qpos="0 1.5708"')
        r = run(xml, duration=8.0)
        print(f"  damping = {d:<6} |θ|max={r['peak_theta']:6.4f} rad   "
              f"trung bình |ω| trong 1 s cuối = {r['tail_speed']:7.4f} rad/s")
    print("\n  Cột cuối là thước đo dao động CÒN LẠI sau 8 s:")
    print("    damping = 0     -> vẫn đung đưa mãi, không mất năng lượng;")
    print("    damping tăng    -> con số giảm dần về 0, pole đứng im.")
    print("  Damping là moment ngược chiều vận tốc khớp: τ = -d·q̇, nên nó")
    print("  luôn rút năng lượng khỏi hệ.")
    print("\n  Để ý damping = 1.0 lại nhỉnh hơn damping = 0.1 một chút. Không")
    print("  phải sai số: damping quá lớn thì pole không còn dao động nữa mà")
    print("  BÒ chậm về vị trí thấp nhất, và sau 8 s nó vẫn đang bò. Đây là")
    print("  chế độ overdamped - nhiều damping không đồng nghĩa về đích nhanh.")


# --- 4. Input force --------------------------------------------------------
def exp_force():
    header(4, "INPUT FORCE", "ctrl lớn hơn thì cart và pole phản ứng thế nào?")
    for u in (1.0, 5.0, 10.0, 50.0):
        show(f"ctrl = {u} N", run(BASE_XML, duration=0.3, ctrl=u,
                                 free_rail=True))
    print("\n  Trong 0.3 s đầu, x xấp xỉ tỉ lệ THUẬN với ctrl: lực gấp 5 thì")
    print("  quãng đường gấp gần 5 (x = ½at², a = F/m).")
    print("  Chú ý dòng cuối: ctrl = 50 cho kết quả HỆT ctrl = 10, vì")
    print("  ctrlrange='-10 10' đã kẹp lệnh lại ở 10 N.")


# --- 5. Timestep -----------------------------------------------------------
def exp_timestep():
    header(5, "TIMESTEP", "Timestep lớn thì mô phỏng còn đáng tin không?")
    # So sánh trên 1 s, với lực đẩy để hệ thực sự vận động mạnh.
    ARGS = dict(duration=1.0, ctrl=8.0, free_rail=True)
    ref = run(BASE_XML.replace('timestep="0.002"', 'timestep="0.00005"'), **ARGS)
    print(f"  Tham chiếu (dt = 0.00005): x = {ref['x']:+.4f} m, "
          f"θ = {ref['theta']:+.4f} rad\n")

    for dt in ("0.0005", "0.002", "0.01", "0.05", "0.1"):
        xml = BASE_XML.replace('timestep="0.002"', f'timestep="{dt}"')
        r = run(xml, **ARGS)
        show(f"dt = {dt}", r)
        err = abs(wrap(r["theta"] - ref["theta"]))
        print(f"  {'':<14} sai lệch θ so với tham chiếu: {err:.4f} rad")

    print("\n  Sai lệch lớn dần theo dt, và KHÔNG tuyến tính: từ 0.0005 lên")
    print("  0.01 sai lệch còn dưới 0.01 rad, nhưng tới dt = 0.1 thì vọt lên")
    print("  hơn 0.5 rad - gần 30 độ, một mô phỏng không còn dùng được.")
    print("  dt nhỏ = chính xác + tốn CPU. Đó là đánh đổi bạn phải chọn ở")
    print("  mọi dự án MuJoCo.")


EXPERIMENTS = [exp_gravity, exp_mass, exp_damping, exp_force, exp_timestep]

if __name__ == "__main__":
    if len(sys.argv) > 1:
        EXPERIMENTS[int(sys.argv[1]) - 1]()
    else:
        for fn in EXPERIMENTS:
            fn()
        print(f"\n{'=' * 72}")
        print("Mỗi thí nghiệm đổi đúng một tham số. Đó là cách duy nhất để")
        print("biết chắc tham số nào gây ra thay đổi nào.")
