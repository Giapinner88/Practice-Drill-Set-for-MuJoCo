"""
Course 2 - Đọc state và ghi lệnh điều khiển, KHÔNG có cửa sổ đồ hoạ.

Trả lời ba câu hỏi của phần 80-95 phút:
    1. qpos / qvel chứa gì, ở index nào?
    2. Làm sao tra index đó thay vì ĐOÁN?
    3. Ghi data.ctrl thì qpos/qvel biến đổi ra sao?

Chạy:  python inspect_state.py
"""

import os

import mujoco

XML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.xml")
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

# --- 1. Tra index thay vì đoán ---------------------------------------------
# mj_name2id trả về id của joint theo TÊN. Từ id đó, model.jnt_qposadr cho
# biết joint chiếm ô nào trong qpos, model.jnt_dofadr cho biết ô nào trong
# qvel. Với cart-pole thì mapping đúng là 0 và 1, nhưng thói quen tra tên
# sẽ cứu bạn ở model lớn - nơi free joint chiếm 7 ô qpos nhưng chỉ 6 ô qvel.
print("=" * 62)
print("MAPPING JOINT -> STATE")
print("=" * 62)
for name in ("cart_slide", "pole_hinge"):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    print(f"  {name:12s}  joint id={jid}  "
          f"qpos[{model.jnt_qposadr[jid]}]  qvel[{model.jnt_dofadr[jid]}]")

print(f"\n  nq = {model.nq} (số ô qpos)   "
      f"nv = {model.nv} (số ô qvel)   nu = {model.nu} (số actuator)")

# --- 2. State ban đầu ------------------------------------------------------
mujoco.mj_resetDataKeyframe(model, data, 0)
print(f"\nState ban đầu (keyframe 'tilted'):")
print(f"  qpos = {data.qpos}   qvel = {data.qvel}")

# --- 3. Ghi ctrl rồi step --------------------------------------------------
# Đẩy cart sang phải bằng lực không đổi 5 N trong 1 giây mô phỏng.
FORCE = 3.0
DURATION = 0.6
n_steps = int(DURATION / model.opt.timestep)

print(f"\n{'=' * 62}")
print(f"ĐẨY CART VỚI ctrl = {FORCE} N TRONG {DURATION} s")
print("=" * 62)
print(f"{'t (s)':>7} {'cart x':>9} {'cart v':>9} "
      f"{'pole θ':>9} {'pole ω':>9}")

for i in range(n_steps + 1):
    if i % (n_steps // 10) == 0:
        print(f"{data.time:7.3f} {data.qpos[0]:9.4f} {data.qvel[0]:9.4f} "
              f"{data.qpos[1]:9.4f} {data.qvel[1]:9.4f}")

    data.ctrl[0] = FORCE          # lệnh điều khiển
    mujoco.mj_step(model, data)   # một bước vật lý

# --- 4. Đọc sensor theo tên ------------------------------------------------
# Cách thứ hai, gọn hơn: khai báo <sensor> trong XML rồi đọc data.sensordata.
print(f"\n{'=' * 62}")
print("ĐỌC QUA SENSOR (data.sensordata)")
print("=" * 62)
for name in ("cart_pos", "cart_vel", "pole_angle", "pole_angvel"):
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    print(f"  {name:12s} = {data.sensordata[model.sensor_adr[sid]]: .4f}")

print("\nNhận xét: cart bị đẩy sang phải, pole ngả về phía SAU so với")
print("chiều đẩy. Đó là coupling - lực chỉ tác động lên cart, pole")
print("chuyển động do quán tính và trọng lực, không có actuator nào.")
