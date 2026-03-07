# Bài 02: Vật thể tĩnh (Static Object)

## 1. Cơ sở Lý thuyết
Trong mô phỏng, chúng ta cần phân biệt rõ hai khái niệm:
- **Body (Khung xương):** Là thực thể mang các tính chất vật lý như khối lượng và quán tính.
- **Geom (Hình khối):** Là thực thể mang tính chất hình học (dùng để hiển thị và tính toán va chạm).

Một sai lầm phổ biến của người mới bắt đầu là nhầm tưởng rằng cứ khai báo `<geom>` là vật thể sẽ tự động rơi hoặc chuyển động. Thực tế, nếu `<geom>` được đặt trực tiếp bên trong `<worldbody>`, nó sẽ bị "khóa" vào hệ quy chiếu quán tính, trở thành một phần của môi trường tĩnh.

## 2. Cấu trúc mô hình (model.xml)

```xml
<mujoco model="static_box">
    <worldbody>
        <geom type="box" size="0.5 0.5 0.5" rgba="0.8 0.2 0.2 1"/>
    </worldbody>
</mujoco>
```

## 3. Phân tích Thẻ (Tag Analysis)
- `<geom>`: Thẻ định nghĩa hình khối hình học.

    - Thuộc tính `type="box"`: MuJoCo hỗ trợ các hình khối cơ bản (primitives) giúp tối ưu hóa tốc độ tính toán va chạm so với việc dùng lưới mesh phức tạp.

    - Thuộc tính `size="0.5 0.5 0.5"`: Lưu ý rằng đối với box, các giá trị này là half-extents (một nửa chiều dài mỗi cạnh). Vậy khối hộp này thực tế có kích thước $1\time 1\time 1$ đơn vị chiều dài.

    - Thuộc tính `rgba`: Định nghĩa diện mạo. `Alpha = 1` là vật thể đặc, `Alpha < 1` sẽ làm vật thể trong suốt.

**Vấn đề trao đổi:**

**Câu hỏi đặt ra:** Tại sao khối hộp này không rơi dù có trọng lực mặc định?

**Trả lời:** Vì nó chưa có bậc tự do (Degrees of Freedom). Trong MuJoCo, vật thể chỉ chuyển động khi nó được gắn vào một <body> và body đó phải có ít nhất một <joint>. Ở đây, geom nằm trực tiếp trong worldbody (vật thể có khối lượng vô hạn), nên nó đứng yên vĩnh viễn.