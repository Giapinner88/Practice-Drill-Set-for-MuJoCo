# Bài 03: Thiết lập Trường lực và Mặt phẳng Tham chiếu (Ground & Gravity)

## 1. Cơ sở Lý thuyết
Một môi trường mô phỏng có ý nghĩa vật lý (physically meaningful) cần tối thiểu hai yếu tố nền tảng trước khi đưa bất kỳ cấu trúc động lực học nào vào:
1.  **Trường lực toàn cục (Global Force Field):** Trong hầu hết các bài toán trên Trái Đất, đây là gia tốc trọng trường. Nó tác dụng một lực phân bố đều lên khối tâm của mọi vật thể có khối lượng trong hệ thống theo phương trình $F_g = m \cdot g$.
2.  **Biên giới không gian (Spatial Boundary):** Một mặt phẳng tham chiếu vô hạn (ground plane) để chặn các vật thể không rơi tự do mãi mãi theo trục $-Z$, đồng thời tạo ra mặt tiếp xúc cơ sở để tính toán phản lực pháp tuyến (normal forces).

Ngoài ra, ở mức độ quản lý tài nguyên tính toán (memory management), trình mô phỏng cần biết trước giới hạn kích thước của các ma trận trạng thái (ví dụ: số lượng va chạm tối đa có thể xảy ra) để cấp phát bộ nhớ tĩnh (static allocation) trong ngôn ngữ C.

## 2. Cấu trúc Mô hình (Giản lược)
Để can thiệp vào các tham số toàn cục của bộ giải (solver) và bộ nhớ, chúng ta sử dụng nhóm thẻ cấu hình:

```xml
<compiler angle="degree"/>
<option gravity="0 0 -9.81" timestep="0.002"/>
<size nconmax="100"/>
```

Để thiết lập mặt đất, chúng ta định nghĩa một <geom> loại plane bên trong hệ quy chiếu gốc:

```xml
<geom name="ground" type="plane" size="10 10 0.1" rgba="0.9 0.9 0.9 1"/>
```

## 3. Phân tích Thẻ (Tag Analysis)
- `<compiler>`: Xác định các quy tắc biên dịch trước khi mô hình được đưa vào bộ giải.
    - `angle="degree"`: Chuyển đổi hệ đo lường góc từ radian (mặc định của toán học) sang độ (dễ hình dung hơn đối với kỹ sư khi thiết lập tọa độ).

- `<option>`: Chứa các siêu tham số (hyperparameters) của môi trường vật lý.

    - `gravity="0 0 -9.81"`: Vector gia tốc trọng trường g​ theo 3 trục (X,Y,Z).

    - `timestep="0.002"`: Bước thời gian tích phân $\delta t$. Giá trị càng nhỏ, mô phỏng càng chính xác nhưng tiêu tốn nhiều thời gian tính toán hơn.

- `<size>`: Khai báo trước giới hạn bộ nhớ (Pre-allocation).

    - `nconmax="100"`: Chỉ định số điểm tiếp xúc (contacts) tối đa mà engine được phép xử lý cùng lúc. Nếu số lượng va chạm vượt qua ngưỡng này, MuJoCo sẽ bỏ qua các va chạm dư thừa, dẫn đến hiện tượng vật thể xuyên thấu (penetration).

- `<geom type="plane">`: Mặt phẳng vô hạn.

    - Khác với hình hộp (box), kích thước `size="10 10 0.1"` của plane mang ý nghĩa: độ dài render theo trục X, độ dài render theo trục Y, và khoảng cách không gian lưới (grid spacing). Về mặt vật lý va chạm, mặt phẳng này mở rộng vô tận.

**Vấn đề trao đổi**

Hãy quan sát kỹ đoạn mã trong tag `option` gia tốc trọng trường đã được cấu hình rõ ràng là `-9.81` theo trục Z, nhưng khối hộp `floating_box` được đặt lơ lửng ở độ cao `pos="0 0 2"` vẫn sẽ không rơi xuống mặt đất khi bạn chạy mô phỏng. 

Dựa vào nền tảng của Bài 02, điều kiện bắt buộc nào về mặt cấu trúc (liên quan đến khối lượng và bậc tự do) đang bị thiếu để gia tốc trọng trường $\vec{g}$ có thể chuyển hóa thành lực động lực học tác dụng lên khối hộp này? Đồng thời, trong các phiên bản MuJoCo mới nhất, trình biên dịch có xu hướng tự động nội suy thẻ `<size>`. Việc chúng ta tiếp tục duy trì khai báo cứng (hard-code) `nconmax` ở đây liệu có phải là một thói quen lập trình an toàn, hay nó đang tự giới hạn năng lực của mô hình nếu sau này bạn đưa vào một bầy đàn robot (swarm robotics) có hàng nghìn điểm tiếp xúc?