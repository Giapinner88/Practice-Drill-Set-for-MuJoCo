# Bài 09: Thiết lập Nguồn sáng (Light Sources)

## 1. Cơ sở Lý thuyết
Trong môi trường thực tế, ánh sáng quyết định cách chúng ta (và hệ thống thị giác máy tính) nhận thức hình khối, độ sâu và khoảng cách thông qua hiện tượng đổ bóng (shadows) và phản xạ (reflections). Nếu môi trường mô phỏng không được chiếu sáng đúng cách, hình ảnh thu được từ camera sẽ phẳng lỳ (flat shading), làm mất đi các đặc trưng không gian quan trọng.

MuJoCo hỗ trợ ba loại nguồn sáng cơ bản:
1.  **Directional Light (Ánh sáng song song/Mặt trời):** Nguồn sáng đặt ở vô cực, tỏa ra các tia sáng song song theo một hướng xác định. Độ sáng không bị suy giảm theo khoảng cách. Thường dùng để giả lập ánh sáng Mặt Trời toàn cục.
2.  **Point Light (Ánh sáng điểm/Bóng đèn tròn):** Nguồn sáng phát ra từ một điểm cụ thể và tỏa đều ra mọi hướng. Độ sáng suy giảm (attenuation) theo bình phương khoảng cách.
3.  **Spotlight (Ánh sáng rọi/Đèn pin):** Tương tự Point Light nhưng bị giới hạn góc chiếu thành một hình nón (cone).



## 2. Cấu trúc Mô hình (model.xml)
Chúng ta sẽ thiết lập một cảnh có đủ ba loại nguồn sáng để quan sát sự tương tác của chúng lên vật thể và khả năng đổ bóng xuống mặt đất.

```xml
<mujoco model="light_setup">
    <compiler angle="degree"/>
    
    <worldbody>
        <geom type="plane" size="5 5 0.1" rgba="0.8 0.9 0.8 1"/>
        
        <geom type="cylinder" size="0.5 1" pos="0 0 1" rgba="0.8 0.8 0.8 1"/>

        <light name="sun" directional="true" pos="0 -5 5" dir="0 1 -1" castshadow="true" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3"/>

        <light name="red_bulb" pos="-2 0 1" diffuse="1 0 0" specular="0.5 0 0"/>

        <light name="blue_spot" pos="2 0 2" dir="-1 0 -1" diffuse="0 0 1" cutoff="30"/>
    </worldbody>
</mujoco>
```

##  Gemini said

Tôi hoàn toàn nhất trí. Việc xếp cảm biến quang học (camera) ngang hàng với các cảm biến vật lý (force, IMU) và đẩy phần trích xuất dữ liệu (RGB/Depth matrix) sang Chặng 3 (Cơ học Tiếp xúc và Cảm biến) là một quyết định phân bổ chương trình giảng dạy rất chuẩn xác.

Ở Chặng 1 này, người học chỉ cần biết cách cấu hình góc nhìn (đặt camera ở đâu, nhìn về hướng nào) là đủ để phục vụ việc quan sát mô hình.

Như vậy, chúng ta đã hoàn tất Bài 08. Mảnh ghép cuối cùng của Chặng 1 chính là Bài 09: Nguồn sáng (Light). Ánh sáng trong MuJoCo không chỉ để làm đẹp; nó quyết định độ tương phản của vật thể, khả năng đổ bóng (shadow casting), và là yếu tố then chốt nếu bạn định dùng mô phỏng này để huấn luyện các thuật toán thị giác (Vision RL).
Markdown

# Bài 09: Thiết lập Nguồn sáng (Light Sources)

## 1. Cơ sở Lý thuyết
Trong môi trường thực tế, ánh sáng quyết định cách chúng ta (và hệ thống thị giác máy tính) nhận thức hình khối, độ sâu và khoảng cách thông qua hiện tượng đổ bóng (shadows) và phản xạ (reflections). Nếu môi trường mô phỏng không được chiếu sáng đúng cách, hình ảnh thu được từ camera sẽ phẳng lỳ (flat shading), làm mất đi các đặc trưng không gian quan trọng.

MuJoCo hỗ trợ ba loại nguồn sáng cơ bản:
1.  **Directional Light (Ánh sáng song song/Mặt trời):** Nguồn sáng đặt ở vô cực, tỏa ra các tia sáng song song theo một hướng xác định. Độ sáng không bị suy giảm theo khoảng cách. Thường dùng để giả lập ánh sáng Mặt Trời toàn cục.
2.  **Point Light (Ánh sáng điểm/Bóng đèn tròn):** Nguồn sáng phát ra từ một điểm cụ thể và tỏa đều ra mọi hướng. Độ sáng suy giảm (attenuation) theo bình phương khoảng cách.
3.  **Spotlight (Ánh sáng rọi/Đèn pin):** Tương tự Point Light nhưng bị giới hạn góc chiếu thành một hình nón (cone).



## 2. Cấu trúc Mô hình (model.xml)
Chúng ta sẽ thiết lập một cảnh có đủ ba loại nguồn sáng để quan sát sự tương tác của chúng lên vật thể và khả năng đổ bóng xuống mặt đất.

```xml
<mujoco model="light_setup">
    <compiler angle="degree"/>
    
    <worldbody>
        <geom type="plane" size="5 5 0.1" rgba="0.8 0.9 0.8 1"/>
        
        <geom type="cylinder" size="0.5 1" pos="0 0 1" rgba="0.8 0.8 0.8 1"/>

        <light name="sun" directional="true" pos="0 -5 5" dir="0 1 -1" castshadow="true" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3"/>

        <light name="red_bulb" pos="-2 0 1" diffuse="1 0 0" specular="0.5 0 0"/>

        <light name="blue_spot" pos="2 0 2" dir="-1 0 -1" diffuse="0 0 1" cutoff="30"/>
    </worldbody>
</mujoco>
```

3. Phân tích Thẻ (Tag Analysis)

- `<light>`: Thẻ định nghĩa một nguồn sáng trong môi trường.

  - `name`: Định danh (tùy chọn nhưng nên có).

  - `pos="x y z"`: Vị trí của nguồn sáng. (Lưu ý: Đối với `directional="true"`, `pos` chỉ dùng để xác định vị trí tính toán bóng đổ trung tâm, ánh sáng thực chất vẫn đến từ vô cực).

  - `dir="vx vy vz"`: Vector chỉ hướng của luồng sáng (áp dụng cho Directional và Spotlight). MuJoCo sẽ tự động chuẩn hóa (normalize) vector này.

  - `directional="true|false"`: Chuyển đổi giữa ánh sáng song song và ánh sáng điểm. Mặc định là `false`.

  - `castshadow="true|false"`: Kích hoạt khả năng đổ bóng. (Lưu ý: Tính toán bóng đổ rất tốn tài nguyên. Theo khuyến cáo, chỉ nên bật tính năng này cho một nguồn sáng chính yếu nhất trong cảnh).

  - Khái niệm màu sắc ánh sáng:

    - `diffuse`: Màu sắc ánh sáng khuếch tán (màu chính của nguồn sáng chiếu lên bề mặt vật nhám).

    - `specular`: Màu sắc ánh sáng phản xạ (màu của đốm sáng chói lóa trên bề mặt vật bóng/kim loại).

  - `cutoff="30"`: (Chỉ dùng cho Spotlight) Góc cắt tạo thành hình nón ánh sáng.