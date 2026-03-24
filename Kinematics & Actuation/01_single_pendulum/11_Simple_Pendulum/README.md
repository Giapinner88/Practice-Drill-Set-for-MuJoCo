# Bài 11: Simple Pendulum (Hệ Động Lực Học, Không Gian Pha và Cấu Trúc Năng Lượng)

## 1. Phương trình Chuyển động (Equations of Motion)
[cite_start]Mục tiêu của chương này là hiểu rõ động lực học của một con lắc đơn[cite: 13]. [cite_start]Hệ thống này đủ phong phú để giới thiệu các khái niệm về động lực học phi tuyến, nhưng cũng đủ tường minh để có thể phân tích cặn kẽ[cite: 15].

Sử dụng phương pháp Lagrangian, phương trình động lực học của con lắc (với khối lượng $m$, chiều dài $l$) được viết dưới dạng:
[cite_start]$$ml^{2}\ddot{\theta}(t) + mgl \sin \theta(t) = Q$$ [cite: 21, 22]

[cite_start]Trong đó, lực suy rộng $Q$ bao gồm momen cản (do ma sát) và tín hiệu điều khiển chủ động $u(t)$: $Q = -b\dot{\theta}(t) + u(t)$[cite: 23, 24]. Từ đó ta có phương trình chuẩn:
[cite_start]$$ml^{2}\ddot{\theta} + b\dot{\theta} + mgl \sin \theta = u$$ [cite: 27]

## 2. Không gian Pha (Phase Portrait)

Hệ thống con lắc là một hệ bậc hai, có thể được biểu diễn dưới dạng hệ phương trình bậc nhất hai chiều:
$$\dot{x}_{1} = x_{2}$$
[cite_start]$$\dot{x}_{2} = f(x_{1}, x_{2})$$ [cite: 140]
[cite_start]Với $x_{1} = \theta$ và $x_{2} = \dot{\theta}$[cite: 141]. [cite_start]Không gian đồ họa tạo bởi vector $[\dot{x}_{1}, \dot{x}_{2}]^{T}$ được gọi là **Không gian pha (Phase portrait)**[cite: 141, 142].

[cite_start]Khi quan sát không gian pha (trường hợp $u=0$), hệ thống có các điểm cân bằng (fixed points) tại các vị trí $\dot{x} = 0$[cite: 59]. 
* [cite_start]Điểm hướng xuống ($\theta = 0, \pm 2\pi...$) là điểm ổn định theo nghĩa Lyapunov (stable i.s.L.)[cite: 66, 67, 146]. [cite_start]Nếu có thêm ma sát ($b > 0$), nó trở thành điểm ổn định tiệm cận (asymptotically stable)[cite: 70, 196].
* [cite_start]Điểm lộn ngược ($\theta = \pi, \pm 3\pi...$) là điểm không ổn định (unstable)[cite: 63, 65, 73].

## 3. Phân tích Năng lượng và Điều khiển (Energy-Shaping)
[cite_start]Với con lắc không ma sát ($b=0$), năng lượng tổng của hệ được bảo toàn trên các quỹ đạo[cite: 154].
Động năng và Thế năng lần lượt là:
[cite_start]$$T = \frac{1}{2}ml^{2}\dot{\theta}^{2}, \quad U = -mgl \cos(\theta)$$ [cite: 153]
[cite_start]Tổng năng lượng $E = \frac{1}{2}ml^{2}\dot{\theta}^{2} - mgl \cos \theta$[cite: 220].

[cite_start]Quỹ đạo đặc biệt đi qua điểm cân bằng lộn ngược (không ổn định) được gọi là **quỹ đạo đồng tà (homoclinic orbit)**[cite: 163, 164]. [cite_start]Năng lượng tại quỹ đạo này là $E^{d} = mgl$[cite: 226, 227].

**Luật điều khiển bơm năng lượng:**
[cite_start]Để đưa con lắc lên vị trí lộn ngược trong điều kiện bị giới hạn lực chấp hành ($|u| \le u_{max}$)[cite: 209], ta cần một bộ điều khiển thay đổi năng lượng hệ thống sao cho tiệm cận $E^{d}$. 
[cite_start]Lực tác dụng cùng chiều với $\dot{\theta}$ sẽ bơm năng lượng, và ngược chiều sẽ rút năng lượng[cite: 224]. [cite_start]Ta định nghĩa sai số năng lượng $\tilde{E} = E - E^{d}$[cite: 228]. Luật điều khiển có dạng:
[cite_start]$$u = -k\dot{\theta}\tilde{E} \quad (k > 0)$$ 

## 4. Lưu ý Kỹ thuật & Sim-to-Real
[cite_start]Tính ưu việt của bộ điều khiển Energy-Shaping trên là sự bền vững (robustness) đáng kinh ngạc trước các sai số của mô hình[cite: 247]. [cite_start]Bạn có thể viết lại $\tilde{E}/m = \frac{1}{2}l^{2}\dot{\theta}^{2} - gl(1 + \cos \theta)$[cite: 249]. [cite_start]Điều này chứng tỏ luật điều khiển này **hoàn toàn không phụ thuộc vào khối lượng $m$** của con lắc, và chỉ phụ thuộc tuyến tính vào ước lượng chiều dài $l$ cũng như gia tốc trọng trường $g$[cite: 250]. [cite_start]Đây là một đặc tính Sim-to-Real cực kỳ quý giá mà các thuật toán tối ưu hóa phức tạp khác khó đạt được[cite: 251].

## 5. Bài tập Thực hành: Khảo sát Bộ điều khiển Bơm Năng Lượng
Dựa vào script `simulate.py`, sinh viên thực hiện các thí nghiệm sau và phân tích biểu đồ quỹ đạo pha (Phase Portrait) cũng như biểu đồ năng lượng (Energy Tracking):

**Bài tập 1: Độ nhạy của hệ số khuếch đại (Gain Sensitivity)**
* **Nhiệm vụ:** Thay đổi hệ số bơm năng lượng $k$ trong dải $[0.1, 0.5, 2.0]$.
* **Quan sát:** Số lần con lắc phải "đu đưa" (pumps) để tích lũy đủ năng lượng $E^d = mgl$ thay đổi như thế nào? Khi $k = 2.0$, tín hiệu $\tau$ có bị bão hòa (clipping) liên tục ở hai đầu $\pm 3.0$ Nm không?

**Bài tập 2: Thắt chặt Ràng buộc Underactuated (Actuator Limits)**
* **Nhiệm vụ:** Trong file `model.xml`, giảm `ctrlrange` của thẻ `<motor>` từ $\pm 3.0$ xuống mức cực đoan $\pm 0.5$. Giữ nguyên $k = 0.5$.
* **Quan sát:** Quỹ đạo pha trong không gian 2D lúc này sẽ cuộn thành một đường xoắn ốc (spiral) mở rộng rất chậm. Chứng minh rằng dù lực bị giới hạn cực nhỏ, thuật toán vẫn hội tụ về quỹ đạo đồng tà (homoclinic orbit) theo thời gian.

**Bài tập 3: Tác động của Damping và Sai số Mô hình (Sim-to-Real)**
* **Nhiệm vụ:** Trong `simulate.py`, loại bỏ khâu bù ma sát (sửa thành `tau = -k * theta_dot * E_tilde`). 
* **Quan sát:** Do khớp vẫn có `damping="0.01"` trong XML nhưng phần mềm không bù trừ, hệ thống sẽ bị rò rỉ năng lượng. Quan sát biểu đồ Energy Tracking để thấy $E(t)$ không bao giờ chạm được vạch đích $mgl$, khiến con lắc không thể văng tới điểm lộn ngược.