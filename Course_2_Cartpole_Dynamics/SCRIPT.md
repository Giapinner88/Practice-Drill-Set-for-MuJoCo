# KỊCH BẢN NÓI — BUỔI 2

> Lời thoại để đọc khi dạy. **Chữ thường** là lời nói, có thể đọc gần như nguyên văn.
> `[Trong ngoặc vuông]` là thao tác, không đọc lên.
> Đọc kèm [README.md](README.md) — file đó là cấu trúc, file này là lời.

**Quy ước ký hiệu:**

| | Nghĩa |
|---|---|
| 🎬 | chiếu video |
| 📊 | chiếu hình |
| 💻 | chuyển sang màn hình code / viewer |
| ❓ | câu hỏi ném ra lớp — **dừng lại chờ trả lời thật** |
| ⏱ | mốc thời gian phải bám |

---

## ⏱ 0–10 · RECAP BUỔI 1

*[Mở slide tiêu đề]*

Chào mọi người. Buổi trước chúng ta đã dựng được một robot trong MuJoCo và cho nó chạy. Hôm nay ta đi sâu vào phần mà buổi trước cố tình bỏ qua.

Nhớ lại chuỗi này:

*[Chiếu sơ đồ MJCF → MjModel → MjData → qpos/qvel/ctrl → mj_step]*

Ta viết file XML, MuJoCo biên dịch nó thành `MjModel`, ta tạo `MjData` để giữ trạng thái, rồi gọi `mj_step` trong vòng lặp. Buổi trước tôi nói với các bạn: cứ coi `mj_step` là một hộp đen, nó lo phần vật lý.

Hôm nay ta mở hộp đen đó ra. Nhưng không phải để đọc mã nguồn MuJoCo — mà để trả lời một câu rất thực tế:

> **Những tham số nào tôi viết trong file XML sẽ quyết định chuyển động mà tôi nhìn thấy?**

Vì sao câu này quan trọng? Vì khi bạn dựng một robot và nó cư xử sai — rơi quá nhanh, rung lắc, hoặc đứng im — thì bạn phải biết quay lại sửa dòng nào trong XML. Không biết thì chỉ còn cách đoán mò.

Để trả lời, hôm nay ta dùng đúng **một** mô hình duy nhất, từ đầu đến cuối buổi: **cart-pole**. Một cái xe trượt trên ray, trên xe cắm một thanh thẳng đứng.

🎬 *[`media/00_overview.mp4`]*

Đây, hệ trông như thế này. Xe trượt ngang, thanh đổ. Chỉ có vậy.

Sao lại chọn một hệ tầm thường thế này? Chính vì nó tầm thường. Nó chỉ có hai bậc tự do, nên khi tôi sửa một tham số và chuyển động thay đổi, ta biết chắc chắn thay đổi đó đến từ đâu. Với một cánh tay robot sáu khớp thì không bao giờ có được sự rõ ràng đó.

Và cart-pole không phải đồ chơi: nó là bài toán kinh điển của ngành điều khiển. Ở part 2 của repo này, chính hệ đó được dùng để dạy LQR và swing-up. Hôm nay ta xây nền cho phần đấy.

---

## ⏱ 10–25 · HIỂU HỆ CART-POLE

### 10–15 · Cấu tạo

Về mặt cấu trúc, hệ có ba tầng:

*[Chiếu sơ đồ World → Cart → Pole]*

Thế giới, trong thế giới có cái xe, và **gắn trên xe** là cái thanh. Chữ "gắn trên" đó rất quan trọng, lát nữa ta quay lại.

Hệ có hai bậc tự do. Bậc thứ nhất: xe trượt ngang, ta gọi là `x`, đơn vị mét. Bậc thứ hai: thanh quay quanh điểm gắn, gọi là `theta`, đơn vị radian.

Trong MuJoCo, hai con số đó nằm trong một mảng tên là `qpos`. Vận tốc của chúng nằm trong `qvel`.

Chỉ cần nhớ thế này thôi:

> `qpos` trả lời **"hệ đang ở đâu"**. `qvel` trả lời **"hệ đang chuyển động thế nào"**.

Cộng hai cái lại là bạn biết đủ để dự đoán tương lai của hệ. Đó là định nghĩa của **state** trong điều khiển.

### 15–20 · Input và coupling

Bây giờ đến phần thú vị. Ta tác động vào hệ này bằng gì?

Bằng đúng **một** thứ: một lực đẩy ngang vào cái xe. Có thế thôi.

❓ *[Dừng lại hỏi]* **Vậy còn cái thanh? Làm sao ta điều khiển nó?**

*[Chờ trả lời. Câu trả lời đúng là: không điều khiển trực tiếp được.]*

Chính xác. **Cái thanh không có động cơ nào cả.** Nó chuyển động hoàn toàn thụ động — do quán tính và do trọng lực. Cách duy nhất để tác động lên nó là lắc cái xe bên dưới.

Hệ có hai bậc tự do nhưng chỉ một đầu vào. Trong điều khiển, người ta gọi đó là hệ **underactuated** — thiếu cơ cấu chấp hành. Và đó chính là lý do cart-pole nổi tiếng: nếu mỗi khớp đều có động cơ riêng thì bài toán quá dễ, chẳng có gì để nghiên cứu.

Hãy nhìn bằng chứng:

🎬 *[`media/01_coupling.mp4`]*

📊 *[`media/01_coupling.png`]*

Nhìn kỹ biểu đồ này. Vùng xám bên trái là lúc tôi đẩy — chỉ 0,35 giây rồi thả tay. Nhìn đồ thị trên: xe chạy sang phải, rồi tới khoảng giây thứ 1,8 thì nó **dừng hẳn** vì chạm vào giới hạn của ray.

Bây giờ nhìn đồ thị dưới. Xe đã dừng từ lâu rồi, mà cái thanh **vẫn đang lắc**. Lắc mãi tới cuối biểu đồ.

Đó là **coupling**. Năng lượng đã truyền từ xe sang thanh, và thanh giữ lại năng lượng đó. Tôi không hề chạm vào cái thanh một lần nào.

### 20–25 · Dynamics ở mức vừa đủ

Phần này tôi sẽ đi nhanh, và tôi nói trước là **ta sẽ không giải phương trình nào cả**.

*[Chiếu công thức]*

$$M(q)\ddot q + C(q,\dot q)\dot q + g(q) = Bu$$

Đây là dạng tổng quát của phương trình chuyển động cho **mọi** hệ cơ khí — cart-pole, cánh tay robot, robot đi bộ, tất cả đều viết được dưới dạng này.

Tôi không bắt các bạn nhớ công thức. Tôi chỉ muốn các bạn nhìn ra: **mỗi số hạng trong đó tương ứng với một thứ bạn gõ trong file XML.**

*[Chiếu bảng, chỉ từng dòng]*

- Chữ **M** là khối lượng và quán tính. Trong XML nó là thuộc tính `mass`, hoặc thẻ `<inertial>`, hoặc suy ra từ hình dạng của geom.
- Chữ **C** là Coriolis và ly tâm. Cái này bạn không phải khai báo — MuJoCo tự tính ra từ M.
- Chữ **g** là trọng lực. Trong XML là thuộc tính `gravity` của thẻ `<option>`.
- Chữ **Bu** là lực từ động cơ. Trong XML là thẻ `<actuator>`, và trong Python là biến `data.ctrl`.

Và mỗi bước mô phỏng — mỗi lần gọi `mj_step` — MuJoCo giải đúng phương trình này một lần.

Vậy nên khi tôi nói "mở hộp đen ra", thì đây là thứ bên trong. Phần còn lại của buổi hôm nay là đi tìm từng chữ cái đó trong file XML.

---

## ⏱ 25–40 · DỰNG CART-POLE BẰNG MJCF

💻 *[Mở `model.xml`, phóng to chữ]*

Giờ ta mở file mô hình. Tôi sẽ đi từ trên xuống.

### 25–30 · Thiết lập mô phỏng

```xml
<option timestep="0.002" gravity="0 0 -9.81">
```

Hai thứ ở đây áp dụng cho toàn bộ mô phỏng.

`gravity` là vector ba chiều — x, y, z. Ở đây là âm 9,81 theo trục z, tức là kéo xuống dưới. Đơn giản.

`timestep` mới là thứ đáng nói. Nó là **bước thời gian** của mô phỏng. Con số 0,002 nghĩa là: mỗi lần gọi `mj_step`, thế giới tiến lên 2 mili giây.

Hãy hiểu thế này: MuJoCo không giải phương trình vi phân một cách chính xác tuyệt đối. Nó **xấp xỉ** — chia thời gian thành từng lát mỏng và tính từng lát một.

Lát càng mỏng thì càng chính xác, nhưng càng tốn CPU. Lát càng dày thì chạy càng nhanh, nhưng sai số tích tụ dần, và đủ dày thì mô phỏng sai hẳn.

Cuối buổi ta sẽ đo cái giá đó bằng số cụ thể. Giờ chỉ cần biết con số này có ở đây và nó quan trọng.

### 30–35 · Cái xe

```xml
<body name="cart" pos="0 0 1.0">
    <joint name="cart_slide" type="slide" axis="1 0 0"
           limited="true" range="-1 1" damping="0.05"/>
    <geom name="cart_geom" type="box" size="0.15 0.1 0.05" mass="1.0"/>
</body>
```

Cấu trúc này lặp lại ở mọi mô hình MuJoCo, nên đáng nhớ:

> Một **body** chứa một **joint** — mô tả nó cử động thế nào so với vật cha — và một **geom** — mô tả nó hình dạng gì, nặng bao nhiêu.

`type="slide"` nghĩa là khớp trượt. `axis="1 0 0"` là trượt dọc trục x. Vậy là cái xe trượt ngang, đúng như ta muốn.

Con số vị trí của khớp này chính là `qpos[0]` mà tôi nói lúc nãy.

### 35–40 · Cái thanh

```xml
<body name="pole" pos="0 0 0.05">
    <joint name="pole_hinge" type="hinge" axis="0 1 0" damping="0.01"/>
    <geom name="pole_geom" type="capsule" fromto="0 0 0  0 0 0.6"
          size="0.02" mass="0.1"/>
</body>
```

Tương tự, nhưng `type="hinge"` — khớp quay thay vì trượt.

**Nhưng đây mới là chỗ quan trọng nhất của cả phần này.** Nhìn kỹ file:

💻 *[Chỉ vào cấu trúc thụt lề, cho thấy body pole nằm bên trong body cart]*

Cái thanh nằm **bên trong** thẻ của cái xe. Nó là con của cái xe trong cây phân cấp.

❓ **Điều đó có ý nghĩa vật lý gì?**

*[Chờ]*

Nghĩa là: khi cái xe di chuyển, **gốc của cái thanh bị kéo đi theo**. Cái thanh không tự dưng lắc — nó lắc vì cái chân của nó bị giật.

Đây chính là coupling mà ta thấy trong video lúc nãy, nhưng nhìn ở mức cấu trúc file.

❓ **Nếu tôi lôi cái thanh ra ngoài, đặt ngang hàng với cái xe trong `<worldbody>`, thì sao?**

*[Chờ]*

Thì hai vật hoàn toàn độc lập. Xe chạy đằng xe, thanh rơi đằng thanh. Không còn là cart-pole nữa — chỉ là hai vật thể rời nhau trong cùng một cảnh.

*[Chốt]* Cây phân cấp trong MJCF không phải là chuyện tổ chức file cho gọn. **Nó là vật lý.**

Một lưu ý nhỏ về đơn vị: `slide` đo bằng **mét**, `hinge` đo bằng **radian**. Cart-pole cần đúng một cái mỗi loại.

---

## ⏱ 40–55 · MASS, INERTIA VÀ GEOMETRY

### 40–45 · Khối lượng — TUNE

Giờ đến phần thực hành. Mọi người mở `model.xml` ra.

💻 *[Hướng dẫn lớp mở file]*

Tìm dòng geom của cái thanh, chỗ `mass="0.1"`. Ta sẽ đổi con số đó.

Nhưng **trước khi đổi, hãy đoán.**

❓ **Nếu tôi làm cái thanh nặng gấp mười lần, thì với cùng một lực đẩy, cái xe sẽ chạy nhanh hơn hay chậm hơn?**

*[Chờ vài người trả lời. Đừng vội xác nhận.]*

Được rồi, giờ thử. Đổi `0.1` thành `1.0`, chạy `python simulate.py`, kéo thanh trượt đẩy xe. Rồi đổi lại thành `0.05` và làm lại.

*[Cho lớp làm 3 phút]*

📊 *[`media/03_mass.png`]*

Đây là kết quả đo. Cùng một lực 5 Newton, ba khối lượng khác nhau. Thanh càng nặng, xe đi càng chậm.

Lý do rất đơn giản, chính là định luật hai Newton. Lực đó không chỉ phải đẩy cái xe — nó phải gia tốc **cả hệ**, xe cộng thanh. Tổng khối lượng lớn hơn thì gia tốc nhỏ hơn.

### 45–50 · Quán tính

Phân biệt hai khái niệm hay bị lẫn:

> **Khối lượng** cản lại gia tốc **thẳng**.
> **Quán tính** cản lại gia tốc **quay**.

Đẩy một vật nặng thì khó. Nhưng làm một vật *quay* nhanh thì độ khó phụ thuộc không chỉ vào nó nặng bao nhiêu, mà còn vào khối lượng đó **phân bố ở đâu** so với trục quay.

Trong MuJoCo, mặc định bạn không phải khai báo quán tính. MuJoCo **tự tính** từ hình dạng và khối lượng của geom. Nếu muốn tự khai thì dùng thẻ `<inertial>`, nhưng hôm nay ta không cần.

Tôi không đi vào tensor quán tính. Ở mức này chỉ cần nắm ý niệm.

### 50–55 · Hình học KHÔNG chỉ để nhìn

Đây là điểm tôi cho là dễ hiểu nhầm nhất trong cả buổi, nên tôi dành thời gian.

Rất nhiều người mới nghĩ rằng phần `size` hay `fromto` của geom là chuyện **hiển thị** — nó quyết định vật trông thế nào trên màn hình.

**Sai.** Khi bạn để MuJoCo tự tính quán tính, thì chuỗi nhân quả là thế này:

*[Chiếu sơ đồ]*

```
kích thước geom → hình dạng → phân bố khối lượng → quán tính → chuyển động
```

Sửa kích thước là sửa vật lý.

Ta kiểm chứng. Tìm dòng `fromto="0 0 0  0 0 0.6"` — đó là chiều dài thanh, 0,6 mét. Đổi thành `1.2`, tức gấp đôi. **Giữ nguyên `mass`.**

❓ **Cùng khối lượng, cùng trọng lực, thanh dài gấp đôi sẽ đổ nhanh hơn hay chậm hơn?**

*[Chờ. Đây là câu hay gây tranh cãi trong lớp — để họ tranh luận một chút.]*

📊 *[`media/04_inertia.png`]*

Chậm hơn. Chậm hơn rõ rệt.

Nhìn chú thích dưới hình: thanh 0,6 mét có quán tính khoảng 3,3 phần nghìn. Thanh 1,2 mét — cùng khối lượng — có quán tính 1,26 phần trăm, tức gần **gấp bốn lần**.

Vì sao gấp bốn mà không phải gấp đôi? Vì quán tính phụ thuộc vào **bình phương** khoảng cách tới trục quay. Chiều dài gấp đôi, quán tính gấp bốn.

*[Chốt]* Cùng một khối lượng, chỉ trải nó ra xa hơn, mà hệ cư xử khác hẳn. Đó là lý do tôi nói: **hình học là vật lý**, không phải trang trí.

---

## ⏱ 55–70 · JOINT DYNAMICS

### 55–60 · Giới hạn khớp — TUNE

```xml
<joint type="slide" limited="true" range="-1 1"/>
```

Hai thuộc tính này nói: khớp chỉ được chạy trong khoảng từ âm 1 đến dương 1 mét.

💻 *[Chạy `simulate.py`, kéo thanh trượt hết cỡ]*

Nhìn đây — tôi đẩy hết cỡ, xe chạy tới một mét rồi **khựng lại**. Nó không đi tiếp được.

Cơ chế bên trong: MuJoCo thêm vào một **ràng buộc** để giữ khớp trong khoảng cho phép. Nó không phải là một bức tường vật lý — không có vật thể nào ở đó cả — mà là một điều kiện toán học.

Thử đổi `range` thành `-0.3 0.3` để thấy xe bị nhốt chật hơn hẳn.

Một lưu ý cho lát nữa khi ta làm thí nghiệm: **nếu xe đâm vào giới hạn, mọi phép so sánh đều hỏng.** Vì dù bạn đẩy mạnh hay nhẹ, xe cũng dừng ở đúng một mét. Nên khi đo đạc, hoặc đẩy trong khoảng ngắn, hoặc tạm đặt `limited="false"`.

### 60–65 · Damping — TUNE

```xml
<joint damping="0.1"/>
```

Damping là **cản nhớt** tại khớp. Về mặt công thức nó rất đơn giản:

$$\tau = -d\,\dot q$$

Mô-men cản bằng hệ số damping nhân với vận tốc khớp, **dấu âm** — tức là luôn ngược chiều chuyển động.

Vì luôn ngược chiều, nó luôn **rút năng lượng** ra khỏi hệ. Đó là toàn bộ bản chất của damping.

Giờ thử bốn giá trị: 0 — 0,01 — 0,1 — và 1,0. Chạy từng cái, nhìn cái thanh đung đưa bao lâu thì tắt.

*[Cho lớp làm 3 phút]*

📊 *[`media/05_damping.png`]*

Đây là bốn trường hợp. Trục đứng là tốc độ quay của thanh — nói cách khác, "thanh còn đang lắc mạnh cỡ nào".

Đường xanh dương là damping bằng không: nó lắc **mãi mãi**, biên độ không giảm. Không có gì lấy năng lượng đi cả.

Đường cam, damping 0,01: lắc rồi tắt dần. Đường xanh lá, damping 0,1: tắt nhanh hơn hẳn.

Bây giờ, đây là chỗ thú vị. Nhìn đường **tím** — damping bằng 1,0, tức mạnh nhất.

❓ **Damping mạnh nhất, vậy nó phải về vị trí nghỉ nhanh nhất chứ?**

*[Chờ. Đa số sẽ nói đúng — rồi cho họ thấy là sai.]*

Không hề. Nhìn kỹ: đường tím **không dao động lần nào cả**, nhưng nó bò rất chậm, và tới cuối biểu đồ vẫn còn đang bò.

Hiện tượng này gọi là **overdamped** — quá giảm chấn. Cản mạnh tới mức cái thanh không còn lắc được nữa, nó chỉ trườn từ từ xuống.

🎬 *[Chiếu `media/05_damping_0.mp4` và `05_damping_1_0.mp4` cạnh nhau]*

Nhìn hai video này cạnh nhau thì rõ. Bên trái lắc không ngừng. Bên phải gần như đứng yên nhưng vẫn đang nhích.

*[Chốt]* Bài học rút ra: **thêm damping không phải lúc nào cũng làm hệ ổn định nhanh hơn.** Có một giá trị tối ưu, và vượt qua nó thì hệ chậm đi. Đây là kiến thức sẽ quay lại ám các bạn khi tinh chỉnh bộ điều khiển sau này.

### 65–70 · Ma sát

```xml
<joint frictionloss="0.01"/>
```

Còn một cơ chế cản nữa, hay bị lẫn với damping.

*[Chiếu bảng so sánh]*

**Damping** tỉ lệ với vận tốc. Khớp quay càng chậm thì lực cản càng yếu. Khi gần dừng hẳn, lực cản gần như bằng không.

**Frictionloss** là ma sát khô — độ lớn gần như không đổi, bất kể nhanh hay chậm. Kể cả khi khớp sắp đứng yên, nó vẫn chống lại.

Về mặt vật lý, frictionloss mô hình hóa ma sát trong hộp số, trong ổ trục — những thứ có thật trên robot thật.

*[Ghi chú cho người dạy — không cần nói ra nếu lớp không hỏi:]*

> Lý thuyết nói ma sát khô làm khớp dừng hẳn sau thời gian hữu hạn, còn damping thuần thì chỉ tiệm cận về không. Nhưng MuJoCo giải frictionloss bằng ràng buộc mềm, nên nếu ai đó thử đo, họ sẽ **không** thấy một mốc dừng dứt khoát. Đừng hứa với lớp điều đó.

---

## ⏱ 70–80 · ACTUATOR

### 70–75 · Động cơ

```xml
<actuator>
    <motor name="cart_motor" joint="cart_slide" gear="1"/>
</actuator>
```

Đây là chỗ ta đưa lực vào hệ.

Chuỗi ánh xạ này cần thuộc lòng:

*[Chiếu, đọc từng bước]*

```
data.ctrl[0]  →  cart_motor  →  cart_slide  →  lực đẩy ngang lên xe
```

Bạn ghi một con số vào `data.ctrl[0]`. Con số đó đi vào actuator tên `cart_motor`. Actuator đó gắn với khớp `cart_slide`. Kết quả là một lực dọc trục của khớp đó.

Thẻ `<motor>` là loại actuator đơn giản nhất: lực ra **tỉ lệ thẳng** với `ctrl`, hệ số là `gear`. Ở đây `gear="1"`, nên `ctrl` bằng 3 tức là 3 Newton. Một-đổi-một.

Và nhắc lại lần nữa: **chỉ có xe mới có actuator.** Cái thanh thì không. Cả file XML này không có dòng nào điều khiển cái thanh.

### 75–80 · Giới hạn lệnh — TUNE

```xml
<motor ctrllimited="true" ctrlrange="-10 10"/>
```

`ctrlrange` **kẹp** lệnh điều khiển lại trước khi nó biến thành lực.

❓ **Nếu tôi ghi `data.ctrl[0] = 50`, thì lực ra là bao nhiêu?**

*[Chờ]*

Mười. Không phải năm mươi. MuJoCo cắt nó xuống mười và không báo lỗi gì cả.

📊 *[`media/06_ctrlrange.png`]*

Nhìn hình này. Bốn mức lệnh: 1, 5, 10, và 50 Newton. Đường của lệnh 50 là **nét đứt màu tím** — và nó nằm **chồng khít** lên đường của lệnh 10. Không phân biệt được.

Hình này gần như không cần giảng thêm. Nó tự nói.

Vì sao lại có cơ chế này? Vì động cơ thật có giới hạn. Một động cơ 10 Newton thì bạn có ra lệnh 500 nó cũng chỉ đẩy được 10. `ctrlrange` là cách mô hình hóa sự thật đó.

💻 *[Demo]* Thử đổi thành `-2 2` rồi chạy lại — thanh trượt trong viewer giờ chỉ kéo được tới 2, và xe yếu hẳn.

Lưu ý: hôm nay ta ra lệnh **bằng tay**, bằng cách kéo thanh trượt. Chưa có thuật toán nào tự tính ra lệnh cả. Đó là nội dung buổi 3.

---

## ⏱ 80–95 · ĐỌC STATE

### 80–90 · qpos và qvel

Quay lại hai mảng tôi nhắc từ đầu buổi.

Với cart-pole, ánh xạ đúng như trực giác:

```
qpos[0] → vị trí xe          qvel[0] → vận tốc xe
qpos[1] → góc thanh          qvel[1] → vận tốc góc thanh
```

📊 *[`media/09_state_sequence.png`]*

Nhìn chuỗi ảnh này — bốn thời điểm khi cái thanh đổ xuống. Con số `qpos[1]` chính là góc mà các bạn nhìn thấy đang tăng dần ở đây.

💻 *[Demo trong viewer]* Và bạn xem được trực tiếp, không cần viết dòng code nào: nhấn `Tab`, panel bên trái, tìm mục **Watch**, gõ `qpos` vào. Giá trị sẽ chạy theo thời gian thực.

**Nhưng có một cảnh báo tôi phải đưa ra ở đây**, vì nó sẽ cứu các bạn sau này.

❓ **Với cart-pole, `qpos` có 2 phần tử và `qvel` cũng có 2. Vậy chúng luôn cùng độ dài chứ?**

*[Chờ — hầu hết sẽ nói có]*

**Không.** Với mô hình lớn có khớp tự do — free joint, loại cho phép vật bay tự do trong không gian — thì một khớp đó chiếm **7 ô** trong `qpos`: ba tọa độ vị trí, cộng bốn số quaternion cho hướng. Nhưng nó chỉ chiếm **6 ô** trong `qvel`: ba vận tốc thẳng, ba vận tốc góc.

Bảy với sáu. Lệch nhau.

Nên nếu bạn quen đoán chỉ số, tới lúc gặp mô hình thật là sai chắc chắn. Cách đúng là tra theo **tên**:

```python
jid   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pole_hinge")
i_pos = model.jnt_qposadr[jid]
i_vel = model.jnt_dofadr[jid]
```

Hoặc gọn hơn: khai báo `<sensor>` trong XML rồi đọc `data.sensordata`. File mô hình hôm nay đã có sẵn bốn sensor để các bạn thấy cú pháp.

### 90–95 · Bắc cầu sang part 2

Đến đây tôi muốn nối buổi này với phần còn lại của khóa.

💻 *[Mở `part_2_models_control/02_cartpole` trên nhánh main]*

Trong repo có một bài dùng **đúng hệ cart-pole này**, nhưng ở mức cao hơn hẳn: tuyến tính hóa quanh điểm cân bằng, lấy ma trận Jacobian bằng `mjd_transitionFD`, kiểm tra tính điều khiển được, thiết kế bộ điều khiển LQR, và ghép với swing-up để dựng cây gậy lên.

Toàn bộ phần đọc state bằng Python đã được xử lý sẵn ở đó. Các bạn không cần viết lại.

*[Chốt]* Buổi hôm nay dạy các bạn về **tham số**. Muốn học **điều khiển** thì sang part 2. Cùng một hệ, hai tầng kiến thức.

---

## ⏱ 95–110 · THÍ NGHIỆM

Đây là phần quan trọng nhất của buổi. Từ giờ tới hết 15 phút, các bạn tự làm, tôi chỉ đi quanh hỗ trợ.

Có một nguyên tắc tôi sẽ nhắc lại nhiều lần:

> **Mỗi lần chỉ đổi đúng MỘT tham số.**

Nếu đổi hai thứ cùng lúc rồi thấy chuyển động khác đi, bạn không biết cái nào gây ra. Đây là nguyên tắc của mọi thí nghiệm khoa học, và nó đặc biệt đúng với mô phỏng.

Và quy trình cho mỗi thí nghiệm là bốn bước:

**Đoán trước → Sửa → Chạy → So với dự đoán.**

Bước "đoán trước" là bước quan trọng nhất. Nếu bỏ nó đi thì các bạn chỉ đang nghịch tham số, không phải học. Khi dự đoán của bạn **sai**, đó chính là lúc bạn học được nhiều nhất.

### Thí nghiệm 1 — Trọng lực

Đổi `gravity` lần lượt: `0 0 -9.81`, rồi `0 0 -1.62`, rồi `0 0 0`.

Con số giữa là trọng trường Mặt Trăng, cho vui.

❓ **Khi g bằng không, số hạng nào trong phương trình chuyển động biến mất?**

📊 *[`media/02_gravity.png`]*

Nhìn đường xanh lá — g bằng không. Nó **phẳng lì**. Cái thanh giữ nguyên góc nghiêng ban đầu, mãi mãi, không nhúc nhích.

🎬 *[`media/02_gravity_zero.mp4`]*

Vì không còn gì kéo nó xuống. Số hạng `g(q)` bằng không, và cái thanh chẳng có lý do gì để đổ.

### Thí nghiệm 2 — Khối lượng thanh

Ta đã làm ở phần trước rồi, nhưng giờ các bạn tự thử với nhiều giá trị hơn: 0,05 — 0,2 — 1,0.

⚠️ *[Nhắc lớp]* Nhớ cái bẫy tôi nói lúc nãy: nếu xe đâm vào giới hạn một mét thì mọi trường hợp đều dừng cùng chỗ và không so sánh được gì. Chỉ đẩy trong khoảng ngắn thôi.

### Thí nghiệm 3 — Damping

Bốn giá trị: 0 — 0,01 — 0,1 — 1,0. Đừng quên nhìn kỹ trường hợp cuối, cái overdamped.

### Thí nghiệm 4 — Lực điều khiển

Kéo thanh trượt tới các mức khác nhau, xem xe tăng tốc thế nào.

Rồi sửa `ctrlrange` xuống `-2 2` và làm lại. Cảm nhận sự khác biệt.

### Thí nghiệm 5 — Timestep

*[Nói rõ đây là thí nghiệm quan trọng nhất]*

Thí nghiệm cuối này, theo tôi, là thứ có giá trị thực tế nhất trong cả buổi.

Đổi `timestep`: 0,001 — 0,002 — 0,01 — 0,05 — rồi 0,1.

Quan sát ba thứ: mô phỏng có mượt không, chuyển động có còn hợp lý không, và tới mức nào thì hệ bắt đầu cư xử kỳ quặc.

📊 *[`media/07_timestep.png`]*

Đây là kết quả đo. Bên trái là quỹ đạo của góc thanh. Đường đen đậm là mô phỏng tham chiếu — timestep cực nhỏ, coi như chính xác. Các đường màu là những timestep lớn dần, và các bạn thấy chúng **tách dần ra** khỏi đường đen.

Bên phải là sai số, vẽ trên thang logarit. Và nó gần như là một **đường thẳng**.

Đường thẳng trên thang log-log có nghĩa là gì? Nghĩa là sai số **tỉ lệ tuyến tính** với timestep. Gấp đôi bước thời gian thì sai số gấp đôi. Đây đúng là điều lý thuyết dự đoán cho phương pháp tích phân Euler bậc một mà MuJoCo dùng mặc định.

*[Chốt]* Vậy nên `timestep` không có giá trị nào "đúng". Nó là một **đánh đổi** giữa độ chính xác và tốc độ, và **các bạn** phải chọn, ở mọi dự án. Chọn quá nhỏ thì mô phỏng chạy chậm không dùng được. Chọn quá lớn thì kết quả sai mà không ai báo cho bạn biết.

---

## ⏱ 110–117 · DEBUG BẰNG VIEWER

💻 *[Chạy `python simulate.py --broken`]*

Bây giờ tôi đưa các bạn một mô hình **cố tình sai**.

Trong file `model_broken.xml` có **ba lỗi**. Và điểm đáng nói là: cả ba đều **hợp lệ về cú pháp**. MuJoCo biên dịch trót lọt, không báo lỗi, không cảnh báo gì hết. Chỉ có chuyển động là sai.

Đây chính là loại lỗi khó chịu nhất khi làm việc thật. Nếu XML sai cú pháp thì MuJoCo báo ngay, dễ sửa. Nhưng sai *ý nghĩa* thì bạn phải tự phát hiện.

Cách phát hiện là **nhìn**. Viewer có sẵn công cụ cho việc này.

💻 *[Thao tác chậm để lớp làm theo]*

Nhấn `Tab`, mở panel bên trái, tìm mục **Rendering**. Ở đó bật:

- **Joint** — vẽ ra trục quay và trục trượt của từng khớp, dưới dạng mũi tên
- **Frame → Body** — vẽ hệ trục tọa độ gắn với từng vật thể
- **Com** — vẽ khối tâm

Bật xong thì những thứ vô hình trở thành nhìn thấy được.

*[Cho lớp tự tìm 5 phút. Đi quanh gợi ý, đừng nói đáp án.]*

Gợi ý nếu lớp bí: hãy so sánh hướng của mũi tên trục với hướng mà vật *đáng lẽ* phải chuyển động.

*[Sau 5 phút — chữa bài]*

📊 *[`media/08_broken_vs_correct.png`]*

Hàng trên là mô hình đúng, hàng dưới là mô hình sai. Cùng thời điểm, cùng góc nhìn.

**Lỗi thứ nhất — sai trục của khớp trượt.**

```xml
<joint name="cart_slide" type="slide" axis="0 1 0"/>   <!-- sai -->
<joint name="cart_slide" type="slide" axis="1 0 0"/>   <!-- đúng -->
```

Trục là `0 1 0` tức trục y, nên cái xe trượt **đâm ngang qua** thanh ray thay vì chạy dọc nó. Nhìn hàng dưới: mũi tên xanh của trục trượt chĩa vuông góc với ray. Xe đứng im tại chỗ vì nó đang cố đi theo hướng mà camera không thấy.

**Lỗi thứ hai — cái thanh treo lơ lửng.**

```xml
<body name="pole" pos="0 0 0.5">    <!-- sai -->
<body name="pole" pos="0 0 0.05">   <!-- đúng -->
```

Gốc thanh nằm cách mặt xe 45 phân, treo giữa không trung. Bật Frame lên là thấy hệ trục của thanh tách rời hẳn khỏi xe. Cái quả cầu trắng to đùng trong hình là khối tâm bị lệch chỗ.

**Lỗi thứ ba — sai trục của khớp quay.** Và đây là lỗi nguy hiểm nhất.

```xml
<joint name="pole_hinge" type="hinge" axis="1 0 0"/>   <!-- sai -->
<joint name="pole_hinge" type="hinge" axis="0 1 0"/>   <!-- đúng -->
```

Cái thanh quay quanh trục x — tức là đổ sang hai bên — trong khi cái xe chỉ chạy dọc trục x.

❓ **Hai chuyển động đó nằm trong hai mặt phẳng vuông góc nhau. Hệ quả là gì?**

*[Chờ]*

**Coupling biến mất hoàn toàn.** Bạn đẩy xe kiểu gì thì cái thanh cũng không phản ứng. Nó rơi độc lập.

Và vì sao lỗi này nguy hiểm nhất? Vì mô hình vẫn **trông có vẻ chạy**. Có xe, có thanh, cả hai đều chuyển động. Nếu không để ý bạn có thể chạy cả tuần trên một mô hình sai mà không biết.

*[Chốt]* Mental model cho việc debug:

```
mô hình cư xử sai
        ↓
đọc lại MJCF  +  bật hiển thị trong Viewer
        ↓
tìm ra lỗi
```

Hai việc đó phải làm **cùng nhau**. Chỉ đọc file thì không thấy được hình học. Chỉ nhìn viewer thì không biết sửa dòng nào.

---

## ⏱ 117–120 · TỔNG KẾT

Ba phút cuối, tôi tóm lại.

*[Chiếu sơ đồ tổng kết]*

Toàn bộ buổi hôm nay xoay quanh một chuỗi:

```
tham số trong MJCF  →  tính chất vật lý  →  hành vi mô phỏng
```

Các tham số ta đã đi qua: khối lượng, quán tính, trọng lực, damping, ma sát, khớp, actuator, và timestep. Mỗi cái ta đều đã sửa bằng tay và nhìn thấy hệ quả.

Và đây là chỗ buổi hôm nay nằm trong mạch chung:

*[Chiếu bảng ba dòng]*

- Buổi 1 hỏi: **làm sao mô tả một hệ?**
- Buổi 2 — hôm nay — hỏi: **MuJoCo mô phỏng hệ đó thế nào?**
- Buổi 3 sẽ hỏi: **làm sao ra lệnh cho nó?**

Hôm nay ta ra lệnh bằng cách kéo thanh trượt bằng tay. Buổi sau, máy tính sẽ tự tính ra lệnh đó.

Nếu các bạn về nhà mà chỉ nhớ được một câu từ buổi hôm nay, tôi mong đó là câu này:

> **Mọi con số bạn gõ trong file XML đều là vật lý. Không có con số nào chỉ để trang trí.**

Cảm ơn mọi người.

---

# PHỤ LỤC A — CÂU HỎI LỚP HAY HỎI

**"Sao không dùng RK4 cho chính xác hơn?"**
Được, đổi `integrator="RK4"` trong `<option>` là xong. Nhưng lưu ý: hàm `mjd_transitionFD` dùng ở part 2 để lấy Jacobian **không hỗ trợ RK4**. Nên nếu định thiết kế LQR thì phải dùng Euler để mô phỏng và Jacobian nhất quán với nhau. Đó là lý do part 2 khai báo Euler một cách tường minh.

**"Vì sao thanh có `contype="0" conaffinity="0"`?"**
Hai thuộc tính đó tắt va chạm cho geom. Nếu không tắt, cái thanh sẽ va vào chính cái xe khi nó đổ xuống, và ta có thêm động lực học va chạm — phức tạp không cần thiết cho buổi này.

**"Thanh ray có phải vật thật không?"**
Không. Nó chỉ là geom để nhìn cho dễ hiểu, cũng tắt va chạm. Thứ thật sự giới hạn xe là `range` của khớp, không phải cái ray.

**"Đơn vị của damping là gì?"**
Với khớp trượt là Newton trên mét-trên-giây. Với khớp quay là Newton-mét trên radian-trên-giây. Nói cách khác, nó luôn là "lực cản trên một đơn vị vận tốc".

**"Vì sao keyframe đặt góc 0,2 radian mà không phải 0?"**
Vì góc 0 là vị trí thẳng đứng — một điểm cân bằng **không bền**. Về lý thuyết thanh sẽ đứng yên vĩnh viễn ở đó, và chẳng có gì để xem. Nghiêng 0,2 radian để nó có lý do mà đổ.

**"Làm sao biết nên chọn timestep bao nhiêu?"**
Không có công thức chung. Cách thực dụng: bắt đầu từ 0,002 — mặc định của MuJoCo — rồi giảm dần cho tới khi kết quả không đổi nữa. Nếu giảm timestep mà kết quả vẫn nhảy lung tung thì vấn đề nằm ở chỗ khác, không phải timestep.

---

# PHỤ LỤC B — NẾU THIẾU THỜI GIAN

Ưu tiên cắt theo thứ tự này:

1. **Cắt trước hết:** mục 65–70 phút (frictionloss). Nói một câu "còn một cơ chế cản nữa là ma sát khô, khác damping ở chỗ không phụ thuộc vận tốc" rồi đi tiếp. Tiết kiệm 5 phút.
2. **Cắt tiếp:** thí nghiệm 2 và 4 trong phần hands-on — hai cái này đã được demo ở phần lý thuyết rồi. Tiết kiệm 5 phút.
3. **Cắt tiếp:** mục 45–50 (quán tính lý thuyết), gộp thẳng vào mục 50–55 vì phần đó đã có ví dụ cụ thể. Tiết kiệm 5 phút.

**Tuyệt đối không cắt:**
- Mục 35–40 — cây phân cấp và coupling. Đây là ý niệm khó nhất và quan trọng nhất.
- Thí nghiệm 5 — timestep. Giá trị thực tế cao nhất.
- Mục 110–117 — debug bằng viewer. Kỹ năng dùng được ngay.

Nếu vượt giờ ở phần hands-on, cứ để lớp làm tiếp và cắt ngắn phần tổng kết xuống một phút. Thời gian tay chạm vào code đáng giá hơn lời tổng kết.
