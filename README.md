# 🤖 Arduino Mega 2560 Line Follower Robot (PID Based)

> “Một con robot nhỏ bé, một vài cảm biến, một đoạn code… nhưng lại mở ra cả thế giới về điều khiển, tối ưu, và đam mê kỹ thuật.”

Dự án này là một **line follower robot** sử dụng **Arduino Mega 2560** kết hợp **5 cảm biến hồng ngoại** và điều khiển bằng **thuật toán PID**.  
Robot có thể vừa **đứng yên căn chỉnh chính xác** với vạch line (ALIGN), vừa **chạy bám line mượt mà và nhanh chóng** (RUN) chỉ với **một bộ PID duy nhất**.  

Mục tiêu của project không chỉ là để robot chạy đúng đường, mà còn là cách để rèn luyện tư duy: từ xử lý tín hiệu nhiễu, chống bão hòa, đến thiết kế hệ thống điều khiển ổn định.  

---

## ⚙️ Phần cứng

- **Arduino Mega 2560** – trung tâm điều khiển
- **5 cảm biến line digital** – đọc trạng thái line qua ngắt
- **Driver cầu H kép** – điều khiển hai động cơ DC
- **2 động cơ DC** – bánh trái và phải
- **Nút nhấn** (D21) – chuyển chế độ RUN ↔ ALIGN
- **Nguồn cấp** – pin LiPo hoặc pack NiMH

### Pin Mapping

| Tín hiệu        | Mega 2560 pin |
|-----------------|---------------|
| PI1             | D2  (INT4)    |
| PI2             | D3  (INT5)    |
| PI3             | D18 (INT3)    |
| PI4             | D19 (INT2)    |
| PI5             | D20 (INT1)    |
| RUN button      | D21 (INT0)    |
| Motor L IN1     | D8            |
| Motor L IN2     | D9            |
| Motor R IN1     | D10           |
| Motor R IN2     | D11           |
| ENA (PWM trái)  | D12           |
| ENB (PWM phải)  | D13           |

> Nếu robot quay sai hướng, chỉ cần đảo lại `SENSOR_LEFT_IS_PI1` hoặc đổi dây IN1/IN2.

---

## 🎯 Nguyên lý điều khiển

Robot đọc trạng thái line từ 5 cảm biến (trọng số từ -2..+2).  
Sai số line (`error`) được đưa vào bộ PID:

- **Proportional (Kp)**: phản ứng nhanh với sai số
- **Integral (Ki)**: loại bỏ sai lệch lâu dài
- **Derivative (Kd)**: giảm rung, ổn định cua gấp
- **Anti-Windup (Kaw)**: ngăn tích phân trôi khi bão hòa
- **Low-pass filter**: lọc nhiễu đạo hàm

Từ lệnh PID (`u_cmd`), robot tính PWM cho hai bánh:
```ino
pwmL = base - u_cmd
pwmR = base + u_cmd
```

- **ALIGN**: base=0 → robot đứng yên xoay về line  
- **RUN**: base giảm dần khi cua gắt (theo |u|), giúp bám line chắc hơn

---

## 🚀 Cách sử dụng

1. **Nạp code** vào Arduino Mega 2560.
2. Kết nối robot với track line (màu nền sáng, line đen).
3. Mở Serial Monitor (115200 baud).
4. Khởi động robot → mặc định ở chế độ **ALIGN**.
5. Nhấn nút D21 để chuyển sang **RUN**.
6. Tuning tham số PID qua Serial để đạt hiệu suất mong muốn.

---

## 🎛️ Serial Command (Tuning trực tiếp)

- `show` – in tham số hiện tại
- `kp 60` – đặt Kp = 60
- `ki +0.01` – tăng Ki thêm 0.01
- `kd 20` – đặt Kd = 20
- `pid 55 0 25` – đặt Kp=55, Ki=0, Kd=25
- `base 120` – đặt tốc độ cơ bản
- `kspeed 0.6` – đặt hệ số giảm tốc khi cua
- `aligndb 0.15` – chỉnh deadband ALIGN
- `mode run | align | auto` – ép chế độ

---

## 🖥️ Serial Log

Ví dụ khi robot chạy:

```ino
RUN bits=11100 e=-0.33 u=-45 base=120 L=75 R=165
ALIGN bits=00100 e=0.00 u=0 base=0 L=0 R=0
```

- `bits` = 5 sensor (1 = line đen)  
- `e` = error (sai lệch line)  
- `u` = lệnh PID  
- `base` = tốc cơ bản  
- `L/R` = PWM hai bánh  

---

## 🔧 Bảng Tuning Nhanh

| Triệu chứng                        | Giải pháp |
|------------------------------------|-----------|
| Robot rung mạnh khi chạy thẳng     | Tăng **Kd**, giảm **Kp** |
| Robot lắc lư chậm khi vào line     | Tăng **Kp** |
| Robot cua chậm, dễ trượt khỏi line | Tăng **Kp**, giảm **kspeed** |
| Robot không nhúc nhích             | Tăng **minPWM** |
| Robot rung ở ALIGN                 | Tăng **alignDeadbandE** |

---

## 🌱 Ý nghĩa dự án

Dự án này không chỉ là một chiếc xe dò line. Nó là nơi hội tụ của:

- **Điện tử**: đọc cảm biến, điều khiển động cơ
- **Điều khiển tự động**: PID, chống bão hòa, lọc nhiễu
- **Kỹ năng thực chiến**: tuning, quan sát, cải tiến
- **Tinh thần sáng tạo**: từ vài linh kiện rẻ tiền tạo ra một hệ thống “tự ra quyết định”

Một chiếc robot nhỏ trên mặt đường, nhưng là một bước lớn trong hành trình học tập kỹ thuật.

---

## 📄 License

MIT License – thoải mái sử dụng, học tập, và cải tiến.

---
