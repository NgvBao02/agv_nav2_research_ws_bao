# Tích hợp phần cứng GA25 encoder, RPLIDAR A1M8 và BNO055

## Kiến trúc đã chọn

Phần cứng thật giữ cùng giao diện ROS với mô phỏng để Nav2, RViz2 và bộ so sánh
thuật toán không cần biết dữ liệu đến từ Gazebo hay từ xe.

```text
Nav2 /cmd_vel
      │
      ▼
base hardware + PID bánh ──► PWM trái/phải ──► 2 × GA25 130 RPM
      ▲                                             │
      └──────── encoder counts ◄────────────────────┘
                    │
                    ▼
             /wheel/odometry ──────┐
                                    ├──► EKF ──► /odom + odom→base_link
BNO055 ───────────── /imu/data ─────┘

RPLIDAR A1M8 ─────── /scan ──► AMCL/costmap
AMCL ─────────────────────────► map→odom
```

`config/real_robot_profile.yaml` lưu các giá trị đã biết và đánh dấu `null` cho
những giá trị chưa được xác nhận. `config/ekf_real.yaml` là cấu hình khởi đầu
cho `robot_localization`; nó chưa được coi là cấu hình đã hiệu chuẩn.

## GA25 encoder 130 RPM

Với bánh Ø85 mm, tốc độ không tải lý thuyết là:

```text
v_no_load = pi × 0,085 × 130 / 60 ≈ 0,579 m/s
```

Đây không phải vận tốc an toàn để đưa thẳng vào Nav2. Tốc độ thực còn phụ thuộc
điện áp, tải, hộp số, driver, pin và PID. Giới hạn mô phỏng 0,08 m/s được giữ để
thu thập dữ liệu an toàn; sau đó tăng từng bước trên xe thật.

Giá trị hiện được cung cấp là `encoder_ticks_per_rev = 1320`. Cấu hình tạm hiểu
đây là tổng tick đã giải mã cho đúng một vòng bánh sau hộp số. Với bánh Ø85 mm:

```text
radians_per_tick = 2 × pi / 1320         ≈ 0,00475999 rad/tick
metres_per_tick  = pi × 0,085 / 1320     ≈ 0,00020230 m/tick
ticks_at_130_rpm = 1320 × 130 / 60       ≈ 2860 tick/s
```

Nếu 1320 là tick ở trục motor thay vì một vòng bánh thì các hệ số trên không
đúng. Quan hệ tổng quát khi encoder nằm trên trục motor là:

```text
counts_per_wheel_rev = encoder_PPR × decode_factor × gearbox_ratio
metres_per_count      = pi × wheel_diameter / counts_per_wheel_rev
```

Nếu datasheet dùng CPR thay cho PPR phải xác định rõ CPR đó đã bao gồm quadrature
x4 hay chưa. Driver chỉ được chốt sau khi quay bánh bằng tay đúng 10 vòng và xác
nhận mỗi phía thu gần 13.200 tick, đồng thời kiểm tra dấu tăng/giảm. Ngoài
count/rev, cần biết bo điều khiển, driver công suất, điện áp motor và cực tính
encoder/motor.

STL cho khoảng cách giữa tâm hai mặt lốp lăn là `0,2548 m`; không dùng khoảng
giữa hai trục joint CAD `0,2000 m` hoặc tâm bounding box toàn part `0,2424 m`.
`0,2548 m` chỉ là giá trị khởi đầu. Giá trị điều khiển cuối phải được hiệu chỉnh
bằng nhiều vòng quay thuận/nghịch vì biến dạng lốp và trượt làm thay đổi track
width động học hiệu dụng.

Nên dùng `ros2_control` `diff_drive_controller` cho đường dài hạn: controller
nhận `cmd_vel`, đọc position/velocity feedback từ encoder, áp timeout và xuất
wheel odometry. Hardware interface hoặc firmware phải đóng vòng PID tốc độ từng
bánh; không điều khiển Nav2 bằng PWM hở vòng.

## RPLIDAR A1M8

Driver ROS 2 chính thức của Slamtec là `sllidar_ros2`. Profile A1 dùng serial
115200, frame `laser`, angle compensation bật và topic `/scan`. Trên xe nên tạo
udev rule để thiết bị luôn là `/dev/rplidar`, không dựa vào `/dev/ttyUSB0` vì tên
này có thể đổi sau khi khởi động lại.

Vị trí và yaw của lidar trong URDF phải đo theo tâm quay quang học. Scan ngược
chiều hoặc lắp úp được xử lý ở tham số `inverted`, không sửa dữ liệu bằng phép
đổi dấu tùy ý trong node khác.

## BNO055

BNO055 chạy fusion tối đa khoảng 100 Hz và phù hợp để bổ sung tốc độ quay yaw.
Tuy nhiên Bosch đánh dấu linh kiện này là “not recommended for new designs”. Nếu
đã có module thì vẫn dùng được cho mẫu nghiên cứu; nếu chưa mua, nên cân nhắc
BHI260AP/BHI260AB hoặc một IMU còn được hỗ trợ dài hạn.

Giai đoạn đầu EKF chỉ fuse `angular_velocity.z` từ BNO055 với vận tốc encoder.
Không fuse yaw tuyệt đối ngay vì từ trường của motor DC, khung thép và dây nguồn
có thể làm heading magnetometer nhảy. Chỉ bật quaternion/yaw sau khi kiểm tra
calibration status, quy ước ENU/REP-103 và thử nhiễu khi hai motor thay đổi PWM.

IMU phải gắn cứng, trục x hướng trước, y sang trái, z hướng lên. Nếu module buộc
phải xoay, biểu diễn đúng phép xoay trong joint `base_link → imu_link`; không sửa
quaternion thủ công trong EKF.

## Trình tự bring-up xe thật

1. Đọc encoder riêng từng bánh, xác nhận dấu và counts/rev bằng 10 vòng thủ công.
2. Đóng vòng PID tốc độ bánh, kiểm tra timeout dừng motor khi mất `cmd_vel`.
3. Xuất `/wheel/odometry`, chưa xuất TF; chạy thẳng 2 m và quay 10 vòng để hiệu
   chỉnh bán kính bánh trái/phải và wheel separation.
4. Bring-up BNO055, kiểm tra timestamp, frame, calibration và nhiễu theo PWM.
5. Chạy EKF, để EKF là nguồn duy nhất của `odom → base_link`.
6. Bring-up A1M8, kiểm tra `/scan` và static transform trong RViz2.
7. Chỉ sau đó mới bật AMCL, Nav2, Collision Monitor và runner Pivot–G2.

## Thông tin còn phải cung cấp

- ảnh hoặc link đúng model GA25 và datasheet encoder;
- xác nhận 1.320 tick là số đếm trên một vòng bánh sau khi đã giải mã quadrature;
- điện áp motor, loại driver công suất và bo điều khiển/MCU;
- BNO055 dùng I2C hay UART;
- tọa độ lắp thực của lidar và IMU so với tâm trục hai bánh.
