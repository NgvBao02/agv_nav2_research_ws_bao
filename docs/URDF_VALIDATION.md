# Kiểm tra URDF/SDF robot vi sai hai bánh

Ngày kiểm tra: 23/07/2026  
Nguồn CAD: `xe_urdf_export_description.zip`  
Môi trường: ROS 2 Jazzy, Gazebo Harmonic 8.11

## Kết luận

Mesh CAD có thể dùng cho RViz2 và Gazebo, nhưng URDF export ban đầu chưa đủ để
làm mô hình động lực học hoặc benchmark Nav2. Các lỗi chính đã được sửa trong
workspace: collision thân quá hẹp, tâm/bề rộng collision bánh không trùng mặt
lốp lăn, wheel separation dùng nhầm khoảng joint CAD, thiếu support contact,
thiếu cấu hình cảm biến và thiếu một nguồn ground truth độc lập.

URDF hiện dùng cho cây link/TF/visual; SDF dùng cho physics và sensor Gazebo.
Việc tách này là chủ ý: KDL không hỗ trợ inertia trên root link `base_link`, nên
inertia thân chỉ nằm trong SDF. Hai file vẫn dùng cùng kích thước và cùng frame.

## Hình học đã chốt

| Đại lượng | Giá trị | Cách dùng |
|---|---:|---|
| Envelope thân | 0,44 × 0,34 × 0,16 m | footprint/collision compound |
| Bán kính bánh | 0,0425 m | URDF, SDF, encoder |
| Bề rộng mặt lốp | 0,0300 m | collision bánh |
| Bề rộng toàn bộ part bánh | 0,0424 m | chỉ để đối chiếu CAD |
| Tâm vệt lăn trái/phải | y = ±0,1274 m | joint/collision bánh |
| Khoảng cách vệt lăn vật lý | 0,2548 m | Pivot–G2 và xe thật |
| Bốn bi đỡ | x = ±0,15; y = ±0,09; r = 0,006 m | contact cân bằng |
| Lidar so với `base_link` | z = +0,10892 m | frame `laser` |
| IMU so với `base_link` | z = −0,0128 m | frame `imu_link`, khay dưới |

Khoảng `0,2000 m` trong export là khoảng hai joint CAD, còn `0,2424 m` là
khoảng tâm bounding box của toàn bộ part. Cả hai không phải khoảng tâm hai mặt
lốp đang lăn. Giá trị đúng lấy từ connected component mặt lốp là `0,2548 m`.

SDF dùng `wheel_separation = 0,2809 m` riêng trong plugin DiffDrive. Đây là
effective track đã hiệu chỉnh để odometry plugin khớp pose vật lý của mô hình
sáu điểm tiếp xúc; nó không thay thế kích thước `0,2548 m` trong robot thật hay
trong thuật toán.

## Collision và các chi tiết màu đen

Một box 0,44 × 0,34 m ở toàn chiều cao sẽ va vào vùng bánh và làm sai contact.
Collision thân hiện là bốn box compound: lower center, front cross, rear cross
và upper deck. Bốn sphere Ø12 mm là contact vô hình cho các bi đỡ. Visual vẫn
giữ nguyên STL nên hình trong RViz2/Gazebo không bị đơn giản hóa.

Các vòng/tròn đen ở hai đầu trong ảnh không phải lidar, IMU hay sensor được thêm
bởi URDF. Không có link/joint/plugin riêng ở các vị trí đó; chúng là hốc/lỗ và
mặt trong tối của mesh CAD khi render. Cụm màu đen hình trụ ở phía trên mới là
housing lidar đã có sẵn trong mesh. IMU không có visual riêng và frame của nó
nằm trong khay dưới.

## Kết quả kiểm tra

- `check_urdf`: cây hợp lệ, root `base_link`, hai joint bánh continuous và các
  fixed joint `base_footprint`, `laser`, `imu_link`.
- `gz sdf -k`: SDF hợp lệ. Hai cảnh báo `gz_frame_id` là extension sensor của
  Gazebo, không phải frame sai.
- `/scan`: frame `laser`, khoảng 5,5 Hz, 1.440 tia/vòng.
- `/imu/data`: frame `imu_link`, khoảng 100 Hz.
- `/odom`: `odom → base_link`; `/ground_truth/odom`: `world → base_link`.
- TF tĩnh đo được: `base_link → laser` z ≈ 0,109 m và
  `base_link → imu_link` z ≈ −0,013 m.
- Chạy thẳng: sai lệch độ dài odometry/ground truth khoảng 0,02%.
- Quay tại chỗ: sai lệch góc odometry/ground truth khoảng 0,02%.
- Nav2 load đủ Navfn, Simple, Savitzky–Golay, Constrained và Pivot–G2; cả năm
  path được sinh thành công trên ba ca regression, tất cả phương pháp đã qua
  một lượt closed-loop với shared RPP.

Regression tĩnh nằm tại
`src/vacuum_robot_gazebo/test/test_robot_description.py`. Test kiểm tra cây
URDF, frame sensor, geometry bánh, collision compound, tensor quán tính, plugin,
bridge, profile encoder, footprint Nav2 và sự tồn tại của mesh.

## Phần phải đo lại trên xe thật

- `1320 tick/rev` phải được xác nhận là tick sau hộp số cho một vòng bánh và đã
  bao gồm đúng hệ số quadrature.
- Đường kính lăn trái/phải và effective wheel separation phải hiệu chỉnh bằng
  chạy thẳng 2 m và quay nhiều vòng hai chiều.
- Tọa độ tâm quang học lidar và trục BNO055 phải đo sau khi lắp thật.
- Inertia thân hiện lấy từ CAD/export để mô phỏng; cần cân/đo lại nếu nghiên cứu
  động lực học, còn benchmark hình học tốc độ thấp ít nhạy hơn với sai số này.

## Lệnh tái kiểm tra

```bash
check_urdf src/vacuum_robot_gazebo/urdf/vacuum_robot.urdf
gz sdf -k src/vacuum_robot_gazebo/models/vacuum_robot/model.sdf
colcon test --packages-select vacuum_robot_gazebo --event-handlers console_direct+
```
