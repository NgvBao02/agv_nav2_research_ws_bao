# Luồng ROS 2 / Gazebo cho Pivot–G2

## Kết quả đã dựng

Hệ thống hiện chạy trên ROS 2 Jazzy, Gazebo Harmonic và Nav2. World warehouse
12 × 8 m chỉ dùng primitive nội bộ nên không phụ thuộc Fuel/Internet. File map
PGM/YAML được sinh bằng script từ đúng cùng kích thước vật cản của world.

Robot mô phỏng dùng mesh từ `xe_urdf_export_description.zip`. Kích thước STL đã
được đọc trực tiếp thay vì tin vào collision cũ trong xacro:

| Thuộc tính | Giá trị |
|---|---:|
| Topic lệnh / odometry / scan / IMU | `/cmd_vel`, `/odom`, `/scan`, `/imu/data` |
| Frames | `map → odom → base_link → {laser, imu_link}` |
| Thân CAD | 0,44 × 0,34 × 0,16 m |
| Bán kính bánh | 0,0425 m |
| Bề rộng toàn bộ part / mặt lốp lăn | 0,0424 / 0,0300 m |
| Khoảng cách tâm hai vệt bánh | 0,2548 m (CAD, chưa hiệu chuẩn xe thật) |
| Wheel separation hiệu dụng riêng của plugin Gazebo | 0,2809 m |
| Giới hạn vận tốc thẳng | 0,08 m/s |
| Giới hạn vận tốc góc | 0,425 rad/s |

Hai bánh đường kính thực từ STL là khoảng 84,99 mm. Toàn bộ part rộng 42,4 mm
nhưng connected component của mặt lốp lăn rộng 30 mm; collision vì vậy dùng
Ø85 × 30 mm và hai link bánh nằm tại tâm vệt lăn. Khoảng cách 254,8 mm được suy
ra từ CAD và phải hiệu chuẩn lại bằng phép quay nhiều vòng trên xe thật.
Riêng plugin DiffDrive của Gazebo dùng `0,2809 m`: đây là hệ số hiệu chỉnh động
học 1,1023 cho mô hình sáu điểm tiếp xúc, không phải kích thước cơ khí. Giá trị
`0,2548 m` vẫn được dùng trong URDF, lõi Pivot–G2 và hồ sơ xe thật.
DiffDrive chỉ điều khiển hai khớp bánh. STL có bốn
bi đỡ Ø12 mm chạm đất tại `(x, y) = (±0,15, ±0,09) m`; Gazebo dùng bốn collision
hình cầu vô hình đúng các vị trí này, không thêm visual và không đưa chúng vào
động học vi sai. Frame lidar lấy từ xacro CAD gốc, cao 0,15142 m so với mặt đất;
housing lidar đã nằm trong mesh nên không thêm primitive hình trụ. Sensor Gazebo
hiện dùng profile A1M8 360°, 5,5 Hz,
1.440 tia/vòng, khoảng đo 0,15–12 m. IMU mô phỏng chạy 100 Hz trong frame
`imu_link`, nằm trong khay dưới tại cao độ CAD 0,0297 m so với mặt đất.

```text
Gazebo world + 2-wheel differential robot
        │  /scan /imu/data /odom /tf /joint_states
        ▼
    ros_gz_bridge ───────────────┐
        │                        │
        ▼                        ▼
 map_server + AMCL       robot_state_publisher
        │
        ▼
 5 global planners ── một raw path ── benchmark runner
                              │
             ┌────────────────┼───────────────┐
             ▼                ▼               ▼
       Nav2 baselines     Pivot–G2       metrics JSON
             └────────────────┬───────────────┘
                              ▼
                    RViz2 / shared RPP
                              ▼
                         `/cmd_vel`
```

## Topics nghiên cứu

| Topic | Nội dung |
|---|---|
| `/research/goal_pose` | Goal dành cho runner so sánh |
| `/planner_selector` | Planner ID do panel RViz2 gửi |
| `/research/planner_active` | Planner ID đã được runner chấp nhận |
| `/research/path/raw` | Raw path của global planner đang chọn |
| `/research/path/simple` | Nav2 Simple Smoother |
| `/research/path/savitzky_golay` | Nav2 Savitzky–Golay |
| `/research/path/constrained` | Nav2 Constrained Smoother |
| `/research/path/pivot_g2` | Phương pháp đề xuất |
| `/research/path/executed` | Quỹ đạo `map → base_link` lấy ở 10 Hz |
| `/research/metrics` | JSON hình học, runtime, RMSE bám đường, sai số cuối và kết quả execution |
| `/research/pivot_g2/diagnostics` | Số corner/G2/pivot và runtime plugin |
| `/research/execute_method` | Chọn method thực thi cho goal kế tiếp |
| `/ground_truth/odom` | Pose vật lý trực tiếp từ Gazebo để đối chiếu `/odom` |

## Pivot–G2 ROS plugin hiện tại

Wrapper đã làm được các phần sau:

1. bỏ điểm trùng, LOS-prune bằng toàn footprint và giảm góc rất nhỏ;
2. sinh ứng viên G2 cho các bán kính 0,20–0,50 m;
3. lấy mẫu swept footprint trên curve và toàn góc quay pivot;
4. loại pose có inflation cost tại tâm trên `max_footprint_cost=200` (xấp xỉ
   0,05 m ngoài footprint với inflation hiện tại), đồng thời kiểm tra full swept
   footprint với lethal cells để không inflation robot hai lần;
5. time-parameterize ứng viên và pivot bằng cùng giới hạn vật lý/common window;
6. chọn G2 khi nhanh hơn pivot ít nhất 0,15 s hoặc pivot không an toàn;
7. fallback pivot bằng hai pose trùng vị trí nhưng khác heading.

Kiểm thử ngày 23/07/2026 dùng cùng raw path Navfn cho cả bốn smoother. Cả năm
output (`raw` và bốn smoother) đều thành công ở ba ca: thẳng 2 m, tránh vật cản
6,7 m và tuyến nhiều góc 13,9 m. Ở tuyến nhiều góc, `∫κ²ds` rời rạc lần lượt là
13,77 (Simple), 64,12 (Savitzky–Golay), 32,13 (Constrained) và 4,53 (Pivot–G2);
Pivot–G2 tạo ba transition G2, không cần fallback pivot và chạy core khoảng
11,2 ms. Đây vẫn chỉ là regression kỹ thuật, chưa phải thống kê để công bố.

Raw và cả bốn smoother đều đã báo `Reached the goal` bằng cùng RPP. Trên các
lượt thẳng khoảng 2 m sau khi sửa metric, RMSE bám đường của Simple,
Savitzky–Golay, Constrained và Pivot–G2 lần lượt khoảng 8,0; 13,0; 9,0 và
15,2 mm. Các lượt đi ngược chiều có thêm rotate-to-heading nên thời gian không
được dùng để xếp hạng giữa method. Runner hiện resample quỹ đạo ở 0,05 m, lọc
nhiễu AMCL và tính curvature trên cửa sổ 0,15 m; cách cũ dùng trực tiếp mẫu 5 mm
đã bị bỏ vì tạo curvature giả rất lớn.

Đối chiếu động lực học độc lập qua `/ground_truth/odom` cho sai lệch quãng đường
thẳng khoảng 0,02% và sai lệch góc quay tại chỗ khoảng 0,02% sau hiệu chỉnh
wheel separation hiệu dụng của plugin. Ground truth lấy trực tiếp từ
`gz::sim::systems::OdometryPublisher`, không dùng lại odometry bánh xe.

## Những phần chưa ổn và không được che giấu

- RPP collision arc và Collision Monitor approach đã bật lại và qua hai smoke
  test. Tuy vậy vẫn phải có regression vật cản thật trước khi dùng trên xe.
- RPP chuẩn có thể bám curve G2 nhưng không bảo đảm thực thi đúng pivot được mã
  hóa bằng hai pose trùng vị trí. Cần một executor/controller nhận biết pivot,
  hoặc dùng controller chung có cùng semantics cho tất cả method.
- Collision thân dùng compound primitives phủ đủ envelope 440 × 340 mm, chừa
  hốc bánh thấp quanh trục; phần nhìn vẫn dùng mesh CAD. Bốn bi đỡ dùng collision
  cầu Ø12 mm suy ra trực tiếp từ các connected component của STL; vẫn cần xác
  nhận chúng đúng là các điểm tỳ thực trên cơ khí đã lắp.
- DiffDrive Gazebo giữ lệnh `/cmd_vel` cuối nếu publish tay bị ngắt; Nav2 có
  velocity timeout và phát zero, nhưng test thủ công phải luôn gửi lệnh dừng.
- Plugin mới có time gate + cost threshold; chưa có score đa mục tiêu đầy đủ,
  clearance liên tục, xử lý tương tác giữa hai corner gần nhau hay global optimum.
- Runner đã lọc metric quỹ đạo và xuất execution time, tracking RMSE/max cùng
  final-position error. Chưa có dataset khóa, rosbag recorder, CSV aggregate,
  commit/params hash và thống kê paired tự động.

## Thứ tự phát triển tiếp theo

1. Thêm regression “đường thoáng không false stop” và “vật cản bắt buộc stop”
   cho RPP/Collision Monitor đang được bật.
2. Viết pivot-aware execution dùng rotate-in-place có dừng hoàn toàn, vẫn dùng
   chung controller pipeline cho mọi baseline.
3. Mở rộng diagnostics từng corner: clearance min, radius, thời gian G2/pivot,
   lý do reject; thêm ablation time-only/cost-only/pivot-only/G2-only.
4. Khóa tập map/start/goal và runner batch ghi rosbag + CSV + metadata; sau đó mới
   chạy số lần lặp và kiểm định trong `EXPERIMENT_PLAN.md`.

## Các file chính

- Launch: `src/vacuum_robot_gazebo/launch/simulation.launch.py`
- World/map/model: `src/vacuum_robot_gazebo/{worlds,maps,models}`
- Nav2: `src/vacuum_robot_gazebo/config/nav2_params.yaml`
- RViz2: `src/vacuum_robot_gazebo/rviz/research_comparison.rviz`
- Plugin: `src/adaptive_pivot_g2_nav2`
- Runner: `src/adaptive_pivot_g2_benchmark`
