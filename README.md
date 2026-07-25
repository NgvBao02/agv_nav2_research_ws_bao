# Adaptive Pivot–G2 research workspace

Workspace ROS 2 Jazzy + Gazebo Harmonic để phát triển và đánh giá bộ hậu xử lý
đường đi chọn thích nghi giữa quay tại chỗ (pivot) và transition Bézier bậc năm
liên tục G2 cho robot vi sai hai bánh. MATLAB chỉ còn là tài liệu tham chiếu;
luồng nghiên cứu chính chạy hoàn toàn trong ROS 2.

## Trạng thái hiện tại

- `adaptive_pivot_g2`: thư viện C++ lõi cho hình học G2 và time-parameterization.
- `adaptive_pivot_g2_nav2`: plugin `nav2_core::Smoother` đã load và chạy trong
  Nav2 Smoother Server; gồm Pivot–G2 và phương pháp lai có safety gate. Nhánh
  lai chỉ nhận chi phí pivot khi proximity cost cải thiện đủ ngưỡng và năng
  lượng độ cong vẫn nằm trong ngân sách công bố trước; nếu cả hai nhánh làm
  mượt không an toàn nhưng raw path an toàn, nó fallback về raw.
- `adaptive_pivot_g2_benchmark`: lập kế hoạch một lần, tuần tự đưa đúng cùng
  `nav_msgs/Path` vào Nav2 Simple, Savitzky–Golay, Constrained, Pivot–G2 và
  adaptive hybrid; xuất CSV/JSON, metric full-footprint, và chạy ma trận vòng
  kín bằng cùng một controller.
- `adaptive_pivot_g2_rviz`: panel RViz2 để đổi trực tiếp bảy môi trường, chọn
  một trong năm global planner, bật/tắt riêng từng baseline/Pivot/Hybrid và
  theo dõi metric cùng trạng thái profile vận tốc.
- `vacuum_robot_gazebo`: robot vi sai hai bánh dùng mesh CAD 440 × 340 mm,
  bảy cặp world/map Gazebo–Nav2 đồng nhất, trong đó có ba layout chuyên cho nhà
  kho, bridge Gazebo–ROS, Nav2 và RViz2. Cấu hình cảm biến mô phỏng đã bám theo
  RPLIDAR A1M8 và BNO055 của xe dự kiến.
- `matlab/pivot_g2`: bản lưu source thử ý tưởng cũ, không nằm trong đường chạy.

Raw và bảy biến thể smoother đã qua ma trận hình học dùng raw-path hash cố
định. Ma trận vòng kín yêu cầu đồng thời Nav2 action và ground truth Gazebo đạt
đích, lưu riêng ground truth, odom, pose ước lượng, command và telemetry profile
vận tốc, rồi dừng server cô lập sau mỗi trial. Báo cáo REV-ECIT hiện hành tách
rõ ma trận hình học toàn phần khỏi ma trận vòng kín phân tầng; không suy diễn
“tối ưu toàn cục” từ số liệu mô phỏng.

## Chạy nhanh

```bash
cd /home/linh-pham/agv_nav2_research_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch vacuum_robot_gazebo switchable_simulation.launch.py gui:=true
```

Sau khi RViz2 mở, đổi map bằng ô **MÔI TRƯỜNG GAZEBO / NAV2** ở panel bên
phải. Environment manager sẽ tắt stack cũ, khởi động đúng cặp world/map rồi
chỉ báo hoàn tất khi Nav2 mới đã active.

Để chạy trực tiếp một map cố định, ba map sát bài toán kho nhất là
`warehouse_long_aisles`, `warehouse_cross_aisles` và `warehouse_dispatch`:

```bash
ros2 launch vacuum_robot_gazebo simulation.launch.py \
  environment:=warehouse_long_aisles planner_id:=ThetaStar \
  execute:=true execute_method:=adaptive_hybrid \
  x_pose:=-2.0 y_pose:=-2.4 yaw:=1.5708
```

## Kiểm tra URDF riêng

Kiểm tra cú pháp và cây link/joint mà không chạy Gazebo:

```bash
check_urdf src/vacuum_robot_gazebo/urdf/vacuum_robot.urdf
```

Mở robot, TF và GUI xoay hai bánh trong RViz2:

```bash
colcon build --symlink-install --packages-select vacuum_robot_gazebo
source install/setup.bash
ros2 launch vacuum_robot_gazebo check_urdf.launch.py
```

Trong `RobotModel`, bật `Collision Enabled` và tắt `Visual Enabled` để kiểm tra
riêng collision. Grid nằm tại mặt đất, còn `base_link` nằm ở cao độ tâm bánh.
Frame lidar và IMU dùng đúng vị trí dự trữ từ xacro CAD gốc; housing lidar đã
nằm sẵn trong mesh thân nên không vẽ thêm cylinder.

## Chọn planner và xem đường trong RViz2

Trong RViz2, chọn tool **2D Goal Pose** rồi click/drag trên map. Mỗi goal sẽ
publish cùng input và các màu:

- đỏ: RAW planner;
- vàng: Nav2 Simple;
- cyan: Nav2 Savitzky–Golay;
- xanh lá: Nav2 Constrained;
- hồng nhạt: Pivot–G2 fixed;
- magenta: Pivot–G2 adaptive;
- xanh lam nhạt: Hybrid fixed;
- xanh lam đậm: Adaptive Hybrid;
- trắng: quỹ đạo xe thực thi.

Để chỉ xem và so sánh đường, đặt một goal sau khi map mới đã active. Ở panel
**Selector** bên phải RViz2:

1. đặt một goal bằng **2D Goal Pose**;
2. chọn `NavFn A*`, `NavFn Dijkstra`, `Theta*`, `Smac 2D` hoặc `Smac Hybrid`;
3. nhấn **Áp dụng và lập lại đường**.

Đường đỏ `RAW planner` được tạo lại bằng đúng planner đã chọn. Bảy nút riêng
cho Simple, Savitzky–Golay, Constrained, Pivot–G2 fixed/adaptive và Hybrid
fixed/adaptive cho phép ẩn/hiện từng đường; **Hiện tất cả** và **Chỉ RAW** là
hai thao tác nhanh. Tất cả phương pháp của một generation nhận đúng cùng raw
path, và bảng metric hiển thị kết quả riêng từng phương pháp.

Để xe bám đường đề xuất ngay từ lúc launch:

```bash
ros2 launch vacuum_robot_gazebo simulation.launch.py \
  execute:=true execute_method:=adaptive_hybrid
```

Có thể đổi phương pháp cho goal kế tiếp khi hệ thống đang chạy:

```bash
ros2 topic pub --once --qos-durability transient_local \
  /research/execute_method std_msgs/msg/String "{data: constrained}"
```

Headless cho test/CI:

```bash
ros2 launch vacuum_robot_gazebo simulation.launch.py \
  gui:=false rviz:=false execute:=false
```

Batch hình học công bằng; planner comparison dùng raw và mọi smoother nhận cùng
đường thô trong từng planner/kịch bản:

```bash
ros2 launch adaptive_pivot_g2_benchmark planner_benchmark.launch.py \
  scenario_file:=$PWD/src/adaptive_pivot_g2_benchmark/config/narrow_aisles_scenarios.yaml \
  output_csv:=$PWD/results/narrow_aisles.csv \
  output_json:=$PWD/results/narrow_aisles_summary.json
```

Launch tự đọc đúng environment từ scenario YAML, chạy xong rồi tắt stack. Danh
sách planner mặc định là `NavFnAStar`, `NavFnDijkstra`, `ThetaStar`, `Smac2D`
và `SmacHybrid`.

Ma trận Gazebo vòng kín, mỗi trial dùng domain/partition sạch và kiểm tra đích
bằng ground truth:

```bash
ros2 run adaptive_pivot_g2_benchmark execution_matrix -- \
  --scenario-file "$PWD/src/adaptive_pivot_g2_benchmark/config/open_arena_scenarios.yaml" \
  --scenario short_open_diagonal \
  --planners NavFnAStar NavFnDijkstra ThetaStar Smac2D SmacHybrid \
  --methods raw simple savitzky_golay constrained pivot_g2_fixed pivot_g2 \
    adaptive_hybrid_fixed adaptive_hybrid \
  --speed-limits 0.15 0.22 0 \
  --repetitions 3 --output-dir "$PWD/results/execution_matrix"
```

`--speed-limits 0` dùng trần tốc độ thích nghi; các giá trị dương tạo nhánh
đối chứng có trần cố định. `--resume` chỉ dùng lại JSON thành công có đúng
planner, smoother, tốc độ và repetition; lỗi khởi tạo Gazebo/Nav2 được retry
riêng, còn timeout/va chạm của thuật toán không bị che bằng retry.

## Build và test có chọn lọc

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  adaptive_pivot_g2 adaptive_pivot_g2_nav2 adaptive_pivot_g2_controller \
  adaptive_pivot_g2_benchmark adaptive_pivot_g2_rviz vacuum_robot_gazebo
colcon test --packages-select \
  adaptive_pivot_g2 adaptive_pivot_g2_nav2 adaptive_pivot_g2_controller \
  adaptive_pivot_g2_benchmark adaptive_pivot_g2_rviz vacuum_robot_gazebo \
  --event-handlers console_direct+
colcon test-result --verbose
```

## Tài liệu

- [Báo cáo toàn diện cho người mới: từ ROS 2, robot vi sai đến Adaptive Hybrid Pivot–G2](docs/BAO_CAO_TOAN_DIEN_ADAPTIVE_HYBRID_PIVOT_G2.html)
- [Bài báo REV-ECIT 2026](docs/REV_ECIT_2026_ADAPTIVE_HYBRID_PIVOT_G2_PAPER.html)
- [Phụ lục kết quả đầy đủ](docs/REV_ECIT_2026_ADAPTIVE_HYBRID_PIVOT_G2_SUPPLEMENT.html)
