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
- `adaptive_pivot_g2_rviz`: panel RViz2 để chọn trực tiếp một trong năm global
  planner, xác nhận planner đang hoạt động và tự lập lại đường tới goal gần
  nhất.
- `vacuum_robot_gazebo`: robot vi sai hai bánh dùng mesh CAD 440 × 340 mm,
  bảy cặp world/map Gazebo–Nav2 đồng nhất, trong đó có ba layout chuyên cho nhà
  kho, bridge Gazebo–ROS, Nav2 và RViz2. Cấu hình cảm biến mô phỏng đã bám theo
  RPLIDAR A1M8 và BNO055 của xe dự kiến.
- `matlab/pivot_g2`: bản lưu source thử ý tưởng cũ, không nằm trong đường chạy.

Raw và năm smoother đã qua regression sinh path. Ma trận vòng kín hiện yêu cầu
cả Nav2 action lẫn ground truth đạt đích; nó cũng dừng Gazebo server tách rời
sau mỗi trial. Kết quả hiện tại đủ cho pilot study nhưng chưa đủ để khẳng định
thống kê: sáu map sinh tự động đã có smoke test planner/smoother, ba map kho đã
có thêm smoke test vòng kín, còn ma trận nhiều lần lặp mới chỉ có một scenario
trên `open_arena`. Xem kết luận trung thực và lộ trình còn lại trong
[RESEARCH_STATUS_20260723.md](docs/RESEARCH_STATUS_20260723.md).

## Chạy nhanh

```bash
cd /home/linh-pham/agv_nav2_research_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch vacuum_robot_gazebo simulation.launch.py execute:=false
```

Chọn một trong bảy map. Ba map sát bài toán kho nhất là
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
- magenta: Pivot–G2 đề xuất;
- xanh lam: adaptive hybrid đề xuất;
- trắng: quỹ đạo xe thực thi.

Để chỉ xem và so sánh đường, nên launch với `execute:=false`. Ở panel
**Selector** bên phải RViz2:

1. đặt một goal bằng **2D Goal Pose**;
2. chọn `NavFn A*`, `NavFn Dijkstra`, `Theta*`, `Smac 2D` hoặc `Smac Hybrid`;
3. nhấn **Áp dụng và lập lại đường**.

Đường đỏ `RAW planner` sẽ được xóa rồi tạo lại từ vị trí hiện tại bằng đúng
planner đã chọn; năm đường smoother cũng được tính lại từ raw path mới. Dòng
trạng thái trong panel chỉ báo thành công sau khi node so sánh phản hồi trên
`/research/planner_active`.

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
  --methods raw adaptive_hybrid \
  --repetitions 3 --output-dir "$PWD/results/execution_matrix"
```

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

- [Luồng ROS 2/Gazebo và các giới hạn hiện tại](docs/ROS2_GAZEBO_PIPELINE.md)
- [Kiểm tra chi tiết URDF/SDF và hình học CAD](docs/URDF_VALIDATION.md)
- [Kiến trúc tích hợp GA25 encoder, A1M8 và BNO055](docs/HARDWARE_INTEGRATION.md)
- [Kiểm toán thuật toán](docs/ALGORITHM_AUDIT.md)
- [Kế hoạch thực nghiệm đến REV-ECIT 2026](docs/EXPERIMENT_PLAN.md)
- [Trạng thái nghiên cứu, số liệu pilot và hướng bài báo](docs/RESEARCH_STATUS_20260723.md)
- [So sánh 5 planner và 3 map Gazebo mới](docs/PLANNER_MAP_BENCHMARK.md)
- [Bộ map nhà kho, scenario và kết quả smoke test](docs/WAREHOUSE_MAPS.md)
- [Cách dùng và kiểm thử ô chọn planner trong RViz2](docs/RVIZ_PLANNER_SELECTOR.md)
- [Kiểm toán source MATLAB lưu trữ](docs/MATLAB_SOURCE_AUDIT.md)
