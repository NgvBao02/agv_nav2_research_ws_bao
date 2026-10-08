# PSTMO — ROS 2/Nav2 research workspace

Workspace ROS 2 Jazzy + Gazebo Harmonic để phát triển, mô phỏng và đánh giá
PSTMO (Path Smoothing and Turning-Maneuver Optimization) cho robot vi sai hai
bánh. PSTMO hậu xử lý đường của global planner bằng các transition Bézier bậc
năm liên tục hình học G² hoặc thao tác quay tại chỗ, đồng thời kiểm tra hình
bao robot, giới hạn động học và khả năng ghép các góc liên tiếp.

Repo chứa mã nguồn ROS 2, mô hình CAD/URDF/SDF, bảy môi trường Gazebo–Nav2,
công cụ benchmark, dữ liệu thực nghiệm và các báo cáo nghiên cứu. Luồng chạy
chính hoàn toàn nằm trong ROS 2; không có thư mục MATLAB trong phiên bản hiện
tại.

## Cấu hình thuật toán hiện tại

- Plugin `pstmo` dùng tiền xử lý `condition_only`, tìm kiếm
  `hierarchical_alpha_two_trim` và tối ưu chuỗi trạng thái góc bằng quy hoạch
  động. Greedy line-of-sight (LOS) không được bật trong cấu hình mặc định.
- Plugin `adaptive_hybrid` là một nhánh nghiên cứu riêng: nó so sánh Nav2
  Simple với Pivot–G2 dùng `legacy_joint_d_q`, rồi chọn đối xứng theo peak cost
  và maneuver effort. Nếu cả hai đường làm mượt không an toàn nhưng Raw an
  toàn, plugin trả về Raw.
- Nav2 Smoother Server nạp năm plugin: Simple, Savitzky–Golay, Constrained,
  PSTMO và Adaptive Hybrid. `raw` là baseline thực thi, không phải smoother.
- Năm global planner có sẵn: `NavFnAStar`, `NavFnDijkstra`, `ThetaStar`,
  `Smac2D` và `SmacHybrid`.
- Controller hiện tại là `RegulatedPurePursuitController` với vận tốc hành
  trình 0,30 m/s và lookahead cố định 0,50 m. Curvature/cost speed scaling bị
  tắt; giới hạn gia tốc, bộ làm mượt vận tốc và Collision Monitor vẫn hoạt động.
- Goal checker là `SimpleGoalChecker` ở chế độ stateful, với dung sai vị trí
  0,06 m và dung sai hướng 0,10 rad. Runner vòng kín còn kiểm tra độc lập trạng
  thái dừng và sai số đích theo Gazebo ground truth.

Các dataset cũ trong `results/` ghi lại nhiều cấu hình thử nghiệm khác nhau,
bao gồm joint \((d,q)\), LOS và padding footprint. Chúng là snapshot lịch sử,
không tự động đại diện cho cấu hình đang nằm trong
`src/vacuum_robot_gazebo/config/nav2_params.yaml`; xem
[`results/README.md`](results/README.md) trước khi trích số liệu.

## Các package

| Package | Vai trò |
| --- | --- |
| `adaptive_pivot_g2` | Thư viện C++ lõi: conditioning, tìm ứng viên, transition G², tối ưu chuỗi góc và tham số hóa thời gian |
| `adaptive_pivot_g2_nav2` | Hai plugin `nav2_core::Smoother`: PSTMO độc lập và Adaptive Hybrid |
| `adaptive_pivot_g2_benchmark` | So sánh hình học, chạy thử vòng kín, metric clearance/localization/velocity và xuất CSV/JSON |
| `adaptive_pivot_g2_rviz` | Panel RViz2 để đổi môi trường, planner, phương pháp thực thi và lớp đường hiển thị |
| `vacuum_robot_gazebo` | Robot 440 × 340 mm, URDF/SDF, bridge, Nav2, RViz2 và bảy cặp world/map |

Bảy môi trường là `research_warehouse`, `open_arena`, `narrow_aisles`,
`office_maze`, `warehouse_long_aisles`, `warehouse_cross_aisles` và
`warehouse_dispatch`.

## Làm việc với coding agent

[`AGENTS.md`](AGENTS.md) là hợp đồng chung cho Codex và các coding agent khác.
Nó yêu cầu đọc context theo phạm vi task, không quét hàng loạt artifact lớn và
coi đồng bộ README/AGENTS là một phần bắt buộc của mọi thay đổi liên quan.

Bộ định tuyến ngắn trong [`docs/agent/`](docs/agent/) gồm bản đồ repo, lệnh
chuẩn và chính sách dataset. Nó là điểm bắt đầu để giảm đọc thừa, không giới
hạn phạm vi điều tra: agent vẫn phải mở rộng sang dependency, call site, test
và cấu hình liên quan khi task yêu cầu. Sau khi sửa README hoặc AGENTS, chạy:

```bash
python3 tools/check_repository_docs.py
git diff --check
```

## Yêu cầu và cài đặt

Môi trường mục tiêu là Ubuntu 24.04, ROS 2 Jazzy và Gazebo Harmonic:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup ros-jazzy-ros-gz ros-dev-tools
```

Nếu `rosdep` chưa được khởi tạo trên máy:

```bash
sudo rosdep init
rosdep update
```

Từ thư mục gốc repo, script sau cài dependency ROS còn thiếu rồi build toàn bộ
workspace bằng `colcon --symlink-install`:

```bash
./tools/bootstrap_workspace.bash
source install/setup.bash
```

Mỗi terminal mới cần source ROS và overlay của workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Chạy mô phỏng và RViz2

Chế độ thuận tiện nhất giữ RViz2 mở trong khi đổi đồng bộ Gazebo world và Nav2
map:

```bash
ros2 launch vacuum_robot_gazebo switchable_simulation.launch.py \
  gui:=true execute_method:=pstmo
```

Trong panel **Selector** bên phải RViz2:

1. chọn môi trường và nhấn **Đổi map và khởi động lại mô phỏng**;
2. đặt goal bằng công cụ **2D Goal Pose**;
3. chọn global planner rồi nhấn **Áp dụng và lập lại đường**;
4. chọn `RAW`, Simple, Savitzky–Golay, Constrained, PSTMO hoặc Adaptive Hybrid
   trong **CHỌN SMOOTHER ĐỂ XE ĐI THEO**;
5. dùng các nút phía dưới để ẩn/hiện từng đường so sánh.

Màu mặc định: Raw đỏ, Simple vàng, Savitzky–Golay cyan, Constrained xanh lá,
PSTMO magenta, Adaptive Hybrid xanh lam và đường xe thực thi màu trắng.

Chạy trực tiếp một môi trường cố định:

```bash
ros2 launch vacuum_robot_gazebo simulation.launch.py \
  environment:=warehouse_long_aisles planner_id:=ThetaStar \
  execute:=true execute_method:=pstmo \
  x_pose:=-2.0 y_pose:=-2.4 yaw:=1.5708
```

Các giá trị hợp lệ của `execute_method` là `none`, `raw`, `simple`,
`savitzky_golay`, `constrained`, `pstmo` và `adaptive_hybrid`. Mặc định launch
là `simple`; đặt `execute:=false` nếu chỉ muốn so sánh đường.

Đổi phương pháp cho goal kế tiếp khi phiên đang chạy:

```bash
ros2 topic pub --once --qos-durability transient_local \
  /research/execute_method std_msgs/msg/String "{data: constrained}"
```

Chạy headless:

```bash
ros2 launch vacuum_robot_gazebo simulation.launch.py \
  gui:=false rviz:=false execute:=false
```

## Benchmark

Benchmark hình học tạo một Raw path cho mỗi cặp planner–scenario rồi đưa đúng
đường đó vào mọi smoother đã chọn:

```bash
ros2 launch adaptive_pivot_g2_benchmark planner_benchmark.launch.py \
  scenario_file:=$PWD/src/adaptive_pivot_g2_benchmark/config/narrow_aisles_scenarios.yaml \
  scenario_names:=southwest_northeast_weave \
  smoothers:=simple,savitzky_golay,constrained,pstmo,adaptive_hybrid \
  output_csv:=$PWD/results/narrow_aisles.csv \
  output_json:=$PWD/results/narrow_aisles_summary.json
```

Ma trận vòng kín khởi động một stack Gazebo/Nav2 cô lập cho từng trial:

```bash
ros2 run adaptive_pivot_g2_benchmark execution_matrix -- \
  --scenario-file "$PWD/src/adaptive_pivot_g2_benchmark/config/open_arena_scenarios.yaml" \
  --scenario short_open_diagonal \
  --planners NavFnAStar NavFnDijkstra ThetaStar Smac2D SmacHybrid \
  --methods raw simple savitzky_golay constrained pstmo adaptive_hybrid \
  --repetitions 3 \
  --output-dir "$PWD/results/execution_matrix"
```

`--resume` chỉ tái sử dụng trial thành công có đúng planner, method, repetition
và fingerprint cấu hình. Lỗi khởi tạo hạ tầng được retry riêng; timeout hoặc
va chạm của thuật toán không bị che bằng retry.

## Build, test và kiểm tra URDF

```bash
colcon build --symlink-install --packages-select \
  adaptive_pivot_g2 adaptive_pivot_g2_nav2 \
  adaptive_pivot_g2_benchmark adaptive_pivot_g2_rviz vacuum_robot_gazebo

colcon test --packages-select \
  adaptive_pivot_g2 adaptive_pivot_g2_nav2 \
  adaptive_pivot_g2_benchmark adaptive_pivot_g2_rviz vacuum_robot_gazebo \
  --event-handlers console_direct+
colcon test-result --verbose
```

Kiểm tra cú pháp/cây link-joint và mở robot riêng trong RViz2:

```bash
check_urdf src/vacuum_robot_gazebo/urdf/vacuum_robot.urdf
ros2 launch vacuum_robot_gazebo check_urdf.launch.py
```

## Tài liệu và dữ liệu

- Báo cáo PSTMO thống nhất: [HTML](docs/PSTMO_unified.html),
  [DOCX](docs/PSTMO.docx), [PDF](docs/PSTMO.pdf).
- Báo cáo thuật toán toàn diện và dữ liệu nguồn 35 ca hình học × 5 phương án:
  [HTML](docs/BAO_CAO_TOAN_DIEN_PSTMO.html) và
  [`docs/pstmo_bao_cao_toan_dien_assets/`](docs/pstmo_bao_cao_toan_dien_assets/).
- Hồ sơ lý thuyết 62 trang:
  [DOCX](docs/CO_SO_LY_THUYET_TOAN_DIEN_DU_AN_AGV_PSTMO.docx) và
  [ghi chú tái dựng](docs/theory_report/README.md).
- Nghiên cứu 125 lượt trên năm tuyến kho giao cắt:
  [PDF](docs/PSTMO_KHO_GIAO_CAT_5_QUY_DAO.pdf) và
  [hồ sơ dữ liệu](docs/warehouse_cross_aisles_5_routes/README.md).
- Hồ sơ mô hình 3D và mô phỏng robot:
  [PDF](docs/BAO_CAO_MO_HINH_3D_STEP_URDF_GAZEBO_RVIZ2.pdf) và
  [hồ sơ kiểm chứng](docs/robot_3d_report/README.md).
- [Thuật ngữ Anh–Việt (PDF)](docs/PSTMO_thuat_ngu_Anh_Viet.pdf),
  [slide PSTMO](<docs/slide PSTMO.pptx>),
  [bài báo REV-ECIT 2026](<bao_Rev_ecit_2026/ver2/REV ECIT2026 PSTMO.pdf>) và
  [bài báo ICEEIS 2026](final_bao_ICEEIS/final/ieee/ICEEIS2026_PSTMO_final.pdf).
- [`results/README.md`](results/README.md) phân loại dataset hiện hành, snapshot
  lịch sử, ablation và pilot; [`REFERENCES/`](REFERENCES/) chứa tài liệu tham
  khảo được lưu cùng repo.
