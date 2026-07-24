# Ô chọn global planner trong RViz2

## Mục đích

Panel **Selector** cho phép đổi toàn bộ cặp Gazebo world/Nav2 map, đổi global
planner và bật/tắt riêng từng smoother mà không sửa YAML hay mở terminal thứ
hai. Năm lựa chọn planner khớp chính xác với plugin ID đã nạp trong Nav2:

| Tên trong panel | Plugin ID | Nhóm thuật toán |
|---|---|---|
| NavFn A* | `NavFnAStar` | A* trên lưới 2D |
| NavFn Dijkstra | `NavFnDijkstra` | Dijkstra trên lưới 2D |
| Theta* | `ThetaStar` | tìm đường any-angle |
| Smac 2D | `Smac2D` | A* cost-aware trên lưới |
| Smac Hybrid | `SmacHybrid` | Hybrid-A* SE(2), Dubins |

Đây là panel riêng của package `adaptive_pivot_g2_rviz`, không phải Selector
mặc định của Nav2. Panel còn nhận xác nhận từ runner/environment manager, nhớ
lựa chọn trong file cấu hình RViz và yêu cầu lập lại goal gần nhất.

## Cách dùng an toàn để xem đường

```bash
cd /home/linh-pham/agv_nav2_research_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch vacuum_robot_gazebo switchable_simulation.launch.py gui:=true
```

Chọn một trong bảy môi trường và nhấn **Đổi map / khởi động lại**. Chỉ đặt goal
sau khi trạng thái báo Nav2 mới đã active:

- `research_warehouse`;
- `warehouse_long_aisles`;
- `warehouse_cross_aisles`;
- `warehouse_dispatch`;
- `narrow_aisles`;
- `office_maze`;
- `open_arena`.

1. chọn **2D Goal Pose** trên thanh công cụ RViz2;
2. click và kéo hướng goal trong vùng trắng của map;
3. xem đường đỏ `RAW planner`;
4. trong panel **Selector** bên phải, chọn planner khác;
5. nhấn **Áp dụng và lập lại đường**.

Đường cũ được xóa ngay khi bắt đầu generation mới. Đường đỏ sau đó là raw path
của planner đang hiển thị trong dòng **Planner đang hoạt động**. Bảy output
smoother cùng nhận raw path mới đó:

| Nút | Phương pháp |
|---|---|
| Simple | Nav2 Simple |
| Savitzky–Golay | Nav2 Savitzky–Golay |
| Constrained | Nav2 Constrained |
| Pivot–G2 fixed | tập bán kính legacy |
| Pivot–G2 adaptive | tìm kiếm liên tục + DP |
| Hybrid fixed | safety-gated hybrid dùng Pivot fixed |
| Hybrid adaptive | safety-gated hybrid dùng Pivot adaptive |

Để so sánh trước và sau smooth:

1. nhấn từng nút phương pháp để bật/tắt đúng một lớp đường;
2. nhấn **Chỉ RAW** để giữ riêng đường planner màu đỏ;
3. nhấn **Hiện tất cả** để bật lại đủ bảy output.

Khi ẩn một phương pháp, runner publish path rỗng đúng topic đó nhưng không xóa
RAW hay các output còn bật. Khi bật lại, runner dùng lại chính RAW đang hiển
thị; planner không bị gọi lại nên so sánh trước/sau không bị thay đổi đầu vào.

Giữ `execute:=false` khi chỉ so sánh hình học. Nếu dùng `execute:=true`, sau
khi raw path và các smoother được tạo xong, robot sẽ bám phương pháp được chọn
bởi `execute_method`.

## Luồng topic và chống lỗi

```text
RViz panel
    ├── planner ID ──────────────► /planner_selector
    ├── JSON smoother visibility ► /research/smoother_visibility
    └── environment ID ──────────► /research/environment_selector
                                       │
                   ┌───────────────────┴────────────────────┐
                   ▼                                        ▼
             compare_paths                         environment_manager
        ComputePath + SmoothPath               restart world/map/Nav2
```

Các topic chọn và xác nhận dùng QoS `reliable + transient_local`. Vì vậy panel
mở sau runner vẫn nhận planner và trạng thái smoother hiện hành. Runner chỉ
chấp nhận đúng năm planner ID trong bảng; chuỗi rỗng, khác chữ hoa/thường hoặc
ID không tồn tại đều bị từ chối và planner cũ được giữ nguyên.

Mỗi lần nhấn nút, kể cả chọn lại cùng planner, runner:

- tăng `generation`;
- hủy yêu cầu planner/smoother cũ nếu còn chạy;
- xóa toàn bộ path cũ để không trộn hai planner;
- dùng lại goal RViz gần nhất;
- ghi `planner`, `generation` và metric của từng path vào
  `/research/metrics`.

Có thể kiểm tra không qua GUI:

```bash
ros2 topic pub --once --qos-durability transient_local \
  /planner_selector std_msgs/msg/String "{data: Smac2D}"

ros2 topic echo --once --qos-durability transient_local \
  /research/planner_active

ros2 topic pub --once --qos-durability transient_local \
  /research/smoother_visibility std_msgs/msg/String \
  '{data: "{\"methods\":[\"pivot_g2\",\"adaptive_hybrid\"]}"}'

ros2 topic pub --once --qos-durability transient_local \
  /research/environment_selector std_msgs/msg/String \
  "{data: warehouse_dispatch}"
```

## Nếu panel không xuất hiện

Trước hết phải source overlay sau khi build:

```bash
source /opt/ros/jazzy/setup.bash
source /home/linh-pham/agv_nav2_research_ws/install/setup.bash
```

Trong RViz2 có thể chọn **Panels → Add New Panel** rồi thêm
`adaptive_pivot_g2_rviz/Planner Selector`. File
`research_comparison.rviz` đã thêm sẵn panel này nên thao tác thủ công thường
không cần thiết.

Nếu mở từ terminal bên trong bản VS Code cài bằng Snap và gặp lỗi
`/snap/core20/.../libpthread.so.0`, hãy mở terminal hệ thống mới. Hoặc xóa các
đường dẫn GTK do Snap chèn trước khi launch:

```bash
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GIO_MODULE_DIR GIO_EXTRA_MODULES
export QT_IM_MODULE=none
```

## Kiểm thử tích hợp

- package C++ build thành công trên ROS 2 Jazzy;
- GTest xác nhận đủ năm planner, bảy environment và bảy smoother ID duy nhất,
  đồng thời từ chối ID không hợp lệ;
- `ament_copyright`, `cppcheck`, `cpplint`, `lint_cmake`, `uncrustify` và
  `xmllint` đều đạt;
- regression test xác nhận file RViz nạp đúng class panel và package mô phỏng
  khai báo đúng runtime dependency;
- RViz2 nạp cấu hình bằng OpenGL 4.6, không có lỗi class loader hay plugin;
- smoke test trên `warehouse_long_aisles`: Theta* tạo raw path 4,80 m, đổi sang
  Smac2D tự tạo generation mới cũng dài 4,80 m; ID giả `GridBased` bị từ chối
  và active planner vẫn là Smac2D.
- toggle test xác nhận ẩn/hiện riêng từng output không đổi RAW hash; regression
  của environment manager xác nhận chỉ chấp nhận bảy ID, dọn stack cũ và phát
  trạng thái chuyển map theo thứ tự xác định.

Đoạn 4,80 m là một lối đi thẳng nên hai planner cho cùng độ dài; mục tiêu của
smoke test này là xác nhận đúng luồng đổi planner. Để đánh giá khác biệt thuật
toán, dùng các scenario có góc cua và vật cản trong batch benchmark.
