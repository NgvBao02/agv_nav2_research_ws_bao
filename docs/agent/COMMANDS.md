# Lệnh chuẩn

Chạy từ root workspace. Chỉ dùng nhóm lệnh phù hợp với task.

## Môi trường và bootstrap

```bash
source /opt/ros/jazzy/setup.bash
./tools/bootstrap_workspace.bash
source install/setup.bash
```

Trong terminal đã build, chỉ cần source hai setup file:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Build và test có chọn lọc

```bash
colcon build --symlink-install --packages-select <package...>
colcon test --packages-select <package...> --event-handlers console_direct+
colcon test-result --verbose
```

Quan hệ package thường gặp:

- Lõi PSTMO: `adaptive_pivot_g2`.
- Plugin PSTMO: `adaptive_pivot_g2 adaptive_pivot_g2_nav2`.
- Tích hợp đầy đủ: thêm `adaptive_pivot_g2_benchmark`,
  `adaptive_pivot_g2_rviz`, `vacuum_robot_gazebo`.

## Mô phỏng

Phiên có thể đổi môi trường trong RViz2:

```bash
ros2 launch vacuum_robot_gazebo switchable_simulation.launch.py \
  gui:=true execute_method:=pstmo
```

Phiên cố định, headless và không thực thi đường:

```bash
ros2 launch vacuum_robot_gazebo simulation.launch.py \
  gui:=false rviz:=false execute:=false
```

Kiểm tra launch argument mà không chạy mô phỏng:

```bash
ros2 launch vacuum_robot_gazebo simulation.launch.py --show-args
ros2 launch adaptive_pivot_g2_benchmark planner_benchmark.launch.py --show-args
```

## Benchmark có giới hạn phạm vi

Một scenario, các smoother được chọn:

```bash
ros2 launch adaptive_pivot_g2_benchmark planner_benchmark.launch.py \
  scenario_file:=$PWD/src/adaptive_pivot_g2_benchmark/config/open_arena_scenarios.yaml \
  scenario_names:=center_block_detour \
  smoothers:=simple,savitzky_golay,constrained,pstmo \
  output_csv:=$PWD/results/<experiment>/open_arena.csv \
  output_json:=$PWD/results/<experiment>/open_arena_summary.json
```

Không ghi vào thư mục dataset đã tồn tại. Xem `DATA_POLICY.md` trước khi tạo
ma trận mới.

## Kiểm tra tài liệu

Sau khi sửa README hoặc AGENTS:

```bash
python3 tools/check_repository_docs.py
git diff --check
```

## Đọc dữ liệu lớn có mục tiêu

```bash
jq '{audit, failures, protocol}' path/to/aggregate.json
rg -n 'error|failed|timeout' path/to/log
sed -n 'START,ENDp' path/to/text-file
```

Không dùng `cat` với trace JSON/log lớn và không render PDF/DOCX nếu task không
liên quan tới bố cục tài liệu.
