# Audit selector Hybrid trung lập — 27/07/2026

Thư mục này là bằng chứng trước–sau cho thay đổi từ gate một chiều ưu tiên
Simple sang selector đối xứng. Các số dưới đây được tính từ file CSV/JSON trong
chính thư mục này; RViz chỉ trực quan hóa path ROS, không được dùng để đo metric.

## Thay đổi được kiểm tra

- Simple và Pivot cùng yêu cầu center cost tối đa 252 (dưới
  `INSCRIBED_INFLATED_OBSTACLE=253`) và cùng phép đánh giá
  swept-footprint/lethal.
- Mỗi candidate nhận `max_time / 2`.
- Nếu chênh peak cost ít nhất 20, candidate có cost thấp hơn thắng theo cả hai
  chiều.
- Trong deadband cost, candidate có maneuver effort thấp hơn ít nhất 5% thắng.
  `maneuver_effort = integral(kappa^2 ds) + total_pivot_angle / 0.2548`.
- Cost dư, chiều dài và cuối cùng exact-metric tie là các tie-break xác định.
  Raw chỉ fallback khi cả hai candidate làm mượt đều không an toàn.

## Ma trận hình học Gazebo/Nav2

Lệnh benchmark giống nhau ở hai phiên bản:

```bash
ros2 launch adaptive_pivot_g2_benchmark planner_benchmark.launch.py \
  scenario_file:=$PWD/src/adaptive_pivot_g2_benchmark/config/narrow_aisles_scenarios.yaml \
  output_csv:=... repetitions:=1 gui:=false
```

Mỗi phía có 320 hàng = 5 planner × 8 scenario × 8 method. Cả hai phía thành
công 320/320; 320/320 cặp có cùng `raw_path_sha256` và
`planner_output_path_sha256`.

| Adaptive Hybrid | Simple | Pivot–G2 | Eκ tịnh tiến TB (1/m) | Clearance min TB (m) | Runtime TB (ms) |
|---|---:|---:|---:|---:|---:|
| Gate cũ | 38 | 2 | 34,683 | 0,291586 | 9,975 |
| Selector đối xứng | 29 | 11 | 33,495 | 0,292867 | 10,350 |

Lý do chọn ở selector mới: 15 Simple lower peak cost, 6 Simple lower maneuver
effort, 1 Simple lower residual cost; 2 Pivot lower peak cost, 8 Pivot lower
maneuver effort, 1 Pivot shorter path; 7 path thẳng hòa mọi metric và dùng
tie-break ổn định. Vì vậy tỷ lệ 29/11 là kết quả metric, không phải quota 50/50.

## Closed-loop Gazebo

Scenario: `narrow_aisles/bottom_alternating_cross`, planner
`NavFnDijkstra`, tốc độ thích nghi, một repetition mỗi method. Hai ma trận đều
`all_successful=true`, `same_raw_path=true`,
`paired_comparison_valid=true`; mọi trial tới đích, settled và có 0 can thiệp
collision monitor.

| Phiên bản/method | Path được chạy | Thời gian (s) | Eκ path kế hoạch (1/m) | Estimated tracking RMSE (m) | Sai số vị trí cuối (m) |
|---|---|---:|---:|---:|---:|
| Gate cũ / Hybrid | Simple | 43,557 | 2,652243 | 0,007610 | 0,029496 |
| Selector mới / Simple | Simple | 43,152 | 2,652243 | 0,006928 | 0,031851 |
| Selector mới / Pivot–G2 | Pivot–G2 | 31,944 | 0,425150 | 0,006113 | 0,026875 |
| Selector mới / Hybrid | Pivot–G2 | 30,678 | 0,425150 | 0,004741 | 0,032334 |

Hybrid mới có `selected_path_sha256` trùng đúng Pivot–G2
(`88c59a...f765`) thay vì Simple (`6af76a...f4537`). So với lượt Hybrid cũ,
thời gian giảm 29,6% và estimated tracking RMSE giảm 37,7%; sai số vị trí cuối
tăng từ 2,95 cm lên 3,23 cm nhưng vẫn dưới tolerance 10 cm. Một repetition chỉ
xác nhận cơ chế và không đủ cho kết luận thống kê về thời gian.

## RViz2

Một phiên `simulation.launch.py` thật được chạy với Gazebo
`narrow_aisles`, RViz2, `compare_paths`, `NavFnDijkstra`, `execute:=false`; goal
được publish tại `(2,0; -2,6)`. RViz nhận và hiển thị Raw cùng bảy path làm mượt.
Diagnostics của Adaptive Hybrid trong phiên này:

- selected: `pivot_g2`
- reason: `pivot_lower_peak_cost`
- peak cost Simple/Pivot: `177/144`
- maneuver effort Simple/Pivot: `3,51609/0,652981`
- candidate time budget: `1,5 s` mỗi nhánh

Ảnh chụp cửa sổ RViz:
[`rviz_neutral_selector.png`](rviz_neutral_selector.png), SHA-256
`1ec7abac66d6764c76c95bb75a645cfd76d5c57b5e578b5c71cce12d0e17407c`.

## File dữ liệu

- `baseline_narrow_aisles.csv` và `neutral_narrow_aisles.csv`: dữ liệu hình học
  trước–sau.
- Hai file `*_summary.json`: tổng hợp do benchmark sinh.
- `baseline_closed_loop/` và `neutral_closed_loop/`: scalar summary, full trace
  JSON và log; full trace/log được gzip lossless.
- `rviz_neutral_selector.png`: bằng chứng trực quan RViz của phiên selector mới.
