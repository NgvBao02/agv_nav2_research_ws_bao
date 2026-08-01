# Benchmark PSTMO rút gọn ngày 02/08/2026

> **Đính chính phần vòng kín:** các kết quả 75,21/113,49 s bên dưới dùng cấu
> hình goal termination gây vòng lặp quay tại đích và không còn được dùng để
> báo cáo. Kết quả chạy lại hợp lệ nằm tại
> `../current_pstmo_reduced_20260802_fixed/`. Phần hình học 70 bản ghi trong
> thư mục này không bị ảnh hưởng.

Dataset này đánh giá phiên bản code hiện tại của `pivot_g2` (PSTMO) và không
chạy `adaptive_hybrid`. Phần hình học có 70 bản ghi đường; phần vòng kín tách
riêng chỉ có 6 lượt robot chạy trong Gazebo.

## Thiết kế

- 7 môi trường, mỗi môi trường chọn 1 tình huống có chuyển hướng hoặc vật cản.
- 5 global planner: NavFn A*, NavFn Dijkstra, Theta*, Smac 2D và Smac Hybrid.
- 1 lần chạy cho mỗi cặp tình huống–planner.
- 2 đường được đánh giá trên cùng planner output: Raw và PSTMO (`pivot_g2`).
- Tổng cộng 35 nhóm ghép cặp và 70 bản ghi đường.
- Commit nền: `37c1d3084adcdaa24021d66cb0d28a8e671afa25`.
- SHA-256 phần sửa bộ lọc benchmark:
  `71bfb2aba6933f4f08a7ea21ba713efd3060d289c6a4df7f3ba81ba0aada9523`.

| Môi trường | Tình huống |
|---|---|
| research_warehouse | lower_left_diagonal |
| narrow_aisles | southwest_northeast_weave |
| office_maze | office_long_diagonal |
| open_arena | center_block_detour |
| warehouse_cross_aisles | cross_aisle_transfer |
| warehouse_dispatch | full_replenishment |
| warehouse_long_aisles | diagonal_replenishment |

Mỗi file được chạy bằng mẫu lệnh sau, thay `scenario_file`, `scenario_names`
và tên file đầu ra theo môi trường:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch adaptive_pivot_g2_benchmark planner_benchmark.launch.py \
  scenario_file:=$PWD/src/adaptive_pivot_g2_benchmark/config/open_arena_scenarios.yaml \
  scenario_names:=center_block_detour \
  smoothers:=pivot_g2 repetitions:=1 gui:=false \
  output_csv:=$PWD/results/current_pstmo_reduced_20260802/open_arena.csv \
  output_json:=$PWD/results/current_pstmo_reduced_20260802/open_arena_summary.json
```

## Kiểm định dữ liệu

- Raw thành công 35/35; PSTMO thành công 35/35.
- Cả 35 nhóm đều có đúng hai dòng Raw/PSTMO.
- Không có nhóm nào sai `raw_path_sha256`.
- Số dòng Adaptive Hybrid bằng 0.
- PSTMO có 0 mẫu va chạm footprint trong các đường được đánh giá.
- Tổng cộng thuật toán chọn 221 chuyển tiếp G² và 6 thao tác pivot.

## Kết quả ghép cặp

Phần trăm thay đổi được tính từ tỷ số giữa hai giá trị trung bình PSTMO và Raw
trên cùng 35 cặp.

| Chỉ số | Raw | PSTMO | Thay đổi |
|---|---:|---:|---:|
| Năng lượng độ cong tịnh tiến trung bình (1/m) | 49,239 | 4,595 | −90,7% |
| Chiều dài đường tịnh tiến trung bình (m) | 10,458 | 10,335 | −1,17% |
| Khoảng hở footprint nhỏ nhất trung bình (m) | 0,157 | 0,172 | +9,4% |

Thời gian thuật toán PSTMO trung bình là 69,51 ms, trung vị 63,0 ms; wall time
trung bình là 94,17 ms.

| Môi trường | Cặp | Δ năng lượng độ cong | Δ chiều dài | Δ khoảng hở nhỏ nhất |
|---|---:|---:|---:|---:|
| narrow_aisles | 5 | −88,4% | −1,18% | −8,3% |
| office_maze | 5 | −95,0% | −1,36% | +15,0% |
| open_arena | 5 | −87,9% | −2,23% | −0,7% |
| research_warehouse | 5 | −87,4% | −1,02% | +17,0% |
| warehouse_cross_aisles | 5 | −92,4% | −1,79% | −8,2% |
| warehouse_dispatch | 5 | −89,6% | −0,97% | +87,4% |
| warehouse_long_aisles | 5 | −87,4% | −0,58% | +18,0% |

Khoảng hở trung bình tăng trên toàn ma trận nhưng giảm ở ba môi trường. Vì chỉ
có một tình huống trên mỗi môi trường và một lần chạy, dataset này phù hợp để
báo cáo kiểm chứng rút gọn, không đủ cho kiểm định thống kê hoặc kết luận rằng
PSTMO luôn làm tăng khoảng hở.

## Kiểm chứng vòng kín ban đầu — không dùng để báo cáo

Ba tình huống được chạy bằng Theta* với cả Raw và PSTMO, một lần cho mỗi
phương pháp, tạo ba cặp và sáu lượt Gazebo:

- `research_warehouse/lower_left_diagonal`;
- `narrow_aisles/southwest_northeast_weave`;
- `warehouse_dispatch/full_replenishment`.

Cả 6/6 lượt đều thành công, robot dừng ổn định, không có can thiệp của collision
monitor và không có mẫu va chạm footprint trên đường lập kế hoạch. Ba cặp đều
dùng cùng `raw_path_sha256`; số lượt Adaptive Hybrid bằng 0.

| Chỉ số vòng kín trung bình | Raw | PSTMO |
|---|---:|---:|
| Thời gian hoàn thành (s) | 75,21 | 113,49 |
| RMSE bám đường theo ground truth (cm) | 3,00 | 2,79 |
| RMSE theo ước lượng controller (cm) | 1,45 | 1,55 |
| Sai số vị trí cuối (cm) | 3,52 | 3,74 |

Các số liệu trong bảng này đã bị thay thế. Lượt
`research_warehouse/lower_left_diagonal`, nơi PSTMO mất 170,01 s trong khi Raw
mất 44,20 s, bị vòng lặp goal termination sau khi robot đã tới đích. Không dùng
bảng này để kết luận về thời gian hoặc chất lượng bám đường; xem dataset
`_fixed` để lấy sáu lượt chạy lại.

## Tệp chính

- `combined.csv`: toàn bộ 70 dòng kèm cột `environment`.
- `aggregate_summary.json`: thiết kế, kiểm định ghép cặp và số liệu tổng hợp.
- `closed_loop_summary.json`: tổng hợp ba cặp Raw/PSTMO chạy kín.
- `*_summary.json`: tổng hợp riêng do benchmark sinh cho từng môi trường.
- `logs/*.log.gz`: log benchmark hình học được nén lossless.
- `closed_loop/*`: trace và log nén lossless (`*.gz`) cùng summary của sáu lượt
  robot chạy kín.
