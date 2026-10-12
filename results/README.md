# Chỉ mục kết quả nghiên cứu

Thư mục này chứa cả dữ liệu báo cáo hiện hành lẫn các snapshot thử nghiệm theo
tiến trình phát triển. Tên có chữ `current` hoặc `final` phản ánh thời điểm tạo
dataset, không có nghĩa dataset đó luôn khớp với source mới nhất.

Cấu hình source hiện tại của plugin `pstmo` là `condition_only` +
`hierarchical_alpha_two_trim`; plugin `adaptive_hybrid` vẫn dùng Pivot–G2
`legacy_joint_d_q`. Khi trích số liệu, phải đối chiếu README/aggregate/config
hash của đúng thư mục, không gộp các cohort khác cấu hình.

## Bộ dữ liệu khớp cấu hình PSTMO hiện tại

### So sánh hình học 35 ca × 5 phương án

Dữ liệu đã chuẩn hóa nằm tại
[`docs/pstmo_bao_cao_toan_dien_assets/`](../docs/pstmo_bao_cao_toan_dien_assets/):

- `benchmark_hinh_hoc_175_luot.csv`: 35 cặp môi trường–planner × Raw, Simple,
  Savitzky–Golay, Constrained và PSTMO;
- `benchmark_hinh_hoc_tong_hop.json`: kiểm định ma trận và số liệu tổng hợp;
- `rviz_cases/`: 35 JSON chẩn đoán xác nhận `condition_only`,
  `hierarchical_alpha_two_trim`, một pipeline và invariant cuối;
- 34/35 ca có đủ năm phương án thành công.

Đây là nguồn hình học của [báo cáo PSTMO thống nhất](../docs/PSTMO.pdf) và
[báo cáo thuật toán toàn diện](../docs/BAO_CAO_TOAN_DIEN_PSTMO.html).

### Ma trận thực thi Gazebo 175 lượt

[`pstmo_execution_full_20260803/`](pstmo_execution_full_20260803/) chứa 7 môi
trường × 5 planner × 5 phương án, một lượt cho mỗi tổ hợp:

- `execution_175_cases.csv`: bảng scalar của 175 lượt;
- `execution_aggregate_5planners_7env.json`: protocol, audit, lỗi và so sánh
  ghép cặp;
- 170/175 lượt đạt, 174/175 dừng vật lý, không có can thiệp Collision Monitor
  và không có mẫu va chạm footprint trên đường đã lập;
- 34/35 nhóm có đủ năm phương án thành công và cùng Raw hash để so sánh ghép
  cặp.

Các JSON/log theo từng môi trường là dữ liệu gốc; không suy diễn thống kê lặp
từ ma trận chỉ có một lượt cho mỗi tổ hợp.

### Nghiên cứu năm tuyến kho giao cắt

Hồ sơ mới hơn theo phạm vi chuyên đề nằm ở
[`docs/warehouse_cross_aisles_5_routes/`](../docs/warehouse_cross_aisles_5_routes/README.md):
125 lượt trên 5 tuyến × 5 planner × 5 phương án, trong đó 123 lượt đạt. R01 dùng
dữ liệu tháng 8 trong `pstmo_execution_full_20260803/` và
`pstmo_execution_theta_star_20260802/`; R02–R05 được chạy tháng 10/2026 và lưu
cùng hồ sơ tài liệu.

### Kiểm chứng lặp PSTMO–Constrained cho bài journal

[`pstmo_journal_repeatability_20261011/`](pstmo_journal_repeatability_20261011/README.md)
chạy 5 tuyến × 5 planner × 2 phương án × 5 lần lặp = 250 lượt với cấu hình
source hiện hành. Mỗi job dùng stack Gazebo/Nav2 sạch; bốn worker dùng dải
`ROS_DOMAIN_ID` và `GZ_PARTITION` không giao nhau.

Đọc `study_manifest.json` trước, rồi `analysis/aggregate.json`. Phân tích ghép
PSTMO với Constrained theo scenario, planner, repetition và Raw-path hash; năm
lần lặp được lấy trung bình trong từng khối route–planner, nên đơn vị thống kê
là 25 khối. CI dùng bootstrap theo khối, Wilcoxon hai phía dùng cho chênh lệch
khối và Holm hiệu chỉnh ba outcome chính. Giao thức không phát lộ seed ngẫu
nhiên, do đó đây là fixed-input execution repeatability, không phải multi-seed
robustness.

Audit cuối có 250/250 trial và 125/125 cặp cùng Raw hash. Constrained đạt
122/125, PSTMO đạt 121/125; bảy thất bại thực thi được giữ nguyên. Không có
can thiệp Collision Monitor hoặc mẫu footprint va chạm trên đường đã lập.
Kết quả chính và các trade-off về sai số đích, khoảng hở, thời gian smoothing
được ghi đầy đủ trong README/aggregate của cohort.

`interrupted_attempts/` giữ nguyên các artifact mà node trial đã ghi nhưng
runner chưa kịp bổ sung repetition/configuration khi phiên điều phối bị ngắt.
Các attempt này có SHA-256 trong manifest, không được tính vào 250 trial chính
thức và không được analyzer đọc như kết quả hoàn chỉnh.
Một lỗi lifecycle hạ tầng đã chốt được lưu riêng trong
`infrastructure_attempts/` rồi retry bằng workflow chỉ chạy lại lỗi hạ tầng;
mọi thất bại thuật toán trong cùng job vẫn được tái sử dụng.

[`journal_validation_smoke_20261011/`](journal_validation_smoke_20261011/) là
pilot ba lượt Raw–Constrained–PSTMO dùng để kiểm tra pipeline trước cohort; không
gộp pilot vào thống kê chính.

## Snapshot PSTMO ngày 02/08/2026

Các thư mục dưới đây là những cấu hình khác nhau trong quá trình chọn pipeline;
giữ nguyên để tái lập ablation, không gọi chung là “PSTMO hiện tại”:

| Thư mục | Cấu hình/phạm vi |
| --- | --- |
| [`current_pstmo_reduced_20260802/`](current_pstmo_reduced_20260802/README.md) | Raw–Pivot-G2, 35 cặp hình học; phần vòng kín ban đầu có lỗi termination |
| [`current_pstmo_reduced_20260802_fixed/`](current_pstmo_reduced_20260802_fixed/README.md) | Sáu lượt Raw–Pivot-G2 chạy lại sau khi đổi goal checker |
| [`current_pstmo_nav2_smoother_comparison_20260802/`](current_pstmo_nav2_smoother_comparison_20260802/README.md) | Snapshot joint \((d,q)\), 175 bản ghi so với smoother Nav2 |
| [`current_pstmo_footprint_padding15_nav2_comparison_20260802/`](current_pstmo_footprint_padding15_nav2_comparison_20260802/README.md) | LOS với footprint padding 0,15 m |
| [`pstmo_greedy_los_single_pipeline_full_20260802/`](pstmo_greedy_los_single_pipeline_full_20260802/README.md) | Greedy LOS bắt buộc, padding 0 |
| [`pstmo_direct_dq_local08_adaptive_los_full_20260802/`](pstmo_direct_dq_local08_adaptive_los_full_20260802/README.md) | Chọn thích nghi giữa nhánh LOS và không LOS |
| [`pstmo_joint_dq_condition_only_ablation_20260802/`](pstmo_joint_dq_condition_only_ablation_20260802/README.md) | Ablation `condition_only` với joint \((d,q)\) |
| [`pstmo_hierarchical_alpha_two_trim_full_20260802/`](pstmo_hierarchical_alpha_two_trim_full_20260802/README.md) | Thử nghiệm tìm kiếm phân cấp hai trim; cơ chế này về sau trở thành mặc định của PSTMO độc lập |

Các thư mục `current_pstmo_los_*`, `current_pstmo_footprint_los_*`,
`pstmo_adaptive_los_*`, `pstmo_direct_dq_*`, `pstmo_joint_dq_*` và
`pstmo_*_smoke_*` không có README riêng là sweep/smoke trung gian của cùng chuỗi
thử nghiệm. Đọc JSON summary và config hash trước khi dùng.

## Adaptive Hybrid

[`neutral_hybrid_20260727/`](neutral_hybrid_20260727/README.md) là audit trước–
sau của selector đối xứng peak-cost/maneuver-effort. Nó gồm hai ma trận hình
học 320 hàng, kiểm chứng vòng kín và ảnh RViz2. Đây là bằng chứng riêng cho
logic Hybrid; không trộn với các benchmark PSTMO-only ở trên.

## Dataset hội nghị và audit controller tháng 7/2026

- `conference_geometry_20260725/`: 7 môi trường, 60 scenario, 5 planner,
  8 phương pháp, 3 repetition (7.200 dòng) của cấu hình tại thời điểm đó.
- `conference_execution_20260725/`: ma trận vòng kín phân tầng; không phải toàn
  bộ tích Descartes của map × planner × smoother × tốc độ.
- `closed_loop_audit_20260725/`: trace dùng để chẩn đoán hướng terminal,
  projection và sai lệch sau đường cong.
- `current_full_audit_20260726/`: bảy lượt kiểm tra controller sau hiệu chuẩn,
  cộng các mốc before/after; không ghép chúng thành một ma trận đầy đủ với dữ
  liệu ngày 25/07.
- `final_*_20260724/` và `terminal_*_20260724/`: các bước kiểm chứng controller,
  pivot và điều kiện kết thúc trong quá trình phát triển.

## Pilot và expected failure ngày 23/07/2026

Các file `fair_batch_*`, `planner_smoke_*`, `execution_matrix_*`,
`execution_trial_*` và `pivot_sweep_*` là pilot/tuning data. Một số mốc hữu ích:

- `fair_batch_v4b_hybrid_20260723.*`: batch offline 12 scenario × 6 phương án;
- `execution_matrix_v7_clean_repeated_20260723/`: ma trận vòng kín sạch sau khi
  sửa cleanup;
- `pivot_sweep_energy_20260723.*`: tuning data, không phải independent test set;
- `execution_spawn_guard_expected_failure_20260723.json` và
  `execution_matrix_timeout_guard_expected_failure_20260723/`: lỗi được tạo có
  chủ đích để kiểm tra guard/timeout;
- `execution_matrix_v5_repeated_20260723/`: không dùng thời gian vì còn một
  Gazebo server không thoát ở lượt 11.

## Quy tắc tạo kết quả mới

- Tạo thư mục mới theo ngày/experiment ID; không ghi đè snapshot cũ.
- Lưu commit hoặc hash source/config, scenario YAML, raw-path hash, seed,
  planner, method và repetition.
- Chỉ so sánh ghép cặp khi các phương án dùng cùng Raw path và cùng protocol.
- Tách lỗi hạ tầng khỏi thất bại thuật toán; giữ cả trial không đạt.
- Ghi rõ số lần lặp. Một lượt cho mỗi cấu hình chỉ hỗ trợ so sánh mô tả, không
  đủ để tuyên bố ý nghĩa thống kê.
