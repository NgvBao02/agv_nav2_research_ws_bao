# Phân loại kết quả nghiên cứu

## Audit selector Hybrid trung lập ngày 27/07/2026

- `neutral_hybrid_20260727/` chứa ma trận trước–sau 320 + 320 hàng trên
  narrow_aisles, closed-loop Gazebo ghép cùng raw path và ảnh RViz2 của selector
  mới. Cả hai ma trận hình học đạt 320/320; Adaptive Hybrid đổi từ
  Simple/Pivot = 38/2 sang 29/11 bằng luật cost/effort hai chiều.
- Đây là audit riêng cho selector hiện tại. Không trộn 40 hàng Adaptive Hybrid
  này vào ma trận hội nghị 7.200 hàng ngày 25/07 vì hai dataset dùng hai phiên
  bản gate khác nhau.
- Xem công thức, lệnh chạy, phân bố lý do chọn và cả metric xấu đi tại
  `neutral_hybrid_20260727/README.md`.

## Dataset hội nghị ngày 25/07/2026

- `conference_geometry_20260725/`: ma trận hình học đầy đủ gồm 7 môi trường,
  60 scenario, 5 planner, 8 phương pháp và 3 repetition (7.200 dòng). Mọi nhóm
  planner/scenario/repetition phải có đúng một `raw_path_sha256`.
- `conference_execution_20260725/conference_execution_compact.csv`: bảng vô
  hướng scalar của ma trận vòng kín phân tầng; high-rate ground-truth, odom,
  estimated pose, command và telemetry được giữ trong JSON gốc tại máy chạy
  thí nghiệm nhưng không lặp lại trong summary.
- `closed_loop_audit_20260725/`: các trace chẩn đoán dùng để tái hiện và sửa
  lỗi hướng terminal, projection nhảy nhánh và lệch sau đường cong.
- `tools/generate_full_algorithm_tutorial_report.py` tự tổng hợp trực tiếp bảy
  CSV trên; repo không giữ thêm snapshot JSON trùng lặp chỉ để sinh báo cáo.

Phần chạy kín là thiết kế phân tầng, không phải toàn bộ tích Descartes 7 map ×
5 planner × 8 smoother × 3 tốc độ. Báo cáo phải giữ rõ giới hạn này.

## Audit controller và động học ngày 26/07/2026

`current_full_audit_20260726/` là dữ liệu Gazebo ground truth của phiên bản
hiện tại sau khi neo start/goal liên tục, căn hướng theo path sau planning,
hiệu chuẩn wheel separation hiệu dụng, thêm bao phanh góc và sửa phân loại
jerk tại biên vận tốc bằng 0.

- Bảy file `*_final.json.gz` tương ứng bảy môi trường đều có
  `success=true`, `physically_settled=true`; jerk danh nghĩa cực đại không vượt
  0,90 m/s³.
- `right_rack_detour_pivot_g2_baseline.json.gz` và
  `lower_left_diagonal_pivot_g2_baseline.json.gz` là hai mốc trước sửa dùng cho
  bảng before/after.
- `narrow_aisles_pivot_g2_optimized.json.gz` là mốc ngay trước khi thêm bao phanh
  góc; nó được giữ để tái lập thời gian căn hướng 209→93 mẫu.
- Các file được nén gzip lossless; bộ tạo báo cáo đọc trực tiếp, không cần giải
  nén ra repo.
- Các trace tối ưu trung gian không được báo cáo sử dụng đã bị xóa, tránh trộn
  chúng với kết quả cuối.

Ma trận 42 trial ở ngày 25/07 dùng để so sánh tương đối các smoother trên cùng
phiên bản khi đó. Bảy trial ngày 26/07 xác nhận controller hiện tại trên từng
map; không được ghép hai tầng thành một ma trận đầy đủ sau hiệu chuẩn.

## Pilot ngày 23/07/2026

Thư mục này chứa pilot và các lượt chẩn đoán trong quá trình sửa pipeline. Chưa
file nào là dataset cuối để đưa nguyên trạng vào manuscript.

## Kết quả pilot có thể dùng để định hướng

- `fair_batch_v4b_hybrid_20260723.csv` và `_summary.json`: batch offline cuối,
  12 scenario × 6 phương pháp, cùng raw path trong từng scenario.
- `execution_matrix_v7_clean_repeated_20260723/lower_left_diagonal_summary.json`:
  ma trận closed-loop sạch, sáu phương pháp × ba lượt, cùng raw-path hash, success
  theo ground truth và không để lại Gazebo orphan.
- `pivot_sweep_energy_20260723.*`: sweep dùng để chọn cấu hình Pivot–G2 hiện tại;
  phải coi đây là tuning data, không phải independent test set.

## Chỉ dùng để chẩn đoán, không trích làm kết quả

- `execution_hybrid_live_lower_left_20260723.json`: action báo thành công nhưng
  robot được tái sử dụng với physical spawn sai; ground-truth final error lớn.
- `execution_spawn_guard_expected_failure_20260723.json`: expected failure dùng
  để xác nhận spawn guard bắt đúng lỗi trên.
- `execution_matrix_timeout_guard_expected_failure_20260723/`: expected failure
  ép timeout ở 10 s; runner trả code 124, ghi trial thất bại và dọn sạch process
  group Gazebo.
- `execution_matrix_v5_repeated_20260723/`: 18 lượt đều success và cùng raw hash,
  nhưng một Gazebo server ở lượt 11 không thoát; không dùng thời gian của batch
  này. Lỗi cleanup được sửa và kiểm tra lại ở `execution_matrix_v6_cleanup_smoke_20260723/`.
- `execution_trial_pivot_g2_20260723.json` và các matrix/batch phiên bản cũ:
  regression trung gian trước khi hoàn thiện ground-truth gate, cleanup Gazebo,
  full-footprint clearance hoặc Hybrid.

Khi benchmark chính thức bắt đầu, tạo thư mục theo commit/experiment ID mới, lưu
config hash, raw-path dataset, seed và environment metadata; không ghi đè các file
pilot này và không trộn các phiên bản vào cùng phép kiểm định.
