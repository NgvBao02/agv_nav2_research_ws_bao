# Phân loại kết quả ngày 23/07/2026

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
