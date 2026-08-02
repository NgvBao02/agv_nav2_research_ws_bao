# Ablation PSTMO `condition_only`

Tập dữ liệu này chỉ dùng để cô lập ảnh hưởng của LOS. Constructor mặc định của
PSTMO được build tạm ở chế độ nội bộ `condition_only`; sau khi benchmark xong,
source và bản cài đặt đã được khôi phục về `condition_then_los`. Không có tham
số ROS hoặc plugin công khai nào được thêm cho ablation này.

## Thiết kế

- 7 tình huống đại diện × 5 global planner;
- mỗi nhóm gồm Raw và PSTMO condition-only, tổng cộng 70 bản ghi;
- 35/35 PSTMO thành công, mỗi bản ghi có `pipeline_execution_count=1` và
  `final_invariants_verified=true`;
- 35/35 hash Raw khớp với tập LOS cuối;
- không có mẫu va chạm footprint.

Kết quả ghép cặp đầy đủ nằm trong
`../pstmo_greedy_los_single_pipeline_full_20260802/aggregate_summary.json` và
`README.md` của cùng thư mục đó.
