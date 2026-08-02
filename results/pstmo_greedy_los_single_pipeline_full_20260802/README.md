# PSTMO với LOS tham lam swept-footprint — benchmark cuối

PSTMO độc lập chạy đúng một pipeline `condition_polyline → greedy LOS → joint (d,q) → stitch/final invariant`. Adaptive Hybrid không nằm trong benchmark hoặc báo cáo này.

## Thiết kế và kiểm định

- 7 môi trường, 7 tình huống đại diện, 5 global planner;
- 5 phương án Raw, Simple, Savitzky–Golay, Constrained và PSTMO;
- 35 nhóm ghép cặp, 175 bản ghi; mọi phương án trong từng nhóm dùng cùng `raw_path_sha256`;
- PSTMO, Raw, Savitzky–Golay và Constrained thành công 35/35; Simple thành công 34/35;
- 35/35 bản ghi PSTMO có `condition_then_los`, `pipeline_execution_count=1` và `final_invariants_verified=true`;
- PSTMO không có mẫu va chạm footprint; không có selector hai nhánh, padding hay fallback;
- ablation condition-only chạy độc lập, cũng một pipeline, và khớp 35/35 Raw hash với cấu hình LOS.

## LOS so với condition-only trên đúng 35 đường Raw

| Chỉ số | Condition-only | Condition + LOS | Thay đổi | Thấp/Bằng/Cao |
|---|---:|---:|---:|---:|
| Chiều dài | 10.325 m | 10.061 m | -2.56% | 35/0/0 |
| Kmax | 3.181 1/m | 1.342 1/m | -57.83% | 32/0/3 |
| Eκ | 4.730 1/m | 1.875 1/m | -60.36% | 33/0/2 |
| Tổng quay tại chỗ | 0.0688 rad | 0.0728 rad | +5.72% | 1/32/2 |
| Clearance nhỏ nhất | 0.162 m | 0.037 m | -77.39% | 34/1/0 |
| Thời gian thuật toán | 153.5 ms | 71.6 ms | -53.38% | 34/0/1 |
| Wall time | 166.6 ms | 84.3 ms | -49.42% | 34/0/1 |

LOS thử 387 shortcut, chấp nhận 82, loại 305; thời gian LOS trung bình 4.95 ms. Số điểm neo trung bình giảm từ 8.54 xuống 4.37.

LOS giảm chiều dài, Kmax, Eκ và thời gian tổng; đổi lại clearance giảm mạnh và tổng góc quay tại chỗ tăng nhẹ. Vì vậy LOS không Pareto-trội condition-only trên mọi chỉ số, dù cả hai cấu hình đều không có mẫu va chạm trong benchmark hình học.

## So sánh trên 34 nhóm mọi phương án đều thành công

| Phương án | L (m) | Kmax (1/m) | Eκ (1/m) | Clearance (m) | Thuật toán (ms) | Wall (ms) |
|---|---:|---:|---:|---:|---:|---:|
| Raw | 10.314 | 11.187 | 50.207 | 0.161 | 3.2 | 9.4 |
| Simple | 10.240 | 4.763 | 9.418 | 0.191 | 1.1 | 9.2 |
| Savitzky–Golay | 10.296 | 6.934 | 22.699 | 0.191 | 0.1 | 7.7 |
| Constrained | 10.332 | 7.388 | 28.065 | 0.213 | 21.9 | 30.9 |
| PSTMO | 9.925 | 1.269 | 1.712 | 0.037 | 69.9 | 82.6 |

Kết luận: cấu hình LOS bắt buộc là lựa chọn tốt hơn nếu ưu tiên đường ngắn, độ cong thấp và thời gian xử lý; nó không tốt hơn nếu ưu tiên clearance. Inflation layer chịu trách nhiệm cho dự phòng vận hành như thiết kế đã chốt, nhưng cần benchmark lặp, vòng kín và phần cứng trước khi khẳng định an toàn vận hành.
