# So sánh PSTMO có bật LOS xét footprint với các smoother Nav2

Bộ dữ liệu này là nguồn số liệu cho abstract tiếng Việt ICEEIS 2026. 
LOS là tiền xử lý nội tại của PSTMO, dùng footprint Nav2 cộng biên 0,15 m và được bật trong toàn bộ cấu hình thử nghiệm.

## Thiết kế và tính hợp lệ

- 7 môi trường, 7 tình huống đại diện, 5 global planner;
- 35 nhóm ghép cặp, 5 phương án, tổng cộng 175 bản ghi;
- 35/35 nhóm có cùng `raw_path_sha256` giữa các phương pháp;
- LOS được xác nhận bật trong 35/35 dòng PSTMO;
- LOS chấp nhận shortcut trong 30/35 dòng và giữ hành lang Raw trong 5/35 dòng không đủ biên;
- không có mẫu va chạm footprint trong đường PSTMO thành công.

## Kết quả hình học trên 34 nhóm cùng thành công

| Phương pháp | Thành công | Eκ tịnh tiến (1/m) | Chiều dài (m) | Clearance nhỏ nhất (m) |
|---|---:|---:|---:|---:|
| Raw | 35/35 | 50.207 | 10.314 | 0.161 |
| Simple | 34/35 | 9.418 | 10.240 | 0.191 |
| Savitzky–Golay | 35/35 | 22.699 | 10.296 | 0.191 |
| Constrained | 35/35 | 28.065 | 10.332 | 0.213 |
| PSTMO | 35/35 | 6.177 | 10.060 | 0.144 |

Trong 30 ca chấp nhận shortcut, LOS rút trung bình từ 247.6 xuống 4.6 pose. Năm ca fallback là quyết định fail-safe do đoạn tịnh tiến hoặc phép xoay không đạt footprint đã cộng biên; LOS không tạo shortcut trong các ca đó.

## Ablation PSTMO: LOS tắt so với LOS bật trên 35 cặp

| Chỉ số | LOS tắt | LOS bật | Thay đổi | Cặp cải thiện |
|---|---:|---:|---:|---:|
| Chiều dài | 10.335 m | 10.204 m | -1.27% | 27/35 |
| Độ cong cực đại | 3.197 1/m | 2.399 1/m | -24.97% | 25/35 |
| Eκ tịnh tiến | 4.595 1/m | 6.213 1/m | +35.22% | 25/35 |
| Clearance nhỏ nhất | 0.172 m | 0.141 m | -18.24% | 4/35 |
| RMSE lệch Raw | 0.029 m | 0.086 m | +195.32% | 1/35 |
| Thời gian thuật toán | 30.943 ms | 22.286 ms | -27.98% | 27/35 |

Dấu âm nghĩa là giá trị trung bình giảm. Với clearance, giảm là đánh đổi; với các chỉ số còn lại, giảm thường là cải thiện. Eκ giảm trong 25/35 cặp nhưng trung bình tăng do một ngoại lệ trả về đường Raw.

## Ablation closed-loop PSTMO trên ba cặp

Cả LOS tắt và LOS bật đều hoàn thành 3/3; không có can thiệp của bộ giám sát va chạm và không có mẫu va chạm footprint trên đường kế hoạch.

| Chỉ số | LOS tắt | LOS bật | Thay đổi |
|---|---:|---:|---:|
| Thời gian hoàn thành | 58.709 s | 57.380 s | -2.26% |
| Eκ thực thi | 2.902 1/m | 2.493 1/m | -14.09% |
| Chiều dài kế hoạch | 11.224 m | 11.132 m | -0.82% |
| Sai số bám cực đại | 7.759 cm | 7.263 cm | -6.39% |
| RMSE bám | 2.945 cm | 3.033 cm | +2.96% |
| Sai số đích | 4.508 cm | 3.985 cm | -11.61% |

Kết luận: LOS cải thiện có điều kiện, rõ nhất ở chiều dài, độ cong cực đại, thời gian xử lý và năng lượng độ cong thực thi; nó không tốt hơn toàn diện vì làm giảm clearance và tăng độ lệch khỏi hành lang Raw.
