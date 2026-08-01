# Kiểm chứng vòng kín PSTMO sau sửa goal termination

Dataset này thay thế **chỉ phần vòng kín** của
`current_pstmo_reduced_20260802`. Phần benchmark hình học 70 bản ghi không bị
ảnh hưởng bởi goal checker và vẫn được giữ nguyên ở thư mục cũ.

## Lỗi đã sửa

`StoppedGoalChecker` trước đây yêu cầu robot vừa nằm trong dung sai pose vừa có
vận tốc góc dưới 0,02 rad/s. Trong khi đó, RPP vẫn phát lệnh quay để hiệu chỉnh
hướng cuối. Với `stateful: false`, nhiễu AMCL còn có thể đưa XY ra ngoài dung
sai sau khi robot đã tới đích, tạo vòng lặp đổi chiều quay kéo dài.

Cấu hình mới dùng `SimpleGoalChecker` với `stateful: true`, dung sai XY 0,06 m
và dung sai yaw 0,10 rad. Sau khi action kết thúc, runner vẫn độc lập:

- chờ ba mẫu ground-truth liên tiếp có |v| ≤ 0,01 m/s và |ω| ≤ 0,02 rad/s;
- kiểm tra sai số đích cuối bằng Gazebo ground truth;
- loại lượt chạy nếu không dừng vật lý hoặc không đạt đích.

## Thiết kế chạy lại

- 3 tình huống, 3 môi trường;
- Theta* cho mọi cặp;
- Raw và PSTMO (`pivot_g2`), một lượt cho mỗi phương pháp;
- tổng cộng 3 cặp/6 lượt;
- không chạy `adaptive_hybrid`.

## Kết quả

| Tình huống | Raw (s) | PSTMO (s) | Thay đổi |
|---|---:|---:|---:|
| `lower_left_diagonal` | 34,23 | 33,45 | −2,28% |
| `southwest_northeast_weave` | 68,61 | 66,71 | −2,78% |
| `full_replenishment` | 81,58 | 76,91 | −5,72% |
| **Trung bình** | **61,47** | **59,02** | **−3,99%** |

RMSE bám đường trung bình theo ground truth giảm từ 2,85 cm xuống 2,81 cm
(−1,34%). Cả 6/6 lượt đều thành công, dừng vật lý, đạt đích ground truth, không
có can thiệp của collision monitor và không có mẫu va chạm footprint trên
đường lập kế hoạch. Ba cặp đều có cùng `raw_path_sha256` trong nội bộ cặp.

Đây vẫn là kiểm chứng rút gọn với một lượt cho mỗi cấu hình; số liệu không đủ
cho kiểm định thống kê.

## Tệp chính

- `closed_loop_summary.json`: tổng hợp gọn ba cặp sau sửa;
- `closed_loop/*/*_summary.json`: summary gốc do execution matrix sinh;
- `closed_loop/**/*.json.gz`: trace đầy đủ;
- `closed_loop/**/*.log.gz`: log từng lượt chạy.
