# Kết quả cuối PSTMO với LOS thích nghi xét footprint

LOS là tiền xử lý nội tại của PSTMO. Thuật toán đánh giá cả nhánh không LOS và nhánh LOS trên cùng đường Raw, sau đó chỉ chọn LOS khi điểm chất lượng toàn đường tốt hơn ít nhất 0,005.

## Thiết kế và kiểm định

- 7 môi trường, 7 tình huống, 5 global planner và 5 phương án;
- 35 nhóm ghép cặp, tổng cộng 175 bản ghi;
- 35/35 nhóm có cùng `raw_path_sha256` giữa các phương án;
- LOS được chọn 25/35 ca; không LOS được chọn 10/35 ca;
- PSTMO thành công 35/35, không fallback và không có mẫu va chạm footprint;
- cả hai nhánh nội bộ hợp lệ và an toàn trong 35/35 ca;
- d được tìm trực tiếp trong 0,02–0,8 m; 75/136 chuyển tiếp chọn q/d khác 0,35.

## LOS thích nghi so với luôn dùng nhánh không LOS

| Chỉ số | Không LOS | Lựa chọn thích nghi | Thay đổi |
|---|---:|---:|---:|
| Chiều dài | 10.3252 m | 10.2081 m | -1.13% |
| Độ cong cực đại | 3.1812 1/m | 2.1145 1/m | -33.53% |
| Năng lượng độ cong | 4.7302 1/m | 3.1647 1/m | -33.10% |
| Tổng góc quay tại chỗ | 0.0688 rad | 0.0393 rad | -42.94% |
| Chi phí lân cận cực đại | 223.5143 cost | 245.6571 cost | +9.91% |

Bốn chỉ số chuyển động đầu giảm; chi phí lân cận vật cản tăng là đánh đổi. LOS không bị ép dùng trong 10 ca mà điểm toàn đường không cải thiện đủ ngưỡng.

## So sánh hình học trên 34 nhóm cùng thành công

| Phương án | Thành công | Chiều dài (m) | Kmax (1/m) | Eκ (1/m) | Clearance (m) | Thời gian thuật toán (ms) |
|---|---:|---:|---:|---:|---:|---:|
| Raw | 35/35 | 10.314 | 11.187 | 50.207 | 0.161 | 3.5 |
| Simple | 34/35 | 10.240 | 4.763 | 9.418 | 0.191 | 1.5 |
| Savitzky–Golay | 35/35 | 10.296 | 6.934 | 22.699 | 0.191 | 0.0 |
| Constrained | 35/35 | 10.332 | 7.388 | 28.065 | 0.213 | 21.3 |
| PSTMO | 35/35 | 10.063 | 2.071 | 3.005 | 0.151 | 121.5 |

## Kiểm chứng vòng kín trên ba cặp

Raw và PSTMO đều hoàn thành 3/3; không có mẫu va chạm footprint trên đường kế hoạch và không có can thiệp của bộ giám sát va chạm.

| Chỉ số | Raw | PSTMO | Thay đổi |
|---|---:|---:|---:|
| Thời gian hoàn thành | 61.074 s | 56.902 s | -6.83% |
| Eκ thực thi | 3.517 1/m | 2.301 1/m | -34.59% |
| Chiều dài kế hoạch | 11.301 m | 11.123 m | -1.57% |
| Sai số bám cực đại | 7.474 cm | 7.413 cm | -0.82% |
| RMSE bám | 2.890 cm | 3.140 cm | +8.64% |
| Sai số đích | 3.519 cm | 4.339 cm | +23.29% |
| Clearance kế hoạch | 0.108 m | 0.122 m | +12.64% |

Kết luận: với tiêu chí hiện tại, LOS thích nghi tốt hơn không LOS về chiều dài, độ cong cực đại, năng lượng độ cong và lượng quay tại chỗ; đánh đổi là đi gần vùng chi phí cao hơn. Kết quả vòng kín có lợi về thời gian và năng lượng, nhưng RMSE bám và sai số đích tăng.
