# PSTMO hierarchical \(q/d\) và hai \(d\) — báo cáo nghiệm thu

Đây là kết quả thử nghiệm bộ sinh `hierarchical_alpha_two_trim`; số liệu lịch sử trong
`pstmo_greedy_los_single_pipeline_full_20260802` không bị sửa. Theo promotion gate đã
chốt trước benchmark, phương án này **không được chọn làm mặc định** và abstract không
được cập nhật vì tổng góc pivot tăng.

## Tập ghép cặp chính

- 7 môi trường × 1 tình huống đại diện × 5 planner = 35 đường Raw;
- 35/35 cặp có cùng `raw_path_sha256` với baseline;
- PSTMO mới thành công 35/35, `pipeline_execution_count=1` và
  `final_invariants_verified=true` 35/35;
- 0 mẫu va chạm footprint;
- 83 góc: 81 transition G2 và 2 pivot, so với 80 transition và 3 pivot của baseline.

| Chỉ số | Joint \((d,q)\) hiện tại | Hierarchical hai \(d\) | Thay đổi |
|---|---:|---:|---:|
| Chiều dài tịnh tiến | 10,0605 m | 10,0652 m | +0,047% |
| \(K_{\max}\) tịnh tiến | 1,3415 1/m | 1,1713 1/m | −12,69% |
| \(E_\kappa\) tịnh tiến | 1,8748 1/m | 1,6527 1/m | −11,85% |
| Số pivot | 3 | 2 | −1 |
| Tổng góc pivot trung bình | 0,07275 rad | 0,08174 rad | **+12,36%** |
| Clearance footprint nhỏ nhất | 0,03669 m | 0,03918 m | +6,78% |
| Thời gian thuật toán | 71,57 ms | 29,83 ms | −58,32% |
| Wall time | 84,27 ms | 50,35 ms | −40,26% |

Phương án đạt yêu cầu an toàn, chất lượng độ cong và thời gian nhưng không đạt điều
kiện “tổng góc pivot không tăng”. Hai pivot mới là các góc lớn 1,381 rad và 1,480 rad.
Tại các góc này, đoạn kề dài làm `d_compat == d_pref == 0,8 m`; toàn bộ coarse grid và
midpoint recovery đều không có transition an toàn. Vì thiết kế cấm tìm thêm \(d\),
thuật toán chỉ còn pivot. Đồng thời nó loại ba pivot nhỏ hơn của baseline, nên số pivot
giảm nhưng tổng góc vẫn tăng.

## Chi phí tìm kiếm

- 1.396 đánh giá hình dạng trên 83 góc, giảm 57,95% so với 3.320 đánh giá cũ;
- trung bình 16,82 đánh giá/góc; chỉ 3 góc cần midpoint recovery;
- \(\alpha=q/d\) được chọn: min 0,28, median 0,32, mean 0,3267, max 0,49;
- \(d\) được chọn: min 0,112 m, median 0,8 m, mean 0,700 m, max 0,8 m.

## Stress test bổ sung

Lệnh benchmark đã chạy toàn bộ tình huống trong bảy file cấu hình, tạo 300 ca PSTMO
thay vì chỉ 35 ca đại diện. Có 297 ca thành công và đều qua invariant với 0 mẫu va
chạm; một ca thất bại từ global planner và hai ca thất bại tại dịch vụ smoothing.
Các ca bổ sung này không được trộn vào so sánh ghép cặp 35 đường ở trên.

## Quyết định

- Giữ `legacy_joint_d_q` làm mặc định của PSTMO và Pivot nội bộ Hybrid.
- Giữ code và unit test của `hierarchical_alpha_two_trim` như một phương án thực
  nghiệm có thể tiếp tục nghiên cứu.
- Không thay số liệu hoặc nội dung abstract hiện tại.
