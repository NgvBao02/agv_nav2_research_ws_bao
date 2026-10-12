# Kiểm chứng lặp PSTMO–Constrained cho bài journal

Cohort này bổ sung bằng chứng lặp trong điều kiện cố định cho báo cáo năm tuyến
kho giao cắt. Dữ liệu được chạy trên commit
`1a1164e878bbe98e98cebefc009c4fe129174e9c`; không gộp pilot và không ghi đè
ma trận lịch sử.

## Thiết kế

- 5 route × 5 planner × 2 phương án × 5 repetition = 250 trial.
- Phương án: Constrained Smoother và PSTMO.
- Planner: NavFnAStar, NavFnDijkstra, ThetaStar, Smac2D, SmacHybrid.
- Mỗi trial dùng stack Gazebo/Nav2 sạch. Bốn worker dùng dải
  `ROS_DOMAIN_ID` và `GZ_PARTITION` không giao nhau.
- Pipeline không phát lộ seed ngẫu nhiên (`random_seed: null`); năm repetition
  đo execution repeatability cho đầu vào cố định, không phải multi-seed.
- Fingerprint cấu hình kết hợp:
  `23ec3a4d3aac8e2af3f4c0d59971557b802e77b51d96efb1a81b6a6fae074bba`.

## Audit và kết quả hoàn tất

- 250/250 trial có đủ metadata; 125/125 cặp PSTMO–Constrained có cùng Raw-path
  SHA-256.
- Constrained đạt 122/125; PSTMO đạt 121/125; tổng cộng 243/250 lượt đạt.
- Discordance thành công: 4 cặp chỉ Constrained đạt và 3 cặp chỉ PSTMO đạt;
  McNemar exact hai phía `p = 1.0`. Kết quả này không chứng minh tương đương
  hoặc non-inferiority.
- 125/125 diagnostics PSTMO xác nhận `condition_only`,
  `hierarchical_alpha_two_trim`, một pipeline và invariant cuối.
- Tổng cộng 2.005 trạng thái DP và 3.165 cạnh tương thích được ghi nhận.
- Không có can thiệp Collision Monitor và không có mẫu footprint va chạm trên
  đường đã lập trong cohort này.
- 14 trial cần ít nhất một retry hạ tầng. Retry hạ tầng không thay thế một
  thất bại thuật toán.

### Bảy lượt không đạt được giữ nguyên

| Route | Planner | Phương án | Lặp | Trạng thái chính | Sai số đích (m) |
| --- | --- | --- | ---: | --- | ---: |
| cross_to_picking_turn | NavFnAStar | PSTMO | 4 | action abort, code 102; đã dừng | 1.580262 |
| cross_to_picking_turn | NavFnDijkstra | PSTMO | 4 | action abort, code 102; đã dừng | 0.685907 |
| cross_to_picking_turn | Smac2D | Constrained | 3 | action abort, code 102; chưa dừng | 3.465366 |
| cross_to_picking_turn | Smac2D | PSTMO | 1 | action abort, code 102; chưa dừng | 1.148750 |
| cross_to_picking_turn | SmacHybrid | Constrained | 4 | action abort, code 102; chưa dừng | 3.655099 |
| reverse_diagonal_transfer | SmacHybrid | PSTMO | 4 | action thành công; vượt dung sai đích | 0.101583 |
| reverse_diagonal_transfer | ThetaStar | Constrained | 4 | action thành công; vượt dung sai đích | 0.100358 |

Ngưỡng vị trí là 0,100000 m. Các lượt action abort và hai lượt vượt ngưỡng là
kết quả thực thi, không phải lỗi khởi tạo, nên không được chạy lại.

## Phân tích theo khối

Năm repetition được lấy trung bình trong từng route–planner cell; 25 cell là
đơn vị phân tích. CI 95% dùng bootstrap theo khối 20.000 lần, kiểm định
Wilcoxon hai phía dùng chênh lệch khối và Holm hiệu chỉnh ba outcome chính.
Chỉ các cặp mà cả hai phương án đạt mới đóng góp metric liên tục (118 cặp).

Chênh lệch dưới đây là PSTMO − Constrained:

| Outcome chính | Constrained | PSTMO | Δ khối trung bình | CI 95% | p Holm |
| --- | ---: | ---: | ---: | ---: | ---: |
| Execution time (s) | 47.56025 | 44.65388 | -2.85727 | [-3.71649; -2.03914] | 1.1921e-6 |
| Tracking RMSE (m) | 0.025683 | 0.022786 | -0.002985 | [-0.004487; -0.001506] | 4.8947e-4 |
| Executed curvature energy (1/m) | 3.27441 | 2.22057 | -1.03773 | [-1.33167; -0.76741] | 5.3644e-7 |

Kết quả phụ cho thấy trade-off phải giữ trong manuscript:

- planned curvature energy giảm trung bình theo khối 23,20237 1/m;
- jerk thực p95 giảm 0,07795 m/s³ và quãng đường giảm 0,08904 m;
- sai số vị trí cuối tăng 0,00338 m;
- khoảng hở footprint nhỏ nhất giảm 0,03516 m;
- smoothing time tăng 0,07122 s; chỉ số này là phụ vì bốn worker tạo tải CPU
  đồng thời.

Không dùng kết quả phụ như claim confirmatory chỉ vì p-value nhỏ. Xem số đầy
đủ, CI và sign count trong [`analysis/aggregate.json`](analysis/aggregate.json).

## Gián đoạn và recovery hạ tầng

Phiên điều phối đầu bị ngắt sau 13 job. Bốn JSON mà node trial đã ghi nhưng
runner chưa kịp bổ sung repetition/configuration được lưu nguyên trạng trong
`interrupted_attempts/`, kèm SHA-256 ở `study_manifest.json`; chúng bị loại
khỏi 250 trial chính thức.

Một trial PSTMO/ThetaStar gặp lỗi
`navigation lifecycle status service is unavailable`. Attempt này được lưu ở
`thetastar/cross_to_picking_turn/infrastructure_attempts/` rồi retry đúng một
trial bằng `--retry-infrastructure-only`. Chín trial đã chốt trong cùng job,
kể cả thất bại thuật toán nếu có, được tái sử dụng. Trial recovery thành công
và JSON cuối ghi tổng số attempt hạ tầng.

## Tái lập

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/run_journal_repeatability_study.py \
  --output-root results/pstmo_journal_repeatability_<YYYYMMDD> \
  --repetitions 5 --workers 4
PYTHONNOUSERSITE=1 python3 tools/analyze_journal_repeatability.py \
  --cohort results/pstmo_journal_repeatability_<YYYYMMDD>
```

Không chạy lệnh trên vào thư mục cohort này. Dùng tên ngày/experiment mới.
Nếu điều phối bị ngắt, xem `docs/agent/COMMANDS.md` trước khi dùng `--resume`;
không tự động chạy lại một trial thuật toán không thuận lợi.

## Phạm vi diễn giải

Cohort chỉ dùng một map tĩnh, một robot mô phỏng và một controller. Chưa có
random-seed/noise injection, vật cản động, thay đổi tải, phép đo Wh hoặc robot
thật. Cũng chưa có ablation nhân quả trên cùng commit cho DP, swept footprint
và two-trim. Các thiếu sót này phải được nêu trong Abstract, Threats to
Validity và Conclusion nếu bài được viết từ cohort này.
