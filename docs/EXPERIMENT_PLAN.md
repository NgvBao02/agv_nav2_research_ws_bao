# Kế hoạch code và thực nghiệm cho bài REV-ECIT 2026

Mốc chính thức: hạn nộp 30/09/2026, bài tiếng Việt theo IEEE A4, tối đa 6 trang.
Mục tiêu là có kết quả khóa trước 12/09 để dành ít nhất hai tuần cho thống kê,
viết và tái lập.

## Câu hỏi nghiên cứu

**RQ1.** Bộ chọn có cổng an toàn có giữ nguyên SimpleSmoother khi Pivot–G2 không
mang lại cải thiện clearance đủ lớn, và kích hoạt maneuver đúng ở tình huống khó
hay không?

**RQ2.** Khi kích hoạt Pivot–G2, mức tăng swept-footprint clearance có đạt được
dưới ngân sách độ cong/thời gian đã công bố hay không?

**RQ3.** Marker pivot và executor maneuver-aware có được thực hiện tin cậy trên
Gazebo và robot thật, xét theo ground-truth success và sai số bám hay không?

Claim dự kiến nên giới hạn ở: **bộ hậu xử lý hybrid có cổng an toàn cho robot vi
sai**, fallback về Simple, kiểm tra swept footprint và thực thi pivot tường minh.
Không claim planner mới, tối ưu toàn cục, hoặc nhanh/mượt hơn mọi baseline. Xem
evidence và go/no-go trong `RESEARCH_STATUS_20260723.md`.

## Kiến trúc code đích

```text
planner path + costmap snapshot + robot profile
                    |
                    v
          adaptive_pivot_g2 (C++ core)
       geometry / collision / timing / decision
          |                 |                 |
          v                 v                 v
 nav2 smoother wrapper  trajectory output  decision diagnostics
          |                 |
          +-------- common controller --------+
                            |
                       rosbag + metrics
```

### Packages

1. `adaptive_pivot_g2` — core C++ và unit/property tests (đã khởi tạo).
2. `adaptive_pivot_g2_nav2` — plugin `nav2_core::Smoother`, lifecycle và pluginlib
   (MVP đã load/chạy trong Smoother Server).
3. `adaptive_pivot_g2_benchmark` — gọi tuần tự cùng input cho mọi smoother và
   publish JSON/path (MVP đã chạy; CSV batch/rosbag metadata/hình còn thiếu).
4. Nếu cần, `adaptive_pivot_g2_msgs` — `CornerDecision` và summary; không tạo
   message riêng nếu JSON diagnostics đã đủ.

## Baseline công bằng

Giữ nguyên planner input cho mọi phương pháp trong một trial:

- `RAW`: path planner chung, không smoothing (nếu dùng LOS-pruning thì phải áp
  dụng một lần cho toàn bộ method);
- `NAV2_SIMPLE`: `nav2_smoother::SimpleSmoother`;
- `NAV2_SG`: `nav2_smoother::SavitzkyGolaySmoother`;
- `NAV2_CONSTRAINED`: `nav2_constrained_smoother::ConstrainedSmoother`;
- `PIVOT_G2`: adaptive pivot–G2 đã tuning;
- `PROPOSED`: safety-gated adaptive hybrid Simple/Pivot–G2.

Savitzky–Golay là baseline nhanh để xử lý nhiễu nhỏ, không phải đối thủ tối ưu cho
góc lớn. Constrained Smoother là baseline mạnh/cost-aware. Simple Smoother là
baseline gần nhất với refinement Laplacian. Không dùng `nav2_velocity_smoother`
làm baseline path vì nó lọc `cmd_vel`, không làm mượt path.

Để giải thích đóng góp, thêm ablation:

- pivot-only;
- G2-only;
- pivot/G2 với time gate nhưng không score;
- bản đầy đủ, không refinement (phương pháp chính giữ G2);
- tùy thời gian: Laplacian refinement như ablation, ghi rõ output không còn G2.

## Giao thức thí nghiệm

### Tầng A — Offline, bắt buộc

- Dùng đúng một `nav_msgs/Path` và một costmap snapshot cho tất cả method.
- Tối thiểu 6 họ tình huống: góc 30/60/90/120/150°, hành lang hẹp, góc sát vật
  cản, hai góc gần nhau, S-turn và đường LOS dài.
- Ít nhất 20 start–goal hợp lệ mỗi họ; lưu cố định thành dataset, không sinh lại
  sau khi đã xem test result.
- Tách 30% tuning và 70% test theo map/start–goal, không theo từng sample point.

Metrics:

- success/failure và lý do;
- collision-free swept footprint, minimum clearance;
- path length, Hausdorff deviation khỏi input;
- `max|κ|`, `∫κ²ds`, total variation của `κ`, `max|dκ/ds|`;
- predicted traversal time bằng **cùng một** time parameterizer;
- max `v`, `ω`, `a`, `α`, tốc độ hai bánh;
- số pivot/full stop, số điểm output và thời gian CPU.

### Tầng B — Closed-loop simulation, bắt buộc

- Một controller và cùng tham số cho tất cả method; không cho proposed controller
  riêng rồi so với baseline controller khác.
- Nếu intermediate pivot không thể được controller Nav2 chuẩn thực hiện, controller
  chung phải nhận diện pose trùng vị trí/orientation discontinuity cho mọi method.
- Chạy ít nhất 10 lần mỗi scenario nếu có noise/randomness; khóa seed và log seed.
- Log `/tf`, `/odom`, `/scan`, costmap, raw/smoothed path, `cmd_vel`, trajectory,
  diagnostics và collision/emergency events.

Metrics bổ sung:

- completion rate, task time;
- RMS/max cross-track error và heading error;
- collision/near-collision, emergency stop, replan count;
- total variation/jerk proxy của `v` và `ω`;
- năng lượng: ưu tiên Wh đo từ dòng/áp; nếu chưa có cảm biến, ghi rõ chỉ là proxy.

### Tầng C — Robot thật, rất nên có

- Chốt wheel separation, bán kính bánh, ticks/rev, giới hạn motor bằng đo thực.
- Ba layout: mở, góc 90° hẹp, hai góc liên tiếp; mỗi method ít nhất 10 lượt nếu
  thời gian cho phép.
- Dùng cùng pin state window, mặt sàn và thứ tự chạy randomized/blocking.
- Ground truth ưu tiên camera trên cao/AprilTag hoặc hệ đo ngoài; odometry của chính
  robot không đủ để kết luận sai số bám tuyệt đối.
- Có nút dừng khẩn cấp và vùng an toàn; test tốc độ thấp trước.

## Thống kê

- Báo median + IQR cho runtime/clearance nếu phân phối lệch; mean ± SD cho biến gần
  chuẩn sau khi kiểm tra.
- So sánh paired theo cùng scenario/start–goal. Dùng Friedman cho nhiều method,
  sau đó Wilcoxon signed-rank có hiệu chỉnh Holm; báo effect size, không chỉ p-value.
- Tỷ lệ success/collision: báo Wilson confidence interval và kiểm định phù hợp cho
  dữ liệu paired.
- Không loại trial thất bại khỏi bảng thời gian mà không báo; trình bày success và
  conditional time riêng.

## Lịch từ 22/07 đến 30/09/2026

| Thời gian | Deliverable / cổng qua |
|---|---|
| 22–27/07 | Chốt hồ sơ robot, lấy đủ source MATLAB, dataset map/start–goal v0 |
| 28/07–07/08 | Core geometry, common-window timing, collision checker, unit/property tests |
| 08–17/08 | Plugin Nav2 + diagnostics + runner cho 4 baseline; build CI sạch |
| 18–31/08 | Offline dataset khóa và closed-loop simulation; không đổi metric sau mốc này |
| 01–12/09 | Robot thật + ablation + sensitivity; đóng băng code và params |
| 13–20/09 | Thống kê, bảng/hình, viết bản IEEE 6 trang |
| 21–26/09 | Tái lập từ clone sạch, phản biện nội bộ, sửa claim |
| 27–29/09 | Kiểm định dạng, PDF, metadata và nộp sớm một ngày |

## Definition of done theo milestone

### M1 — Core

- Test G2 endpoint trái/phải và nhiều góc.
- Test không NaN/singularity, overlap, inner-wheel, limits.
- Time comparison dùng cùng entry/exit và cùng speed profile.
- Collision sampling hội tụ khi giảm spacing.

### M2 — Nav2

- Plugin load được trong Smoother Server Jazzy 1.3.12.
- `SmoothPath` trả path hợp lệ và collision check bật.
- Decision log gắn trial ID; không dùng global mutable state.
- Baselines chạy từ cùng launch/YAML.

### M3 — Benchmark

- Một lệnh tái tạo toàn bộ CSV và hình từ dataset khóa.
- Mỗi row có commit SHA, package versions, params hash, map/start/goal và seed.
- Failure là dữ liệu hạng nhất, có taxonomy.

### M4 — Paper

- Claim trong abstract trùng đúng test đã chạy.
- Có ít nhất một bảng offline, một bảng closed-loop, một hình qualitative và một
  ablation.
- Limitations nói rõ sampling, greedy corner selection và phạm vi robot vi sai.

## Trạng thái triển khai ngày 23/07/2026

MATLAB đã hoàn thành vai trò thử ý tưởng và chỉ còn được lưu trong
`matlab/pivot_g2`; không tiếp tục dùng làm pipeline thực nghiệm. Core C++, năm
plugin smoother (bốn method trực tiếp và Hybrid), Gazebo, RViz2, executor hiểu
pivot, batch 12 scenario và closed-loop matrix đều đã chạy. Runner đã kiểm tra
physical spawn, ground-truth goal và tự dừng Gazebo sau từng trial. Pilot hiện
không ủng hộ claim “Pivot–G2 mượt/nhanh nhất”; hướng bài đã chuyển sang hybrid có
cổng an toàn. Toàn bộ số liệu, giới hạn và kế hoạch test tiếp theo nằm trong
`RESEARCH_STATUS_20260723.md`; tuyệt đối không trộn với CSV cũ trong ZIP.
