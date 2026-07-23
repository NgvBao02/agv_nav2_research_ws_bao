# Kiểm toán kỹ thuật thuật toán Pivot–G2 thích nghi

Ngày kiểm toán: 22/07/2026  
Phạm vi: báo cáo Word 16 trang ngày 17/07/2026, repository
`conmeodit/robot_ros2_ws`, ROS 2 Jazzy/Nav2 1.3.12 đang cài trên máy.

## Kết luận ngắn

Ý tưởng có thể phát triển thành bài báo, nhưng phiên bản mô tả ban đầu chưa đủ để
đưa ra kết luận khoa học. Pilot ngày 23/07 còn cho thấy Pivot–G2 không mượt hoặc
nhanh hơn SimpleSmoother một cách tổng quát. Vì vậy hướng chính đã được thu hẹp
thành bộ Hybrid có cổng an toàn: giữ Simple khi lợi ích an toàn không đủ và chỉ
chọn Pivot–G2 dưới một energy budget. Kết quả và claim cập nhật nằm trong
`RESEARCH_STATUS_20260723.md`.

Điểm mạnh là bài toán được đặt đúng ở tầng hậu xử lý, có lựa chọn pivot/G2 giải
thích được và đã nghĩ đến footprint, clearance cùng giới hạn hai bánh. Bốn blocker
kỹ thuật ban đầu là:

1. thời gian pivot và G2 chưa được tính trên cùng một mô hình;
2. tham số mô hình không khớp robot thật;
3. đường sau refinement chưa còn được bảo đảm G2;
4. output đầy đủ không khớp trực tiếp giao diện Smoother Server của Nav2.

## Những phần đang đúng

- Công thức góc có dấu `atan2(cross(e_in,e_out), dot(e_in,e_out))` là đúng.
- Với cách đặt `P1=P0+q e_in`, `P2=P0+2q e_in` (và đối xứng ở đầu
  ra), đạo hàm bậc hai của Bézier tại hai đầu bằng 0. Khi đạo hàm bậc nhất
  khác 0, độ cong tại hai đầu bằng 0; ghép với đoạn thẳng đạt G2 hình học.
- Quan hệ robot vi sai `v_l=v-Lω/2`, `v_r=v+Lω/2` và giới hạn tốc độ theo
  độ cong trong tài liệu là nhất quán.
- Điều kiện không cho bánh trong quay ngược tương đương với
  `|κ| <= 2/L` khi robot chạy tiến; đây là một lựa chọn thiết kế hợp lệ.
- Kiểm tra swept footprint của pivot là cần thiết vì một footprint chữ nhật có
  thể va chạm trong lúc quay dù hai pose đầu/cuối đều an toàn.
- Tách planner, path postprocessor, speed profile và controller là cách định vị
  đóng góp phù hợp.

Không nên đổi cách gọi G2 thành C2. Tài liệu mới chỉ chứng minh liên tục hình học
về vị trí, tiếp tuyến và độ cong; C2 còn phụ thuộc tham số hóa hai đoạn ghép.

## Các vấn đề phải sửa trước benchmark

### P0 — Blocker

#### 1. So sánh thời gian chưa công bằng

`T_pivot` có giảm tốc–quay–tăng tốc, còn `T_curve` chủ yếu tích phân
`ds/v_limit`; profile gia tốc được sinh sau quyết định. Vì vậy cổng `0,15 s`
có thể chọn sai maneuver.

Cách sửa đã bắt đầu trong code mới:

- so sánh trên cùng cửa sổ từ entry đến exit;
- pivot gồm entry→đỉnh, dừng, quay, đỉnh→exit;
- G2 đi từ chính entry đến chính exit;
- hai phương án dùng cùng giới hạn `v`, `ω`, `a`, giảm tốc, `α` và tốc độ bánh;
- dùng forward/backward speed pass trước khi so sánh;
- giới hạn `α` từ sai phân `ω=vκ`, không chỉ giới hạn `v` theo `κ`.

Khi xếp hạng các bán kính có entry/exit khác nhau, phải đặt chúng trong một cửa
sổ so sánh chung hoặc cộng thời gian các đoạn thẳng bù. Không được so trực tiếp
thời gian của hai đường có đầu/cuối khác nhau.

#### 2. Mô hình trong báo cáo không phải robot năm trước

| Đại lượng | Báo cáo Pivot–G2 | Hệ năm trước | Mô hình hai bánh hiện tại |
|---|---:|---:|---:|
| Footprint | 0,40 × 0,30 m | thân 0,35 × 0,42 m + padding | 0,44 × 0,34 m |
| Khoảng hai bánh `L` | 0,24 m | 0,42 m | 0,2548 m từ vệt lăn CAD |
| `v_max` | 0,35 m/s | 0,08–0,12 m/s | 0,08 m/s khi smoke test |
| `ω_max` | 1,20 rad/s | 0,425–0,80 rad/s | 0,425 rad/s khi smoke test |

Sai `L` làm sai đồng thời bán kính tối thiểu không đảo bánh, `ω`, tốc độ hai
bánh và thời gian. Trước thí nghiệm phải đo/chốt một hồ sơ robot duy nhất; không
lấy giá trị mô phỏng để kết luận cho robot thật.

#### 3. Laplacian refinement có thể phá bảo đảm G2

Bézier ban đầu đạt `κ(0)=κ(1)=0`, nhưng refinement đang di chuyển các mẫu không
thuộc pivot rồi tính lại heading/curvature. Nếu không khóa entry/exit và các điều
kiện đạo hàm, reference cuối không còn là Bézier đã chứng minh G2.

Chọn một trong hai hướng:

- bỏ refinement khỏi phương pháp đề xuất trong thí nghiệm chính; hoặc
- chỉ tối ưu control point/nội điểm dưới ràng buộc endpoint, tangent và curvature,
  sau đó kiểm tra lại G2 bằng tolerance số.

#### 4. Smoother plugin không mang đủ output

Trên Nav2 Jazzy 1.3.12, `nav2_core::Smoother::smooth()` chỉ biến đổi
`nav_msgs/Path`. `SmoothPath` không trả `v`, `ω`, mode, thời gian hay decision
log. Pose trùng vị trí nhưng đổi orientation cũng chưa chắc buộc controller chuẩn
thực hiện pivot.

Kiến trúc cần tách:

- core hình học/động học độc lập;
- wrapper `nav2_core::Smoother` cho path và orientation;
- diagnostic topic cho quyết định;
- cùng một controller/executor hiểu pivot cho tất cả phương pháp end-to-end;
- `nav_msgs/Trajectory` dùng để ghi reference có velocity/acceleration khi cần.

### P1 — Rủi ro làm sai kết luận

#### Lấy mẫu an toàn rời rạc

`validationStride=2` có thể bỏ sót va chạm. Bước lấy mẫu phải phụ thuộc độ phân
giải costmap và bán kính quét footprint, ví dụ thỏa cả
`Δs <= resolution/2` và `r_footprint Δθ <= resolution/2`. Cần validation cuối
ở mọi mẫu, không dùng stride, và test hội tụ khi giảm bước lấy mẫu.

#### Chưa rõ dùng costmap inflate hay obstacle map

LOS theo centerline có thể dùng lớp đã inflate. Footprint collision/clearance phải
nói rõ đang đọc lethal obstacle hay toàn bộ inflation cost; nếu vừa inflate đủ bán
kính robot vừa quét full footprint sẽ bị bảo thủ hai lần. Benchmark mọi phương pháp
phải dùng cùng một costmap snapshot và cùng collision checker.

#### `R_d` không phải bán kính thật

Điều này đã được ghi đúng trong báo cáo, nhưng tập cố định 0,20–0,60 m chưa được
chuẩn hóa theo robot/corner. Nên log `max|κ|`, `min 1/|κ|`, trim distance và
thử tập ứng viên theo tỷ lệ chiều dài đoạn hoặc tìm kiếm trên trim distance.

Hình 3 đang ghi các ví dụ 0,45/0,75/1,05 m trong khi bảng và mã giả ghi
0,20–0,60 m. Phải ghi rõ “minh họa, không phải tập tham số thực nghiệm” hoặc vẽ
lại đúng tập dùng trong code.

#### Giới hạn gia tốc góc cần viết đúng theo hình học

Với `ω=vκ`, ta có `α = κ a_t + v² dκ/ds`. Chỉ giới hạn `v` bằng `ω_max/|κ|`
không bảo đảm `α_max`, nhất là gần điểm ghép nơi `κ=0` nhưng `dκ/ds` có thể lớn.
Code mới đã thêm kiểm tra sai phân `Δω/Δt`; bản production nên dùng time scaling
ổn định hơn và kiểm tra hội tụ theo bước lấy mẫu.

#### Score phụ thuộc tập ứng viên cục bộ

Min–max normalization tại từng góc làm cùng một ứng viên nhận score khác khi chỉ
thêm/bớt một ứng viên khác. Cần:

- định nghĩa `|ω|` là max, RMS hay integral;
- cố định chuẩn hóa bằng các giới hạn vật lý hoặc tập training;
- khóa trọng số trước test set;
- báo ablation và sensitivity, không chọn trọng số bằng kết quả test.

#### Quyết định từng góc là greedy

Gia tốc và tốc độ ở hai góc gần nhau có liên hệ; chọn tốt cục bộ có thể không tốt
cho toàn path. Bài đầu có thể giữ greedy nhưng phải nêu giới hạn và luôn đánh giá
profile sau khi ghép toàn đường. Nếu sai khác lớn, dùng beam search/dynamic
programming trên chuỗi maneuver ở phiên bản sau.

### P2 — Chất lượng nghiên cứu và tái lập

- Báo cáo 16 trang hiện là đặc tả kỹ thuật, chưa phải bài REV-ECIT. Hội nghị yêu
  cầu bài tiếng Việt, IEEE A4, tối đa 6 trang.
- Source MATLAB đã được cung cấp ngày 23/07/2026 và đưa vào
  `matlab/pivot_g2`. Kiểm toán xác nhận time gate bất đối xứng, refinement phá
  G2 và dữ liệu CSV cũ được tạo trước source G2 hiện tại. Xem
  `docs/MATLAB_SOURCE_AUDIT.md`.
- Repository năm trước là ROS 2 tùy biến, chưa phải Nav2: A* + LOS + lookahead
  controller nằm trong một node khoảng 1.900 dòng.
- Bảy test hiện có của repo cũ chỉ kiểm tra vision/tracking, chưa kiểm tra A*,
  smoothing, collision, odometry hay controller.
- Cần ghi commit SHA, ROS distro/Nav2 version, seed, map, start–goal, YAML và rosbag
  cho mỗi lượt chạy.

## Phần đã sửa trong workspace mới

Package `adaptive_pivot_g2` hiện có:

- sinh Bézier bậc năm và đạo hàm giải tích;
- kiểm tra dấu độ cong, singular derivative, overlap và đảo bánh trong;
- giới hạn `v`, `ω`, tốc độ bánh theo `L=0,2548 m` mặc định của mô hình hai bánh;
- forward/backward time parameterization với giới hạn gia tốc tuyến tính;
- lặp giảm tốc để thỏa giới hạn gia tốc góc sai phân;
- thời gian pivot trên cùng cửa sổ entry–exit, gồm cả hai đoạn tịnh tiến;
- unit test cho G2 endpoint, trái/phải, overlap, đảo bánh, profile thời gian và `α`.

Wrapper Nav2 hiện đã tích hợp costmap, swept-footprint, lựa chọn đa ứng viên và
diagnostics. Plugin Hybrid trực tiếp compose Simple/Pivot–G2 và dùng một hàm chọn
thuần đã có unit test. Benchmark chạy chung một raw path qua raw + bốn smoother
và shared maneuver-aware RPP. Runner closed-loop còn xác nhận spawn vật lý và
đích bằng ground truth, thay vì tin riêng action status. Đây là mức regression kỹ
thuật đã kiểm chứng, chưa phải benchmark khoa học hoàn thành vì còn thiếu dataset
đa map đã khóa, lặp nhiều lần, robot thật, metadata/rosbag và kiểm định thống kê.

Phía MATLAB hiện cũng đã có common-window time model, forward/backward time
parameterization, giới hạn gia tốc góc, swept-footprint SE(2), adaptive curve
sampling và regression script. Chưa thể tuyên bố test MATLAB pass vì máy này
không cài MATLAB/Octave.
