# Trạng thái nghiên cứu và hướng bài báo ngày 23/07/2026

## Kết luận khoa học hiện tại

Hướng nghiên cứu **có thể tiếp tục**, nhưng claim ban đầu “Pivot–G2 làm đường đi
mượt hơn hoặc nhanh hơn mọi smoother Nav2” không được dữ liệu hiện tại ủng hộ.
Trong 12 tình huống chung một raw path, `SimpleSmoother` có năng lượng độ cong
tịnh tiến thấp nhất; ở lượt closed-loop đã kiểm tra, nó cũng hoàn thành nhanh nhất.
Nếu giữ claim cũ, bài rất dễ bị phản biện vì chọn baseline hoặc diễn giải kết quả
không công bằng.

Hướng phù hợp với bằng chứng hơn là:

> **Bộ hậu xử lý thích nghi có cổng an toàn cho robot vi sai, chỉ kích hoạt
> maneuver Pivot–G2 khi thu được cải thiện clearance đủ lớn dưới một ngân sách
> độ cong; nếu không thì giữ đường SimpleSmoother.**

Tên làm việc: **Safety-Gated Adaptive Hybrid Pivot–G2**. Đóng góp không phải một
global planner mới, mà là (1) bộ chọn có fallback rõ ràng, (2) kiểm tra swept
footprint có xét hướng của thân xe 0,44 × 0,34 m, và (3) thực thi pose pivot tường
minh trên robot hai bánh vi sai.

Đây là hướng đủ hợp lý để xây dựng bài hội nghị, nhưng **chưa thể bảo đảm được
chấp nhận**. Hiện đã có pilot hình học trên ba map mới và smoke test vòng kín
năm planner, nhưng benchmark smoother nhiều lần lặp vẫn tập trung ở một scenario;
cần khóa dataset, tăng số lần lặp và chạy robot thật trước khi viết claim cuối.

## Hệ thống đã chốt để thử nghiệm

- ROS 2 Jazzy, Nav2 và Gazebo Harmonic; toàn bộ pipeline chính không dùng MATLAB.
- Robot vi sai: thân 0,44 × 0,34 m, bán kính bánh 0,0425 m, khoảng vệt lăn CAD
  0,2548 m; hệ số separation hiệu dụng trong mô phỏng được calibration là
  0,2809 m.
- Encoder GA25: 1320 tick/vòng; lidar A1M8; IMU BNO055.
- Baseline: raw path, Nav2 Simple, Savitzky–Golay và Constrained Smoother.
- Phương pháp: Pivot–G2 đã tuning và bộ Hybrid tự chọn giữa Simple/Pivot–G2.
- Global planner pilot: NavFn A*, NavFn Dijkstra, Theta*, Smac 2D và Smac
  Hybrid; planner và smoother được phân tích thành hai yếu tố riêng.
- Cùng planner input, map, footprint, collision checker và controller cho tất cả
  phương pháp. Controller chung hiểu marker pivot nên phương pháp đề xuất không
  được hưởng một controller riêng thiếu công bằng.
- Success closed-loop phải thỏa cả kết quả action của Nav2 và ground-truth Gazebo
  trong 0,10 m, 0,15 rad so với goal.

## Bổ sung multi-planner và multi-map

Ba môi trường `open_arena`, `narrow_aisles` và `office_maze` đã có world SDF,
map PGM/YAML và 8 scenario/map được sinh từ cùng một source hình học. Open arena
và narrow aisles đạt 240/240 phép planner–smoother; office maze đạt 228/240 và
giữ nguyên các failure có ý nghĩa ở cửa hẹp. Cả năm planner cũng đạt ground
truth goal trong smoke test vòng kín `short_open_diagonal`.

Ba môi trường kho chuyên biệt `warehouse_long_aisles`,
`warehouse_cross_aisles` và `warehouse_dispatch` bổ sung thêm 24 scenario. Cả
ba đạt 240/240 phép planner–smoother. Một lượt Theta* + Adaptive Hybrid trên mỗi
map cũng đạt đích theo ground truth, không có can thiệp từ collision monitor.
Chi tiết nằm trong `WAREHOUSE_MAPS.md`.

Office maze dẫn tới hai sửa lỗi:

1. Constrained Smoother không còn downsample factor 2 vì đoạn NavFn A–B–A có
   thể bị alias thành A–A và tạo residual NaN trong Ceres.
2. Hybrid thêm raw-path safety fallback. Nếu Simple và Pivot–G2 đều không tạo
   được đường an toàn nhưng raw path vẫn qua swept-footprint check, Hybrid trả
   raw và ghi chẩn đoán thay vì abort.

Số liệu, hình và lệnh tái lập nằm trong `PLANNER_MAP_BENCHMARK.md`. Đây vẫn là
pilot `n=1` cho mỗi scenario, không thay cho test set khóa.

## Kết quả offline trên 12 tình huống

File gốc: `results/fair_batch_v4b_hybrid_20260723.csv` và
`results/fair_batch_v4b_hybrid_20260723_summary.json`. Mỗi tình huống dùng
**chính xác cùng một raw path** cho sáu phương pháp trong lượt đó. Clearance là
khoảng cách của swept footprint hình chữ nhật tới vật cản, không phải khoảng cách
của tâm robot.

| Phương pháp | Trung bình `∫κ²ds` tịnh tiến | Clearance min TB (m) | Clearance p05 TB (m) | Tổng marker pivot |
|---|---:|---:|---:|---:|
| Raw | 13,6394 | 0,2056 | 0,2682 | 0 |
| Nav2 Simple | **1,7798** | 0,2507 | 0,2774 | 0 |
| Nav2 Savitzky–Golay | 3,6514 | **0,2598** | 0,2908 | 0 |
| Nav2 Constrained | 2,6459 | 0,2575 | **0,2915** | 0 |
| Pivot–G2 đã tuning | 3,1306 | 0,2518 | 0,2858 | 14 |
| Adaptive Hybrid | 2,0718 | 0,2528 | 0,2817 | 4 |

Hybrid giữ Simple ở 9/12 tình huống và chọn Pivot–G2 ở ba tình huống:
`horizontal_rack_detour`, `right_rack_detour` và `lower_left_diagonal`. Hash
output của Hybrid trùng bit-for-bit với phương án mà diagnostics báo đã chọn.
Điều này xác nhận bộ chọn đang fallback đúng, chứ không âm thầm biến đổi thêm
đường baseline. Một path có tie-break khác khi planner được khởi động lại ở một
batch cũ, nên benchmark chính phải lưu/freeze raw path thay vì chỉ dựa vào tính
xác định qua nhiều lần restart.

Kết quả này cũng bác bỏ một diễn giải quá mạnh: Pivot–G2 không phải smoother có
độ cong thấp nhất. Giá trị của Hybrid nằm ở việc tránh kích hoạt pivot khi lợi ích
an toàn không đủ lớn. Tập tham số Pivot–G2 mới giảm năng lượng độ cong trung bình
từ 3,8024 xuống 3,1797 trong sweep độc lập (khoảng 16,4%) và vẫn cho xu hướng tốt
trên nửa held-out, nhưng đây chỉ là kiểm tra tuning ban đầu, không thay cho test set
khóa.

## Kết quả closed-loop đã xác thực

Ma trận sạch, mỗi phương pháp × lần lặp khởi động một Gazebo partition/process
group riêng:
`results/execution_matrix_v7_clean_repeated_20260723/lower_left_diagonal_summary.json`.
Sáu phương pháp × ba lần lặp nhận raw path có cùng SHA-256. Cả 18/18 trial tới
đích theo action **và** ground truth, không có footprint collision sample hoặc
collision-monitor intervention. Ba lần lặp vẫn quá ít để kiểm định ưu thế thống
kê; số liệu này dùng để xác nhận pipeline, repeatability sơ bộ và trade-off.

| Phương pháp | Thời gian mean±SD (s) | Clearance min/p05 (m) | RMS bám mean±SD (m) | Sai số đích TB vị trí/yaw | `∫κ²ds` reference |
|---|---:|---:|---:|---:|---:|
| Raw | 62,258 ± 0,232 | 0,145 / 0,215 | 0,01443 ± 0,00095 | 0,042 m / 0,096 rad | 12,001 |
| Simple | **62,007 ± 0,245** | 0,215 / 0,220 | 0,01563 ± 0,00551 | 0,040 m / 0,084 rad | **1,847** |
| Savitzky–Golay | 62,211 ± 0,182 | **0,247 / 0,247** | 0,01384 ± 0,00248 | 0,048 m / 0,078 rad | 3,254 |
| Constrained | 62,320 ± 0,023 | 0,234 / 0,247 | 0,01272 ± 0,00104 | 0,049 m / 0,079 rad | 3,068 |
| Pivot–G2 | 65,665 ± 0,362 | 0,234 / 0,247 | 0,01785 ± 0,00224 | 0,043 m / 0,094 rad | 2,764 |
| Adaptive Hybrid | 65,543 ± 0,167 | 0,234 / 0,247 | **0,01239 ± 0,00109** | 0,045 m / 0,099 rad | 2,764 |

Trong tình huống này, Hybrid chọn Pivot–G2 ở cả ba lượt, output hash trùng Pivot
và tăng clearance tối thiểu khoảng 1,9 cm so với Simple, đổi lại tốn thêm khoảng
3,54 giây và có năng lượng độ cong reference cao hơn. RMS bám trung bình thấp nhất
của Hybrid là tín hiệu đáng kiểm tra, không phải bằng chứng rằng controller bám
Hybrid tốt hơn nói chung vì `n=3` và chỉ có một scenario.

## Các lỗi thực nghiệm đã phát hiện và sửa

1. Một action controller từng báo thành công trong khi Gazebo đang tái sử dụng
   robot có spawn pose sai, khiến sai số cuối khoảng 1,76 m. Runner nay kiểm tra
   physical spawn trước planning và chỉ ghi success khi ground truth thật sự tới
   goal. File chẩn đoán lỗi không được dùng làm kết quả khoa học.
2. Các tiến trình `gz sim` của lượt cũ không dừng, làm CPU bị tranh chấp và thời
   gian chạy không đáng tin. Chỉ gọi `/server_control` vẫn có thể để sót một server.
   Matrix runner nay tạo session riêng cho mỗi trial, thử dừng mềm rồi xác minh và
   fallback SIGINT→SIGTERM→SIGKILL trên **đúng process group đó**. Ma trận v7 đã
   chạy sạch 18 lượt và không để lại orphan.
3. Subscription `/map` của runner dùng volatile QoS nên node khởi động muộn có
   thể không nhận static map. QoS đã đổi sang reliable + transient-local.
4. Diagnostics của Hybrid có thể đến sau action rất ngắn. Runner nay đợi một cửa
   sổ ngắn và đối chiếu hash output với method được chọn.
5. Metric độ cong cũ phạt vô hạn marker quay tại chỗ. Metric mới tách năng lượng
   độ cong của chuyển động tịnh tiến khỏi tổng góc/số lần pivot.

## Giả thuyết và cổng quyết định cho bài báo

Giả thuyết chính nên khóa trước test set:

- H1: Hybrid giữ nguyên Simple khi mức cải thiện an toàn không đạt ngưỡng.
- H2: Khi Hybrid kích hoạt Pivot–G2, clearance swept-footprint tăng có ý nghĩa,
  trong khi năng lượng độ cong không vượt ngân sách đã khai báo.
- H3: Executor maneuver-aware thực hiện được pivot marker với tỷ lệ thành công và
  sai số đích chấp nhận được trên mô phỏng và robot thật.

Quy tắc hiện tại chọn Pivot khi Simple không an toàn; hoặc khi cả hai an toàn,
Pivot giảm peak proximity-cost ít nhất 20 cost unit và năng lượng của Pivot không
vượt `2 × (E_simple + 0,25)`. Sau khi đóng băng tập tuning, không thay ngưỡng theo
kết quả test.

Cổng **go/no-go** trước khi viết claim mạnh:

- Nếu Hybrid không tăng clearance ở nhóm góc/hành lang khó trên test set, bỏ claim
  an toàn và viết bài theo hướng framework thực thi maneuver, hoặc dừng hướng này.
- Nếu tăng clearance nhưng collision/success/time xấu đáng kể, điều chỉnh bài toán
  thành đánh đổi Pareto; không gọi là cải thiện toàn diện.
- Nếu chỉ tốt trên một map hoặc một cặp start–goal, chưa đủ cho bài thực nghiệm.
- Nếu robot thật không tái hiện xu hướng mô phỏng, báo riêng sim-to-real gap và
  không gộp hai nguồn dữ liệu.

## Thực nghiệm còn bắt buộc

1. Dùng ba layout pilot hiện có để khóa train/test split; tăng từ 8 lên tối
   thiểu 20–30 cặp start–goal hợp lệ mỗi map. Lưu raw path/costmap snapshot
   thành dataset để mọi method nhận input đồng nhất.
2. Tách scenario tuning và held-out theo map/start–goal. Không chỉnh ngưỡng sau
   khi đọc held-out.
3. Chạy closed-loop ít nhất 10 lần cho từng method trên tập tình huống đại diện,
   private Gazebo domain/partition và metadata cố định. Randomize thứ tự nếu có
   nhiễu hoặc tải hệ thống.
4. Ablation: Pivot–G2 cũ, Pivot–G2 đã tuning, Hybrid bỏ safety gate, Hybrid bỏ
   energy budget, và executor không hiểu pivot.
5. Metrics chính: success theo ground truth, collision/intervention, clearance
   min/p05, thời gian, path length, smoothing CPU time, năng lượng độ cong tịnh
   tiến, góc/số pivot, RMS/max tracking, sai số đích, `cmd_vel` variation và
   executed-energy proxy.
6. So sánh paired theo cùng scenario. Dùng Friedman rồi Wilcoxon signed-rank có
   Holm correction; báo effect size và bootstrap confidence interval, không chỉ
   p-value.
7. Robot thật: đo lại wheel radius/separation, kiểm tra 1320 tick/vòng, đồng bộ
   A1M8–BNO055, có e-stop; chạy ít nhất 10 lượt ở ba layout. Dùng camera/AprilTag
   hoặc ground truth ngoài nếu muốn kết luận sai số bám tuyệt đối.

## Mốc làm việc đến hạn nộp

| Mốc | Kết quả cần khóa |
|---|---|
| 24–31/07 | Unit/integration test sạch; khóa thuật toán, threshold và dataset schema |
| 01–18/08 | Ba map, raw-path dataset, offline benchmark và ablation |
| 19–31/08 | Closed-loop nhiều lần lặp, phân tích failure và sensitivity |
| 01–10/09 | Robot thật, calibration và paired trials |
| 11–17/09 | Thống kê, bảng/hình và khóa claim |
| 18–25/09 | Viết bài IEEE A4 tối đa 6 trang, phản biện nội bộ |
| 26–29/09 | Tái lập từ workspace sạch, kiểm PDF/metadata và nộp |

## Lệnh tái lập hiện tại

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Offline: sáu phương pháp, 12 tình huống
ros2 run adaptive_pivot_g2_benchmark batch_benchmark --ros-args \
  -p "planners:=['ThetaStar']" \
  -p output_csv:=$PWD/results/fair_batch.csv \
  -p output_json:=$PWD/results/fair_batch_summary.json

# Closed-loop riêng Hybrid
ros2 launch adaptive_pivot_g2_benchmark execution_trial.launch.py \
  scenario:=lower_left_diagonal method:=adaptive_hybrid

# Ma trận sáu phương pháp; dùng >=10 repetitions cho benchmark chính
ros2 run adaptive_pivot_g2_benchmark execution_matrix -- \
  --scenario lower_left_diagonal --repetitions 10
```

Kết quả hiện tại là bằng chứng kỹ thuật để quyết định hướng, chưa phải dataset cuối
của bài báo. Mọi số liệu dùng trong manuscript phải được sinh lại sau khi khóa
commit, tham số, map, scenario split và số lần lặp.
