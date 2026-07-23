# Benchmark bộ hậu xử lý đường đi cho robot vi sai

Thư mục này đánh giá đúng đóng góp của bộ chọn **pivot–arc thích nghi**: giữ cố định global planner, bản đồ, đường planner đầu vào, mô hình robot, profile tốc độ và controller; chỉ thay bộ hậu xử lý.

Pipeline chính:

```text
Planner chạy đúng 1 lần
          |
          +--> No smoother
          +--> Nav2 Simple (MATLAB-equivalent)
          +--> Nav2 Savitzky–Golay (MATLAB-equivalent)
          +--> Nav2 Constrained (objective-equivalent, không phải Ceres C++)
          +--> Fixed-radius arc
          +--> Proposed adaptive pivot–arc
                          |
             cùng speed profile + cùng controller
```

## Chạy nhanh

Mở MATLAB, chuyển **Current Folder** đến thư mục này hoặc chạy:

```matlab
cd('/home/linh-pham/agv_nav2_research_ws/matlab/pivot_g2')
```

So sánh sáu bộ hậu xử lý với Theta* trên map 1, scenario 1:

```matlab
close all
result = compare_path_postprocessors('THETA_STAR',1,1);
```

Đổi sang NavFn A*:

```matlab
close all
result = compare_path_postprocessors('NAVFN_ASTAR',1,1);
```

Các planner hợp lệ cho benchmark:

```text
THETA_STAR
NAVFN_ASTAR
NAVFN_DIJKSTRA
SMAC_2D
TURN_ASTAR
```

Lệnh cũ sau đây vẫn chạy nhưng sẽ chuyển sang benchmark mới với Theta*:

```matlab
compare_nav2_planners(1,1)
```

## Chạy từng map mà không sửa file

Mỗi map có 5 scenario. Thay số cuối từ 1 đến 5.

```matlab
compare_path_postprocessors('THETA_STAR',1,1) % SMALL_WAREHOUSE
compare_path_postprocessors('THETA_STAR',2,1) % MEDIUM_WAREHOUSE
compare_path_postprocessors('THETA_STAR',3,1) % LARGE_WAREHOUSE
compare_path_postprocessors('THETA_STAR',4,1) % DENSE_RACK_WAREHOUSE
compare_path_postprocessors('THETA_STAR',5,1) % OPEN_FACTORY
compare_path_postprocessors('THETA_STAR',6,1) % MIXED_CORRIDOR
```

Chạy không giao diện trên máy Linux:

```bash
matlab -batch "cd('/home/linh-pham/agv_nav2_research_ws/matlab/pivot_g2'); runPivotG2Verification()"
```

## Chạy batch để lấy số liệu bài báo

Theta* và NavFn A*, toàn bộ 6 map × 5 scenario:

```matlab
benchmark = benchmark_path_postprocessors();
```

Chỉ Theta*:

```matlab
benchmark = benchmark_path_postprocessors({'THETA_STAR'});
```

Chỉ map 1, 3, 6 và scenario 1, 2:

```matlab
benchmark = benchmark_path_postprocessors( ...
    {'THETA_STAR','NAVFN_ASTAR'},[1 3 6],[1 2]);
```

Tạo lại toàn bộ dữ liệu phục vụ bài hội thảo (A* ablation, maneuver ablation và benchmark hậu xử lý):

```matlab
outputs = generateConferencePaperData();
```

## Sáu phương pháp được so sánh

| Tên trong CSV | Ý nghĩa | Mức tương đương |
|---|---|---|
| `NO_SMOOTHER` | Giữ nguyên XY từ planner | baseline |
| `NAV2_SIMPLE` | Công thức lặp `w_data`, `w_smooth`, refinement và tham số mặc định Nav2 | MATLAB-equivalent |
| `NAV2_SAVITZKY_GOLAY` | Cửa sổ 7, đa thức bậc 3, hệ số từ Vandermonde/pseudoinverse, refinement | MATLAB-equivalent |
| `NAV2_CONSTRAINED` | Mục tiêu distance + smoothness + obstacle + curvature | MATLAB objective-equivalent |
| `FIXED_RADIUS_ARC` | Bo góc bằng `R = 0.30 m`, fallback pivot nếu không an toàn | custom baseline |
| `PROPOSED_PIVOT_ARC` | Transition quintic G2 thích nghi, fallback pivot | phương pháp đề xuất |

LOS-pruning riêng cho hai phương pháp custom đã bị tắt. Nếu cần pruning, phải
áp dụng nó như một bước chung trước khi chia cùng path cho toàn bộ baseline.
Laplacian micro-refinement cũng tắt mặc định vì dịch chuyển từng mẫu sẽ phá bảo
đảm G2 của đường Bézier ban đầu.

Lưu ý khoa học: Constrained Smoother chính thức dùng Ceres. Máy này không chạy ROS 2/Nav2 C++ trong benchmark, nên không được ghi các thời gian MATLAB là thời gian runtime của plugin Nav2. Cột `Implementation` và file manifest luôn ghi rõ phạm vi này.

Tài liệu/mã nguồn chính thức đã dùng để đối chiếu:

- [Danh sách smoother plugins của Nav2](https://docs.nav2.org/plugins/index.html)
- [Simple Smoother](https://docs.nav2.org/configuration/packages/configuring-simple-smoother.html)
- [Constrained Smoother](https://docs.nav2.org/configuration/packages/configuring-constrained-smoother.html)
- [Mã nguồn Savitzky–Golay Smoother](https://github.com/ros-navigation/navigation2/blob/main/nav2_smoother/src/savitzky_golay_smoother.cpp)
- [Mã nguồn Simple Smoother](https://github.com/ros-navigation/navigation2/blob/main/nav2_smoother/src/simple_smoother.cpp)

## Animation và hình kết quả

Mỗi lần chạy `compare_path_postprocessors` tạo một folder:

```text
results/postprocessor_comparison/
  <PLANNER>_<MAP>_<SCENARIO>_<TIMESTAMP>/
```

Các file chính:

| File | Nội dung |
|---|---|
| `input_planner_path.png` | đúng một đường planner gốc dùng chung |
| `comparison_reference_paths.png` | reference sau sáu bộ hậu xử lý |
| `comparison_actual_trajectories.png` | quỹ đạo robot thực tế |
| `comparison_method_panels.png` | 2 × 3 panel, reference và actual của từng method |
| `comparison_metrics.png` | thời gian, độ cong, stop, RMSE, clearance |
| `postprocessor_animation.mp4` | sáu robot chạy đồng bộ theo cùng thời gian vật lý |
| `animation_final_frame.png` | frame cuối animation |
| `postprocessor_comparison.csv` | bảng metric đầy đủ |
| `corner_decisions.csv` | quyết định pivot/arc tại từng góc |
| `experiment_manifest.txt` | planner/path/controller/tham số dùng trong thí nghiệm |
| `comparison_summary.txt` | tóm tắt tự động, không thay cho phân tích thống kê batch |

Animation dùng một cửa sổ 2 × 3. Đường xám đứt là reference; đường màu là robot thực tế; hình chữ nhật là footprint robot. Tất cả panel tiến theo cùng `timeNow`, không phải mỗi thuật toán tự chạy theo phần trăm đường riêng.

Nếu muốn tắt animation để chạy nhanh:

```matlab
config = defaultCornerOptimizerConfig();
comparison = defaultPostprocessorComparisonConfig(config);
comparison.enableAnimation = false;
```

Lệnh trên dành cho người sửa pipeline bằng hàm thấp hơn. Entry `compare_path_postprocessors` mặc định luôn tạo animation đầy đủ.

## Tính công bằng được kiểm tra thế nào

`runPathPostprocessorComparison`:

1. chạy planner đúng một lần;
2. sao chép cùng ma trận `N × 2` cho sáu method;
3. kiểm tra `isequaln` sau từng method để phát hiện sửa input tại chỗ;
4. lưu `InputPathSignature` giống nhau ở mọi dòng CSV;
5. dùng cùng `simulateDifferentialDrive`, `dt`, gain, giới hạn vận tốc/gia tốc và footprint;
6. xuất `fairness.passed`, `sameInputPathForAll`, `sameControllerForAll` trong MAT/manifest.

Các metric hình học và metric robot thực tế được tách biệt. Không được dùng một đường reference đẹp để tuyên bố robot bám chính xác nếu `PositionRMSE`, collision hoặc limit violation không đạt.

## Bộ chọn pivot–arc đã cải tiến

Phiên bản cũ so thời gian curve-only với thời gian pivot có giảm tốc/quay/tăng
tốc, nên cổng thời gian bị thiên lệch. Phiên bản hiện tại đặt pivot và mọi bán
kính vào cùng một cửa sổ entry–exit, cộng các đoạn thẳng bù, rồi dùng chung
profile giới hạn `v`, `omega`, gia tốc tuyến tính, gia tốc góc và tốc độ bánh.
Sau đó bộ chọn dùng hai tầng:

1. chỉ nhận arc nếu `T_arc + delta_T < T_pivot`;
2. lấy các arc có `T_arc <= T_arc,min + 0.20 s`;
3. trong tập cạnh tranh, chuẩn hóa clearance, `abs(omega)` và curvature energy
   bằng các mốc vật lý cố định rồi chấm:

```text
score = 0.35 * clearance_score
      + 0.25 * low_angular_rate_score
      + 0.40 * low_curvature_energy_score
```

Đoạn chuyển dùng Bézier bậc năm với ba control point đầu thẳng hàng theo tiếp tuyến vào và ba control point cuối thẳng hàng theo tiếp tuyến ra. Vì vậy curvature bằng 0 tại hai điểm ghép với đoạn thẳng, tránh bước nhảy `0 -> 1/R` của cung tròn thường. Hệ số control mặc định sau sweep đa map là `0.35`.

Nếu không có transition footprint-safe, thuật toán fallback pivot. `corner_decisions.csv` lưu tập ứng viên cạnh tranh, score, bán kính, thời gian, clearance và lý do chọn để có thể audit từng góc.

Kiểm tra hồi quy sau khi mở MATLAB:

```matlab
reports = runPivotG2Verification();
```

Không dùng lại CSV được tạo trước ngày sửa time model. Đặc biệt bộ kết quả trong
ZIP có timestamp sớm hơn mã G2 hiện tại và cho adaptive trùng tuyệt đối với
fixed-radius; các số đó chỉ có giá trị chẩn đoán, không phải kết quả bài báo.

Các tham số nằm ở:

- `defaultCornerOptimizerConfig.m`: robot, giới hạn động học, tập bán kính, `delta_T`;
- `defaultPostprocessorComparisonConfig.m`: tham số Nav2-equivalent, luật adaptive và animation.

Profile mặc định được đánh dấu `PAPER_SIMULATION_UNVERIFIED`. Khi đã đo robot
thật, dùng `applyMeasuredRobotProfile(config, measured)` để gán toàn bộ footprint,
wheelbase và giới hạn chuyển động trong một lần; hàm sẽ tính lại inflation radius
và các scale phụ thuộc. Không đổi rời rạc vài field vì sẽ làm benchmark tự mâu
thuẫn.

## Kết quả batch dùng cho bài báo

Folder batch chứa:

| File | Dùng để làm gì |
|---|---|
| `all_postprocessor_runs.csv` | toàn bộ dòng thí nghiệm |
| `aggregate_postprocessor_summary.csv` | mean/success/fallback theo planner và method |
| `paired_vs_proposed.csv` | hiệu số paired của proposed so với từng baseline trên cùng case |
| `postprocessor_benchmark.mat` | dữ liệu MATLAB và từng case |

Quy ước trong `paired_vs_proposed.csv`:

- `MeanDelta... = proposed - baseline`;
- với completion time, curvature, stop, RMSE, `Jomega`: số âm tốt hơn;
- với clearance: số dương tốt hơn;
- win rate được tính chỉ trên các cặp mà cả hai method hoàn thành an toàn.

Không kết luận phương pháp mới tốt hơn chỉ từ một hình. Bài báo nên báo ít nhất: success rate, fallback rate, postprocess time, curvature energy, completion time, full stops, RMSE, clearance, `Jomega`, paired delta và win rate trên toàn bộ map/scenario.

## Các file lõi

- `compare_path_postprocessors.m`: entry một case;
- `benchmark_path_postprocessors.m`: batch nhiều planner/map/scenario;
- `runPathPostprocessorComparison.m`: pipeline công bằng;
- `applyPathPostprocessor.m`: API sáu hậu xử lý;
- `nav2SimpleSmootherEquivalent.m`;
- `nav2SavitzkyGolaySmootherEquivalent.m`;
- `nav2ConstrainedSmootherEquivalent.m`;
- `selectCornerManeuver.m`: bộ chọn đề xuất;
- `simulateDifferentialDrive.m`: controller/mô hình dùng chung;
- `plotPostprocessorComparison.m` và `animatePostprocessorComparison.m`;
- `exportPostprocessorComparison.m`: CSV/MAT/manifest/summary.

Pipeline so sánh global planner cũ vẫn còn trong các file `runNav2PlannerComparison.m`, `plotNav2ComparisonResults.m` để tham khảo legacy, nhưng không còn là thí nghiệm chính dùng chứng minh đóng góp của pivot–arc.
