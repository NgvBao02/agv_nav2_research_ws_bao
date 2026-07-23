# Kiểm toán bộ mã MATLAB hiện tại

Ngày kiểm toán: 23/07/2026  
Nguồn: `y_tuong_hientai.zip` (91 file `.m`)  
Mã đã đưa vào workspace: `matlab/pivot_g2`

## Kết luận

Bộ mã có cấu trúc nghiên cứu tốt hơn bản báo cáo: đã có nhiều map/scenario,
footprint hình chữ nhật, giới hạn hai bánh, controller chung, log quyết định,
baseline MATLAB-equivalent và regression script. Tuy nhiên kết quả cũ chưa thể
dùng để chứng minh thuật toán thích nghi. Hai lỗi P0 trong hiện thực đã được xác
nhận trực tiếp:

1. `generateTransitionCurveCandidates` chỉ tính thời gian đi trên curve bằng
   `sum(2*ds/(v_i+v_{i+1}))`, trong khi `estimatePivotTime` cộng giảm tốc, quay và
   tăng tốc. Cổng `T_G2 + delta_T < T_pivot` vì vậy so hai đại lượng khác nhau.
2. `smoothReferenceTrajectoryCommon` dịch chuyển từng mẫu ARC bằng Laplacian,
   sau đó ước lượng lại heading/curvature. Đường cuối không còn là Bézier đã được
   chứng minh có `kappa=0` tại hai điểm ghép.

Ngoài ra, chỉ hai phương pháp custom từng được visibility-prune nội bộ, nên dù
ma trận input ban đầu giống nhau, hình học thực sự đưa vào hậu xử lý không giống
các smoother còn lại.

## Dữ liệu cũ không khớp mã hiện tại

Các timestamp trong ZIP cho thấy:

- `maneuver_ablation_summary.csv`: 17/07/2026 11:05;
- `generateTransitionCurveCandidates.m`: 17/07/2026 16:21;
- `selectCornerManeuver.m`: 17/07/2026 16:38.

Như vậy CSV được tạo trước khi mã transition G2 hiện tại tồn tại. Dữ liệu cũng
cho `ADAPTIVE_PIVOT_OR_ARC` trùng **từng metric** với `FIXED_RADIUS` trên 30 ca;
79/83 quyết định là `R=0.30 m`, đúng bán kính fixed baseline. Đây là bằng chứng
rằng kết quả và source không cùng phiên bản.

Ngay cả khi chỉ xem như pilot, kết quả so với pivot-only là hỗn hợp:

| Metric | Thay đổi adaptive so với pivot |
|---|---:|
| Completion time | -3,74% |
| Full stops | -95,18% |
| Position RMSE | +18,04% |
| Heading RMSE | +211,15% |
| Minimum clearance | -6,92% |
| `Jv` | +40,05% |
| `Jomega` | +28,36% |

Không nên viết claim “mượt hơn/an toàn hơn” từ bộ số này. Tất cả bảng và hình
cho bài báo phải chạy lại sau khi khóa code, robot profile và dataset.

## Các sửa đổi đã áp dụng

| Vấn đề | Sửa đổi |
|---|---|
| Time gate bất đối xứng | `estimateCornerManeuverTimes.m` đặt mọi bán kính và pivot vào cùng common window, có đoạn thẳng bù và cùng boundary-speed target. |
| Thiếu gia tốc trong dự đoán G2 | `timeParameterizeMovingPath.m` chạy forward/backward theo gia tốc tuyến tính và lặp để thỏa `Delta omega / Delta t`. |
| So bán kính có endpoint khác nhau | Mọi option dùng chung hai outer endpoint của corner window. |
| Score phụ thuộc tập ứng viên | Thay min–max cục bộ bằng scale vật lý cố định trong config. |
| Refinement phá G2 | Tắt mặc định cho proposed; chỉ được bật như ablation không mang claim G2. |
| Bỏ sót va chạm giữa pose | `evaluatePoseSequenceSafety.m` nội suy SE(2) theo cả dịch chuyển và cung quét của footprint. |
| Lấy mẫu Bézier không chặn chord | Tăng mẫu thích nghi đến khi mọi chord không vượt `arcSampleSpacing`. |
| LOS-pruning riêng cho proposed | Tắt mặc định; mọi postprocessor nhận cùng polyline hình học. |
| Thiếu regression cho các lỗi trên | Thêm `verifyAdaptivePivotG2Regression.m`. |
| Nhầm tham số mô phỏng với robot thật | Gắn profile mặc định là `PAPER_SIMULATION_UNVERIFIED`, `measured=false`. |

## Các blocker còn lại

1. Máy hiện tại không có MATLAB/Octave trong `PATH`; mã MATLAB mới chỉ được kiểm
   tra tĩnh. Ba regression script phải chạy trên MATLAB trước khi chấp nhận patch.
2. Robot profile vẫn là mô phỏng: `L=0.24 m`, footprint `0.40 x 0.30 m`,
   `v_max=0.35 m/s`. Không được dùng nó để kết luận cho robot ROS 2 năm trước.
3. Resolution `0.20 m/cell` quá thô để bảo vệ claim clearance `0.05 m`. Cần
   sensitivity ít nhất 0,20/0,10/0,05 m và dùng 0,05 m cho kết quả chính nếu
   runtime cho phép.
4. `NAV2_SIMPLE`, `NAV2_SAVITZKY_GOLAY` và `NAV2_CONSTRAINED` trong MATLAB là
   bản tương đương, không phải plugin ROS 2 thực. Chúng hợp lệ cho debug, không
   thay benchmark Nav2 C++ chính thức.
5. Bộ chọn vẫn greedy theo từng góc. Kết quả chính phải đo lại sau khi ghép toàn
   path bằng một time parameterizer chung; cần ablation time-gate-only và score.
6. Benchmark hiện chưa có train/test split, Friedman/Wilcoxon-Holm, provenance
   theo commit/params hash và taxonomy failure đầy đủ.

Lõi C++ tương ứng đã được build/test lại: 47 checks, 0 failure. Một regression
common-window cố ý dùng giới hạn gia tốc góc chặt và xác nhận pivot (5,35 s) có
thể nhanh hơn G2 (7,66 s); đây là hành vi thích nghi cần giữ, không phải ép mọi
góc đều chọn G2.

## Lệnh xác minh trên máy có MATLAB

```matlab
cd('/home/linh-pham/agv_nav2_research_ws/matlab/pivot_g2')
runPivotG2Verification()

% Smoke test nhỏ trước, chưa sinh số liệu bài báo
compare_path_postprocessors('THETA_STAR',1,1)
```

Chỉ sau khi các regression và smoke test trên chạy sạch mới chạy
`benchmark_path_postprocessors` và
`generateConferencePaperData` vào một thư mục kết quả mới.
