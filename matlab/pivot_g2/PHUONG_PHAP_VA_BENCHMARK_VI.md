# Đặc tả phương pháp và cách diễn giải kết quả

## 1. Câu hỏi nghiên cứu

Biến độc lập là **bộ hậu xử lý path**. Planner, path đầu vào và controller là biến kiểm soát. Hai thí nghiệm chính nên trình bày tách nhau:

- Theta* + 6 postprocessor;
- NavFn A* + 6 postprocessor.

Thiết kế này trả lời: cùng một đường planner rời rạc, bộ hậu xử lý nào tạo maneuver an toàn, trơn và khả thi hơn cho robot vi sai?

## 2. Phương pháp đề xuất

Với mỗi góc có góc quay `phi`, thuật toán sinh tập bán kính:

```text
R = [0.20 0.25 0.30 0.35 0.40 0.50 0.60] m
```

Một arc bị loại nếu:

- không đủ chiều dài tiếp tuyến ở hai đoạn kề;
- footprint va chạm;
- clearance nhỏ hơn ngưỡng;
- bánh trong phải quay ngược;
- tốc độ thân, tốc độ góc hoặc tốc độ bánh vượt giới hạn.

Với arc hợp lệ:

```text
v_arc = min(v_max, R * omega_max,
            wheel_speed_max / (1 + wheel_base/(2R)))
omega_arc = sign(phi) * v_arc / R
```

Mỗi ứng viên được đặt vào cùng một cửa sổ quanh góc. G2 gồm hai đoạn thẳng bù
và transition; pivot gồm đoạn vào, dừng, profile quay tam giác/hình thang và
đoạn ra. Hai phương án dùng cùng boundary-speed target và cùng bộ time
parameterization có giới hạn vận tốc, gia tốc tuyến tính, gia tốc góc và tốc độ
hai bánh. Không được dùng lại ước lượng curve-only của phiên bản cũ.

Arc chỉ qua cổng thời gian khi:

```text
T_arc,min + delta_T < T_pivot
```

Sau đó, các arc trong `0.20 s` so với arc nhanh nhất được coi là cạnh tranh.
Thuật toán chọn score clearance–angular-rate–curvature-energy cao nhất; ba thành
phần được chuẩn hóa bằng các scale vật lý cố định, không min–max theo tập ứng
viên cục bộ.

Ứng viên proposed không còn là cung tròn có bước nhảy độ cong. Nó dùng đoạn Bézier quintic G2 với curvature bằng 0 ở hai đầu; fixed-radius baseline vẫn dùng cung tròn để giữ phép so sánh ablation rõ ràng.

Laplacian micro-refinement và LOS-pruning riêng cho proposed đều tắt trong cấu
hình chính. Bật refinement chỉ hợp lệ như ablation và output sau đó không được
gọi là G2.

## 3. Tính khả thi động học

Mô phỏng hiện dùng robot vi sai với giới hạn:

- vận tốc tuyến tính/góc;
- gia tốc tăng/giảm;
- gia tốc góc;
- vận tốc từng bánh;
- bánh trong không đảo chiều khi chạy arc;
- footprint chữ nhật và clearance.

Đây là kiểm tra tính khả thi theo mô hình và giới hạn lệnh, chưa phải xác nhận phần cứng. Khi có xe thật cần bổ sung khối lượng, mô-men động cơ, bán kính bánh, encoder, ma sát, độ trễ truyền động, chu kỳ điều khiển và nhiễu localization.

## 4. Phạm vi tương đương Nav2

`NAV2_SIMPLE` và `NAV2_SAVITZKY_GOLAY` tái hiện phương trình/tham số mặc định từ mã nguồn Nav2 nhưng chạy trên dữ liệu MATLAB và costmap của dự án.

`NAV2_CONSTRAINED` chỉ tương đương về nhóm hàm mục tiêu. Plugin chính thức dùng Ceres, footprint/cost term và cơ chế nội suy của C++; vì vậy bảng phải ghi `MATLAB_EQUIVALENT_OBJECTIVE`.

Muốn tuyên bố kết quả Nav2 runtime, cần viết plugin ROS 2 cho proposed, dùng cùng rosbag/costmap/robot và chạy trực tiếp trong Nav2.

## 5. Cách đọc metric

| Metric | Tốt khi | Ý nghĩa |
|---|---|---|
| `PostprocessTime` | nhỏ | chi phí tính toán của hậu xử lý |
| `IntegratedSquaredCurvature` | nhỏ | độ cong tập trung/đột ngột thấp hơn |
| `CompletionTime` | nhỏ | robot hoàn thành nhanh hơn |
| `NumberOfFullStops` | nhỏ | ít dừng/quay tại chỗ hơn |
| `PositionRMSE` | nhỏ | controller bám reference tốt hơn |
| `MinimumClearance` | lớn | biên an toàn thực tế tốt hơn |
| `Jomega` | nhỏ | lệnh góc ít biến thiên hơn |
| `FallbackRate` | nhỏ | bộ hậu xử lý ít phải trả path gốc/pivot |
| `TaskSuccessRate` | lớn | đạt goal, không collision/vi phạm giới hạn |

Không có một metric duy nhất quyết định thắng. Proposed có thể chậm hơn một smoother tối ưu nhưng tạo maneuver dễ giải thích hơn, giữ footprint-safe theo từng ứng viên và fallback pivot có chủ đích. Đây là trade-off cần báo trung thực.

## 6. Ablation nên có trong bài báo

- fixed `R=0.30 m` so với adaptive radius;
- adaptive chỉ tối thiểu thời gian so với adaptive hai tầng;
- bỏ clearance term;
- bỏ angular-rate term;
- thay đổi `delta_T` và time slack;
- Theta* so với NavFn để kiểm tra khả năng tổng quát qua planner.

## 7. Kết luận nào được phép

Nếu batch cho thấy proposed giảm stop/`Jomega`, giữ success/clearance và chi phí tính toán chấp nhận được, có thể kết luận bộ chọn chuyển path góc gãy thành maneuver phù hợp robot vi sai tốt hơn các baseline trong bộ benchmark.

Không nên kết luận proposed thay thế Theta*, NavFn hoặc toàn bộ Nav2: nó là lớp hậu xử lý có thể đặt sau các planner đó.
