# Bản đồ repository tối giản

Tài liệu này trả lời “đọc ở đâu” chứ không liệt kê mọi file. Mã nguồn và cấu
hình đang chạy luôn được ưu tiên hơn báo cáo hoặc snapshot kết quả.

Không coi danh sách dưới đây là đầy đủ tuyệt đối. Trước khi thay đổi một giao
diện, tham số hoặc ID, dùng `rg` để tìm producer, consumer, test và tài liệu của
nó trên toàn `src/`; mở thêm phạm vi khi kết quả vượt ra ngoài package ban đầu.

## Cây chức năng

```text
.
├── src/
│   ├── adaptive_pivot_g2/             # thư viện C++ lõi PSTMO
│   ├── adaptive_pivot_g2_nav2/        # plugin Nav2 Smoother và safety
│   ├── adaptive_pivot_g2_benchmark/   # benchmark hình học/vòng kín
│   ├── adaptive_pivot_g2_rviz/        # panel RViz2
│   └── vacuum_robot_gazebo/            # robot, world/map, Nav2, launch
├── tools/                              # script bootstrap, báo cáo, audit
├── docs/                               # báo cáo và hồ sơ tái lập
├── results/                            # dataset hiện hành và snapshot
├── file_3D/                            # STEP cơ khí gốc
├── REFERENCES/                         # tài liệu tham khảo
├── bao_Rev_ecit_2026/                  # bản thảo REV-ECIT
└── final_bao_ICEEIS/                   # bản thảo/bài báo ICEEIS
```

## Nguồn sự thật theo chủ đề

| Chủ đề | Nguồn ưu tiên |
| --- | --- |
| Hình học, tìm ứng viên, G², DP, time parameterization | `src/adaptive_pivot_g2/include/` và `src/adaptive_pivot_g2/src/` |
| Hành vi plugin PSTMO/Hybrid | `src/adaptive_pivot_g2_nav2/` |
| Tham số Nav2 đang chạy | `src/vacuum_robot_gazebo/config/nav2_params.yaml` |
| Planner/smoother/environment hỗ trợ | catalog trong `src/adaptive_pivot_g2_rviz/include/` và `nav2_params.yaml` |
| Metric và giao thức benchmark | `src/adaptive_pivot_g2_benchmark/` |
| Robot mô phỏng, sensor, bridge | `src/vacuum_robot_gazebo/models/`, `urdf/`, `config/bridge.yaml` |
| World/map | `src/vacuum_robot_gazebo/worlds/` và `maps/` cùng basename |
| Kết quả dùng để báo cáo | `results/README.md`, rồi README/aggregate của đúng cohort |
| Cách dựng báo cáo | README hồ sơ tương ứng và script trong `tools/` |

## Luồng chạy chính

```text
Gazebo + robot + bridge
        ↓
Map/AMCL → global planner → Raw path
        ↓
Nav2 Smoother Server → Simple / Savitzky–Golay / Constrained / PSTMO / Hybrid
        ↓
Regulated Pure Pursuit → velocity smoother → Collision Monitor → cmd_vel
```

`raw` là baseline thực thi, không phải plugin smoother. Cấu hình mặc định hiện
tại của PSTMO độc lập là `condition_only + hierarchical_alpha_two_trim`;
Adaptive Hybrid dùng nhánh Pivot `legacy_joint_d_q`. Luôn xác minh lại hai kết
luận này trong source/config nếu task thay đổi thuật toán.

## Định tuyến task

- Thay đổi thuật toán thuần C++: bắt đầu ở `adaptive_pivot_g2`, sau đó kiểm tra
  wrapper Nav2 và test liên quan.
- Thay đổi plugin hoặc footprint safety: bắt đầu ở `adaptive_pivot_g2_nav2`, rồi
  đối chiếu `nav2_params.yaml` và benchmark diagnostics.
- Thay đổi launch/map/robot: bắt đầu ở `vacuum_robot_gazebo`; bảo đảm world/map
  và catalog môi trường vẫn đồng nhất.
- Thay đổi method/planner ID: cập nhật config, benchmark, RViz catalog/panel,
  root README và tài liệu agent bị ảnh hưởng.
- Thay đổi dataset/báo cáo: không suy ngược hành vi hiện tại từ tên thư mục;
  đọc `docs/agent/DATA_POLICY.md` trước.

Không dùng PDF/DOCX/HTML sinh sẵn làm nguồn duy nhất để sửa code. Không dùng
snapshot trong `results/` để kết luận cấu hình hiện tại nếu chưa đối chiếu hash
hoặc source.
