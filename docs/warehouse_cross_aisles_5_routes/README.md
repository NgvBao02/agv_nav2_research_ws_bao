# Nghiên cứu PSTMO trên năm tuyến của kho giao cắt

PDF chính: `../PSTMO_KHO_GIAO_CAT_5_QUY_DAO.pdf`.

Báo cáo hoàn thiện ngày 04/10/2026: 96 trang, 92 hình, kèm PNG 220 dpi
và SVG. Đợt mới ngày 03/10 có 100 lượt, 98 đạt; toàn bộ hồ sơ có 125 lượt,
123 đạt. Hai lượt không đạt vẫn được giữ: R04/ThetaStar/Raw (sai số đích
0,100234 m) và R03/SmacHybrid/PSTMO (0,102385 m), vượt ngưỡng 0,100000 m.

R01 giữ tuyến `cross_aisle_transfer` trong mục 7.5, trang 81–91 của `PSTMO.pdf`.
R02–R05 được chọn trước khi chạy: chéo dài toàn kho, chữ U phía Nam,
chéo ngược Tây Bắc–Đông Nam, rẽ từ hành lang ngang vào lối lấy hàng.

## Nguồn dữ liệu

- R01: 25 lượt thực thi lịch sử trong `results/pstmo_execution_full_20260803/warehouse_cross_aisles/`
  và `results/pstmo_execution_theta_star_20260802/warehouse_cross_aisles/`.
- Hình học và diagnostics R01: các snapshot RViz trong
  `docs/pstmo_bao_cao_toan_dien_assets/rviz_cases/`, đúng bộ số liệu PDF gốc.
- R02–R05: 100 lượt Gazebo/Nav2 mới trong `execution/`.
- `geometry_preflight.csv`: 100 phép so sánh hình học sơ bộ, không phải 100 lượt di chuyển bổ sung.
- `all_125_trials.csv`: bảng tổng hợp từ các lượt thực thi; cột `source` trỏ tới JSON gốc.
- `audit_manifest.json`: hash nguồn và kiểm tra cùng đầu vào Raw theo nhóm tuyến–planner.
- `figures/`: PNG 220 dpi và SVG vector từ dữ liệu đo. Không sử dụng hình sinh AI.
- `page_index.json`, `figure_index.json`: chỉ mục trang và hình.
- `quality_checks.json`: kiểm tra bản PDF cuối, số trang/hình và SHA-256.

Các trang hình học R01 giữ nguyên snapshot của PDF gốc. Đường thực thi và
thời gian xử lý ở đợt Gazebo lịch sử là một lần thu riêng, nên một số số liệu
Constrained/thời gian CPU có thể khác nhẹ snapshot. Các bảng tổng hợp dùng
đường thực thi nhất quán cho cả 125 lượt.

## Giao thức

Mỗi tuyến có 5 planner × 5 phương án (Raw, Simple, Savitzky–Golay,
Constrained, PSTMO). Một lượt cho mỗi tổ hợp. Năm planner không phải năm
lần lặp cùng đầu vào. Không gán khoảng tin cậy hoặc tuyên bố ưu thế thống kê
từ số liệu một lượt.

Bốn ma trận mới chạy đồng thời với các ROS domain không giao nhau và
Gazebo partition riêng. Mỗi lượt khởi động stack sạch. Thời gian robot là
đồng hồ mô phỏng; thời gian xử lý CPU chịu ảnh hưởng tải máy. Không sửa world,
tham số smoother hoặc controller. Recorder diagnostics chỉ đăng ký topic.

Lỗi khởi tạo hạ tầng được phép retry tối đa một lần theo ma trận gốc. Log giữ
các lần thử; JSON ghi `infrastructure_attempt_count`. Một lượt đã thực thi
nhưng không đạt tiêu chí đích/dừng không được chạy lại để thay kết quả.

Các hash Raw phải trùng trong nhóm năm phương án. Một lượt chỉ đạt khi action
thành công, vào dung sai đích theo ground truth và dừng ổn định. Phân loại thất
bại được giữ kể cả khi chỉ vượt ngưỡng một lượng nhỏ.

## Tái lập

Từ gốc workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 python3 tools/run_cross_aisle_study.py wide_diagonal_transfer 60 results/cross_aisle_new_repeat
```

Đối số cuối là thư mục kết quả mới. Script từ chối ghi đè thư mục đã có JSON.
Để chạy đồng thời các tuyến khác, dùng base domain 90, 120, 150 và cùng
thư mục gốc mới. Các khoảng domain mỗi ma trận dài 25 ID.

Xuất PDF với Python có `reportlab`, `matplotlib`, `numpy`, `scipy`, `Pillow`,
`PyYAML` và font DejaVu; dùng NumPy/Matplotlib tương thích của hệ thống:

```bash
python3 -m venv --system-site-packages /tmp/pstmo-cross5-pdf-venv
/tmp/pstmo-cross5-pdf-venv/bin/pip install reportlab
PYTHONNOUSERSITE=1 /tmp/pstmo-cross5-pdf-venv/bin/python tools/build_cross_aisle_report.py
```

Builder chỉ xuất PDF cuối khi đủ 125 kết quả. Cờ `--preview` dùng để kiểm tra
bố cục của những nhóm đã hoàn tất; preview không phải kết quả giao cuối.

## Giới hạn diễn giải

Map tĩnh, mô phỏng một robot, một lần/tổ hợp. Chưa có nhiều seed, vật cản động,
thay đổi tải, đo điện năng hay thử nghiệm phần cứng. Eκ là tích phân bình
phương độ cong, không phải điện năng. Hậu kiểm footprint trên PGM không đồng
nhất với bằng chứng tiếp xúc vật lý. Các hình 3D/tái dựng Bézier được ghi rõ
là minh họa khoa học; không được trình bày như ảnh chụp GUI.
