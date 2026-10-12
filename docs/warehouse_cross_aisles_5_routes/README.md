# Nghiên cứu PSTMO trên năm tuyến của kho giao cắt

PDF chính: [PSTMO_KHO_GIAO_CAT_5_QUY_DAO.pdf](../PSTMO_KHO_GIAO_CAT_5_QUY_DAO.pdf).

Báo cáo cập nhật ngày 12/10/2026 có 109 trang và 93 hình, kèm PNG 220 dpi và
SVG. Bản này giữ nguyên toàn bộ 92 hình của báo cáo 05/10 rồi bổ sung một hình
thống kê theo khối. Ma trận chính vẫn có 125 lượt, 123 đạt. Cohort kiểm chứng
lặp bổ sung 250 lượt PSTMO–Constrained, 243 đạt; Constrained đạt 122/125 và
PSTMO đạt 121/125. Mọi lượt không đạt đều được giữ trong mẫu số.

Hai lượt không đạt của ma trận chính vẫn là R04/ThetaStar/Raw (sai số đích
0,100234 m) và R03/SmacHybrid/PSTMO (0,102385 m), vượt ngưỡng 0,100000 m.
Bảy lượt không đạt của cohort lặp được định danh ở trang 22 và README cohort;
một lỗi lifecycle hạ tầng riêng đã được lưu rồi retry theo policy.

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
- `figure_page_index.json`: số hình, trang PDF, tên ảnh và chú thích sau khi sắp xếp lại.
- `quality_checks.json`: kiểm tra bản PDF cuối, số trang/hình và SHA-256.
- [`../../results/pstmo_journal_repeatability_20261011/`](../../results/pstmo_journal_repeatability_20261011/README.md):
  250 trial lặp, manifest, log, diagnostics, aggregate và bảng phân tích.

Các trang hình học R01 giữ nguyên snapshot của PDF gốc. Đường thực thi và
thời gian xử lý ở đợt Gazebo lịch sử là một lần thu riêng, nên một số số liệu
Constrained/thời gian CPU có thể khác nhẹ snapshot. Các bảng tổng hợp dùng
đường thực thi nhất quán cho cả 125 lượt.

## Giao thức

Mỗi tuyến có 5 planner × 5 phương án (Raw, Simple, Savitzky–Golay,
Constrained, PSTMO). Một lượt cho mỗi tổ hợp. Năm planner không phải năm
lần lặp cùng đầu vào. Không gán khoảng tin cậy hoặc tuyên bố ưu thế thống kê
từ số liệu một lượt.

Cohort journal chạy riêng Constrained và PSTMO năm lần cho mỗi route–planner
cell. Năm lần lặp được lấy trung bình trong 25 cell; CI 95% dùng bootstrap theo
khối 20.000 lần, Wilcoxon hai phía dùng chênh lệch cell và Holm hiệu chỉnh ba
outcome chính. 125/125 cặp có cùng Raw hash. Pipeline không phát lộ seed, nên
đây là fixed-input repeatability, không phải multi-seed robustness.

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

## Cấu trúc đọc của bản cập nhật 12/10

Trang 1–3: bìa, tóm tắt và mục lục có liên kết. Trang 4–8: thuật ngữ,
chuỗi xử lý, thiết kế thí nghiệm, chỉ số và năm tuyến. Trang 9-10: kết quả
tổng hợp. Trang 11-20: mỗi tuyến hai trang tình huống và diễn giải kết quả.
Trang 21–25: thảo luận, cohort lặp, audit, bằng chứng đa môi trường/ablation
lịch sử và giới hạn. Trang 26 là bảng tra phụ lục; trang 27–106 giữ đủ hồ sơ
cơ chế và 25 nhóm phân tích chi tiết. Trang 107–109: đối chiếu cách lấy mẫu,
nguồn dữ liệu và mã kiểm tra.

Mọi ảnh khoa học của bản 05/10 và dữ liệu ma trận 125 lượt giữ nguyên so với
baseline `7602317e4d23806730fbb64a9f3f4cace469286e`. Một hình mới được tạo từ
cohort 250 lượt; không sửa dữ liệu lịch sử để thay kết quả không thuận lợi.

## Tái lập báo cáo và thí nghiệm

Từ gốc workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 python3 tools/run_cross_aisle_study.py wide_diagonal_transfer 60 results/cross_aisle_new_repeat
```

Đối số cuối là thư mục kết quả mới. Script từ chối ghi đè thư mục đã có JSON.
Để chạy đồng thời các tuyến khác, dùng base domain 90, 120, 150 và cùng
thư mục gốc mới. Các khoảng domain mỗi ma trận dài 25 ID.

Lệnh tạo và phân tích cohort journal mới phải dùng thư mục ngày khác:

```bash
python3 tools/run_journal_repeatability_study.py \
  --output-root results/pstmo_journal_repeatability_<YYYYMMDD> \
  --repetitions 5 --workers 4
PYTHONNOUSERSITE=1 python3 tools/analyze_journal_repeatability.py \
  --cohort results/pstmo_journal_repeatability_<YYYYMMDD>
```

Xuất PDF với Python có `reportlab`, `matplotlib`, `numpy`, `scipy`, `Pillow`,
`PyYAML` và font DejaVu; dùng NumPy/Matplotlib tương thích của hệ thống:

```bash
python3 -m venv --system-site-packages /tmp/pstmo-cross5-pdf-venv
/tmp/pstmo-cross5-pdf-venv/bin/pip install reportlab
PYTHONNOUSERSITE=1 /tmp/pstmo-cross5-pdf-venv/bin/python tools/build_cross_aisle_report.py
```

Builder chỉ xuất PDF cuối khi đủ 125 kết quả ma trận chính và aggregate cohort
lặp đã audit đủ 250 trial/125 cặp. Cờ `--preview` dùng để kiểm tra bố cục của
những nhóm đã hoàn tất; preview không phải kết quả giao cuối.

Builder mặc định dùng lại 92 ảnh đã kiểm tra và dựng hình thống kê thứ 93 từ
aggregate mới; `--regenerate-figures` dùng khi cần dựng lại toàn bộ hình từ dữ
liệu. Mục lục và tham chiếu trang được xác định qua hai lượt dàn trang. Kiểm
tra dữ liệu, số hình, liên kết và render toàn bộ PDF:

```bash
/tmp/pstmo-cross5-pdf-venv/bin/pip install pymupdf
PYTHONNOUSERSITE=1 /tmp/pstmo-cross5-pdf-venv/bin/python tools/verify_cross_aisle_report.py
```

Sau khi xem các ảnh trong `tmp/pdfs/cross5_revision/`, có thể ghi nhận kết quả
kiểm tra trực quan bằng cờ `--accept-visual-review`. Cờ này từ chối xác nhận
nếu PDF đã thay đổi kể từ lần render.

## Giới hạn diễn giải

Map tĩnh và một robot mô phỏng. Ma trận năm phương án có một lần/tổ hợp; cohort
PSTMO–Constrained có năm lần lặp cố định nhưng chưa có nhiều seed. Chưa có vật
cản động, thay đổi tải, đo điện năng hay thử nghiệm phần cứng. Eκ là tích phân bình
phương độ cong, không phải điện năng. Hậu kiểm footprint trên PGM không đồng
nhất với bằng chứng tiếp xúc vật lý. Các hình 3D/tái dựng Bézier được ghi rõ
là minh họa khoa học; không được trình bày như ảnh chụp GUI.
