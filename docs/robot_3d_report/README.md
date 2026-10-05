# Hồ sơ mô hình 3D robot

Báo cáo chính: `../BAO_CAO_MO_HINH_3D_STEP_URDF_GAZEBO_RVIZ2.pdf`.
69 trang, 69 hình; dữ liệu ghi ngày 05/10/2026.

## Phạm vi

- Đọc hình học thật từ `file_3D/Xe.step`: 542 solid và 5.287 mặt ngoài solid.
- Đối chiếu URDF, STL, SDF, Nav2, profile phần cứng và EKF.
- Ảnh chụp cửa sổ Gazebo/RViz2 thật; ảnh dựng CAD và biểu đồ ghi nhãn riêng.
- 7 dạng chuyển động, mỗi dạng 3 lần lặp: 21 lượt mô phỏng mới.
- Một lượt Nav2/ThetaStar thành công trong `warehouse_cross_aisles`.
- Không sửa tệp cơ khí hoặc cấu hình nguồn; không thử robot thật.

## Kết quả quan trọng

- Tâm phần lốp STEP: **230,8 mm**, so với **254,8 mm** trong URDF/SDF.
- Gazebo DiffDrive dùng **283,4 mm** làm khoảng cách hiệu dụng, không phải kích thước cơ khí.
- Chi tiết mặt rời STEP làm bao bánh lớn hơn phần solid lốp; không bỏ chúng khi kiểm tra hình học.
- Khối lượng 5 kg và COM là **khai báo/tính từ SDF**, không phải phép cân.
- Từ 0,30 m/s, sau lệnh zero: quãng dừng trung bình **100,78 mm**, SD **0,51 mm**, n = 3.
- Chạy cung: sai lệch vị trí odom-ground truth cuối khoảng **17,1 mm**.
- Minh họa kho: action SUCCEEDED; sai lệch GT-goal khoảng **56,46 mm**, với quy ước map/world trùng nhau.
- Lần chạy kho khởi tạo đã abort do BT chọn mặc định `GridBased` không tồn tại; dữ liệu lần đó được giữ. Lượt thành công chọn `ThetaStar` qua `/planner_selector`.

## Tệp dữ liệu

| Đường dẫn | Ý nghĩa |
| --- | --- |
| `figure_page_index.json` | Nối số hình, trang PDF, chú thích và PNG gốc |
| `figures/` | Dựng CAD/STL, sơ đồ kỹ thuật, biểu đồ từ dữ liệu |
| `screenshots/` | Ảnh chụp cửa sổ ứng dụng, không chụp toàn desktop |
| `capture_manifest.jsonl` | Camera, cấu hình và thời điểm chụp; nếu chụp lại cùng tên, bản ghi cuối là bản PNG hiện tại |
| `rviz/` | Cấu hình RViz dùng để chụp ảnh |
| `geometry.json` | Phép đo solid, mesh, kích thước và hash nguồn ban đầu |
| `step_topology.json` | Bao solid riêng và các mặt nằm ngoài solid |
| `step_solids.csv` | Tất cả 542 solid: volume, area, bounding box, centroid và triangle count |
| `cad_product_records.txt` | 168 PRODUCT record; không phải BOM đã xác nhận số lượng |
| `cad_audit.json` | Phép quy trục, nhóm bộ phận và sai khác bánh |
| `registration.json` | Phép thử quy trục bằng cloud sampling; không phải kiểm định dung sai |
| `mass_properties.json` | COM và inertia khai báo/tổng hợp từ SDF |
| `motion/trials.json` | Lệnh, thời điểm phát/zero, pose reset của từng lượt |
| `motion/metrics.csv` | Chỉ số chi tiết của 21 lượt |
| `motion/summary.json` | Trung bình và SD mẫu theo 7 ca |
| `motion/rates.json` | Thống kê nhịp timestamp và số mẫu |
| `warehouse_demo_attempt1.json` | Lượt abort do chưa chọn planner hợp lệ |
| `warehouse_demo.json` | Ground truth và plans của lượt thành công |
| `warehouse_metrics.json` | Các chỉ số minh họa kho |
| `provenance.json` | Commit, môi trường, SHA-256 nguồn và mã tạo báo cáo |
| `qa/verification.json` | Kết quả kiểm tra PDF, số trang, số hình và hash PDF |
| `qa/pages/` | Render Poppler của mọi trang để kiểm tra bố cục |

Các tệp NPZ là cache lưới tam giác từ CAD/STL. `cad_aligned.npz` phục vụ dựng hình đã quy trục; không thay thế STEP gốc hoặc sửa mô hình nguồn.

## Các luồng thô E1

`motion/truth.csv`, `odom.csv`, `joints.csv`, `imu.csv`, `scan.csv`, `commands.csv`.

- `t`: giây của **thời gian mô phỏng**, không phải đồng hồ thực.
- `trial`: ví dụ `arc_left_2`; `reset` và `warmup` không phải lượt đo chính.
- Tọa độ thô GT ở frame world; odom có thể tiếp tục tích lũy sau teleport.
- Phân tích đưa mỗi luồng pose về pose đầu lượt rồi nội suy timestamp.
- Scan CSV chỉ lưu thống kê; `scan_snapshot.json` giữ một bản tin đầy đủ.
- Chưa ghi lực contact, dòng điện, điện áp hoặc dữ liệu phần cứng.

## Mã tái lập

Các script nằm trong `tools/` tại workspace:

1. `robot3d_extract.py`: đọc STEP/STL, đo và dựng hình.
2. `robot3d_step_topology.py`: bổ sung mặt không thuộc solid.
3. `robot3d_extract.py --render-only`: dựng lại toàn STEP, có cả mặt ngoài solid.
4. `robot3d_cad_audit.py`: hình nhóm bộ phận, phép đối chiếu.
5. `robot3d_capture.py gazebo` và `robot3d_capture.py rviz`: ảnh cửa sổ ứng dụng.
6. `robot3d_motion_test.py`: 21 phép thử chỉ trong phiên Gazebo cô lập.
7. `robot3d_warehouse.py`: ảnh kho và action NavigateToPose.
8. `robot3d_analysis.py`, `robot3d_supplement.py`: chỉ số và đồ thị.
9. `build_robot3d_report.py`: PDF hai lượt, mục lục và bookmark.
10. `verify_robot3d_report.py`: kiểm tra bằng text bounds, liên kết dữ liệu và render toàn bộ PDF.

Môi trường đã dùng: `/tmp/robot3d-report-venv`, Python 3.12, numpy 1.26.4,
cadquery-ocp 8.0.1.0.0, VTK 9.6.2, trimesh 5.1.1, matplotlib, scipy,
ReportLab 5.0.1, PyMuPDF 1.28.2, PyYAML, Pillow. Ảnh cửa sổ dùng xwd/ffmpeg;
PDF QA dùng pdftoppm. Thư mục `/tmp` có thể mất sau reboot; phải tạo lại venv nếu cần.

Các script ROS dùng Python hệ thống sau khi source ROS 2 Jazzy và install/setup.bash.
E1 chỉ chạy với ROS_DOMAIN_ID=183, GZ_PARTITION=robot3d_report.
E2 dùng domain 184, GZ_PARTITION=robot3d_warehouse.
Đặt PYTHONNOUSERSITE=1 để tránh thư viện Python user không tương thích.

**Chạy lại có thể ghi đè dữ liệu mới tạo có cùng tên. Sao lưu bộ kết quả trước.**
Không chạy bộ phát lệnh trên robot thật; không đổi domain để nối vào hệ điều khiển thật.

## Giới hạn kết luận

Đây là hồ sơ hình học và kiểm tra mô phỏng. Chưa chứng minh cơ khí STEP là revision
đã lắp thực, chưa xác nhận tải trọng, độ bền, an toàn nguồn điện hoặc hiệu năng robot thật.
Ba lượt cùng một cấu hình không thay thế thử nghiệm nhiều tải/sàn/seed độc lập.
21 lượt này không thuộc 125 lượt benchmark PSTMO của báo cáo năm quỹ đạo trước.
