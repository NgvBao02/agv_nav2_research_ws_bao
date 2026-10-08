# Hồ sơ biên soạn báo cáo lý thuyết

Bản Word chính:
[CO_SO_LY_THUYET_TOAN_DIEN_DU_AN_AGV_PSTMO.docx](../CO_SO_LY_THUYET_TOAN_DIEN_DU_AN_AGV_PSTMO.docx).

Báo cáo đã kiểm tra gồm 62 trang, 15 chương, 4 phụ lục, 45 hình, 60 công thức
Word và 10 bảng.
Không có trang bìa riêng. Mục lục có liên kết tới các chương. Kiểu trang A4,
font và các kiểu tiêu đề được kế thừa từ `../PSTMO.docx`; bản gốc không bị sửa.
Các chương chính được nối theo dòng nội dung để hạn chế trang chỉ còn vài dòng.
Chương 1 và từng phụ lục bắt đầu trang mới.

## Phạm vi và bằng chứng

- Đối chiếu mã nguồn, mô hình và cấu hình trong workspace ngày 06/10/2026.
- Bao phủ nền tảng hình học, động học, mô hình 3D, cảm biến, định vị, ROS 2,
  costmap, planner, smoother, PSTMO, Adaptive Hybrid, RPP, mô phỏng và đánh giá.
- Phân biệt mô hình lý thuyết, cấu hình tĩnh, dữ liệu thực nghiệm lịch sử và
  những giả thiết phần cứng chưa được đo. Không khẳng định đã bao phủ mọi
  lý thuyết robot học hoặc bảo đảm vật lý liên tục từ phép lấy mẫu rời rạc.
- Không chạy lại Gazebo, benchmark hay unit test trong công việc biên soạn này.
- Hình CAD, Gazebo, RViz2 và đồ thị được lấy từ các hồ sơ thật sẵn có trong
  dự án; không tạo ảnh giả làm bằng chứng mô phỏng.
- Không thay đổi thuật toán, thông số điều hướng hoặc mô hình robot.

## Các tệp hỗ trợ

| Tệp | Mục đích |
| --- | --- |
| `noi_dung.md` | Nội dung có thể chỉnh sửa và tái dựng |
| `source_manifest.json` | Git HEAD và SHA-256 đầy đủ của nguồn đối chiếu |
| `figure_manifest.json` | 45 chú thích, đường dẫn và hash của hình gốc |
| `build_summary.json` | Thống kê và hash bản Word sau khi dựng |
| `qa_summary.json` | Kiểm tra cấu trúc, hình, số trang và bản dựng |

Số trang trong `qa_summary.json` được xác nhận bằng bản xuất kiểm tra, có thể
thay đổi nhẹ giữa các phiên bản Microsoft Word do font và máy in. Công thức là
OMML có thể chỉnh sửa. Bản xuất kiểm tra dùng LibreOffice đóng gói, không lưu
ngược lại DOCX. Chưa kiểm tra bằng Microsoft Word desktop. Trường đánh số trang
được giữ và `updateFields` được bật; mục lục liên kết không dùng số trang cache.

## Tái dựng và kiểm tra

Từ thư mục gốc dự án, dùng Python của runtime tài liệu đã cài, không dùng
Python hệ thống. Trong môi trường hiện tại:

```bash
TASK_RUNTIME=/home/linh-pham/.cache/codex-runtimes/codex-primary-runtime
"$TASK_RUNTIME/dependencies/python/bin/python3.12" tools/build_theory_docx.py
```

Sau khi dựng phải xuất lại PNG bằng `render_docx.py` của skill documents,
với `--emit_pdf --dpi 110`, rồi chạy:

```bash
"$TASK_RUNTIME/dependencies/python/bin/python3.12" tools/verify_theory_docx.py /duong/dan/thu/muc/render
```

Luôn kiểm tra trực quan tất cả các trang sau lần chỉnh sửa cuối; script không
thay thế bước này. Kết quả kiểm tra tự động đặt trạng thái trực quan về pending
để không kế thừa nhầm xác nhận từ phiên bản trước. Không đưa PDF/PNG kiểm tra
tạm vào tài liệu bàn giao trừ khi cần xuất PDF riêng.
