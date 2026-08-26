# Bàn giao bài báo PSTMO ver2

## Bản thảo chính

- `ICEEIS_2026_PSTMO_ver2_tieng_Viet.docx`: bản có thể chỉnh sửa.
- `ICEEIS_2026_PSTMO_ver2_tieng_Viet.pdf`: bản kiểm tra dàn trang A4, 6 trang.
- `NHAN_XET_DAN_Y_VER2.md`: phân tích nhận xét của cô Lý và phần làm rõ của Thảo.
- `KIEM_TRA_BAO_CAO_VER2.md`: biên bản đối chiếu logic, mã triển khai, dữ liệu, tài liệu tham khảo và dàn trang của toàn văn.
- `dàn ý ver2 hiệu chỉnh.docx` và `dàn ý ver2 hiệu chỉnh.html`: dàn ý đã sửa logic trước khi viết toàn văn.

## Hình ảnh và Draw.io

Bản thảo giữ đúng 8 hình xuất hiện trong PDF ver1:

- Hình 1–8 lấy nguyên tệp từ `ver1/ICEEIS_2026_PSTMO_drawio_assets/image1.png` đến `image8.png`.
- Các bản sao dùng trong bài nằm tại `ver2/assets/ver1_hinh_1.png` đến `ver1_hinh_8.png`; nội dung tệp được giữ nguyên.
- `image9.png` và `image10.png` có trong thư mục tài sản của ver1 nhưng không xuất hiện trong PDF ver1, nên không được đưa vào bài ver2.

Các nguồn Draw.io:

- `PSTMO_ver2_tat_ca_hinh.drawio`: 8 trang, mỗi trang chứa đúng hình đã nhúng trong DOCX.
- `PSTMO_ver2_hinh_ky_thuat_editable.drawio`: bộ kỹ thuật Hình 1–5 có thể sửa theo đối tượng vector.
- `PSTMO_ver2_hinh_1_5_goc_tu_ver1/`: năm Draw.io gốc Hình 1–5 được sao chép nguyên từ ver1.

Hình 6–8 trong PDF ver1 không có bộ vector riêng khớp hoàn toàn với ảnh nhúng. Vì vậy, `PSTMO_ver2_tat_ca_hinh.drawio` giữ nguyên các ảnh này theo từng trang Draw.io, không thay chúng bằng bộ `section_v_redesigned_figures`.

## Các điểm logic đã khóa trong ver2

- Related work được đưa vào Introduction theo mạch: planner → nhu cầu smoothing → ba nhóm giải pháp → khoảng trống → đóng góp PSTMO.
- PSTMO được mô tả là bộ hậu xử lý `nav_msgs/Path`, không phải trajectory optimizer có tham số thời gian.
- Profile vận tốc nội bộ chỉ dùng để kiểm tra vận tốc, gia tốc dọc/ngang và gia tốc góc của ứng viên; nó không biến đầu ra thành trajectory theo thời gian.
- G² chỉ được dùng để kết luận liên tục độ cong tại mối nối; bài không suy diễn jerk theo thời gian bằng không.
- `Eκ=∫κ²ds` được gọi là chỉ số hình học, đơn vị m⁻¹, không gọi là năng lượng.
- Chỉ tuyên bố bảo toàn vị trí đầu, vị trí đích và orientation tại đích; không tuyên bố bảo toàn orientation đầu.
- Raw là mốc đầu vào, không phải smoother. Simple, Savitzky–Golay và Constrained đều có nguồn chính thức [15]–[17].
- Cấu hình Constrained có `w_curve=w_dist=0`, nên bài không diễn giải như thể phương pháp này đã được hiệu chỉnh để tối thiểu trực tiếp `Eκ`.
- 15 tổ hợp bản đồ–planner là 15 nhóm ghép cặp, mỗi tổ hợp chạy một lần; kết quả là thống kê mô tả, không dùng để lập khoảng tin cậy.
- Kết luận nêu đúng đánh đổi: PSTMO giảm `Eκ` và không tăng chiều dài trung bình trong tập thử, nhưng xử lý chậm hơn các đối chứng.

## Kiểm tra cuối

- PDF: A4, 6 trang, nhỏ hơn giới hạn 8 trang.
- DOCX: kiểm tra ZIP/OOXML thành công.
- Draw.io: toàn bộ tệp XML đọc được.
- Mã phương pháp: 14/14 kiểm thử của gói `adaptive_pivot_g2` đạt khi nạp môi trường ROS 2 Jazzy.
- Trích dẫn: đủ và đúng thứ tự [1]–[23].
- PDF tham khảo: 18 tệp hợp lệ; Yang–Sukkarieh [9] và Bu et al. [10] chưa có tệp nội bộ hợp lệ. Tệp mang tên [9] nhưng chứa sai/thiếu nội dung đã được loại; đường dẫn DOI/IEEE được ghi trong `REFERENCES/TAI_LIEU_THAM_KHAO_VER2.md`.

## Tái tạo bản thảo

```bash
python3 tools/generate_iceeis_vietnamese_paper_ver2.py
soffice --headless --convert-to pdf --outdir final_bao_ICEEIS/ver2 final_bao_ICEEIS/ver2/ICEEIS_2026_PSTMO_ver2_tieng_Viet.docx
python3 tools/create_ver2_drawio.py
```
