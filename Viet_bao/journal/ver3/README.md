# Bản thảo journal PSTMO phiên bản 3

Thư mục chứa **bản viết đầy đủ bằng tiếng Việt của phần II và III**, không phải
dàn ý. Các phiên bản dàn ý cũ và dữ liệu thực nghiệm không bị ghi đè.

- [Bản Word phần II và III](PSTMO_JOURNAL_VER3_PHAN_II_III.docx).
- [Nguồn nội dung](manuscript.md), 19 hình khoa học và 20 bảng chỉnh sửa được.
- [Nguồn hình PNG 450 dpi và SVG](figures/).
- [Hash nguồn và ánh xạ hình](provenance.json).
- [Kết quả kiểm tra artifact](quality_checks.json).
- [Script dựng bản thảo](../../../tools/build_journal_sections_v3.py).
- [Script kiểm tra](../../../tools/verify_journal_sections_v3.py).

Văn bản nộp bài dùng ký hiệu khoa học thay cho tên trường trong mã nguồn:
`Eκ,ref` cho năng lượng độ cong của đường tham chiếu và `Eκ,exec` cho năng
lượng độ cong của quỹ đạo thực. Tên biến, tên chế độ nội bộ, mã phiên bản dài
và mã trạng thái của log không xuất hiện trong phần trình bày chính. Thông tin
định danh chi tiết vẫn được giữ trong dữ liệu nguồn và `provenance.json` để
phục vụ tái lập kết quả.

## Bố cục và phạm vi

Bản render kiểm tra gồm 15 trang A4 hai cột. Phần II chiếm khoảng 2,2 trang;
phần III khoảng 12 trang, tiếp theo là các tài liệu tham khảo dùng cho hai
phần. III.A tương ứng 3.1; III.B–F tương ứng năm tuyến 3.2–3.6; III.G–I tương
ứng tổng hợp, kiểm chứng lặp và thảo luận 3.7–3.9. Cách đánh số chữ cái tuân
theo hệ phân cấp của mẫu IEEE.

Nguồn bố cục là
[IEEE Xplore conference-template-a4.docx](<../../final_bao_ICEEIS/IEEE Xplore conference-template-a4.docx>).
Giữ khổ 595,3 × 841,9 pt, lề trên/dưới 62,35 pt, lề trái/phải 45,35 pt,
khoảng cột 18 pt, Times New Roman 10 pt, bảng/chú thích 8 pt, công thức
Office Math chỉnh sửa được. Mẫu Strict OOXML được chuẩn hóa trong bộ nhớ;
tệp mẫu gốc không thay đổi. Hình thuật toán và footprint là sơ đồ minh họa,
còn hình đường/biểu đồ lấy từ dữ liệu có thật; không sử dụng ảnh AI.

**Đây là mẫu IEEE conference A4 đang có trong workspace, không phải xác nhận
đúng template của một journal cụ thể.** Khi chọn journal cần đối chiếu mẫu,
giới hạn trang, ngôn ngữ và yêu cầu của tạp chí. Bản này chỉ có II/III; chưa
gồm tóm tắt, giới thiệu, kết luận và danh mục 70–80 tài liệu của toàn bài.
Bảy trích dẫn cục bộ cần được đánh số lại khi ghép toàn bản thảo. Không có
cam kết bài sẽ được nhận đăng.

## Nguồn dữ liệu và giới hạn diễn giải

- [Ma trận 125 lượt](../../../docs/warehouse_cross_aisles_5_routes/README.md):
  một lượt/tổ hợp, 123 đạt; R01 là dữ liệu lịch sử. Dùng JSON thực thi cho cả
  hình và chỉ số, không trộn snapshot hình học RViz.
- [Cohort lặp 250 lượt](../../../results/pstmo_journal_repeatability_20261011/README.md):
  243 đạt, 125 cặp cùng Raw hash, 118 cặp cùng đạt và 25 khối tuyến–planner.
  Chỉ số liên tục có điều kiện trên cặp thành công; failure giữ trong mẫu số.
- Trung bình gộp 118 cặp khác với hiệu ứng trung bình đều 25 khối. Bootstrap
  khối là phân tích có điều kiện, không chứng minh độc lập giữa các tuyến
  chung bản đồ hay độ bền vững nhiều seed.
- Constrained chạy với `w_curve=0`; không tuyên bố hai phương án có cùng
  bảo đảm quét footprint nội bộ. G² chỉ áp dụng đoạn tịnh tiến chính quy,
  không bảo đảm toàn pose trajectory hoặc vết thực thi G².
- `smoothing_time_s` lấy từ `SmoothPath.smoothing_duration`. Theo nguồn Nav2
  Jazzy, đây là hiệu thời gian node; khi `use_sim_time` bật, không được gọi
  là CPU-time hoặc wall-clock benchmark. Bản thảo làm rõ điểm này dù một
  số diễn giải trong báo cáo lịch sử gọi chung là thời gian xử lý CPU.
- Khoảng hở giảm, sai số đích và thời lượng SmoothPath tăng được báo cáo
  cùng các kết quả thuận lợi; không sửa hoặc chạy lại trial xấu để thay số.

## Tái tạo và kiểm tra

Từ root workspace, dùng môi trường Python riêng có các gói dưới đây. Bộ dựng
chỉ đọc dataset, ghi đè artifact **trong ver3** do chính nó sinh; sao lưu nếu
đã chỉnh Word thủ công. Không khởi động ROS/Gazebo hay điều khiển robot.

```bash
python3 -m venv --system-site-packages /tmp/pstmo-journal-v3-env
PYTHONNOUSERSITE=1 /tmp/pstmo-journal-v3-env/bin/pip install python-docx pymupdf pillow pdf2image lxml latex2mathml mathml2omml
PYTHONNOUSERSITE=1 /tmp/pstmo-journal-v3-env/bin/python tools/build_journal_sections_v3.py
```

Cần NumPy, Matplotlib, PyYAML tương thích, Times New Roman và DejaVu Math
TeX Gyre. Các PNG được tạo với kích thước in một cột, giữ cỡ nhãn khi chèn
vào Word. Nếu thiếu font, không coi render thay thế là kiểm tra font chuẩn.

Dùng `render_docx.py` của skill Documents để xuất PDF/PNG vào thư mục tạm,
xem **toàn bộ** trang, rồi kiểm tra (thay đường dẫn bằng thư mục render):

Trình render dùng hồ sơ LibreOffice tạm, nên font cài riêng cho người dùng
có thể không được tìm thấy. Khi đó truyền `FONTCONFIG_FILE` trỏ tới cấu hình
fontconfig có đường dẫn tuyệt đối tới thư mục Times New Roman; không chấp nhận
bản render dùng Liberation Serif thay thế. Lượt QA hiện tại dùng thư mục
`/tmp/pstmo-v3/final-font`; hash PDF và từng trang được lưu trong kết quả kiểm tra.

```bash
PYTHONNOUSERSITE=1 /tmp/pstmo-journal-v3-env/bin/python tools/verify_journal_sections_v3.py --render-dir /tmp/pstmo-v3/final-font
PYTHONNOUSERSITE=1 /tmp/pstmo-journal-v3-env/bin/python tools/verify_journal_sections_v3.py --render-dir /tmp/pstmo-v3/final-font --accept-visual-review
python3 tools/check_repository_docs.py
git diff --check
```

Chỉ dùng `--accept-visual-review` sau khi đã xem hết ảnh của đúng bản PDF.
Thay đổi nội dung/Word cần render và kiểm tra lại; số trang có thể dịch
chuyển theo Word/LibreOffice và bộ font của máy nhận. PDF/PNG QA không phải
artifact bàn giao, và không cần chạy toàn bộ ROS test cho thay đổi báo cáo này.
