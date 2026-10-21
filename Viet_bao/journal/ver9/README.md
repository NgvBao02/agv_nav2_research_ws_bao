# Bản thảo journal PSTMO phiên bản 9

Hiệu đính khoa học phần II–III từ [ver8](../ver8/README.md), giữ nguyên kết
quả thực nghiệm. Không chạy mô phỏng mới, không thay mã ROS/config, không
ghi đè phiên bản hoặc dataset lịch sử.

- [Bản Word](PSTMO_JOURNAL_VER9_PHAN_II_III.docx).
- [Báo cáo phản biện và sửa trước–sau](REVISION_REPORT.md),
  [nhật ký có vị trí và nguồn](revision_log.json).
- [Nội dung đã sinh](manuscript.md), [số liệu bảng](table_values.json).
- [Hồ sơ nguồn](provenance.json), [đối chiếu source lịch sử](source_validation.json).
- [Kiểm tra chất lượng](quality_checks.json), [duyệt từng trang](visual_review.json),
  [quy cách biên tập](template_contract.md).
- [Script dựng](../../../tools/build_journal_sections_v9.py) và
  [script kiểm tra](../../../tools/verify_journal_sections_v9.py).

## Nội dung hiệu đính

21 đoạn được sửa có bằng chứng: bước conditioning, tính chính quy tại điểm
nối, phạm vi G², cách lấy mẫu độ cong/va chạm, vòng lặp gia tốc góc, hai chiều
dài cắt, tìm hình dạng phân cấp, cửa sổ thời gian chung, định nghĩa cmax và
quy hoạch động. Phần thử nghiệm phân biệt vận tốc xoay hướng RPP với giới
hạn vận tốc góc, RMSE theo chiều dài cung với trung bình theo thời gian,
trung bình chênh lệch với kiểm định Wilcoxon, hiệu chỉnh p với CI và trọng
số nhóm với thiên lệch do chỉ dùng cặp cùng đạt.

Giữ nguyên phân tích từng tuyến R01–R05, III-G và các ca thất bại. Không
biến lợi ích quan sát được thành khẳng định nhân quả, tối ưu toàn cục,
an toàn liên tục, tiết kiệm điện năng hoặc ưu thế tỷ lệ thành công. Các
giới hạn chưa giải quyết được ghi trong báo cáo: chưa có ablation cùng
phiên bản, robot thực, vật cản động hoặc hồ sơ phụ thuộc binary đầy đủ.

23 bảng/696 ô, 18 hình, 12 đối tượng Office Math/chín công thức và bảy tài
liệu tham khảo được bảo toàn. Các phần tử không sửa giữ nguyên XML; mọi
thành phần ZIP ngoài document.xml và core.xml giữ nguyên byte.

## Nguồn và kiểm chứng

Bộ dựng kiểm tra các hash nguồn trong provenance ver6–8, đồng thời so sánh
mười tệp C++/config/mô hình với commit của cohort lặp. Đây không phải xác
nhận mọi binary của các lượt lịch sử. Không lấy bản Gazebo đang cài trên
máy để gán ngược phiên bản cho thực nghiệm.

Bộ kiểm tra đối chiếu toàn bộ ô Word với table_values.json, bảo toàn văn
bản ngoài 21 đoạn, tái kiểm tra 210 ô mô tả từ 125 bản ghi và trung bình
chênh lệch của 25 nhóm từ bảng đã lưu. Không chạy lại bootstrap, Wilcoxon
hoặc Holm. CI/p được giữ nguyên qua đối chiếu bảng và hồ sơ nguồn.

## Tái tạo

Nguồn biên tập nằm trong script dựng. manuscript.md là đầu ra, không phải
nơi sửa để tái tạo. Lưu riêng chỉnh sửa Word thủ công trước khi chạy lại.
Script chỉ nhập helper ánh xạ/hash từ ver7, không gọi pipeline dựng cũ.

Dùng Python có lxml, python-docx và PyMuPDF. Phiên này dùng Python 3.12.14
của runtime Documents, môi trường tạm `/tmp/pstmo-v9-env`, PyMuPDF 1.26.7,
LibreOffice đóng gói 25.2 và bộ font Times New Roman tại
`/home/linh-pham/.local/share/fonts/pstmo-times`. Môi trường `/tmp` cần tạo
lại nếu chuyển phiên/máy.

```bash
PYTHONNOUSERSITE=1 /tmp/pstmo-v9-env/bin/python tools/build_journal_sections_v9.py
# Render bằng render_docx.py của Documents, --emit_pdf, vào thư mục QA mới.
# FONTCONFIG_FILE phải trỏ tới cấu hình có thư mục font Times New Roman.
PYTHONNOUSERSITE=1 /tmp/pstmo-v9-env/bin/python tools/verify_journal_sections_v9.py --render-dir /tmp/pstmo-v9-final-compact
# Xem TẤT CẢ PNG, ghi nhận xét và hash bản vừa xem vào visual_review.json.
PYTHONNOUSERSITE=1 /tmp/pstmo-v9-env/bin/python tools/verify_journal_sections_v9.py --render-dir /tmp/pstmo-v9-final-compact --visual-review Viet_bao/journal/ver9/visual_review.json
python3 tools/check_repository_docs.py
git diff --check
```

File duyệt có docx_sha256, pdf_sha256, danh sách page_png_sha256 theo thứ
tự và pages gồm page/accepted/notes cho từng trang. Bộ kiểm tra từ chối
hash không khớp hoặc thiếu trang; không có cờ tự động tuyên bố đã xem.
Mỗi lần dựng lại đưa quality_checks về trạng thái chờ xác minh.

Đã duyệt đủ 15 trang ngày 22/10/2026. Bản render đầu bị thay font đã được
loại; bản 16 trang có hai tài liệu tham khảo lẻ được thay bằng bản cuối sau
khi rút gọn 69 từ trong các đoạn mới, không giảm font/lề hoặc đổi thứ tự
khối. Hai lần dựng cho cùng SHA-256 DOCX và provenance.

PDF/PNG trong `/tmp` chỉ phục vụ QA, không phải đầu ra bàn giao. Trạng thái
duyệt và số trang chính thức nằm trong quality_checks.json. Chưa cần chạy
ROS test vì không thay mã runtime. Đã cập nhật mục tài liệu ở root README;
results/README và docs/agent không đổi vì dữ liệu, nguồn sự thật và quy trình
chung của repo không đổi. Trước khi nộp, cần ghép toàn bài, đánh số lại tài
liệu tham khảo và đối chiếu mẫu tạp chí đích.
