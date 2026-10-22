# Bản thảo journal PSTMO phiên bản 11

[Bản Word đầy đủ](PSTMO_JOURNAL_VER11_FULL_MANUSCRIPT.docx) ghép phần II–III
của [ver9](../ver9/README.md) với tên bài, Tóm tắt và Kết luận của
[ver10](../ver10/README.md). Viết lại và mở rộng Introduction, thêm Abstract
tiếng Anh cùng các đoạn nền tảng/thảo luận vào phần II–III.

Tên bài: **Hậu xử lý đường đi có xét tính khả thi chuyển động cho robot vi sai
trong môi trường kho**. Tên tiếng Anh: **Motion-Feasibility-Aware Path
Post-Processing for Differential-Drive Robots in Warehouse Environments**.

## Phạm vi và tài liệu tham khảo

- Ánh xạ đủ **74/74 mục corpus** vào nội dung có trích dẫn, với **80 mục
  trong danh mục cuối**. Sáu nguồn ngoài corpus được kế thừa từ ver9/ver10:
  Choi 2014, tài liệu Constrained Jazzy, mã Smoother Server Jazzy, Efron,
  Wilcoxon và Holm. Hai tài liệu Constrained Rolling/Jazzy được giữ riêng vì
  nhận diện hai phiên bản, không tính như hai phương pháp nghiên cứu độc lập.
- Giữ nguyên toàn bộ văn bản khoa học phần II–III, ngoài đổi số trích dẫn;
  thêm bảy đoạn theo các điểm chèn có kiểm tra. Giữ đúng 23 bảng/696 ô,
  18 hình, 12 đối tượng Office Math và chín số công thức.
- Không sửa ROS/config, chạy mô phỏng mới hay phân tích thống kê lại.
  Không ghi đè ver9/ver10 hoặc bất kỳ tệp nào trong REFERENCES.
- Đã sửa nhận diện mục 016 theo PDF thực tế; mục 026 là slide đi kèm chứ
  không phải toàn văn bài journal. URL sai, phiên bản preprint, tên tác giả
  và khác biệt Rolling/Jazzy được ghi trong [hồ sơ biên tập](EDITORIAL_REPORT.md).

## Nguồn biên tập và kiểm tra

- [front_conclusion.md](front_conclusion.md): tên bài, hai tóm tắt và kết luận.
- [introduction.md](introduction.md): Introduction mở rộng.
- [insertions.md](insertions.md): các đoạn chèn vào thân ver9; điểm chèn phải
  khớp duy nhất để tránh ghép sai vị trí.
- [references.json](references.json): thông tin trích dẫn, nguồn, hash, trang
  đọc có mục tiêu và các ghi chú sửa metadata.
- [REFERENCE_COVERAGE.md](REFERENCE_COVERAGE.md) và
  [citation_map.json](citation_map.json): ánh xạ từng nguồn đến số trích dẫn,
  đề mục và nguyên đoạn sử dụng.
- [manuscript_numbered.md](manuscript_numbered.md): văn bản trích từ Word;
  không phải nguồn biên tập hoặc bản thay thế hình/công thức.
- [revision_log.json](revision_log.json), [provenance.json](provenance.json),
  [quality_checks.json](quality_checks.json), [visual_review.json](visual_review.json)
  và [template_contract.md](template_contract.md): truy vết thay đổi và QA.
- [Script dựng/kiểm tra](../../../tools/build_journal_full_v11.py) dùng helper
  XML của [script ver10](../../../tools/build_journal_sections_v10.py), không
  gọi pipeline tạo ver10 hay các phiên bản lịch sử.

## Tái tạo

Dùng Python runtime Documents có lxml, python-docx và PyMuPDF. Phiên dựng
dùng môi trường tạm tạo từ Python runtime, PyMuPDF 1.26.7 và LibreOffice
25.2 đóng gói. Cấu hình FONTCONFIG_FILE phải trỏ tới Times New Roman trong
`/home/linh-pham/.local/share/fonts/pstmo-times`; thư mục tạm cần tạo lại khi
đổi phiên. Không dùng LibreOffice lưu đè Word nguồn.

```bash
# Thay /path/to/runtime/python bằng Python runtime Documents.
/path/to/runtime/python tools/build_journal_full_v11.py --replace-generated
/path/to/runtime/python tools/build_journal_full_v11.py --check
# Render bằng render_docx.py --emit_pdf vào thư mục QA mới.
# Xem tất cả PNG, ghi nhận xét từng trang và hash vào visual_review.json.
/path/to/runtime/python tools/build_journal_full_v11.py --check --render-dir /path/to/new/render
python3 tools/check_repository_docs.py
git diff --check
```

Lần đầu chưa có Word thì bỏ `--replace-generated`. Cờ này chỉ cho ghi lại
đúng bản đã sinh có hash khớp; từ chối nếu Word được sửa tay. Cờ `--init`
chỉ phục vụ nhập nguồn lần đầu trước biên tập; không chạy lại trên bản đã
hiệu đính. Mỗi lần dựng đưa QA về trạng thái chờ duyệt. File duyệt cũ không
hợp lệ nếu hash DOCX/PDF/PNG thay đổi. PDF/PNG chỉ phục vụ QA, không bàn giao.

Đã render và duyệt đủ **21 trang** ngày 23/10/2026. Bản cuối giữ font thân
Times New Roman; phép render công thức và một số ký hiệu dùng font dự
phòng đã được xem trực tiếp. Các mục tham khảo được căn trái để URL dài
không gây giãn chữ. Trạng thái chính thức và hash nằm trong hồ sơ QA.

## Giới hạn trước khi nộp

Đây là bản thảo đầy đủ để trao đổi với giảng viên, chưa xác nhận phù hợp một
tạp chí đích. Cần chốt ngôn ngữ, giới hạn độ dài, mẫu, tác giả/cơ quan,
funding, xung đột lợi ích và yêu cầu chia sẻ dữ liệu. Không tự điền các
thông tin chưa có. Nếu giảng viên sửa một bản Word khác ngoài workspace,
cần đối chiếu bản đó trước khi thay thế.

Corpus được đọc có mục tiêu theo các luận điểm cần dẫn, không tuyên bố đọc
toàn bộ 70 PDF. Những nguồn ở hệ robot khác chỉ dùng cho phân loại/giới hạn,
không tạo bằng chứng rằng PSTMO vượt trội chúng. Giữ nguyên giới hạn một
bản đồ tĩnh, một robot mô phỏng, cấu hình RPP cố định và thiếu ablation.
Các CI/p vẫn thuộc [cohort lặp](../../../results/pstmo_journal_repeatability_20261011/README.md),
không gộp với ma trận 125 lượt.

Đã cập nhật chỉ mục root README. Không đổi results/README, AGENTS hoặc
docs/agent vì không đổi dataset, nguồn sự thật runtime hay quy trình chung.
Không cần ROS test cho thay đổi bản thảo và script xuất Word này.
