# Bản thảo journal PSTMO phiên bản 10

[Bản Word riêng](PSTMO_JOURNAL_VER10_ABSTRACT_INTRO_CONCLUSION.docx) gồm tên
bài tiếng Việt/Anh, Tóm tắt tiếng Việt, từ khóa, Phần I Giới thiệu, Phần IV
Kết luận và tài liệu tham khảo được trích trong phần này. **Không chứa nội
dung Phần II–III**; khi ghép bài, dùng [ver9](../ver9/README.md) cho hai phần
đó. Không sửa bản Word ver9, mã ROS, cấu hình hay dữ liệu thực nghiệm.

Tên bài: **Hậu xử lý đường đi có xét tính khả thi chuyển động cho robot vi sai
trong môi trường kho**. Tên tiếng Anh: **Motion-Feasibility-Aware Path
Post-Processing for Differential-Drive Robots in Warehouse Environments**.

## Nguồn biên tập và kiểm chứng

- [manuscript.md](manuscript.md) là nguồn để sửa nội dung; khóa `[Rxxx]` được
  đánh số theo lần xuất hiện khi dựng, không sửa số trích dẫn bằng tay.
- [references.json](references.json) chứa 21 mục, vị trí bằng chứng trong
  PDF, nguồn chính thức và ghi chú đối chiếu metadata.
- [manuscript_numbered.md](manuscript_numbered.md) là văn bản đầu ra đã đánh
  số, không phải nguồn biên tập.
- [CLAIM_EVIDENCE.md](CLAIM_EVIDENCE.md) ghi phạm vi các khẳng định, đối
  chiếu số liệu và phần việc còn thiếu trước khi nộp.
- [template_contract.md](template_contract.md), [provenance.json](provenance.json),
  [quality_checks.json](quality_checks.json), [visual_review.json](visual_review.json)
  ghi nguồn định dạng, hash và kiểm chứng bản bàn giao.
- [Script dựng và kiểm tra](../../../tools/build_journal_sections_v10.py).

Số liệu lấy từ [bảng đã kiểm chứng ver9](../ver9/table_values.json), đối chiếu
với [cohort lặp](../../../results/pstmo_journal_repeatability_20261011/README.md).
Không chạy lại mô phỏng, bootstrap, Wilcoxon hoặc Holm. Thời gian, RMSE và
các đánh đổi dùng trung bình đều chênh lệch của 25 nhóm trên 118 cặp cùng
đạt, không lấy hiệu hai trung bình gộp hoặc trộn hai bộ dữ liệu.

## Danh mục tài liệu và giới hạn bàn giao

21 nguồn của bản này gồm 19 PDF trong REFERENCES và hai nguồn chính thức
bổ sung qua web: Choi 2014, Nav2 Jazzy Constrained Smoother. Các trang chính
thức khác được dùng đối chiếu metadata. Số trang trong references.json là
vị trí các đoạn được đọc có mục tiêu, **không phải cam kết đọc toàn bộ từng
bài**. Ngân hàng 74 mục không đồng nghĩa 74 trích dẫn của manuscript.

Đối chiếu với bảy mục hiện có trong ver9, hai mục trùng là RPP và Constrained
Smoother; tổng hợp hiện có 26 nguồn khác nhau. **Chưa đạt yêu cầu 50–100 tài
liệu cho toàn bài.** Cần bổ sung nguồn thực sự hỗ trợ mô hình, kiểm tra khả
thi, thuật toán và phương pháp đánh giá trong toàn bản thảo, rồi khử trùng
và đánh số lại khi ghép. Không tăng số lượng bằng các mục không được trích.

Đây là bản nội dung để ghép và trao đổi với giảng viên, không phải xác nhận
đã đáp ứng mẫu hay tiêu chí nhận đăng của một tạp chí cụ thể. Cần chọn tạp
chí, kiểm tra giới hạn Abstract/ngôn ngữ, thông tin tác giả và mẫu cuối.
Tóm tắt hiện bằng tiếng Việt; chưa tạo bản dịch Abstract tiếng Anh.
Các tham chiếu tới Phần II–III là cấu trúc của bài sẽ ghép, không phải phần
nội dung bị thiếu do lỗi xuất Word.

## Tái tạo và duyệt

Dùng Python của runtime Documents có lxml, python-docx, PyMuPDF; phiên này
dùng `/tmp/pstmo-v10-env` tạo từ Python runtime, PyMuPDF 1.26.7. LibreOffice
đóng gói nằm trong dependencies/bin/override. Renderer là `render_docx.py`
của skill Documents; không dùng LibreOffice lưu lại Word làm nguồn.
Times New Roman tại `/home/linh-pham/.local/share/fonts/pstmo-times` được
khai báo bằng FONTCONFIG_FILE để renderer không thay font khi dùng profile
tạm. Những đường dẫn `/tmp` chỉ là môi trường phiên, cần tạo lại nếu mất.

```bash
PYTHONNOUSERSITE=1 /tmp/pstmo-v10-env/bin/python tools/build_journal_sections_v10.py --replace-generated
PYTHONNOUSERSITE=1 /tmp/pstmo-v10-env/bin/python tools/build_journal_sections_v10.py --check
# Render DOCX vào thư mục QA mới với render_docx.py --emit_pdf.
# Xem mọi PNG, ghi nhận xét và hash DOCX/PDF/PNG vào visual_review.json.
PYTHONNOUSERSITE=1 /tmp/pstmo-v10-env/bin/python tools/build_journal_sections_v10.py --check --render-dir /path/to/new/render
python3 tools/check_repository_docs.py
git diff --check
```

Lần đầu chưa có Word thì bỏ `--replace-generated`. Cờ này chỉ cho phép
ghi lại Word có hash khớp bản đã sinh, từ chối nếu phát hiện chỉnh sửa tay.
Lưu bản chỉnh tay riêng trước khi tái tạo. Mỗi lần dựng đưa quality_checks
về trạng thái chờ duyệt; file visual_review cũ bị từ chối nếu hash thay đổi.
Không dùng số trang/nhận xét cũ cho một bản render mới chưa xem.

Thiết kế giữ A4, font 10 pt, lề và thân hai cột của ver9; thêm tên bài toàn
chiều ngang. Đã loại bảng, hình và công thức khỏi thân tài liệu. Các phần
ZIP ngoài document.xml, styles.xml và core.xml được bảo toàn byte; do đó
media của mẫu vẫn có thể nằm trong package nhưng không hiện trong Word.
PDF/PNG là dữ liệu QA trong thư mục tạm, không phải đầu ra bàn giao.

Đã duyệt đủ ba trang ngày 22/10/2026. Sửa một ký hiệu đơn vị dùng font
fallback bằng run chỉ số trên và cân lại hai dòng tên bài. Bản cuối dùng
hoàn toàn Times New Roman; hai lần dựng liên tiếp có cùng SHA-256 DOCX.
Trạng thái duyệt chính thức và hash nằm trong các hồ sơ kiểm tra ở trên.

Cập nhật mục tài liệu tại root README. Không thay results/README hoặc
docs/agent vì nguồn dữ liệu, hành vi runtime và quy trình chung không đổi.
Không cần chạy ROS test cho thay đổi bản thảo và script xuất Word này.
