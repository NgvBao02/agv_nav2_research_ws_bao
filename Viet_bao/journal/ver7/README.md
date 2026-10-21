# Bản thảo journal PSTMO phiên bản 7

Bản biên tập từ [ver6](../ver6/README.md), tách cấu hình triển khai khỏi mô
tả thuật toán và bổ sung phân tích ngay sau kết quả so sánh. Giữ nguyên
ma trận 125 lượt, bộ kiểm chứng 250 lượt, 18 hình và số liệu của 23 bảng.
Không chạy mô phỏng mới, không sửa dữ liệu hoặc bản Word cũ đang mở.

- [Bản Word](PSTMO_JOURNAL_VER7_PHAN_II_III.docx).
- [Nội dung đã sinh](manuscript.md).
- [Nguồn biên tập](editorial_revisions.json), [nhật ký sửa](revision_log.json).
- [Bằng chứng số liệu cho phần phân tích](editorial_evidence.json).
- [Số liệu bảng](table_values.json), [hồ sơ nguồn](provenance.json).
- [Kiểm tra cuối](quality_checks.json), [quy cách](template_contract.md).
- [Bộ dựng](../../../tools/build_journal_sections_v7.py),
  [bộ kiểm tra](../../../tools/verify_journal_sections_v7.py).

## Nội dung thay đổi

- Gộp điều kiện chấp nhận đầu ra của II-F vào II-E, bỏ tóm tắt lặp.
- Chuyển bảng cấu hình PSTMO, tích hợp RPP và phân biệt thời gian dự đoán
  với thực thi vào III-A.2, giữa robot và bản đồ. Bảng robot thành Bảng I,
  bảng cấu hình PSTMO thành Bảng II; cập nhật tham chiếu tương ứng.
- Giữ các dãy số đủ năm phương án, bổ sung nhận xét chiều dài, độ cong,
  sai số bám cực đại, độ giật và sai số cuối tại từng tuyến. Mỗi tuyến có
  kết luận về lợi ích và đánh đổi, không suy diễn nguyên nhân chưa đo.
- Phân tích thêm tín hiệu Hình 16 từ năm bản ghi R02/Theta*. Khoảng mẫu có
  vận tốc góc trên 0,24 rad/s trong cửa sổ 34–40 s chỉ là mô tả tín hiệu
  minh họa, không phải chỉ số chính hoặc thí nghiệm mới. Giữ nguyên hình.
- Giải thích lý do cần kiểm chứng lặp và phạm vi chọn Constrained; không
  khẳng định đây là đối chứng mạnh nhất hoặc suy rộng sang các đối chứng
  không có dữ liệu lặp.
- Giữ nguyên phần phân tích thất bại III-I và bảy tài liệu tham khảo.

Nguồn dữ liệu và giới hạn diễn giải vẫn như [ver6](../ver6/README.md),
[ma trận năm tuyến](../../../docs/warehouse_cross_aisles_5_routes/README.md)
và [bộ lặp](../../../results/pstmo_journal_repeatability_20261011/README.md).
Không gộp hai bộ dữ liệu, không loại lượt thất bại và không coi các bộ
lập kế hoạch khác nhau là các lần lặp của cùng đầu vào.

## Tái tạo

Nguồn là Word và `manuscript.md` ver6, `editorial_revisions.json` của ver7
và các phép chuyển vị trí trong bộ dựng. Không sửa trực tiếp đầu ra sinh
`manuscript.md` rồi dựng lại. Lưu riêng các chỉnh sửa Word thủ công.

Dùng Python trong runtime Documents với lxml và PyMuPDF. Môi trường ảo
tạm `/tmp/pstmo-journal-v7-env` phải tạo lại nếu phiên làm việc thay đổi.

```bash
PYTHONNOUSERSITE=1 /tmp/pstmo-journal-v7-env/bin/python tools/build_journal_sections_v7.py
```

Bộ dựng vá gói Word gốc, chỉ thay `word/document.xml` và thuộc tính phiên
bản trong `docProps/core.xml`. Tất cả media, styles, numbering, settings
và relationships giữ nguyên byte. Không gọi lại bộ dựng ver6.

Render bằng `render_docx.py` và LibreOffice đóng gói của Documents. Nếu
font cài riêng chưa được nhận, dùng `FONTCONFIG_FILE` trỏ tới cấu hình
Times New Roman. PDF và ảnh trang trong `/tmp` chỉ dùng QA. Chạy bộ kiểm
tra với đúng thư mục render; chỉ thêm cờ xác nhận sau khi xem mọi trang:

```bash
PYTHONNOUSERSITE=1 /tmp/pstmo-journal-v7-env/bin/python tools/verify_journal_sections_v7.py --render-dir /tmp/pstmo-v7-render-final --accept-visual-review
python3 tools/check_repository_docs.py
git diff --check
```

Kiểm tra gồm mã nguồn dữ liệu không đổi, mọi ô bảng và cấu trúc công
thức giữ nguyên, tham chiếu Bảng I/II, 225 trung bình tính lại từ 125 bản
ghi gốc và năm chuỗi tín hiệu minh họa. Kiểm tra thống kê số liệu gốc
được kế thừa từ ver6 qua đối chiếu nguyên vẹn bảng và hồ sơ nguồn, không
tuyên bố chạy lại thử nghiệm. Không cần chạy ROS test.

Đầu ra ngày 21/10/2026 có 15 trang hai cột; đã kiểm tra trực quan toàn bộ
15 trang, gồm vị trí chú thích Bảng XVIII và XXI. Giữ nguyên 696 ô bảng
(ngoại trừ một tham chiếu Bảng I được đổi thành Bảng II), 18 ảnh nhúng và
12 đối tượng công thức. Kết quả đối chiếu số liệu, mã kiểm tra đầu ra và
xác nhận trực quan được ghi trong `quality_checks.json`.

Bản này chỉ gồm phần II–III và tài liệu tham khảo
cục bộ, kế thừa bố cục IEEE conference A4; chưa xác nhận mẫu riêng của
tạp chí đích. Cần ghép toàn bài và đánh số lại tài liệu tham khảo trước
khi nộp.
