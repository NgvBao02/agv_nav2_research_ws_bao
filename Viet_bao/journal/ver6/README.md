# Bản thảo journal PSTMO phiên bản 6

Bổ sung so sánh năm phương án trên năm tuyến từ dữ liệu đã có, kế thừa văn
phong và quy cách [ver5](../ver5/README.md). Không chạy thêm mô phỏng, không
thay đổi phần II, không ghi đè bản cũ hoặc dữ liệu thực nghiệm.

- [Bản Word](PSTMO_JOURNAL_VER6_PHAN_II_III.docx).
- [Nội dung đã sinh](manuscript.md) và [nguồn biên tập III-G–I](sections_ghi.md).
- [Ghi nhận thay đổi](revision_notes.md), [quy cách trình bày](template_contract.md).
- [Hình PNG 450 dpi và SVG](figures/).
- [Số liệu bảng](table_values.json), [hồ sơ nguồn](provenance.json),
  [kết quả kiểm tra](quality_checks.json).
- [Bộ dựng](../../../tools/build_journal_sections_v6.py),
  [bộ kiểm tra](../../../tools/verify_journal_sections_v6.py).

## Phạm vi và nguồn dữ liệu

Nguồn chính là [ma trận 125 lượt](../../../docs/warehouse_cross_aisles_5_routes/README.md):
năm tuyến, năm bộ lập kế hoạch và năm phương án gồm đường gốc, Simple,
Savitzky–Golay, Constrained và PSTMO. Mỗi tổ hợp có một lượt. Giữ riêng
[bộ kiểm chứng 250 lượt](../../../results/pstmo_journal_repeatability_20261011/README.md)
PSTMO–Constrained đã có trong ver5; không mở rộng kết luận thống kê của bộ
này sang các đối chứng khác.

Phần III-B–F bổ sung chiều dài tham chiếu, độ cong cực đại, quãng đường thực
thi, sai số bám cực đại, thời lượng xử lý, độ cong thực thi, độ giật, sai số
vị trí và hướng cuối cho đủ năm phương án trên từng tuyến. Giữ các chỉ số
thời gian, RMSE, khoảng hở và tỷ lệ hoàn thành. Phần III-G so sánh ghép cặp
với cả bốn đối chứng, bổ sung hình học phần tịnh tiến sau lấy mẫu lại,
vận tốc, gia tốc và kiểm tra an toàn trên toàn ma trận.

Các tệp trong [analysis](analysis/) là **phân tích dẫn xuất**, không phải bộ
thực nghiệm mới:

- `all_125_metrics.csv`: chỉ số cho từng lượt và đường dẫn bản ghi gốc.
- `summary_metrics.csv`: trung bình, cực tiểu, cực đại, số mẫu và cờ lọc
  lượt đạt cho từng chỉ số, từng tuyến/phương án và toàn ma trận.
- `pairwise_comparisons.csv`: 672 thống kê chênh lệch PSTMO với đối chứng,
  ghép theo cùng tuyến/bộ lập kế hoạch; không phải 672 phép thử mới.
- `data_audit.json`: kiểm tra đủ trường, 25 nhóm đầu vào giống nhau, tính
  lại chỉ số vận tốc/gia tốc/độ giật từ 125 chuỗi trạng thái và tích phân
  bình phương độ cong từ 125 quỹ đạo thực thi đã xử lý lưu trong bản ghi.

Không coi thời lượng hậu xử lý là thời gian CPU; không coi vận tốc bánh
tương đương là đo trực tiếp từng bánh; không coi các mẫu không va chạm là
bảo đảm an toàn liên tục. Độ cong và RMSE thực thi dùng dữ liệu đã xử lý
như mô tả trong III-A. Phép kiểm tra ở đây không chạy lại toàn bộ bước lọc
hình học từ chuỗi trạng thái thô.

## Tái tạo và kiểm tra

Nguồn biên tập gồm `ver5/manuscript.md`, `sections_ghi.md` và các bổ sung
được tính trong bộ dựng ver6. `manuscript.md` của ver6 là đầu ra được sinh;
không chỉnh trực tiếp rồi dựng lại. Lưu riêng mọi bản Word sửa thủ công.
Bộ dựng chỉ ghi vào ver6 và sử dụng lại bộ dựng ver5 như thư viện đọc dữ
liệu, định dạng và tạo hình. Không chạy bộ dựng ver5 để tái tạo ver6.

Dùng Python của runtime Documents, tạo môi trường ảo riêng trong `/tmp`;
không sửa môi trường ROS. Dependency gồm python-docx, matplotlib, numpy,
Pillow, PyYAML, lxml, latex2mathml, mathml2omml và PyMuPDF. Môi trường tạm
phải được tạo lại khi phiên làm việc thay đổi.

```bash
PYTHONNOUSERSITE=1 /tmp/pstmo-journal-v6-env/bin/python tools/build_journal_sections_v6.py
```

Render bằng `render_docx.py` của kỹ năng Documents và LibreOffice đóng gói
trong runtime. Dùng `FONTCONFIG_FILE` trỏ tuyệt đối tới font Times New Roman
nếu font cài riêng chưa được nhận. PDF và ảnh trang là tệp QA trong `/tmp`,
không phải sản phẩm bàn giao. Sau khi xem từng trang của đúng bản render:

```bash
PYTHONNOUSERSITE=1 /tmp/pstmo-journal-v6-env/bin/python tools/verify_journal_sections_v6.py --render-dir /tmp/pstmo-v6-render-verified --accept-visual-review
python3 tools/check_repository_docs.py
git diff --check
```

Chỉ dùng cờ xác nhận trực quan sau khi đã xem toàn bộ ảnh trang. Bộ kiểm tra
đối chiếu bảng Word, tính lại 222 ô số mới và 672 thống kê ghép cặp, kiểm tra
công thức và phần II không đổi, mã kiểm tra nguồn, thuật ngữ, phông, độ phân
giải hình và biên trang. Số trang và vị trí các mục được ghi chính xác trong
`quality_checks.json`. Không cần ROS test cho thay đổi bản thảo này.

Bản render kiểm tra ngày 18/10/2026 có 15 trang hai cột, đã xem toàn bộ
ảnh trang. Theo vị trí các mục trong hai cột, phần II chiếm khoảng 2,2
trang và phần III khoảng 12,5 trang; phần còn lại là tài liệu tham khảo.
Không giảm cỡ chữ thân bài để đạt độ dài này. Phân trang có thể thay đổi
nếu mở bằng trình soạn thảo hoặc bộ phông khác.

Bản Word chỉ gồm phần II, III và bảy tài liệu tham khảo cục bộ. Đây vẫn là
bố cục IEEE conference A4 kế thừa, chưa được xác nhận là mẫu riêng của tạp
chí đích. Cần hợp nhất các phần còn lại, đánh số lại tài liệu tham khảo và
đối chiếu yêu cầu tạp chí trước khi nộp toàn bài.
