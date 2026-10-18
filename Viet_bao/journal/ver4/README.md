# Bản thảo journal PSTMO phiên bản 4

Bản này biên tập lại phần II và III bằng tiếng Việt theo ưu điểm diễn đạt của
REV-ECIT ver5: câu trực tiếp, thuật ngữ nhất quán, kết quả đi cùng số liệu và
điều kiện so sánh. Giữ chiều sâu phương pháp và thống kê của journal ver3;
không chuyển dữ liệu hoặc kết luận thực nghiệm của REV-ECIT sang journal.

- [Bản Word phần II và III](PSTMO_JOURNAL_VER4_PHAN_II_III.docx).
- [Nguồn nội dung](manuscript.md).
- [Ghi nhận thay đổi](revision_notes.md) và [quy cách trình bày](template_contract.md).
- [19 hình PNG 450 dpi và SVG](figures/).
- [Hồ sơ nguồn và ánh xạ hình](provenance.json).
- [Kết quả kiểm tra](quality_checks.json).
- [Bộ dựng](../../../tools/build_journal_sections_v4.py) và
  [bộ kiểm tra](../../../tools/verify_journal_sections_v4.py).

## Phạm vi và bố cục

Bản render đã kiểm tra có 14 trang: phần II khoảng 2 trang, phần III khoảng
11,8 trang và phần còn lại là bảy tài liệu tham khảo cục bộ. Số trang thấp
hơn dự kiến ban đầu do rút gọn câu văn và giảm khoảng trắng, không bỏ bảng,
hình hoặc kết quả. Word trên máy khác có thể phân trang hơi khác.

Giữ 19 hình, 20 bảng và 9 công thức đánh số, trong đó công thức dùng Office
Math và bảng có thể chỉnh sửa trong Word. Hình Bézier thể hiện các điểm điều
khiển, hai điểm nối, hướng tiếp tuyến và chiều dài cắt. Hình thực nghiệm dùng
dữ liệu nguồn, không dùng ảnh sinh bằng AI. Hình, bảng và văn bản dùng tên
thuật toán chuẩn, không đưa tên trường dữ liệu hoặc mã phiên bản vào thân bài.

Khổ A4 hai cột, Times New Roman 10 pt, chú thích/bảng/tài liệu tham khảo 8 pt;
giữ lề và khoảng cột của mẫu gốc. Nguồn bố cục là
[mẫu IEEE conference A4](<../../final_bao_ICEEIS/IEEE Xplore conference-template-a4.docx>),
không phải mẫu riêng của một tạp chí đã chọn. Chỉ có phần II và III, chưa có
tóm tắt, giới thiệu, kết luận và danh mục tài liệu tham khảo của toàn bài.
Bảy trích dẫn cục bộ kế thừa ver3 cần đánh số lại khi ghép toàn bản thảo.

## Nguồn dữ liệu và giới hạn diễn giải

- [Ma trận khảo sát 125 lượt](../../../docs/warehouse_cross_aisles_5_routes/README.md):
  123 lượt đạt; chỉ dùng mô tả, không coi năm bộ lập kế hoạch là năm lần lặp.
- [Bộ kiểm chứng lặp 250 lượt](../../../results/pstmo_journal_repeatability_20261011/README.md):
  243 lượt đạt, 125 cặp cùng đường đầu vào, 118 cặp cùng đạt, 25 khối.
- So sánh thêm số cặp PSTMO có thời gian hoặc RMSE thấp hơn từng đối chứng.
  Bộ kiểm tra tính lại các số đếm từ dữ liệu gốc, không suy từ trung bình.
- RPP của journal tắt điều tiết vận tốc theo độ cong và chi phí. Không sao
  chép cách giải thích cơ chế điều khiển từ một cấu hình thực nghiệm khác.
- Constrained có trọng số độ cong bằng không. Kết luận thống kê chỉ áp dụng
  PSTMO–Constrained theo cấu hình đã thử, chưa đại diện cho mọi đối chứng.
- Giữ các đánh đổi về khoảng hở, sai số đích, thời lượng hậu xử lý và thất bại.
  Thời lượng hậu xử lý không được gọi là thời gian CPU.
- Không chạy lại mô phỏng hoặc thay dữ liệu. Bản Word journal ver3, nguồn
  ver3 và bản REV-ECIT ver5 giữ nguyên; mã kiểm tra được xác nhận riêng.

Các giới hạn dữ liệu trong [README ver3](../ver3/README.md) tiếp tục có hiệu lực.

## Tái tạo và kiểm tra

Dùng Python của runtime tài liệu trong môi trường ảo riêng, với python-docx,
NumPy, Matplotlib, Pillow, PyYAML, lxml, latex2mathml, mathml2omml, PyMuPDF
và pdf2image. Cần Times New Roman và DejaVu Math TeX Gyre. Không cài gói vào
môi trường ROS của workspace.

Bộ dựng chỉ ghi artifact do nó sinh **trong ver4**. Nếu đã sửa Word thủ công,
cần lưu bản riêng trước khi dựng lại vì nguồn tái tạo là `manuscript.md`.

```bash
PYTHONNOUSERSITE=1 /tmp/pstmo-journal-v4-env/bin/python tools/build_journal_sections_v4.py
```

Môi trường `/tmp/pstmo-journal-v4-env` phải được tạo trước; không được lưu
trong repository. Dùng `render_docx.py` của skill Documents cùng LibreOffice
đóng gói trong runtime để xuất PDF và ảnh trang vào thư mục tạm. Nếu font cài
riêng không được nhận, truyền `FONTCONFIG_FILE` có đường dẫn tuyệt đối tới
thư mục font. Không chấp nhận bản render thay font thân bài.

```bash
PYTHONNOUSERSITE=1 /tmp/pstmo-journal-v4-env/bin/python tools/verify_journal_sections_v4.py --render-dir /tmp/pstmo-v4-render-verified
PYTHONNOUSERSITE=1 /tmp/pstmo-journal-v4-env/bin/python tools/verify_journal_sections_v4.py --render-dir /tmp/pstmo-v4-render-verified --accept-visual-review
python3 tools/check_repository_docs.py
git diff --check
```

Chỉ dùng `--accept-visual-review` sau khi đã xem mọi ảnh trang của đúng bản
render. Bộ kiểm tra đối chiếu số liệu bảng và công thức với ver3, xác nhận
nguồn không đổi, tính lại so sánh ghép cặp, kiểm tra font, độ phân giải hình
và biên trang. PDF/PNG kiểm tra là tệp tạm, không phải sản phẩm bàn giao.
Không cần chạy ROS test cho thay đổi chỉ liên quan bản thảo và bộ dựng này.
