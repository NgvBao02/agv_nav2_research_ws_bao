# Bản thảo journal PSTMO phiên bản 5

Bản biên tập thuật ngữ và cách diễn đạt của phần II và III, phát triển từ
[ver4](../ver4/README.md). Không thay số liệu, không chạy lại mô phỏng, không
chỉnh sửa các phiên bản cũ hoặc dữ liệu gốc.

- [Bản Word](PSTMO_JOURNAL_VER5_PHAN_II_III.docx).
- [Nguồn nội dung](manuscript.md).
- [Thuật ngữ thống nhất](terminology.md) và [ghi nhận thay đổi](revision_notes.md).
- [Quy cách trình bày](template_contract.md).
- [Hình PNG 450 dpi và SVG](figures/).
- [Hồ sơ nguồn](provenance.json) và [kết quả kiểm tra](quality_checks.json).
- [Bộ dựng](../../../tools/build_journal_sections_v5.py) và
  [bộ kiểm tra](../../../tools/verify_journal_sections_v5.py).

## Phạm vi và nguồn dữ liệu

Giữ 19 hình, 20 bảng, 9 công thức đánh số và 7 tài liệu tham khảo cục bộ.
Giữ mẫu A4 hai cột, Times New Roman 10 pt của ver4; bảng và chú thích 8 pt.
Bản render đã kiểm tra có 15 trang: phần II khoảng 2,2 trang và phần III
khoảng 12,5 trang, tính theo vị trí các mục trong bố cục hai cột; phần còn
lại dành cho tài liệu tham khảo. Phân trang có thể khác nhẹ trên máy khác.
Chỉ có phần II và III, chưa có các phần khác của toàn bài. Cần xác nhận mẫu
tạp chí và hợp nhất danh mục tài liệu tham khảo trước khi nộp toàn bản thảo.

Dữ liệu giữ nguyên: [ma trận 125 lượt](../../../docs/warehouse_cross_aisles_5_routes/README.md)
và [bộ kiểm chứng 250 lượt](../../../results/pstmo_journal_repeatability_20261011/README.md).
Các giới hạn diễn giải trong ver4 tiếp tục áp dụng: đây là mô phỏng với
đầu vào cố định; RPP tắt điều tiết vận tốc theo độ cong và chi phí; kết luận
thống kê chỉ áp dụng cho PSTMO–Constrained trong cấu hình đã thử. Không
diễn giải tích phân bình phương độ cong thành điện năng, độ rung hoặc bảo
đảm G² của quỹ đạo thực thi. Thời lượng hậu xử lý không phải thời gian CPU.

## Tái tạo và kiểm tra

Nguồn tái tạo là `manuscript.md`; bộ dựng chỉ ghi đầu ra trong ver5. Nếu
Word đã được sửa thủ công, lưu bản riêng trước khi dựng lại. Các dependency
và yêu cầu font giống [ver4](../ver4/README.md#tái-tạo-và-kiểm-tra). Tạo môi
trường ảo riêng từ Python của runtime tài liệu; không sửa môi trường ROS.

```bash
PYTHONNOUSERSITE=1 /tmp/pstmo-journal-v5-env/bin/python tools/build_journal_sections_v5.py
```

Dùng `render_docx.py` của kỹ năng Documents và LibreOffice đóng gói trong
runtime; xuất PDF và PNG kiểm tra vào `/tmp/pstmo-v5-render-complete`, không
đưa các tệp kiểm tra đó vào sản phẩm bàn giao. Kiểm tra trực quan từng trang
trước khi dùng cờ xác nhận dưới đây.

```bash
PYTHONNOUSERSITE=1 /tmp/pstmo-journal-v5-env/bin/python tools/verify_journal_sections_v5.py --render-dir /tmp/pstmo-v5-render-complete --accept-visual-review
python3 tools/check_repository_docs.py
git diff --check
```

Bộ kiểm tra đối chiếu số liệu bảng, khoảng tin cậy, công thức và mã kiểm tra
của các bản trước; tính lại so sánh ghép cặp; kiểm tra thuật ngữ, độ phân
giải hình, phông chữ và biên trang. Không cần ROS test cho thay đổi chỉ liên
quan bản thảo và bộ dựng.
