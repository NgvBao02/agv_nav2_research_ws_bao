# Quy cách hiệu đính journal ver9

Tham chiếu là [Word ver8](../ver8/PSTMO_JOURNAL_VER8_PHAN_II_III.docx),
SHA-256 `bb3e225b95938fa54138ef6474ff3fddab675a757656309ef0ee17a2249ccc8e`.
Kế thừa bố cục IEEE A4 hai cột; chưa xác nhận mẫu của tạp chí đích.

- Giữ một section hai cột, khoảng cột 18 pt, lề bên 45,35 pt và lề trên/dưới
  62,35 pt. Không thêm header/footer hoặc ép số trang.
- Giữ Times New Roman 10 pt, giãn dòng 1,06, sau đoạn 4 pt; bảng/chú thích
  8 pt và định dạng tiêu đề gốc. Giữ nguyên pPr của từng đoạn sửa.
- Chỉ thay văn bản tại 21 khối đã ghi trong revision_log.json; không đổi thứ
  tự khối, hình, bảng, công thức, section hoặc phần tài liệu tham khảo.
- Giữ nguyên XML của 23 bảng/696 ô, 18 hình và 12 đối tượng Office Math,
  tương ứng chín công thức đánh số. Ký hiệu mới trong văn xuôi dùng chỉ số
  dưới chỉnh sửa được, không chuyển công thức thành ảnh.
- Chỉ thay word/document.xml và docProps/core.xml; mọi thành phần ZIP khác
  giữ nguyên byte. Không dựng lại hoặc ghi đè ver6–8 hay dữ liệu cũ.
- Đối chiếu mô tả thuật toán với source/config của cohort; ghi rõ những gì
  chưa có bằng chứng, không tạo ablation, giả thuyết lịch sử hoặc số đo mới.
- Render bằng Documents/LibreOffice đóng gói, cung cấp font Times New Roman
  rõ ràng qua fontconfig vì renderer dùng profile tạm. Kiểm tra font thực
  trong PDF, giới hạn trang và toàn bộ ảnh trang trước khi duyệt.
- Xác nhận trực quan gắn SHA-256 DOCX/PDF/từng PNG trong visual_review.json;
  kiểm tra tự động không tự thay thế việc xem trang.
