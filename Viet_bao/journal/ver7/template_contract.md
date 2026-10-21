# Quy cách bản thảo journal phiên bản 7

Thiết kế kế thừa trực tiếp [Word ver6](../ver6/PSTMO_JOURNAL_VER6_PHAN_II_III.docx).
Mã SHA-256 của bản gốc và kiểm kê từng thành phần gói Word được lưu trong
[provenance.json](provenance.json). Bản gốc không bị sửa.

- A4, lề trên/dưới 62,35 pt, trái/phải 45,35 pt, hai cột cách nhau 18 pt.
- Times New Roman 10 pt, giãn dòng thân bài 1,06 và cách sau đoạn 4 pt.
- Chú thích và bảng 8 pt. Không giảm cỡ chữ hoặc thay lề để ép trang.
- Giữ ngắt liên tục cuối tài liệu để cân bằng hai cột.
- Giữ nguyên 18 ảnh nhúng, 23 bảng và 12 đối tượng Office Math trong chín
  công thức đánh số. Một ô bảng đổi tham chiếu Bảng I thành Bảng II.
- Chuyển bảng cấu hình PSTMO sang III-A.2; mục II-F được gộp vào II-E.
  Đánh số lại các tiểu mục III-A. Không đổi thứ tự hình và công thức.
- Cho phép chuyển vị trí hình trong cùng mục để tránh hai hình cao đứng
  liền nhau tạo khoảng trắng; giữ nguyên kích thước và dữ liệu hình.
- Giữ toàn bộ từng đoạn chú thích bảng trong một cột để tránh tách mẫu
  số hoặc điều kiện diễn giải sang cột khác.
- Chỉ `word/document.xml` và `docProps/core.xml` được phép thay đổi trong
  gói; kiểu chữ, định nghĩa kiểu, media, quan hệ và các thành phần khác
  giữ nguyên byte.
- Đối chiếu ảnh render gốc với ảnh cuối; kiểm tra mọi trang, biên chữ,
  công thức, bảng, chú giải và tham chiếu trước bàn giao.

Thay đổi nội dung tuân theo kế hoạch người dùng đã duyệt: thêm diễn giải
từng phép so sánh, kết luận từng tuyến và lý do thử nghiệm lặp. Không
thêm trang bìa, không thay tên phương án, không tạo bằng chứng mới ngoài
phép tính dẫn xuất từ dữ liệu đã lưu.
