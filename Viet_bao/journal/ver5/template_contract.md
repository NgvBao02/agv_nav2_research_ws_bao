# Quy cách bản biên tập journal phiên bản 5

Nguồn thiết kế là [Word ver4](../ver4/PSTMO_JOURNAL_VER4_PHAN_II_III.docx),
SHA-256 `ab04f962109b12457151338de8e606e1fd9d8799590c9d9bb5d4dbb83f3538ef`.
Giữ lại tệp này và toàn bộ thư mục ver4. Bản dựng mở gói Word ver4, sử dụng
lại hệ thống kiểu và thuộc tính trang khi thay nội dung phần II và III.

- A4, 595,3 × 841,9 pt; lề trên/dưới 62,35 pt, trái/phải 45,35 pt.
- Hai cột, khoảng cột 18 pt; ngắt liên tục ở cuối để cân bằng cột.
- Times New Roman 10 pt; giãn dòng thân bài 1,06, cách sau đoạn 4 pt.
- Bảng, chú thích và tài liệu tham khảo 8 pt; không giảm cỡ chữ để ép trang.
- Giữ 20 bảng, 19 hình, 9 công thức đánh số với 12 đối tượng Office Math.
- Phần II có các mục A–F, phần III có A–I; giữ thứ tự bảng, hình, công thức.
- Công thức giữ nguyên cấu trúc XML; bảng giữ giá trị số và khoảng tin cậy.
- Nhãn hình được sửa theo thuật ngữ mới; hình dữ liệu vẫn dựng từ cùng nguồn.
- Thay nội dung, nhãn bảng, chú thích, ảnh có nhãn sửa và thuộc tính phiên bản
  là thay đổi có chủ đích. Không thêm đầu trang, chân trang hay trang bìa.
- Kiểm tra từng ảnh trang sau render, đối chiếu biên trang và phông thực.

Mục tiêu phần II khoảng 2–2,5 trang, phần III khoảng 12–12,5 trang. Số trang
và vị trí tiêu đề của bản render được ghi trong `quality_checks.json`.
Quy cách này kế thừa mẫu IEEE conference A4, chưa phải mẫu riêng của một
tạp chí đã được lựa chọn.
