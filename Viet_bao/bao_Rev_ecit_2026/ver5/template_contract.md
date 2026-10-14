# Quy cách bản Word

Nguồn thiết kế: `../ecit_word_template.docx`, tương ứng mẫu được trang hướng
dẫn REV-ECIT liên kết tại
https://rev-ecit.vn/sites/default/files/File2024/ecit2024-word.docx.

Mã SHA-256 của mẫu:
`19dcd2e8a6183a2674961adacb58c784b52a1ff0bf364c60d0cd16929e18ec60`.

## Hình học và kiểu chữ

- Khổ giấy 11909 × 16834 twip, A4 dọc.
- Lề trái/phải 1361 twip, trên 1474 twip, dưới 2155 twip.
- Nội dung hai cột, khoảng cách cột 360 twip.
- Tiêu đề toàn chiều rộng, Times New Roman 24 pt; tác giả và đơn vị 11 pt.
- Nội dung Times New Roman 10 pt; tóm tắt và từ khóa 9 pt.
- Chú thích, bảng và tài liệu tham khảo 8 pt. Công thức dùng toán học Word.
- Tiêu đề mục đánh số La Mã; tiểu mục chữ cái; bảng đánh số La Mã, hình đánh
  số Ả Rập. Chú thích hình dưới ảnh và tên bảng trên bảng.

## Phạm vi tái dựng

Nội dung mẫu minh họa được thay toàn bộ bằng bài PSTMO theo yêu cầu sửa đầy
đủ. Gộp các phân đoạn riêng của khối tác giả thành một khối; giữ phân đoạn
hai cột cho thân bài và phân đoạn liên tục cuối để cân cột. Các bảng và hình
là nội dung mới, không giữ hình hoặc dữ liệu ví dụ của mẫu. Kiểu tiêu đề và
chú thích được chuẩn hóa màu đen. Mẫu gốc không thay đổi.

Các phần hỗ trợ `word/numbering.xml`, `word/fontTable.xml` và
`word/theme/theme1.xml` được giữ nguyên byte. Phần nội dung, quan hệ đến hình,
kiểu đoạn và metadata được phép cập nhật. Không đưa chú thích biên tập hay
ghi chú nộp bài vào thân bài.

## Kiểm chứng

Xuất bằng quy trình `render_docx.py` của kỹ năng Documents và xem ảnh từng
trang. Khi xuất trong hồ sơ tạm của LibreOffice, cấu hình font phải trỏ đến
nơi có Times New Roman thật; kiểm tra font nhúng trong PDF để tránh thay thế
âm thầm. Kiểm tra biểu thức tổ hợp không có gạch phân số, căn bậc hai đầy đủ,
không có ô nhập toán học rỗng, số công thức không tràn cột, bảng không bị
chia hàng và hình không tách khỏi chú thích. Không suy ra bố cục chỉ từ XML.

Nội dung văn xuôi dùng Times New Roman nhúng trong PDF. Bộ xuất công thức
Word của LibreOffice sử dụng thêm font toán học/ký hiệu và font cho dấu trừ
trên cao; danh sách đầy đủ được ghi trong `quality_checks.json`. Các hình
khoa học dùng DejaVu Serif để hỗ trợ đầy đủ ký hiệu và tiếng Việt. Đây không
phải việc thay font của văn xuôi hoặc thay công thức bằng ảnh.

