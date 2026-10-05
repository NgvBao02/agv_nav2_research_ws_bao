# Mẫu trình bày và phạm vi thay đổi

Nguồn: `docs/PSTMO.docx`, SHA-256
`c00a63ca1f5dc96361d63330787a81aa16fd0516f9e7730d81dcce6565012579`.
Tệp gốc được giữ nguyên. Bản render tham chiếu có 121 trang.

Bản lý thuyết mới thay toàn bộ phần thân tài liệu nhưng giữ một section A4,
kích thước và lề gốc, footer đánh số trang, các kiểu Report Heading, Caption,
Equation, References và Normal. Title kế thừa Report Title, màu đen, không viền.
Không thêm trang bìa. Thân bài dùng khoảng cách sau đoạn 4 pt. Công thức mới dùng
OMML, không dùng ảnh. Hình giữ tỷ lệ và đi cùng chú thích.

Chương 1 và phụ lục bắt đầu trang mới. Các chương chính còn lại nối theo dòng
nội dung để tránh trang kết thúc chỉ có vài dòng; không lược bỏ nội dung để
giảm số trang. Chú thích được bỏ keep-with-next kế thừa để không nối chuỗi nhiều
hình một cách ngoài ý muốn. Bảng lặp header và không cắt một hàng qua hai trang.

Các phần ZIP được phép thay đổi: document.xml, document.xml.rels, settings.xml,
core.xml, app.xml và riêng phần tử style Title trong styles.xml. Các phần ZIP
cũ khác được bảo toàn byte-for-byte; mọi relationship cũ được giữ nguyên. Hình
mới có thể thêm relationship và media. Section và các style ngoài Title được
so sánh cấu trúc trong script kiểm tra.

Mục lục là 19 liên kết nội bộ được kiểm tra đích bookmark; không có số trang
cache cần refresh. Footer PAGE giữ nguyên, updateFields được bật. Chưa có bước
refresh hoặc render bằng Word desktop. LibreOffice đóng gói chỉ xuất PDF/PNG
kiểm tra, không ghi đè hoặc lưu lại file Word.

Kết quả cuối được xác nhận trong `qa_summary.json`. Mọi trang cuối đã được xem
ở độ phân giải gốc của PNG; những trang không đổi sau sửa công thức được xác
nhận giống hoàn toàn bằng hash raster. PDF và PNG là dữ liệu kiểm tra tạm,
không phải tệp bàn giao bổ sung.
