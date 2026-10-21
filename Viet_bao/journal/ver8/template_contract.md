# Quy cách biên tập journal phiên bản 8

Nguồn duy nhất về bố cục là [Word ver7 gốc](reference_ver7_original.docx),
SHA-256 `bef87e2d34ea41b54e1647f8954fa7a4cce00405e73e6a7189c8707c62697aa9`.
Nguồn đã được kiểm tra trực quan đủ 15 trang, xác nhận trong hồ sơ ver7;
render tham chiếu của lượt sửa này nằm tại `/tmp/pstmo-v8-reference`.

- Giữ A4, hai cột cách nhau 18 pt; lề trái/phải 45,35 pt, trên/dưới 62,35 pt.
- Dùng một section hai cột; bỏ section rỗng một cột ở cuối vốn gây trang trắng
  trong bản render. Giữ nguyên khổ giấy và lề; không header/footer.
- Đoạn rỗng mang ngắt section cuối dùng khoảng dòng và dấu đoạn 1 pt, trước/sau
  bằng 0 để tránh trang trắng cuối; không đổi định dạng chữ hoặc đoạn có nội dung.
- Giữ nguyên styles và pPr theo vai trò đoạn: thân bài Times New Roman 10 pt,
  giãn dòng 1,06, sau đoạn 4 pt; chú thích/bảng 8 pt; tiêu đề như nguồn.
- Giữ nguyên 23 bảng (696 ô), 18 ảnh, kích thước/quan hệ ảnh và thứ tự hình.
- Đặt Hình 8 sau Bảng VIII và đoạn diễn giải bảng, ngay trước phân tích theo
  bộ lập kế hoạch; không đổi nội dung hoặc kích thước hình.
- Đặt Hình 18 sau hai đoạn phân tích thất bại, trước mục giới hạn; giữ nguyên
  từng đoạn, chú thích và kích thước hình.
- Giữ 12 đối tượng Office Math; chỉ đổi vị trí và nhãn công thức 5/6.
- Không có field, content control, footnote hoặc tracked changes cần xử lý.
- Chỉ document.xml và core.xml được phép đổi. Mọi thành phần gói khác phải
  giữ nguyên byte; kiểm kê SHA-256 được lưu trong provenance.json.
- Mỗi khối manuscript được ánh xạ vào phần tử body tương ứng và kiểm tra
  nội dung trước khi sửa. Nhật ký ghi chỉ số khối nguồn, đoạn cũ/mới và phép
  chuyển vị trí. Đoạn thêm mới kế thừa pPr đúng vai trò, không thu nhỏ chữ.
- Phạm vi được sửa: II-D và chú thích cấu hình; III-A.3–7; các tiêu đề và
  đoạn diễn giải đã duyệt tại III-B–F; phần tổng hợp, thiết kế lặp và giới hạn
  nghiên cứu. Giữ mọi số liệu bảng, dãy số năm phương án và ca thất bại.
- Bỏ phép mô tả ngưỡng 0,24 rad/s trong cửa sổ 34–40 s ở Hình 16; giữ ảnh
  gốc và số liệu thời gian toàn nhiệm vụ. Không thêm giả thuyết chọn ca trước
  thử nghiệm hoặc suy luận nhân quả.
- Kiểm tra định dạng, công thức, bảng, ảnh và mọi trang của bản cuối;
  mục tiêu khoảng 15 trang. Không ép trang bằng giảm font hoặc đổi lề.
