# Hợp đồng biên tập Word ver12

- Nhiệm vụ: major scientific revision, không thay thuật toán hoặc dữ liệu.
- Nguồn có thẩm quyền: Word ver11 thực tế sau chỉnh sửa của tác giả; SHA-256 `debafc4f83a9b7a4dfd52230908cb37fac42ccfb5c23bebea45be996ce556fbc`.
- Nguồn này khác bản dựng cũ ghi trong provenance ver11. Không chạy lại builder ver11; không khôi phục Abstract tiếng Anh đã bị tác giả xóa.
- Quyết định tác giả: chỉ Tóm tắt và từ khóa tiếng Việt; giữ hai tên bài hiện có.
- Kế thừa package Word: chỉ sửa `word/document.xml`; giữ styles, relationships, media, numbering, section geometry và mọi phần khác.
- A4: 11906 × 16838 twip; lề trái/phải 907, trên/dưới 1247; thân hai cột, khoảng cách 360. Khối tên bài rộng toàn trang.
- BodyText 10 pt Times New Roman; Heading1 10 pt small caps, Heading2 10 pt nghiêng; ScientificReference 8 pt, thụt treo 240 twip. Giữ nguyên styles thực tế, không ép số trang bằng giảm chữ/lề.
- Giữ 23 bảng/696 ô, 18 drawing, 12 OMML trong 9 phương trình đánh số. So sánh XML trực tiếp với Word nguồn.
- Thay I và IV, Tóm tắt; II-A–E giữ thứ tự và nội dung cũ, chỉ thêm giải thích có mục tiêu ở II-D/E; III nguyên văn trừ số citation.
- Citation IEEE theo thứ tự xuất hiện; nguồn Markdown dùng khóa ổn định, không nhập số thủ công. Mỗi nguồn bỏ phải có lý do và không còn được dẫn ở II/III.
- Bản nguồn đã render 21 trang, xem trực quan trang 1–21: không thấy tràn cột/cắt công thức, bảng và chú thích đọc được. Các ký hiệu đặc biệt có fallback font trong bộ render; không thay OMML gốc để cưỡng ép font.
- Đầu ra phải render lại và kiểm tra riêng từng trang; số trang cuối được ghi sau kiểm tra, không dự đoán trước.
- Không chạy ROS/Gazebo/benchmark hoặc tái phân tích thống kê. Không sửa ver11, REFERENCES, results hoặc mã ROS.
- Hồ sơ: nguồn biên tập, builder, citation map, quyết định nguồn, audit khoa học, so sánh trước/sau, provenance, QA, visual review, README.
