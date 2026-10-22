# Hồ sơ biên tập và giới hạn xác minh ver11

## Kết quả ghép

Giữ tên bài đã chọn, Tóm tắt và Kết luận từ ver10. Dịch Abstract sang tiếng
Anh, không thêm số liệu mới. Introduction được tổ chức thành bối cảnh, năm
nhóm lập luận từ tìm kiếm/hình học/tối ưu–điều khiển đến câu hỏi nghiên cứu
và đóng góp. Các đoạn bổ sung trong II giải thích cơ sở biểu diễn, mô hình
vi sai, ý nghĩa tích phân độ cong và nguyên lý quy hoạch động; III phân
biệt kiến trúc, tài liệu phần mềm, cấu hình thử và hướng mở rộng môi trường động.

Giữ 74 khóa corpus, dẫn vào các phát biểu đúng vai trò. Danh mục thống nhất
có 80 mục, đánh số theo lần xuất hiện đầu trong toàn bài. Không đặt trích
dẫn vào abstract/conclusion chỉ để tăng số lượng. Không gọi 80 mục là 80
bài peer-reviewed: có sách, chương sách, preprint, tài liệu phần mềm và mã nguồn.

## Sửa nhận diện và phiên bản

1. **016:** PDF thực là LaValle–Kuffner, *Rapidly-exploring random trees:
   Progress and prospects*, chương sách 2001, trang 293–308. Catalog gắn
   nhãn bài *Randomized Kinodynamic Planning*. Dùng metadata của đúng PDF;
   đối chiếu trang đầu và [danh mục tác giả](https://lavalle.pl/rrtpubs.html).
2. **026:** PDF là slide cùng chủ đề của Farouki. Đọc slide 25 về cơ sở
   Bernstein; nhận diện bài journal bằng [trang nhà xuất bản](https://www.sciencedirect.com/science/article/pii/S0167839612000192).
   Mục này ánh xạ đến công trình cùng chủ đề, **không phải xác nhận PDF là
   toàn văn journal hoặc đã đọc toàn văn journal**.
3. **001/003:** PDF là working paper 2004/2006, danh mục nhận diện bài
   journal 2006/2007. Luận điểm tổng quan sử dụng có trong working paper;
   không gán số trang PDF cho bản final.
4. **012/016/017/050:** lớp chữ thiếu hoặc hỏng; kiểm tra trực tiếp ảnh trang
   đầu thay vì dùng văn bản trích xuất sai. Mục 060 bắt đầu ở nửa dưới trang
   đầu, phía trên là phần kết bài khác.
5. **050/057/059:** URL trong catalog trỏ survey Ravankar, không phải bài
   được nhận diện. Không giữ các URL sai trong metadata ver11. Mục 059 có
   bản tác giả tại TU Dortmund; 057 dựa trên PDF cục bộ, không đoán URL.
6. **023/037/040/063:** dùng đúng phiên bản preprint thực có; không lấy năm
   trong tên tệp để ghi năm bài. Metadata mục 041, RPP và Piazzi kế thừa
   các đối chiếu nguồn chính thức đã ghi ở ver10.
7. **042/066/073:** chuẩn hóa tên tác giả theo PDF, giữ tên đầy đủ Việt Nam
   và họ kép Çekirdek Yavuz. Không suy tên tác giả từ tên tệp.
8. **067–070:** giữ nhãn snapshot Rolling 09/10/2026. Các nguồn này giải
   thích chức năng phần mềm, không chứng minh giá trị mặc định hoặc phiên
   bản Jazzy đã chạy. Tài liệu Constrained Jazzy và mã nguồn đo thời gian
   được nhận diện riêng.

Không đổi các tệp corpus hoặc manifest gốc. `references.json` lưu SHA-256
từng nguồn và phạm vi đọc có mục tiêu; `citation_map.json` lưu đầy đủ đoạn
trong bài sử dụng nguồn. Đây là kiểm chứng mức phát biểu được dẫn, không
phải systematic review hay đánh giá chất lượng toàn bộ từng công trình.

## Hàng rào khoa học giữ nguyên

- Quy hoạch động tối ưu trên đồ thị hữu hạn sau sàng lọc, không tối ưu
  toàn bộ đường liên tục hoặc thời gian thực thi nhiệm vụ.
- G² chỉ cho mối nối Bézier–đoạn thẳng chính quy; không cho quay tại chỗ
  hoặc toàn chuỗi tư thế rời rạc.
- Hình bao kiểm tra tại mẫu; không chứng minh an toàn liên tục hoặc thay
  phép đo khoảng hở trong chuyển động thực tế.
- 125 lượt khảo sát và 250 lượt kiểm chứng lặp là hai bộ riêng. Các hiệu
  chính là trung bình đều chênh lệch của 25 nhóm trên 118 cặp cùng đạt.
- Giữ các thất bại, giảm khoảng hở, tăng sai số đích và thời lượng xử lý.
  Không chứng minh ưu thế thành công, tương đương hay không kém hơn.
- Không quy riêng lợi ích cho quy hoạch động vì chưa có ablation cùng
  phiên bản; không suy thành tiết kiệm điện năng, rung tải hay năng suất kho.

## Kiểm tra

Script so sánh nguyên XML của 23 bảng/696 ô, 18 hình và 12 Office Math với
ver9; kiểm tra chín nhãn công thức, thứ tự và toàn bộ văn bản khoa học
nguồn sau khi hoàn nguyên số trích dẫn. Các ZIP part ngoài document.xml,
styles.xml, core.xml giữ nguyên byte. Kiểm tra hash ver9/ver10 và mọi
nguồn corpus, số dẫn theo lần xuất hiện, không nguồn thiếu/không được dẫn.
Các hiệu chính trong hai tóm tắt/kết luận đối chiếu bảng nguồn đã lưu.

Kết quả render và nhận xét từng trang thuộc `visual_review.json`; chỉ có
`quality_checks.json` với status `passed` khi hash khớp bản đã xem. Font
chính giữ Times New Roman; renderer dùng font toán/ký hiệu dự phòng cho
OMML/chỉ số Unicode của ver9 và chữ Ș, không đổi đối tượng khoa học để ép
toàn bộ glyph vào một font.

Trước khi nộp cần lựa chọn tạp chí và kiểm tra lại yêu cầu về độ dài,
ngôn ngữ, tác giả, dữ liệu và metadata final của các preprint nếu muốn thay
bằng bản xuất bản. Không tự thêm thông tin tác giả hoặc tuyên bố funding.
