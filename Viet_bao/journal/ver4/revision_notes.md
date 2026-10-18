# Ghi nhận biên tập journal phiên bản 4

## Những ưu điểm tiếp thu từ REV-ECIT ver5

- Mở đoạn bằng đối tượng, kết quả hoặc vấn đề cần giải thích.
- Tách câu nhiều mệnh đề; thay lời dẫn dài bằng quan hệ trực tiếp giữa số
  liệu, nhận xét và phạm vi kết luận.
- Dùng nhất quán đường tham chiếu, quỹ đạo thực, thời gian thực thi, sai số
  bám đường, khoảng hở và thời lượng hậu xử lý.
- Phần kết quả theo tuyến đi từ mục tiêu, hình học đến số liệu và đánh đổi.
- Phần thuật toán nêu đầu vào, đầu ra, cơ chế lựa chọn và điều kiện chấp nhận.

## Những nội dung không sao chép

Không lấy số liệu, tỷ lệ cải thiện hoặc cơ chế điều khiển trong REV-ECIT làm
bằng chứng cho journal. Bản journal tiếp tục dùng 125 lượt khảo sát và 250
lượt kiểm chứng lặp. Không thu gọn thiết kế thống kê thành cách trình bày
của bài hội nghị sáu trang, không bỏ các kết quả bất lợi để tăng tính thuyết phục.

## Cải thiện phân tích và trình bày

- Đối chiếu PSTMO với đường gốc, Simple, Savitzky–Golay và Constrained theo
  từng cặp cùng đạt; không chỉ so sánh các trung bình khác tập mẫu.
- Làm rõ ở R02 và R04: trung bình thời gian của PSTMO thấp hơn Simple nhưng
  số cặp nhanh hơn chỉ là 2/5. Nêu các ngoại lệ về RMSE và thời gian.
- Phân biệt tác động hình học với tác động thực thi dưới cấu hình RPP tắt
  điều tiết vận tốc theo độ cong và chi phí.
- Giữ phân tích thống kê theo 25 khối và ba chỉ số chính, khoảng tin cậy,
  hiệu chỉnh Holm, điều kiện cùng đạt và các giới hạn suy rộng.
- Phân biệt lượt kết thúc trước đích với lượt hoàn tất tác vụ nhưng vượt
  dung sai vị trí; không đồng nhất kết thúc lượt với robot dừng hẳn.
- Vẽ lại hình theo nhãn thống nhất, tăng khả năng đọc các hình nhiều ô;
  thêm nhãn điểm nối và kích thước cắt trong hình Bézier bậc năm.
- Dùng chỉ số dưới trong Word, dấu phẩy thập phân ở ô số, ký hiệu khoa học
  cho giá trị p; giữ dấu chấm phân cách hàng nghìn trong mô tả tham số.
- Giữ đủ 20 bảng, 19 hình và 9 công thức, bố trí hình gần phần phân tích;
  không dùng định danh trong mã nguồn làm thuật ngữ bài báo.

## Bảo toàn và kiểm chứng

Không thay đổi mã nguồn ROS/Nav2, cấu hình chạy hoặc dataset. Hai bản Word
tham chiếu và nguồn ver3 giữ nguyên. Bộ dựng ver4 và bộ kiểm tra riêng cho
phép tái tạo bản thảo, truy xuất nguồn hình và đối chiếu số liệu. Các kết quả
kiểm tra cụ thể được lưu trong `quality_checks.json`; không xem đây là xác
nhận một tạp chí sẽ chấp nhận bài.
