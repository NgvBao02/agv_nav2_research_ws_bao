# Ghi nhận thay đổi từ journal ver5 sang ver6

## Nội dung bổ sung

1. Giữ nguyên phần II và hệ thống công thức; tiếp tục dùng cách diễn đạt đã
   thống nhất ở ver5, không đưa tên trường dữ liệu hoặc mã kiểm tra vào bài.
2. Mỗi tuyến R01–R05 có hai bảng cho cả năm phương án: bảng kết quả chính
   và bảng chiều dài, độ cong cực đại, quãng đường thực thi, sai số cực đại,
   thời lượng hậu xử lý. Đoạn phân tích bổ sung độ cong thực thi, phân vị
   độ giật, sai số vị trí và hướng cuối của cả năm phương án.
3. Mục III-G có so sánh ghép cặp với cả bốn đối chứng, không chỉ Constrained
   hoặc Simple. Nêu rõ 23 cặp với đường gốc và 24 cặp với mỗi phương án làm
   mượt khi chỉ xét các cặp cùng hoàn thành.
4. Bổ sung bảng phần tịnh tiến sau lấy mẫu lại, số điểm, dấu quay tại chỗ,
   chỉ số chuyển động, sai số cuối, thời lượng xử lý và kiểm tra an toàn.
   Phân biệt cực đại toàn bộ lượt với trung bình trên lượt đạt.
5. Vẽ lại hình tổng hợp đối chứng và hình diễn biến vận tốc góc để có đủ
   năm phương án. Chỉ dùng bản ghi có sẵn; không thêm số liệu suy đoán.
6. Giữ kiểm chứng 250 lượt PSTMO–Constrained ở mục III-H. Mục III-I bổ
   sung hai lượt không đạt của ma trận 125 lượt vào bảng phân tích lỗi;
   giữ riêng mẫu số của hai bộ dữ liệu.

## Những điểm được làm rõ

- Năm bộ lập kế hoạch là năm đầu vào khác nhau, không phải năm lần lặp.
- Một phương án có thể nhanh hơn trung bình nhưng không nhanh hơn trên
  đa số đầu vào; cũng có thể giảm độ cong nhưng tăng RMSE hoặc độ giật.
- R03: PSTMO có độ cong cực đại tham chiếu lớn hơn Simple và độ giật cao
  hơn các đối chứng trong tập lượt đạt. Không kết luận ưu thế toàn diện.
- R04/R05: sai số vị trí cuối của PSTMO cao nhất ở mức trung bình lượt
  đạt, dù một số chỉ số bám đường và hình học tốt hơn.
- Trung bình quãng đường thực thi gộp và chênh lệch ghép cặp có thể khác
  dấu vì số lượt đạt khác nhau; không dùng hai đại lượng thay thế nhau.
- Thời lượng 0 ms của SG không có nghĩa không tốn thời gian tính toán.
- Vận tốc bánh là đại lượng tương đương suy từ vận tốc thân xe; các cực
  đại lấy mẫu không chứng minh ràng buộc liên tục hoặc an toàn tuyệt đối.

## Kiểm chứng và giới hạn bàn giao

Đã dùng lại dữ liệu 125 lượt và kết quả phân tích 250 lượt hiện hữu, không
chạy ROS/Gazebo. Bộ dựng kiểm tra tính đầy đủ của dữ liệu và tính lại chỉ
số chuyển động cùng độ cong thực thi từ các chuỗi được lưu. Bộ kiểm tra
độc lập đối chiếu số liệu bảng và toàn bộ thống kê ghép cặp dẫn xuất.

Ver5, các bản cũ và dữ liệu gốc được giữ nguyên. Thay đổi mã chỉ nằm trong
bộ dựng/kiểm tra tài liệu ver6; không sửa thuật toán, plugin hay cấu hình.
README gần nhất và danh mục báo cáo trong README gốc được cập nhật.
Không có thay đổi quy trình dữ liệu hoặc nguồn sự thật nên không cần sửa
AGENTS, tài liệu agent hay README của tập kết quả.
