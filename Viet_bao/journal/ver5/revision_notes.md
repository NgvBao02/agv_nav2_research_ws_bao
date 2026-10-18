# Ghi nhận biên tập journal phiên bản 5

Ngày biên tập: 17/10/2026. Bản cơ sở là journal ver4, không phải REV-ECIT
ver5. Không sửa bản cơ sở, mã nguồn ROS, cấu hình hay dữ liệu thực nghiệm.

## Nội dung đã sửa

- Chuẩn hóa thuật ngữ trong toàn bộ phần II và III, bảng và chú thích hình;
  bảng đối chiếu chi tiết ở [terminology.md](terminology.md).
- Phân biệt hình chiếu bằng, đường bao, đường gốc, đường tham chiếu và quỹ
  đạo thực thi trong mô phỏng. Định nghĩa chiều dài cắt và nhóm phân tích.
- Viết lại các đoạn về lựa chọn đường cong, kiểm tra thời gian, quy hoạch
  động và kiểm tra đầu ra; giữ nguyên chín công thức đánh số.
- Thay tên gọi năng lượng độ cong bằng tích phân bình phương độ cong,
  giải thích phép xấp xỉ rời rạc và giới hạn diễn giải.
- Sửa các câu có thể nhầm số liệu của đối chứng thành mức chênh lệch, đặc
  biệt R01, R02, R03 và R05. Không đổi giá trị báo cáo.
- Sửa câu kiểm tra khả năng đi qua vật cản thành kiểm tra di chuyển không
  giao cắt vật cản. Làm rõ giới hạn lấy mẫu của kiểm tra va chạm.
- Bảng cấu hình Constrained ghi rõ trọng số khoảng cách và độ cong bằng
  không; bảng PSTMO ghi đúng trọng số vận tốc góc thay cho nhãn góc.
- Bỏ cách ghi tập tham số theo cú pháp dấu hai chấm, dùng tập giá trị rõ
  ràng. Tọa độ trong bảng dùng dấu phẩy thập phân và dấu chấm phẩy phân cách.
- Định dạng chỉ số dưới trong chú thích hình giống thân bài và bảng.
- Giữ cách dẫn chiếu công thức bằng từ “công thức” kèm số. Giữ tên tuyến
  trung tính và toàn bộ so sánh với các phương án đối chứng.
- Bố trí lại một số bảng và đoạn phân tích cạnh hình để giảm khoảng trắng;
  giữ ghi chú cùng bảng, không đổi thứ tự đánh số hình hoặc bảng.

## Những nội dung không thay đổi

Giữ 19 hình, 20 bảng, 9 công thức, 7 tài liệu tham khảo cục bộ và các nguồn
dữ liệu của ver4. Không bổ sung thí nghiệm hay thay số liệu. Ma trận khảo sát
vẫn có 123/125 lượt đạt; bộ kiểm chứng có 243/250 lượt đạt, 125 cặp cùng đầu
vào và 118 cặp có cả hai phương án đạt. Giữ các trường hợp thất bại và các
đánh đổi về khoảng hở, sai số đích, thời lượng hậu xử lý.

Giữ khổ A4, hai cột, lề, phông chữ và cỡ chữ của bản cơ sở. Không dùng
giới hạn sáu trang của REV-ECIT để cắt nội dung journal. Hồ sơ kiểm tra cuối
nằm ở [quality_checks.json](quality_checks.json).

Bản cuối được kiểm tra trực quan đủ 15 trang. Phần II khoảng 2,2 trang,
phần III khoảng 12,5 trang. Kiểm tra số liệu bảng, tọa độ, toàn bộ giá trị
thập phân trong nội dung, XML công thức, tài liệu tham khảo và 145 dấu kiểm
tra nguồn đều đạt. Kiểm tra tài liệu repository và khoảng trắng đạt.

## Phạm vi bàn giao

Đây là phần II và III đã biên tập, chưa phải toàn bộ bài journal. Bảy tài
liệu tham khảo cần được hợp nhất và đánh số lại khi ghép toàn bài. Mẫu trình
bày kế thừa IEEE conference A4, chưa thay thế kiểm tra quy định của tạp chí
được lựa chọn. Biên tập ngôn ngữ không bảo đảm bài được chấp nhận.
