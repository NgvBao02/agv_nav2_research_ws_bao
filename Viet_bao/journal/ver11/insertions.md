# Các đoạn bổ sung vào thân ver9

@before Chiều dài cắt d là khoảng cách

Biểu diễn Bézier dựa trên cơ sở Bernstein [R026]; B-spline [R027] và NURBS [R028] cung cấp những lựa chọn khác cho thiết kế đường cong. PSTMO dùng một đoạn đa thức bậc năm tại mỗi góc để áp điều kiện đầu mút bằng điểm điều khiển, không tối ưu một spline trên toàn tuyến. Các xoắn ốc Bézier bậc năm Pythagorean-hodograph có những tính chất riêng về độ dài cung và chuyển tiếp [R032]; đường bậc năm ở đây không được giả định thuộc lớp PH. Việc chọn bậc và cơ sở biểu diễn vì vậy tách biệt với các tính chất cần chứng minh cho đoạn cụ thể.

@before Công thức 3 xác định độ cong

Tối ưu hình dạng dưới ràng buộc độ cong có tiền lệ trong phương pháp biến dạng đường của Delingette và cộng sự [R046]. Trong PSTMO, tích phân bình phương độ cong được dùng như một tiêu chí hình học khi tìm hình dạng và một thành phần chi phí sau chuẩn hóa. Vai trò này không đồng nhất với tối thiểu thời gian hoặc năng lượng cơ học.

@before Với khoảng cách hai bánh b

Mô hình hai bánh chủ động độc lập cho phép biểu diễn vận tốc thân xe theo vận tốc từng bánh [R014]. Các đường Dubins [R019], phân loại hình học đường ngắn nhất bị chặn độ cong [R021] và đường Reeds–Shepp có tiến–lùi [R020] thuộc những bài toán khác với chuyển động quay tại chỗ của robot vi sai. PSTMO không áp một bán kính quay dương chung cho mọi trạng thái: các giới hạn theo độ cong dưới đây áp cho phần tịnh tiến, còn thao tác quay được mô hình hóa riêng.

@before Gọi Sᵢ là tập phương án

Bài toán lựa chọn dùng nguyên lý tối ưu theo giai đoạn của quy hoạch động [R013]. Sau khi tập ứng viên đã cố định, ảnh hưởng của lựa chọn tại một góc đến góc kế tiếp được biểu diễn bằng chiều dài chiếm dụng trên cạnh chung. Điều này cho phép lưu chi phí tốt nhất đến mỗi trạng thái thay vì liệt kê mọi chuỗi tổ hợp. Phát biểu tối ưu chỉ có ý nghĩa với chi phí cộng và quan hệ tương thích đã định nghĩa, không bao gồm sai số điều khiển phát sinh khi thực thi.

@before Thử nghiệm sử dụng robot vi sai

Kiến trúc Nav2 đã được trình bày trong The Marathon 2 [R062]; tổng quan ROS 2 của Macenski và cộng sự [R074] đặt các thuật toán điều hướng trong hệ phần mềm có nhiều mô-đun. Nghiên cứu này kế thừa kiến trúc đó để thay đổi riêng tầng hậu xử lý khi so sánh. Các nguồn kiến trúc không được dùng để gán phiên bản thư viện hoặc cấu hình cho những lượt thử; những thông tin này phải lấy từ hồ sơ của bộ dữ liệu.

@before Năm phương án gồm đường gốc

Tài liệu Smoother Server mô tả giao diện tác vụ và cơ chế nạp nhiều plugin cùng dùng tài nguyên bản đồ, hình bao và biến đổi tọa độ [R067]. Ở cấp thuật toán, Simple cân bằng độ bám đường đầu vào với giảm dao động [R068]; Savitzky–Golay sử dụng lọc đa thức cục bộ để xử lý các nhiễu nhỏ của đường [R069]; Constrained tối ưu nhiều thành phần liên quan độ trơn, vật cản và độ cong [R070]. Bốn nguồn này là snapshot tài liệu Rolling, dùng để giải thích vai trò các thành phần, không thay thế cấu hình Jazzy trong Bảng IV hoặc chứng minh các giá trị mặc định đã được dùng. Nền tảng bộ lọc và tài liệu Jazzy của đối chứng được trích riêng dưới đây.

@before Lợi ích hình học và thực thi của PSTMO đi kèm

Mở rộng sang môi trường động cần giải quyết thêm thông tin dự đoán vật cản và cập nhật quỹ đạo. Ahn và cộng sự [R034] tích hợp dự đoán với trường thế nhân tạo và Bézier cho xe tự động; Xu và cộng sự [R040] dùng thị giác và tối ưu B-spline để tránh vật cản động cho UAV. Hai ví dụ thuộc mô hình phương tiện khác, nên chỉ chỉ ra những yêu cầu nghiên cứu bổ sung, không chứng minh PSTMO sẽ giữ hiệu quả khi chuyển sang môi trường động. Khảo sát hiện tại chưa đánh giá các thành phần cảm nhận và dự đoán đó.
