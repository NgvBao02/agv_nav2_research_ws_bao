# KIỂM TRA TOÀN VĂN BÀI BÁO PSTMO VER2

## 1. Kết luận kiểm tra

Bản ver2 hiện đã đạt cấu trúc mà cô Lý yêu cầu và đã sửa các chỗ Thảo diễn giải quá mạnh hoặc sai phạm vi. Mạch chính của bài là:

`planner tạo Raw path → góc gãy còn cần hậu xử lý → các nhóm phương pháp hiện có và giới hạn → khoảng trống có điều kiện → PSTMO → mô hình/cổng khả thi/DP → giao thức ghép cặp → ba kịch bản → đánh đổi và giới hạn`.

Không còn dùng các lập luận sai như đồng nhất path với trajectory, gọi `Eκ` là năng lượng, suy `G²` thành jerk bằng không, xem Raw là smoother, tuyên bố bảo toàn orientation đầu, tuyên bố có biên clearance định lượng hoặc xem 15 nhóm ghép cặp là 15 lần lặp độc lập.

Không thể làm một bài thực nghiệm trở thành “không thể phản biện”; cách phòng thủ khoa học đúng là giới hạn kết luận đúng bằng chứng. Bản hiện tại đã làm điều đó: kết luận mạnh ở chất lượng hình học trong tập thử, nhưng chủ động không suy rộng sang robot thật, năng lượng, tracking, vật cản động hoặc tối ưu toàn cục.

## 2. Đối chiếu nhận xét của cô Lý và phần làm rõ của Thảo

| Yêu cầu | Cách thực hiện trong toàn văn | Trạng thái |
|---|---|---|
| Gộp Related Work vào Introduction | Tài liệu planner, smoothing hình học, biến dạng/tối ưu không gian và tối ưu không–thời gian được nối thành các đoạn văn, sau đó mới hình thành khoảng trống | Đạt |
| Giải thích đối chứng | Raw được xác định là mốc; Simple, Savitzky–Golay và Constrained đều có nguyên lý, vai trò và nguồn Nav2 [15]–[17] | Đạt |
| Thu gọn mô hình toán | Mô hình và phát biểu bài toán nằm ở II-A, không còn là một phần độc lập tách khỏi phương pháp | Đạt |
| Giải thích công thức | (1)–(5) mô tả động học/chỉ số/giới hạn; (6)–(7) xác định Bézier; (8) là ngân sách cạnh; (9) là truy hồi DP | Đạt |
| Điều kiện thử nghiệm đứng trước kết quả | III-A trình bày hệ thống, thông số, đối chứng và giao thức ghép cặp trước ba kịch bản | Đạt |
| Mỗi môi trường có dẫn nhập riêng | III-C, III-D và III-E lần lượt có mục đích, start–goal, hình, bảng và phần diễn giải riêng | Đạt |
| Nêu phạm vi và hạn chế | Phạm vi có ngay cuối Introduction; giới hạn được phân tích tại III-F và nhắc lại ở Conclusion | Đạt |
| Không biến Nav2 thành controller | Bài dùng đúng chuỗi `global planner → Smoother Server → controller` trong Nav2 | Đạt |
| Không đưa RRT/RRT* chỉ để đủ số bài | Introduction chỉ giữ các planner cần cho thiết kế thí nghiệm và mạch lập luận | Đạt |
| Không tuyên bố thời gian tối thiểu/khả năng nhúng | PSTMO được báo cáo chậm hơn các đối chứng; phần nhúng không được tuyên bố | Đạt |

## 3. Kiểm tra logic phương pháp với mã triển khai

### 3.1. Bézier bậc năm và tính liên tục

Với sáu điểm điều khiển trong (6):

- `B′(0)=5qu` và `B′(1)=5qw`, nên tiếp tuyến đầu/cuối cùng hướng với hai cạnh thẳng;
- `B″(0)=20(P₂−2P₁+P₀)=0` và `B″(1)=20(P₅−2P₄+P₃)=0`;
- do hai đạo hàm bậc nhất không suy biến và đạo hàm bậc hai bằng không tại đầu mút, độ cong đầu/cuối bằng không;
- vì đoạn thẳng kề có độ cong bằng không, kết luận nối `G²` là hợp lý.

Bài chỉ kết luận liên tục độ cong hình học ở mối nối. Bài không suy rằng jerk theo thời gian bằng không.

### 3.2. Cổng khả thi

Mô tả đã được đối chiếu với `quintic_transition.cpp`, `time_parameterization.cpp`, `footprint_safety.cpp` và `adaptive_pivot_g2_smoother.cpp`:

- kiểm tra đạo hàm hữu hạn, dấu độ cong và không cho bánh trong phải quay ngược trong chuyển tiếp tịnh tiến;
- giới hạn vận tốc tịnh tiến, vận tốc góc, vận tốc từng bánh và gia tốc ngang;
- profile vận tốc nội bộ kiểm tra gia tốc dọc, giảm tốc dọc và gia tốc góc;
- vùng quét hình bao được lấy mẫu và kiểm tra trên costmap;
- profile nội bộ chỉ là cổng kiểm tra; đầu ra vẫn là `nav_msgs/Path`, không phải trajectory được tham số hóa theo thời gian.

Bộ kiểm thử `adaptive_pivot_g2` đã được chạy trong môi trường ROS 2 Jazzy: 14/14 phép kiểm thử đạt, gồm kiểm thử chuyển tiếp quintic, tham số hóa thời gian, chọn ứng viên, tìm kiếm hình dạng phân cấp, tối ưu trạng thái, điều kiện hóa đường và line-of-sight.

### 3.3. Lựa chọn toàn đường và hậu kiểm

- Điều kiện (8) đúng với phép kiểm tra `dᵢ+dᵢ₊₁+m≤Lᵢ` trong mã.
- Truy hồi (9) đúng với quy hoạch động một trạng thái mỗi góc; với tối đa `K` trạng thái/góc, vòng lặp chuyển trạng thái có độ phức tạp `O(NK²)`.
- Đầu ra bảo toàn vị trí đầu, vị trí đích và quaternion orientation đích. Orientation đầu được dựng theo đoạn chuyển động đầu.
- Sau ghép, mã kiểm tra lại đầu mút, profile chuyển động khả thi và swept footprint; khi vi phạm, smoother báo thất bại thay vì trả đường không hợp lệ.

## 4. Kiểm tra số liệu

Nguồn số liệu là `docs/pstmo_bao_cao_toan_dien_assets/benchmark_hinh_hoc_175_luot.csv`. Bản ver2 chỉ lấy ba môi trường đã nêu trong bài và năm phương án đã nêu, tổng cộng 75 dòng:

- `3 môi trường × 5 planner = 15` nhóm ghép cặp;
- mỗi nhóm có đúng 5 dòng: Raw, Simple, Savitzky–Golay, Constrained và PSTMO;
- cả 75 dòng đều có `success=True`;
- không có phép nhân số mẫu bằng cách gọi năm phương pháp trong cùng một nhóm là năm lần lặp độc lập.

| Môi trường | Phương pháp | L (m) | Eκ (m⁻¹) | T (ms) |
|---|---|---:|---:|---:|
| Không gian mở | PSTMO | 4,224 | 1,954 | 54,0 |
| Lối đi hẹp | PSTMO | 14,169 | 4,326 | 154,2 |
| Kho có lối giao cắt | PSTMO | 7,541 | 2,380 | 79,8 |
| Toàn bộ 15 nhóm | Raw | 8,818 | 166,224 | 4,8 |
| Toàn bộ 15 nhóm | Simple | 8,730 | 11,744 | 1,0 |
| Toàn bộ 15 nhóm | Savitzky–Golay | 8,787 | 32,286 | 0,2 |
| Toàn bộ 15 nhóm | Constrained | 8,822 | 31,213 | 19,0 |
| Toàn bộ 15 nhóm | PSTMO | 8,645 | 2,887 | 96,0 |

Các tỷ lệ trong Tóm tắt, III-F và Kết luận đã được tính lại từ giá trị trung bình chưa làm tròn:

- giảm `Eκ` so với Raw/Simple/Savitzky–Golay/Constrained: `98,26% / 75,42% / 91,06% / 90,75%`;
- giảm `L`: `1,97% / 0,98% / 1,62% / 2,01%`;
- vì PSTMO có `T=96,0 ms`, bài chỉ gọi đây là đánh đổi chi phí xử lý, không gọi là nhanh nhất.

## 5. Kiểm tra đối chứng và phạm vi kết luận

- Cấu hình Constrained thực tế là `w_smooth=200000`, `w_cost=0,015`, `w_curve=w_dist=0`. Vì vậy bài không diễn giải như thể Constrained đã tối ưu trực tiếp `Eκ`.
- Raw không gồm thời gian global planning; giá trị Raw chỉ là chi phí chuyển tiếp/sao chép trong cùng bộ đo.
- Thời gian dưới lượng tử khoảng 3 ms được mô tả là dưới độ phân giải đáng tin cậy, không phải chi phí bằng không.
- Chiều dài PSTMO nhỏ hơn không được dùng để tuyên bố tìm tuyến toàn cục tốt hơn; planner vẫn quyết định thứ tự hành lang.
- Không phát hiện va chạm theo footprint/costmap không được đổi thành tuyên bố có clearance tối thiểu định lượng.
- `Eκ` là chỉ số độ uốn hình học, không đại diện trực tiếp cho năng lượng, tracking error hoặc tải động lực học.

## 6. Kiểm tra tài liệu tham khảo

- Danh mục trong bài có đủ [1]–[23] và mỗi mục đều được gọi trong Introduction.
- Thư mục `REFERENCES` có 18 PDF mở được bằng `pdfinfo`.
- Hai bản quét khó trích chữ là Hart–Nilsson–Raphael [3] và Quinlan–Khatib [13] đã được đối chiếu trực quan trang đầu, đúng tên bài và tác giả.
- Yang–Sukkarieh [9] và Bu et al. [10] chưa có PDF nội bộ hợp lệ. Tệp từng mang tên [9] nhưng thực tế chứa sai/thiếu nội dung đã bị loại, không dùng một PDF khác để lấp chỗ trống.
- Simple [15] và Constrained [17] không có bài báo PDF riêng mô tả đúng plugin Nav2 hiện tại; bài dùng tài liệu và mã nguồn chính thức. Nền tảng toán của Savitzky–Golay [16] có PDF [14].

Chi tiết tên tệp và liên kết nằm trong `../../REFERENCES/TAI_LIEU_THAM_KHAO_VER2.md`.

## 7. Kiểm tra hình thức

- DOCX mở được và cấu trúc ZIP/OOXML không có lỗi.
- PDF là A4, 6 trang, thấp hơn giới hạn 8 trang.
- Có đúng 8 nhãn Hình 1–8 và 8 ảnh nhúng; không có Hình 9 hoặc Hình 10.
- Tám ảnh trong `assets/ver1_hinh_1.png`–`ver1_hinh_8.png` giống từng byte với tám ảnh tương ứng trích từ PDF ver1.
- `PSTMO_ver2_tat_ca_hinh.drawio` có đúng 8 trang tên Hình 1–8; trang tổng hợp phụ đã được bỏ để không tạo cảm giác có hình thứ chín.
- Có chín phương trình được đánh số liên tục (1)–(9), sáu bảng I–VI và 23 mục tham khảo.

## 8. Phần còn dành cho tác giả

Theo yêu cầu mới nhất, không chỉnh nội dung hình thêm. Tám vị trí hình và chú thích đã được giữ cố định để tác giả thay hình sau nếu cần. Trước khi nộp chính thức, tác giả chỉ cần kiểm tra lại chất lượng/chữ trong Hình 1–8, thông tin tác giả liên hệ và chuyển bản tiếng Việt sang ngôn ngữ nộp nếu hội nghị yêu cầu tiếng Anh.
