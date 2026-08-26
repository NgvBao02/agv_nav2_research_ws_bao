# NHẬN XÉT DÀN Ý VER2 SAU KHI ĐỐI CHIẾU BÀI BÁO VER1

## 1. Cách hiểu tài liệu và kết luận chung

Quan hệ giữa ba phần tài liệu được hiểu như sau:

1. `ICEEIS 2026 PSTMO tieng viet ver1.pdf` là bản báo cáo mà cô Lý nhận xét.
2. Khối “ver2 cô Lý” là yêu cầu sửa cấu trúc và cách lập luận của bản ver1.
3. Khối “ver2 Thảo” là cách Thảo làm rõ và cụ thể hóa yêu cầu của cô Lý; vì vậy phần của Thảo phải được đánh giá theo hai tầng: có hiểu đúng lời cô Lý không, và cách cụ thể hóa đó có đúng với thuật toán, số liệu và phạm vi bằng chứng của ver1 không.

Kết luận tổng quát: hướng sửa của cô Lý hợp lý và giải quyết đúng các vấn đề lớn của ver1, đặc biệt là mạch lập luận của phần mở đầu, vị trí của mô hình toán, lý do chọn đối chứng và cách trình bày thử nghiệm. Bản của Thảo đã hiểu đúng khung lớn nhưng chưa thể dùng trực tiếp để viết bài. Một số câu Thảo bổ sung làm lệch thuật ngữ vốn đã đúng trong ver1, một số khẳng định vượt quá bằng chứng, cách phân nhóm tài liệu chưa nhất quán và thiết kế thực nghiệm của ver1 chưa được phản ánh đầy đủ. Vì vậy cần giữ cấu trúc mà Thảo đề xuất, nhưng phải sửa nội dung theo bản hiệu chỉnh kèm theo tài liệu này.

Phạm vi nhận xét này không xem hệ thuật ngữ chuyên ngành hay nội dung phương pháp trong ver1 là phần cần sửa. Ver1 được dùng làm chuẩn về tên gọi, ký hiệu và bản chất thuật toán. Phần cần sửa của ver1 là cách viết và logic tổ chức toàn bài; các hiệu chỉnh thuật ngữ nêu dưới đây chỉ áp dụng cho những chỗ phần diễn giải của Thảo vô tình khác hoặc mạnh hơn ver1.

## 2. Những điểm cô Lý nhận xét đúng và nên thực hiện

### 2.1. Gộp tổng quan nghiên cứu vào Introduction

Ver1 có Phần I Giới thiệu và Phần II Các nghiên cứu liên quan tách biệt. Với một bài hội nghị ngắn khoảng bảy trang, Phần II hiện chiếm nhiều dung lượng và làm mạch lập luận bị ngắt: vấn đề đã được nêu ở cuối Phần I, sau đó bài quay lại trình bày lịch sử phương pháp khá dài, rồi mới trở lại khoảng trống nghiên cứu ở cuối Phần II.

Yêu cầu của cô Lý là hợp lý: đưa các nghiên cứu liên quan vào Introduction dưới dạng văn viết, theo chuỗi “bối cảnh hệ thống → các bộ lập kế hoạch tạo đầu vào gì → vì sao còn cần làm mượt → các hướng làm mượt đã giải quyết gì → còn thiếu gì → PSTMO giải quyết phần nào”. Cách này làm khoảng trống nghiên cứu xuất hiện như kết quả của lập luận, không phải một tuyên bố tách rời.

Tuy nhiên, “lồng ghép” không có nghĩa liệt kê ba nhóm bằng các gạch đầu dòng trong bản bài báo. Các nhãn “Xu hướng 1, 2, 3” chỉ nên dùng ở dàn ý; khi viết bài phải chuyển thành các đoạn văn nối tiếp và có câu chuyển ý.

### 2.2. Nêu vai trò và nguồn gốc của các phương án đối chứng

Cô Lý yêu cầu không chỉ ghi tên Simple, Savitzky–Golay và Constrained mà phải nói chúng dựa trên nguyên lý nào, vì sao có tính đại diện và vì sao được chọn làm nền so sánh. Điều này hoàn toàn đúng. Ver1 mới mô tả các cấu hình trong Bảng I, nhưng phần tổng quan chưa nối rõ các phương pháp nghiên cứu gốc với đúng ba plugin đối chứng.

Cách viết chính xác nên là:

- Simple là bộ làm mượt lặp cục bộ, cân bằng độ bám dữ liệu gốc và độ trơn giữa các điểm lân cận. Không gọi là “đã thương mại hóa”; nên gọi là plugin mã nguồn mở được tích hợp chính thức trong hệ sinh thái Navigation2.
- Savitzky–Golay bắt nguồn từ phép xấp xỉ đa thức bình phương tối thiểu cục bộ; cấu hình ver1 dùng bộ lọc bảy điểm `[-2, 3, 6, 7, 6, 3, -2]/21` và hai lần tinh chỉnh.
- Constrained là bộ tối ưu dựa trên ràng buộc, dùng Ceres và có thể tối ưu độ dài, độ trơn, chi phí vật cản và độ cong. Phải nêu rõ trong đúng cấu hình ver1, `w_curve = 0` và `w_dist = 0`, nên không được diễn giải kết quả như thể Constrained đã được hiệu chỉnh để tối thiểu trực tiếp chỉ số độ cong.
- Raw phải được giữ trong thiết kế thí nghiệm với vai trò mốc đường đầu vào, dù Raw không phải bộ làm mượt.

Tài liệu chính thức của Nav2 xác nhận Simple, Savitzky–Golay và Constrained là ba plugin smoother; nhiệm vụ của smoother là nhận một path và trả về phiên bản được cải thiện. Điều này cũng cho thấy cách gọi “trả đường về cho Nav” hoặc coi Nav2 là bộ điều khiển cục bộ là không chính xác về kiến trúc.

### 2.3. Thu gọn mô hình toán vào phần phương pháp

Ver1 dành hẳn Phần III cho mô hình toán và phát biểu bài toán, sau đó Phần IV mới trình bày PSTMO. Việc cô Lý yêu cầu đưa mô hình toán thành một tiểu mục của phần phương pháp giúp bài gọn hơn và tránh lặp lại đầu vào, ràng buộc giữa hai phần.

Nhưng không nên cắt mô hình toán đến mức không còn khả năng kiểm chứng. Bài vẫn phải giữ các công thức cốt lõi để chứng minh ba tuyên bố chính: mối nối (G^2), khả thi động học robot vi sai và an toàn theo hình bao. Những chi tiết cài đặt như số điểm lưới tìm kiếm, tìm thô–tinh, điều kiện dừng hoặc mọi trường hợp loại ứng viên có thể lược bỏ.

Yêu cầu “mỗi phương trình phải giải thích các thành phần và chức năng” là rất đúng. Ver2 cần giải thích ngay sau công thức, không gom ký hiệu thành một đoạn xa công thức.

### 2.4. Đưa điều kiện thử nghiệm lên trước và dẫn nhập từng bảng/hình

Ver1 trình bày bảng phương án so sánh trước bảng thông số hệ thống. Cô Lý yêu cầu điều kiện thử nghiệm được nêu trước là hợp lý vì người đọc phải biết nền tảng, robot, costmap và giới hạn chuyển động trước khi đánh giá kết quả.

Yêu cầu mô tả từng tình huống rồi mới đặt ảnh cũng đúng. Ba Hình 6–8 hiện được đặt gần nhau, trong khi phần mô tả ba môi trường được gộp trong một đoạn. Bản mới nên có ba tiểu mục hoặc ít nhất ba cụm đoạn độc lập:

1. mô tả mục đích và hình học của môi trường;
2. nêu cặp start–goal và điều muốn quan sát;
3. đặt hình minh họa của môi trường đó;
4. giới thiệu bảng số liệu tương ứng;
5. phân tích kết quả và đánh đổi, rồi mới chuyển sang môi trường kế tiếp.

### 2.5. Khoanh vùng phạm vi và nêu hạn chế

Cô Lý đúng khi yêu cầu ngay trong Introduction phải nói rõ phạm vi của bài: đề xuất một bộ hậu xử lý đường trong ROS 2 Jazzy/Nav2, đánh giá mô phỏng trên ba môi trường tĩnh quy mô nhỏ và so sánh trong cùng điều kiện. Điều này ngăn người đọc hiểu nhầm rằng bài đã chứng minh hiệu quả trên nhà kho lớn, robot thật, vật cản động hoặc nền tảng nhúng.

Phần hạn chế phải được nhắc lại ở kết luận: ba bản đồ, một cặp start–goal mỗi bản đồ, năm nguồn đường, một lần chạy mỗi tổ hợp, chưa có robot thật, chưa đo sai số bám, năng lượng hay độ bền trước sai số mô hình.

## 3. Những chỗ trong lời cô Lý cần hiểu linh hoạt, không nên làm máy móc

### 3.1. “Khoảng 10 bài” về lập kế hoạch không nên biến Introduction thành tổng quan planner

Ý của cô Lý là phải tạo đủ nền để dẫn đến vấn đề đường gấp khúc, không phải dành đúng mười tài liệu và mô tả dài từng thuật toán. Đóng góp của bài nằm ở path smoothing, nên phần Dijkstra/A*/Theta*/Smac chỉ cần đủ để phân biệt: tìm đường tối ưu trên đồ thị/lưới, tìm đường any-angle và tìm đường có xét trạng thái/động học. Phần lớn dung lượng tham khảo vẫn nên dành cho làm mượt hình học, biến dạng đường và tối ưu quỹ đạo.

Khuyến nghị thực tế là khoảng 20–22 tài liệu toàn bài, gồm 4–6 tài liệu về kiến trúc/planner, 10–12 tài liệu về smoothing và 4–5 tài liệu về trajectory optimization hoặc phần mềm đối chứng. Không nên đưa RRT/RRT* vào chỉ để tăng số lượng nếu chúng không tham gia thí nghiệm và không giúp trực tiếp hình thành khoảng trống nghiên cứu.

### 3.2. “Không cần đưa chi tiết con số trong kết luận” không có nghĩa bỏ hết số liệu

Kết luận không nên lặp lại toàn bộ các bảng, nhưng vẫn cần một hoặc hai con số đại diện để chứng minh tuyên bố. Với ver1, cặp số hợp lý nhất là mức giảm (E_\kappa) 75,42% so với Simple và 90,75% so với Constrained, kèm đánh đổi thời gian xử lý trung bình 96,0 ms. Nếu chỉ viết “tốt hơn” hoặc “vượt trội” thì kết luận thiếu sức nặng và dễ bị xem là quảng bá.

### 3.3. “Chưa tìm được giới hạn” phải viết thành giới hạn của bằng chứng

Không được viết rằng thuật toán “chưa có giới hạn”. Cách diễn đạt khoa học là: phạm vi thử nghiệm hiện tại chưa đủ để đặc trưng giới hạn làm việc hoặc khả năng khái quát của PSTMO. Hai mệnh đề này khác nhau: một bên là tuyên bố về thuật toán, bên kia là tuyên bố trung thực về bằng chứng.

### 3.4. Không nên hứa “sẽ được công bố trong bài báo sau”

Nên viết “các thử nghiệm này được dành cho nghiên cứu tiếp theo” hoặc “future work will evaluate…”. Không nên cam kết một công bố tương lai khi chưa có lịch và kết quả.

## 4. Đánh giá chi tiết phần Thảo làm rõ nhận xét cô Lý

| Nội dung cô Lý yêu cầu | Cách Thảo làm rõ | Đánh giá | Cách sửa bắt buộc |
|---|---|---|---|
| Đưa bối cảnh điều khiển AGV/AMR và các planner vào Introduction | Chia hệ thống thành “global path planning” và “local trajectory tracking/Nav2” | Đúng ý lớn nhưng sai thuật ngữ: Nav2 là cả khung điều hướng, không phải tên của tầng điều khiển cục bộ; đồng thời còn thiếu tầng smoother | Viết chuỗi `global planner → path smoother → controller` trong Nav2 |
| Giải thích Dijkstra, A*, Theta*… làm gì | Gom Dijkstra, A*, Theta*, RRT/RRT* thành cùng nhóm và nói cùng tối ưu độ dài | Quá khái quát; RRT không phải grid search, RRT không bảo đảm tối ưu độ dài, còn RRT* chỉ tối ưu tiệm cận | Chia ngắn theo graph/grid, any-angle và state-aware; chỉ nêu thuật toán thực sự cần cho bài |
| Từ hạn chế planner dẫn đến nhu cầu smoothing | Nói đường là waypoint rời rạc, gấp khúc và không thỏa non-holonomic | Hướng lập luận đúng | Đổi “các thuật toán trên chỉ…” thành “các đầu ra dựa trên lưới có thể…” để tránh phủ định tuyệt đối SmacHybrid hoặc planner khả thi động học |
| Nêu hệ quả của góc gãy | Nói robot buộc giảm tốc/dừng, gây trượt bánh, tăng năng lượng và rung cơ khí nghiêm trọng | Vượt bằng chứng của ver1; ver1 không đo trượt, năng lượng hay rung | Chỉ nói controller có thể phải giảm tốc, lệch đường hoặc quay tại chỗ, ảnh hưởng khả năng bám và tính liên tục chuyển động |
| Gom các nghiên cứu thành khoảng ba xu hướng | Curves & Splines; Local Filtering & Direct Optimization; Trajectory Optimization | Khung ba nhóm dùng được | Chuyển Evolutionary/GRIPS/CHOMP vào nhóm tối ưu không gian; không coi DP là một xu hướng trajectory optimization vì DP ở đây là công cụ chọn ứng viên của chính PSTMO |
| Phân tích nhóm đường cong | Nêu Dubins/Reeds–Shepp, Clothoid, B-spline, Bézier | Có nhiều tên nhưng chưa gắn với tài liệu của ver1 | Ưu tiên Clothoid và Bézier vì có tài liệu trực tiếp; chỉ thêm Dubins/B-spline khi có tài liệu và mục đích rõ |
| Nêu hạn chế Clothoid | Nói Fresnel phức tạp, khó đáp ứng thời gian thực | Quá mạnh; đã có nghiên cứu clothoid làm mượt trực tuyến | Viết “thường cần tích phân Fresnel, xấp xỉ hoặc giải số; kiểm tra khoảng hở/footprint vẫn cần cơ chế bổ sung” |
| Nêu hạn chế Bézier/Spline | Nói bậc thấp khó bảo đảm (C^2) nếu không tăng bậc | Sai và mâu thuẫn với chính tài liệu cubic Bézier liên tục độ cong trong ver1 | Phân biệt (C^2) tham số và (G^2) hình học; nói tính liên tục phụ thuộc cấu trúc điểm điều khiển và điều kiện biên, không chỉ phụ thuộc bậc |
| Nêu nguồn gốc Simple, Savitzky–Golay, Constrained | Nêu Simple/SG và gọi là đã “chuẩn hóa và thương mại hóa”; không làm rõ Constrained | Chưa đạt yêu cầu | Gọi là plugin mã nguồn mở chính thức của Nav2; nêu nguyên lý và nguồn của cả ba; giữ Raw là mốc |
| Khoảng trống nghiên cứu | Ba yếu tố: (G^2), tương tác góc + footprint, thời gian tối thiểu cho nhúng | Hai yếu tố đầu đúng hướng; yếu tố thứ ba không phù hợp số liệu | Đổi thành “đánh giá rõ đánh đổi chất lượng hình học–chi phí xử lý trong một bộ hậu xử lý Nav2”; không tuyên bố nhúng |
| Ý nghĩa của (G^2) | “Triệt tiêu hiện tượng giật gia tốc góc” | Sai về mặt suy luận | (G^2) loại bước nhảy độ cong tại mối nối hình học; không tự bảo đảm jerk theo thời gian bằng 0 |
| Tên đầy đủ PSTMO | “Parametric Segment Transition & Multi-objective Optimization” | Sai so với ver1 và tiêu đề nghiên cứu | Dùng đúng: “Path Smoothing via Footprint-Aware Corner-Transition Optimization for Differential-Drive Mobile Robots” |
| Chỉ số (E_\kappa) | “Năng lượng biến thiên độ cong” | Sai; ver1 chủ động cảnh báo đây không phải năng lượng | Gọi là “tích phân bình phương độ cong” hoặc “chỉ số độ uốn hình học”, đơn vị m⁻¹ |
| Đối chứng trong mục tiêu | Chỉ nêu Simple và Savitzky–Golay | Thiếu | Phải nêu Raw, Simple, Savitzky–Golay và Constrained; khi so sánh thuật toán làm mượt thì Raw là mốc, ba phương pháp còn lại là smoother đối chứng |
| Quy hoạch động | Nói chọn tổ hợp ((\alpha_i,d_i)) | Chưa đủ | DP chọn trạng thái/ứng viên xử lý ở mỗi góc, có thể gồm giữ nguyên, quay tại chỗ hoặc đoạn chuyển tiếp với (d,\alpha); điều kiện tương thích dùng ngân sách cạnh chung |
| Ghép đầu ra | “Bảo toàn tuyệt đối tọa độ và góc hướng Start và Goal” | Sai với ver1 và mã nguồn | Chỉ khẳng định bảo toàn vị trí start, vị trí goal và orientation của goal; orientation start được suy theo đoạn chuyển động đầu |
| Kịch bản lối hẹp | “Đánh giá duy trì biên cách chướng ngại vật” | Không có metric clearance trong ver1 | Chỉ nói kiểm tra không phát hiện va chạm theo footprint trên costmap cấu hình; không tuyên bố biên khoảng hở |
| Kịch bản kho giao cắt | “Kiểm tra độ ổn định của liên tục (G^2)” | Không có phép đo độ ổn định (G^2) | Nói minh họa xử lý nhiều góc và đánh giá (L,E_\kappa,T); (G^2) là tính chất cấu trúc được chứng minh ở phần phương pháp |
| Số lần thử nghiệm | Viết “kết quả sau… lần thử nghiệm” nhưng chưa xác định | Chưa đủ và dễ gây nhầm | Ghi chính xác 15 nhóm ghép cặp = 3 môi trường × 5 planner, mỗi tổ hợp chạy một lần; không gọi là 15 lần lặp độc lập |

## 5. Các lỗi logic/khoa học cần sửa trước khi viết ver2

### 5.1. Phân biệt path và trajectory

PSTMO trả về một đường hình học (`nav_msgs/Path`). Thuật toán có thể dùng giới hạn chuyển động hoặc ước lượng thời gian để kiểm tra ứng viên, nhưng đầu ra của bài ver1 không phải quỹ đạo được tham số hóa đầy đủ theo thời gian. Vì vậy:

- dùng “làm mượt đường đi” thay cho “làm mượt quỹ đạo” ở các tuyên bố chính;
- không suy trực tiếp từ hình học (G^2) sang jerk, gia tốc góc hoặc năng lượng;
- khi nói đến TEB và các phương pháp không gian–thời gian, phải trình bày chúng là lớp bài toán rộng hơn và mang tính bổ sung, không phải đối chứng hoàn toàn tương đương với PSTMO.

### 5.2. Không biến mọi planner thành nguồn đường không khả thi động học

NavFn và Theta* thường tạo polyline trên lưới/any-angle và có thể có góc gãy rõ. SmacHybrid sử dụng mô hình chuyển động có xét hướng và bán kính quay, nên không nên viết rằng toàn bộ năm planner đều “chỉ tạo waypoint rời rạc và không xét động học”. Cách viết an toàn hơn là: ngay cả khi planner có xét trạng thái hoặc mô hình chuyển động, đường rời rạc đầu ra và mối nối sau lấy mẫu vẫn có thể cần hậu xử lý để đạt mục tiêu hình học, kiểm tra footprint và giao diện thống nhất với controller.

### 5.3. Khoảng trống nghiên cứu phải giới hạn theo tập tài liệu khảo sát

Không nên tuyên bố tuyệt đối “chưa có phương pháp nào”. Ver1 đã dùng cách viết thận trọng hơn: “Trong tập tài liệu được khảo sát… chưa thấy một phương pháp hậu xử lý Nav2 cho robot vi sai phối hợp đồng thời…”. Bản mới nên giữ tinh thần này và tránh từ “đầu tiên” nếu chưa có systematic review.

### 5.4. Không tuyên bố mục tiêu về thời gian mà kết quả không hỗ trợ

PSTMO trung bình 96,0 ms, trong khi Simple là 1,0 ms, Savitzky–Golay 0,2 ms và Constrained 19,0 ms trong ver1. Vì vậy không thể viết PSTMO “giữ thời gian ở mức tối thiểu” hoặc “vượt trội về thời gian”. Kết luận đúng là: PSTMO cải thiện mạnh (E_\kappa) và không làm tăng chiều dài trong tập thử, đổi lại chi phí xử lý cao hơn.

### 5.5. Không dùng kết quả mô phỏng để tuyên bố sẵn sàng cho vi điều khiển

Ver1 chạy trên Intel Core i5-12450HX, không có phép đo trên vi điều khiển hay máy tính nhúng. Mục tiêu “sẵn sàng triển khai thực tế trên vi điều khiển/máy tính nhúng” phải bỏ. Có thể ghi đây là hướng đánh giá tương lai.

### 5.6. Thống kê phải đúng với thiết kế thí nghiệm

Ver1 có 15 nhóm đầu vào ghép cặp, không phải 15 lần lặp ngẫu nhiên. Mỗi tổ hợp chỉ chạy một lần, nên chỉ được dùng thống kê mô tả; không lập khoảng tin cậy và không suy rộng về độ ổn định. Cụm “ổn định”, “bền vững” hoặc “luôn luôn” phải tránh nếu không có repeated trials.

## 6. Những điểm mạnh của ver1 cần giữ lại

Không nên sửa ver2 theo hướng bỏ toàn bộ nội dung tốt của ver1. Các điểm sau cần giữ:

- Ba đóng góp đã được nêu khá rõ: đoạn chuyển tiếp Bézier bậc năm (G^2), kiểm tra động học + swept footprint + chống chồng lấn, và đánh giá ghép cặp.
- Ver1 phân biệt đúng (E_\kappa) với năng lượng tiêu thụ.
- Ver1 nói rõ Raw không phải một smoother và các cấu hình đối chứng chưa được tinh chỉnh để tối ưu cùng một mục tiêu.
- Ver1 ghi trung thực giới hạn của phép đo thời gian, chỉ một lần chạy mỗi tổ hợp và giới hạn của kiểm tra va chạm nhị phân.
- Ver1 không tuyên bố PSTMO tìm được tuyến toàn cục ngắn hơn; nó chỉ phân bố lại chuyển hướng trong cùng hành lang.
- Ver1 ghi đúng bất biến đầu ra: bảo toàn vị trí start, vị trí goal và orientation tại goal.

## 7. Cấu trúc ver2 được khuyến nghị

Sơ đồ chuyển phần nên là:

| Ver1 | Ver2 đề xuất | Lý do |
|---|---|---|
| I. Giới thiệu + II. Nghiên cứu liên quan | I. Introduction, trong đó lồng tổng quan và khoảng trống | Làm mạch vấn đề–literature–gap liền nhau |
| III. Mô hình toán + IV. Phương pháp | II. Proposed PSTMO Method, với mô hình toán là tiểu mục đầu | Thu gọn và gắn công thức trực tiếp với chức năng thuật toán |
| V. Thử nghiệm và đánh giá | III. Experimental Evaluation | Đưa điều kiện lên trước; tách ba tình huống |
| VI. Kết luận | IV. Conclusion | Phù hợp cấu trúc bài ngắn sau khi gộp phần |

Bản dàn ý chi tiết đã hiệu chỉnh được lưu riêng trong `dàn ý ver2 hiệu chỉnh.docx`.

## 8. Nguồn kỹ thuật dùng để kiểm tra các điểm dễ nhầm

- Nav2 mô tả planner, smoother và controller là các server/plugin khác nhau; smoother nhận một path và trả về phiên bản được cải thiện: https://docs.nav2.org/concepts/
- Danh sách plugin smoother chính thức gồm Simple, Savitzky–Golay và Constrained: https://docs.nav2.org/configuration/index.html
- Simple Smoother cân bằng dữ liệu gốc và độ trơn, đồng thời Nav2 cảnh báo nó có thể phá điều kiện khả thi động học: https://docs.nav2.org/configuration/packages/configuring-simple-smoother.html
- Savitzky–Golay Smoother là bộ lọc bảy điểm trong mã Nav2 Jazzy: https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_smoother/src/savitzky_golay_smoother.cpp
- Constrained Smoother là tối ưu Ceres theo độ dài, độ trơn, vật cản và độ cong; tài liệu cũng cảnh báo chi phí tính toán nặng hơn: https://docs.nav2.org/configuration/packages/configuring-constrained-smoother.html
