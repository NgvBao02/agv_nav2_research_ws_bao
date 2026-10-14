Đánh giá ảnh hưởng của làm mượt đường đi đến hiệu năng thực thi của robot di động vi sai

Hai Linh Pham, Tien Cuong Nguyen, Viet Bao Nguyen,
Thi Phuong Thao Nguyen, Thi Ly Pham*

Khoa Điện – Điện tử, Trường Đại học Giao thông vận tải, Hà Nội, Việt Nam

*Tác giả liên hệ: ptyldk@utc.edu.vn

Tóm tắt— Bài báo đề xuất một phương pháp hậu xử lý đường đa tuyến bằng chuyển tiếp Bézier bậc năm, được ký hiệu là PSTMO, và đánh giá ảnh hưởng của phương pháp đến thực thi của robot di động vi sai trong ROS 2 Navigation2. Đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO được khảo sát qua 175 lượt trên bảy môi trường và năm bộ lập kế hoạch. So sánh ghép cặp sử dụng 34 nhóm có đủ năm lượt thành công, cùng đường đầu vào và cấu hình điều khiển. PSTMO đạt thời gian thực thi trung bình 53,148 s và căn trung bình bình phương sai số bám đường 25,51 mm, thấp nhất trong năm phương án. Thời gian giảm 2,39–7,23% so với bốn đối chứng, nhưng lợi ích không xuất hiện ở mọi nhóm và chi phí làm mượt cao hơn. Phân tích ba trường hợp cùng năm lượt thất bại chỉ ra rằng cải thiện hình học không bảo đảm giảm thời gian hoặc hoàn thành nhiệm vụ. Kết quả phản ánh cấu hình mô phỏng đã khảo sát.

Từ khóa— Làm mượt đường đi; robot vi sai; PSTMO; điều khiển bám đường; Navigation2.

I. GIỚI THIỆU

Robot di động tự hành trong kho vận và sản xuất cần thực hiện đường đi an toàn, khả thi, với thời gian phù hợp và sai số bám đường nhỏ. Trong Navigation2 (Nav2), bộ lập kế hoạch toàn cục tạo đường tham chiếu, còn bộ điều khiển sinh lệnh vận tốc từ đường và trạng thái robot [1], [2]. Vì vậy, hiệu quả của bộ làm mượt phải được đánh giá đồng thời ở cấp độ hình học và khi thực thi trong vòng điều khiển phản hồi.

Các phương pháp đã công bố gồm dải đàn hồi duy trì khoảng cách vật cản [3], đường giải tích liên tục độ cong [4], tối ưu gradient cho robot bánh xe [5] và các họ spline hoặc Bézier [7]–[12]. Những phương pháp này xử lý tính liên tục, giới hạn độ cong, động học và tránh vật cản theo các cách khác nhau. Các nghiên cứu về AGV còn đưa ràng buộc động học và vật cản động vào quỹ đạo spline [13]. Tuy nhiên, cải thiện một chỉ số hình học chưa tự xác định hiệu quả của đường trong vòng điều khiển kín.

Regulated Pure Pursuit điều chỉnh vận tốc theo độ cong và khoảng cách vật cản [6]. Bài báo này đề xuất PSTMO, một phương pháp hậu xử lý đường đa tuyến bằng chuyển tiếp Bézier bậc năm có kiểm tra hình bao và giới hạn chuyển động, sau đó đánh giá cùng ba bộ làm mượt Nav2 và đường gốc. Đóng góp gồm so sánh ghép cặp trên nhiều môi trường và bộ lập kế hoạch, đối chiếu hình học với thực thi, và phân tích cơ chế dừng của các ca thất bại. Phần II trình bày PSTMO; Phần III mô tả thử nghiệm và phân tích; Phần IV kết luận.

II. PHƯƠNG PHÁP PSTMO

A. Đường đầu vào và chuyển tiếp tại góc

PSTMO nhận đường đa tuyến từ bộ lập kế hoạch toàn cục, bản đồ chi phí, hình bao robot và các giới hạn chuyển động. Thuật toán thay cục bộ các vùng chuyển hướng, bảo toàn vị trí đầu, vị trí đích và hướng tại đích. Đầu ra là đường hình học làm tham chiếu cho bộ điều khiển; thuật toán không ấn định thời điểm robot phải đi qua từng điểm.

Tại đỉnh $V$, gọi $\mathbf{u}$ và $\mathbf{v}$ là vectơ đơn vị theo cạnh vào và cạnh ra. Hai điểm nối được đặt tại $A=V-d\mathbf{u}$ và $B=V+d\mathbf{v}$, với $d>0$ là chiều dài cắt trên mỗi cạnh. Đặt $q=\alpha d$, $0<\alpha\leq0,5$. Sáu điểm điều khiển được xác định bởi

Equation (1): P_0=A,\quad P_1=A+q\mathbf{u},\quad P_2=A+2q\mathbf{u}

Equation (2): P_3=B-2q\mathbf{v},\quad P_4=B-q\mathbf{v},\quad P_5=B

Equation (3): \mathbf{B}(t)=\sum_{i=0}^{5}\binom{5}{i}(1-t)^{5-i}t^iP_i,\quad t\in[0,1]

Hình 1. Cấu trúc chuyển tiếp Bézier bậc năm tại góc vuông, với α = 0,30. Sáu điểm điều khiển xác định theo các Công thức 1–3; hai chiều dài cắt đều bằng d. Đây là sơ đồ hình học minh họa, không phải kết quả thực nghiệm.

Figure: figure_01_bezier

Cách đặt điểm điều khiển khai thác tính chất cục bộ của Bézier bậc năm, một dạng cũng được dùng cho chuyển động liên tục độ cong [8]. Ba điểm ở mỗi đầu thẳng hàng và cách đều nên đạo hàm bậc hai tại hai đầu bằng không. Khi đạo hàm bậc nhất không suy biến, độ cong hai đầu bằng không, tạo mối nối $G^2$ với cạnh thẳng. Kết luận này chỉ áp dụng cho các góc được thay bằng chuyển tiếp; thao tác quay tại chỗ không được xem là một đường cong $G^2$ trên toàn miền.

B. Kiểm tra khả thi và lựa chọn chuyển tiếp

Mỗi phương án chuyển tiếp tại góc được kiểm tra theo giới hạn vận tốc tịnh tiến $v$, vận tốc góc $\omega=v\kappa$, vận tốc hai bánh và gia tốc ngang. Với khoảng cách hai bánh $b$ và độ cong $\kappa$, vận tốc bánh trái và bánh phải là

Equation (4): v_L=v(1-b\kappa/2),\quad v_R=v(1+b\kappa/2)

Đa giác hình bao robot được biến đổi theo các tư thế lấy mẫu trên từng phương án và kiểm tra với ô vật cản. Phương án không đạt điều kiện hình học, chuyển động hoặc va chạm bị loại trước bước tối ưu. Ước lượng thời gian chuyển tiếp được so sánh với phương án quay tại chỗ. Khi không có chuyển tiếp tịnh tiến phù hợp, thuật toán có thể giữ phương án quay tại chỗ nếu phương án đó vượt qua kiểm tra khả thi.

Hai chuyển tiếp kề nhau chỉ được ghép khi tổng chiều dài chúng sử dụng trên cạnh chung không vượt chiều dài khả dụng sau khi trừ biên phân cách. Quy hoạch động chọn chuỗi phương án tương thích có tổng chi phí nhỏ nhất; chi phí xét khoảng cách vật cản, vận tốc góc và tích phân bình phương độ cong. Sau khi ghép và lấy mẫu lại, toàn đường được kiểm tra độc lập; đường không vượt qua kiểm tra cuối bị từ chối.

III. THIẾT KẾ THỬ NGHIỆM VÀ PHÂN TÍCH

A. Hệ thống mô phỏng và phương án đối chứng

Thử nghiệm sử dụng Ubuntu 24.04, ROS 2 Jazzy, Nav2 và Gazebo Harmonic trên bộ xử lý Intel Core i5-12450HX. Robot vi sai có hình bao chữ nhật 0,44 × 0,34 m. Khoảng cách hai bánh trong mô hình kiểm tra chuyển động là 0,2548 m; mô phỏng truyền động sử dụng giá trị hiệu dụng 0,2834 m theo cấu hình đã hiệu chỉnh. Bảng I liệt kê các thông số chung.

BẢNG I. THÔNG SỐ THỬ NGHIỆM

Đại lượng | Giá trị

Độ phân giải bản đồ | 0,05 m/ô

Vận tốc tịnh tiến tối đa | 0,30 m/s

Vận tốc góc tối đa | 0,80 rad/s

Vận tốc tuyến tính mỗi bánh tối đa | 0,36 m/s

Gia tốc ngang tối đa | 0,18 m/s²

Bước kiểm tra hình bao / xuất đường | 0,02 m / 0,05 m

Dung sai vị trí / hướng tại đích | 0,10 m / 0,15 rad

Đường gốc giữ nguyên đường đầu vào. Simple làm trơn lặp cục bộ, tối đa 1000 vòng và hai lượt tinh chỉnh. Savitzky–Golay dùng cửa sổ bảy mẫu, hai lượt tinh chỉnh và dựa trên phép lọc bình phương tối thiểu cục bộ [14], [15]. Constrained tối ưu độ trơn và chi phí vật cản [16], với trọng số 200000 và 0,015; trọng số độ cong và độ lệch đường gốc bằng không. Vì vậy, kết quả chỉ đại diện cho cấu hình Constrained đã thử, không phải hiệu năng tốt nhất sau hiệu chỉnh tham số.

PSTMO giới hạn chiều dài cắt tối đa 0,80 m, phát hiện chuyển hướng từ 5° và chỉ xét chuyển tiếp tịnh tiến dưới 170°. Trọng số khoảng cách với vật cản, vận tốc góc và tích phân bình phương độ cong lần lượt là 0,15; 0,10 và 0,75. Các giới hạn trong Bảng I được giữ cố định khi thay đổi môi trường và bộ lập kế hoạch.

Hình 2. Quy trình so sánh ghép cặp. Mỗi phương án được thực thi riêng với cùng đường đầu vào và cấu hình điều khiển.

Figure: figure_02_protocol

Năm bộ lập kế hoạch gồm NavFn A*, NavFn Dijkstra, Theta*, Smac 2D và Smac Hybrid. Bảy môi trường được ký hiệu M1–M7 theo thứ tự cố định trong bộ dữ liệu. Mỗi môi trường sử dụng một cặp điểm đầu–đích, tạo 35 nhóm môi trường–bộ lập kế hoạch. Mỗi nhóm gồm năm phương án, tổng cộng 175 lượt. Mô phỏng được khởi tạo mới cho từng lượt; các phương án dùng cùng bộ điều khiển Regulated Pure Pursuit, cấu hình vận tốc và môi trường tĩnh. Hình 2 mô tả phép so sánh.

B. Các chỉ số đánh giá và tổng hợp

Chiều dài đường tham chiếu $L_{\mathrm{plan}}$ được tính bằng tổng khoảng cách Euclid giữa các điểm liên tiếp của đường xuất ra. Để so sánh độ cong với cùng thang không gian, tất cả đường được lấy mẫu lại theo chiều dài cung với bước 0,05 m; đoạn cuối có thể ngắn hơn. Với ba điểm liên tiếp, độ cong rời rạc được ước lượng bằng hai lần tích có hướng chia cho tích ba chiều dài cạnh tam giác. Các bộ ba suy biến có tích chiều dài không vượt $10^{-12}$ m³ được bỏ qua.

Equation (5): K_{\max}=\max_i|\kappa_i|,\quad E_\kappa=\sum_i\kappa_i^2\frac{a_i+b_i}{2}

Trong Công thức 5, $a_i$ và $b_i$ là khoảng cách từ điểm giữa tới hai điểm kề. $K_{\max}$ và $E_\kappa$ có đơn vị m⁻¹. Đây là chỉ số hình học ở bước lấy mẫu đã quy định; $E_\kappa$ không phải năng lượng điện và không đo đầy đủ chi phí quay tại chỗ. Các hình quỹ đạo hiển thị tọa độ gốc đã ghi, không dịch chuyển đường để làm nổi bật khác biệt.

Thời gian làm mượt $T_{\mathrm{smooth}}$ là thời lượng xử lý do máy chủ làm mượt Nav2 trả về, báo cáo bằng ms. Thời gian thực thi $T_{\mathrm{exec}}$ dùng đồng hồ mô phỏng, từ lúc bộ điều khiển nhận đường đến lúc robot dừng ổn định, gồm cả giai đoạn dừng sau khi bộ điều khiển báo hoàn tất. Điều kiện dừng yêu cầu vận tốc tịnh tiến không quá 0,01 m/s và vận tốc góc không quá 0,02 rad/s trong ba lần kiểm tra liên tiếp. Thời gian lập kế hoạch và làm mượt không nằm trong $T_{\mathrm{exec}}$.

Quãng đường thực thi $D_{\mathrm{exec}}$ là tổng khoảng cách giữa các vị trí thực của robot trong Gazebo; mẫu mới được lưu khi dịch chuyển ít nhất 0,1 mm. Sai số bám đường được tính từ chuỗi vị trí này sau khi lấy mẫu lại 0,05 m và lọc trung bình đối xứng tối đa năm mẫu, giữ nguyên hai đầu. Gọi $r_j^{\mathrm{f}}$ là vị trí sau xử lý và $P_m$ là đường tham chiếu của phương án $m$, ta có

Equation (6): \mathrm{RMSE}=\sqrt{\frac{1}{N}\sum_{j=1}^{N}d(r_j^{\mathrm{f}},P_m)^2}

Trong Công thức 6, $N$ là số mẫu vị trí sau xử lý; $d$ là khoảng cách nhỏ nhất tới các đoạn thẳng của đường tham chiếu. RMSE phản ánh độ bám đường riêng của mỗi phương án, không phải khoảng cách tới một đường chung và không phải sai số định vị. Phép lọc có thể làm giảm các đỉnh sai số ngắn; cùng quy trình được áp dụng cho mọi phương án.

Một lượt thành công khi bộ điều khiển hoàn tất, robot dừng ổn định và sai số vị trí, hướng thực tại đích nằm trong dung sai Bảng I. Tỷ lệ thành công dùng toàn bộ 35 lượt của mỗi phương án. Các giá trị trung bình và so sánh theo cặp chỉ dùng 34 nhóm có đủ năm lượt thành công và có đường đầu vào trùng khớp. Mỗi tổ hợp được chạy một lần; các chênh lệch được diễn giải dưới dạng thống kê mô tả, không dùng để khẳng định ý nghĩa thống kê.

C. Môi trường và quỹ đạo đặc trưng

1) Môi trường M2 với NavFn A*

Trường hợp thứ nhất sử dụng điểm đầu (−5,00; −3,00) m và điểm đích (−2,00; −0,50) m. Đường tương đối ngắn và chủ yếu chuyển hướng quanh một khối cản. Hình 3 phóng đại hai vùng khác biệt mà hình toàn tuyến khó thể hiện. Phương án PSTMO giảm dao động cục bộ trên đoạn đầu và tạo chuyển tiếp liên tục hơn khi đi vòng qua vật cản.

Hình 3. Đường tham chiếu tại môi trường M2 với NavFn A*: (a) toàn tuyến; (b), (c) hai vùng phóng đại. Vùng xám biểu diễn vật cản; vòng tròn và ngôi sao đánh dấu đầu và đích.

Figure: figure_03_research_warehouse

BẢNG II. MÔI TRƯỜNG M2 VỚI NAVFN A*

Phương án | $L_{\mathrm{plan}}$
(m) | $E_\kappa$
(m⁻¹) | $T_{\mathrm{exec}}$
(s) | RMSE
(mm)

Đường gốc | 4,656 | 45,442 | 31,911 | 22,10

Simple | 4,583 | 4,483 | 31,194 | 24,58

Savitzky–Golay | 4,605 | 16,540 | 31,839 | 21,95

Constrained | 4,616 | 24,631 | 31,674 | 21,28

PSTMO | 4,557 | 2,951 | 31,416 | 15,19

Bảng II cho thấy PSTMO có chiều dài 4,557 m và $E_\kappa=2,951$ m⁻¹, thấp hơn Simple 0,57% và 34,18%. Tuy nhiên, thời gian thực thi tăng từ 31,194 lên 31,416 s, tức tăng 0,222 s. Ngược lại, RMSE giảm từ 24,58 xuống 15,19 mm. Đây là trường hợp cải thiện hình học và độ bám đường nhưng không cải thiện thời gian hoàn thành.

Trong cả năm phương án, PSTMO có RMSE nhỏ nhất và thời gian đứng thứ hai sau Simple. Mức giảm độ cong ở M2 vì vậy cần được tách khỏi thứ hạng thời gian. PSTMO giảm RMSE khoảng 38,18% so với Simple nhưng chậm hơn 0,222 s; mức chênh lệch nhỏ này chưa đủ để kết luận về độ ổn định khi chưa lặp phép đo. Trên tuyến ngắn, quá trình tiếp cận và dừng tại đích có thể chiếm tỷ trọng đáng kể trong thời gian đo. Tuy nhiên, dữ liệu tổng hợp chưa tách riêng thời lượng từng giai đoạn, nên đây là một khả năng giải thích, không phải nguyên nhân đã được xác nhận.

2) Môi trường M4 với Smac 2D

Trường hợp thứ hai nối (−5,20; −3,00) m với (5,20; 3,00) m qua các vách ngăn và nhiều vùng chuyển hướng. Hình 4 cho thấy các phương án đi qua cùng hành lang nhưng khác nhau tại các đoạn đổi hướng. Các vùng phóng đại giúp quan sát khác biệt cục bộ mà không thay đổi tỷ lệ hai trục tọa độ.

Hình 4. Đường tham chiếu tại môi trường M4 với Smac 2D. Hai khung nét đứt trong (a) xác định các vùng phóng đại (b), (c); ký hiệu và màu được giữ như Hình 3.

Figure: figure_04_office_maze

BẢNG III. MÔI TRƯỜNG M4 VỚI SMAC 2D

Phương án | $L_{\mathrm{plan}}$
(m) | $E_\kappa$
(m⁻¹) | $T_{\mathrm{exec}}$
(s) | RMSE
(mm)

Đường gốc | 12,806 | 8,671 | 68,370 | 19,66

Simple | 12,715 | 5,501 | 63,408 | 24,40

Savitzky–Golay | 12,804 | 8,338 | 67,170 | 21,05

Constrained | 12,885 | 10,165 | 70,590 | 20,70

PSTMO | 12,566 | 2,781 | 56,559 | 20,91

Bảng III cho thấy, so với Simple, PSTMO giảm $E_\kappa$ từ 5,501 xuống 2,781 m⁻¹, tương ứng 49,45%. Thời gian giảm 6,849 s (10,80%), còn RMSE giảm 14,28%. Mặc dù PSTMO nhanh nhất trong trường hợp này, đường gốc vẫn có RMSE thấp hơn PSTMO: 19,66 so với 20,91 mm. Vì vậy, thứ hạng theo thời gian và theo sai số không hoàn toàn trùng nhau.

PSTMO cũng nhanh hơn đường gốc, Savitzky–Golay và Constrained trong trường hợp này, nhưng RMSE còn cao hơn Constrained. Khác với M2, ưu thế hình học của PSTMO ở M4 đi cùng giảm thời gian thực thi. Kết quả phù hợp với giả thuyết rằng các vùng chuyển hướng phân bố dọc đường có thể ảnh hưởng tích lũy tới điều tiết vận tốc. Tuy nhiên, không thể quy toàn bộ mức giảm thời gian cho độ cong vì chiều dài và hình dạng đường cũng thay đổi. Mặt khác, RMSE được đo tới đường riêng của mỗi phương án: giá trị thấp hơn của đường gốc thể hiện khả năng bám đường đó, không đồng nghĩa với thời gian hoặc chất lượng hình học tốt hơn.

3) Môi trường M6 với Smac 2D

Trường hợp thứ ba đi từ (−5,20; 3,20) m đến (5,00; −2,80) m, qua hành lang bên trái rồi chuyển sang vùng phía dưới các khối cản. Hình 5 thể hiện hai vùng chuyển hướng chính. Đường dài hơn hai trường hợp trước, nhưng phần lớn chiều dài nằm trên các đoạn gần thẳng; không thể suy ra mức cải thiện chỉ từ tổng chiều dài.

Hình 5. Đường tham chiếu trong môi trường M6 với Smac 2D. Các vùng phóng đại thể hiện góc rẽ ra khỏi hành lang và chuyển hướng gần đích.

Figure: figure_05_warehouse_dispatch

BẢNG IV. MÔI TRƯỜNG M6 VỚI SMAC 2D

Phương án | $L_{\mathrm{plan}}$
(m) | $E_\kappa$
(m⁻¹) | $T_{\mathrm{exec}}$
(s) | RMSE
(mm)

Đường gốc | 15,280 | 6,856 | 79,428 | 37,40

Simple | 15,212 | 4,908 | 77,085 | 40,37

Savitzky–Golay | 15,278 | 6,586 | 80,970 | 39,53

Constrained | 15,348 | 10,113 | 81,882 | 40,37

PSTMO | 15,166 | 4,461 | 76,221 | 33,86

Bảng IV cho thấy PSTMO có chiều dài 15,166 m, ngắn hơn Simple 0,30%. $E_\kappa$ giảm 9,10%, nhưng $K_{\max}$ tăng từ 1,900 lên 2,858 m⁻¹. Hai chỉ số phản ánh hai đặc điểm khác nhau: mức uốn tích lũy trên toàn đường và độ cong lớn nhất tại một vị trí. Thời gian thực thi vẫn giảm 0,864 s, còn RMSE giảm 16,13%.

Tại M6, PSTMO đạt thời gian và RMSE nhỏ nhất trong cả năm phương án, nhưng không đồng thời tối thiểu hóa mọi đặc trưng hình học. Một vùng cong hẹp có thể tạo giá trị cực đại lớn trong khi đóng góp vào tích phân toàn đường vẫn nhỏ. Do đó, chỉ dùng độ cong cực đại sẽ bỏ qua sự phân bố độ cong; chỉ dùng tích phân lại có thể che khuất một đoạn khó bám cục bộ. Việc thời gian giảm 1,12% dù độ cong cực đại tăng cho thấy cần đối chiếu cả hai chỉ số với kết quả điều khiển, thay vì xem một chỉ số hình học là đại diện đầy đủ cho hiệu năng thực thi.

D. Kết quả tổng hợp và so sánh ghép cặp

Bảng V sử dụng chỉ số độ cong tính lại ở cùng bước 0,05 m. PSTMO có $E_\kappa$ trung bình 4,660 m⁻¹, thấp hơn Simple 50,52%. Đổi lại, thời gian làm mượt trung bình là 107,74 ms, cao hơn Simple (2,38 ms) và Constrained (23,65 ms). Giá trị thời gian rất nhỏ chịu ảnh hưởng của độ phân giải đồng hồ; giá trị ghi bằng không ở một lượt không có nghĩa là không tốn thời gian tính toán. Đường gốc không thực hiện làm mượt nên cột này không áp dụng.

BẢNG V. KẾT QUẢ HÌNH HỌC TRUNG BÌNH TRÊN 34 NHÓM ĐẦY ĐỦ

Phương án | $L_{\mathrm{plan}}$
(m) | $K_{\max}$
(m⁻¹) | $E_\kappa$
(m⁻¹) | $T_{\mathrm{smooth}}$
(ms)

Đường gốc | 10,340 | 11,187 | 50,207 | —

Simple | 10,242 | 4,763 | 9,418 | 2,38

Savitzky–Golay | 10,302 | 6,934 | 22,699 | 0,09

Constrained | 10,338 | 7,388 | 28,065 | 23,65

PSTMO | 10,176 | 3,299 | 4,660 | 107,74

So với đường gốc, Savitzky–Golay và Constrained, PSTMO giảm $E_\kappa$ trung bình lần lượt 90,72%, 79,47% và 83,40%. Độ cong cực đại và chiều dài tham chiếu trung bình cũng nhỏ nhất, trong khi chi phí làm mượt cao nhất. Đây là đánh đổi giữa mức cải thiện đường và thời gian xử lý, không phải ưu thế đồng thời trên mọi chỉ tiêu.

BẢNG VI. KẾT QUẢ THỰC THI TRUNG BÌNH TRÊN 34 NHÓM ĐẦY ĐỦ

Phương án | $D_{\mathrm{exec}}$
(m) | $T_{\mathrm{exec}}$
(s) | RMSE
(mm) | Đạt

Đường gốc | 10,156 | 57,120 | 26,86 | 34/35

Simple | 10,111 | 54,449 | 27,36 | 34/35

Savitzky–Golay | 10,152 | 56,677 | 27,83 | 34/35

Constrained | 10,170 | 57,293 | 27,58 | 34/35

PSTMO | 10,072 | 53,148 | 25,51 | 34/35

Bảng VI cho thấy PSTMO có thời gian thực thi và RMSE trung bình thấp nhất trong năm phương án. So với đường gốc, Simple, Savitzky–Golay và Constrained, thời gian giảm lần lượt 3,973 s (6,96%), 1,301 s (2,39%), 3,529 s (6,23%) và 4,145 s (7,23%). Các tỷ lệ được tính từ chênh lệch giữa hai giá trị trung bình chia cho trung bình đối chứng, không phải trung bình của các phần trăm riêng lẻ.

Quãng đường thực thi trung bình của PSTMO là 10,072 m, thấp hơn Simple 0,039 m và thấp hơn ba đối chứng còn lại khoảng 0,080–0,098 m. Mức giảm nhỏ này đi cùng mức giảm thời gian khác nhau, cho thấy không thể giải thích lợi ích chỉ bằng rút ngắn đường. RMSE trung bình giảm khoảng 1,35–2,32 mm so với bốn đối chứng. Tuy nhiên, đây là mức cải thiện độ bám đường tham chiếu đã xử lý, không phải cải thiện trực tiếp độ chính xác định vị.

Hình 6. Chênh lệch đối chứng trừ PSTMO trên 34 nhóm đầy đủ cho từng đối chứng. Giá trị dương có lợi cho PSTMO. Mỗi chấm là một nhóm; vạch đen là trung bình. Màu chỉ M1–M7; độ lệch nhỏ theo phương đứng chỉ giúp phân biệt các điểm.

Figure: figure_06_paired

Hình 6 cho thấy PSTMO nhanh hơn đường gốc, Simple, Savitzky–Golay và Constrained lần lượt ở 32, 21, 31 và 31 trên 34 nhóm; không có trường hợp bằng nhau về thời gian. Số nhóm có RMSE nhỏ hơn tương ứng là 21, 24, 25 và 23. Như vậy, ưu thế trung bình không có nghĩa PSTMO tốt hơn ở mọi đầu vào. Ba trường hợp trong Mục III-C minh họa những đánh đổi khác nhau; thống kê tổng hợp vẫn sử dụng đủ 34 nhóm.

Chi phí làm mượt cũng cần được xét cùng lợi ích thực thi. Khi cộng hai thời lượng đã ghi, PSTMO vẫn thấp hơn Simple trung bình 1,196 s. Đại lượng này không bao gồm lập kế hoạch và toàn bộ độ trễ giao tiếp; vì vậy chưa phải thời gian đáp ứng đầu cuối của hệ thống.

Hình 7. Quan hệ giữa mức giảm tích phân bình phương độ cong và thời gian tiết kiệm của PSTMO so với từng đối chứng. Mỗi ô gồm 34 nhóm; giá trị dương có lợi cho PSTMO; màu như Hình 6. Các ô sử dụng thang trục riêng.

Figure: figure_07_association

Hệ số tương quan hạng Spearman trong Hình 7 lần lượt là 0,411, 0,634, 0,535 và 0,323 theo thứ tự đường gốc, Simple, Savitzky–Golay và Constrained. Các hệ số được tính riêng cho từng đối chứng; không gộp 136 cặp thành các quan sát độc lập vì chúng dùng chung 34 kết quả PSTMO. Mức liên hệ phụ thuộc đối chứng và không xác lập quan hệ nhân quả. Các điểm có thời gian tiết kiệm âm dù độ cong tích lũy giảm tiếp tục cho thấy ưu thế hình học không bảo đảm ưu thế thực thi.

E. Phân tích các trường hợp thất bại

Cả năm thất bại đều thuộc M6 với Smac Hybrid, nhưng xảy ra ở hai giai đoạn khác nhau. Với Simple, máy chủ làm mượt từ chối đường do phát hiện tư thế trên đường ứng viên giao với vật cản, tại vị trí xấp xỉ (−4,741; 3,482) m. Robot chưa bước vào phép đo thực thi; vì vậy không có thời gian, quãng đường và sai số đích để so sánh như một lượt hoàn thành.

Với bốn phương án còn lại, bộ điều khiển liên tục cảnh báo nguy cơ va chạm phía trước, sau đó hủy thực thi khi vượt thời gian cho phép duy trì trạng thái không tạo được lệnh điều khiển hợp lệ. Bảng VII cho thấy đường gốc, Savitzky–Golay và PSTMO gần như chưa dịch chuyển, còn Constrained đi được khoảng 0,353 m. Cả bốn đều dừng ổn định nhưng còn cách đích hơn 11,7 m; dừng ổn định không đồng nghĩa với hoàn thành nhiệm vụ.

BẢNG VII. NĂM THẤT BẠI TẠI M6 VỚI SMAC HYBRID

Phương án | Giai đoạn dừng | Quãng đường (m) | Sai số đích (m)

Đường gốc | Điều khiển | <0,001 | 11,834

Simple | Làm mượt | — | —

Savitzky–Golay | Điều khiển | <0,001 | 11,834

Constrained | Điều khiển | 0,353 | 11,712

PSTMO | Điều khiển | <0,001 | 11,834

Cảnh báo dự đoán của bộ điều khiển cần được phân biệt với va chạm thực tế. Bốn đường được đưa vào thực thi không có mẫu hình bao va chạm theo phép kiểm tra đã ghi và không kích hoạt bộ giám sát va chạm. Điều này không mâu thuẫn với việc bộ điều khiển từ chối chuyển động vì kiểm tra đường và dự đoán chuyển động diễn ra ở các giai đoạn khác nhau. Nhật ký xác định cơ chế dừng trực tiếp nhưng chưa tách được ảnh hưởng của bản đồ chi phí, tư thế và hướng chuyển động tới cảnh báo.

Thời gian hoặc RMSE rất nhỏ khi robot gần như đứng yên trong lượt bị hủy không biểu thị hiệu năng tốt. Nhóm thất bại bị loại đồng thời khỏi trung bình ghép cặp nhưng vẫn tính vào tỷ lệ thành công; mọi phương án đạt 34/35 lượt (97,14%). PSTMO chưa cải thiện tỷ lệ hoàn thành trong tập thử. Nghiên cứu chỉ chạy một lượt mỗi tổ hợp, dùng một cấu hình điều khiển và đối chứng trong mô phỏng tĩnh, nên chưa cho phép suy rộng sang độ ổn định khi lặp hoặc robot thực.

IV. KẾT LUẬN

Trên 34 nhóm thành công đầy đủ của 175 lượt thử, PSTMO có thời gian thực thi và RMSE trung bình thấp nhất trong năm phương án; thời gian giảm 2,39–7,23% so với bốn đối chứng. Lợi ích không đồng đều giữa các đường và đi kèm chi phí làm mượt cao hơn. Các ca thất bại cho thấy khả thi hình học và trạng thái dừng ổn định chưa bảo đảm hoàn thành nhiệm vụ. Vì vậy, đánh giá bộ làm mượt cần kết hợp hình học, thực thi và thất bại. Nghiên cứu tiếp theo cần lặp phép thử, khảo sát cấu hình và kiểm chứng trên robot thực.

TÀI LIỆU THAM KHẢO

[1] S. Macenski et al., “The Marathon 2: A Navigation System,” Proc. IEEE/RSJ IROS, 2020, tr. 2718–2725.

[2] S. Macenski et al., “From the Desks of ROS Maintainers: A Survey of Modern & Capable Mobile Robotics Algorithms in the Robot Operating System 2,” Robot. Auton. Syst., tập 168, mã bài 104493, 2023.

[3] S. Quinlan và O. Khatib, “Elastic Bands: Connecting Path Planning and Control,” Proc. IEEE ICRA, tập 2, 1993, tr. 802–807.

[4] K. Yang và S. Sukkarieh, “An Analytical Continuous-Curvature Path-Smoothing Algorithm,” IEEE Trans. Robot., tập 26, số 3, tr. 561–568, 2010.

[5] E. Heiden et al., “Gradient-Informed Path Smoothing for Wheeled Mobile Robots,” Proc. IEEE ICRA, 2018, tr. 1710–1717.

[6] S. Macenski et al., “Regulated Pure Pursuit for Robot Path Tracking,” Auton. Robot., tập 47, số 6, tr. 685–694, 2023.

[7] A. Ravankar et al., “Path Smoothing Techniques in Robot Navigation: State-of-the-Art, Current and Future Challenges,” Sensors, tập 18, số 9, mã bài 3170, 2018.

[8] B. Lau, C. Sprunk và W. Burgard, “Kinodynamic Motion Planning for Mobile Robots Using Splines,” Proc. IEEE/RSJ IROS, 2009, tr. 2427–2433.

[9] T. Berglund et al., “Planning Smooth and Obstacle-Avoiding B-Spline Paths for Autonomous Mining Vehicles,” IEEE Trans. Autom. Sci. Eng., tập 7, số 1, tr. 167–172, 2010.

[10] M. Elbanhawi, M. Simic và R. N. Jazar, “Continuous Path Smoothing for Car-Like Robots Using B-Spline Curves,” J. Intell. Robot. Syst., tập 80, số S1, tr. 23–56, 2015.

[11] X. Bu et al., “Curvature Continuous Path Smoothing Based on Cubic Bezier Curves for Car-Like Vehicles,” Proc. IEEE ROBIO, 2015, tr. 1453–1458.

[12] N. T. Nguyen et al., “B-Spline Path Planner for Safe Navigation of Mobile Robots,” Proc. IEEE/RSJ IROS, 2021, tr. 339–345.

[13] T. Mercy, R. Van Parys và G. Pipeleers, “Spline-Based Motion Planning for Autonomous Guided Vehicles in a Dynamic Environment,” IEEE Trans. Control Syst. Technol., tập 26, số 6, tr. 2182–2189, 2018.

[14] A. Savitzky và M. J. E. Golay, “Smoothing and Differentiation of Data by Simplified Least Squares Procedures,” Anal. Chem., tập 36, số 8, tr. 1627–1639, 1964.

[15] Navigation2 Project, “Nav2 Smoother,” tài liệu phiên bản Jazzy. https://api.nav2.org/nav2-jazzy/html/md_nav2_smoother_README.html (truy cập 14/10/2026).

[16] Navigation2 Project, “Constrained Smoother,” tài liệu phiên bản Jazzy. https://api.nav2.org/nav2-jazzy/html/md_nav2_constrained_smoother_README.html (truy cập 14/10/2026).
