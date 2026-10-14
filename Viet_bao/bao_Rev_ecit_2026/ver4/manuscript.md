Đánh giá ảnh hưởng của làm mượt đường đi đến hiệu năng thực thi của robot di động vi sai

Hai Linh Pham, Tien Cuong Nguyen, Viet Bao Nguyen,
Thi Phuong Thao Nguyen, Thi Ly Pham*

Khoa Điện – Điện tử, Trường Đại học Giao thông vận tải, Hà Nội, Việt Nam

*Tác giả liên hệ: ptyldk@utc.edu.vn

Tóm tắt— Nghiên cứu đánh giá ảnh hưởng của làm mượt đường đi đến thời gian hoàn thành nhiệm vụ và sai số bám đường của robot di động vi sai trong ROS 2 Navigation2. Năm phương án gồm đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO được so sánh trên bảy môi trường với năm bộ lập kế hoạch. Tổng cộng 175 lượt được thực hiện, mỗi tổ hợp một lần; 34 nhóm có đủ năm kết quả thành công được sử dụng cho so sánh ghép cặp. Các phương án trong mỗi nhóm nhận cùng đường đầu vào và sử dụng cùng cấu hình điều khiển. PSTMO đạt thời gian thực thi trung bình 53,148 s và căn trung bình bình phương sai số bám đường 0,0255 m, thấp hơn Simple lần lượt 1,301 s và 0,0019 m. PSTMO nhanh hơn Simple ở 21/34 nhóm và có sai số nhỏ hơn ở 24/34 nhóm. Phân tích từng trường hợp cho thấy giảm độ cong không luôn đi kèm giảm thời gian thực thi. Kết quả xác định lợi ích và giới hạn của PSTMO trong cấu hình mô phỏng được khảo sát.

Từ khóa— Làm mượt đường đi; robot vi sai; PSTMO; điều khiển bám đường; Navigation2.

I. GIỚI THIỆU

Robot di động tự hành trong kho vận và sản xuất cần thực hiện các đường đi an toàn với thời gian phù hợp và sai số bám đường nhỏ. Trong Navigation2 (Nav2), bộ lập kế hoạch toàn cục xác định đường tham chiếu, còn bộ điều khiển tính lệnh vận tốc từ đường và trạng thái robot [1], [2]. Vì vậy, hiệu quả của một bộ làm mượt cần được xem xét đồng thời ở cấp độ hình học và cấp độ thực thi.

Các hướng làm mượt đã được phát triển gồm biến dạng dải đàn hồi để duy trì khoảng cách với vật cản [3], xây dựng đường có độ cong liên tục bằng biểu diễn giải tích [4] và sử dụng gradient để cải thiện đường cho robot bánh xe [5]. Các nghiên cứu này cung cấp cơ sở để xử lý đường đa tuyến sau lập kế hoạch. Tuy nhiên, mức giảm một chỉ số hình học không tự xác định được mức cải thiện khi đường được đưa vào hệ điều khiển phản hồi.

Bộ điều khiển Regulated Pure Pursuit điều chỉnh vận tốc theo độ cong và khoảng cách đến vật cản [6]. Do đó, một đường ngắn hơn hoặc có tích phân bình phương độ cong nhỏ hơn vẫn có thể không hoàn thành nhanh hơn: vị trí các đoạn cong, điều kiện tiếp cận đích và phản ứng điều khiển cũng ảnh hưởng đến chuyển động. So sánh các bộ làm mượt trên những đầu vào hoặc cấu hình điều khiển khác nhau có thể che khuất ảnh hưởng này.

Nghiên cứu này sử dụng PSTMO, phương pháp lựa chọn chuyển tiếp tại góc do nhóm xây dựng [7], làm đối tượng đánh giá cùng ba bộ làm mượt của Nav2. Đóng góp của bài báo là thiết lập so sánh ghép cặp trên nhiều môi trường và bộ lập kế hoạch; đối chiếu chất lượng hình học với thời gian, quãng đường thực thi và sai số bám đường; đồng thời phân tích các trường hợp mà ưu thế hình học không chuyển thành ưu thế thực thi. Phần II tóm tắt phương pháp; Phần III trình bày thiết kế, kết quả và giới hạn thực nghiệm; Phần IV đưa ra kết luận.

II. PHƯƠNG PHÁP PSTMO

A. Đường đầu vào và chuyển tiếp tại góc

PSTMO nhận đường đa tuyến từ bộ lập kế hoạch toàn cục, bản đồ chi phí, hình bao robot và các giới hạn chuyển động. Thuật toán điều chỉnh cục bộ các vùng chuyển hướng, bảo toàn vị trí đầu, vị trí đích và hướng tại đích. Đầu ra là đường hình học làm tham chiếu cho bộ điều khiển; thuật toán không ấn định thời điểm robot phải đi qua từng điểm.

Tại đỉnh $V$, gọi $\mathbf{u}$ và $\mathbf{v}$ là vectơ đơn vị theo cạnh vào và cạnh ra. Hai điểm nối được đặt tại $A=V-d\mathbf{u}$ và $B=V+d\mathbf{v}$, với $d>0$ là chiều dài cắt trên mỗi cạnh. Đặt $q=\alpha d$, $0<\alpha\leq0,5$. Sáu điểm điều khiển được xác định bởi

Equation (1): P_0=A,\quad P_1=A+q\mathbf{u},\quad P_2=A+2q\mathbf{u}

Equation (2): P_3=B-2q\mathbf{v},\quad P_4=B-q\mathbf{v},\quad P_5=B

Equation (3): \mathbf{B}(t)=\sum_{i=0}^{5}\binom{5}{i}(1-t)^{5-i}t^iP_i,\quad t\in[0,1]

Ba điểm ở mỗi đầu thẳng hàng và cách đều nên đạo hàm bậc hai tại hai đầu bằng không. Khi đạo hàm bậc nhất không suy biến, độ cong đầu và cuối bằng không, cho phép nối với cạnh thẳng theo liên tục hình học bậc hai $G^2$. Điều kiện này áp dụng cho các mối nối được thay bằng chuyển tiếp; đường còn thao tác quay tại chỗ không được xem là một đường cong $G^2$ trên toàn miền.

B. Kiểm tra khả thi và lựa chọn chuyển tiếp

Mỗi ứng viên được kiểm tra theo giới hạn vận tốc tịnh tiến $v$, vận tốc góc $\omega=v\kappa$, vận tốc hai bánh và gia tốc ngang. Với khoảng cách hai bánh $b$ và độ cong $\kappa$, vận tốc bánh trái và bánh phải là

Equation (4): v_L=v(1-b\kappa/2),\quad v_R=v(1+b\kappa/2)

Đa giác hình bao robot được biến đổi theo các tư thế lấy mẫu trên ứng viên và kiểm tra với ô vật cản. Ứng viên không đạt điều kiện hình học, chuyển động hoặc va chạm bị loại trước bước tối ưu. Ước lượng thời gian chuyển tiếp được so sánh với phương án quay tại chỗ. Khi không có chuyển tiếp tịnh tiến phù hợp, thuật toán có thể giữ phương án quay tại chỗ nếu phương án đó vượt qua kiểm tra khả thi.

Hai chuyển tiếp kề nhau chỉ được ghép khi tổng phần chiều dài chúng sử dụng trên cạnh chung không vượt chiều dài cạnh sau khi trừ biên phân cách. Quy hoạch động lựa chọn chuỗi ứng viên có tổng chi phí nhỏ nhất trong tập tương thích. Chi phí xét khoảng cách với vật cản, vận tốc góc và tích phân bình phương độ cong. Sau khi ghép và lấy mẫu lại, toàn đường được kiểm tra độc lập; đường không vượt qua kiểm tra cuối không được chấp nhận. Mô tả chi tiết cấu trúc thuật toán được trình bày trong [7].

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

Raw giữ nguyên đường đầu vào. Simple thực hiện làm trơn lặp cục bộ, tối đa 1000 vòng và hai lượt tinh chỉnh. Savitzky–Golay sử dụng cửa sổ bảy mẫu và hai lượt tinh chỉnh [8]. Constrained giải bài toán tối ưu có xét độ trơn và chi phí vật cản [9]; các trọng số tương ứng là 200000 và 0,015, còn trọng số độ cong và độ lệch khỏi đường gốc bằng không. Do đó, kết quả chỉ đại diện cho cấu hình Constrained được thử, không đại diện cho hiệu năng tốt nhất sau hiệu chỉnh tham số.

PSTMO giới hạn chiều dài cắt tối đa 0,80 m, phát hiện chuyển hướng từ 5° và chỉ xét chuyển tiếp tịnh tiến dưới 170°. Trọng số khoảng cách với vật cản, vận tốc góc và tích phân bình phương độ cong lần lượt là 0,15; 0,10 và 0,75. Các giới hạn trong Bảng I được giữ cố định khi thay đổi môi trường và bộ lập kế hoạch.

Hình 1. Quy trình so sánh ghép cặp. Mỗi phương án được thực thi riêng với cùng đường đầu vào và cấu hình điều khiển.

Figure: figure_01_protocol

Năm bộ lập kế hoạch gồm NavFn A*, NavFn Dijkstra, Theta*, Smac 2D và Smac Hybrid. Bảy môi trường gồm không gian mở, kho nghiên cứu, lối đi hẹp, văn phòng, kho giao cắt, kho điều phối và kho hành lang dài. Mỗi môi trường sử dụng một cặp điểm đầu–đích, tạo 35 nhóm môi trường–bộ lập kế hoạch. Mỗi nhóm gồm năm phương án, tổng cộng 175 lượt. Mô phỏng được khởi tạo mới cho từng lượt; các phương án dùng cùng bộ điều khiển Regulated Pure Pursuit, cấu hình vận tốc và môi trường tĩnh. Hình 1 mô tả phép so sánh.

B. Đại lượng đo và quy tắc tổng hợp

Chiều dài đường tham chiếu $L_{\mathrm{plan}}$ được tính bằng tổng khoảng cách Euclid giữa các điểm liên tiếp của đường xuất ra. Để so sánh độ cong với cùng thang không gian, tất cả đường được lấy mẫu lại theo chiều dài cung với bước 0,05 m; đoạn cuối có thể ngắn hơn. Với ba điểm liên tiếp, độ cong rời rạc được ước lượng bằng hai lần tích có hướng chia cho tích ba chiều dài cạnh tam giác. Các bộ ba suy biến có tích chiều dài không vượt $10^{-12}$ m³ được bỏ qua.

Equation (5): K_{\max}=\max_i|\kappa_i|,\quad E_\kappa=\sum_i\kappa_i^2\frac{a_i+b_i}{2}

Trong (5), $a_i$ và $b_i$ là khoảng cách từ điểm giữa tới hai điểm kề. $K_{\max}$ và $E_\kappa$ có đơn vị m⁻¹. Đây là chỉ số hình học ở bước lấy mẫu đã quy định; $E_\kappa$ không phải năng lượng điện và không đo đầy đủ chi phí quay tại chỗ. Các hình quỹ đạo hiển thị tọa độ gốc đã ghi, không dịch chuyển đường để làm nổi bật khác biệt.

Thời gian làm mượt $T_{\mathrm{smooth}}$ là thời lượng xử lý do máy chủ làm mượt Nav2 trả về, báo cáo bằng ms. Thời gian thực thi $T_{\mathrm{exec}}$ dùng đồng hồ mô phỏng, từ lúc bộ điều khiển nhận đường đến lúc robot dừng ổn định, gồm cả giai đoạn dừng sau khi bộ điều khiển báo hoàn tất. Điều kiện dừng yêu cầu vận tốc tịnh tiến không quá 0,01 m/s và vận tốc góc không quá 0,02 rad/s trong ba lần kiểm tra liên tiếp. Thời gian lập kế hoạch và làm mượt không nằm trong $T_{\mathrm{exec}}$.

Quãng đường thực thi $D_{\mathrm{exec}}$ là tổng khoảng cách giữa các vị trí thực của robot trong Gazebo; mẫu mới được lưu khi dịch chuyển ít nhất 0,1 mm. Sai số bám đường được tính từ chuỗi vị trí này sau khi lấy mẫu lại 0,05 m và lọc trung bình đối xứng tối đa năm mẫu, giữ nguyên hai đầu. Gọi $r_j^{\mathrm{f}}$ là vị trí sau xử lý và $P_m$ là đường tham chiếu của phương án $m$, ta có

Equation (6): \mathrm{RMSE}=\sqrt{\frac{1}{N}\sum_{j=1}^{N}d(r_j^{\mathrm{f}},P_m)^2}

Trong (6), $N$ là số mẫu vị trí sau xử lý; $d$ là khoảng cách nhỏ nhất tới các đoạn thẳng của đường tham chiếu. RMSE phản ánh độ bám đường riêng của mỗi phương án, không phải khoảng cách tới một đường chung và không phải sai số định vị. Phép lọc có thể làm giảm các đỉnh sai số ngắn; cùng quy trình được áp dụng cho mọi phương án.

Một lượt thành công khi bộ điều khiển hoàn tất, robot dừng ổn định và sai số vị trí, hướng thực tại đích nằm trong dung sai Bảng I. Tỷ lệ thành công dùng toàn bộ 35 lượt của mỗi phương án. Các giá trị trung bình và so sánh theo cặp chỉ dùng 34 nhóm có đủ năm lượt thành công và có đường đầu vào trùng khớp. Mỗi tổ hợp được chạy một lần; các chênh lệch được diễn giải dưới dạng thống kê mô tả, không dùng để khẳng định ý nghĩa thống kê.

C. Kho nghiên cứu với NavFn A*

Trường hợp thứ nhất sử dụng điểm đầu (−5,00; −3,00) m và điểm đích (−2,00; −0,50) m. Đường tương đối ngắn và chủ yếu chuyển hướng quanh một khối cản. Hình 2 phóng đại hai vùng khác biệt mà hình toàn tuyến khó thể hiện. Phương án PSTMO giảm dao động cục bộ trên đoạn đầu và tạo chuyển tiếp liên tục hơn khi đi vòng qua vật cản.

Bảng II cho thấy PSTMO có chiều dài 4,557 m và $E_\kappa=2,951$ m⁻¹, thấp hơn Simple 0,57% và 34,18%. Tuy nhiên, thời gian thực thi tăng từ 31,194 lên 31,416 s, tức tăng 0,222 s. Ngược lại, RMSE giảm từ 24,58 xuống 15,19 mm. Đây là trường hợp cải thiện hình học và độ bám đường nhưng không cải thiện thời gian hoàn thành.

Hình 2. Đường tham chiếu tại kho nghiên cứu với NavFn A*: (a) toàn tuyến; (b), (c) hai vùng phóng đại. Vùng xám biểu diễn vật cản; vòng tròn và ngôi sao đánh dấu đầu và đích.

Figure: figure_02_research_warehouse

BẢNG II. KHO NGHIÊN CỨU VỚI NAVFN A*

Phương án | $L_{\mathrm{plan}}$
(m) | $E_\kappa$
(m⁻¹) | $T_{\mathrm{exec}}$
(s) | RMSE
(mm)

Raw | 4,656 | 45,442 | 31,911 | 22,10

Simple | 4,583 | 4,483 | 31,194 | 24,58

Savitzky–Golay | 4,605 | 16,540 | 31,839 | 21,95

Constrained | 4,616 | 24,631 | 31,674 | 21,28

PSTMO | 4,557 | 2,951 | 31,416 | 15,19

D. Văn phòng với Smac 2D

Trường hợp thứ hai nối (−5,20; −3,00) m với (5,20; 3,00) m qua các vách ngăn và nhiều vùng chuyển hướng. Hình 3 cho thấy các phương án đi qua cùng hành lang nhưng khác nhau tại các đoạn đổi hướng. Các vùng phóng đại giúp quan sát khác biệt cục bộ mà không thay đổi tỷ lệ hai trục tọa độ.

So với Simple, PSTMO giảm $E_\kappa$ từ 5,501 xuống 2,781 m⁻¹, tương ứng 49,45%. Thời gian giảm 6,849 s (10,80%), còn RMSE giảm 14,28%. Mặc dù PSTMO nhanh nhất trong trường hợp này, Raw vẫn có RMSE thấp hơn PSTMO: 19,66 so với 20,91 mm. Vì vậy, thứ hạng theo thời gian và theo sai số không hoàn toàn trùng nhau.

Hình 3. Đường tham chiếu trong văn phòng với Smac 2D. Hai khung nét đứt trong (a) xác định các vùng phóng đại (b), (c); ký hiệu và màu được giữ như Hình 2.

Figure: figure_03_office_maze

BẢNG III. VĂN PHÒNG VỚI SMAC 2D

Phương án | $L_{\mathrm{plan}}$
(m) | $E_\kappa$
(m⁻¹) | $T_{\mathrm{exec}}$
(s) | RMSE
(mm)

Raw | 12,806 | 8,671 | 68,370 | 19,66

Simple | 12,715 | 5,501 | 63,408 | 24,40

Savitzky–Golay | 12,804 | 8,338 | 67,170 | 21,05

Constrained | 12,885 | 10,165 | 70,590 | 20,70

PSTMO | 12,566 | 2,781 | 56,559 | 20,91

E. Kho điều phối với Smac 2D

Trường hợp thứ ba đi từ (−5,20; 3,20) m đến (5,00; −2,80) m, qua hành lang bên trái rồi chuyển sang vùng phía dưới các dãy kệ. Hình 4 thể hiện hai vùng chuyển hướng chính. Đường dài hơn hai trường hợp trước, nhưng phần lớn chiều dài nằm trên các đoạn gần thẳng; không thể suy ra mức cải thiện chỉ từ tổng chiều dài.

PSTMO có chiều dài 15,166 m, ngắn hơn Simple 0,30%. $E_\kappa$ giảm 9,10%, nhưng $K_{\max}$ tăng từ 1,900 lên 2,858 m⁻¹. Hai chỉ số phản ánh hai đặc điểm khác nhau: mức uốn tích lũy trên toàn đường và độ cong lớn nhất tại một vị trí. Thời gian thực thi vẫn giảm 0,864 s, còn RMSE giảm 16,13%.

Hình 4. Đường tham chiếu trong kho điều phối với Smac 2D. Các vùng phóng đại thể hiện góc rẽ ra khỏi hành lang và chuyển hướng gần đích.

Figure: figure_04_warehouse_dispatch

BẢNG IV. KHO ĐIỀU PHỐI VỚI SMAC 2D

Phương án | $L_{\mathrm{plan}}$
(m) | $E_\kappa$
(m⁻¹) | $T_{\mathrm{exec}}$
(s) | RMSE
(mm)

Raw | 15,280 | 6,856 | 79,428 | 37,40

Simple | 15,212 | 4,908 | 77,085 | 40,37

Savitzky–Golay | 15,278 | 6,586 | 80,970 | 39,53

Constrained | 15,348 | 10,113 | 81,882 | 40,37

PSTMO | 15,166 | 4,461 | 76,221 | 33,86

F. Kết quả tổng hợp và so sánh ghép cặp

BẢNG V. HÌNH HỌC TRÊN 34 NHÓM ĐẦY ĐỦ

Phương án | $L_{\mathrm{plan}}$
(m) | $K_{\max}$
(m⁻¹) | $E_\kappa$
(m⁻¹) | $T_{\mathrm{smooth}}$
(ms)

Raw | 10,340 | 11,187 | 50,207 | —

Simple | 10,242 | 4,763 | 9,418 | 2,38

Savitzky–Golay | 10,302 | 6,934 | 22,699 | 0,09

Constrained | 10,338 | 7,388 | 28,065 | 23,65

PSTMO | 10,176 | 3,299 | 4,660 | 107,74

Bảng V sử dụng chỉ số độ cong tính lại ở cùng bước 0,05 m. PSTMO có $E_\kappa$ trung bình 4,660 m⁻¹, thấp hơn Simple 50,52%. Đổi lại, thời gian làm mượt trung bình là 107,74 ms, cao hơn Simple (2,38 ms) và Constrained (23,65 ms). Giá trị thời gian rất nhỏ chịu ảnh hưởng của độ phân giải đồng hồ; giá trị ghi bằng không ở một lượt không có nghĩa là không tốn thời gian tính toán. Raw không thực hiện làm mượt nên cột này không áp dụng.

BẢNG VI. THỰC THI TRÊN 34 NHÓM ĐẦY ĐỦ

Phương án | $D_{\mathrm{exec}}$
(m) | $T_{\mathrm{exec}}$
(s) | RMSE
(mm) | Đạt

Raw | 10,156 | 57,120 | 26,86 | 34/35

Simple | 10,111 | 54,449 | 27,36 | 34/35

Savitzky–Golay | 10,152 | 56,677 | 27,83 | 34/35

Constrained | 10,170 | 57,293 | 27,58 | 34/35

PSTMO | 10,072 | 53,148 | 25,51 | 34/35

PSTMO đạt thời gian trung bình thấp nhất trong năm phương án, giảm 1,301 s (2,39%) so với Simple. RMSE trung bình giảm 1,85 mm (6,77%). Nếu cộng thêm thời gian làm mượt đã ghi, chênh lệch thời gian trung bình giữa Simple và PSTMO còn 1,196 s; giá trị này vẫn không gồm thời gian lập kế hoạch hoặc toàn bộ độ trễ giao tiếp.

Hình 5. So sánh PSTMO với Simple trên 34 nhóm đầy đủ. Mỗi điểm là một nhóm môi trường–bộ lập kế hoạch; điểm dưới đường chéo có giá trị PSTMO nhỏ hơn. Màu biểu diễn môi trường.

Figure: figure_05_paired

Hình 5 cho thấy PSTMO nhanh hơn Simple ở 21/34 nhóm và có RMSE thấp hơn ở 24/34 nhóm. Các nhóm còn lại cho kết quả bằng hoặc kém hơn ở từng chỉ số. Ba trường hợp ở các mục C–E chỉ là minh họa có chọn lọc; kết luận tổng hợp sử dụng đầy đủ 34 nhóm để tránh suy rộng từ một đường thuận lợi.

Hình 6. Mối liên hệ giữa mức giảm tích phân bình phương độ cong và thời gian tiết kiệm của PSTMO so với Simple. Giá trị dương biểu thị lợi ích của PSTMO; màu như Hình 5.

Figure: figure_06_association

Với chênh lệch được tính bằng giá trị Simple trừ PSTMO, hệ số tương quan hạng Spearman giữa mức giảm $E_\kappa$ và thời gian tiết kiệm là 0,634. Xu hướng đồng biến trong Hình 6 hỗ trợ việc xem xét độ cong như một yếu tố liên quan đến thời gian thực thi. Tuy nhiên, các nhóm đồng thời khác môi trường và bộ lập kế hoạch; tương quan này không xác lập quan hệ nhân quả hoặc một quy luật dự báo thời gian cho mọi quỹ đạo.

G. Thất bại và giới hạn của nghiên cứu

Cả năm lượt không đạt thuộc nhóm kho điều phối với Smac Hybrid. Simple không tạo được đường được chấp nhận; bốn phương án còn lại có đường nhưng robot không hoàn thành nhiệm vụ theo tiêu chí đã định. Nhóm này được giữ trong tỷ lệ thành công và loại đồng thời khỏi trung bình ghép cặp của cả năm phương án. Vì vậy, mỗi phương án đạt 34/35 lượt, tương ứng 97,14%; không có bằng chứng trong tập thử này rằng PSTMO làm tăng tỷ lệ hoàn thành so với các đối chứng.

Không ghi nhận can thiệp của bộ giám sát va chạm hoặc mẫu hình bao va chạm trên các đường đã được kiểm tra. Kết quả đó chỉ phản ánh phép kiểm tra rời rạc và dữ liệu giám sát đã thu; không đủ để khẳng định an toàn liên tục trong mọi điều kiện. Các giá trị trung bình ở Bảng V–VI cũng là trung bình có điều kiện trên các nhóm thành công đầy đủ, không mô tả thời gian hoàn thành của các lượt thất bại.

Phạm vi đánh giá còn ba giới hạn chính. Thứ nhất, một lượt cho mỗi tổ hợp chưa cho phép ước lượng độ biến thiên khi lặp lại và chưa hỗ trợ kiểm định độ ổn định. Thứ hai, kết quả phụ thuộc bộ điều khiển, tham số đối chứng, bước đánh giá độ cong và phép xử lý vị trí dùng cho RMSE. Thứ ba, thử nghiệm chỉ gồm môi trường tĩnh trong mô phỏng; chưa đánh giá vật cản động, nhiễu cảm biến ngoài thực tế hoặc sai lệch mô hình truyền động. Các kết quả là bằng chứng thực nghiệm cho cấu hình đã thử, không phải khẳng định ưu thế phổ quát của thuật toán.

IV. KẾT LUẬN

Nghiên cứu đã đánh giá PSTMO cùng bốn phương án đối chứng qua 175 lượt thực thi trên bảy môi trường và năm bộ lập kế hoạch. Trong 34 nhóm đầy đủ, PSTMO có thời gian và sai số bám đường trung bình thấp nhất; so với Simple, mức giảm tương ứng là 1,301 s và 0,0019 m. Tuy nhiên, lợi ích không xuất hiện đồng thời ở mọi trường hợp, và tỷ lệ hoàn thành của các phương án bằng nhau trong tập thử. Do đó, đánh giá bộ làm mượt cần kết hợp chỉ số hình học, hiệu năng thực thi và các lượt thất bại. Hướng tiếp theo là lặp lại phép thử, khảo sát tham số điều khiển và đối chứng, sau đó kiểm chứng trên robot thực.

TÀI LIỆU THAM KHẢO

[1] S. Macenski, F. Martín, R. White và J. Ginés Clavero, “The Marathon 2: A Navigation System,” Proc. IEEE/RSJ IROS, 2020, tr. 2718–2725, doi: 10.1109/IROS45743.2020.9341207.

[2] S. Macenski, T. Moore, D. Lu, A. Merzlyakov và M. Ferguson, “From the Desks of ROS Maintainers: A Survey of Modern & Capable Mobile Robotics Algorithms in the Robot Operating System 2,” Robotics and Autonomous Systems, 2023, mã bài 104493, doi: 10.1016/j.robot.2023.104493.

[3] S. Quinlan và O. Khatib, “Elastic Bands: Connecting Path Planning and Control,” Proc. IEEE ICRA, tập 2, 1993, tr. 802–807, doi: 10.1109/ROBOT.1993.291936.

[4] K. Yang và S. Sukkarieh, “An Analytical Continuous-Curvature Path-Smoothing Algorithm,” IEEE Transactions on Robotics, tập 26, số 3, 2010, tr. 561–568, doi: 10.1109/TRO.2010.2042990.

[5] E. Heiden, L. Palmieri, S. Koenig, K. O. Arras và G. S. Sukhatme, “Gradient-Informed Path Smoothing for Wheeled Mobile Robots,” Proc. IEEE ICRA, 2018, tr. 1710–1717, doi: 10.1109/ICRA.2018.8460818.

[6] S. Macenski, S. Singh, F. Martín và J. Ginés, “Regulated Pure Pursuit for Robot Path Tracking,” Autonomous Robots, 2023. Bản tác giả: arXiv:2305.20026.

[7] H. L. Pham, T. C. Nguyen, V. B. Nguyen, T. P. T. Nguyen và T. L. Pham, “Path Smoothing via Footprint-Aware Corner-Transition Optimization for Differential-Drive Mobile Robots,” bản thảo chưa công bố, 2026.

[8] Navigation2 Project, “Nav2 Smoother,” tài liệu phiên bản Jazzy. https://api.nav2.org/nav2-jazzy/html/md_nav2_smoother_README.html (truy cập 14/10/2026).

[9] Navigation2 Project, “Constrained Smoother,” tài liệu phiên bản Jazzy. https://api.nav2.org/nav2-jazzy/html/md_nav2_constrained_smoother_README.html (truy cập 14/10/2026).
