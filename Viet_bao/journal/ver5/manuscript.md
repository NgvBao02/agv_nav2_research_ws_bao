# II. XÂY DỰNG THUẬT TOÁN PSTMO

## A. Bài toán hậu xử lý và phạm vi của phương pháp

PSTMO nhận đường gấp khúc từ bộ lập kế hoạch toàn cục, bản đồ chi phí, hình chiếu bằng của robot và các giới hạn chuyển động. Hình chiếu bằng mô tả diện tích robot chiếm chỗ trên mặt phẳng di chuyển; đường biên của hình này được gọi là đường bao robot. Thuật toán xử lý từng góc bằng đoạn chuyển tiếp Bézier bậc năm hoặc thao tác quay tại chỗ đáp ứng các ràng buộc. Đầu ra là chuỗi tư thế làm đường tham chiếu cho bộ điều khiển Nav2. Bộ lập kế hoạch xác định hành lang di chuyển, còn bộ điều khiển quyết định vận tốc thực thi.

Gọi Q = {q₀, …, qₙ} là đường đầu vào. Tại đỉnh C, các vectơ đơn vị u và w chỉ hướng của cạnh vào và cạnh ra. Mỗi phương án tại góc phải đáp ứng điều kiện hình học, giới hạn chuyển động và kiểm tra đường bao trên bản đồ. Hai phương án kế tiếp còn phải sử dụng chiều dài tương thích trên cạnh chung. Hình 1 trình bày quan hệ giữa lựa chọn tại từng góc và tối ưu chuỗi chuyển tiếp.

@figure pipeline Luồng xử lý PSTMO và vị trí tích hợp vào Nav2. Các khối biểu diễn các bước của thuật toán; quỹ đạo thực thi trong mô phỏng được thu từ trạng thái do Gazebo cung cấp, độc lập với đường tham chiếu.

Bước tiền xử lý loại các điểm liên tiếp trùng hoặc quá gần nhau và xử lý dao động hình học cục bộ, có kiểm tra va chạm. Cấu hình khảo sát không bật bước rút gọn riêng theo đường nhìn giữa các điểm. Việc xử lý dao động cục bộ vẫn có thể thay đổi đường đầu vào. Kết quả thực nghiệm vì thế đánh giá toàn bộ PSTMO, bao gồm cả tiền xử lý và lựa chọn chuyển tiếp.

## B. Xây dựng đoạn chuyển tiếp có tính liên tục hình học G²

Chiều dài cắt d là khoảng cách từ đỉnh C đến điểm nối trên mỗi cạnh kề. Với d > 0 và hệ số α > 0, đặt A = C − du, B = C + dw và q = αd. Sáu điểm điều khiển được xác định trong công thức 1; đường cong Bézier bậc năm được biểu diễn trong công thức 2. Hình 2 thể hiện các điểm nối và hướng tiếp tuyến tại hai đầu.

@equation P_0=A,\quad P_1=A+qu,\quad P_2=A+2qu; || P_3=B-2qw,\quad P_4=B-qw,\quad P_5=B.

@equation r(t)=\sum_{j=0}^{5}\binom{5}{j}(1-t)^{5-j}t^jP_j,\quad t\in[0,1].

Ba điểm điều khiển ở mỗi đầu thẳng hàng và cách đều. Do đó, r′(0) = 5qu, r′(1) = 5qw và r″(0) = r″(1) = 0. Khi đạo hàm bậc nhất khác không, độ cong hai đầu bằng không và trùng với độ cong của cạnh thẳng. Mối nối liên tục về vị trí, hướng tiếp tuyến và độ cong, đạt G² trên phần đường tịnh tiến có đạo hàm bậc nhất khác không. Phạm vi này không bao gồm thao tác quay tại chỗ hoặc bảo đảm liên tục theo thời gian của vận tốc và gia tốc thực thi.

@figure bezier Cấu trúc Bézier bậc năm cho một góc vuông minh họa. P₀ và P₅ nằm trên hai cạnh; ba điểm đầu và ba điểm cuối thẳng hàng theo các tiếp tuyến tương ứng. Hình không biểu diễn một lượt đo cụ thể.

Công thức 3 xác định độ cong κ và tích phân bình phương độ cong Eκ theo chiều dài cung. Đại lượng Eκ có đơn vị m⁻¹, dùng để đánh giá độ cong trên toàn đoạn xét và không biểu thị năng lượng tiêu thụ của robot. Eκ nhỏ hơn không có nghĩa độ cong nhỏ hơn tại mọi điểm. Thuật toán loại các đường có đạo hàm bậc nhất suy biến, giá trị tính toán không hữu hạn hoặc sự đổi dấu độ cong không đáp ứng điều kiện hình dạng.

@equation \kappa=\frac{x'y''-y'x''}{(x'^2+y'^2)^{3/2}},\qquad E_\kappa=\int\kappa^2\,ds.

## C. Ràng buộc chuyển động và kiểm tra va chạm

Với khoảng cách hai bánh b, vận tốc bánh trái và phải là vL = v(1 − bκ/2) và vR = v(1 + bκ/2). Khi robot di chuyển tiến trên đoạn chuyển tiếp, cả hai hệ số 1 ± bκ/2 phải không âm. Quay tại chỗ được xét như một phương án riêng. Công thức 4 kết hợp giới hạn vận tốc thân xe, vận tốc góc, gia tốc ngang và vận tốc bánh. Hai lượt tính toán từ đầu đến cuối đoạn và theo chiều ngược lại tiếp tục áp đặt giới hạn tăng tốc, giảm tốc và chuyển động góc. Đây là thứ tự tính toán, không phải yêu cầu robot di chuyển lùi.

@equation v\leq\min\left(v_{\max},\frac{\omega_{\max}}{\left|\kappa\right|},\sqrt{\frac{a_{\mathrm{lat},\max}}{\left|\kappa\right|}},\frac{v_{\mathrm{wheel},\max}}{f_{\max}}\right), || f_{\max}=\max\left(\left|1-b\kappa/2\right|,\left|1+b\kappa/2\right|\right).

Tại đoạn thẳng, các hạng chia cho |κ| trong công thức 4 không tạo thêm giới hạn hữu hạn. Thuật toán kiểm tra cả chi phí tại tâm và đường bao được đặt theo vị trí, hướng của robot. Chi phí tại tâm phản ánh vùng chi phí mở rộng quanh vật cản; kiểm tra đường bao xét thêm kích thước và hướng thân xe. Phép kiểm tra được thực hiện tại các tư thế lấy mẫu nên phụ thuộc độ phân giải bản đồ và khoảng cách lấy mẫu, chưa phải bảo đảm không va chạm liên tục giữa mọi tư thế.

## D. Xây dựng và lựa chọn phương án chuyển động tại góc

Hai phương án được so sánh trên cùng đoạn từ vị trí bắt đầu đến vị trí kết thúc xử lý góc. Đoạn chuyển tiếp duy trì chuyển động tiến trong giới hạn của từng bánh. Phương án quay tại chỗ đưa vận tốc tịnh tiến về không rồi thay đổi hướng theo giới hạn quay riêng. Như vậy, thuật toán có thể lựa chọn quay tại chỗ khi đoạn chuyển tiếp không đáp ứng yêu cầu.

PSTMO xét hai chiều dài cắt tại mỗi góc. Chiều dài ưu tiên được chặn bởi hai cạnh kề và giới hạn trên dmax. Chiều dài thứ hai dành phần cạnh chung cho góc tiếp theo. Với cạnh dài L và khoảng dự phòng m, phần chiều dài dành cho mỗi đầu là (L − m)/2. Thuật toán bỏ các chiều dài cắt trùng nhau ở độ phân giải tìm kiếm hoặc nhỏ hơn dmin. Hai lựa chọn cho phép cân bằng độ rộng chuyển tiếp với khả năng ghép các góc liên tiếp.

Với mỗi chiều dài cắt, thuật toán xét tập α = {0,1; 0,2; 0,3; 0,4; 0,5}. Nếu chưa có đường cong khả thi, thuật toán xét thêm {0,15; 0,25; 0,35; 0,45}, rồi tinh chỉnh α quanh giá trị tốt nhất và loại các giá trị trùng. Bước tìm hình dạng chọn đường cong có tích phân bình phương độ cong nhỏ nhất trong tập khả thi. Tiêu chí này khác với hàm chi phí tổng hợp dùng để lựa chọn phương án chuyển động ở bước sau.

Thời gian của các phương án được tính trên cùng đoạn xử lý góc, với cùng giới hạn vận tốc đầu và cuối có thể đạt được. Khi quay tại chỗ khả thi, phương án chuyển tiếp chỉ được xem xét nếu thỏa công thức 6. Trong đó, Tmin là thời gian ngắn nhất của các chuyển tiếp khả thi, Tp là thời gian của phương án quay tại chỗ và δT là mức giảm thời gian tối thiểu yêu cầu. Chuyển tiếp được chọn phải nằm trong khoảng thời gian cho phép so với Tmin và vẫn nhanh hơn phương án quay tại chỗ ít nhất δT.

Hàm chi phí trong công thức 5 dùng để lựa chọn phương án tại góc. Gọi cmax là chi phí bản đồ lớn nhất tại các mẫu tâm và ωpeak là trị tuyệt đối vận tốc góc lớn nhất. Các thành phần chuẩn hóa là ρc = min(1, cmax/252), ρω = min(1, ωpeak/ωmax) và ρE = Eκ/(Eκ + Escale), lần lượt tương ứng với chi phí vật cản, vận tốc góc và tích phân bình phương độ cong. Ba trọng số có tổng bằng một. Phương án quay tại chỗ nhận chi phí 1 nếu có chuyển tiếp đáp ứng điều kiện thời gian, và 0 nếu không có.

@equation J=0.15\rho_c+0.10\rho_\omega+0.75\rho_E.

@equation T_{\min}+\delta_T<T_p.

Nếu quay tại chỗ không khả thi, thuật toán vẫn xét chuyển tiếp đáp ứng các điều kiện còn lại. Ngược lại, nếu không có chuyển tiếp đáp ứng yêu cầu, thuật toán sử dụng phương án quay tại chỗ khả thi. Chuỗi chuyển động có thể kết hợp cả hai phương án; đường sau khi ghép tiếp tục được kiểm tra trước khi chuyển tới bộ điều khiển.

## E. Lựa chọn chuỗi chuyển tiếp và tạo đường tham chiếu

Gọi Sᵢ là tập phương án khả thi tại góc i; mỗi phương án tương ứng với một trạng thái trong bài toán quy hoạch động. Chuyển tiếp sử dụng chiều dài cắt dᵢ, còn quay tại chỗ sử dụng chiều dài bằng không. Hai phương án liên tiếp tương thích khi tổng chiều dài cắt và khoảng dự phòng m không vượt chiều dài cạnh chung. Với chi phí Jᵢ(k) của phương án k, công thức 7 xác định Fᵢ(k), là tổng chi phí nhỏ nhất đến phương án đó tại góc i. Truy vết từ góc cuối xác định chuỗi phương án được chọn.

@equation F_i(k)=J_i(k)+\min_{j\in A_i(k)}F_{i-1}(j).

Trong công thức 7, Aᵢ(k) = {j: dᵢ₋₁(j) + dᵢ(k) + m ≤ Lᵢ₋₁} là tập phương án tương thích tại góc liền trước. Khi chi phí bằng nhau trong sai số số học, thuật toán ưu tiên chuỗi có ít trạng thái quay tại chỗ hơn. Nếu vẫn bằng nhau, thuật toán chọn theo thứ tự đã xác định của các phương án. Với Kᵢ trạng thái tại lớp i, số phép tính tỷ lệ với ∑Kᵢ₋₁Kᵢ và bộ nhớ truy vết tỷ lệ với ∑Kᵢ. Tính tối ưu áp dụng cho đồ thị các phương án khả thi đã xây dựng.

Sau khi ghép, PSTMO kiểm tra vị trí đầu–cuối, giới hạn chuyển động, đường bao tại các mối nối và hướng cuối. Đường chỉ được chấp nhận khi vượt qua toàn bộ kiểm tra. Bảng I trình bày cấu hình khảo sát. Đường được chuyển tới bộ điều khiển Regulated Pure Pursuit, viết tắt RPP [1]. RPP tạo lệnh vận tốc khi bám đường; thời gian dự đoán tại góc và thời gian thực thi có phản hồi điều khiển được đánh giá riêng.

@table params Cấu hình PSTMO trong nghiên cứu

@note Ký hiệu δT là mức giảm thời gian tối thiểu đối với phương án quay tại chỗ; khoảng thời gian cho phép so với Tmin còn bị giới hạn bởi điều kiện này. Khoảng cách lấy mẫu đường đầu ra khác khoảng cách lấy mẫu kiểm tra đường cong. Khoảng dự phòng trên cạnh chung 0,05 m được suy ra từ cấu hình và độ phân giải bản đồ.

## F. Trình tự thực hiện và điều kiện chấp nhận

Sau khi nhận dữ liệu hình học và giới hạn chuyển động, PSTMO tiền xử lý đường, xác định các góc và kiểm tra khả năng quay tại chỗ. Với hai chiều dài cắt, thuật toán xây dựng đường cong, kiểm tra hình dạng và va chạm, rồi tính vận tốc và thời gian di chuyển dọc đoạn. Các phương án đáp ứng điều kiện thời gian được đánh giá bằng hàm chi phí và đưa vào bài toán quy hoạch động.

Quy hoạch động liên kết các trạng thái tương thích và truy vết chuỗi có tổng chi phí nhỏ nhất. Các đoạn thẳng, chuyển tiếp và tư thế quay tại chỗ được ghép theo thứ tự đường đi. Bước cuối kiểm tra toàn bộ đầu ra và bảo toàn tư thế đích. Thuật toán từ chối đường nếu thiếu trạng thái khả thi, không có chuỗi tương thích hoặc không đáp ứng phép kiểm tra đầu ra.

Ba bước xây dựng phương án, lựa chọn chuỗi và kiểm tra đầu ra có vai trò khác nhau. Bước đầu xác định các đường cong và phương án quay tại chỗ khả thi. Quy hoạch động chọn chuỗi có tổng chi phí nhỏ nhất trong tập này. Phép kiểm tra cuối xác nhận đường ghép đáp ứng các điều kiện đầu ra đã định trước khi đánh giá chuyển động qua bộ điều khiển.

Hai chiều dài cắt và phép tìm kiếm phân cấp giới hạn số đường cong cần đánh giá nhưng có thể bỏ sót hình dạng khả thi khác. Hàm chi phí J cũng chưa mô tả đầy đủ thời gian thực thi khi có phản hồi điều khiển. Phần III đánh giá chuỗi được chọn bằng thời gian thực thi, sai số bám đường và tỷ lệ hoàn thành nhiệm vụ.

# III. XÂY DỰNG KỊCH BẢN THỬ NGHIỆM VÀ PHÂN TÍCH

## A. Môi trường mô phỏng và thiết kế đánh giá

### 1) Robot và chuỗi điều hướng

Thử nghiệm sử dụng robot vi sai trong môi trường mô phỏng có các dãy vật cản song song và hành lang giao cắt. Hệ thống gồm ROS 2 Jazzy, Nav2 và Gazebo. Sau hậu xử lý, đường toàn cục được dùng làm tham chiếu cho RPP. Lệnh vận tốc được xử lý qua bộ làm mượt vận tốc trước khi robot thực thi; bộ giám sát va chạm hoạt động trong cùng hệ điều hướng. Trong mỗi nhóm tuyến–bộ lập kế hoạch, các phương án dùng chung bản đồ, robot, bộ điều khiển và tiêu chí đích.

@figure robot Sơ đồ đường bao chữ nhật và khoảng cách bánh dùng trong mô hình PSTMO. O là gốc biểu diễn đường bao, x hướng về phía trước. Khoảng cách bánh hiệu dụng của mô-đun động lực học được báo riêng trong Bảng II; sơ đồ không phải ảnh chụp robot thực.

Hình 3 và Bảng II phân biệt hai khoảng cách bánh trong mô hình. PSTMO dùng 0,2548 m để kiểm tra động học; mô phỏng dẫn động dùng giá trị hiệu dụng 0,2834 m. Hai thông số được giữ theo cấu hình đã sinh dữ liệu. Sự khác biệt cần được tính đến khi đối chiếu giới hạn vận tốc bánh với đáp ứng chuyển động của robot.

@table robot Tham số robot và giao thức điều hướng

RPP sử dụng vận tốc đặt 0,30 m/s và khoảng nhìn trước cố định 0,50 m [1]. Chức năng điều chỉnh vận tốc theo độ cong và chi phí được tắt trong thử nghiệm. Các chức năng kiểm tra va chạm, xoay hướng và giảm tốc gần đích vẫn hoạt động, cùng các giới hạn vận tốc phía sau. Do đó, kết quả phản ánh khả năng bám các đường tham chiếu dưới cấu hình điều khiển chung này.

### 2) Bản đồ và năm bài toán di chuyển

Hình 4 thể hiện năm tuyến trên bản đồ chiếm chỗ có độ phân giải 0,05 m/ô. Các đường gốc do Theta* tạo được dùng để minh họa vị trí từng tuyến. Bảng III liệt kê tọa độ đầu và đích. R01 chuyển lối qua giao cắt; R02 đi chéo dài; R03 chuyển lối hình chữ U; R04 đi chéo ngược; R05 rẽ từ hành lang ngang vào hành lang dọc.

@figure routes Vị trí năm tuyến trong cùng bản đồ mô phỏng. Mỗi màu là một đường gốc của Theta*; chấm tròn chỉ xuất phát và ngôi sao chỉ đích. Các đường có thể chồng lên nhau trên những hành lang dùng chung.

@table scenarios Tọa độ đầu và đích trong hệ tọa độ bản đồ

Hướng đích của R01–R05 lần lượt là 0,950547 rad, π/2, −π/2, −π/2 và π/2. Tư thế xuất phát của từng lượt được lưu trong hồ sơ thực nghiệm. Cả vị trí và hướng đều tham gia xác định nhiệm vụ. Robot tới gần tọa độ đích vẫn cần đáp ứng sai số hướng và điều kiện dừng để được ghi nhận hoàn thành.

Năm tuyến khảo sát các dạng chuyển hướng trên cùng một bố trí vật cản. Các tuyến dùng chung độ phân giải bản đồ và một số đoạn hành lang. R04 khác R02 cả điểm đích và cách tiếp cận, nên không tạo thành cặp đối xứng hoàn toàn. R05 có quãng đường ngắn nhưng tập trung chuyển hướng và tiếp cận đích trong một vùng nhỏ.

### 3) Bộ lập kế hoạch và các phương án đối chứng

Năm bộ lập kế hoạch gồm NavFn A*, NavFn Dijkstra, Theta*, Smac 2D và Smac Hybrid. Hai cấu hình NavFn tạo đường trên lưới; Theta* bổ sung đường theo hướng tự do. Smac 2D và Smac Hybrid cung cấp thêm các cấu trúc đường đầu vào. Smac Hybrid sử dụng mô hình Dubins, bán kính quay nhỏ nhất 0,35 m và 72 ô hướng; chức năng làm mượt nội bộ được tắt. Giới hạn bán kính này thuộc bộ lập kế hoạch, không thay thế mô hình quay tại chỗ của PSTMO.

Năm phương án xử lý gồm đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO. Đường gốc giữ nguyên đầu ra của bộ lập kế hoạch. Simple làm mượt lặp; Savitzky–Golay, viết tắt SG trong bảng và hình, dựa trên bộ lọc bình phương tối thiểu cục bộ [2]. Constrained tối ưu số các thành phần chất lượng đường [3]. Bảng IV trình bày cấu hình từng phương án.

@table baselines Cấu hình các phương án hậu xử lý

Constrained được thử với trọng số độ cong và bán kính quay nhỏ nhất bằng không. Kết quả vì vậy chỉ đại diện cho cấu hình này. Các phương án dùng chung bản đồ và đường bao robot, nhưng có cơ chế kiểm tra nội bộ khác nhau. Nghiên cứu chưa tối ưu tham số cho từng phương án với cùng giới hạn số lần đánh giá tham số. Các khác biệt này được xem xét khi diễn giải so sánh.

### 4) Hai bộ dữ liệu và kiểm soát đầu vào

Ma trận khảo sát gồm 125 lượt, tương ứng 5 tuyến × 5 bộ lập kế hoạch × 5 phương án. Mỗi tổ hợp được thực thi một lần. R01 sử dụng dữ liệu ngày 02–03/08/2026; R02–R05 thuộc đợt bổ sung sau đó. Hình và chỉ số được lấy từ cùng bản ghi thực thi. Mã kiểm tra của đường gốc được đối chiếu trong từng nhóm để xác nhận các phương án nhận cùng đầu vào hình học.

Ma trận 125 lượt được dùng để phân tích mô tả theo tuyến và bộ lập kế hoạch. Năm bộ lập kế hoạch tạo năm đầu vào khác nhau, không phải năm lần lặp của một đầu vào. Do đó, các bảng theo tuyến báo trung bình mô tả, không gán khoảng tin cậy từ việc gộp các bộ lập kế hoạch.

Bộ kiểm chứng lặp gồm 250 lượt PSTMO và Constrained: 5 tuyến × 5 bộ lập kế hoạch × 2 phương án × 5 lần lặp. Bộ này dùng một phiên bản phần mềm và cấu hình cố định. Mỗi lượt khởi động mới hệ thống; bốn tiến trình chạy trong các miền ROS và phân vùng Gazebo tách biệt. Các lần lặp đo biến thiên thực thi với đầu vào cố định, không khảo sát nhiều giá trị khởi tạo của bộ sinh số giả ngẫu nhiên. Kết quả được phân tích riêng tại Mục III-H.

### 5) Tiêu chí hoàn thành và xử lý lỗi

Một lượt đạt khi tác vụ điều hướng thành công, sai số vị trí không vượt 0,10 m và sai số hướng không vượt 0,15 rad. Robot đồng thời phải dừng với |v| ≤ 0,01 m/s và |ω| ≤ 0,02 rad/s. Điều kiện dừng được kiểm tra qua ba vòng quan sát liên tiếp trong thời gian chờ tối đa 3 s. Sai số được tính từ trạng thái mô phỏng. Dung sai đánh giá sau thực thi được phân biệt với dung sai bộ kiểm tra đích Nav2 trong Bảng II.

Các lượt không hoàn thành nhiệm vụ được giữ trong mẫu số tính tỷ lệ thành công. Chính sách thử lại chỉ áp dụng cho lỗi khởi tạo hạ tầng. Bộ kiểm chứng lặp có 14 lượt cần ít nhất một lần thử lại loại này; hồ sơ lưu riêng các lần gián đoạn và kết quả đã chốt. Các lượt thất bại sau khi thực thi không được chạy lại để thay kết quả.

Phân tích ghép cặp các chỉ số liên tục chỉ sử dụng các cặp mà cả hai phương án đều đạt. Các lượt còn lại được báo riêng bằng trạng thái kết thúc và sai số đích. Quy tắc này phân biệt thời gian hoàn thành với thời gian bị hủy trước đích. Mỗi bảng và phép so sánh đều nêu rõ tập lượt được sử dụng.

### 6) Chỉ số đo và cách diễn giải

Thời gian thực thi T được đo bằng đồng hồ mô phỏng, gồm thời gian điều hướng và thời gian chờ robot dừng. Thời lượng hậu xử lý do tác vụ làm mượt Nav2 trả về, được tính bằng đồng hồ của nút phần mềm [4]. Khi dùng thời gian mô phỏng, đại lượng này không phải thời gian CPU hoặc thời gian thực độc lập của máy tính.

Sai số bám đường là khoảng cách nhỏ nhất từ vị trí robot xₖ đến các đoạn của đường tham chiếu P. Với N vị trí, căn trung bình bình phương sai số, viết tắt RMSE, được xác định trong công thức 8. Mỗi phương án được đánh giá theo đường tham chiếu riêng. Vì vậy, RMSE phản ánh khả năng bám đường tham chiếu, không phải sai số định vị.

@equation e_{\mathrm{RMSE}}=\sqrt{\frac{1}{N}\sum_{k=1}^{N}\operatorname{dist}(x_k,P)^2}.

Giá trị xấp xỉ của tích phân bình phương độ cong được tính từ các bộ ba điểm liên tiếp. Với độ dài ba cạnh a, b, c và diện tích tam giác có dấu A, độ cong mẫu bằng 4A/(abc). Mỗi giá trị κ² được nhân với nửa tổng chiều dài hai cạnh kề rồi cộng trên toàn đường. Các bộ ba suy biến được bỏ qua theo ngưỡng số học. Chỉ số phụ thuộc cách lấy mẫu, đặc biệt khi đường có các đoạn rất ngắn.

Quỹ đạo thực thi được lấy mẫu lại theo chiều dài cung với bước 0,05 m. Sau đó, phép trung bình trượt đối xứng sử dụng hai mẫu ở mỗi phía để làm mượt dữ liệu, đồng thời giữ hai đầu đường. Cùng quy trình được áp dụng trước khi tính RMSE và tích phân bình phương độ cong cho mọi phương án. RMSE vì vậy được báo trên quỹ đạo đã xử lý.

Ký hiệu Eκ,ref chỉ tích phân bình phương độ cong của đường tham chiếu; Eκ,exec chỉ tích phân bình phương độ cong của quỹ đạo thực thi đã xử lý trong mô phỏng. Các giá trị báo cáo là xấp xỉ từ dữ liệu rời rạc theo quy trình lấy mẫu tương ứng. Hồ sơ còn giữ chỉ số của phần tịnh tiến để phân biệt với các tư thế quay tại chỗ. Mức giảm Eκ,ref không mặc nhiên dẫn đến mức giảm tương ứng của Eκ,exec.

Khoảng hở c là khoảng cách nhỏ nhất giữa đường bao đã định hướng và vật cản trên bản đồ tĩnh. Phép đo khoảng hở nội suy tư thế với bước tịnh tiến 0,05 m và bước hướng 5°. Đường bao được lấy mẫu với bước không quá 0,025 m. Trường khoảng cách được hiệu chỉnh nửa đường chéo ô. Ô chưa biết và ô có mức chiếm chỗ từ 65 trở lên được xem là vật cản. Độ phân giải bản đồ giới hạn cách diễn giải những chênh lệch khoảng hở rất nhỏ.

Các chỉ số bổ sung gồm sai số đích, quãng đường thực thi, phân vị 95% của trị tuyệt đối độ giật tịnh tiến và thời lượng hậu xử lý. Hệ thống cũng ghi số lần bộ giám sát va chạm can thiệp và số mẫu đường bao va chạm. Độ giật là đạo hàm của gia tốc theo thời gian; trong nghiên cứu này, đại lượng đó được ước lượng từ chuỗi vận tốc mô phỏng bằng sai phân và phụ thuộc nhịp lấy mẫu. Nghiên cứu chưa đo điện năng, rung tải hàng hoặc lực tiếp xúc.

## B. R01 Chuyển lối tại giao cắt

### 1) Mục tiêu và hình học của tuyến

R01 chuyển từ hành lang phía dưới tại x = −2 m sang hành lang phía trên tại x = 2 m. Robot đổi hướng qua hành lang ngang trước khi tiếp tục dọc các dãy vật cản. Tuyến này khảo sát các mối nối giữa đoạn thẳng và vùng rẽ. Trên đường tìm kiếm theo lưới, nhiều đoạn ngắn có thể tạo biến thiên độ cong cục bộ dù vẫn nằm trong cùng hành lang.

Hình 5 so sánh năm đường tham chiếu khi sử dụng Theta*. Bộ lập kế hoạch này được giữ thống nhất trong các hình quỹ đạo R01–R05. Vùng phóng to được chọn tại điểm trên đường PSTMO có khoảng cách lớn nhất tới các đỉnh đường gốc. Quy tắc cố định giúp quan sát sự khác biệt cục bộ giữa các phương án trên cùng hệ tọa độ.

@table r01 Kết quả mô tả R01 trên năm bộ lập kế hoạch

@note T và e là trung bình các lượt đạt; Eκ,ref và c lấy trên năm đường tham chiếu. Eκ,ref có đơn vị m⁻¹; c là trung bình của khoảng hở nhỏ nhất từng đường, không phải khoảng hở nhỏ nhất của cả nhóm. SG viết tắt Savitzky–Golay.

@figure r01_paths R01 với Theta*: (a) đường tham chiếu trên toàn tuyến; (b) vùng phóng to. Cùng màu và kiểu nét được dùng xuyên suốt các tuyến. Đây là đường tham chiếu, không phải quỹ đạo thực thi của robot.

### 2) Thời gian thực thi và tích phân bình phương độ cong

Bảng V cho thấy cả 25 lượt R01 đều hoàn thành. PSTMO có thời gian trung bình 39,74 s, trong khi đường gốc có giá trị 43,50 s. Simple đạt 41,49 s, Savitzky–Golay đạt 42,88 s và Constrained đạt 43,21 s. RMSE của PSTMO là 21,9 mm, so với 24,4 mm của Constrained. PSTMO có thời gian và RMSE trung bình thấp hơn Constrained trên tuyến này.

Tích phân bình phương độ cong của đường tham chiếu PSTMO là 2,38 m⁻¹; các giá trị tương ứng của Constrained và đường gốc là 18,62 m⁻¹ và 146,25 m⁻¹. Kết quả xác nhận thay đổi hình học rõ rệt, trong khi mức giảm thời gian nhỏ hơn về tỷ lệ. Hình 6 đối chiếu trực tiếp thời gian và RMSE theo từng bộ lập kế hoạch. Cách so sánh này cho phép đánh giá phần thay đổi hình học đi kèm thay đổi thực thi.

@table r01_paired Chênh lệch PSTMO trừ Constrained ở R01

@figure r01_metrics R01: thời gian thực thi và RMSE của đủ năm phương án trên từng bộ lập kế hoạch. Các cột là một lượt thực thi, không phải trung bình nhiều lần lặp; không vẽ thanh sai số.

### 3) Đánh đổi khoảng hở và diễn giải

Trong năm cặp theo bộ lập kế hoạch, PSTMO nhanh hơn đường gốc, Savitzky–Golay và Constrained ở cả năm cặp; nhanh hơn Simple ở bốn cặp. Tuy nhiên, Bảng VI cho thấy RMSE tăng so với Constrained khi dùng Theta* và Smac Hybrid, lần lượt 1,87 và 0,97 mm. Như vậy, lợi thế thời gian nhất quán hơn lợi thế bám đường ở R01.

Khoảng hở nhỏ nhất trung bình của PSTMO là 15,7 cm, so với 23,5 cm của Constrained. Đường có tích phân bình phương độ cong thấp hơn nhưng sử dụng phần hành lang gần vật cản hơn. Đây là đánh đổi hình học quan sát được; mức đóng góp của từng trọng số mục tiêu chưa được tách riêng.

Thời lượng hậu xử lý của PSTMO ở R01 khoảng 0,102 s, trong khi Constrained là 0,026 s. Mức tăng này cần được xem cùng lợi ích thời gian thực thi. Hai đại lượng có phạm vi đo riêng theo Mục III-A, nên chưa xác định đầy đủ độ trễ toàn hệ thống. Kết quả R01 cho thấy khả năng giảm thời gian nhiệm vụ đi cùng khoảng hở nhỏ hơn và thời lượng hậu xử lý dài hơn.

## C. R02 Di chuyển chéo dài

### 1) Tác động của chiều dài và các đoạn thẳng

R02 nối (−4,7; −2,8) m với (4,7; 2,8) m qua hành lang ngang. So với R01, tuyến trải trên phạm vi bản đồ rộng hơn và gồm các đoạn gần thẳng dài hơn. Tuyến này khảo sát hiệu quả hậu xử lý khi vùng chuyển hướng chỉ chiếm một phần đường đi. Vì thời gian còn phụ thuộc các đoạn thẳng và tiếp cận đích, mức giảm Eκ,ref cần được đối chiếu với thời gian toàn nhiệm vụ.

Hình 7 cho thấy các phương án đi qua cùng hệ hành lang và khác nhau chủ yếu ở vùng chuyển hướng. Trong hình toàn tuyến, nhiều đường gần như chồng lên nhau. Vùng phóng to thể hiện độ lệch cục bộ theo đơn vị mét, cùng vị trí vật cản xung quanh. Bảng VII tổng hợp kết quả của cả năm bộ lập kế hoạch.

@figure r02_paths R02 với Theta*: đường tham chiếu toàn tuyến và vùng phóng to theo cùng quy tắc chọn vùng như R01. Trục tọa độ có đơn vị mét; tỷ lệ hiển thị toàn tuyến và vùng phóng to khác nhau.

@table r02 Kết quả mô tả R02 trên năm bộ lập kế hoạch

@note T và e lấy trên lượt đạt; Eκ,ref và c lấy trên đường tham chiếu. Cả 25 lượt R02 đều đạt. Các giá trị c là trung bình của cực tiểu từng đường.

### 2) Hiệu quả thời gian không đồng nghĩa cải thiện mọi chỉ số

PSTMO đạt thời gian trung bình 55,39 s, thấp nhất trong năm phương án ở R02. Đường gốc đạt 59,57 s, Savitzky–Golay 58,37 s và Constrained 59,06 s. Simple đạt 56,40 s, gần PSTMO hơn các đối chứng còn lại. Vì vậy, phần lợi ích bổ sung so với Simple nhỏ hơn lợi ích so với đường chưa xử lý.

Tuy nhiên, RMSE trung bình của PSTMO là 24,2 mm, trong khi đường gốc là 21,3 mm; ba đối chứng làm mượt cũng có RMSE thấp hơn PSTMO. Simple, Savitzky–Golay và Constrained lần lượt đạt 21,1; 19,9 và 21,7 mm. Tích phân bình phương độ cong của đường tham chiếu PSTMO là 2,48 m⁻¹. Trường hợp này cho thấy giảm tích phân bình phương độ cong chưa bảo đảm giảm sai số bám đường dưới cấu hình RPP đã dùng.

@figure r02_metrics R02: thời gian và RMSE theo bộ lập kế hoạch. Sự khác biệt giữa hai hàng chỉ số cho phép nhận diện trường hợp nhanh hơn nhưng bám đường kém hơn; không suy ra ưu thế toàn diện từ một hàng duy nhất.

@table r02_paired Chênh lệch PSTMO trừ Constrained ở R02

Bảng VIII cho thấy PSTMO nhanh hơn Constrained ở bốn trong năm cặp. Mức giảm lớn nhất là 8,32 s với NavFn A*, trong khi Smac Hybrid chậm hơn 0,49 s. RMSE tăng với Theta*, Smac 2D và Smac Hybrid. Khi so với Simple, PSTMO chỉ nhanh hơn ở 2/5 cặp dù thời gian trung bình thấp hơn; RMSE cao hơn ở cả năm cặp. Như vậy, thời gian trung bình thấp hơn không đồng nghĩa nhanh hơn trên đa số đầu vào.

### 3) Khoảng hở và ý nghĩa vận hành

Khoảng hở nhỏ nhất trung bình của PSTMO là 19,3 cm, so với 22,9 cm của Constrained. Mức giảm khoảng hở đi cùng giảm thời gian nhưng tăng RMSE. Khi lựa chọn phương án cho hành lang hẹp, cần đối chiếu cả khoảng hở và sai số bám đường. Trong thử nghiệm này, phép kiểm tra đường tham chiếu không phát hiện mẫu đường bao giao cắt vật cản.

R02 biểu hiện rõ đánh đổi giữa tốc độ hoàn thành và độ bám đường. Hình dạng tham chiếu thay đổi cùng đáp ứng của bộ điều khiển, nên dữ liệu chưa tách được đóng góp riêng của từng yếu tố. Kết quả thích hợp để lựa chọn ưu tiên theo yêu cầu nhiệm vụ. Một đánh giá chỉ dùng thời gian hoặc Eκ,ref sẽ bỏ qua RMSE cao hơn của PSTMO trên tuyến này.

## D. R03 Chuyển lối hình chữ U

### 1) Chuỗi góc liên tiếp và khả năng tương thích

R03 bắt đầu tại (−2,0; −2,8) m và kết thúc tại (2,0; −2,8) m, với hướng đích −π/2. Robot đi vào hệ hành lang rồi trở về hàng ngang xuất phát ở một lối khác. Các góc rẽ liên tiếp dùng chung chiều dài của những đoạn nối. Vì vậy, tăng chiều dài cắt ở một góc có thể hạn chế lựa chọn tại góc kế tiếp.

@figure r03_paths R03 với Theta*: đường chữ U và vùng chuyển hướng được phóng to. Hình biểu diễn một bộ lập kế hoạch cố định; trường hợp không đạt ở tuyến này thuộc Smac Hybrid và được báo riêng.

Hình 9 thể hiện đường chữ U và vùng chuyển hướng khi sử dụng Theta*. Đây là tình huống phù hợp để khảo sát khả năng ghép các chuyển tiếp của toàn bộ PSTMO. Các lượt hiện có đều dùng đầy đủ thuật toán. Việc xác định riêng đóng góp của quy hoạch động cần thêm thí nghiệm loại bỏ thành phần, được nêu tại Mục III-I.

@table r03 Kết quả mô tả R03 trên năm bộ lập kế hoạch

@note PSTMO có 4/5 lượt đạt; các phương án còn lại có 5/5. T và e của PSTMO chỉ lấy trên bốn lượt đạt. Eκ,ref và c vẫn lấy trên đủ năm đường; không diễn giải chênh lệch T giữa các trung bình này như một ước lượng ghép cặp.

### 2) Thời gian thực thi và trường hợp không đạt dung sai đích

Bảng IX ghi nhận PSTMO đạt 4/5 lượt, trong khi các phương án còn lại đạt 5/5. Thời gian trung bình của các lượt PSTMO đạt là 41,94 s; Simple đạt 42,34 s và Constrained 44,15 s. Do các trung bình dùng số lượt khác nhau, Bảng X cung cấp thêm chênh lệch trên bốn cặp có cả hai phương án đạt. Cặp Smac Hybrid được để trống trong phép so sánh liên tục.

Lượt PSTMO với Smac Hybrid hoàn tất tác vụ điều hướng nhưng có sai số vị trí cuối 0,102385 m. Giá trị này vượt ngưỡng 0,10 m khoảng 2,385 mm, nên lượt được phân loại không đạt. Hình 10 đánh dấu riêng kết quả này. Việc báo đồng thời trạng thái và sai số cho thấy mức vượt ngưỡng cụ thể, thay vì chỉ ghi một nhãn thất bại.

@figure r03_metrics R03: cột gạch và dấu × đánh dấu PSTMO/Smac Hybrid không đạt. Giá trị ghi ở cột đó chỉ mô tả trạng thái kết thúc; không được tính vào trung bình hoàn thành hoặc dùng để kết luận thời gian tốt hơn.

@table r03_paired Chênh lệch ghép cặp trên các lượt cùng đạt ở R03

### 3) So sánh với Simple và giới hạn của chỉ số đường tham chiếu

Tích phân bình phương độ cong trung bình của đường tham chiếu là 5,47 m⁻¹ với PSTMO, 5,78 m⁻¹ với Simple và 22,93 m⁻¹ với Constrained. Chênh lệch hình học giữa PSTMO và Simple nhỏ hơn rõ rệt so với R01. RMSE trên các lượt đạt cũng gần nhau: 27,2 mm với PSTMO và 27,0 mm với Simple. Kết quả này cho thấy mức cải thiện của toàn bộ PSTMO thay đổi theo cấu trúc đường đầu vào; dữ liệu chưa tách riêng tác dụng của chuyển tiếp Bézier.

Trên bốn cặp có cả hai phương án đạt, PSTMO nhanh hơn đường gốc, Savitzky–Golay và Constrained ở cả bốn cặp; nhanh hơn Simple ở ba cặp. RMSE chỉ thấp hơn Simple ở một cặp, trong khi thấp hơn Constrained ở ba cặp. Bảng X xác định ngoại lệ với Constrained là Theta*, có RMSE tăng 5,36 mm. Tốc độ hoàn thành và độ bám đường tiếp tục cho thứ hạng khác nhau.

Khoảng hở nhỏ nhất trung bình của PSTMO là 18,8 cm, so với Constrained 21,4 cm. Cùng với lượt không đạt, kết quả này đặt ra hai điều kiện riêng cho triển khai: đủ khoảng cách vật cản và dừng trong dung sai đích. Kiểm tra hình học ở đầu ra PSTMO không thay thế đánh giá chuyển động thực. Kết quả thử nghiệm lặp tại Mục III-H được dùng để xem xét thêm tính lặp lại của hiệu năng.

## E. R04 Chuyển lối chéo ngược

### 1) Đổi chiều tiếp cận và vai trò của tư thế đích

R04 nối (−4,6; 2,8) m với (2,0; −2,8) m và kết thúc ở hướng −π/2. So với R02, robot tiếp cận hành lang ngang từ phía đối diện và đi tới điểm đích khác. Tuyến này khảo sát ảnh hưởng của thứ tự các đoạn chuyển hướng. Các chuyển tiếp phải phù hợp với tiếp tuyến đầu–cuối và khoảng trống theo chiều di chuyển.

Hình 11 cho thấy khác biệt cục bộ giữa năm đường tham chiếu khi sử dụng Theta*. Kết quả với bốn bộ lập kế hoạch còn lại được thể hiện trong Bảng XI và Hình 12. Các đường gần nhau trên hình toàn tuyến vẫn có thể tạo thời gian và RMSE khác nhau. Vì vậy, nhận xét hình học được đối chiếu với cả hai chỉ số thực thi.

@figure r04_paths R04 với Theta*: toàn tuyến chéo ngược và vùng phóng to. Việc đảo chiều tiếp cận không làm tuyến này trở thành bản sao đối xứng của R02.

@table r04 Kết quả mô tả R04 trên năm bộ lập kế hoạch

@note Đường gốc có 4/5 lượt đạt; bốn phương án hậu xử lý có 5/5. T và e của đường gốc dùng bốn lượt đạt, còn Eκ,ref và c vẫn dùng đủ năm đường. So sánh PSTMO–Constrained không bị thiếu cặp ở R04.

### 2) Kết quả thời gian và bám đường

PSTMO hoàn thành cả năm lượt R04, đạt thời gian trung bình 47,71 s và RMSE 24,4 mm. Các giá trị tương ứng của Simple là 49,06 s và 28,4 mm; Savitzky–Golay là 50,46 s và 28,8 mm. Constrained đạt 50,83 s và 30,9 mm. Khác với R02, PSTMO có cả thời gian và RMSE trung bình thấp hơn các đối chứng làm mượt.

Tích phân bình phương độ cong của đường PSTMO là 2,89 m⁻¹, so với Simple 5,99 m⁻¹ và Constrained 28,31 m⁻¹. Khoảng hở nhỏ nhất trung bình giảm còn 19,3 cm, trong khi Constrained đạt 21,6 cm. Như vậy, cải thiện đồng thời thời gian và RMSE vẫn đi kèm giảm khoảng cách tới vật cản.

@figure r04_metrics R04: thời gian và RMSE trên từng bộ lập kế hoạch. Đường gốc/Theta* được đánh dấu không đạt do sai số vị trí cuối vượt ngưỡng; các đường đã làm mượt đều hoàn thành trong ma trận một lượt này.

@table r04_paired Chênh lệch PSTMO trừ Constrained ở R04

### 3) Trường hợp sát ngưỡng và tiêu chí hoàn thành

Bảng XII cho thấy PSTMO nhanh hơn Constrained ở bốn cặp; với Smac Hybrid, thời gian tăng 0,05 s. RMSE thấp hơn Constrained ở cả năm cặp. So với Savitzky–Golay, PSTMO cũng giảm cả thời gian và RMSE ở năm cặp. So với Simple, PSTMO chỉ nhanh hơn ở hai cặp nhưng có RMSE thấp hơn ở bốn cặp. Trung bình toàn tuyến cần được đọc cùng phân bố theo bộ lập kế hoạch.

Đường gốc với Theta* có sai số vị trí cuối 0,100234 m, vượt ngưỡng 0,10 m khoảng 0,234 mm. Lượt này được phân loại không đạt theo cùng tiêu chí với PSTMO tại R03. Mức vượt nhỏ mô tả độ gần ngưỡng của kết quả, chưa chứng minh một khác biệt cơ học đáng kể. Tỷ lệ thành công của ma trận vẫn là 123/125.

Hai trường hợp sát ngưỡng ở R03 và R04 cho thấy cần báo sai số liên tục bên cạnh tỷ lệ hoàn thành. Đường tham chiếu bảo toàn vị trí đích, nhưng robot còn chịu ảnh hưởng của bám đường, tiếp cận và dừng. Dữ liệu hiện tại xác định được kết quả cuối; chưa đủ để quy mức vượt ngưỡng cho một thành phần điều khiển riêng.

## F. R05 Rẽ từ hành lang ngang vào hành lang dọc

### 1) Nhiệm vụ ngắn với chuyển hướng tập trung

R05 bắt đầu tại (−5,0; 0,0) m và kết thúc tại (0,0; 3,0) m, với hướng đích π/2. Tuyến tập trung chuyển hướng từ hành lang ngang sang hành lang dọc. Thời gian nhiệm vụ phụ thuộc cả đoạn rẽ lẫn quá trình giảm tốc gần đích. Đây là trường hợp bổ sung cho tuyến chéo dài R02.

Hình 13 thể hiện năm đường tham chiếu và vùng khác biệt hình học theo quy tắc chọn vùng tại R01. Bảng XIII tổng hợp kết quả trên năm bộ lập kế hoạch. Toàn bộ 25 lượt khảo sát đều đạt, tạo tập so sánh đầy đủ ở R05. Kết quả kiểm chứng lặp của cùng tuyến được trình bày riêng ở Mục III-H.

@table r05 Kết quả mô tả R05 trên năm bộ lập kế hoạch

@note Cả 25 lượt của ma trận khảo sát R05 đều đạt. Eκ,ref và c là chỉ số đường tham chiếu; T và e là chỉ số thực thi. Kết quả này không được thay thế cho tỷ lệ thành công của bộ thử nghiệm lặp.

### 2) So sánh thời gian và chất lượng đường

PSTMO đạt thời gian trung bình 37,69 s. Các giá trị tương ứng của đường gốc, Savitzky–Golay và Constrained là 40,27 s, 40,02 s và 39,33 s. Simple đạt 38,25 s, gần PSTMO nhất về thời gian. RMSE của PSTMO là 20,8 mm; Constrained và đường gốc lần lượt có giá trị 24,4 mm và 21,2 mm. Mức chênh lệch về thời gian và RMSE phụ thuộc phương án đối chứng.

@figure r05_paths R05 với Theta*: rẽ từ hành lang ngang vào hành lang dọc. Hình sử dụng ma trận khảo sát một lượt; các thất bại của bộ thử nghiệm lặp được báo riêng ở Mục III-H.

Tích phân bình phương độ cong của PSTMO là 2,44 m⁻¹, so với Constrained 25,35 m⁻¹ và đường gốc 141,50 m⁻¹. Tỷ lệ giảm Eκ,ref lớn hơn tỷ lệ giảm thời gian thực thi. Robot thực thi qua bộ điều khiển và các tầng giới hạn vận tốc, nên tổng thời gian không chỉ phụ thuộc chỉ số độ cong. Hình 14 thể hiện sự khác biệt giữa các bộ lập kế hoạch.

@table r05_paired Chênh lệch PSTMO trừ Constrained ở R05

@figure r05_metrics R05: kết quả một lượt cho từng tổ hợp bộ lập kế hoạch–phương án. Tất cả các lượt trong hình đều đạt, nhưng điều đó chưa mô tả tính lặp lại của nhiệm vụ.

### 3) Khoảng hở và chuyển tiếp sang kiểm chứng lặp

Bảng XIV cho thấy PSTMO nhanh hơn Constrained ở bốn cặp, nhưng với Theta* tăng 0,01 s và tăng RMSE 0,33 mm. So với Simple, PSTMO giảm thời gian ở ba cặp và giảm RMSE ở cả năm cặp. So với đường gốc và Savitzky–Golay, số cặp nhanh hơn lần lượt là bốn và năm. Các chênh lệch rất nhỏ là kết quả mô tả của một lượt, chưa xác định tính ổn định khi lặp.

Khoảng hở nhỏ nhất trung bình của PSTMO là 20,8 cm, so với 22,2 cm của Constrained; chênh lệch nhỏ hơn tại R01. Thời lượng hậu xử lý tương ứng là khoảng 0,077 s và 0,022 s. Trên R05, PSTMO giảm thời gian thực thi và RMSE trung bình so với Constrained nhưng có thời lượng hậu xử lý dài hơn. Các chỉ số này được diễn giải theo phạm vi đo tại Mục III-A.

Việc hoàn thành 25 lượt khảo sát chưa xác định tỷ lệ thành công khi chạy lặp lại. Bộ kiểm chứng tiếp theo ghi nhận một số lượt kết thúc trước đích ngay trên R05. Đối với nhiệm vụ ngắn, chi phí phục hồi một lượt thất bại có thể ảnh hưởng đáng kể thời gian chu kỳ. Nghiên cứu chưa đo chi phí này, nên thời gian trung bình trên các lượt đạt cần được báo cùng tỷ lệ hoàn thành.

## G. Tổng hợp theo tuyến và bộ lập kế hoạch

### 1) Kết quả chung của ma trận năm phương án

Ma trận khảo sát ghi nhận 123/125 lượt đạt. Đường gốc và PSTMO cùng đạt 24/25; Simple, Savitzky–Golay và Constrained đạt 25/25. Hai lượt không đạt đều vượt dung sai vị trí sau khi tác vụ điều hướng hoàn tất. Bộ giám sát va chạm không ghi nhận can thiệp, và phép kiểm tra đường tham chiếu không phát hiện mẫu đường bao giao cắt vật cản trên đường tham chiếu.

@table pooled Tổng hợp mô tả ma trận 125 lượt

@note T và e dùng các lượt đạt của từng phương án, nên đường gốc/PSTMO có 24 lượt còn các phương án khác có 25. Eκ,ref dùng đủ 25 đường mỗi phương án. Không dùng trực tiếp hiệu hai trung bình khác mẫu để thay thế chênh lệch ghép cặp.

Bảng XV cho thấy PSTMO có thời gian trung bình thấp nhất trong tập lượt đạt của từng phương án. Tuy nhiên, PSTMO và đường gốc dùng 24 lượt, trong khi ba đối chứng còn lại dùng 25 lượt. Các trung bình này mô tả kết quả từng phương án; chênh lệch ghép cặp phải dùng các lượt cùng đạt. Simple là đối chứng gần PSTMO nhất về thời gian trung bình, nên cần được giữ trong đánh giá cùng Constrained.

@figure matrix_delta Chênh lệch PSTMO − Constrained trong ma trận khảo sát. Mỗi ô là một cặp cùng tuyến và bộ lập kế hoạch; màu xanh biểu thị chênh lệch âm. Dấu gạch là cặp không cùng đạt, không phải hiệu bằng không. Các số là một lượt, không kèm suy luận thống kê.

RMSE thay đổi khác nhau giữa các tuyến. PSTMO có RMSE trung bình thấp hơn các đối chứng ở R04 nhưng cao hơn các đối chứng tại R02. Hình 15 trình bày chênh lệch PSTMO trừ Constrained cho từng tổ hợp tuyến–bộ lập kế hoạch. Biểu đồ cho thấy các cặp có thời gian giảm nhưng RMSE tăng. Những cặp không cùng đạt được để trống, bảo toàn điều kiện so sánh.

### 2) Quan hệ giữa hình học và kết quả thực thi

Tích phân bình phương độ cong của đường tham chiếu PSTMO thấp hơn các đối chứng ở mức trung bình từng tuyến. Tuy nhiên, chỉ số rời rạc còn phụ thuộc vị trí và mật độ điểm. Các bộ ba điểm trên đường lưới có đoạn nối rất ngắn có thể tạo giá trị độ cong ước lượng lớn. Vì vậy, mức chênh lệch phải được đọc theo phép tính tại Mục III-A và đối chiếu với các chỉ số thực thi.

Các kết quả R02 và R03 xác định hai giới hạn khác nhau của đánh giá hình học. Tại R02, Eκ,ref thấp hơn đi cùng RMSE cao hơn. Tại R03, một đường PSTMO vẫn dẫn tới lượt không đạt dung sai đích. Do đó, chất lượng đường tham chiếu, độ bám đường và hoàn thành nhiệm vụ cần được báo đồng thời. Chỉ số hình học riêng lẻ chưa đại diện đầy đủ cho hiệu năng điều hướng.

Hình 16 đối chiếu đường tham chiếu với quỹ đạo thực thi tại R02 khi sử dụng Theta*. Đồ thị phía dưới biểu diễn vận tốc góc của cùng hai lượt, giữ nguyên tín hiệu đã ghi. Sự khác biệt giữa đường tham chiếu và chuyển động thực thi cho thấy vai trò của phản hồi điều khiển. Để xác định nguyên nhân từng dao động, cần phân tích đồng bộ lệnh điều khiển và trạng thái định vị.

@figure execution_trace R02/Theta* trong ma trận khảo sát: trên là đường tham chiếu (nét đứt) và quỹ đạo thực thi trong mô phỏng (nét liền) tại vùng rẽ; dưới là vận tốc góc thực thi theo thời gian tương đối. Mỗi phương án có mốc thời gian riêng; không căn chỉnh các đỉnh tín hiệu sau khi quan sát kết quả.

Trong cấu hình khảo sát, RPP không sử dụng chức năng điều tiết vận tốc theo độ cong hoặc chi phí. Vì vậy, không thể giải thích trực tiếp mức giảm thời gian bằng cơ chế điều tiết độ cong của cấu hình RPP mặc định. Hình dạng đường vẫn có thể ảnh hưởng lệnh quay, yêu cầu xoay hướng và tiếp cận đích. Những thành phần này chưa được phân rã thời lượng riêng trong dữ liệu tổng hợp.

### 3) Tính công bằng của đối chứng và phạm vi kết luận

Ghép cặp theo cùng đường gốc giúp so sánh các phương án trên đầu vào hình học tương ứng. Cùng tọa độ đầu–cuối chưa đủ cho điều kiện này, vì bộ lập kế hoạch có thể tạo đường khác nhau giữa các lượt. Bản đồ và bộ điều khiển được giữ chung để đánh giá tác động của chuỗi hậu xử lý trong cùng hệ điều hướng.

Constrained được chọn cho kiểm chứng lặp vì sử dụng tối ưu số và có thể triển khai trong cùng hệ Nav2. Cơ chế này tạo đối chứng khác với lựa chọn trạng thái góc của PSTMO. Ma trận 125 lượt vẫn giữ Simple và Savitzky–Golay để đánh giá phạm vi so sánh. Do hai phương án này chưa được kiểm chứng lặp tương ứng, kết luận thống kê ở phần sau chỉ áp dụng cho PSTMO và Constrained.

So sánh sử dụng cấu hình Constrained không bật thành phần độ cong, với các thông số đã nêu trong Bảng IV. Kết quả chưa đại diện cho hiệu năng tốt nhất sau tinh chỉnh của mỗi phương pháp. Đánh giá rộng hơn cần cùng giới hạn số lần đánh giá tham số và tập tuyến kiểm tra tách biệt. Điều này đặc biệt cần thiết khi suy rộng từ năm tuyến hiện có sang các bố trí vật cản khác.

## H. Kiểm chứng lặp PSTMO và Constrained

### 1) Ghép cặp và đơn vị phân tích

Bộ kiểm chứng gồm 250 lượt, tạo 125 cặp cùng tuyến, bộ lập kế hoạch và số lần lặp. Đường gốc trùng khớp ở toàn bộ cặp. Constrained đạt 122/125 lượt và PSTMO đạt 121/125, tổng cộng 243/250. Có 118 cặp có cả hai phương án đạt được dùng cho các chỉ số liên tục. Bảy cặp còn lại được giữ trong phân tích tỷ lệ thành công và bảng các lượt không đạt.

Mỗi tổ hợp tuyến–bộ lập kế hoạch tạo một nhóm phân tích b. Gọi Rb là tập lần lặp mà hai phương án cùng đạt trong nhóm. Chênh lệch db là trung bình các hiệu PSTMO trừ Constrained trong tập này. Công thức 9 tính chênh lệch chung Δ bằng trung bình đều của 25 nhóm. Vì số cặp có cả hai phương án đạt có thể khác nhau, Δ không nhất thiết bằng hiệu hai trung bình gộp của 118 cặp.

@equation d_b=\frac{1}{\left|R_b\right|}\sum_{r\in R_b}(y_{P,br}-y_{C,br}), || \Delta=\frac{1}{25}\sum_{b=1}^{25}d_b.

Phân tích theo nhóm giữ mỗi đầu vào hình học ở một đơn vị thống kê. Năm lần chạy lặp đo biến thiên thực thi của đầu vào đó. Các nhóm vẫn chia sẻ tuyến, bộ lập kế hoạch và bản đồ, nên chưa độc lập hoàn toàn. Việc lấy mẫu lại các nhóm được dùng như một phân tích có điều kiện trên thiết kế hiện tại. Giới hạn suy rộng được trình bày tại Mục III-I.

### 2) Phương pháp thống kê và ba chỉ số chính

Ba chỉ số chính gồm thời gian thực thi, RMSE bám đường và tích phân bình phương độ cong của quỹ đạo thực thi. Khoảng tin cậy 95% của Δ được ước lượng bằng bootstrap trên 25 nhóm với 20.000 lần lấy mẫu lại. Giá trị khởi tạo của bộ sinh số giả ngẫu nhiên dùng trong phân tích được cố định cho từng chỉ số. Khoảng tin cậy lấy theo các phân vị của phân bố trung bình lấy mẫu lại [5].

Kiểm định hạng có dấu Wilcoxon hai phía được áp dụng cho chênh lệch trung bình trong từng nhóm [6]. Các hiệu có trị tuyệt đối không quá 10⁻¹² được loại theo quy tắc số học của phân tích. Ba giá trị p được hiệu chỉnh bằng thủ tục Holm [7]. Kiểm định đánh giá xu hướng chênh lệch theo giả định của thủ tục; bootstrap ước lượng bất định của trung bình. Giá trị p được báo cùng độ lớn chênh lệch và khoảng tin cậy.

@table means Trung bình trên các cặp hoàn thành và số nhóm có chỉ số giảm

@note Hai cột phương án là trung bình gộp 118 cặp có cả hai phương án đạt. “Số nhóm giảm” là số tổ hợp tuyến–bộ lập kế hoạch có db < 0, tức chỉ số trung bình của PSTMO thấp hơn Constrained. Trung bình gộp không phải cơ sở số học trực tiếp để tái tính Δ trong Bảng XVII khi số cặp mỗi nhóm khác nhau.

@table primary Chênh lệch trung bình giữa PSTMO và Constrained và khoảng tin cậy

@note Δ = PSTMO − Constrained, trung bình đều trên 25 nhóm. Khoảng tin cậy dùng bootstrap nhóm; p Holm hiệu chỉnh đúng ba chỉ số chính. Các phép suy luận có điều kiện trên các cặp có cả hai phương án đạt và tập tuyến–bộ lập kế hoạch đã thử.

Bảng XVII cho thấy PSTMO giảm thời gian trung bình theo nhóm 2,857 s. Khoảng tin cậy 95% là [−3,716; −2,039] s, với p hiệu chỉnh Holm bằng 1,19 × 10⁻⁶. Thời gian giảm ở 24/25 nhóm. Như vậy, lợi ích thời gian xuất hiện trên phần lớn tổ hợp tuyến–bộ lập kế hoạch trong bộ thử nghiệm lặp.

RMSE giảm trung bình 2,985 mm, với khoảng tin cậy [−4,487; −1,506] mm và p hiệu chỉnh bằng 4,89 × 10⁻⁴. Tích phân bình phương độ cong của quỹ đạo thực thi giảm 1,038 m⁻¹; khoảng tin cậy là [−1,332; −0,767] m⁻¹, với p hiệu chỉnh bằng 5,36 × 10⁻⁷. Số nhóm giảm tương ứng là 19/25 và 24/25. Các kết quả này áp dụng cho tập cặp hoàn thành.

Hình 17 cho thấy mức thay đổi không đồng đều giữa 25 nhóm. RMSE có nhiều trường hợp tăng hơn thời gian và Eκ,exec. Điều này phù hợp với các đánh đổi bám đường trong ma trận khảo sát. Dù cả ba chỉ số đều giảm ở mức trung bình, PSTMO không giảm đồng thời cả ba trên mọi đầu vào.

@table by_planner Chênh lệch mô tả theo từng bộ lập kế hoạch trong bộ thử nghiệm lặp

@note Mỗi hàng là trung bình đều của năm chênh lệch trung bình nhóm thuộc cùng bộ lập kế hoạch, không phải một kiểm định mới. ΔEκ,exec có đơn vị m⁻¹. Không sử dụng bảng này để suy ra thứ hạng bộ lập kế hoạch trên các bài toán ngoài năm tuyến đã thử.

Bảng XVIII cho thấy thời gian giảm trung bình với cả năm bộ lập kế hoạch. Mức giảm lớn nhất là 4,658 s với Smac 2D, nhỏ nhất là 0,891 s với Smac Hybrid. Với RMSE, mức giảm tương ứng là 1,561 và 5,148 mm. Bộ lập kế hoạch có mức giảm thời gian lớn nhất không đồng thời có mức giảm RMSE lớn nhất. Mỗi hàng tổng hợp năm nhóm phân tích tương ứng với năm tuyến, cần được đối chiếu với phân bố trong Hình 17.

@figure repeat_blocks Chênh lệch trung bình của 25 nhóm trong bộ thử nghiệm lặp. Trong mỗi nhóm tuyến, thứ tự là NavFn A*, NavFn Dijkstra, Theta*, Smac 2D và Smac Hybrid. Mỗi điểm là trung bình chênh lệch trên các lần lặp cùng đạt; đường đứt là trung bình đều của 25 điểm.

### 3) Các chỉ số bổ sung và đánh đổi

@table secondary Chênh lệch các chỉ số bổ sung giữa PSTMO và Constrained

@note Các khoảng tin cậy được báo như phân tích bổ sung; hiệu chỉnh Holm ở Bảng XVII chỉ áp dụng cho ba chỉ số chính, không áp dụng cho các chỉ số trong bảng này. Dấu âm thuận lợi cho tích phân bình phương độ cong, độ giật, quãng đường và thời gian xử lý, nhưng bất lợi cho khoảng hở.

Bảng XIX ghi nhận tích phân bình phương độ cong của đường tham chiếu giảm 23,202 m⁻¹ và quãng đường thực thi giảm 0,089 m. Phân vị 95% của trị tuyệt đối độ giật tịnh tiến giảm 0,078 m/s³. Những thay đổi này bổ sung mô tả về hình dạng đường và chuyển động. Chúng chưa xác định mức tiết kiệm điện năng hoặc mức giảm rung của tải hàng, vì nghiên cứu chưa có các phép đo tương ứng.

@figure tradeoffs Đánh đổi trong bộ thử nghiệm lặp, mỗi điểm là một nhóm tuyến–bộ lập kế hoạch. Trên: chênh lệch Eκ,ref và khoảng hở; dưới: chênh lệch sai số đích và thời gian thực thi. Mọi chênh lệch đều là PSTMO − Constrained.

Lợi ích trên đi kèm ba đánh đổi. Sai số vị trí cuối tăng trung bình 3,380 mm, khoảng hở nhỏ nhất giảm 35,161 mm và thời lượng hậu xử lý tăng 71,220 ms. Khoảng hở giảm ở 18 nhóm, tăng ở năm nhóm và bằng nhau ở hai nhóm. Sai số đích tăng ở 18/25 nhóm; thời lượng hậu xử lý tăng ở cả 25 nhóm. Vì vậy, lợi ích về thời gian và RMSE cần được cân nhắc cùng sai số đích, khoảng hở và thời lượng hậu xử lý.

Hình 18 đối chiếu các đại lượng ngay trong từng nhóm. Một số tổ hợp giảm thời gian nhưng tăng sai số đích; giảm tích phân bình phương độ cong cũng có thể đi cùng giảm khoảng hở. Việc chấp nhận đánh đổi phụ thuộc dung sai đích và biên an toàn của ứng dụng. Nghiên cứu báo riêng các đại lượng, chưa xây dựng hệ số quy đổi thành một điểm hiệu năng chung.

### 4) Thành công nhiệm vụ và các lượt không đạt

Trong 125 cặp, bốn cặp chỉ Constrained đạt và ba cặp chỉ PSTMO đạt. Kiểm định McNemar chính xác hai phía cho p = 1,0. Với kích thước mẫu này, chưa có bằng chứng về khác biệt tỷ lệ thành công. Kết quả cũng chưa xác nhận tương đương hoặc không kém hơn, vì nghiên cứu chưa đặt biên so sánh và thiết kế cỡ mẫu cho các mục tiêu đó.

@table failures Bảy lượt không đạt của bộ kiểm chứng lặp

@note P: PSTMO; C: Constrained. “Vượt ngưỡng vị trí” nghĩa là tác vụ điều hướng hoàn thành nhưng sai số vị trí cuối vượt 0,10 m. “Kết thúc trước đích” nghĩa là lượt chạy kết thúc khi chưa đến đích, không đồng nghĩa robot đã dừng hẳn.

Bảng XX phân biệt năm lượt kết thúc trước đích ở R05 với hai lượt vượt dung sai tại R04. Nhóm R05 gồm ba lượt PSTMO và hai lượt Constrained. Sai số vị trí còn lại từ khoảng 0,686 m đến 3,655 m, lớn hơn rõ rệt ngưỡng 0,10 m. Hai lượt R04 có sai số lần lượt 0,101583 m và 0,100358 m, chỉ vượt ngưỡng một lượng nhỏ.

Các trường hợp kết thúc xa đích và các trường hợp sát ngưỡng cần được phân tích riêng. Nhóm đầu phản ánh nhiệm vụ chưa hoàn thành quãng đường yêu cầu; nhóm sau hoàn tất tác vụ nhưng không đạt tiêu chí sai số vị trí cuối. Trạng thái kết thúc trong bảng mô tả hiện tượng đã ghi, chưa xác định nguyên nhân vật lý của từng thất bại.

@figure endpoints Sai số vị trí cuối của toàn bộ 250 lượt, bao gồm không đạt. Trục đứng dùng thang log để đồng thời hiển thị sai số cỡ centimét và các lượt kết thúc trước đích. Ngưỡng vị trí 0,10 m chỉ là một phần của tiêu chí hoàn thành.

Hình 19 giữ toàn bộ 250 lượt khi biểu diễn sai số cuối. Phần lớn điểm nằm dưới ngưỡng vị trí, nhưng năm lượt tại R05 kết thúc khi còn cách đích đáng kể. Thời gian trung bình của 118 cặp có cả hai phương án đạt không bao gồm chi phí phục hồi các lượt này. Đánh giá thời gian chu kỳ trong ứng dụng cần một chính sách phục hồi chung và phép đo toàn nhiệm vụ.

## I. Thảo luận đóng góp và giới hạn áp dụng

### 1) Bằng chứng cho cơ chế hậu xử lý

PSTMO kết hợp chuyển tiếp Bézier bậc năm với quay tại chỗ, kiểm tra giới hạn chuyển động và lựa chọn chuỗi tương thích. Các kết quả cho thấy phương pháp tạo đường tham chiếu có tích phân bình phương độ cong thấp trên nhiều đầu vào. Bộ kiểm chứng lặp còn ghi nhận giảm trung bình thời gian, RMSE và Eκ,exec so với Constrained. Đây là bằng chứng ở cấp độ đường đi và chuyển động robot trong cấu hình khảo sát.

Hồ sơ chẩn đoán xác nhận cả 125 lượt PSTMO sử dụng tiền xử lý không bật bước rút gọn riêng theo đường nhìn và tìm kiếm phân cấp với hai chiều dài cắt. Tổng cộng 2.005 trạng thái quy hoạch động và 3.165 cạnh tương thích được ghi nhận. Các kiểm tra đầu ra cũng được thực hiện. Những số liệu này xác nhận thuật toán đã chạy theo cấu trúc mô tả; đóng góp riêng của từng thành phần cần thêm thí nghiệm loại bỏ có kiểm soát.

Hiệu quả thực thi phụ thuộc tương tác giữa đường tham chiếu và bộ điều khiển. Nghiên cứu hiện dùng một cấu hình RPP với điều tiết vận tốc theo độ cong và chi phí được tắt. Đổi bộ điều khiển hoặc bật các chức năng này có thể thay đổi độ lớn chênh lệch và thứ hạng phương án. Vì vậy, kết quả cần được kiểm chứng lại khi thay đổi cấu hình điều khiển.

### 2) Tính trơn hình học và an toàn là hai yêu cầu riêng

Tính liên tục G² tại mối nối hỗ trợ giảm biến thiên độ cong của đường tham chiếu. Quỹ đạo thực thi còn chịu sai số bám đường và động lực học mô phỏng. Các thao tác quay tại chỗ cũng được xử lý riêng trong PSTMO. Phạm vi bảo đảm hình học vì vậy cần được giữ như đã xác định ở Phần II khi diễn giải kết quả thực thi.

Phép kiểm tra va chạm xét khả năng di chuyển dọc đường tham chiếu mà không giao cắt với vật cản, có tính đến kích thước và hướng robot. Đường có tích phân bình phương độ cong nhỏ vẫn có thể đưa một góc thân xe tới gần vật cản hơn. Khoảng hở giảm trong bộ kiểm chứng cho thấy đây là đánh đổi thực tế của cấu hình khảo sát. Ứng dụng đòi hỏi biên an toàn lớn cần kiểm tra lại tham số cùng sai số mô hình và định vị.

Hệ thống không ghi nhận can thiệp của bộ giám sát hoặc mẫu đường bao va chạm trên đường tham chiếu. Kết quả này phản ánh các phép kiểm tra đang sử dụng, chưa thay thế đo tiếp xúc trên robot thực. Với bản đồ 0,05 m/ô, chênh lệch khoảng hở cỡ milimét còn phụ thuộc nội suy và lượng tử hóa. Các chữ số báo cáo phục vụ tính toán, không biểu thị độ chính xác vật lý tương ứng.

### 3) Độ tin cậy nội tại của thiết kế thực nghiệm

Thiết kế giữ cùng đường gốc, bản đồ và bộ điều khiển trong mỗi phép so sánh. Các lượt không đạt được giữ trong mẫu số; thử lại do lỗi hạ tầng được báo riêng. Hình và bảng sử dụng dữ liệu có hồ sơ nguồn để đối chiếu. Những biện pháp này hạn chế thay đổi đầu vào và lựa chọn kết quả thuận lợi.

Ma trận khảo sát có R01 được thu ở đợt khác R02–R05. Vì vậy, bộ này chủ yếu mô tả kết quả theo kịch bản. Bộ kiểm chứng lặp dùng cùng phiên bản và cấu hình cho PSTMO–Constrained, tạo cơ sở so sánh nhất quán hơn. Đường gốc, Simple và Savitzky–Golay chưa có bộ thử nghiệm lặp tương ứng để mở rộng kết luận thống kê.

Hồ sơ chưa xác nhận thứ tự thử nghiệm được ngẫu nhiên hóa. Bốn tiến trình có miền truyền thông riêng nhưng dùng chung tài nguyên tính toán. Tải máy và nhịp đồng hồ mô phỏng có thể ảnh hưởng thời lượng hậu xử lý. Chỉ số này được dùng bổ sung; đánh giá độ trễ tính toán trên phần cứng cần phép đo thời gian thực riêng.

### 4) Giới hạn của suy luận thống kê

Năm lần lặp đo biến thiên thực thi với cùng đầu vào. Thiết kế chưa khảo sát nhiều mức nhiễu, ma sát hoặc giá trị khởi tạo của bộ sinh số giả ngẫu nhiên. Các nhóm còn chia sẻ tuyến và bộ lập kế hoạch, nên việc lấy trung bình theo nhóm chưa loại hết phụ thuộc chéo. Khoảng tin cậy và kiểm định được diễn giải trong phạm vi tập cấu hình đã thử.

Chọn các cặp có cả hai phương án đạt giúp so sánh đúng thời gian hoàn thành. Tuy nhiên, kết quả có thể thiên lệch về các nhiệm vụ dễ hơn nếu khả năng thất bại liên quan độ khó của đường. Báo đồng thời tỷ lệ thành công và từng lượt không đạt giúp làm rõ giới hạn này. Chênh lệch của các chỉ số liên tục vẫn là kết quả có điều kiện trên thành công.

Ba chỉ số chính và hiệu chỉnh Holm được xác định trong quy trình phân tích. Hồ sơ chưa xác nhận kế hoạch đã đăng ký trước khi xem dữ liệu. Các chỉ số phụ vì vậy được trình bày như phân tích bổ sung. Giá trị p và khoảng tin cậy hỗ trợ đánh giá bất định; chưa xác định quan hệ nhân quả giữa từng thành phần thuật toán và kết quả chuyển động.

### 5) Phạm vi triển khai và hướng kiểm chứng tiếp theo

Trong cấu hình khảo sát, PSTMO giảm thời gian và sai số bám đường trung bình trên các cặp hoàn thành so với Constrained. Việc triển khai cần đồng thời đáp ứng khoảng hở tối thiểu, dung sai đích và yêu cầu về chi phí phục hồi. Các tiêu chí này phải được xác định theo nhiệm vụ vận hành. Thứ hạng trung bình hiện có chưa đủ để quyết định cho mọi ứng dụng.

Các bước kiểm chứng tiếp theo gồm robot thực, vật cản động, nhiều bố trí bản đồ và thay đổi tải hoặc ma sát. Cần bổ sung thử nghiệm với cấu hình điều khiển khác, nhiều điều kiện nhiễu và lặp đầy đủ các đối chứng. Thí nghiệm loại bỏ tiền xử lý, tìm kiếm phân cấp, hai chiều dài cắt, quy hoạch động và kiểm tra đầu ra sẽ làm rõ đóng góp riêng. Việc tinh chỉnh tham số cần dùng cùng giới hạn số lần đánh giá tham số và tập đánh giá tách biệt.

Kết quả hiện tại hỗ trợ lợi ích trung bình của PSTMO về thời gian và hai chỉ số chuyển động trên các cặp hoàn thành. Chưa có bằng chứng về ưu thế tỷ lệ thành công. Khoảng hở nhỏ hơn, sai số đích lớn hơn và thời lượng hậu xử lý cao hơn là các đánh đổi đã được đo. Những yếu tố này xác định phạm vi sử dụng và các ưu tiên kiểm chứng tiếp theo.

# TÀI LIỆU THAM KHẢO CHO PHẦN II VÀ III

[1] S. Macenski, S. Singh, F. Martín, and J. Ginés, “Regulated pure pursuit for robot path tracking,” Autonomous Robots, vol. 47, pp. 685–694, 2023, doi: 10.1007/s10514-023-10097-6.

[2] A. Savitzky and M. J. E. Golay, “Smoothing and differentiation of data by simplified least squares procedures,” Analytical Chemistry, vol. 36, no. 8, pp. 1627–1639, 1964, doi: 10.1021/ac60214a047.

[3] Nav2, “Constrained Smoother,” Nav2 Jazzy API documentation. [Online]. Available: https://api.nav2.org/nav2-jazzy/html/md_nav2_constrained_smoother_README.html. Accessed: Oct. 13, 2026.

[4] Nav2, “Smoother server implementation,” navigation2, branch jazzy. [Online]. Available: https://github.com/ros-navigation/navigation2. Accessed: Oct. 13, 2026.

[5] B. Efron, “Bootstrap methods: Another look at the jackknife,” The Annals of Statistics, vol. 7, no. 1, pp. 1–26, 1979, doi: 10.1214/aos/1176344552.

[6] F. Wilcoxon, “Individual comparisons by ranking methods,” Biometrics Bulletin, vol. 1, no. 6, pp. 80–83, 1945, doi: 10.2307/3001968.

[7] S. Holm, “A simple sequentially rejective multiple test procedure,” Scandinavian Journal of Statistics, vol. 6, no. 2, pp. 65–70, 1979.
