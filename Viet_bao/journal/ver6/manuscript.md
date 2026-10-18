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

Chiều dài L được tính bằng tổng khoảng cách giữa các điểm đường tham chiếu; D là tổng quãng đường từ các vị trí robot trước bước xử lý quỹ đạo. Sai số cực đại emax dùng cùng phép đo khoảng cách như RMSE. Sai số cuối ep và eψ lần lượt là khoảng cách tới vị trí đích và trị tuyệt đối sai lệch hướng đã quy về [0; π]. Các chỉ số hình học được báo trên toàn bộ 25 đường của mỗi phương án; các trung bình thực thi dùng tập lượt đạt và ghi rõ mẫu số.

Độ cong cực đại κmax là cực đại trị tuyệt đối độ cong ước lượng trên các điểm đầu ra chưa lấy mẫu lại. Để kiểm tra ảnh hưởng lấy mẫu, phần tịnh tiến được tách tại các tư thế quay tại chỗ và lấy mẫu lại với bước 0,05 m, cho các chỉ số κt,max và Eκ,t. Dấu quay tại chỗ được nhận diện khi dịch chuyển không quá 0,0001 m và thay đổi hướng ít nhất 5°. Đây là dấu trong đường tham chiếu, không phải số lần robot thực sự xoay trong quá trình điều khiển.

Toàn bộ 125 bản ghi có đủ chỉ số quỹ đạo thực thi, độ giật và chuỗi trạng thái. Các chỉ số bổ sung được trích cùng nguồn với thời gian và RMSE. Vận tốc bánh tương đương được suy từ v ± bω/2 với b = 0,2548 m; gia tốc ngang được tính bằng |vω|. Gia tốc và độ giật dùng sai phân với bước thời gian hợp lệ từ 0,0001 đến 0,5 s. Cực đại của các tín hiệu chỉ mô tả những mẫu đã ghi, chưa chứng minh ràng buộc liên tục giữa các mẫu.

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

@table r01_extra Chiều dài, độ cong cực đại và chỉ số bổ sung tại R01

@note L: chiều dài đường tham chiếu; κmax: độ cong tuyệt đối cực đại chưa lấy mẫu lại; D: quãng đường thực thi; emax: sai số bám đường cực đại; τ: thời lượng hậu xử lý. L, κmax và τ dùng năm lượt; D và emax chỉ dùng lượt đạt.

@figure r01_metrics R01: thời gian thực thi và RMSE của đủ năm phương án trên từng bộ lập kế hoạch. Các cột là một lượt thực thi, không phải trung bình nhiều lần lặp; không vẽ thanh sai số.

Theo thứ tự đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO, Eκ,exec lần lượt là 2,659; 2,208; 2,517; 2,673; 1,613 m⁻¹; phân vị 95% của trị tuyệt đối độ giật lần lượt là 2,521; 2,428; 2,404; 2,471; 2,485 m/s³. PSTMO giảm sai số cực đại cùng với RMSE, nhưng độ giật không thấp nhất. Vì vậy, lợi ích về hình dạng quỹ đạo không đồng nghĩa giảm đồng thời mọi biến thiên theo thời gian.

### 3) Đánh đổi khoảng hở và diễn giải

Trong năm cặp theo bộ lập kế hoạch, PSTMO nhanh hơn đường gốc, Savitzky–Golay và Constrained ở cả năm cặp; nhanh hơn Simple ở bốn cặp. Tuy nhiên, so sánh ghép cặp cho thấy RMSE tăng so với Constrained khi dùng Theta* và Smac Hybrid, lần lượt 1,87 và 0,97 mm. Như vậy, lợi thế thời gian nhất quán hơn lợi thế bám đường ở R01.

Khoảng hở nhỏ nhất trung bình của PSTMO là 15,7 cm, so với 23,5 cm của Constrained. Đường có tích phân bình phương độ cong thấp hơn nhưng sử dụng phần hành lang gần vật cản hơn. Đây là đánh đổi hình học quan sát được; mức đóng góp của từng trọng số mục tiêu chưa được tách riêng.

Thời lượng hậu xử lý của PSTMO ở R01 khoảng 0,102 s, trong khi Constrained là 0,026 s. Mức tăng này cần được xem cùng lợi ích thời gian thực thi. Hai đại lượng có phạm vi đo riêng theo Mục III-A, nên chưa xác định đầy đủ độ trễ toàn hệ thống. Kết quả R01 cho thấy khả năng giảm thời gian nhiệm vụ đi cùng khoảng hở nhỏ hơn và thời lượng hậu xử lý dài hơn.

Sai số vị trí cuối trung bình của năm phương án theo thứ tự trên là 35,7; 36,1; 37,0; 33,6; 32,5 mm; sai số hướng tương ứng là 0,075; 0,083; 0,072; 0,076; 0,079 rad, chỉ tính các lượt đạt.

## C. R02 Di chuyển chéo dài

### 1) Tác động của chiều dài và các đoạn thẳng

R02 nối (−4,7; −2,8) m với (4,7; 2,8) m qua hành lang ngang. So với R01, tuyến trải trên phạm vi bản đồ rộng hơn và gồm các đoạn gần thẳng dài hơn. Tuyến này khảo sát hiệu quả hậu xử lý khi vùng chuyển hướng chỉ chiếm một phần đường đi. Vì thời gian còn phụ thuộc các đoạn thẳng và tiếp cận đích, mức giảm Eκ,ref cần được đối chiếu với thời gian toàn nhiệm vụ.

Hình 7 cho thấy các phương án đi qua cùng hệ hành lang và khác nhau chủ yếu ở vùng chuyển hướng. Trong hình toàn tuyến, nhiều đường gần như chồng lên nhau. Vùng phóng to thể hiện độ lệch cục bộ theo đơn vị mét, cùng vị trí vật cản xung quanh. Bảng VII tổng hợp kết quả của cả năm bộ lập kế hoạch.

@table r02 Kết quả mô tả R02 trên năm bộ lập kế hoạch

@note T và e lấy trên lượt đạt; Eκ,ref và c lấy trên đường tham chiếu. Cả 25 lượt R02 đều đạt. Các giá trị c là trung bình của cực tiểu từng đường.

@figure r02_paths R02 với Theta*: đường tham chiếu toàn tuyến và vùng phóng to theo cùng quy tắc chọn vùng như R01. Trục tọa độ có đơn vị mét; tỷ lệ hiển thị toàn tuyến và vùng phóng to khác nhau.

### 2) Hiệu quả thời gian không đồng nghĩa cải thiện mọi chỉ số

PSTMO đạt thời gian trung bình 55,39 s, thấp nhất trong năm phương án ở R02. Đường gốc đạt 59,57 s, Savitzky–Golay 58,37 s và Constrained 59,06 s. Simple đạt 56,40 s, gần PSTMO hơn các đối chứng còn lại. Vì vậy, phần lợi ích bổ sung so với Simple nhỏ hơn lợi ích so với đường chưa xử lý.

Tuy nhiên, RMSE trung bình của PSTMO là 24,2 mm, trong khi đường gốc là 21,3 mm; ba đối chứng làm mượt cũng có RMSE thấp hơn PSTMO. Simple, Savitzky–Golay và Constrained lần lượt đạt 21,1; 19,9 và 21,7 mm. Tích phân bình phương độ cong của đường tham chiếu PSTMO là 2,48 m⁻¹. Trường hợp này cho thấy giảm tích phân bình phương độ cong chưa bảo đảm giảm sai số bám đường dưới cấu hình RPP đã dùng.

@figure r02_metrics R02: thời gian và RMSE theo bộ lập kế hoạch. Sự khác biệt giữa hai hàng chỉ số cho phép nhận diện trường hợp nhanh hơn nhưng bám đường kém hơn; không suy ra ưu thế toàn diện từ một hàng duy nhất.

@table r02_extra Chiều dài, độ cong cực đại và chỉ số bổ sung tại R02

@note Ký hiệu như Bảng VI. L, κmax và τ dùng năm lượt; D và emax chỉ dùng lượt đạt.

So sánh theo bộ lập kế hoạch cho thấy PSTMO nhanh hơn Constrained ở bốn trong năm cặp. Mức giảm lớn nhất là 8,32 s với NavFn A*, trong khi Smac Hybrid chậm hơn 0,49 s. RMSE tăng với Theta*, Smac 2D và Smac Hybrid. Khi so với Simple, PSTMO chỉ nhanh hơn ở 2/5 cặp dù thời gian trung bình thấp hơn; RMSE cao hơn ở cả năm cặp. Như vậy, thời gian trung bình thấp hơn không đồng nghĩa nhanh hơn trên đa số đầu vào.

Theo thứ tự đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO, Eκ,exec lần lượt là 2,976; 2,020; 2,761; 2,913; 1,536 m⁻¹; phân vị 95% của trị tuyệt đối độ giật lần lượt là 2,349; 2,157; 2,294; 2,276; 2,120 m/s³. PSTMO có RMSE cao nhất ở tuyến này, trong khi sai số cực đại thấp hơn đường gốc và Constrained nhưng cao hơn Simple. Hai chỉ số sai số mô tả hai khía cạnh khác nhau: độ lệch duy trì dọc đường và độ lệch lớn nhất tại một vị trí.

### 3) Khoảng hở và ý nghĩa vận hành

Khoảng hở nhỏ nhất trung bình của PSTMO là 19,3 cm, so với 22,9 cm của Constrained. Mức giảm khoảng hở đi cùng giảm thời gian nhưng tăng RMSE. Khi lựa chọn phương án cho hành lang hẹp, cần đối chiếu cả khoảng hở và sai số bám đường. Trong thử nghiệm này, phép kiểm tra đường tham chiếu không phát hiện mẫu đường bao giao cắt vật cản.

R02 biểu hiện rõ đánh đổi giữa tốc độ hoàn thành và độ bám đường. Hình dạng tham chiếu thay đổi cùng đáp ứng của bộ điều khiển, nên dữ liệu chưa tách được đóng góp riêng của từng yếu tố. Kết quả thích hợp để lựa chọn ưu tiên theo yêu cầu nhiệm vụ. Một đánh giá chỉ dùng thời gian hoặc Eκ,ref sẽ bỏ qua RMSE cao hơn của PSTMO trên tuyến này.

Sai số vị trí cuối trung bình của năm phương án theo thứ tự trên là 37,1; 36,4; 37,5; 42,4; 37,0 mm; sai số hướng tương ứng là 0,086; 0,081; 0,083; 0,074; 0,083 rad, chỉ tính các lượt đạt.

## D. R03 Chuyển lối hình chữ U

### 1) Chuỗi góc liên tiếp và khả năng tương thích

R03 bắt đầu tại (−2,0; −2,8) m và kết thúc tại (2,0; −2,8) m, với hướng đích −π/2. Robot đi vào hệ hành lang rồi trở về hàng ngang xuất phát ở một lối khác. Các góc rẽ liên tiếp dùng chung chiều dài của những đoạn nối. Vì vậy, tăng chiều dài cắt ở một góc có thể hạn chế lựa chọn tại góc kế tiếp.

@figure r03_paths R03 với Theta*: đường chữ U và vùng chuyển hướng được phóng to. Hình biểu diễn một bộ lập kế hoạch cố định; trường hợp không đạt ở tuyến này thuộc Smac Hybrid và được báo riêng.

Hình 9 thể hiện đường chữ U và vùng chuyển hướng khi sử dụng Theta*. Đây là tình huống phù hợp để khảo sát khả năng ghép các chuyển tiếp của toàn bộ PSTMO. Các lượt hiện có đều dùng đầy đủ thuật toán. Việc xác định riêng đóng góp của quy hoạch động cần thêm thí nghiệm loại bỏ thành phần, được nêu tại Mục III-I.

@table r03 Kết quả mô tả R03 trên năm bộ lập kế hoạch

@note PSTMO có 4/5 lượt đạt; các phương án còn lại có 5/5. T và e của PSTMO chỉ lấy trên bốn lượt đạt. Eκ,ref và c vẫn lấy trên đủ năm đường; không diễn giải chênh lệch T giữa các trung bình này như một ước lượng ghép cặp.

### 2) Thời gian thực thi và trường hợp không đạt dung sai đích

Bảng IX ghi nhận PSTMO đạt 4/5 lượt, trong khi các phương án còn lại đạt 5/5. Thời gian trung bình của các lượt PSTMO đạt là 41,94 s; Simple đạt 42,34 s và Constrained 44,15 s. Các trung bình này dùng số lượt khác nhau. Vì vậy, so sánh ghép cặp với PSTMO chỉ dùng bốn đầu vào có cả hai phương án đạt; lượt Smac Hybrid được giữ trong tỷ lệ thành công. Bảng X bổ sung chỉ số hình học và thực thi của cả năm phương án.

Lượt PSTMO với Smac Hybrid hoàn tất tác vụ điều hướng nhưng có sai số vị trí cuối 0,102385 m. Giá trị này vượt ngưỡng 0,10 m khoảng 2,385 mm, nên lượt được phân loại không đạt. Hình 10 đánh dấu riêng kết quả này. Việc báo đồng thời trạng thái và sai số cho thấy mức vượt ngưỡng cụ thể, thay vì chỉ ghi một nhãn thất bại.

@figure r03_metrics R03: cột gạch và dấu × đánh dấu PSTMO/Smac Hybrid không đạt. Giá trị ghi ở cột đó chỉ mô tả trạng thái kết thúc; không được tính vào trung bình hoàn thành hoặc dùng để kết luận thời gian tốt hơn.

@table r03_extra Chiều dài, độ cong cực đại và chỉ số bổ sung tại R03

@note Ký hiệu như Bảng VI. L, κmax và τ dùng năm lượt; D và emax chỉ dùng lượt đạt.

Theo thứ tự đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO, Eκ,exec lần lượt là 4,511; 4,037; 4,577; 4,659; 4,023 m⁻¹; phân vị 95% của trị tuyệt đối độ giật lần lượt là 2,533; 2,530; 2,421; 2,418; 2,540 m/s³. Độ cong cực đại của PSTMO lớn hơn Simple dù tích phân bình phương độ cong nhỏ hơn. Độ giật cũng cao hơn các đối chứng trong tập lượt đạt. Các kết quả này không hỗ trợ kết luận PSTMO luôn tạo chuyển động trơn hơn theo mọi tiêu chí.

### 3) So sánh với Simple và giới hạn của chỉ số đường tham chiếu

Tích phân bình phương độ cong trung bình của đường tham chiếu là 5,47 m⁻¹ với PSTMO, 5,78 m⁻¹ với Simple và 22,93 m⁻¹ với Constrained. Chênh lệch hình học giữa PSTMO và Simple nhỏ hơn rõ rệt so với R01. RMSE trên các lượt đạt cũng gần nhau: 27,2 mm với PSTMO và 27,0 mm với Simple. Kết quả này cho thấy mức cải thiện của toàn bộ PSTMO thay đổi theo cấu trúc đường đầu vào; dữ liệu chưa tách riêng tác dụng của chuyển tiếp Bézier.

Trên bốn cặp có cả hai phương án đạt, PSTMO nhanh hơn đường gốc, Savitzky–Golay và Constrained ở cả bốn cặp; nhanh hơn Simple ở ba cặp. RMSE chỉ thấp hơn Simple ở một cặp, trong khi thấp hơn Constrained ở ba cặp. Ngoại lệ khi so với Constrained là Theta*, có RMSE tăng 5,36 mm. Tốc độ hoàn thành và độ bám đường tiếp tục cho thứ hạng khác nhau.

Khoảng hở nhỏ nhất trung bình của PSTMO là 18,8 cm, so với Constrained 21,4 cm. Cùng với lượt không đạt, kết quả này đặt ra hai điều kiện riêng cho triển khai: đủ khoảng cách vật cản và dừng trong dung sai đích. Kiểm tra hình học ở đầu ra PSTMO không thay thế đánh giá chuyển động thực. Kết quả thử nghiệm lặp tại Mục III-H được dùng để xem xét thêm tính lặp lại của hiệu năng.

Sai số vị trí cuối trung bình của năm phương án theo thứ tự trên là 86,8; 86,5; 82,3; 83,3; 89,6 mm; sai số hướng tương ứng là 0,080; 0,069; 0,056; 0,056; 0,039 rad, chỉ tính các lượt đạt.

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

@table r04_extra Chiều dài, độ cong cực đại và chỉ số bổ sung tại R04

@note Ký hiệu như Bảng VI. L, κmax và τ dùng năm lượt; D và emax chỉ dùng lượt đạt.

Theo thứ tự đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO, Eκ,exec lần lượt là 3,607; 2,681; 3,140; 3,283; 2,049 m⁻¹; phân vị 95% của trị tuyệt đối độ giật lần lượt là 2,374; 2,265; 2,335; 2,325; 2,271 m/s³. PSTMO giảm sai số bám đường cực đại so với cả bốn đối chứng, nhưng sai số vị trí cuối lớn hơn các phương án còn lại trong tập lượt đạt. Khả năng bám đường và độ chính xác dừng tại đích cần được đánh giá riêng.

### 3) Trường hợp sát ngưỡng và tiêu chí hoàn thành

So sánh theo bộ lập kế hoạch cho thấy PSTMO nhanh hơn Constrained ở bốn cặp; với Smac Hybrid, thời gian tăng 0,05 s. RMSE thấp hơn Constrained ở cả năm cặp. So với Savitzky–Golay, PSTMO cũng giảm cả thời gian và RMSE ở năm cặp. So với Simple, PSTMO chỉ nhanh hơn ở hai cặp nhưng có RMSE thấp hơn ở bốn cặp. Trung bình toàn tuyến cần được đọc cùng phân bố theo bộ lập kế hoạch.

Đường gốc với Theta* có sai số vị trí cuối 0,100234 m, vượt ngưỡng 0,10 m khoảng 0,234 mm. Lượt này được phân loại không đạt theo cùng tiêu chí với PSTMO tại R03. Mức vượt nhỏ mô tả độ gần ngưỡng của kết quả, chưa chứng minh một khác biệt cơ học đáng kể. Tỷ lệ thành công của ma trận vẫn là 123/125.

Hai trường hợp sát ngưỡng ở R03 và R04 cho thấy cần báo sai số liên tục bên cạnh tỷ lệ hoàn thành. Đường tham chiếu bảo toàn vị trí đích, nhưng robot còn chịu ảnh hưởng của bám đường, tiếp cận và dừng. Dữ liệu hiện tại xác định được kết quả cuối; chưa đủ để quy mức vượt ngưỡng cho một thành phần điều khiển riêng.

Sai số vị trí cuối trung bình của năm phương án theo thứ tự trên là 82,6; 85,2; 81,8; 85,6; 87,4 mm; sai số hướng tương ứng là 0,075; 0,066; 0,065; 0,067; 0,076 rad, chỉ tính các lượt đạt.

## F. R05 Rẽ từ hành lang ngang vào hành lang dọc

### 1) Nhiệm vụ ngắn với chuyển hướng tập trung

R05 bắt đầu tại (−5,0; 0,0) m và kết thúc tại (0,0; 3,0) m, với hướng đích π/2. Tuyến tập trung chuyển hướng từ hành lang ngang sang hành lang dọc. Thời gian nhiệm vụ phụ thuộc cả đoạn rẽ lẫn quá trình giảm tốc gần đích. Đây là trường hợp bổ sung cho tuyến chéo dài R02.

Hình 13 thể hiện năm đường tham chiếu và vùng khác biệt hình học theo quy tắc chọn vùng tại R01. Bảng XIII tổng hợp kết quả trên năm bộ lập kế hoạch. Toàn bộ 25 lượt khảo sát đều đạt, tạo tập so sánh đầy đủ ở R05. Kết quả kiểm chứng lặp của cùng tuyến được trình bày riêng ở Mục III-H.

@table r05 Kết quả mô tả R05 trên năm bộ lập kế hoạch

@note Cả 25 lượt của ma trận khảo sát R05 đều đạt. Eκ,ref và c là chỉ số đường tham chiếu; T và e là chỉ số thực thi. Kết quả này không được thay thế cho tỷ lệ thành công của bộ thử nghiệm lặp.

### 2) So sánh thời gian và chất lượng đường

PSTMO đạt thời gian trung bình 37,69 s. Các giá trị tương ứng của đường gốc, Savitzky–Golay và Constrained là 40,27 s, 40,02 s và 39,33 s. Simple đạt 38,25 s, gần PSTMO nhất về thời gian. RMSE của PSTMO là 20,8 mm; Constrained và đường gốc lần lượt có giá trị 24,4 mm và 21,2 mm. Mức chênh lệch về thời gian và RMSE phụ thuộc phương án đối chứng.

Tích phân bình phương độ cong của PSTMO là 2,44 m⁻¹, so với Constrained 25,35 m⁻¹ và đường gốc 141,50 m⁻¹. Tỷ lệ giảm Eκ,ref lớn hơn tỷ lệ giảm thời gian thực thi. Robot thực thi qua bộ điều khiển và các tầng giới hạn vận tốc, nên tổng thời gian không chỉ phụ thuộc chỉ số độ cong. Hình 14 thể hiện sự khác biệt giữa các bộ lập kế hoạch.

@table r05_extra Chiều dài, độ cong cực đại và chỉ số bổ sung tại R05

@note Ký hiệu như Bảng VI. L, κmax và τ dùng năm lượt; D và emax chỉ dùng lượt đạt.

@figure r05_paths R05 với Theta*: rẽ từ hành lang ngang vào hành lang dọc. Hình sử dụng ma trận khảo sát một lượt; các thất bại của bộ thử nghiệm lặp được báo riêng ở Mục III-I.

@figure r05_metrics R05: kết quả một lượt cho từng tổ hợp bộ lập kế hoạch–phương án. Tất cả các lượt trong hình đều đạt, nhưng điều đó chưa mô tả tính lặp lại của nhiệm vụ.

Theo thứ tự đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO, Eκ,exec lần lượt là 2,864; 2,117; 2,587; 2,545; 1,900 m⁻¹; phân vị 95% của trị tuyệt đối độ giật lần lượt là 2,364; 2,425; 2,432; 2,384; 2,331 m/s³. PSTMO có sai số cực đại và tích phân bình phương độ cong thực thi thấp nhất ở mức trung bình, nhưng sai số vị trí cuối cao nhất. Lợi ích trong đoạn di chuyển không bảo đảm cải thiện tương ứng khi dừng tại đích.

### 3) Khoảng hở và chuyển tiếp sang kiểm chứng lặp

So sánh theo bộ lập kế hoạch cho thấy PSTMO nhanh hơn Constrained ở bốn cặp, nhưng với Theta* tăng 0,01 s và tăng RMSE 0,33 mm. So với Simple, PSTMO giảm thời gian ở ba cặp và giảm RMSE ở cả năm cặp. So với đường gốc và Savitzky–Golay, số cặp nhanh hơn lần lượt là bốn và năm. Các chênh lệch rất nhỏ là kết quả mô tả của một lượt, chưa xác định tính ổn định khi lặp.

Khoảng hở nhỏ nhất trung bình của PSTMO là 20,8 cm, so với 22,2 cm của Constrained; chênh lệch nhỏ hơn tại R01. Thời lượng hậu xử lý tương ứng là khoảng 0,077 s và 0,022 s. Trên R05, PSTMO giảm thời gian thực thi và RMSE trung bình so với Constrained nhưng có thời lượng hậu xử lý dài hơn. Các chỉ số này được diễn giải theo phạm vi đo tại Mục III-A.

Việc hoàn thành 25 lượt khảo sát chưa xác định tỷ lệ thành công khi chạy lặp lại. Bộ kiểm chứng tiếp theo ghi nhận một số lượt kết thúc trước đích ngay trên R05. Đối với nhiệm vụ ngắn, chi phí phục hồi một lượt thất bại có thể ảnh hưởng đáng kể thời gian chu kỳ. Nghiên cứu chưa đo chi phí này, nên thời gian trung bình trên các lượt đạt cần được báo cùng tỷ lệ hoàn thành.

Sai số vị trí cuối trung bình của năm phương án theo thứ tự trên là 40,3; 45,6; 38,3; 44,8; 46,5 mm; sai số hướng tương ứng là 0,081; 0,079; 0,077; 0,063; 0,063 rad, chỉ tính các lượt đạt.

## G. Tổng hợp so sánh năm phương án

### 1) Kết quả chung và điều kiện ghép cặp

Ma trận khảo sát có 123/125 lượt hoàn thành. Bảng XV cho thấy PSTMO có thời gian, RMSE và tích phân bình phương độ cong tham chiếu trung bình thấp nhất. Tuy nhiên, thời gian và RMSE dùng tập lượt đạt riêng của mỗi phương án, nên hiệu giữa hai trung bình chưa phải chênh lệch trên cùng một tập đầu vào.

@table pooled Tổng hợp mô tả ma trận 125 lượt

@note T và e lấy trên 24 lượt đạt của đường gốc/PSTMO và 25 lượt đạt của mỗi phương án còn lại. Eκ,ref dùng toàn bộ 25 đường mỗi phương án. Năm bộ lập kế hoạch là năm đầu vào khác nhau, không phải năm lần lặp.

Bảng XVI ghép PSTMO với từng đối chứng cùng tuyến và bộ lập kế hoạch, chỉ giữ cặp cùng hoàn thành. Có 23 cặp với đường gốc và 24 cặp với mỗi phương án làm mượt. PSTMO giảm cả ba chỉ số trung bình trên các cặp này; mức giảm thời gian nhỏ nhất khi so với Simple.

@table all_pairs Chênh lệch PSTMO so với bốn đối chứng trên các cặp cùng hoàn thành

@note n là số cặp; Δ = PSTMO − đối chứng. Mỗi cặp dùng cùng đường gốc. Các chênh lệch là thống kê mô tả của một lượt cho mỗi cấu hình, không kèm kiểm định ý nghĩa thống kê.

PSTMO nhanh hơn đường gốc ở 21/23 cặp, Simple ở 14/24, Savitzky–Golay ở 23/24 và Constrained ở 21/24. Số cặp có RMSE thấp hơn tương ứng là 12/23, 14/24, 15/24 và 17/24. Như vậy, lợi ích thời gian xuất hiện thường xuyên hơn lợi ích bám đường, đặc biệt khi so với Savitzky–Golay. Không có cơ sở để kết luận PSTMO giảm đồng thời mọi chỉ số trên mọi đầu vào.

@figure matrix_delta So sánh PSTMO với bốn đối chứng. Chấm là chênh lệch trung bình; đoạn thẳng là khoảng từ nhỏ nhất đến lớn nhất, không phải khoảng tin cậy. Hai hàng đầu dùng các cặp cùng đạt; hàng cuối dùng đủ 25 cặp đường tham chiếu. Số phía trên mỗi cột là số cặp có chỉ số giảm trên tổng số cặp tương ứng.

Hình 15 cho thấy chênh lệch thời gian và RMSE có thể đổi dấu giữa các đầu vào dù trung bình âm. Phân tích theo tuyến tại Mục III-B–F xác định những trường hợp này. Mức giảm độ cong tham chiếu còn cần được xem xét theo mật độ điểm.

### 2) Chiều dài và độ cong của đường tham chiếu

Các bảng theo tuyến cho thấy PSTMO đồng thời thay đổi chiều dài và độ cong đường tham chiếu. Vì vậy, thời gian thực thi thấp hơn không thể được quy hoàn toàn cho một yếu tố hình học. Bảng XVII kiểm tra thêm độ cong sau khi thống nhất bước lấy mẫu của phần tịnh tiến.

@table translation Hình học phần tịnh tiến sau lấy mẫu lại và dấu quay tại chỗ

@note κt,max và Eκ,t là trung bình trên 25 đường sau khi tách phần tịnh tiến và lấy mẫu lại với bước 0,05 m. N là số điểm đầu ra trung bình trước lấy mẫu lại; “Quay” là tổng số dấu quay tại chỗ trong 25 đường. Đây không phải số lần xoay hướng do bộ điều khiển thực hiện.

Bảng XVII cho thấy số điểm đầu ra của PSTMO lớn hơn các đối chứng. Sau khi thống nhất bước lấy mẫu phần tịnh tiến, Eκ,t của PSTMO vẫn thấp hơn ở mức trung bình. Phép so sánh này không khẳng định các đường gấp khúc có độ cong liên tục theo nghĩa giải tích.

Hai dấu quay tại chỗ được ghi trong 25 đường PSTMO, với tổng góc khoảng 1,608 rad. Các đối chứng không chứa dấu theo tiêu chí đã định; tuy nhiên, RPP vẫn có thể yêu cầu xoay hướng. Số dấu không thay thế số lần quay quan sát được khi thực thi.

Sai lệch vị trí đầu–cuối do hậu xử lý không vượt 2,8 × 10⁻¹⁷ m, ở mức sai số số học. Kết quả chỉ kiểm tra vị trí của đường, không thay cho kiểm tra hướng, định vị hoặc sai số robot dừng tại đích.

### 3) Quỹ đạo thực thi và độ chính xác cuối

@table execution_extra Chỉ số chuyển động và sai số cuối của năm phương án

@note Trung bình trên các lượt đạt: 24 lượt cho đường gốc/PSTMO, 25 lượt cho mỗi phương án còn lại. j95 là phân vị 95% của trị tuyệt đối độ giật tịnh tiến trong từng lượt; bảng lấy trung bình các phân vị này. ep và eψ là sai số vị trí và hướng cuối. Eκ,exec được tính trên quỹ đạo đã xử lý như Mục III-A.

PSTMO có Eκ,exec trung bình thấp nhất trong Bảng XVIII. Trên các cặp cùng hoàn thành, chỉ số này giảm ở 23/23 cặp với đường gốc, 21/24 với Simple và toàn bộ 24 cặp với mỗi đối chứng Savitzky–Golay và Constrained. Mức cải thiện hình học thực thi nhất quán hơn mức giảm RMSE trong ma trận khảo sát.

Quãng đường thực thi trung bình là 8,510; 8,507; 8,545; 8,555 và 8,539 m theo cùng thứ tự năm phương án. Giá trị gộp của PSTMO lớn hơn Simple, nhưng chênh lệch ghép cặp là −0,040 m trên 24 cặp cùng đạt. Sự khác nhau xuất phát từ việc Simple có thêm một lượt hoàn thành mà PSTMO không đạt. Đây là lý do cần báo mẫu số và dùng đầu vào tương ứng khi đánh giá chênh lệch.

PSTMO có sai số bám đường cực đại trung bình thấp hơn bốn đối chứng, nhưng sai số cuối không giảm theo cùng thứ tự. Độ chính xác dừng còn phụ thuộc cách tiếp cận đích, ngưỡng bộ kiểm tra đích và quá trình giảm tốc sau khi tác vụ hoàn tất.

@figure execution_trace Minh họa R02/Theta* cho đủ năm phương án. Trên: đường tham chiếu nét đứt và quỹ đạo thực thi nét liền tại vùng rẽ; giữa: vận tốc góc của đường gốc, Simple và SG; dưới: Constrained và PSTMO. Hai đồ thị vận tốc dùng cùng thang đo, tín hiệu không lọc và thời gian riêng tính từ đầu từng lượt. Đây là một trường hợp minh họa, không phải kết quả trung bình toàn ma trận.

Hình 16 đối chiếu hình dạng đường với đáp ứng góc mà không căn chỉnh các đỉnh tín hiệu. Do RPP tắt điều tiết vận tốc theo độ cong và chi phí, không thể dùng hai chức năng này để giải thích mức giảm thời gian. Các phương án cũng chịu cùng tần số điều khiển và giới hạn vận tốc.

### 4) Biên độ chuyển động, thời lượng xử lý và an toàn

@table dynamics Cực đại quan sát được của các đại lượng chuyển động

@note Mỗi ô là giá trị lớn nhất trong toàn bộ 25 lượt, bao gồm không đạt. vb là vận tốc bánh tương đương suy từ vận tốc thân xe với khoảng cách bánh 0,2548 m; a⊥ = vω; a và α lần lượt là gia tốc tịnh tiến và gia tốc góc. Đây là đại lượng ước lượng tại các mẫu đã ghi, không phải phép đo trực tiếp vận tốc từng bánh.

Bảng XIX ghi nhận vận tốc góc cực đại khoảng 0,249 rad/s cho cả năm phương án, thấp hơn giới hạn 0,80 rad/s trong mô hình PSTMO. Vận tốc bánh tương đương không vượt khoảng 0,316 m/s và gia tốc ngang không vượt 0,063 m/s²; các giới hạn mô hình tương ứng là 0,36 m/s và 0,18 m/s². Vận tốc tịnh tiến cực đại xấp xỉ 0,30 m/s; phần vượt nhỏ hơn 10⁻⁷ m/s ở dữ liệu lưu được xem là sai số số học của mô phỏng.

Gia tốc tịnh tiến cực đại của PSTMO là 0,302 m/s², cao hơn bốn đối chứng trong ma trận dù vẫn thấp hơn giới hạn tăng tốc 0,35 m/s². Cực đại gia tốc góc là 0,732 rad/s², thấp hơn giới hạn 1,20 rad/s² nhưng cao hơn Constrained. Các kết quả này mô tả đáp ứng của toàn hệ thống dưới cùng bộ điều khiển; chúng chưa tách được đóng góp riêng của các ràng buộc nội bộ PSTMO hoặc bảo đảm rằng mọi trạng thái giữa hai mẫu đều thỏa giới hạn.

@table safety Thời lượng hậu xử lý và các chỉ số giám sát trong ma trận khảo sát

@note τ là trung bình trên 25 lượt; đường gốc không gọi hậu xử lý. “Dừng” là số lượt đáp ứng điều kiện dừng vật lý; “Can thiệp” là tổng số lần bộ giám sát va chạm can thiệp; “Mẫu va chạm” là tổng mẫu đường bao giao cắt vật cản trên đường tham chiếu. Các số đếm bao gồm cả lượt không đạt.

Bảng XX cho thấy PSTMO có thời lượng hậu xử lý cao nhất. Giá trị bằng không của Savitzky–Golay được ghi theo đồng hồ mô phỏng, không chứng minh chi phí tính toán bằng không. Các phép đo này chưa phải thời gian CPU hoặc độ trễ trên phần cứng độc lập.

Toàn bộ 125 lượt đáp ứng điều kiện dừng vật lý; hai lượt vẫn không đạt do sai số vị trí. Không ghi nhận can thiệp của bộ giám sát va chạm hoặc mẫu đường bao giao cắt vật cản trên đường tham chiếu. Khoảng hở ở các bảng theo tuyến cho thấy PSTMO thường đi gần vật cản hơn Constrained. Phép kiểm tra trên bản đồ tĩnh không thay thế phép đo tiếp xúc vật lý và không xác định được khoảng hở nhỏ nhất của toàn bộ chuyển động thực thi.

## H. Kiểm chứng bổ sung về tính lặp lại

### 1) Thiết kế và phương pháp phân tích

Bộ kiểm chứng riêng gồm 250 lượt: năm tuyến, năm bộ lập kế hoạch, hai phương án PSTMO–Constrained và năm lần lặp. Constrained là đối chứng tối ưu số có thể tích hợp trong cùng hệ Nav2. Kết luận thống kê trong mục này chỉ áp dụng cho hai phương án với cấu hình đã nêu; so sánh năm phương án vẫn dựa trên ma trận mô tả ở Mục III-B–G.

Toàn bộ 125 cặp có cùng đường gốc. Constrained đạt 122/125 và PSTMO đạt 121/125; có 118 cặp cùng đạt để phân tích chỉ số liên tục. Mỗi tổ hợp tuyến–bộ lập kế hoạch tạo một nhóm b. Gọi Rb là tập lần lặp cùng đạt trong nhóm; db là trung bình hiệu PSTMO trừ Constrained. Công thức 9 tính chênh lệch chung Δ bằng trung bình đều của 25 nhóm, tránh coi các lần lặp của cùng đầu vào là các đầu vào độc lập.

@equation d_b=\frac{1}{\left|R_b\right|}\sum_{r\in R_b}(y_{P,br}-y_{C,br}), || \Delta=\frac{1}{25}\sum_{b=1}^{25}d_b.

Khoảng tin cậy 95% dùng bootstrap trên 25 nhóm với 20.000 lần lấy mẫu lại [5]. Kiểm định hạng có dấu Wilcoxon hai phía áp dụng cho db; các hiệu không quá 10⁻¹² về trị tuyệt đối được xem là bằng không [6]. Ba giá trị p của thời gian, RMSE và Eκ,exec được hiệu chỉnh Holm [7]. Các nhóm còn chia sẻ tuyến, bộ lập kế hoạch và bản đồ, nên kết quả suy luận được giới hạn trong thiết kế đã khảo sát. Đây là lặp thực thi với đầu vào cố định, chưa phải thử nghiệm nhiều mức nhiễu hay nhiều điều kiện ngẫu nhiên.

### 2) Kết quả chính và các đánh đổi

@table primary Chênh lệch PSTMO so với Constrained trong bộ kiểm chứng lặp

@note Δ = PSTMO − Constrained, trung bình đều trên 25 nhóm. Khoảng tin cậy dùng bootstrap nhóm; p Holm hiệu chỉnh ba chỉ số chính. Phân tích chỉ dùng những cặp cả hai phương án cùng đạt và không gộp dữ liệu với ma trận 125 lượt.

PSTMO giảm thời gian trung bình theo nhóm 2,857 s, RMSE 2,985 mm và Eκ,exec 1,038 m⁻¹. Khoảng tin cậy của cả ba chênh lệch đều nằm dưới không; số nhóm có chỉ số giảm lần lượt là 24/25, 19/25 và 24/25. Không dùng trung bình gộp 118 cặp thay Δ vì số cặp cùng đạt khác nhau giữa các nhóm.

Hình 17 cho thấy RMSE tăng ở nhiều nhóm hơn hai chỉ số còn lại. PSTMO có lợi ích trung bình nhưng không cải thiện đồng thời mọi đầu vào. Bảng XXII bổ sung quãng đường, độ giật, sai số đích, khoảng hở và thời lượng hậu xử lý để đánh giá những mặt bất lợi cùng với ba chỉ số chính.

@table secondary Các chỉ số bổ sung trong bộ kiểm chứng lặp

@note Các khoảng tin cậy là phân tích bổ sung; hiệu chỉnh Holm chỉ áp dụng cho ba chỉ số trong Bảng XXI. Dấu âm thuận lợi cho quãng đường, độ giật, tích phân bình phương độ cong và thời lượng xử lý; đối với khoảng hở, dấu âm biểu thị robot được dẫn gần vật cản hơn.

@figure repeat_blocks Chênh lệch trung bình của 25 nhóm trong bộ kiểm chứng lặp. Trong mỗi tuyến, thứ tự là A*, Dijkstra, Theta*, Smac 2D và Smac Hybrid. Mỗi điểm dùng các lần lặp cùng đạt; đường đứt là trung bình đều của 25 điểm. Hình thể hiện mức thay đổi giữa các đầu vào, không biểu diễn 125 quan sát độc lập.

Khoảng hở giảm trung bình 35,161 mm, sai số vị trí cuối tăng 3,380 mm và thời lượng hậu xử lý tăng 71,220 ms. Vì vậy, lợi ích thời gian và bám đường cần được cân nhắc cùng yêu cầu khoảng hở và dung sai đích. Mức giảm độ giật 0,078 m/s³ chưa xác định độ rung tải hàng hay điện năng tiêu thụ, do nghiên cứu chưa đo các đại lượng đó.

Trong 125 cặp, bốn cặp chỉ Constrained đạt và ba cặp chỉ PSTMO đạt. Kiểm định McNemar chính xác hai phía cho p = 1,0. Dữ liệu chưa cho thấy khác biệt tỷ lệ thành công; kết quả cũng không chứng minh tương đương hoặc không kém hơn. Năm lần lặp ở mỗi cấu hình còn ít để đánh giá các thất bại hiếm. Các lượt không đạt được giữ đầy đủ trong Mục III-I.

## I. Phân tích các trường hợp không đạt và giới hạn nghiên cứu

### 1) Các lượt vượt dung sai vị trí

@table failures Các lượt không đạt trong hai bộ dữ liệu

@note KS là ma trận khảo sát 125 lượt; số 1–5 là lần lặp trong bộ 250 lượt. G: đường gốc; P: PSTMO; C: Constrained. Hai bộ được báo riêng, không cộng mẫu số để tính một tỷ lệ thành công chung. “Kết thúc trước đích” mô tả trạng thái quan sát được, chưa xác định nguyên nhân và không đồng nghĩa đã dừng vật lý.

Trong ma trận khảo sát, PSTMO/Smac Hybrid tại R03 có sai số vị trí cuối 0,102385 m; đường gốc/Theta* tại R04 có sai số 0,100234 m. Cả hai tác vụ đều hoàn tất và robot đã dừng, nhưng vượt ngưỡng đánh giá 0,10 m. Giữ nguyên phân loại này tránh việc điều chỉnh dung sai sau khi biết kết quả. Những lượt này được tính vào tỷ lệ thất bại và không được dùng như thời gian hoàn thành hợp lệ khi so sánh ghép cặp.

Bộ lặp ghi thêm hai trường hợp R04 vượt ngưỡng: PSTMO/Smac Hybrid ở lần 4 đạt sai số 0,101583 m; Constrained/Theta* ở lần 4 đạt 0,100358 m. Phần vượt lần lượt khoảng 1,6 mm và 0,4 mm, nhỏ hơn nhiều so với sai số của các lượt kết thúc xa đích. Các con số phản ánh trạng thái mô phỏng, không biểu thị độ chính xác đo trên robot thực ở mức dưới milimét.

### 2) Các lượt kết thúc trước đích

Năm thất bại còn lại tập trung tại R05 trong bộ lặp, gồm ba lượt PSTMO và hai lượt Constrained. Sai số vị trí còn lại từ 0,686 đến 3,655 m. Các lượt này kết thúc trước khi hoàn thành hành trình, khác về bản chất với những trường hợp chỉ vượt nhẹ dung sai. Bảng XXIII giữ rõ tuyến, bộ lập kế hoạch, phương án và lần chạy để đối chiếu dữ liệu.

@figure endpoints Sai số vị trí cuối của toàn bộ 250 lượt kiểm chứng lặp. Trục đứng dùng thang log để hiển thị đồng thời sai số cỡ centimét và các lượt kết thúc xa đích. Ngưỡng 0,10 m chỉ là một phần của tiêu chí hoàn thành; các lượt thuộc ma trận 125 lượt không nằm trong hình này.

Hình 18 cho thấy phần lớn sai số nằm dưới ngưỡng, nhưng một số lượt R05 dừng phép thử khi còn xa đích. Thời gian trung bình trên các cặp cùng đạt chưa bao gồm chi phí phục hồi những thất bại này. Vì chưa có một chính sách phục hồi chung được đo, không thể quy đổi kết quả thành thời gian chu kỳ vận hành thực tế có tính đến thử lại.

Dữ liệu hiện tại xác định được trạng thái kết thúc, chưa chứng minh nguyên nhân cụ thể của từng thất bại. Việc quy lỗi cho hình dạng đường, bộ điều khiển hoặc định vị cần đối chiếu đồng bộ nhật ký và tín hiệu ở thời điểm sự kiện. Các thất bại xa đích xuất hiện ở cả PSTMO và Constrained; số lượng hiện có chưa đủ để gán ưu thế về độ tin cậy cho một phương án.

### 3) Phạm vi kết luận

Nghiên cứu đánh giá năm dạng tuyến trên một bản đồ tĩnh, một robot mô phỏng và một cấu hình RPP. R01 của ma trận khảo sát được thu ở đợt khác R02–R05; dữ liệu một lượt chỉ hỗ trợ so sánh mô tả. Bộ lặp dùng cấu hình cố định nhưng chỉ có PSTMO và Constrained. Thứ tự chạy chưa được xác nhận ngẫu nhiên hóa; bốn tiến trình có miền truyền thông riêng vẫn dùng chung tài nguyên máy tính. Các giới hạn này đặc biệt ảnh hưởng cách diễn giải thời lượng xử lý và khả năng suy rộng.

Constrained được đánh giá với trọng số độ cong bằng không. Các phương án chưa được tinh chỉnh theo cùng ngân sách đánh giá tham số trên tập hiệu chỉnh tách biệt, nên kết quả đại diện cho các cấu hình đã báo. Phân tích cặp cùng đạt còn có thể thiên lệch về các nhiệm vụ dễ hoàn thành; tỷ lệ thành công và từng trường hợp không đạt phải được đọc cùng các chỉ số liên tục.

PSTMO cho thấy lợi ích về hình học và một số chỉ số thực thi trong phạm vi khảo sát, đồng thời có đánh đổi về khoảng hở, sai số cuối và thời lượng xử lý. Kiểm tra robot thực, nhiều bố trí bản đồ và thí nghiệm loại bỏ từng thành phần sẽ giúp xác định phạm vi áp dụng và đóng góp riêng của thuật toán. Dữ liệu hiện có chưa hỗ trợ kết luận về điện năng, rung tải hàng hoặc ưu thế tỷ lệ thành công.

# TÀI LIỆU THAM KHẢO CHO PHẦN II VÀ III

[1] S. Macenski, S. Singh, F. Martín, and J. Ginés, “Regulated pure pursuit for robot path tracking,” Autonomous Robots, vol. 47, pp. 685–694, 2023, doi: 10.1007/s10514-023-10097-6.

[2] A. Savitzky and M. J. E. Golay, “Smoothing and differentiation of data by simplified least squares procedures,” Analytical Chemistry, vol. 36, no. 8, pp. 1627–1639, 1964, doi: 10.1021/ac60214a047.

[3] Nav2, “Constrained Smoother,” Nav2 Jazzy API documentation. [Online]. Available: https://api.nav2.org/nav2-jazzy/html/md_nav2_constrained_smoother_README.html. Accessed: Oct. 13, 2026.

[4] Nav2, “Smoother server implementation,” navigation2, branch jazzy. [Online]. Available: https://github.com/ros-navigation/navigation2. Accessed: Oct. 13, 2026.

[5] B. Efron, “Bootstrap methods: Another look at the jackknife,” The Annals of Statistics, vol. 7, no. 1, pp. 1–26, 1979, doi: 10.1214/aos/1176344552.

[6] F. Wilcoxon, “Individual comparisons by ranking methods,” Biometrics Bulletin, vol. 1, no. 6, pp. 80–83, 1945, doi: 10.2307/3001968.

[7] S. Holm, “A simple sequentially rejective multiple test procedure,” Scandinavian Journal of Statistics, vol. 6, no. 2, pp. 65–70, 1979.
