# II. XÂY DỰNG THUẬT TOÁN PSTMO

## A. Bài toán hậu xử lý và phạm vi của phương pháp

PSTMO được xây dựng như một bộ hậu xử lý đường đi toàn cục cho robot dẫn động vi sai. Đầu vào gồm đường gấp khúc do planner cung cấp, bản đồ chi phí, footprint của robot và các giới hạn chuyển động. Đầu ra là một chuỗi pose có thể chuyển trực tiếp cho bộ điều khiển bám đường của Nav2. Phương pháp giữ nhiệm vụ tìm hành lang di chuyển ở planner và tập trung xử lý các điểm đổi hướng: đi qua bằng đoạn chuyển tiếp trơn khi khả thi, hoặc giữ khả năng quay tại chỗ khi không có chuyển tiếp phù hợp. Do đó, PSTMO không thay thế bộ lập kế hoạch toàn cục và không được xem là một bộ điều khiển vận tốc mới.

Gọi Q = {q₀, …, qₙ} là đường vào. Tại một đỉnh C, hai vectơ đơn vị u và w biểu diễn hướng đến và hướng đi của robot. Một phương án tại góc phải thỏa mãn đồng thời ba yêu cầu: hình học không suy biến, chuyển động đáp ứng giới hạn robot và footprint không xâm nhập vùng cấm theo phép kiểm tra rời rạc của hệ thống. Các phương án tại hai góc liên tiếp còn phải tương thích về phần chiều dài sử dụng trên đoạn chung. Hình 1 trình bày chuỗi xử lý; lựa chọn tại từng góc chỉ là bước tạo trạng thái trước khi tối ưu trên toàn chuỗi.

@figure pipeline Luồng xử lý PSTMO và vị trí tích hợp vào Nav2. Các khối là sơ đồ thuật toán; dữ liệu thực thi được thu độc lập từ ground truth của mô phỏng.

Tiền xử lý sử dụng chế độ condition_only: xử lý điểm trùng, nhiễu hình học và dao động cục bộ trong phạm vi kiểm tra an toàn, không bật nhánh rút gọn tham lam theo đường nhìn thẳng. Quyết định này hạn chế việc thay đổi đồng thời hành lang đường đi và cơ chế xử lý góc. Tuy nhiên, đường sau conditioning vẫn có thể khác đường Raw; kết quả của toàn bộ PSTMO vì vậy không thể được quy hoàn toàn cho riêng một đoạn Bézier.

## B. Xây dựng đoạn chuyển tiếp có nối hình học G²

Với khoảng cắt d > 0 và hệ số điều khiển α > 0, đặt A = C − du, B = C + dw và q = αd. Đường cong Bézier bậc năm được xác định bởi sáu điểm điều khiển trong (1); biểu thức tham số của đường cong được cho trong (2). Hình 2 minh họa quan hệ giữa góc gốc, hai điểm cắt và đa giác điều khiển.

@equation P_0=A,\quad P_1=A+qu,\quad P_2=A+2qu; || P_3=B-2qw,\quad P_4=B-qw,\quad P_5=B.

@equation r(t)=\sum_{j=0}^{5}\binom{5}{j}(1-t)^{5-j}t^jP_j,\quad t\in[0,1].

Từ cấu trúc đối xứng theo tiếp tuyến ở hai đầu, r′(0) = 5qu, r′(1) = 5qw và r″(0) = r″(1) = 0. Khi đạo hàm bậc nhất khác không, độ cong tại hai đầu bằng không, trùng với độ cong của đoạn thẳng kề bên. Điểm nối vì vậy liên tục về vị trí, hướng tiếp tuyến và độ cong, tức đạt G² trên phần tịnh tiến chính quy. Đây là điều kiện hình học, không phải khẳng định vận tốc và gia tốc theo thời gian của robot thực thi liên tục đến cùng bậc.

@figure bezier Cấu trúc Bézier bậc năm cho một góc vuông minh họa. P₀ và P₅ nằm trên hai cạnh; ba điểm đầu và ba điểm cuối thẳng hàng theo các tiếp tuyến tương ứng. Hình không biểu diễn một trial đo được.

Độ cong κ(t) được tính từ đạo hàm của đường cong; năng lượng độ cong Eκ là tích phân κ² theo chiều dài cung. Đại lượng này có đơn vị m⁻¹ và chỉ phản ánh mức uốn của đường. Nó không phải điện năng, công cơ học hay đại lượng tiêu thụ pin. Các ứng viên có đạo hàm suy biến, giá trị không hữu hạn hoặc dạng đổi dấu độ cong không phù hợp với góc đang xử lý bị loại trước khi so sánh chất lượng.

@equation \kappa=\frac{x'y''-y'x''}{(x'^2+y'^2)^{3/2}},\qquad E_\kappa=\int\kappa^2\,ds.

## C. Ràng buộc chuyển động và kiểm tra footprint

Với khoảng cách hai bánh b, vận tốc bánh trái và phải trên đoạn tịnh tiến là vL = v(1 − bκ/2) và vR = v(1 + bκ/2). PSTMO loại chuyển tiếp đòi hỏi đảo chiều bánh phía trong; thao tác quay tại chỗ được xử lý bằng trạng thái pivot riêng. Giới hạn vận tốc tại mẫu đường cong kết hợp vận tốc thân xe, vận tốc góc, gia tốc ngang và vận tốc bánh. Sau đó, các lượt lan truyền tiến và lùi áp đặt giới hạn tăng tốc, giảm tốc và điều kiện chuyển động góc. Vì vậy, κ nhỏ không phải điều kiện duy nhất để một ứng viên được chấp nhận.

@equation v\leq\min\left(v_{\max},\frac{\omega_{\max}}{\left|\kappa\right|},\sqrt{\frac{a_{\mathrm{lat},\max}}{\left|\kappa\right|}},\frac{v_{\mathrm{wheel},\max}}{f_{\max}}\right), || f_{\max}=\max\left(\left|1-b\kappa/2\right|,\left|1+b\kappa/2\right|\right).

Các hạng chứa |κ| trong mẫu số không tạo giới hạn hữu hạn tại đoạn thẳng. Kiểm tra an toàn sử dụng toàn bộ footprint theo vị trí và hướng robot dọc ứng viên, đồng thời áp dụng ngưỡng chi phí tại tâm. Hai kiểm tra này có vai trò khác nhau: chi phí tại tâm đánh giá mức tiến gần vùng lạm phát, còn phép quét footprint kiểm tra kích thước và hướng thân xe. Một tâm đường nằm trong vùng tự do không đủ để kết luận thân xe có thể đi qua. Do phép kiểm tra dựa trên bản đồ và lấy mẫu hữu hạn, kết quả an toàn của thuật toán phải được hiểu trong phạm vi độ phân giải và mô hình footprint sử dụng.

## D. Tìm ứng viên và lựa chọn theo cửa sổ chung

Việc dùng giới hạn vận tốc của hai bánh làm rõ sự khác nhau giữa chuyển tiếp và pivot. Trên chuyển tiếp tiến, các hệ số 1 ± bκ/2 phải không âm; một ứng viên vượt điều kiện này không được “sửa” bằng cách bỏ giới hạn bánh. Trong trạng thái pivot, vận tốc tịnh tiến được đưa về không và hướng thay đổi tại góc theo giới hạn quay riêng. Cách phân loại này giữ nhất quán mô hình robot vi sai với khả năng quay tại chỗ, thay vì áp dụng một bán kính quay dương cố định cho mọi chế độ chuyển động.

Tìm kiếm hierarchical_alpha_two_trim xét một khoảng cắt ưu tiên và một khoảng cắt tương thích với ngân sách đoạn kề. Khoảng cắt ưu tiên bị chặn bởi chiều dài hai đoạn và dmax; khoảng cắt tương thích còn xét phần chiều dài dành cho góc lân cận. Với đoạn nội bộ dài L và biên m, ngân sách chia cho mỗi đầu được lấy bằng (L − m)/2. Các khoảng cắt trùng nhau ở độ phân giải tìm kiếm hoặc nhỏ hơn dmin không được lặp lại. Mục đích là giữ cả lựa chọn uốn rộng và lựa chọn tiết kiệm chiều dài, thay vì chỉ giữ một nghiệm tốt cục bộ.

Tại mỗi khoảng cắt, tập α thô là {0,1; 0,2; 0,3; 0,4; 0,5}. Nếu toàn bộ tập thô không khả thi, thuật toán thử tập phục hồi {0,15; 0,25; 0,35; 0,45}; vùng quanh ứng viên tốt nhất sau đó được chia nhỏ để tinh chỉnh, loại các giá trị trùng. Mục tiêu hình dạng ở bước này là Eκ nhỏ nhất trong các ứng viên khả thi, không phải cực tiểu tổng thời gian chạy của robot. Sự phân biệt giữa tìm hình dạng và chọn trạng thái cuối giúp tránh diễn giải sai hàm mục tiêu của PSTMO.

Để so sánh chuyển tiếp với pivot, các phương án được tham số hóa trên cùng cửa sổ vào–ra và cùng các giới hạn vận tốc biên đạt được. Nếu pivot an toàn, nhánh chuyển tiếp chỉ được mở khi thời gian nhanh nhất của nhánh này cộng δT nhỏ hơn thời gian pivot. Các ứng viên còn phải nằm trong khoảng thời gian cạnh tranh tính từ ứng viên nhanh nhất; giới hạn này bị thu hẹp để duy trì lợi thế δT đối với pivot. Trên tập giữ lại, chi phí cục bộ kết hợp chi phí bản đồ chuẩn hóa, vận tốc góc chuẩn hóa và Eκ/(Eκ + Escale), với trọng số tương ứng 0,15; 0,10; 0,75. Pivot có chi phí 1 khi nhánh chuyển tiếp được mở và 0 khi nhánh này không được mở.

Gọi cmax là chi phí bản đồ lớn nhất trên các mẫu kiểm tra tại tâm và ωpeak là trị tuyệt đối vận tốc góc lớn nhất của hồ sơ cửa sổ chung. Đặt ρc = min(1, cmax/252), ρω = min(1, ωpeak/ωmax) và ρE = Eκ/(Eκ + Escale). Chi phí trong (5) là tổ hợp không thứ nguyên, phân biệt với Eκ dùng ở bước chọn hình dạng. Trọng số hiện tại có tổng bằng một. Điều kiện mở nhánh chuyển tiếp khi pivot an toàn được viết trong (6), với Tmin là thời gian chuyển tiếp khả thi nhanh nhất và Tp là thời gian pivot trên cùng cửa sổ.

@equation J=0.15\rho_c+0.10\rho_\omega+0.75\rho_E.

@equation T_{\min}+\delta_T<T_p.

Nếu pivot không an toàn, một chuyển tiếp khả thi vẫn có thể được giữ mà không cần vượt một phương án pivot không sử dụng được. Ngược lại, khi không có chuyển tiếp cạnh tranh và pivot an toàn, pivot bảo toàn một lựa chọn tại góc. Vì vậy, phương pháp không ép mọi góc trở thành cung trơn; việc giữ pivot là một phần của bài toán khả thi. Chi phí cộng theo góc cũng không bao gồm đầy đủ mọi tương tác động lực học toàn đường, nên kết quả DP cần được kiểm tra lại sau khi ghép.

## E. Tối ưu chuỗi góc và xuất đường đi

Gọi Sᵢ là tập trạng thái giữ lại tại góc i. Một trạng thái chuyển tiếp dùng khoảng cắt dᵢ; trạng thái pivot dùng khoảng cắt bằng không. Hai trạng thái liên tiếp tương thích nếu tổng hai khoảng cắt cộng biên m không vượt chiều dài đoạn chung Lᵢ. Với Jᵢ(k) là chi phí trạng thái k, quy hoạch động thực hiện truy hồi (7), chỉ xét tiền nhiệm tương thích. Truy vết từ trạng thái cuối cho chuỗi lựa chọn có tổng chi phí nhỏ nhất trên đồ thị hữu hạn đã xây dựng.

@equation F_i(k)=J_i(k)+\min_{j\in A_i(k)}F_{i-1}(j).

Trong (7), Aᵢ(k) = {j: dᵢ₋₁(j) + dᵢ(k) + m ≤ Lᵢ₋₁}. Khi chi phí bằng nhau trong sai số số học, thuật toán ưu tiên ít pivot hơn rồi sử dụng thứ tự trạng thái ổn định. Nếu mỗi lớp có Kᵢ trạng thái, chi phí tính phần DP tỷ lệ với ∑Kᵢ₋₁Kᵢ; bộ nhớ truy vết tỷ lệ với ∑Kᵢ. Kết luận tối ưu chỉ đúng trên các trạng thái và cạnh được giữ lại, không phải tối ưu toàn cục trên mọi đường cong liên tục trong không gian tự do.

Sau khi ghép đường, hệ thống kiểm tra lại bảo toàn điểm đầu–cuối, khả thi chuyển động, quét footprint trên các mối nối và quay hướng cuối. Đường chỉ được trả về khi vượt qua các kiểm tra cuối; trạng thái không có phương án an toàn được báo lỗi làm trơn. Bảng I nêu cấu hình sử dụng trong nghiên cứu. Đầu ra đưa sang RPP là đường hình học, không phải hồ sơ vận tốc bắt buộc mà bộ điều khiển phải thực thi [1]. Vì vậy, thời gian ước tính trong lựa chọn cục bộ và thời gian chạy vòng kín là hai đại lượng khác nhau, được đánh giá riêng ở Phần III.

@table params Cấu hình PSTMO trong nghiên cứu

@note Ký hiệu δT là biên lợi thế thời gian đối với pivot; time slack là ngưỡng trước khi bị chặn bởi điều kiện pivot. Bước xuất path không đồng nhất với bước quét ứng viên. Biên đoạn 0,05 m được suy ra từ cấu hình và độ phân giải bản đồ.

## F. Trình tự thực hiện và điều kiện chấp nhận

Quy trình thực hiện được tóm tắt theo thứ tự sau. Trước hết, hệ thống đọc footprint, costmap và các giới hạn, chuẩn hóa đường vào rồi xác định những góc cần xử lý. Tiếp theo, với mỗi góc, thuật toán kiểm tra pivot và sinh các ứng viên tại hai mức khoảng cắt; mỗi ứng viên phải vượt kiểm tra hình dạng, quét footprint và tham số hóa chuyển động trước khi vào tập so sánh. Các hồ sơ được đưa về cửa sổ chung, lọc theo điều kiện thời gian, chấm điểm và giữ một tập trạng thái hữu hạn cho từng góc.

Sau bước tạo trạng thái, thuật toán tạo cạnh giữa các trạng thái tương thích trên đoạn chung, chạy DP và truy vết lựa chọn. Cuối cùng, các đoạn thẳng, chuyển tiếp và pose pivot được ghép theo thứ tự đường đi, giữ pose đích và kiểm tra invariant toàn đầu ra. Thiếu trạng thái an toàn, không có chuỗi tương thích hoặc vi phạm kiểm tra cuối đều dẫn tới không chấp nhận đường. Do đó, trạng thái “đã tìm được một cung Bézier” chưa đủ để trả về kết quả thành công của smoother.

Trong pipeline này, tìm ứng viên, DP và hậu kiểm là ba nhiệm vụ có điều kiện kết thúc khác nhau. Tìm ứng viên giới hạn phạm vi hình dạng được xem xét; DP tìm nghiệm tốt nhất trong phạm vi đã giữ; hậu kiểm xác minh đường ghép đáp ứng hợp đồng đầu ra. Việc công bố riêng các tầng này cho phép tái lập quyết định thuật toán và tránh diễn giải kiểm tra cuối như một bước sửa đường không được mô tả. Chất lượng thực thi vẫn cần được đo sau bộ điều khiển, như thiết kế ở phần tiếp theo.

Giới hạn tối ưu của phương pháp xuất phát từ chính cách rút gọn không gian tìm kiếm. Hai khoảng cắt và tập α phân cấp giúp kiểm soát số phép kiểm tra, nhưng có thể bỏ qua một hình dạng khả thi ở khoảng cắt khác. Tương tự, một chuỗi có chi phí lớn hơn trong đồ thị có thể thực thi nhanh hơn dưới một controller khác vì chi phí J không phải hàm thời gian vòng kín đầy đủ. PSTMO vì vậy được đánh giá như một chiến lược hậu xử lý có cấu trúc và kiểm tra khả thi, không như bộ giải tối ưu thời gian liên tục của toàn robot trong mọi môi trường.

# III. XÂY DỰNG KỊCH BẢN THỬ NGHIỆM VÀ PHÂN TÍCH

## A. Môi trường mô phỏng và thiết kế đánh giá

### 1) Robot và chuỗi điều hướng

Nghiên cứu đánh giá PSTMO trong kho có các dãy kệ song song, hành lang ngang và các lối lấy hàng. Robot được mô phỏng bằng Gazebo, sử dụng ROS 2 Jazzy và Nav2. Chuỗi xử lý gồm lập kế hoạch toàn cục, hậu xử lý đường, bám đường bằng Regulated Pure Pursuit, làm trơn vận tốc và giám sát va chạm. Trong một nhóm tuyến–planner, phương án hậu xử lý là thành phần được thay đổi; bản đồ, robot, bộ điều khiển và điều kiện đích được giữ chung. Cách bố trí này đánh giá tác động của đường tham chiếu lên toàn hệ điều hướng, thay vì chỉ đánh giá đường cong trên hình vẽ.

@figure robot Sơ đồ footprint chữ nhật và khoảng cách bánh dùng trong mô hình PSTMO. O là gốc biểu diễn footprint, x hướng về phía trước. Track hiệu dụng của plugin động lực học được báo riêng trong Bảng II; sơ đồ không phải ảnh chụp robot thực.

Một khác biệt mô hình cần được làm rõ là track 0,2548 m dùng trong giới hạn động học PSTMO không trùng track hiệu dụng 0,2834 m của plugin dẫn động trong mô phỏng. Không thể sử dụng hai giá trị này thay thế nhau khi tái lập hồ sơ vận tốc bánh. Nghiên cứu giữ nguyên cấu hình đã sinh dữ liệu và báo cáo riêng từng giá trị, thay vì hợp nhất thành một thông số robot duy nhất. Sự khác biệt này cũng giới hạn việc suy trực tiếp từ khả thi động học trên đường tham chiếu sang đáp ứng vật lý của chassis.

@table robot Tham số robot và giao thức điều hướng

RPP được sử dụng theo triển khai Nav2 [1], với vận tốc đặt 0,30 m/s và lookahead cố định 0,50 m. Điều chỉnh vận tốc theo độ cong và chi phí của RPP được tắt trong cấu hình thử nghiệm; kiểm tra va chạm, xoay hướng, giảm tốc khi tiếp cận đích và các giới hạn vận tốc phía sau vẫn hoạt động. Do đó, đây là cấu hình bám đường kiểm soát chung cho các phương án, không phải phép đánh giá toàn bộ đặc tính điều tiết vận tốc của RPP trong cấu hình mặc định. Lợi thế thời gian quan sát được chỉ có ý nghĩa trong chuỗi điều khiển đã mô tả.

### 2) Bản đồ và năm bài toán di chuyển

Hình 4 trình bày bản đồ occupancy có độ phân giải 0,05 m/ô và năm đường Raw của ThetaStar để định vị các bài toán. Bản đồ này cũng được dùng cho hậu kiểm khoảng hở; các đường vẽ được lấy từ dữ liệu trial, không dựng bằng mắt. Bảng III cho tọa độ đầu–cuối theo hệ map. Năm tuyến giữ các vai trò khác nhau: R01 chuyển giữa hai lối qua giao cắt; R02 đi chéo dài qua kho; R03 chuyển lối theo hình chữ U; R04 đi chéo theo chiều ngược; R05 rẽ từ hành lang ngang vào lối lấy hàng.

@figure routes Vị trí năm tuyến trong cùng bản đồ kho. Mỗi màu là một đường Raw của ThetaStar; chấm tròn chỉ xuất phát và ngôi sao chỉ đích. Các đường có thể chồng lên nhau trên những hành lang dùng chung.

@table scenarios Tọa độ năm bài toán trong hệ map

Các hướng đích lần lượt là 0,950547 rad, π/2, −π/2, −π/2 và π/2 cho R01 đến R05. Hướng xuất phát được lưu trong trial cùng tọa độ; đối với các kịch bản sử dụng hướng suy ra từ tuyến, việc tái lập phải dùng pose đã ghi chứ không chỉ hai tọa độ trong Bảng III. Bài toán vì vậy là điều hướng giữa hai pose, không chỉ nối hai điểm. Đặc biệt, sai số quay cuối và quãng giảm tốc có thể quyết định một lượt có đạt hay không dù đường đi đã đưa tâm robot đến gần đích.

Năm tuyến được dùng để khảo sát các dạng đổi hướng trên một bố trí kho, không phải năm môi trường độc lập. Chúng chia sẻ kệ, độ phân giải bản đồ và nhiều đoạn hành lang. R04 không được coi là bản sao đối xứng vật lý của R02 vì tọa độ đích và các đoạn tiếp cận khác nhau. R05 cũng không được coi là tuyến dễ chỉ vì quãng đường ngắn hơn; giai đoạn chuyển từ đi ngang sang đi dọc và tiếp cận đích có thể tạo yêu cầu bám đường khác với một đoạn thẳng dài.

### 3) Planner và các phương án đối chứng

Mỗi tuyến được kết hợp với năm planner: NavFnAStar, NavFnDijkstra, ThetaStar, Smac2D và SmacHybrid. NavFnAStar và NavFnDijkstra cho phép khảo sát hai cấu hình tìm kiếm của NavFn; ThetaStar bổ sung đường any-angle; hai planner Smac cung cấp thêm cấu trúc đường và mô hình tìm kiếm. SmacHybrid sử dụng mô hình DUBIN, bán kính quay nhỏ nhất 0,35 m, 72 ô hướng và tắt làm trơn nội bộ. Bán kính của planner là ràng buộc của bộ lập kế hoạch đó, không phải giới hạn chung áp cho mọi đường của robot có khả năng pivot.

Năm phương án gồm Raw, Simple Smoother, Savitzky–Golay Smoother (SG), Constrained Smoother và PSTMO. Raw giữ đầu ra planner làm mức tham chiếu; Simple và SG đại diện cho các cách làm trơn sẵn có trong Nav2, trong đó SG có cơ sở từ bộ lọc bình phương tối thiểu cục bộ [2]. Constrained sử dụng tối ưu số để cân bằng các thành phần chất lượng đường [3]. Các tên phương án không được đồng nhất với tên planner: chẳng hạn Dijkstra tạo đường toàn cục, còn PSTMO xử lý đường đó sau khi lập kế hoạch.

@table baselines Cấu hình các phương án hậu xử lý

Cấu hình Constrained trong Bảng IV đặt w_curve = 0 và bán kính quay nhỏ nhất bằng không, phù hợp với lựa chọn không áp đặt độ cong của xe kiểu ô tô lên robot vi sai. Vì vậy, kết quả không phải phép so sánh PSTMO với Constrained đã bật ràng buộc độ cong. Đồng thời, việc dùng chung costmap và footprint điều hướng không có nghĩa các smoother thực hiện phép quét footprint nội bộ giống nhau. Nghiên cứu so sánh các cấu hình triển khai xác định, không tuyên bố tất cả phương án có cùng bảo đảm an toàn nội tại hoặc được tối ưu tham số đến mức tốt nhất có thể.

### 4) Hai bộ dữ liệu và kiểm soát đầu vào

Ma trận khảo sát gồm 5 tuyến × 5 planner × 5 phương án, tổng cộng 125 lượt, mỗi tổ hợp có một lượt thực thi. R01 sử dụng dữ liệu lịch sử ngày 02–03/08/2026; R02–R05 thuộc đợt bổ sung sau đó. Toàn bộ bảng và hình ở đây dùng đường đã chọn và trace của JSON thực thi, không trộn đường từ snapshot RViz với chỉ số của một trial khác. SHA-256 đường Raw được đối chiếu trong từng nhóm tuyến–planner để bảo đảm các phương án nhận cùng đầu vào hình học.

Ma trận này phục vụ phân tích mô tả theo kịch bản và theo planner. Năm planner không được xem như năm lần chạy lại cùng một thuật toán, và một lượt cho mỗi ô không đủ để ước lượng độ biến thiên thực thi của ô đó. Vì vậy, các bảng R01–R05 không kèm khoảng tin cậy giả tạo từ việc gộp các planner. Những khác biệt trung bình ở ma trận khảo sát là mô tả trên tập trường hợp đã thử, không phải kết luận xác suất cho các kho chưa quan sát.

Bộ kiểm chứng lặp được chạy riêng trên commit 1a1164e878bbe98e98cebefc009c4fe129174e9c, gồm Constrained và PSTMO, năm lần cho mỗi tổ hợp tuyến–planner: 5 × 5 × 2 × 5 = 250 lượt. Mỗi lượt khởi động stack sạch; bốn worker sử dụng ROS domain và Gazebo partition tách biệt. Pipeline không phát lộ seed ngẫu nhiên, do đó các repetition đo tính lặp lại thực thi với đầu vào cố định, không đại diện cho thử nghiệm nhiều seed hoặc nhiều mức nhiễu. Phân tích thống kê của bộ này được trình bày ở Mục III.H và không được gộp thành các lượt lặp của ma trận 125 lượt.

### 5) Tiêu chí hoàn thành và xử lý lỗi

Một lượt chỉ được ghi nhận đạt khi action điều hướng thành công, sai số vị trí ground truth không vượt 0,10 m, sai số hướng không vượt 0,15 rad và robot dừng theo tiêu chí vận tốc. Ngưỡng dừng là |v| ≤ 0,01 m/s và |ω| ≤ 0,02 rad/s, được kiểm tra qua ba vòng quan sát liên tiếp trong cửa sổ chờ tối đa 3 s. Ba vòng quan sát không mặc nhiên tương đương ba mẫu cảm biến độc lập. Ngưỡng hậu kiểm này được phân biệt với dung sai nội bộ của Nav2 goal checker trong Bảng II.

Lỗi khởi tạo hạ tầng được xử lý theo chính sách retry và lưu dấu vết riêng. Một lượt đã thực thi nhưng không đạt đích hoặc không dừng không được chạy lại để thay bằng kết quả thuận lợi. Bộ lặp có 14 lượt cần ít nhất một lần thử lại hạ tầng; các kết quả đã chốt và các lần bị gián đoạn được định danh trong manifest. Chính sách này bảo toàn mẫu số đánh giá thành công và giúp phân biệt không khởi động được hệ thống với không hoàn thành được nhiệm vụ.

Các lượt không đạt vẫn được hiển thị ở hình và bảng phân loại. Tuy nhiên, thời gian kết thúc sớm do abort không thể được diễn giải như thời gian hoàn thành ngắn. Với thống kê liên tục ghép cặp, chỉ cặp mà cả hai phương án đều đạt được sử dụng; số cặp được nêu cùng kết quả. Đây là một phân tích có điều kiện trên thành công, không phải cách chuyển các lượt không đạt thành dữ liệu bị bỏ quên.

### 6) Chỉ số đo và cách diễn giải

Thời gian thực thi T được đo bằng đồng hồ mô phỏng, gồm thực thi action và chờ robot dừng theo giao thức. Chỉ số smoothing_time được lấy từ smoothing_duration do action SmoothPath trả về. Nav2 Jazzy tính trường này bằng đồng hồ node; với use_sim_time, đây không phải phép đo CPU-time hoặc thời gian tường độc lập [4]. Sai số bám đường tại vị trí ground truth xₖ là khoảng cách nhỏ nhất đến các đoạn của đường tham chiếu đã chọn. Với N vị trí dùng trong tính toán, RMSE được xác định theo (8). Chỉ số này đo mức robot đi sát đường được giao; vì mỗi smoother có thể tạo một đường khác nhau, RMSE không phải khoảng cách của mọi robot đến cùng một đường chuẩn bên ngoài.

@equation e_{\mathrm{RMSE}}=\sqrt{\frac{1}{N}\sum_{k=1}^{N}\operatorname{dist}(x_k,P)^2}.

Năng lượng độ cong của đường rời rạc được xấp xỉ từ các bộ ba điểm liên tiếp. Nếu a, b, c là độ dài ba cạnh và A là diện tích có dấu của tam giác tương ứng, độ cong mẫu là 4A/(abc). Mỗi κ² được nhân với nửa tổng chiều dài hai cạnh kề để cộng vào Eκ. Bộ ba suy biến được bỏ qua theo ngưỡng số học. Cách tính này khiến Eκ nhạy với mật độ điểm và đoạn rất ngắn; vì vậy, Eκ của đường lập và Eκ của vết thực thi được báo riêng, không so sánh trực tiếp hai giá trị như thể cùng một phép đo vật lý.

Đường thực thi được lấy mẫu lại theo chiều dài cung với bước 0,05 m rồi làm trơn bằng trung bình trượt đối xứng bán kính hai mẫu, giữ hai đầu đường. Cùng quy trình này được áp dụng cho các phương án trước khi tính RMSE và E thực thi; do đó RMSE được báo trên đường ground truth đã xử lý, không phải trực tiếp mọi mẫu thời gian thô. Ngoài chỉ số trên toàn polyline, hồ sơ lưu chỉ số phần tịnh tiến để tránh gán tính trơn G² cho các pose pivot. Trong các bảng khảo sát, E là planned_curvature_energy, tức chỉ số rời rạc của đường đã lập; ở ba kết quả chính của bộ lặp, E thực thi là executed_curvature_energy. Việc nêu rõ tên đại lượng ngăn một mức giảm lớn của đường tham chiếu bị trình bày nhầm thành mức giảm tương đương của robot thực.

Khoảng hở c là giá trị nhỏ nhất của footprint đã định hướng đối với ô vật cản trên bản đồ tĩnh. Hậu kiểm nội suy pose với bước tịnh tiến 0,05 m và bước hướng 5°, lấy mẫu footprint với bước không lớn hơn 0,025 m, rồi sử dụng trường khoảng cách có hiệu chỉnh nửa đường chéo ô. Ô chưa biết và ô occupancy từ 65 trở lên được xử lý như vật cản. Do đó, c là một đại lượng dựa trên bản đồ rời rạc; số chữ số thập phân của thống kê không làm tăng độ phân giải vật lý của bản đồ.

Cuối cùng, nghiên cứu ghi sai số vị trí và hướng cuối, quãng đường thực thi, jerk tịnh tiến p95, thời gian smoother, số can thiệp Collision Monitor và số mẫu footprint va chạm. Jerk được suy từ chuỗi vận tốc ground truth, nên phụ thuộc nhịp lấy mẫu và phép sai phân. Không có phép đo điện năng, tải hàng thay đổi hoặc lực tiếp xúc thực nghiệm. Các chỉ số về độ trơn và chuyển động chỉ được dùng trong đúng nghĩa đo lường của chúng.

## B. R01 Chuyển lối tại giao cắt

### 1) Mục tiêu và hình học của tuyến

R01 đưa robot từ lối phía Nam ở x = −2 m sang lối phía Bắc ở x = 2 m. Tuyến buộc robot đổi hướng qua hành lang ngang trước khi tiếp tục dọc kệ. Đây là bài toán kiểm tra trực tiếp vai trò của hậu xử lý tại các vùng nối giữa đoạn gần thẳng và đoạn rẽ. Với đường dựa trên lưới, các bước nhỏ và điểm đổi hướng gần nhau có thể tạo nhiều cực trị độ cong rời rạc, dù hình dạng tổng thể vẫn theo cùng hành lang.

Hình 5 biểu diễn năm đường đã lập khi dùng ThetaStar, được chọn thống nhất làm ví dụ hình học cho cả năm tuyến. Vùng phóng to được xác định theo điểm trên PSTMO xa nhất so với các đỉnh Raw, không chọn riêng vùng có hình ảnh thuận lợi nhất cho từng phương pháp. Hình cho phép kiểm tra đường chuyển tiếp thực sự nằm ở đâu trong không gian kho; nó không thay thế đánh giá footprint hoặc bảng kết quả của bốn planner còn lại.

@figure r01_paths R01 với ThetaStar: (a) đường đã lập trên toàn tuyến; (b) vùng phóng to. Cùng màu và kiểu nét được dùng xuyên suốt các tuyến. Đây là đường tham chiếu, không phải vết ground truth của robot.

### 2) Kết quả thực thi và mức giảm độ cong

@table r01 Kết quả mô tả R01 trên năm planner

@note T và e là trung bình các lượt đạt; E và c lấy trên năm đường đã lập. E có đơn vị m⁻¹; c là trung bình của khoảng hở nhỏ nhất từng đường, không phải khoảng hở nhỏ nhất của cả nhóm. SG viết tắt Savitzky–Golay.

Toàn bộ 25 lượt R01 đạt tiêu chí hoàn thành. PSTMO có thời gian trung bình 39,74 s, thấp hơn Constrained 43,21 s, Simple 41,49 s, SG 42,88 s và Raw 43,50 s. RMSE của PSTMO là 21,9 mm so với 24,4 mm của Constrained. Vì tất cả phương án đều đạt ở R01, phép so sánh trung bình không bị thay đổi tập planner do lọc thành công. Tuy nhiên, mỗi số vẫn bắt nguồn từ một lượt cho từng planner, nên chưa mô tả độ phân tán của các lần thực thi cùng một đường vào.

E của PSTMO bằng 2,38 m⁻¹, trong khi Constrained đạt 18,62 m⁻¹ và Raw đạt 146,25 m⁻¹. Sự khác biệt cho thấy đường tham chiếu PSTMO ít tích lũy độ cong rời rạc hơn trong tập đường R01. Không thể chuyển tỷ lệ này thành phần trăm giảm rung cơ học: điều khiển, tiếp xúc bánh–sàn và hiệu chỉnh định vị có thể tạo đáp ứng không tỷ lệ với chỉ số đường lập. Hình 6 vì vậy trình bày đồng thời thời gian và RMSE của từng planner để nối kết quan sát hình học với hành vi vòng kín.

@figure r01_metrics R01: thời gian thực thi và RMSE của đủ năm phương án trên từng planner. Các cột là một lượt thực thi, không phải trung bình nhiều lần lặp; không vẽ thanh sai số.

@table r01_paired Chênh lệch PSTMO trừ Constrained ở R01

### 3) Đánh đổi khoảng hở và diễn giải

Khoảng hở nhỏ nhất trung bình của PSTMO là 15,7 cm, thấp hơn Constrained 23,5 cm. Như vậy, lợi ích thời gian và độ cong đi kèm việc sử dụng phần hành lang gần vật cản hơn theo thước đo footprint. Kết quả này phù hợp với khả năng đường chuyển tiếp uốn qua vùng giao cắt thay vì giữ khoảng cách như đối chứng, nhưng chỉ là diễn giải về hình học đã quan sát; dữ liệu chưa tách riêng tác động của từng trọng số mục tiêu.

Thời lượng SmoothPath trung bình của PSTMO ở R01 khoảng 0,102 s, so với 0,026 s của Constrained. Phần tăng nhỏ hơn chênh lệch thời gian thực thi quan sát được, nhưng không phải số đo CPU-time và không bao gồm toàn bộ độ trễ giao tiếp action. Vì vậy, không cộng trực tiếp hai đại lượng để tuyên bố mức giảm độ trễ đầu–cuối của hệ thống. Điểm quan trọng của R01 là đường trơn hơn đã đi kèm thực thi nhanh hơn trong tập planner, trong khi biên khoảng hở và thời lượng hậu xử lý cần được theo dõi như các đánh đổi độc lập.

## C. R02 Chuyển lối chéo dài toàn kho

### 1) Tác động của chiều dài và các đoạn thẳng

R02 nối (−4,7; −2,8) m với (4,7; 2,8) m, tạo bài toán chuyển qua phần rộng hơn của kho. So với R01, tỷ trọng thời gian chạy trên các đoạn tương đối thẳng tăng lên. Hậu xử lý tại góc chỉ tác động trực tiếp đến một phần của đường; do đó, mức giảm mạnh E không nhất thiết chuyển thành mức giảm tương ứng của tổng thời gian. Tuyến này kiểm tra liệu lợi ích của PSTMO có còn hiện diện khi các đoạn thẳng chiếm vai trò lớn hơn trong nhiệm vụ.

Hình 7 cho thấy các phương án cùng sử dụng hành lang của bài toán nhưng khác nhau tại vùng chuyển hướng. Việc chỉ nhìn toàn tuyến dễ làm các đường chồng nhau và che mất khác biệt; vùng phóng to cho thấy quy mô thay đổi trong hệ tọa độ mét. Đồng thời, hình vẫn giữ nền bản đồ để không tạo ấn tượng rằng một chuyển tiếp trơn có thể mở rộng tùy ý ra ngoài hành lang tự do.

@figure r02_paths R02 với ThetaStar: đường tham chiếu toàn tuyến và vùng phóng to theo cùng quy tắc chọn vùng như R01. Trục tọa độ có đơn vị mét; tỷ lệ hiển thị toàn tuyến và vùng phóng to khác nhau.

@table r02 Kết quả mô tả R02 trên năm planner

@note T và e lấy trên lượt đạt; E và c lấy trên đường đã lập. Cả 25 lượt R02 đều đạt. Các giá trị c là trung bình của cực tiểu từng đường.

### 2) Hiệu quả thời gian không đồng nghĩa cải thiện mọi chỉ số

PSTMO đạt thời gian trung bình 55,39 s, thấp hơn Constrained 59,06 s và Raw 59,57 s. Simple đạt 56,40 s, khá gần PSTMO ở chỉ số tổng thời gian. Điều này cho thấy một hậu xử lý đơn giản cũng có thể thu được phần đáng kể lợi ích thực thi trên tuyến dài. Vì vậy, đánh giá PSTMO chỉ với Raw sẽ phóng đại mức cải tiến so với các lựa chọn làm trơn đã có; Simple là đối chứng cần thiết trong ma trận khảo sát.

Trái với R01, RMSE trung bình của PSTMO ở R02 là 24,2 mm, cao hơn Constrained 21,7 mm, Simple 21,1 mm và SG 19,9 mm. Đường PSTMO có E thấp, 2,48 m⁻¹, nhưng điều đó không bảo đảm robot đi sát đường này hơn. Do RMSE được tính theo đường tham chiếu riêng của mỗi phương án, sự khác biệt có thể phản ánh mức độ phù hợp giữa hình dạng đường và bộ bám RPP cố định, ngoài bản thân độ trơn. Dữ liệu hiện có chưa cho phép phân rã chính xác các thành phần này.

@figure r02_metrics R02: thời gian và RMSE theo planner. Sự khác biệt giữa hai hàng chỉ số cho phép nhận diện trường hợp nhanh hơn nhưng bám đường kém hơn; không suy ra ưu thế toàn diện từ một hàng duy nhất.

@table r02_paired Chênh lệch PSTMO trừ Constrained ở R02

Bảng VIII giữ phép ghép cặp trong cùng planner thay vì lấy một đường thuận lợi của PSTMO so với một đường bất lợi của đối chứng. Dấu âm của ΔT biểu thị thời gian thấp hơn; dấu dương của Δe biểu thị RMSE cao hơn. Hai dấu này có thể đồng thời xuất hiện mà không mâu thuẫn, vì hai đại lượng trả lời hai câu hỏi khác nhau. Bảng cũng tránh gộp năm planner thành năm repetition của cùng một đầu vào.

### 3) Khoảng hở và ý nghĩa vận hành

Khoảng hở nhỏ nhất trung bình của PSTMO là 19,3 cm, so với 22,9 cm của Constrained. Đây là đánh đổi cùng chiều với R01, nhưng không được dùng để đặt một ngưỡng an toàn mới sau khi đã biết kết quả. Ngưỡng chấp nhận cần xuất phát từ footprint, sai số định vị và yêu cầu vận hành của hệ thống, không từ việc chọn mức thấp hơn tất cả quan sát hiện có. Trong phạm vi thử nghiệm, hậu kiểm không ghi nhận mẫu footprint va chạm; điều này không loại trừ rủi ro khi sai số định vị hoặc tải thay đổi.

R02 cho thấy vai trò của phân tích đa chỉ số: PSTMO giảm thời gian và E của đường lập, nhưng không dẫn đầu RMSE. Một quyết định triển khai có thể ưu tiên thời gian chu kỳ hoặc sai số bám đường tùy nhiệm vụ lấy hàng. Dữ liệu của tuyến này ủng hộ việc báo cáo đồng thời lợi ích và chi phí, thay vì xếp hạng phương án bằng một chỉ số tổng hợp có trọng số được chọn sau thí nghiệm.

## D. R03 Chuyển lối hình chữ U phía Nam

### 1) Chuỗi góc liên tiếp và khả năng tương thích

R03 bắt đầu ở (−2,0; −2,8) m và kết thúc tại (2,0; −2,8) m, với hướng đích quay về phía Nam. Robot phải đi vào hệ hành lang rồi trở lại hàng ngang xuất phát ở lối khác. Hình dạng chữ U làm nổi bật bài toán phối hợp giữa các góc: lựa chọn khoảng cắt rộng tại một góc có thể giảm chiều dài dành cho góc tiếp theo. Đây là tình huống mà điều kiện tương thích đoạn chung trong DP có ý nghĩa thiết kế rõ ràng.

Tuy nhiên, quan sát R03 không phải thí nghiệm loại bỏ DP. Tất cả trial PSTMO trong bộ hiện tại sử dụng pipeline đầy đủ; không có cặp chạy cùng commit chỉ khác bật hoặc tắt DP. Vì vậy, kết quả được dùng để đánh giá toàn phương pháp trên chuỗi rẽ liên tiếp, còn phát biểu riêng về mức đóng góp nhân quả của DP cần một ablation kiểm soát. Phân biệt này quan trọng để mô tả đúng đóng góp thuật toán mà không gán mọi cải thiện cho một khối xử lý duy nhất.

@figure r03_paths R03 với ThetaStar: đường chữ U và vùng chuyển hướng được phóng to. Hình biểu diễn một planner cố định; trường hợp không đạt ở tuyến này thuộc SmacHybrid và được báo riêng.

@table r03 Kết quả mô tả R03 trên năm planner

@note PSTMO có 4/5 lượt đạt; các phương án còn lại có 5/5. T và e của PSTMO chỉ lấy trên bốn lượt đạt. E và c vẫn lấy trên đủ năm đường; không diễn giải chênh lệch T giữa các trung bình này như một ước lượng ghép cặp.

### 2) Kết quả thuận lợi và một trường hợp không đạt

Trên các lượt đạt, PSTMO có thời gian trung bình 41,94 s, so với 44,15 s của Constrained và 42,34 s của Simple. Do PSTMO thiếu một lượt trong tập thành công, các giá trị này không dùng cùng tập planner. Bảng X cung cấp chênh lệch trên các cặp cùng đạt; ô không đủ điều kiện được để dấu gạch, không thay bằng số không. Đây là cách tránh đưa thất bại thuật toán thành lợi thế trung bình bằng cách chỉ giữ các trường hợp thuận lợi.

Lượt PSTMO/SmacHybrid có action thành công nhưng sai số vị trí cuối 0,102385 m, vượt ngưỡng 0,100000 m. Theo giao thức đã dùng, lượt này được giữ là không đạt dù mức vượt chỉ khoảng 2,385 mm. Không có cơ sở đổi ngưỡng riêng cho trial này hoặc coi nó như một lượt hoàn thành chỉ vì đường nhìn trực quan hợp lý. Ngược lại, giá trị vượt nhỏ cho thấy tiêu chí nhị phân gần ngưỡng cần được đọc cùng sai số liên tục để hiểu quy mô vấn đề.

@figure r03_metrics R03: cột gạch và dấu × đánh dấu PSTMO/SmacHybrid không đạt. Giá trị ghi ở cột đó chỉ mô tả trace kết thúc; không được tính vào trung bình hoàn thành hoặc dùng để kết luận thời gian tốt hơn.

@table r03_paired Chênh lệch ghép cặp trên các lượt cùng đạt ở R03

### 3) So sánh với Simple và giới hạn của chỉ số đường lập

E trung bình của PSTMO là 5,47 m⁻¹, gần Simple 5,78 m⁻¹ và thấp hơn Constrained 22,93 m⁻¹. So với các tuyến khác, khoảng cách giữa PSTMO và Simple ở chỉ số này nhỏ hơn rõ rệt. Kết quả cho thấy lợi ích hình học của cơ chế chuyển tiếp phụ thuộc cấu trúc đường vào; không thể giả định PSTMO tạo cùng một hệ số cải thiện trên mọi dạng tuyến. RMSE của các lượt PSTMO đạt là 27,2 mm, cũng gần Simple 27,0 mm.

Khoảng hở nhỏ nhất trung bình đạt 18,8 cm với PSTMO và 21,4 cm với Constrained. Bên cạnh đánh đổi này, R03 nhấn mạnh rằng bảo toàn điểm cuối của đường lập không đồng nghĩa robot dừng chính xác tại điểm cuối. Sai số động học, bám đường và quá trình dừng nằm sau bước tạo path. Do đó, một kiểm tra invariant hình học thành công không được dùng để phủ nhận thất bại theo ground truth của hệ điều hướng.

R03 cung cấp một trường hợp kiểm tra khắt khe hơn cho cách trình bày kết quả: phương pháp có thể tạo đường có E nhỏ và vẫn không đạt một nhiệm vụ trong ma trận. Kết luận phù hợp là PSTMO có chất lượng đường tham chiếu thuận lợi trên tuyến chữ U nhưng chưa chứng minh ưu thế về xác suất hoàn thành. Bằng chứng lặp ở Mục III.H được sử dụng để xem xét độ ổn định thực thi, không để thay thế hoặc xóa trial không thuận lợi của R03.

## E. R04 Chuyển lối chéo ngược

### 1) Đổi chiều tiếp cận và vai trò của pose đích

R04 đưa robot từ (−4,6; 2,8) m ở phía Tây Bắc đến (2,0; −2,8) m ở phía Nam. Tuyến đổi hướng tiếp cận so với R02 và kết thúc với hướng −π/2. Mục tiêu là kiểm tra hậu xử lý khi trình tự đi vào và rời hành lang ngang thay đổi, không phải chỉ lặp lại một đường chéo bằng cách đảo tên điểm đầu–cuối. Các vùng chuyển tiếp cần đáp ứng hướng tiếp tuyến và khoảng trống của hành lang theo chiều di chuyển thực tế.

Hình 11 giữ cùng quy ước màu và chọn ThetaStar làm ví dụ, trong khi các kết quả của NavFn và Smac được thể hiện trong bảng và Hình 12. Nhờ đó, khác biệt hình học của một planner có thể được đối chiếu với toàn ma trận mà không ngầm coi đường vẽ là đại diện trung bình cho mọi planner. Các đường gần nhau trong hình toàn tuyến vẫn có thể tạo chỉ số bám đường và thời gian khác nhau do hình dạng tại vùng rẽ.

@figure r04_paths R04 với ThetaStar: toàn tuyến chéo ngược và vùng phóng to. Việc đảo chiều tiếp cận không làm tuyến này trở thành bản sao đối xứng của R02.

@table r04 Kết quả mô tả R04 trên năm planner

@note Raw có 4/5 lượt đạt; bốn phương án hậu xử lý có 5/5. T và e của Raw dùng bốn lượt đạt, còn E và c vẫn dùng đủ năm đường. So sánh PSTMO–Constrained không bị thiếu cặp ở R04.

### 2) Kết quả thời gian và bám đường

PSTMO đạt 5/5 lượt ở R04, với thời gian trung bình 47,71 s và RMSE 24,4 mm. Constrained đạt 50,83 s và 30,9 mm; Simple đạt 49,06 s và 28,4 mm. Trong tập năm planner của tuyến này, PSTMO có cả thời gian trung bình và RMSE trung bình thấp hơn các đối chứng làm trơn được khảo sát. Khác với R02, hai chỉ số vòng kín biến đổi cùng chiều thuận lợi ở mức trung bình tuyến.

E đường lập của PSTMO là 2,89 m⁻¹, so với 5,99 m⁻¹ của Simple và 28,31 m⁻¹ của Constrained. Khoảng hở nhỏ nhất trung bình của PSTMO là 19,3 cm, vẫn thấp hơn Constrained 21,6 cm. Như vậy, ngay ở tuyến có kết quả thời gian và RMSE thuận lợi, không phải mọi chỉ số đều cải thiện. Khoảng hở là một đại lượng cần bảo toàn trong báo cáo, không thể bỏ vì không cùng chiều với kết luận chính.

@figure r04_metrics R04: thời gian và RMSE trên từng planner. Raw/ThetaStar được đánh dấu không đạt do hậu kiểm đích; các đường đã làm trơn đều hoàn thành trong ma trận một lượt này.

@table r04_paired Chênh lệch PSTMO trừ Constrained ở R04

### 3) Trường hợp sát ngưỡng và ý nghĩa của hậu kiểm

Raw/ThetaStar có sai số vị trí cuối 0,100234 m, vượt ngưỡng 0,10 m khoảng 0,234 mm. Lượt này vẫn được phân loại không đạt theo cùng quy tắc với PSTMO/SmacHybrid ở R03. Áp dụng đối xứng tiêu chí cho cả Raw và PSTMO là cần thiết: nếu chỉ giữ ngưỡng nghiêm ngặt cho đối chứng mà nới cho phương pháp đề xuất, so sánh thành công sẽ bị thiên lệch.

Mức vượt rất nhỏ không chứng minh một khác biệt cơ học đáng kể giữa hai trial lân cận ngưỡng; nó chỉ xác định kết quả của bộ tiêu chí đang dùng. Bởi vậy, bài báo trình bày cả nhãn đạt/không đạt và sai số liên tục chính xác hơn cho các trường hợp này. Không thực hiện điều chỉnh hậu nghiệm để biến 123/125 thành 125/125 ở ma trận khảo sát. Tỷ lệ hoàn thành phải gắn với định nghĩa hoàn thành cụ thể.

R04 cũng cho thấy cần tách bảo đảm của smoother với kết quả của bộ điều khiển. Đường Raw và đường hậu xử lý khác nhau nhưng robot vẫn đi qua cùng hệ hành lang; sai số kết thúc nhỏ có thể phát sinh ở giai đoạn tiếp cận và dừng hơn là ở vùng rẽ chính. Dữ liệu hiện có hỗ trợ mô tả hiện tượng, chưa đủ để quy nguyên nhân sát ngưỡng cho một thành phần điều khiển riêng. Phần thảo luận do đó tránh khẳng định rằng chỉ riêng tính G² đã giải quyết mọi sai số đích.

## F. R05 Rẽ từ hành lang ngang vào lối lấy hàng

### 1) Nhiệm vụ ngắn với chuyển hướng tập trung

R05 bắt đầu tại (−5,0; 0,0) m với hướng dọc hành lang ngang và kết thúc tại (0,0; 3,0) m trong lối lấy hàng, hướng π/2. Thay vì phân bố nhiều thay đổi hướng trên một đường dài, tuyến tập trung vào chuyển từ di chuyển ngang sang đi dọc và tiếp cận vị trí cuối. Vì thế, thời gian tổng của tuyến không chỉ phụ thuộc chiều dài mà còn phụ thuộc cách thực hiện đoạn rẽ và quá trình giảm tốc đến đích.

Hình 13 cho thấy các đường tham chiếu và vùng có khác biệt hình học lớn nhất theo quy tắc đã xác định. Việc giữ cùng một planner minh họa giúp so sánh cách mỗi smoother xử lý cấu trúc vào lối. Tuy nhiên, hình từ một lượt đạt không cung cấp bằng chứng về xác suất hoàn thành khi lặp lại. Điều này đặc biệt quan trọng ở R05, vì bộ kiểm chứng lặp sau đó ghi nhận một số lượt abort trên chính tuyến này.

@figure r05_paths R05 với ThetaStar: rẽ từ hành lang ngang vào lối lấy hàng. Hình sử dụng ma trận khảo sát một lượt; các thất bại của bộ lặp được báo riêng ở Mục III.H.

@table r05 Kết quả mô tả R05 trên năm planner

@note Cả 25 lượt của ma trận khảo sát R05 đều đạt. E và c là chỉ số đường đã lập; T và e là chỉ số thực thi. Kết quả này không được thay thế cho tỷ lệ thành công của bộ lặp.

### 2) So sánh thời gian và chất lượng đường

Trong ma trận khảo sát, PSTMO đạt thời gian trung bình 37,69 s, thấp hơn Constrained 39,33 s và Raw 40,27 s. Simple đạt 38,25 s, nên lợi thế của PSTMO so với đối chứng đơn giản này nhỏ hơn lợi thế so với Raw. RMSE của PSTMO là 20,8 mm, thấp hơn Constrained 24,4 mm và gần Raw 21,2 mm. Raw có sai số bám tương đối thấp không có nghĩa đường Raw trơn hơn; độ khó của đường tham chiếu và cách robot đi theo đường cùng ảnh hưởng RMSE.

PSTMO có E đường lập 2,44 m⁻¹, so với Constrained 25,35 m⁻¹ và Raw 141,50 m⁻¹. Tuy vậy, độ giảm E lớn hơn rất nhiều độ giảm thời gian theo tỷ lệ. Đây là bằng chứng trực quan cho việc không nên dùng E như một biến thay thế hoàn hảo cho hiệu năng nhiệm vụ. Robot không thực hiện chính xác một hồ sơ vận tốc tối ưu hóa toàn hành trình theo E; nó nhận đường hình học rồi phản hồi qua RPP và các tầng giới hạn vận tốc chung.

@figure r05_metrics R05: kết quả một lượt cho từng tổ hợp planner–phương án. Tất cả các lượt trong hình đều đạt, nhưng điều đó chưa mô tả tính lặp lại của nhiệm vụ.

@table r05_paired Chênh lệch PSTMO trừ Constrained ở R05

### 3) Khoảng hở và chuyển tiếp sang kiểm chứng lặp

Khoảng hở nhỏ nhất trung bình của PSTMO ở R05 là 20,8 cm, tương đối gần Constrained 22,2 cm. Thời lượng SmoothPath của PSTMO khoảng 0,077 s, so với khoảng 0,022 s của Constrained. Đây là số đo theo đồng hồ node trong môi trường thử nghiệm, không phải thời gian CPU hay thời gian thực thi xấu nhất của thuật toán. Các cấu hình dùng nhiều worker và nhịp cập nhật đồng hồ mô phỏng còn giới hạn việc diễn giải các thời lượng rất ngắn.

Kết quả R05 minh họa lý do cần hai tầng đánh giá. Ma trận một lượt cho thấy phương pháp hoạt động được trên mọi planner của tuyến và có các trung bình thuận lợi. Tuy nhiên, 25 lượt đạt không chứng minh rằng mọi lần chạy lại đều đạt, càng không chứng minh độ bền vững dưới nhiễu. Bộ lặp có năm lần cho từng cặp PSTMO–Constrained được sử dụng để kiểm tra khoảng trống bằng chứng này, đồng thời giữ các trường hợp abort trong mẫu số.

Về ý nghĩa vận hành, một nhiệm vụ ngắn có thời gian trung bình tốt vẫn có thể bị ảnh hưởng lớn bởi một lần không hoàn thành. Vì vậy, đánh giá lựa chọn smoother cho tác vụ lấy hàng phải đặt xác suất hoàn thành bên cạnh thời gian có điều kiện trên thành công. Không thể lấy phần tiết kiệm vài giây ở các lượt đạt để bù cho một thất bại bằng một phép cộng tùy ý, khi chi phí phục hồi nhiệm vụ chưa được đo trong nghiên cứu.

## G. Tổng hợp theo tuyến và planner

### 1) Kết quả chung của ma trận năm phương án

Ma trận khảo sát có 123/125 lượt đạt. Raw và PSTMO cùng đạt 24/25; Simple, SG và Constrained đạt 25/25. Hai lượt không đạt đã được định danh ở R03 và R04, đều do vượt dung sai vị trí sau action thành công. Không ghi nhận can thiệp Collision Monitor hoặc mẫu footprint va chạm trong bộ này. Các số đếm cuối chỉ áp dụng cho kênh giám sát và phép hậu kiểm đã dùng, không phải bằng chứng tuyệt đối không có rủi ro va chạm trong mọi điều kiện.

@table pooled Tổng hợp mô tả ma trận 125 lượt

@note T và e dùng các lượt đạt của từng phương án, nên Raw/PSTMO có 24 lượt còn các phương án khác có 25. E dùng đủ 25 đường mỗi phương án. Không dùng trực tiếp hiệu hai trung bình khác mẫu để thay thế chênh lệch ghép cặp.

PSTMO có thời gian trung bình thấp nhất trong từng tuyến của ma trận, nhưng R03 dùng bốn lượt đạt thay vì năm. Phát biểu này là xếp hạng mô tả của các trung bình đã xác định, không phải bằng chứng rằng PSTMO nhanh nhất trong mọi ô tuyến–planner. Simple thường là đối chứng gần PSTMO hơn về thời gian so với Raw hoặc Constrained. Điều đó củng cố nhu cầu giữ cả phương án đơn giản trong bài báo, dù bộ kiểm chứng lặp tập trung vào một đối chứng tối ưu số.

Đối với RMSE, xu hướng không đồng nhất giữa các tuyến. R02 cho thấy PSTMO không dẫn đầu, trong khi R04 có trung bình thuận lợi rõ hơn. Một số ô cụ thể có thể đổi chiều ngay cả khi trung bình tuyến giảm. Do vậy, bản đồ chênh lệch trong Hình 15 trình bày riêng ΔT và Δe cho từng cặp PSTMO–Constrained. Cách trình bày này cho thấy độ không đồng nhất mà một bảng trung bình toàn bộ dễ che khuất.

@figure matrix_delta Chênh lệch PSTMO − Constrained trong ma trận khảo sát. Mỗi ô là một cặp cùng tuyến và planner; màu xanh biểu thị chênh lệch âm. Dấu gạch là cặp không cùng đạt, không phải hiệu bằng không. Các số là một lượt, không kèm suy luận thống kê.

### 2) Vì sao không chỉ xếp hạng bằng năng lượng độ cong

PSTMO tạo E đường lập thấp hơn nhiều đối chứng trong các bảng tuyến, nhưng quy mô chênh lệch chịu ảnh hưởng cấu trúc điểm của đường. Đường lưới có nhiều đoạn ngắn có thể tích lũy E lớn, còn hậu xử lý thay đổi vị trí và mật độ điểm. Điều này không làm chỉ số vô nghĩa: E vẫn mô tả mức uốn theo phép tính được công bố. Tuy nhiên, nó đòi hỏi tránh so sánh tỷ lệ giữa các đại lượng được đo trên các loại đường khác nhau mà không nêu cách rời rạc hóa.

Ba tầng kết quả cần được phân biệt. Tầng đầu là hình dạng và tính khả thi của đường tham chiếu. Tầng thứ hai là khả năng bám đường và chuyển động của robot theo ground truth. Tầng thứ ba là hoàn thành nhiệm vụ với dung sai và trạng thái dừng. Một phương pháp có thể cải thiện tầng đầu nhưng không cải thiện tầng thứ hai, như RMSE trung bình ở R02, hoặc không đạt tầng thứ ba ở một trial, như PSTMO/SmacHybrid tại R03. Vì vậy, đánh giá chỉ trên path sẽ bỏ sót thông tin quyết định đối với triển khai.

Hình 16 bổ sung một đối chiếu giữa path và ground truth cho R02/ThetaStar. Khác với các hình đường tham chiếu trước đó, nét liền ở hình trên là vị trí thực thu từ mô phỏng; đồ thị phía dưới là vận tốc góc theo thời gian của cùng hai trial. Ví dụ này giữ nguyên tín hiệu, không làm mượt đường vận tốc để tăng cảm giác ổn định. Nó cho thấy loại bằng chứng cần dùng khi liên hệ thay đổi hình học với đáp ứng điều khiển; việc xác định nguyên nhân của từng dao động vẫn cần phân tích lệnh điều khiển và trạng thái định vị đồng bộ.

@figure execution_trace R02/ThetaStar trong ma trận khảo sát: trên là path (nét đứt) và ground truth (GT, nét liền) tại vùng rẽ; dưới là vận tốc góc ground truth theo thời gian tương đối từ trace. Mỗi phương án có mốc thời gian riêng; không căn chỉnh các đỉnh tín hiệu hậu nghiệm.

Sự phân biệt này cũng giải thích tại sao bài báo không gọi E là “năng lượng vận hành”. Không có dòng điện, điện áp, mô hình hiệu suất truyền động hoặc đo công suất trong bộ dữ liệu. Từ một đường có E thấp hơn không thể suy ra robot tiết kiệm pin cùng tỷ lệ. Tương tự, jerk p95 thấp hơn không đủ để kết luận hàng hóa rung ít hơn nếu chưa đo gia tốc tại tải hoặc mô hình liên kết cơ khí tương ứng.

### 3) Tính công bằng của đối chứng và phạm vi kết luận

Các phương án được ghép cặp theo đường Raw chứ không chỉ theo tọa độ đầu–cuối. Đây là kiểm soát quan trọng vì hai lần planner chạy có thể tạo đường khác nhau ngay cả với cùng bài toán. Cùng đầu vào, cùng controller và cùng bản đồ làm rõ phần khác biệt do chuỗi hậu xử lý và tương tác thực thi của nó. Dù vậy, quy trình không loại bỏ mọi nguồn sai lệch: cấu hình tham số của từng phương án không được tối ưu với một ngân sách tìm kiếm tham số đồng đều trong nghiên cứu này.

Constrained được chọn cho bộ lặp vì là một đối chứng tối ưu số có thể triển khai trực tiếp trong cùng hệ Nav2, khác về cơ chế so với lựa chọn trạng thái góc của PSTMO. Đây là lý do thiết kế kiểm chứng tập trung, không phải chứng minh Constrained là đối chứng mạnh nhất trên tất cả chỉ số. Ma trận thực tế cho thấy Simple cạnh tranh về thời gian ở nhiều tuyến. Vì chưa lặp đủ năm phương án, nghiên cứu không đưa ra kết luận thống kê rằng PSTMO vượt Simple hoặc SG về tổng quát.

Hơn nữa, Constrained trong cấu hình thử nghiệm không bật thành phần độ cong; PSTMO và Constrained không có cùng phép bảo đảm footprint nội bộ. Kết quả do đó được diễn đạt là so sánh các cấu hình cụ thể được công bố. Muốn kết luận về ưu thế của hai họ phương pháp sau tinh chỉnh tham số, cần thêm giao thức tối ưu tham số công bằng, các cấu hình Constrained khác và tập tuyến đánh giá tách biệt với tập dùng để chọn tham số.

## H. Kiểm chứng lặp PSTMO và Constrained

### 1) Ghép cặp và đơn vị phân tích

Bộ kiểm chứng có 250 lượt và 125 cặp PSTMO–Constrained cùng tuyến, planner và số lần lặp. Toàn bộ 125 cặp có Raw-path SHA-256 trùng khớp; kiểm tra metadata không phát hiện vấn đề audit. Constrained đạt 122/125 lượt và PSTMO đạt 121/125 lượt, tổng cộng 243/250. Có 118 cặp mà cả hai đều đạt; chỉ các cặp này đóng góp vào phân tích chỉ số liên tục. Bảy cặp còn lại vẫn tham gia phân tích thành công và được liệt kê theo nguyên nhân kết thúc.

Đặt b là một khối tuyến–planner và Rb là tập repetition mà cả hai phương án đạt trong khối đó. Chênh lệch khối db được tính bằng trung bình các hiệu PSTMO trừ Constrained trên Rb. Hiệu ứng chung Δ là trung bình đều trên 25 khối, như (9). Vì số cặp còn lại có thể khác nhau giữa các khối, hiệu ứng này không nhất thiết bằng hiệu của hai trung bình gộp trên 118 cặp.

@equation d_b=\frac{1}{\left|R_b\right|}\sum_{r\in R_b}(y_{P,br}-y_{C,br}), || \Delta=\frac{1}{25}\sum_{b=1}^{25}d_b.

Phân tích dùng khối thay cho coi mọi repetition là quan sát độc lập của một quần thể đường đi. Năm lần chạy cùng đầu vào chủ yếu phản ánh biến thiên thực thi, không tạo năm bài toán hình học mới. Tuy vậy, 25 khối vẫn dùng chung năm tuyến và năm planner trên một bản đồ; giả định các hiệu ứng khối có thể trao đổi khi bootstrap cần được hiểu như một quy ước phân tích có điều kiện. Khoảng tin cậy không bao phủ tự động sự thay đổi sang bản đồ, robot hoặc bộ điều khiển khác.

### 2) Phương pháp thống kê và ba chỉ số chính

Ba chỉ số chính của phân tích là thời gian thực thi, RMSE bám đường và E của vết thực thi. Khoảng tin cậy 95% của Δ được ước lượng bằng bootstrap lấy mẫu lại 25 khối, 20.000 lần, dùng seed tính toán cố định bắt đầu từ 20261011 và dịch theo chỉ số. Đây là seed của phép phân tích thống kê, không phải seed của mô phỏng robot. Bootstrap sử dụng phân vị của phân bố trung bình tái lấy mẫu [5].

Kiểm định Wilcoxon signed-rank hai phía được áp dụng cho các chênh lệch khối [6], loại các hiệu có trị tuyệt đối không quá 10⁻¹² theo quy tắc số học của script. Ba p-value chính được hiệu chỉnh Holm [7]. Kiểm định đánh giá xu hướng chênh lệch theo giả định của thủ tục; khoảng tin cậy bootstrap mô tả trung bình khối. Hai công cụ có mục tiêu thống kê liên quan nhưng không đồng nhất, nên p-value không được trình bày như xác suất phương pháp đề xuất tốt hơn.

@table means Trung bình có điều kiện và số khối giảm

@note Hai cột phương án là trung bình gộp 118 cặp cùng đạt. “Khối giảm” đếm số db < 0 trên 25 khối. Trung bình gộp không phải cơ sở số học trực tiếp để tái tính Δ trong Bảng XVII khi số cặp mỗi khối khác nhau.

@table primary Hiệu ứng trung bình khối và bất định

@note Δ = PSTMO − Constrained, trung bình đều trên 25 khối. CI dùng bootstrap khối; p Holm hiệu chỉnh đúng ba chỉ số chính. Các phép suy luận có điều kiện trên các cặp cùng đạt và tập tuyến–planner đã thử.

PSTMO giảm thời gian trung bình theo khối 2,857 s, với CI 95% [−3,716; −2,039] s và p Holm = 1,19 × 10⁻⁶. Có 24/25 khối có ΔT âm. RMSE giảm 2,985 mm, CI [−4,487; −1,506] mm, p Holm = 4,89 × 10⁻⁴; số khối giảm là 19/25. E thực thi giảm 1,038 m⁻¹, CI [−1,332; −0,767] m⁻¹, p Holm = 5,36 × 10⁻⁷, với 24/25 khối giảm. Như vậy, lợi thế không chỉ xuất hiện ở E của đường đã lập mà còn có bằng chứng trên vết chuyển động thực thi, trong phạm vi các cặp hoàn thành.

@figure repeat_blocks Hiệu ứng của 25 khối trong bộ lặp. Trong mỗi nhóm tuyến, thứ tự là NavFnAStar, NavFnDijkstra, ThetaStar, Smac2D và SmacHybrid. Mỗi điểm là trung bình chênh lệch trên các repetition cùng đạt; đường đứt là trung bình đều của 25 điểm.

Hình 17 bổ sung phân bố hiệu ứng để tránh chỉ dựa vào p-value nhỏ. RMSE có nhiều khối đổi chiều hơn hai chỉ số còn lại; điều này nhất quán với việc ma trận khảo sát đã chỉ ra đánh đổi bám đường ở một số trường hợp. Không phải mọi điểm nằm dưới đường không, và mức cải thiện giữa các khối khác nhau. Do đó, kết luận thích hợp là xu hướng cải thiện có điều kiện trên tập đánh giá, không phải tính trội tuyệt đối cho mọi đầu vào.

@table by_planner Hiệu ứng mô tả theo từng planner trong bộ lặp

@note Mỗi hàng là trung bình đều của năm hiệu ứng khối thuộc cùng planner, không phải một kiểm định mới. ΔE thực thi có đơn vị m⁻¹. Không sử dụng bảng này để suy ra thứ hạng planner trên các bài toán ngoài năm tuyến đã thử.

Bảng XVIII cho thấy quy mô hiệu ứng phụ thuộc planner đầu vào. Cả năm trung bình ΔT đều âm, nhưng mức giảm không đồng đều; một trung bình theo planner cũng có thể che khối đổi chiều ở một tuyến cụ thể. Với RMSE, các hiệu ứng theo planner cần được đọc cùng Hình 17 để phân biệt tính nhất quán trong từng nhóm với hiệu ứng trung bình chung. Bảng không tăng cỡ mẫu suy luận từ 25 khối và không biến năm tuyến dùng chung bản đồ thành năm môi trường độc lập.

### 3) Chỉ số phụ và chi phí của cải thiện

@table secondary Hiệu ứng phụ trong bộ lặp

@note Các CI được báo như phân tích bổ sung; không mở rộng họ ba kiểm định chính để tuyên bố xác nhận thống kê cho mọi chỉ số phụ. Dấu âm thuận lợi cho E, jerk, quãng đường và thời gian xử lý, nhưng bất lợi cho khoảng hở.

E đường lập giảm trung bình theo khối 23,202 m⁻¹ và quãng đường thực thi giảm 0,089 m. Jerk tịnh tiến p95 giảm 0,078 m/s³, nhỏ hơn nhiều về quy mô tương đối so với mức giảm E đường lập. Điều này tiếp tục cho thấy đáp ứng vật lý của robot không tỷ lệ đơn giản với độ trơn hình học. Các chỉ số phụ hỗ trợ mô tả cơ chế khả dĩ nhưng không thay thế phép đo tải hàng, rung chassis hoặc điện năng.

Ba đánh đổi cần được giữ nguyên trong diễn giải. Sai số vị trí cuối tăng 3,380 mm theo trung bình khối; khoảng hở nhỏ nhất giảm 35,161 mm; thời lượng SmoothPath tăng 71,220 ms. Khoảng hở giảm ở 18 khối, tăng ở 5 khối và bằng nhau ở 2 khối. Sai số đích tăng ở 18/25 khối, còn thời lượng SmoothPath tăng ở cả 25 khối. Số đo cuối không phải benchmark CPU. Vì vậy, dữ liệu không hỗ trợ phát biểu rằng PSTMO đồng thời tốt hơn về thời gian, độ chính xác đích, khoảng hở và mọi thành phần độ trễ.

@figure tradeoffs Đánh đổi trong bộ lặp, mỗi điểm là một khối tuyến–planner. Trên: chênh lệch E đường lập và khoảng hở; dưới: chênh lệch sai số đích và thời gian thực thi. Mọi chênh lệch đều là PSTMO − Constrained.

Hình 18 cho phép nhận diện tổ hợp chỉ số có ý nghĩa vận hành: một điểm ở phía thời gian giảm nhưng sai số đích tăng không phải lỗi biểu đồ mà là một đánh đổi thực nghiệm. Tương tự, E thấp hơn đi kèm khoảng hở nhỏ hơn có thể chấp nhận hoặc không tùy yêu cầu an toàn của ứng dụng. Nghiên cứu không gán sẵn một hệ số quy đổi để cộng các đại lượng khác đơn vị này thành điểm số triển khai duy nhất.

### 4) Thành công nhiệm vụ và các lượt không đạt

Trong 125 cặp, có 4 cặp chỉ Constrained đạt và 3 cặp chỉ PSTMO đạt. Kiểm định McNemar exact hai phía cho p = 1,0. Kết quả không cung cấp bằng chứng khác biệt tỷ lệ thành công ở kích thước mẫu hiện tại, nhưng không chứng minh tương đương hoặc không kém hơn. Nghiên cứu chưa xác định biên non-inferiority và chưa thiết kế cỡ mẫu cho mục tiêu đó; vì vậy, không dùng p lớn như một bảo đảm hai phương án có cùng độ tin cậy.

@table failures Bảy lượt không đạt của bộ kiểm chứng lặp

@note P: PSTMO; C: Constrained. “Vượt xy” là action thành công nhưng sai số vị trí cuối vượt 0,10 m. Abort 102 được giữ theo mã ghi trong dữ liệu; bảng không quy nguyên nhân vật lý khi chưa có phân tích sự kiện chuyên biệt.

Năm lượt abort xuất hiện tại R05, gồm ba lượt PSTMO và hai lượt Constrained. Hai lượt còn lại ở R04 có sai số đích lần lượt 0,101583 m và 0,100358 m. Những trường hợp abort có sai số đích lớn không thể được gộp với trường hợp sát ngưỡng để diễn giải như cùng một cơ chế thất bại. Bảng XX giữ cả phương án, planner, repetition và trạng thái để phép kiểm tra có thể truy ngược về đúng trace.

@figure endpoints Sai số vị trí cuối của toàn bộ 250 lượt, bao gồm không đạt. Trục đứng dùng thang log để đồng thời hiển thị sai số cỡ centimét và các abort còn cách đích xa. Ngưỡng vị trí 0,10 m chỉ là một phần của tiêu chí hoàn thành.

Hình 19 thể hiện vì sao chỉ báo trung bình có điều kiện chưa đủ cho độ tin cậy nhiệm vụ. Phần lớn điểm nằm dưới ngưỡng vị trí, nhưng một số lượt kết thúc khi còn cách đích đáng kể. Thời gian trung bình của 118 cặp cùng đạt không tính chi phí phục hồi những lượt này. Nếu ứng dụng cần thời gian chu kỳ bao gồm phục hồi, cần định nghĩa một chính sách phục hồi chung và đo lại thời gian toàn nhiệm vụ, thay vì tự gán một giá trị phạt cho dữ liệu hiện tại.

## I. Thảo luận đóng góp và giới hạn áp dụng

### 1) Bằng chứng cho cơ chế hậu xử lý

Kết quả cho thấy PSTMO có thể chuyển các đường toàn cục từ nhiều planner thành đường tham chiếu có E thấp và thực thi được trong cùng chuỗi Nav2. Điểm thiết kế chính là kết hợp chuyển tiếp Bézier G² với trạng thái pivot, lựa chọn ứng viên dưới giới hạn chuyển động và giải quyết tương thích giữa các góc bằng DP. Khác với việc chỉ làm mượt tọa độ từng điểm, phương pháp biểu diễn trực tiếp các lựa chọn tại góc và ràng buộc chia sẻ chiều dài giữa chúng.

Diagnostics của bộ lặp xác nhận 125/125 lượt PSTMO sử dụng condition_only và hierarchical_alpha_two_trim, cùng kiểm tra invariant cuối. Tổng cộng có 2.005 trạng thái DP và 3.165 cạnh tương thích được ghi nhận. Các số đếm chứng minh nhánh triển khai dự kiến đã được thực thi trong bộ dữ liệu, nhưng không tự chứng minh DP tạo ra toàn bộ lợi ích đo được. Số trạng thái nhỏ hoặc invariant thành công cũng không tương đương một chứng minh thời gian thực xấu nhất cho mọi đường đầu vào.

Ở cấp độ hệ thống, ba chỉ số chính của bộ lặp cùng có Δ trung bình âm và CI không chứa không. Điều này hỗ trợ nhận định rằng lợi ích của PSTMO không chỉ dừng ở hình vẽ đường đi. Tuy nhiên, nghiên cứu chỉ đánh giá phản ứng của một cấu hình RPP với các đường được đưa vào. Một controller khác, hoặc cùng RPP nhưng bật điều tiết vận tốc theo độ cong và chi phí, có thể tạo thứ hạng và quy mô hiệu ứng khác. Tính tách rời giao diện path giúp tích hợp thuận tiện, nhưng không tạo tính độc lập về hiệu năng đối với controller.

### 2) Tính trơn hình học và an toàn là hai yêu cầu riêng

Đoạn nối có độ cong đầu–cuối bằng không giúp loại sự nhảy độ cong giữa đoạn thẳng và chuyển tiếp chính quy. Tuy nhiên, pivot chứa thay đổi hướng tại vị trí tịnh tiến gần như không đổi, nên không thể mô tả toàn chuỗi pose bằng một đường cong chính quy G² duy nhất. Sau khi rời smoother, sai số bám và động lực học còn có thể tạo thay đổi độ cong ở vết thực thi. Cách phát biểu chính xác là PSTMO xây dựng các chuyển tiếp tịnh tiến G², không phải bảo đảm mọi quỹ đạo thực thi toàn cục đều G².

Tương tự, footprint an toàn trên bản đồ là điều kiện cần được kiểm tra cùng tính trơn, không phải hệ quả của tính trơn. Một cung tròn hoặc Bézier đẹp trên hình vẫn có thể đưa góc thân xe sát kệ. Kết quả khoảng hở của nghiên cứu cho thấy đánh đổi này hiện hữu: PSTMO thường có khoảng hở nhỏ hơn Constrained dù không ghi nhận mẫu va chạm. Do đó, ứng dụng có yêu cầu biên an toàn cao cần kiểm tra lại tham số và sai số mô hình theo yêu cầu của chính ứng dụng.

Không có can thiệp Collision Monitor trong bộ lặp chỉ cho biết kênh này không ghi nhận can thiệp theo cấu hình đã chạy. Điều đó không thay thế đo tiếp xúc, không chứng minh robot an toàn dưới lỗi cảm biến và không tạo một chứng nhận an toàn chức năng. Với bản đồ 0,05 m/ô, thay đổi khoảng hở trung bình vài milimét còn cần được đọc trong giới hạn lượng tử hóa bản đồ và phép nội suy footprint. Việc lưu nhiều chữ số phục vụ tính toán không đồng nghĩa độ chính xác đo ở cùng mức.

### 3) Độ tin cậy nội tại của thiết kế thực nghiệm

Điểm mạnh của thiết kế là kiểm soát đầu vào bằng hash Raw, giữ chung controller và tách chỉ số hình học khỏi ground truth. Đồng thời, việc giữ các trial không đạt, không thay chúng bằng lượt chạy lại và báo riêng các retry hạ tầng hạn chế một nguồn thiên lệch chọn kết quả. Các hình và bảng được sinh từ nguồn JSON/CSV đã định danh; ma trận lịch sử không bị chỉnh sửa để phù hợp với giả thuyết của phương pháp.

Mặc dù vậy, R01 của ma trận năm phương án thuộc đợt lịch sử khác với R02–R05. Sự khác nhau về thời điểm thu và snapshot triển khai khiến ma trận đó phù hợp hơn với mục tiêu mô tả diện bao phủ kịch bản hơn là ước lượng một hiệu ứng nhân quả đồng nhất trên một phiên bản duy nhất. Bộ lặp riêng với commit và fingerprint cấu hình xác định được dùng để củng cố so sánh PSTMO–Constrained; nó không tự động chuyển các đối chứng Raw, Simple và SG của ma trận cũ thành đối chứng lặp cùng phiên bản.

Thứ tự phương án không được chứng minh là đã ngẫu nhiên hóa, và bốn worker chạy đồng thời tạo tải máy biến thiên. ROS domain và Gazebo partition tách biệt hạn chế giao thoa thông điệp nhưng không loại bỏ chia sẻ CPU. Thời gian smoother vì vậy được xem là chỉ số bổ sung dưới điều kiện tải đã ghi nhận. Đồng hồ mô phỏng giúp xác định thời gian nhiệm vụ theo mô phỏng, song không cung cấp kết quả benchmark thời gian tường đơn luồng hoặc giới hạn độ trễ triển khai trên phần cứng nhúng.

### 4) Giới hạn của suy luận thống kê

Năm repetition trong một khối không thay thế thử nghiệm đa seed, nhiễu cảm biến hoặc thay đổi ma sát. Cùng bản đồ và cùng hình học đầu vào có thể tạo phụ thuộc giữa các lượt, trong khi các khối còn chia sẻ tuyến và planner. Phân tích theo khối giảm nguy cơ nhân tạo hóa cỡ mẫu bằng cách đếm các repetition như bài toán mới, nhưng không giải quyết hoàn toàn phụ thuộc chéo theo tuyến và planner. Vì thế, các CI và kiểm định được diễn giải có điều kiện trên thiết kế hiện tại.

Chỉ các cặp cùng đạt được dùng cho chỉ số liên tục, tạo một tập lựa chọn dựa trên kết quả thực thi. Đây là lựa chọn minh bạch để tránh so sánh thời gian hoàn thành với thời gian abort, nhưng có thể tạo thiên lệch sống sót nếu khả năng không đạt liên quan đến độ khó của đường. Báo cáo tỷ lệ thành công và toàn bộ failure song song giúp người đọc nhận diện giới hạn này; nó không biến phân tích có điều kiện thành ước lượng hiệu năng vô điều kiện của mọi nhiệm vụ.

Ba chỉ số chính và phép Holm được ghi rõ trong script phân tích, nhưng hồ sơ hiện có không chứng minh một kế hoạch phân tích đã được đăng ký trước khi xem dữ liệu. Do đó, nghiên cứu không sử dụng ngôn ngữ tiền đăng ký hoặc xác nhận độc lập. Các chỉ số phụ được xem là bổ sung, và p-value không dùng để chứng minh quan hệ nhân quả giữa từng thành phần thuật toán với một thay đổi vật lý cụ thể. Quy mô hiệu ứng, độ không đồng nhất giữa các khối và ý nghĩa vận hành vẫn là cơ sở diễn giải chính.

### 5) Phạm vi triển khai và hướng kiểm chứng tiếp theo

Trong phạm vi đã khảo sát, PSTMO là một lựa chọn hậu xử lý có lợi về thời gian và chất lượng bám đường trung bình có điều kiện, đồng thời cung cấp cách phối hợp chuyển tiếp trơn với pivot cho robot vi sai. Tuy nhiên, triển khai trong kho thực cần quyết định trước mức khoảng hở tối thiểu, sai số đích chấp nhận được và chi phí khi nhiệm vụ không hoàn thành. Các yêu cầu này không thể được suy ra chỉ từ xếp hạng trung bình của bảng kết quả.

Các kiểm chứng còn thiếu gồm robot thật, vật cản động, nhiều bố trí kho, biến thiên tải và ma sát, nhiều seed hoặc mức nhiễu, cũng như cấu hình controller khác. Chưa có ablation cùng commit tách conditioning, tìm α phân cấp, hai khoảng cắt, DP và kiểm tra cuối để lượng hóa đóng góp riêng. Tương tự, chưa có nghiên cứu tinh chỉnh tham số đối xứng cho các smoother hoặc kiểm chứng lặp đầy đủ đối với Simple và SG. Những nội dung này là giới hạn của bằng chứng hiện tại, không được trình bày như kết quả đã hoàn tất.

Từ hai bộ dữ liệu, kết luận thực nghiệm được giới hạn như sau: PSTMO cải thiện ba chỉ số liên tục chính so với cấu hình Constrained được thử trên các cặp hoàn thành; không có bằng chứng về ưu thế tỷ lệ thành công, và tồn tại đánh đổi về khoảng hở, sai số đích và thời gian xử lý. Cách phát biểu này bảo toàn cả đóng góp của phương pháp lẫn những điều kiện mà kết quả còn cần được kiểm chứng, tạo cơ sở cho việc phát triển và đánh giá tiếp theo trên hệ thống điều hướng thực.

# TÀI LIỆU THAM KHẢO CHO PHẦN II VÀ III

[1] S. Macenski, S. Singh, F. Martín, and J. Ginés, “Regulated pure pursuit for robot path tracking,” Autonomous Robots, vol. 47, pp. 685–694, 2023, doi: 10.1007/s10514-023-10097-6.

[2] A. Savitzky and M. J. E. Golay, “Smoothing and differentiation of data by simplified least squares procedures,” Analytical Chemistry, vol. 36, no. 8, pp. 1627–1639, 1964, doi: 10.1021/ac60214a047.

[3] Nav2, “Constrained Smoother,” Nav2 Jazzy API documentation. [Online]. Available: https://api.nav2.org/nav2-jazzy/html/md_nav2_constrained_smoother_README.html. Accessed: Oct. 13, 2026.

[4] Nav2, “SmootherServer::smoothPlan,” navigation2, branch jazzy, nav2_smoother/src/nav2_smoother.cpp. [Online]. Available: https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_smoother/src/nav2_smoother.cpp. Accessed: Oct. 13, 2026.

[5] B. Efron, “Bootstrap methods: Another look at the jackknife,” The Annals of Statistics, vol. 7, no. 1, pp. 1–26, 1979, doi: 10.1214/aos/1176344552.

[6] F. Wilcoxon, “Individual comparisons by ranking methods,” Biometrics Bulletin, vol. 1, no. 6, pp. 80–83, 1945, doi: 10.2307/3001968.

[7] S. Holm, “A simple sequentially rejective multiple test procedure,” Scandinavian Journal of Statistics, vol. 6, no. 2, pp. 65–70, 1979.
