# Hồ sơ hiệu đính khoa học journal ver9

Ngày 22/10/2026. Bản gốc: ver8; phạm vi phần II–III. Không chạy ROS/Gazebo/benchmark hoặc tính lại kiểm định.

## Kết quả audit

Lỗi triển khai rõ nhất là định nghĩa cmax chỉ lấy chi phí tâm. Các sửa khác làm rõ conditioning, gia tốc góc, hai trim, chính quy/G², phạm vi tối ưu và suy luận thống kê. Không phát hiện lý do để đổi chín công thức, bảng số liệu hoặc hình.

Phân loại: implementation_correction = sửa mô tả sai/thiếu bước triển khai; ambiguity_correction = sửa câu dễ hiểu sai; clarification = bổ sung điều kiện/giới hạn. Không có đợt viết lại văn phong thuần túy.

Đã đọc toàn bộ manuscript và xem đủ 15 trang render ver8. Giữ nguyên R01–R05: đã có quan sát, so sánh theo planner và đánh đổi; giữ cả RMSE bất lợi ở R02, thất bại R03/R04, độ giật bất lợi R01/R03 và sai số cuối. Giữ III-G, mô tả thất bại III-I.1–2, tiêu chí thành công, các bảng/hình và hệ trích dẫn.

## Đối chiếu nguồn

Đã kiểm tra 166 tệp nguồn duy nhất trong provenance ver6–8 và 10 tệp C++/config/mô hình đối với commit cohort `1a1164e878bbe98e98cebefc009c4fe129174e9c`. Kết quả chi tiết: [source_validation.json](source_validation.json).

Hash khớp xác nhận nguồn của cohort lặp; không suy rộng thành hồ sơ đầy đủ của mọi binary phụ thuộc hay mọi lượt lịch sử R01. Các phiên bản và dữ liệu cũ không được dựng lại.

## Chỉnh sửa có bằng chứng

### A01 — II-A — implementation_correction

Vị trí ver8: khối 5 (đánh số từ 0); `/w:document/w:body/w:p[7]` trong word/document.xml.

Trước: Bước tiền xử lý loại các điểm liên tiếp trùng hoặc quá gần nhau và xử lý dao động hình học cục bộ, có kiểm tra va chạm. Cấu hình khảo sát không bật bước rút gọn riêng theo đường nhìn giữa các điểm. Việc xử lý dao động cục bộ vẫn có thể thay đổi đường đầu vào. Kết quả thực nghiệm vì thế đánh giá toàn bộ PSTMO, bao gồm cả tiền xử lý và lựa chọn chuyển tiếp.

Sau: Bước tiền xử lý loại các điểm liên tiếp trùng nhau, rút gọn đường theo giới hạn độ lệch và xử lý dao động hình học cục bộ; các đoạn thay thế đều được kiểm tra va chạm. Cấu hình khảo sát không bật bước rút gọn riêng theo đường nhìn giữa các điểm. Tiền xử lý vẫn có thể thay đổi hình học đường đầu vào. Kết quả thực nghiệm vì thế đánh giá toàn bộ PSTMO, bao gồm tiền xử lý và lựa chọn chuyển tiếp.

Lý do: Ver8 chưa nêu bước rút gọn theo giới hạn độ lệch trong condition_polyline, dễ khiến condition_only bị hiểu là chỉ loại điểm/dao động.

Nguồn:

- `src/adaptive_pivot_g2/src/path_conditioning.cpp:condition_polyline`
- `src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp:prepare_path`

### B01 — II-B — clarification

Vị trí ver8: khối 7 (đánh số từ 0); `/w:document/w:body/w:p[9]` trong word/document.xml.

Trước: Chiều dài cắt d là khoảng cách từ đỉnh C đến điểm nối trên mỗi cạnh kề. Với d > 0 và hệ số α > 0, đặt A = C − du, B = C + dw và q = αd. Sáu điểm điều khiển được xác định trong công thức 1; đường cong Bézier bậc năm được biểu diễn trong công thức 2. Hình 2 thể hiện các điểm nối và hướng tiếp tuyến tại hai đầu.

Sau: Chiều dài cắt d là khoảng cách từ đỉnh C đến điểm nối trên mỗi cạnh kề. Với d > 0 và 0 < α ≤ 0,5, đặt A = C − du, B = C + dw và q = αd. Sáu điểm điều khiển được xác định trong công thức 1; công thức 2 biểu diễn đường Bézier bậc năm theo tham số hình học t, không phải thời gian. Hình 2 thể hiện các điểm nối và hướng tiếp tuyến tại hai đầu.

Lý do: Bổ sung miền α của triển khai và phân biệt tham số đường cong với thời gian.

Nguồn:

- `src/adaptive_pivot_g2/src/quintic_transition.cpp:generate_quintic_transition_impl`

### B02 — II-B — clarification

Vị trí ver8: khối 10 (đánh số từ 0); `/w:document/w:body/w:p[13]` trong word/document.xml.

Trước: Ba điểm điều khiển ở mỗi đầu thẳng hàng và cách đều. Do đó, r′(0) = 5qu, r′(1) = 5qw và r″(0) = r″(1) = 0. Khi đạo hàm bậc nhất khác không, độ cong hai đầu bằng không và trùng với độ cong của cạnh thẳng. Mối nối liên tục về vị trí, hướng tiếp tuyến và độ cong, đạt G² trên phần đường tịnh tiến có đạo hàm bậc nhất khác không. Phạm vi này không bao gồm thao tác quay tại chỗ hoặc bảo đảm liên tục theo thời gian của vận tốc và gia tốc thực thi.

Sau: Ba điểm điều khiển ở mỗi đầu thẳng hàng và cách đều. Do đó, r′(0) = 5qu, r′(1) = 5qw và r″(0) = r″(1) = 0. Vì q > 0 và u, w là vectơ đơn vị, đạo hàm bậc nhất tại hai đầu khác không. Độ cong hai đầu bằng không, trùng với cạnh thẳng. Các mối nối này đạt G² về vị trí, hướng tiếp tuyến và độ cong; tính chất đó không áp dụng cho quay tại chỗ và không bảo đảm liên tục theo thời gian của vận tốc, gia tốc hoặc độ giật khi thực thi.

Lý do: Nêu trực tiếp điều kiện chính quy tại điểm nối, giới hạn G² tại các mối nối được xây dựng; không gán bảo đảm động học theo thời gian.

Nguồn:

- `src/adaptive_pivot_g2/src/quintic_transition.cpp:evaluate`
- `Suy ra trực tiếp từ sáu điểm điều khiển trong công thức 1`

### B03 — II-B — clarification

Vị trí ver8: khối 12 (đánh số từ 0); `/w:document/w:body/w:p[16]` trong word/document.xml.

Trước: Công thức 3 xác định độ cong κ và tích phân bình phương độ cong Eκ theo chiều dài cung. Đại lượng Eκ có đơn vị m⁻¹, dùng để đánh giá độ cong trên toàn đoạn xét và không biểu thị năng lượng tiêu thụ của robot. Eκ nhỏ hơn không có nghĩa độ cong nhỏ hơn tại mọi điểm. Thuật toán loại các đường có đạo hàm bậc nhất suy biến, giá trị tính toán không hữu hạn hoặc sự đổi dấu độ cong không đáp ứng điều kiện hình dạng.

Sau: Công thức 3 xác định độ cong κ và tích phân bình phương độ cong Eκ theo chiều dài cung. Eκ có đơn vị m⁻¹, không biểu thị năng lượng tiêu thụ; giá trị nhỏ hơn không có nghĩa độ cong nhỏ hơn tại mọi điểm. Trong tìm hình dạng, độ cong được tính từ đạo hàm Bézier tại các mẫu; Eκ được xấp xỉ bằng quy tắc hình thang với chiều dài dây cung. Thuật toán loại đường khi phát hiện đạo hàm suy biến, giá trị không hữu hạn hoặc đổi dấu độ cong ngoài dung sai tại các mẫu kiểm tra.

Lý do: Phân biệt tích phân lý thuyết, xấp xỉ nội bộ và phép đo ba điểm ở III-A; giới hạn phát hiện suy biến/đổi dấu theo mẫu.

Nguồn:

- `src/adaptive_pivot_g2/src/quintic_transition.cpp:sample_curve`
- `src/adaptive_pivot_g2/src/quintic_transition.cpp:generate_quintic_transition_impl`

### C01 — II-C — implementation_correction

Vị trí ver8: khối 15 (đánh số từ 0); `/w:document/w:body/w:p[19]` trong word/document.xml.

Trước: Với khoảng cách hai bánh b, vận tốc bánh trái và phải là vL = v(1 − bκ/2) và vR = v(1 + bκ/2). Khi robot di chuyển tiến trên đoạn chuyển tiếp, cả hai hệ số 1 ± bκ/2 phải không âm. Quay tại chỗ được xét như một phương án riêng. Công thức 4 kết hợp giới hạn vận tốc thân xe, vận tốc góc, gia tốc ngang và vận tốc bánh. Hai lượt tính toán từ đầu đến cuối đoạn và theo chiều ngược lại tiếp tục áp đặt giới hạn tăng tốc, giảm tốc và chuyển động góc. Đây là thứ tự tính toán, không phải yêu cầu robot di chuyển lùi.

Sau: Với khoảng cách hai bánh b, vận tốc tịnh tiến v và độ cong có dấu κ, vận tốc dài tại bánh trái và phải là vL = v(1 − bκ/2) và vR = v(1 + bκ/2). Chuyển tiếp tiến yêu cầu hai hệ số 1 ± bκ/2 không âm; quay tại chỗ được xét riêng. Công thức 4 kết hợp giới hạn vận tốc thân xe, vận tốc góc, gia tốc ngang và vận tốc bánh. Các lượt quét xuôi–ngược áp đặt tăng tốc, giảm tốc; vòng lặp tiếp tục hạ giới hạn vận tốc để thỏa gia tốc góc ước lượng giữa các mẫu. Hồ sơ không hội tụ bị loại. Tính khả thi theo mô hình này chưa xác định khả năng bám đường của bộ điều khiển.

Lý do: Ver8 gộp áp đặt gia tốc góc vào hai lượt quét; mã dùng vòng lặp hạ caps và kiểm tra hội tụ. Làm rõ vL/vR là vận tốc dài, không phải vận tốc góc bánh.

Nguồn:

- `src/adaptive_pivot_g2/src/time_parameterization.cpp:parameterize_time`
- `src/adaptive_pivot_g2/src/quintic_transition.cpp:generate_quintic_transition_impl`

### C02 — II-C — clarification

Vị trí ver8: khối 17 (đánh số từ 0); `/w:document/w:body/w:p[22]` trong word/document.xml.

Trước: Tại đoạn thẳng, các hạng chia cho |κ| trong công thức 4 không tạo thêm giới hạn hữu hạn. Thuật toán kiểm tra cả chi phí tại tâm và đường bao được đặt theo vị trí, hướng của robot. Chi phí tại tâm phản ánh vùng chi phí mở rộng quanh vật cản; kiểm tra đường bao xét thêm kích thước và hướng thân xe. Phép kiểm tra được thực hiện tại các tư thế lấy mẫu nên phụ thuộc độ phân giải bản đồ và khoảng cách lấy mẫu, chưa phải bảo đảm không va chạm liên tục giữa mọi tư thế.

Sau: Tại đoạn thẳng, các hạng chia cho |κ| trong công thức 4 không tạo thêm giới hạn hữu hạn. Chi phí tại tâm phải qua ngưỡng cấu hình; vùng hình bao đã định hướng phải không chứa ô vật cản gây va chạm hoặc ô chưa biết. Ngưỡng chi phí mở rộng quanh vật cản được áp tại tâm, không áp thêm cùng ngưỡng lên toàn đường bao. Kiểm tra chuyển tiếp nội suy cả vị trí và hướng giữa các mẫu; quay tại chỗ được lấy mẫu theo góc. Đây vẫn là kiểm tra hình bao tại các tư thế rời rạc, phụ thuộc độ phân giải và khoảng lấy mẫu, chưa bảo đảm an toàn liên tục.

Lý do: Làm rõ phân biệt ngưỡng inflation tại tâm với va chạm footprint, và nội suy tư thế không phải chứng minh swept volume liên tục.

Nguồn:

- `src/adaptive_pivot_g2_nav2/src/footprint_safety.cpp:pose_is_safe`
- `src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp:transition_is_safe`

### D01 — II-D — clarification

Vị trí ver8: khối 19 (đánh số từ 0); `/w:document/w:body/w:p[24]` trong word/document.xml.

Trước: Hai phương án được so sánh trên cùng đoạn từ vị trí bắt đầu đến vị trí kết thúc xử lý góc. Đoạn chuyển tiếp duy trì chuyển động tiến trong giới hạn của từng bánh. Phương án quay tại chỗ đưa vận tốc tịnh tiến về không rồi thay đổi hướng theo giới hạn quay riêng. Như vậy, thuật toán có thể lựa chọn quay tại chỗ khi đoạn chuyển tiếp không đáp ứng yêu cầu.

Sau: Sau tiền xử lý, độ đổi hướng được xác định từ hai vectơ cạnh kề. Đỉnh có độ đổi hướng nhỏ hơn 5° được giữ làm điểm đi qua, không tạo dấu quay tại chỗ. Với các góc còn lại, hai dạng chuyển động được xét là chuyển tiếp tiến và dừng để quay tại chỗ. Bộ sinh Bézier chỉ nhận góc nhỏ hơn 170°; ngoài miền này, thuật toán chỉ còn phương án quay tại chỗ nếu khả thi.

Lý do: Thiếu ngưỡng phát hiện góc, pass-through và miền góc của bộ sinh; tránh diễn giải toàn bộ đường đầu ra đều G².

Nguồn:

- `src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp:smooth_pipeline`
- `src/adaptive_pivot_g2/include/adaptive_pivot_g2/types.hpp:TransitionOptions`
- `src/vacuum_robot_gazebo/config/nav2_params.yaml`

### D02 — II-D — implementation_correction

Vị trí ver8: khối 20 (đánh số từ 0); `/w:document/w:body/w:p[25]` trong word/document.xml.

Trước: PSTMO xét hai chiều dài cắt tại mỗi góc. Chiều dài ưu tiên được chặn bởi hai cạnh kề và giới hạn trên dmax. Chiều dài thứ hai dành phần cạnh chung cho góc tiếp theo. Với cạnh dài L và khoảng dự phòng m, phần chiều dài dành cho mỗi đầu là (L − m)/2. Thuật toán bỏ các chiều dài cắt trùng nhau ở độ phân giải tìm kiếm hoặc nhỏ hơn dmin. Hai lựa chọn cho phép cân bằng độ rộng chuyển tiếp với khả năng ghép các góc liên tiếp.

Sau: PSTMO xét tối đa hai chiều dài cắt tại mỗi góc. Chiều dài ưu tiên là giá trị nhỏ nhất của hai cạnh kề và dmax. Chiều dài tương thích được chặn thêm bởi phần dành cho mỗi đầu của từng cạnh chung, bằng max(0, L − m)/2; cạnh nối trực tiếp điểm đầu hoặc đích được dùng toàn chiều dài. Thuật toán bỏ giá trị dưới dmin hoặc trùng chiều dài ưu tiên trong độ phân giải tìm kiếm. Cách tạo này dành chiều dài cho cả góc trước và góc sau.

Lý do: Ver8 chỉ nói dành phần cạnh cho góc tiếp theo; công thức triển khai xét cả hai cạnh, trường hợp biên và tối đa hai giá trị.

Nguồn:

- `src/adaptive_pivot_g2/src/hierarchical_shape_search.cpp:derive_two_trim_candidates`
- `src/adaptive_pivot_g2/test/test_hierarchical_shape_search.cpp:DerivedTrimCandidates`

### D03 — II-D — clarification

Vị trí ver8: khối 21 (đánh số từ 0); `/w:document/w:body/w:p[26]` trong word/document.xml.

Trước: Với mỗi chiều dài cắt, thuật toán xét tập α = {0,1; 0,2; 0,3; 0,4; 0,5}. Nếu chưa có đường cong khả thi, thuật toán xét thêm {0,15; 0,25; 0,35; 0,45}, rồi tinh chỉnh α quanh giá trị tốt nhất và loại các giá trị trùng. Bước tìm hình dạng chọn đường cong có tích phân bình phương độ cong nhỏ nhất trong tập khả thi. Tiêu chí này khác với hàm chi phí tổng hợp dùng để lựa chọn phương án chuyển động ở bước sau.

Sau: Với mỗi chiều dài cắt, thuật toán xét α = {0,1; 0,2; 0,3; 0,4; 0,5}; nếu chưa có hình dạng khả thi thì xét thêm {0,15; 0,25; 0,35; 0,45}. Quanh giá trị có Eκ nhỏ nhất, khoảng tinh chỉnh nối hai mốc thô kề hoặc là ô thô chứa giá trị bổ sung; tại biên miền chỉ dùng khoảng một phía. Khoảng này được chia thành mười phần bằng nhau và bỏ mẫu trùng. Mỗi giá trị phải qua kiểm tra hình học, hình bao và tham số hóa thời gian. Sau tinh chỉnh, chỉ giữ một hình dạng có Eκ nhỏ nhất cho mỗi chiều dài cắt; khi bằng nhau trong dung sai, ưu tiên α nhỏ hơn. Hàm chi phí tổng hợp ở bước sau không được tối ưu trực tiếp trên toàn miền α.

Lý do: Bổ sung quy tắc tái lập refinement, khả thi và mỗi trim chỉ giữ một shape; không nhầm mục tiêu Eκ với J.

Nguồn:

- `src/adaptive_pivot_g2/src/hierarchical_shape_search.cpp:search_control_fraction_coarse_to_fine`
- `src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp:evaluate_shape`

### D04 — II-D — clarification

Vị trí ver8: khối 22 (đánh số từ 0); `/w:document/w:body/w:p[27]` trong word/document.xml.

Trước: Thời gian của các phương án được tính trên cùng đoạn xử lý góc, với cùng giới hạn vận tốc đầu và cuối có thể đạt được. Gọi Tmin là thời gian ngắn nhất của các chuyển tiếp khả thi, Tp là thời gian quay tại chỗ trên đoạn này và δT là mức giảm thời gian tối thiểu yêu cầu. Khi quay tại chỗ khả thi, nhánh chuyển tiếp chỉ được mở nếu thỏa công thức 5.

Sau: Các chuyển tiếp sau tìm hình dạng được bổ sung đoạn thẳng để cùng bắt đầu và kết thúc trên cửa sổ có chiều dài cắt lớn nhất. Vận tốc biên chung được hạ lặp đến trạng thái có thể đạt, rồi tính lại hồ sơ; phương án không đáp ứng trạng thái biên bị loại. Tmin là thời gian dự đoán nhỏ nhất trong tập còn lại. Tp gồm thời gian tiến đến đỉnh, dừng–quay và rời đỉnh trên cùng cửa sổ với cùng vận tốc biên. Khi quay tại chỗ khả thi, nhánh chuyển tiếp chỉ mở nếu đạt mức giảm tối thiểu δT trong công thức 5.

Lý do: Định nghĩa Tmin trên tập hữu hạn sau tìm hình dạng, Tp gồm tịnh tiến và quay; giải thích cửa sổ và biên chung.

Nguồn:

- `src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp:parameterize_common_window`
- `src/adaptive_pivot_g2/src/time_parameterization.cpp:estimate_pivot_window_time`

### D05 — II-D — implementation_correction

Vị trí ver8: khối 25 (đánh số từ 0); `/w:document/w:body/w:p[30]` trong word/document.xml.

Trước: Hàm chi phí trong công thức 6 dùng để lựa chọn phương án tại góc. Gọi cmax là chi phí bản đồ lớn nhất tại các mẫu tâm và ωpeak là trị tuyệt đối vận tốc góc lớn nhất. Các thành phần chuẩn hóa là ρc = min(1, cmax/252), ρω = min(1, ωpeak/ωmax) và ρE = Eκ/(Eκ + Escale), lần lượt tương ứng với chi phí vật cản, vận tốc góc và tích phân bình phương độ cong. Ba trọng số có tổng bằng một. Phương án quay tại chỗ nhận chi phí 1 nếu có chuyển tiếp đáp ứng điều kiện thời gian, và 0 nếu không có.

Sau: Hàm chi phí trong công thức 6 cung cấp chi phí cục bộ cho bước chọn chuỗi. Gọi cmax là cực đại, trên các tư thế kiểm tra chuyển tiếp, của giá trị lớn hơn giữa chi phí tại tâm và chi phí trên đường bao; đây là chỉ số chi phí bản đồ, không phải khoảng hở đo bằng mét. ωpeak là trị tuyệt đối vận tốc góc lớn nhất của hồ sơ dự đoán trên cửa sổ chung. Các thành phần không thứ nguyên là ρc = min(1, cmax/252), ρω = min(1, ωpeak/ωmax) và ρE = Eκ/(Eκ + Escale), với Escale = 1 m⁻¹. Ba trọng số có tổng bằng một và là cấu hình dùng trong nghiên cứu; chưa có bằng chứng xác lập bộ trọng số tối ưu.

Lý do: Sửa định nghĩa cmax: pose_is_safe trả max(center_cost, footprint_cost), transition_is_safe lấy cực đại qua tư thế. Bổ sung đơn vị Escale và giới hạn diễn giải trọng số.

Nguồn:

- `src/adaptive_pivot_g2_nav2/src/footprint_safety.cpp:pose_is_safe`
- `src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp:transition_is_safe`
- `src/adaptive_pivot_g2/src/candidate_selection.cpp:stable_candidate_cost`
- `src/vacuum_robot_gazebo/config/nav2_params.yaml`

### D06 — II-D — clarification

Vị trí ver8: khối 27 (đánh số từ 0); `/w:document/w:body/w:p[32]` trong word/document.xml.

Trước: Nếu không có chuyển tiếp đáp ứng yêu cầu, thuật toán sử dụng phương án quay tại chỗ khả thi. Chuỗi được chọn có thể kết hợp hai dạng chuyển động và được kiểm tra lại sau khi ghép.

Sau: Điều kiện thời gian và khoảng cho phép lọc tập phương án trước khi so sánh chi phí, nên thời gian không phải một hạng bù trừ trong J. Phương án quay tại chỗ khả thi nhận chi phí quy ước 1 khi nhánh chuyển tiếp mở, và 0 khi nhánh đóng; không áp Eκ cho thao tác quay. Các chuyển tiếp qua bộ lọc và phương án quay khả thi được đưa vào quy hoạch động. Vì phải bảo đảm tương thích giữa các góc, một góc vẫn có thể được chọn quay tại chỗ dù có chuyển tiếp khả thi cục bộ.

Lý do: Nêu logic của hai tầng dưới dạng quy tắc triển khai; pivot là chi phí quy ước và vẫn có thể được DP chọn do xung đột cạnh chung.

Nguồn:

- `src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp:transition_branch_open`
- `src/adaptive_pivot_g2/src/path_optimization.cpp:optimize_corner_states`

### E01 — II-E — clarification

Vị trí ver8: khối 29 (đánh số từ 0); `/w:document/w:body/w:p[34]` trong word/document.xml.

Trước: Gọi Sᵢ là tập phương án khả thi tại góc i; mỗi phương án tương ứng với một trạng thái trong bài toán quy hoạch động. Chuyển tiếp sử dụng chiều dài cắt dᵢ, còn quay tại chỗ sử dụng chiều dài bằng không. Hai phương án liên tiếp tương thích khi tổng chiều dài cắt và khoảng dự phòng m không vượt chiều dài cạnh chung. Với chi phí Jᵢ(k) của phương án k, công thức 7 xác định Fᵢ(k), là tổng chi phí nhỏ nhất đến phương án đó tại góc i. Truy vết từ góc cuối xác định chuỗi phương án được chọn.

Sau: Gọi Sᵢ là tập phương án khả thi được giữ tại góc i; mỗi phương án là một trạng thái của quy hoạch động. Chuyển tiếp dùng chiều dài cắt dᵢ, còn quay tại chỗ dùng chiều dài cắt bằng không. Chọn riêng phương án có J nhỏ nhất ở từng góc có thể làm các chuyển tiếp chồng lấn trên cạnh chung. Quy hoạch động chỉ nối các trạng thái tương thích và tối thiểu hóa tổng chi phí cục bộ. Công thức 7 xác định Fᵢ(k), tổng chi phí nhỏ nhất đến phương án k tại góc i; khởi tạo F₁(k) = J₁(k). Trạng thái không có tiền nhiệm đạt tới được nhận chi phí vô cùng. Truy vết từ trạng thái có chi phí nhỏ nhất ở góc cuối xác định chuỗi được chọn.

Lý do: Bổ sung lý do cần DP, khởi tạo, trạng thái không đạt tới và điều kiện chọn trạng thái cuối.

Nguồn:

- `src/adaptive_pivot_g2/src/path_optimization.cpp:optimize_corner_states`

### E02 — II-E — clarification

Vị trí ver8: khối 33 (đánh số từ 0); `/w:document/w:body/w:p[38]` trong word/document.xml.

Trước: Hai chiều dài cắt và phép tìm kiếm phân cấp giới hạn số đường cong cần đánh giá nhưng có thể bỏ sót hình dạng khả thi khác. Tính tối ưu chỉ áp dụng cho tập phương án đã xây dựng; đường được chấp nhận chưa bảo đảm robot hoàn thành nhiệm vụ khi thực thi. Phần III đánh giá riêng kết quả chuyển động dưới bộ điều khiển.

Sau: Hai chiều dài cắt và phép tìm kiếm phân cấp giới hạn số đường cong được đánh giá nhưng có thể bỏ sót hình dạng khả thi hoặc có chi phí thấp hơn. Tối ưu trên đồ thị không phải tối ưu toàn cục trên mọi đường liên tục hay thời gian thực thi. Tính G² thuộc các mối nối Bézier–đoạn thẳng đã xây dựng; chuỗi tư thế rời rạc, các điểm đi qua dưới ngưỡng và thao tác quay không tạo thành bảo đảm G² cho toàn bộ đầu ra. Phần III đánh giá riêng chuyển động dưới bộ điều khiển.

Lý do: Giới hạn phát biểu G²/tối ưu theo đúng đối tượng; đầu ra không phải quỹ đạo thời gian tối ưu.

Nguồn:

- `src/adaptive_pivot_g2/src/hierarchical_shape_search.cpp`
- `src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp:build_output_path`

### R01 — III-A.2 — clarification

Vị trí ver8: khối 45 (đánh số từ 0); `/w:document/w:body/w:p[53]` trong word/document.xml.

Trước: RPP sử dụng vận tốc đặt 0,30 m/s và khoảng nhìn trước cố định 0,50 m [1]. Chức năng điều chỉnh vận tốc theo độ cong và chi phí được tắt trong thử nghiệm. Các chức năng kiểm tra va chạm, xoay hướng và giảm tốc gần đích vẫn hoạt động, cùng các giới hạn vận tốc phía sau. Do đó, kết quả phản ánh khả năng bám các đường tham chiếu dưới cấu hình điều khiển chung này.

Sau: RPP sử dụng vận tốc đặt 0,30 m/s và khoảng nhìn trước cố định 0,50 m [1]. Điều chỉnh vận tốc theo độ cong và chi phí được tắt; kiểm tra va chạm, xoay hướng và giảm tốc gần đích vẫn hoạt động. Giá trị 0,25 rad/s trong Bảng I là vận tốc đặt cho xoay hướng, không phải giới hạn tuyệt đối của mọi lệnh vận tốc góc khi bám đường. Mô hình PSTMO và bộ làm mượt vận tốc dùng giới hạn 0,80 rad/s. Bộ làm mượt còn áp đặt tăng tốc, giảm tốc; giám sát va chạm có thể điều chỉnh lệnh trước khi tới robot. Do đó, thời gian dự đoán trong PSTMO không phải cam kết thời gian thực thi của hệ điều khiển.

Lý do: Phân biệt vận tốc xoay hướng RPP với giới hạn góc PSTMO/downstream và dự đoán thời gian với thực thi.

Nguồn:

- `src/vacuum_robot_gazebo/config/nav2_params.yaml`
- `https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp:computeVelocityCommands,rotateToHeading`

### R02 — III-A.7 — clarification

Vị trí ver8: khối 71 (đánh số từ 0); `/w:document/w:body/w:p[82]` trong word/document.xml.

Trước: Sai số bám đường là khoảng cách nhỏ nhất từ vị trí xₖ trên quỹ đạo thực thi đã xử lý đến các đoạn của đường tham chiếu P. Với N vị trí, căn trung bình bình phương sai số, viết tắt RMSE, được xác định trong công thức 8. Mỗi phương án dùng đường tham chiếu riêng; RMSE phản ánh khả năng bám đường, không phải sai số định vị.

Sau: Sai số bám đường là khoảng cách nhỏ nhất từ vị trí xₖ trên quỹ đạo thực thi đã xử lý đến các đoạn của đường tham chiếu P. Với N vị trí, căn trung bình bình phương sai số, viết tắt RMSE, được xác định trong công thức 8. Mỗi phương án dùng đường tham chiếu riêng; RMSE phản ánh khả năng bám đường, không phải sai số định vị. Do lấy mẫu theo chiều dài cung, đây là chỉ số trên không gian đường đi, không phải trung bình theo thời gian; nó không mô tả riêng sai số hướng khi quay tại chỗ.

Lý do: Làm rõ đối tượng đo RMSE và giới hạn với quay tại chỗ, dựa trên phép lấy mẫu lại theo cung đã công bố.

Nguồn:

- `src/adaptive_pivot_g2_benchmark/adaptive_pivot_g2_benchmark/execution_trial.py`
- `src/adaptive_pivot_g2_benchmark/adaptive_pivot_g2_benchmark/compare_paths.py`

### H01 — III-H.1 — ambiguity_correction

Vị trí ver8: khối 215 (đánh số từ 0); `/w:document/w:body/w:p[254]` trong word/document.xml.

Trước: Toàn bộ 125 cặp có cùng đường gốc. Constrained đạt 122/125 và PSTMO đạt 121/125; có 118 cặp cùng đạt để phân tích chỉ số liên tục. Mỗi tổ hợp tuyến–bộ lập kế hoạch tạo một nhóm b. Gọi Rb là tập lần lặp cùng đạt trong nhóm; db là trung bình hiệu PSTMO trừ Constrained. Công thức 9 tính chênh lệch chung Δ bằng trung bình đều của 25 nhóm, tránh coi các lần lặp của cùng đầu vào là các đầu vào độc lập.

Sau: Trong toàn bộ 125 cặp, hai phương án nhận cùng đường gốc trong từng cặp. Constrained đạt 122/125 và PSTMO đạt 121/125; có 118 cặp cùng đạt để phân tích chỉ số liên tục. Mỗi tổ hợp tuyến–bộ lập kế hoạch tạo một nhóm b. Gọi Rb là tập lần lặp cùng đạt trong nhóm; db là trung bình hiệu PSTMO trừ Constrained. Công thức 9 tính chênh lệch chung Δ bằng trung bình đều của 25 nhóm, tránh coi các lần lặp cùng đầu vào là các đầu vào độc lập. Các đại lượng yP,br và yC,br là giá trị chỉ số của hai phương án tại lần lặp r trong nhóm b.

Lý do: Sửa câu dễ bị hiểu 125 cặp dùng duy nhất một Raw path; bổ sung biến y trong công thức 9.

Nguồn:

- `results/pstmo_journal_repeatability_20261011/analysis/aggregate.json`
- `tools/analyze_journal_repeatability.py`

### H02 — III-H.1 — clarification

Vị trí ver8: khối 217 (đánh số từ 0); `/w:document/w:body/w:p[257]` trong word/document.xml.

Trước: Khoảng tin cậy 95% dùng bootstrap trên 25 nhóm với 20.000 lần lấy mẫu lại [5]. Kiểm định Wilcoxon hạng có dấu hai phía áp dụng cho db, coi các hiệu có trị tuyệt đối không quá 10⁻¹² là bằng không [6]. Giá trị p của thời gian, RMSE và Eκ,exec được hiệu chỉnh Holm [7]. Các nhóm còn chia sẻ tuyến, bộ lập kế hoạch và bản đồ; suy luận chỉ áp dụng cho thiết kế lặp với đầu vào cố định, không đại diện cho nhiều mức nhiễu hoặc điều kiện ngẫu nhiên.

Sau: Khoảng tin cậy 95% của Δ dùng bootstrap phân vị trên 25 nhóm, lấy mẫu lại 20.000 lần [5]. Kiểm định Wilcoxon hạng có dấu hai phía dùng db, loại các hiệu có trị tuyệt đối không quá 10⁻¹² [6]; xét phân bố chênh lệch đối xứng quanh không, không trực tiếp kiểm định trung bình Δ. Giá trị p của thời gian, RMSE và Eκ,exec được hiệu chỉnh Holm [7], nhưng khoảng tin cậy chỉ ở mức từng chỉ số. Các nhóm chia sẻ tuyến, bộ lập kế hoạch và bản đồ; gom nhóm chưa loại hết phụ thuộc. Suy luận giới hạn trong thiết kế đầu vào cố định.

Lý do: Phân biệt ước lượng mean/CI với Wilcoxon, xử lý zero và hiệu chỉnh p với CI. Giữ toàn bộ số thống kê.

Nguồn:

- `tools/analyze_journal_repeatability.py:bootstrap_mean_ci,wilcoxon_pvalue,holm_adjust`
- `https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.wilcoxon.html`

### H03 — III-H.2 — clarification

Vị trí ver8: khối 222 (đánh số từ 0); `/w:document/w:body/w:p[263]` trong word/document.xml.

Trước: Hình 17 cho thấy RMSE tăng ở một số đầu vào dù chênh lệch chung âm. Vì số cặp cùng đạt khác nhau giữa các nhóm, trung bình đều theo công thức 9 tránh để nhóm có nhiều lượt đạt hơn chi phối kết quả. Bảng XXII đối chiếu các chỉ số bổ sung để xác định những đánh đổi đi kèm.

Sau: Hình 17 cho thấy RMSE tăng ở một số đầu vào dù chênh lệch chung âm. Trung bình đều theo công thức 9 tránh để nhóm nhiều cặp cùng đạt hơn chi phối kết quả, nhưng không hiệu chỉnh thiên lệch do loại cặp không cùng đạt. Ý nghĩa vận hành còn phụ thuộc yêu cầu thời gian, sai số và khoảng hở; p nhỏ không thay thế các tiêu chí đó. Bảng XXII trình bày các đánh đổi.

Lý do: Tách equal block weighting khỏi xử lý selection bias và phân biệt ý nghĩa thống kê với ý nghĩa thực tiễn.

Nguồn:

- `results/pstmo_journal_repeatability_20261011/analysis/aggregate.json`
- `tools/analyze_journal_repeatability.py`

### I01 — III-I.3 — clarification

Vị trí ver8: khối 240 (đánh số từ 0); `/w:document/w:body/w:p[285]` trong word/document.xml.

Trước: Khảo sát dùng một bản đồ tĩnh, một robot mô phỏng và một cấu hình RPP. R01 được thu ở đợt khác R02–R05; bộ lặp chỉ gồm PSTMO và Constrained. Chưa xác nhận thứ tự chạy được ngẫu nhiên hóa; bốn tiến trình dùng chung tài nguyên máy tính dù có miền truyền thông riêng. Các điều kiện này hạn chế khả năng suy rộng và cách diễn giải thời lượng xử lý.

Sau: Khảo sát dùng một bản đồ tĩnh, một robot mô phỏng và một cấu hình RPP; chưa xét vật cản động, nhiều mức khởi tạo ngẫu nhiên hay nhiễu. R01 khác đợt thu R02–R05; bộ lặp chỉ gồm PSTMO và Constrained. Chưa xác nhận ngẫu nhiên hóa thứ tự chạy; bốn tiến trình dùng chung tài nguyên máy dù tách miền truyền thông. Điều này hạn chế suy rộng và diễn giải thời lượng xử lý.

Lý do: Gom rõ giới hạn vật cản động/multi-seed vào phần giới hạn, giữ khác đợt thu và shared resources.

Nguồn:

- `results/pstmo_journal_repeatability_20261011/README.md`
- `docs/warehouse_cross_aisles_5_routes/README.md`

### I02 — III-I.3 — clarification

Vị trí ver8: khối 242 (đánh số từ 0); `/w:document/w:body/w:p[287]` trong word/document.xml.

Trước: PSTMO cho thấy lợi ích về hình học và một số chỉ số thực thi trong phạm vi khảo sát, đồng thời có đánh đổi về khoảng hở, sai số cuối và thời lượng xử lý. Kiểm tra robot thực, nhiều bố trí bản đồ và thí nghiệm loại bỏ từng thành phần sẽ giúp xác định phạm vi áp dụng và đóng góp riêng của thuật toán. Dữ liệu hiện có chưa hỗ trợ kết luận về điện năng, rung tải hàng hoặc ưu thế tỷ lệ thành công.

Sau: Lợi ích hình học và thực thi của PSTMO đi kèm đánh đổi về khoảng hở, sai số cuối và thời lượng xử lý. Chưa kiểm chứng trên robot thực hoặc loại bỏ từng thành phần trên cùng phiên bản để tách đóng góp của tiền xử lý, tìm hình dạng và quy hoạch động. Kiểm tra hình bao phụ thuộc bản đồ và lấy mẫu. Dữ liệu chưa hỗ trợ kết luận về điện năng, rung tải hàng hay ưu thế tỷ lệ thành công.

Lý do: Nêu rõ thiếu ablation cùng phiên bản và robot thực, giữ đánh đổi và phạm vi kết luận về reliability/energy.

Nguồn:

- `results/pstmo_journal_repeatability_20261011/README.md`
- `docs/agent/DATA_POLICY.md`

## Giới hạn chưa giải quyết

- Chưa có hồ sơ tối ưu trọng số hoặc ablation cùng phiên bản; ver9 chỉ báo đúng cấu hình, không đặt lý do thiết kế giả định.
- Không chứng minh an toàn liên tục, tối ưu quỹ đạo liên tục hay bảo đảm thực thi từ kiểm tra rời rạc.
- Bộ lặp có phụ thuộc giữa các nhóm dùng chung tuyến/planner/bản đồ; chưa có phân tích mô hình phụ thuộc khác, kiểm tra tương đương hoặc chính sách phục hồi được đo. Giữ nguyên CI/p lịch sử và giảm phạm vi diễn giải.
- Mã phân tích dùng scipy.stats.wilcoxon(method='auto'); manifest không khóa đầy đủ phiên bản thư viện thống kê. Đợt này đối chiếu và bảo toàn kết quả đã lưu, không tuyên bố tái tính đúng bit.
- Tên Gazebo Harmonic được yêu cầu trong brief, nhưng manifest cohort không ghi bản phát hành Gazebo. Giữ cách gọi Gazebo của ver8; không suy ra phiên bản thực nghiệm chỉ từ máy hiện tại.
- Nguồn RPP Jazzy và tài liệu SciPy trực tuyến được đọc ngày 22/10/2026 để kiểm tra ý nghĩa tham số/kiểm định; đây là nguồn bổ trợ, không xác nhận hash binary lịch sử. Không thêm trích dẫn mới vào bài.
- Hai phần vẫn cần ghép toàn bài và kiểm tra mẫu tạp chí đích trước khi nộp.

## Kiểm chứng đầu ra

Các giá trị thực nghiệm, 18 hình, 23 bảng, 12 đối tượng Office Math và chín số công thức được giữ nguyên. Script kiểm tra đối chiếu cấu trúc gói, nội dung, lineage và số liệu nguồn; trạng thái thực tế nằm trong [quality_checks.json](quality_checks.json). Xác nhận trực quan phải gắn với hash DOCX/PDF/PNG của bản cuối qua [visual_review.json](visual_review.json).
