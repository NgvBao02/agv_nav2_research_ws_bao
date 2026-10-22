# Văn bản trích xuất từ Word ver11

Bảng, hình và công thức gốc nằm trong Word; đây không phải nguồn dựng.

Hậu xử lý đường đi có xét tính khả thi chuyển độngcho robot vi sai trong môi trường kho

Motion-Feasibility-Aware Path Post-Processing for Differential-Drive Robots in Warehouse Environments


TÓM TẮT

Đường đi qua các hành lang kho cần được đánh giá không chỉ bằng chiều dài và độ trơn mà còn bằng khả năng thực hiện chuyển hướng của robot. Với robot vi sai, chuyển tiếp cong và dừng để quay tại chỗ tạo ra những lựa chọn khác nhau về thời gian, khoảng hở và khả năng ghép nối. Bài báo đề xuất PSTMO (Path Smoothing and Turning-Maneuver Optimization), một phương pháp hậu xử lý đường đi kết hợp chuyển tiếp Bézier bậc năm với phương án quay tại chỗ. Các ứng viên được kiểm tra hình bao robot trên bản đồ, sàng lọc theo giới hạn chuyển động và thời gian dự đoán trên cùng cửa sổ so sánh. Quy hoạch động lựa chọn chuỗi ứng viên có tổng chi phí nhỏ nhất trong đồ thị hữu hạn, với ràng buộc không chồng lấn phần cắt trên cạnh chung. Phương pháp được đánh giá trong ROS 2 Jazzy/Nav2/Gazebo bằng ma trận khảo sát 125 lượt và bộ kiểm chứng lặp riêng gồm 250 lượt PSTMO–Constrained. Trên 118 cặp cùng đạt, trung bình đều chênh lệch của 25 nhóm tuyến–bộ lập kế hoạch cho thấy PSTMO giảm thời gian thực thi 2,857 s và căn trung bình bình phương sai số bám đường 2,985 mm. Khoảng tin cậy bootstrap nhóm 95% của hai chênh lệch lần lượt là [−3,716; −2,039] s và [−4,487; −1,506] mm. Lợi ích đi kèm giảm khoảng hở và tăng thời lượng hậu xử lý; số lượt đạt là 121/125 với PSTMO và 122/125 với Constrained. Kết quả hỗ trợ cách lựa chọn chuyển tiếp có xét khả năng thực thi trong cấu hình khảo sát, chưa xác lập ưu thế tỷ lệ thành công hoặc khả năng khái quát ngoài một bản đồ tĩnh và một robot mô phỏng.

Từ khóa — Robot vi sai; hậu xử lý đường đi; tính khả thi chuyển động; chuyển tiếp Bézier; quy hoạch động; điều hướng trong kho.

ABSTRACT

Paths through warehouse aisles must be assessed not only by length and smoothness but also by the robot's ability to execute turns. This paper presents Path Smoothing and Turning-Maneuver Optimization (PSTMO), a path post-processing method that combines quintic Bézier transitions with in-place rotation for differential-drive robots. Candidates are screened using sampled robot-footprint checks, motion limits, and predicted traversal times over a common comparison window. Dynamic programming selects a minimum-cost sequence in the retained finite candidate graph while preventing overlapping trims on shared edges. Evaluation in ROS 2 Jazzy/Nav2/Gazebo uses a 125-run exploratory matrix and a separate 250-run PSTMO–Constrained repeatability study. Among 118 jointly successful pairs, equally weighting the paired mean differences of 25 route–planner blocks yields reductions of 2.857 s in execution time and 2.985 mm in path-tracking root-mean-square error. The respective 95% block-bootstrap confidence intervals for PSTMO-minus-Constrained differences are [−3.716, −2.039] s and [−4.487, −1.506] mm. These improvements accompany reduced clearance and increased post-processing duration. PSTMO succeeds in 121 of 125 runs, compared with 122 for Constrained. The results support coordinated transition selection under the tested configuration but do not establish a success-rate advantage or generalization beyond one static map and one simulated robot.

Keywords — Differential-drive robot; path post-processing; motion feasibility; Bézier transition; dynamic programming; warehouse navigation.

I GIỚI THIỆU

Trong hệ thống kho, điều hướng robot là một mắt xích của bài toán vận chuyển và tổ chức tác nghiệp. Các tổng quan về hệ AGV [1], robot di động tự hành trong logistics [2] và lấy hàng trong kho [3] phân biệt những quyết định về bố trí, định tuyến, lập lịch và điều khiển. Bài học từ Amazon Picking Challenge còn cho thấy thao tác lấy hàng đặt ra các yêu cầu riêng về cảm nhận, thao tác và tích hợp hệ thống [4]. Vì vậy, cải thiện đường di chuyển của một robot không tự động đồng nghĩa tăng năng suất toàn kho. Nghiên cứu này tập trung ở cấp chuyển động: xử lý đường đi qua hành lang và giao cắt để robot vi sai chuyển hướng và tiếp cận đích trong không gian hạn chế.

Các tổng quan điều hướng [5] và lập kế hoạch đường đi [6] cho thấy lựa chọn thuật toán phụ thuộc biểu diễn môi trường, mô hình robot và tiêu chí đánh giá. Riêng với hậu xử lý, Ravankar và cộng sự [7] hệ thống hóa nhu cầu làm trơn cùng các yêu cầu về độ cong, kích thước robot và vật cản. Một đường ngắn nhưng đổi hướng gấp có thể đòi hỏi giảm tốc hoặc dừng; nắn đường để giảm độ cong lại có thể làm giảm khoảng hở. Do đó, chất lượng hậu xử lý cần được đánh giá bằng cả đường tham chiếu lẫn chuyển động dưới bộ điều khiển, thay vì chỉ dùng một chỉ số hình học.

A Từ đường tìm kiếm đến chuyển động khả thi

Tìm kiếm đường trên đồ thị có các nền tảng khác nhau: Dijkstra giải bài toán đường ngắn nhất [8], A* bổ sung thông tin heuristic [9], còn Focussed D* xử lý cập nhật kế hoạch khi chi phí cạnh thay đổi [10]. Theta* mở rộng khả năng kết nối theo hướng tự do trên lưới [11]. Trong khi đó, các phương pháp lấy mẫu như PRM [12] và RRT [13] khám phá không gian cấu hình hoặc trạng thái bằng những cơ chế khác với tìm kiếm lưới; PRM* và RRT* có tính tối ưu tiệm cận dưới các giả thiết của chúng [14]. Những bảo đảm này gắn với bài toán tìm kiếm được phát biểu, không trực tiếp xác định sai số bám hay thời gian hoàn thành của hệ điều khiển triển khai.

Không phải mọi bộ lập kế hoạch đều bỏ qua động học. Dolgov và cộng sự [15] kết hợp tìm kiếm trong không gian trạng thái xe với tối ưu số đường đi; họ Smac cung cấp tìm kiếm có xét chi phí và khả thi động học [16]. Cũng có cách sinh trực tiếp đường trơn bằng tích phân độ cong, như FDSPC trong môi trường địa hình 2,5D [17]. Các công trình này cho thấy hậu xử lý không phải bước bắt buộc của mọi kiến trúc. Ở đây, nó được chọn như một tầng có thể dùng chung với nhiều bộ lập kế hoạch đã có, để nghiên cứu tác động của lựa chọn chuyển hướng trên cùng đầu vào.

B Làm trơn hình học và ràng buộc không gian

Biểu diễn đường cong quyết định những điều kiện có thể áp đặt tại điểm nối. Komoriya và Tanie [18] dùng B-spline cho thiết kế đường và điều khiển robot bánh xe; Piazzi và cộng sự [19] xây dựng spline bậc năm G² cho dẫn hướng xe; Phan Gia Luan và Nguyen Truong Thinh [20] nghiên cứu làm trơn bằng Bézier bậc ba từng đoạn C². Đây là các khái niệm cần phân biệt: C² yêu cầu liên tục đạo hàm theo tham số, còn G² mô tả sự phù hợp hình học về tiếp tuyến và độ cong tại mối nối chính quy. Chọn một biểu diễn có khả năng tạo đường trơn chưa đồng thời chứng minh không va chạm hoặc thỏa giới hạn chuyển động.

Clothoid là một hướng xây dựng chuyển tiếp khác, đã được dùng cho robot vi sai [21]. Fleury và cộng sự [22] xét các nguyên thủy làm trơn khi đường có điểm đổi chiều; Lekkas và cộng sự [23] xây dựng đường liên tục độ cong từ xoắn ốc Fermat. Tuttle và Wilhelm [24] tối ưu các đường trở lại tuyến bằng nhiều đoạn clothoid cho phương tiện có ràng buộc tốc độ quay. Tuy bối cảnh ứng dụng khác nhau, các nghiên cứu này cùng cho thấy cần gắn điều kiện nối hình học với loại chuyển động được phép, không áp một mẫu chuyển tiếp cho mọi mô hình robot.

Về tối ưu hình dạng, Parque và Miyashita [25] kết hợp tiêu chí khớp và độ trơn khi xấp xỉ quỹ đạo robot bằng tiến hóa vi phân. Xu và cộng sự [26] phối hợp tối ưu bầy đàn với Bézier liên tục; Türkkol và cộng sự [27] xây dựng đường Bézier từ đa giác điều khiển do RRT cung cấp cho phương tiện mặt nước. Những phương án này khác nhau ở việc khớp dữ liệu, tối ưu đường hoặc làm trơn đầu ra tìm kiếm. Không thể chuyển nguyên kết quả của phương tiện mặt nước hay bài toán khớp quỹ đạo sang robot vi sai trong hành lang hẹp.

Ràng buộc không gian có thể được đưa vào quá trình làm trơn thay vì chỉ kiểm tra sau cùng. Pan và cộng sự [28] kết hợp tính liên tục độ cong với kiểm tra va chạm liên tục; Andrei và cộng sự [29] dùng miền đa diện an toàn và tối ưu bậc hai cho Bézier từng đoạn C¹. Serdel và cộng sự [30] khai thác bản đồ hồi quy quá trình Gaussian về khả năng đi qua địa hình và khoảng cách vật cản để tối ưu Bézier. Các bảo đảm an toàn phụ thuộc biểu diễn môi trường và phép kiểm tra của từng phương pháp. PSTMO sử dụng bản đồ chi phí cùng hình bao robot tại các tư thế lấy mẫu, nên không kế thừa bảo đảm va chạm liên tục của các công trình trên.

C Tối ưu quỹ đạo và kết nối với điều khiển

Elastic Bands [31] biến dạng đường do bộ lập kế hoạch cung cấp để kết nối lập kế hoạch với điều khiển dựa cảm biến. Richardson và Olson [32] tách tìm đường ngắn khỏi bước tăng khoảng hở. Hai nghiên cứu của Andreasson và cộng sự lần lượt xét chuyển kế hoạch rời rạc thành quỹ đạo chạy được [33] và làm trơn trong không gian trạng thái liên tục để cải thiện độ chính xác điều hướng [34]. GRIPS [35] kết hợp thông tin gradient với hàm kết nối phù hợp chuyển động. Điểm cần kế thừa là quan hệ giữa hình học, vật cản và mô hình robot; giảm một chỉ số độ cong riêng lẻ không thay thế kiểm chứng thực thi.

Các bộ tối ưu quỹ đạo còn khác nhau ở cách tìm kiếm lời giải. CHOMP [36] dùng cập nhật gradient hiệp biến để cải thiện quỹ đạo; STOMP [37] sử dụng nhiễu ngẫu nhiên để cập nhật quỹ đạo mà không đòi hỏi gradient của hàm chi phí. ITOMP [38] tối ưu tăng dần và xen kẽ lập kế hoạch với thực thi trong môi trường động. Những nghiên cứu có đánh giá trên hệ nhiều bậc tự do này cung cấp các nguyên lý tối ưu, nhưng không phải đối chứng trực tiếp đã được chạy trong thí nghiệm robot vi sai của bài báo. Zhu và cộng sự [39] xét gần hơn bài toán xe mặt đất, luân phiên tối ưu hình dạng và vận tốc bằng các bài toán lồi cho robot kiểu ô tô.

Yếu tố thời gian có thể được đưa trực tiếp vào biến tối ưu. Từ việc sửa đổi quỹ đạo dưới giới hạn vận tốc và gia tốc [40], Timed Elastic Band được phát triển với biểu diễn tối ưu thưa [41], điều khiển dự báo phi tuyến tối ưu thời gian [42], và mô hình động học–động lực học cho robot kiểu ô tô [43]. Việc tối ưu trên các lớp tôpô khác nhau mở rộng khả năng lựa chọn quỹ đạo [44]. Đối với robot vi sai, Zhang và cộng sự [45] xây dựng khuôn khổ tối ưu quỹ đạo cho nhiều dạng truyền động và trạng thái chuyển động. Vì vậy, xét thời gian hoặc quay tại chỗ không phải tự thân một tuyên bố mới; vấn đề cần làm rõ là các quyết định này được tổ chức ở tầng nào và được kiểm chứng ra sao.

Các nghiên cứu Bézier có xét thời gian đặc biệt gần với bài toán hậu xử lý. Choi và cộng sự [46] xây dựng đường tránh vật cản và chuyển động có ràng buộc động lực cho robot đa hướng. Benko Loknar và cộng sự [47] dùng Bézier bậc năm với giới hạn vận tốc, gia tốc và độ giật. Trong môi trường kho, Blažič và cộng sự [48] ghép các nguyên thủy Bézier bậc ba rồi xác định hồ sơ vận tốc; bước tìm hình dạng trong ví dụ của họ dùng tích phân bình phương độ cong và hình phạt ra ngoài hành lang. Dorpmüller và cộng sự [49] tối ưu tham số B-spline cùng biến thời gian trong kịch bản cao tốc. Các mô hình và mục tiêu khác nhau này yêu cầu phân biệt chi phí hình dạng, thời gian dự đoán và thời gian thực thi.

Tương tác với bộ điều khiển cũng là một phần của bài toán. Dynamic Window Approach tìm lệnh trong không gian vận tốc có thể đạt trong thời gian ngắn [50], trong khi BSPOP tham số hóa tín hiệu điều khiển liên tục bằng B-spline để nối các tầng chạy ở tần số khác nhau [51]. Phan Gia Luan và Nguyen Truong Thinh [52] kết hợp đường toàn cục với tránh vật cản động cho robot vi sai; Șomîtcă và cộng sự [53] khảo sát độ sát giữa đường Bézier và đường robot thực thi trong môi trường bất định. Những kết quả này củng cố yêu cầu đánh giá vòng kín. Đầu ra PSTMO vẫn là đường tham chiếu, không phải một chính sách tránh vật cản động hoặc chuỗi lệnh vận tốc bắt buộc bộ điều khiển tuân theo.

D Bài toán nghiên cứu và định hướng phương pháp

Đối với robot vi sai có khả năng quay tại chỗ, không phải mọi góc đều cần được thay bằng đường cong. Chuyển tiếp cong có thể duy trì chuyển động tiến, nhưng cần đủ không gian và chiều dài ở hai cạnh kề. Dừng để quay tại chỗ sử dụng không gian theo cách khác và vẫn đòi hỏi kiểm tra hình bao trong quá trình đổi hướng. Nếu hai góc liên tiếp dùng chung một cạnh ngắn, lựa chọn tốt cho từng góc riêng lẻ có thể chiếm phần chiều dài cần cho góc bên cạnh. Mặt khác, so sánh riêng thời gian đi trên đường cong với thời gian quay sẽ bỏ qua các đoạn tiến đến và rời đỉnh. Các phương án cần được đặt trên cùng cửa sổ không gian và điều kiện vận tốc biên.

Từ đó, câu hỏi nghiên cứu là: một tầng hậu xử lý lựa chọn có phối hợp giữa chuyển tiếp cong và quay tại chỗ, có xét hình bao và giới hạn chuyển động, đem lại những lợi ích và đánh đổi nào khi đường được thực thi dưới cùng bộ điều khiển? Câu hỏi này hướng đánh giá vào thời gian hoàn thành và sai số bám đường, đồng thời giữ khoảng hở, độ chính xác cuối và tỷ lệ thành công như những tiêu chí không thể bỏ qua. Persson và cộng sự [54] đã khảo sát ảnh hưởng của đường khởi tạo đến Timed Elastic Band; vì thế, nghiên cứu hiện tại dùng nhiều bộ lập kế hoạch và ghép cặp trên cùng đường gốc, thay vì suy ra hiệu quả chung từ một đường minh họa.

PSTMO tổ chức lựa chọn tại chuỗi góc bằng quy hoạch động trên một tập hữu hạn ứng viên Bézier bậc năm và quay tại chỗ. Quy hoạch động kết hợp Bézier đã có tiền lệ: Choi [55] dùng quy hoạch động để tìm đường gấp khúc trước bước làm trơn và xây dựng hồ sơ vận tốc. Trong bài báo này, quyết định rời rạc nằm ở việc chọn chuỗi chuyển tiếp tương thích trên một đường đầu vào. Tính tối ưu chỉ áp dụng cho đồ thị ứng viên đã giữ lại, không phải toàn bộ không gian đường liên tục hay thời gian hoàn thành nhiệm vụ. Tính G² chỉ thuộc các mối nối Bézier–đoạn thẳng được xây dựng, không áp dụng cho thao tác quay tại chỗ hoặc toàn bộ chuỗi tư thế rời rạc.

E Đóng góp và phạm vi đánh giá

Đóng góp về phương pháp là cách tổ chức bài toán hậu xử lý thành lựa chọn chuỗi chuyển tiếp dưới ràng buộc cạnh chung. Phép kiểm tra hình bao và giới hạn chuyển động xác định tập phương án; so sánh thời gian dùng cửa sổ chung; quy hoạch động xử lý phụ thuộc giữa các góc kế tiếp. Trọng tâm là sự phối hợp những quyết định này trong PSTMO, không phải tính mới của riêng Bézier bậc năm hoặc nguyên lý quy hoạch động. Kiểm tra hình bao được thực hiện tại các tư thế lấy mẫu, nên không tạo bảo đảm an toàn liên tục giữa mọi cặp mẫu.

Đóng góp thực nghiệm là đánh giá đồng thời đường tham chiếu và chuyển động thực thi trên năm tuyến, năm bộ lập kế hoạch, trong cùng hệ ROS 2 Jazzy/Nav2/Gazebo. Ma trận 125 lượt so sánh đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO; bộ 250 lượt riêng kiểm chứng lặp giữa PSTMO và Constrained. Constrained là đối chứng làm trơn tối ưu số với các thành phần chi phí có thể cấu hình [56]. Mọi kết quả chỉ đại diện cho cấu hình đã thử, không mặc định là hiệu năng tối ưu của từng phương pháp. Bộ điều khiển Regulated Pure Pursuit (RPP) [57] được giữ cùng cấu hình; hồ sơ vận tốc nội bộ PSTMO dùng để lựa chọn đường, không phải lệnh mà RPP bắt buộc phải thực hiện.

Phân tích tách ma trận khảo sát khỏi bộ kiểm chứng lặp, báo riêng các lượt không đạt và không xem năm bộ lập kế hoạch là năm lần lặp độc lập. Ý nghĩa ứng dụng được xem xét qua lợi ích thời gian, chất lượng bám đường và các đánh đổi, không suy thành năng suất kho hoặc điện năng tiết kiệm. Phần II trình bày cấu trúc PSTMO và phạm vi ràng buộc; Phần III mô tả thiết kế thử nghiệm, kết quả và giới hạn suy luận; Phần IV tổng kết những kết luận được dữ liệu hỗ trợ.

II. XÂY DỰNG THUẬT TOÁN PSTMO

A. Bài toán hậu xử lý và phạm vi của phương pháp

PSTMO nhận đường gấp khúc từ bộ lập kế hoạch toàn cục, bản đồ chi phí, hình chiếu bằng của robot và các giới hạn chuyển động. Hình chiếu bằng mô tả diện tích robot chiếm chỗ trên mặt phẳng di chuyển; đường biên của hình này được gọi là đường bao robot. Thuật toán xử lý từng góc bằng đoạn chuyển tiếp Bézier bậc năm hoặc thao tác quay tại chỗ đáp ứng các ràng buộc. Đầu ra là chuỗi tư thế làm đường tham chiếu cho bộ điều khiển Nav2. Bộ lập kế hoạch xác định hành lang di chuyển, còn bộ điều khiển quyết định vận tốc thực thi.

Gọi Q = {q₀, …, qₙ} là đường đầu vào. Tại đỉnh C, các vectơ đơn vị u và w chỉ hướng của cạnh vào và cạnh ra. Mỗi phương án tại góc phải đáp ứng điều kiện hình học, giới hạn chuyển động và kiểm tra đường bao trên bản đồ. Hai phương án kế tiếp còn phải sử dụng chiều dài tương thích trên cạnh chung. Hình 1 trình bày quan hệ giữa lựa chọn tại từng góc và tối ưu chuỗi chuyển tiếp.

[Hình được bảo toàn trong Word]

Hình 1. Luồng xử lý PSTMO và vị trí tích hợp vào Nav2. Các khối biểu diễn các bước của thuật toán; quỹ đạo thực thi trong mô phỏng được thu từ trạng thái do Gazebo cung cấp, độc lập với đường tham chiếu.

Bước tiền xử lý loại các điểm liên tiếp trùng nhau, rút gọn đường theo giới hạn độ lệch và xử lý dao động hình học cục bộ; các đoạn thay thế đều được kiểm tra va chạm. Cấu hình khảo sát không bật bước rút gọn riêng theo đường nhìn giữa các điểm. Tiền xử lý vẫn có thể thay đổi hình học đường đầu vào. Kết quả thực nghiệm vì thế đánh giá toàn bộ PSTMO, bao gồm tiền xử lý và lựa chọn chuyển tiếp.

B. Xây dựng đoạn chuyển tiếp có tính liên tục hình học G²

Biểu diễn Bézier dựa trên cơ sở Bernstein [58]; B-spline [59] và NURBS [60] cung cấp những lựa chọn khác cho thiết kế đường cong. PSTMO dùng một đoạn đa thức bậc năm tại mỗi góc để áp điều kiện đầu mút bằng điểm điều khiển, không tối ưu một spline trên toàn tuyến. Các xoắn ốc Bézier bậc năm Pythagorean-hodograph có những tính chất riêng về độ dài cung và chuyển tiếp [61]; đường bậc năm ở đây không được giả định thuộc lớp PH. Việc chọn bậc và cơ sở biểu diễn vì vậy tách biệt với các tính chất cần chứng minh cho đoạn cụ thể.

Chiều dài cắt d là khoảng cách từ đỉnh C đến điểm nối trên mỗi cạnh kề. Với d > 0 và 0 < α ≤ 0,5, đặt A = C − du, B = C + dw và q = αd. Sáu điểm điều khiển được xác định trong công thức 1; công thức 2 biểu diễn đường Bézier bậc năm theo tham số hình học t, không phải thời gian. Hình 2 thể hiện các điểm nối và hướng tiếp tuyến tại hai đầu.

[Office Math được bảo toàn trong Word] 

[Office Math được bảo toàn trong Word] (1)

[Office Math được bảo toàn trong Word] (2)

Ba điểm điều khiển ở mỗi đầu thẳng hàng và cách đều. Do đó, r′(0) = 5qu, r′(1) = 5qw và r″(0) = r″(1) = 0. Vì q > 0 và u, w là vectơ đơn vị, đạo hàm bậc nhất tại hai đầu khác không. Độ cong hai đầu bằng không, trùng với cạnh thẳng. Các mối nối này đạt G² về vị trí, hướng tiếp tuyến và độ cong; tính chất đó không áp dụng cho quay tại chỗ và không bảo đảm liên tục theo thời gian của vận tốc, gia tốc hoặc độ giật khi thực thi.

[Hình được bảo toàn trong Word]

Hình 2. Cấu trúc Bézier bậc năm cho một góc vuông minh họa. P₀ và P₅ nằm trên hai cạnh; ba điểm đầu và ba điểm cuối thẳng hàng theo các tiếp tuyến tương ứng. Hình không biểu diễn một lượt đo cụ thể.

Tối ưu hình dạng dưới ràng buộc độ cong có tiền lệ trong phương pháp biến dạng đường của Delingette và cộng sự [62]. Trong PSTMO, tích phân bình phương độ cong được dùng như một tiêu chí hình học khi tìm hình dạng và một thành phần chi phí sau chuẩn hóa. Vai trò này không đồng nhất với tối thiểu thời gian hoặc năng lượng cơ học.

Công thức 3 xác định độ cong κ và tích phân bình phương độ cong Eκ theo chiều dài cung. Eκ có đơn vị m⁻¹, không biểu thị năng lượng tiêu thụ; giá trị nhỏ hơn không có nghĩa độ cong nhỏ hơn tại mọi điểm. Trong tìm hình dạng, độ cong được tính từ đạo hàm Bézier tại các mẫu; Eκ được xấp xỉ bằng quy tắc hình thang với chiều dài dây cung. Thuật toán loại đường khi phát hiện đạo hàm suy biến, giá trị không hữu hạn hoặc đổi dấu độ cong ngoài dung sai tại các mẫu kiểm tra.

[Office Math được bảo toàn trong Word] (3)

C. Ràng buộc chuyển động và kiểm tra va chạm

Mô hình hai bánh chủ động độc lập cho phép biểu diễn vận tốc thân xe theo vận tốc từng bánh [63]. Các đường Dubins [64], phân loại hình học đường ngắn nhất bị chặn độ cong [65] và đường Reeds–Shepp có tiến–lùi [66] thuộc những bài toán khác với chuyển động quay tại chỗ của robot vi sai. PSTMO không áp một bán kính quay dương chung cho mọi trạng thái: các giới hạn theo độ cong dưới đây áp cho phần tịnh tiến, còn thao tác quay được mô hình hóa riêng.

Với khoảng cách hai bánh b, vận tốc tịnh tiến v và độ cong có dấu κ, vận tốc dài tại bánh trái và phải là vL = v(1 − bκ/2) và vR = v(1 + bκ/2). Chuyển tiếp tiến yêu cầu hai hệ số 1 ± bκ/2 không âm; quay tại chỗ được xét riêng. Công thức 4 kết hợp giới hạn vận tốc thân xe, vận tốc góc, gia tốc ngang và vận tốc bánh. Các lượt quét xuôi–ngược áp đặt tăng tốc, giảm tốc; vòng lặp tiếp tục hạ giới hạn vận tốc để thỏa gia tốc góc ước lượng giữa các mẫu. Hồ sơ không hội tụ bị loại. Tính khả thi theo mô hình này chưa xác định khả năng bám đường của bộ điều khiển.

[Office Math được bảo toàn trong Word] 

[Office Math được bảo toàn trong Word] (4)

Tại đoạn thẳng, các hạng chia cho |κ| trong công thức 4 không tạo thêm giới hạn hữu hạn. Chi phí tại tâm phải qua ngưỡng cấu hình; vùng hình bao đã định hướng phải không chứa ô vật cản gây va chạm hoặc ô chưa biết. Ngưỡng chi phí mở rộng quanh vật cản được áp tại tâm, không áp thêm cùng ngưỡng lên toàn đường bao. Kiểm tra chuyển tiếp nội suy cả vị trí và hướng giữa các mẫu; quay tại chỗ được lấy mẫu theo góc. Đây vẫn là kiểm tra hình bao tại các tư thế rời rạc, phụ thuộc độ phân giải và khoảng lấy mẫu, chưa bảo đảm an toàn liên tục.

D. Xây dựng và lựa chọn phương án chuyển động tại góc

Sau tiền xử lý, độ đổi hướng được xác định từ hai vectơ cạnh kề. Đỉnh có độ đổi hướng nhỏ hơn 5° được giữ làm điểm đi qua, không tạo dấu quay tại chỗ. Với các góc còn lại, hai dạng chuyển động được xét là chuyển tiếp tiến và dừng để quay tại chỗ. Bộ sinh Bézier chỉ nhận góc nhỏ hơn 170°; ngoài miền này, thuật toán chỉ còn phương án quay tại chỗ nếu khả thi.

PSTMO xét tối đa hai chiều dài cắt tại mỗi góc. Chiều dài ưu tiên là giá trị nhỏ nhất của hai cạnh kề và dmax. Chiều dài tương thích được chặn thêm bởi phần dành cho mỗi đầu của từng cạnh chung, bằng max(0, L − m)/2; cạnh nối trực tiếp điểm đầu hoặc đích được dùng toàn chiều dài. Thuật toán bỏ giá trị dưới dmin hoặc trùng chiều dài ưu tiên trong độ phân giải tìm kiếm. Cách tạo này dành chiều dài cho cả góc trước và góc sau.

Với mỗi chiều dài cắt, thuật toán xét α = {0,1; 0,2; 0,3; 0,4; 0,5}; nếu chưa có hình dạng khả thi thì xét thêm {0,15; 0,25; 0,35; 0,45}. Quanh giá trị có Eκ nhỏ nhất, khoảng tinh chỉnh nối hai mốc thô kề hoặc là ô thô chứa giá trị bổ sung; tại biên miền chỉ dùng khoảng một phía. Khoảng này được chia thành mười phần bằng nhau và bỏ mẫu trùng. Mỗi giá trị phải qua kiểm tra hình học, hình bao và tham số hóa thời gian. Sau tinh chỉnh, chỉ giữ một hình dạng có Eκ nhỏ nhất cho mỗi chiều dài cắt; khi bằng nhau trong dung sai, ưu tiên α nhỏ hơn. Hàm chi phí tổng hợp ở bước sau không được tối ưu trực tiếp trên toàn miền α.

Các chuyển tiếp sau tìm hình dạng được bổ sung đoạn thẳng để cùng bắt đầu và kết thúc trên cửa sổ có chiều dài cắt lớn nhất. Vận tốc biên chung được hạ lặp đến trạng thái có thể đạt, rồi tính lại hồ sơ; phương án không đáp ứng trạng thái biên bị loại. Tmin là thời gian dự đoán nhỏ nhất trong tập còn lại. Tp gồm thời gian tiến đến đỉnh, dừng–quay và rời đỉnh trên cùng cửa sổ với cùng vận tốc biên. Khi quay tại chỗ khả thi, nhánh chuyển tiếp chỉ mở nếu đạt mức giảm tối thiểu δT trong công thức 5.

[Office Math được bảo toàn trong Word] (5)

Trong nhánh này, thời gian của chuyển tiếp được giữ không vượt Tmin cộng khoảng thời gian cho phép đã cấu hình và không lớn hơn Tp − δT, bỏ qua dung sai số học. Nếu quay tại chỗ không khả thi, chỉ áp dụng giới hạn so với Tmin.

Hàm chi phí trong công thức 6 cung cấp chi phí cục bộ cho bước chọn chuỗi. Gọi cmax là cực đại, trên các tư thế kiểm tra chuyển tiếp, của giá trị lớn hơn giữa chi phí tại tâm và chi phí trên đường bao; đây là chỉ số chi phí bản đồ, không phải khoảng hở đo bằng mét. ωpeak là trị tuyệt đối vận tốc góc lớn nhất của hồ sơ dự đoán trên cửa sổ chung. Các thành phần không thứ nguyên là ρc = min(1, cmax/252), ρω = min(1, ωpeak/ωmax) và ρE = Eκ/(Eκ + Escale), với Escale = 1 m⁻¹. Ba trọng số có tổng bằng một và là cấu hình dùng trong nghiên cứu; chưa có bằng chứng xác lập bộ trọng số tối ưu.

[Office Math được bảo toàn trong Word] (6)

Điều kiện thời gian và khoảng cho phép lọc tập phương án trước khi so sánh chi phí, nên thời gian không phải một hạng bù trừ trong J. Phương án quay tại chỗ khả thi nhận chi phí quy ước 1 khi nhánh chuyển tiếp mở, và 0 khi nhánh đóng; không áp Eκ cho thao tác quay. Các chuyển tiếp qua bộ lọc và phương án quay khả thi được đưa vào quy hoạch động. Vì phải bảo đảm tương thích giữa các góc, một góc vẫn có thể được chọn quay tại chỗ dù có chuyển tiếp khả thi cục bộ.

E. Lựa chọn chuỗi chuyển tiếp và tạo đường tham chiếu

Bài toán lựa chọn dùng nguyên lý tối ưu theo giai đoạn của quy hoạch động [67]. Sau khi tập ứng viên đã cố định, ảnh hưởng của lựa chọn tại một góc đến góc kế tiếp được biểu diễn bằng chiều dài chiếm dụng trên cạnh chung. Điều này cho phép lưu chi phí tốt nhất đến mỗi trạng thái thay vì liệt kê mọi chuỗi tổ hợp. Phát biểu tối ưu chỉ có ý nghĩa với chi phí cộng và quan hệ tương thích đã định nghĩa, không bao gồm sai số điều khiển phát sinh khi thực thi.

Gọi Sᵢ là tập phương án khả thi được giữ tại góc i; mỗi phương án là một trạng thái của quy hoạch động. Chuyển tiếp dùng chiều dài cắt dᵢ, còn quay tại chỗ dùng chiều dài cắt bằng không. Chọn riêng phương án có J nhỏ nhất ở từng góc có thể làm các chuyển tiếp chồng lấn trên cạnh chung. Quy hoạch động chỉ nối các trạng thái tương thích và tối thiểu hóa tổng chi phí cục bộ. Công thức 7 xác định Fᵢ(k), tổng chi phí nhỏ nhất đến phương án k tại góc i; khởi tạo F₁(k) = J₁(k). Trạng thái không có tiền nhiệm đạt tới được nhận chi phí vô cùng. Truy vết từ trạng thái có chi phí nhỏ nhất ở góc cuối xác định chuỗi được chọn.

[Office Math được bảo toàn trong Word] (7)

Trong công thức 7, Aᵢ(k) = {j: dᵢ₋₁(j) + dᵢ(k) + m ≤ Lᵢ₋₁} là tập phương án tương thích tại góc liền trước. Khi chi phí bằng nhau trong sai số số học, thuật toán ưu tiên chuỗi có ít trạng thái quay tại chỗ hơn. Nếu vẫn bằng nhau, thuật toán chọn theo thứ tự đã xác định của các phương án. Với Kᵢ trạng thái tại lớp i, số phép tính tỷ lệ với ∑Kᵢ₋₁Kᵢ và bộ nhớ truy vết tỷ lệ với ∑Kᵢ. Tính tối ưu áp dụng cho đồ thị các phương án khả thi đã xây dựng.

Sau khi truy vết, các đoạn thẳng, chuyển tiếp và tư thế quay tại chỗ được ghép theo thứ tự đường đi. PSTMO kiểm tra vị trí đầu–cuối, giới hạn chuyển động, đường bao tại các mối nối và hướng cuối. Thuật toán từ chối đường nếu thiếu phương án khả thi, không có chuỗi tương thích hoặc đầu ra không vượt qua toàn bộ kiểm tra.

Hai chiều dài cắt và phép tìm kiếm phân cấp giới hạn số đường cong được đánh giá nhưng có thể bỏ sót hình dạng khả thi hoặc có chi phí thấp hơn. Tối ưu trên đồ thị không phải tối ưu toàn cục trên mọi đường liên tục hay thời gian thực thi. Tính G² thuộc các mối nối Bézier–đoạn thẳng đã xây dựng; chuỗi tư thế rời rạc, các điểm đi qua dưới ngưỡng và thao tác quay không tạo thành bảo đảm G² cho toàn bộ đầu ra. Phần III đánh giá riêng chuyển động dưới bộ điều khiển.

III. XÂY DỰNG KỊCH BẢN THỬ NGHIỆM VÀ PHÂN TÍCH

A. Môi trường mô phỏng và thiết kế đánh giá

1) Robot và chuỗi điều hướng

Kiến trúc Nav2 đã được trình bày trong The Marathon 2 [68]; tổng quan ROS 2 của Macenski và cộng sự [69] đặt các thuật toán điều hướng trong hệ phần mềm có nhiều mô-đun. Nghiên cứu này kế thừa kiến trúc đó để thay đổi riêng tầng hậu xử lý khi so sánh. Các nguồn kiến trúc không được dùng để gán phiên bản thư viện hoặc cấu hình cho những lượt thử; những thông tin này phải lấy từ hồ sơ của bộ dữ liệu.

Thử nghiệm sử dụng robot vi sai trong ROS 2 Jazzy, Nav2 và Gazebo. Đường sau hậu xử lý làm tham chiếu cho bộ điều khiển Regulated Pure Pursuit, viết tắt RPP [57]. Lệnh vận tốc đi qua bộ làm mượt vận tốc trước khi robot thực thi; bộ giám sát va chạm hoạt động trong cùng hệ điều hướng. Trong mỗi nhóm tuyến–bộ lập kế hoạch, các phương án dùng chung bản đồ, robot, bộ điều khiển và tiêu chí đích.

[Hình được bảo toàn trong Word]

Hình 3. Sơ đồ đường bao chữ nhật và khoảng cách bánh dùng trong mô hình PSTMO. O là gốc biểu diễn đường bao, x hướng về phía trước. Khoảng cách bánh hiệu dụng của mô-đun động lực học được báo riêng trong Bảng I; sơ đồ không phải ảnh chụp robot thực.

Hình 3 và Bảng I phân biệt hai khoảng cách bánh trong mô hình. PSTMO dùng 0,2548 m để kiểm tra động học; mô phỏng dẫn động dùng giá trị hiệu dụng 0,2834 m. Hai thông số được giữ theo cấu hình đã sinh dữ liệu. Sự khác biệt cần được tính đến khi đối chiếu giới hạn vận tốc bánh với đáp ứng chuyển động của robot.

BẢNG I. THAM SỐ ROBOT VÀ GIAO THỨC ĐIỀU HƯỚNG

| Đại lượng | Giá trị |
| Hệ phần mềm | ROS 2 Jazzy, Nav2, Gazebo |
| Kích thước hình chiếu bằng | 0,44 × 0,34 m |
| Bán kính bánh dẫn động | 0,0425 m |
| Khoảng cách bánh trong PSTMO | 0,2548 m |
| Khoảng cách bánh hiệu dụng trong mô phỏng | 0,2834 m |
| Độ phân giải bản đồ | 0,05 m/ô |
| RPP: tần số / khoảng nhìn trước | 20 Hz / 0,50 m |
| Vận tốc đặt / xoay hướng | 0,30 m/s / 0,25 rad/s |
| Bộ kiểm tra đích Nav2: vị trí / hướng | 0,06 m / 0,10 rad |
| Tiêu chí hoàn thành: vị trí / hướng | 0,10 m / 0,15 rad |
| Tiêu chí dừng: |v| / |ω| | ≤ 0,01 m/s / ≤ 0,02 rad/s |


2) Cấu hình PSTMO và tích hợp trong hệ điều hướng

Bảng II trình bày các giá trị cấu hình PSTMO dùng trong khảo sát. Sau các phép kiểm tra đầu ra ở Mục II-E, đường được chuyển tới RPP để tạo lệnh vận tốc bám đường. Thời gian dự đoán tại góc dùng để lựa chọn chuyển tiếp; thời gian thực thi đo toàn nhiệm vụ có phản hồi điều khiển. Hai đại lượng không được dùng thay thế cho nhau.

BẢNG II. CẤU HÌNH PSTMO TRONG NGHIÊN CỨU

| Thông số PSTMO | Giá trị |
| Chiều dài cắt d / tập α ban đầu | 0,02–0,80 m / {0,1; 0,2; 0,3; 0,4; 0,5} |
| Khoảng cách lấy mẫu kiểm tra / đầu ra | 0,02 / 0,05 m |
| Ngưỡng góc / khoảng dự phòng | 5° / 0,05 m (tự suy ra) |
| Mức giảm thời gian tối thiểu / khoảng thời gian cho phép | 0,15 / 10,0 s |
| Trọng số ρc / ρω / ρE | 0,15 / 0,10 / 0,75 |
| Hệ số Escale | 1 m⁻¹ |
| Vận tốc thẳng / góc cực đại | 0,30 m/s / 0,80 rad/s |
| Vận tốc bánh / gia tốc ngang cực đại | 0,36 m/s / 0,18 m/s² |
| Gia tốc / giảm tốc / gia tốc góc | 0,35 / 0,45 m/s² / 1,20 rad/s² |
| Tiền xử lý | Loại điểm trùng, xử lý dao động; không rút gọn bằng đường nhìn |
| Tìm đường cong | Phân cấp theo α với hai chiều dài cắt |


Ký hiệu δT là mức giảm thời gian tối thiểu so với quay tại chỗ. Khoảng thời gian cho phép so với Tmin được chặn thêm bởi điều kiện tại Mục II-D. Khoảng cách lấy mẫu đường đầu ra khác khoảng cách kiểm tra đường cong; khoảng dự phòng 0,05 m được suy ra từ cấu hình và độ phân giải bản đồ.

RPP sử dụng vận tốc đặt 0,30 m/s và khoảng nhìn trước cố định 0,50 m [57]. Điều chỉnh vận tốc theo độ cong và chi phí được tắt; kiểm tra va chạm, xoay hướng và giảm tốc gần đích vẫn hoạt động. Giá trị 0,25 rad/s trong Bảng I là vận tốc đặt cho xoay hướng, không phải giới hạn tuyệt đối của mọi lệnh vận tốc góc khi bám đường. Mô hình PSTMO và bộ làm mượt vận tốc dùng giới hạn 0,80 rad/s. Bộ làm mượt còn áp đặt tăng tốc, giảm tốc; giám sát va chạm có thể điều chỉnh lệnh trước khi tới robot. Do đó, thời gian dự đoán trong PSTMO không phải cam kết thời gian thực thi của hệ điều khiển.

3) Bản đồ và năm bài toán di chuyển

Hình 4 thể hiện năm tuyến trên bản đồ chiếm chỗ có độ phân giải 0,05 m/ô. Các đường gốc do Theta* tạo được dùng để minh họa vị trí từng tuyến. Bảng III liệt kê tọa độ đầu và đích. R01 chuyển lối qua giao cắt; R02 đi chéo dài; R03 chuyển lối hình chữ U; R04 đi chéo ngược; R05 rẽ từ hành lang ngang vào hành lang dọc.

[Hình được bảo toàn trong Word]

Hình 4. Vị trí năm tuyến trong cùng bản đồ mô phỏng. Mỗi màu là một đường gốc của Theta*; chấm tròn chỉ xuất phát và ngôi sao chỉ đích. Các đường có thể chồng lên nhau trên những hành lang dùng chung.

BẢNG III. TỌA ĐỘ ĐẦU VÀ ĐÍCH TRONG HỆ TỌA ĐỘ BẢN ĐỒ

| Tuyến | Xuất phát (x, y) m | Đích (x, y) m |
| R01 | -2,0; -2,8 | 2,0; 2,8 |
| R02 | -4,7; -2,8 | 4,7; 2,8 |
| R03 | -2,0; -2,8 | 2,0; -2,8 |
| R04 | -4,6; 2,8 | 2,0; -2,8 |
| R05 | -5,0; 0,0 | 0,0; 3,0 |


Hướng đích của R01–R05 lần lượt là 0,950547 rad, π/2, −π/2, −π/2 và π/2. Tư thế xuất phát của từng lượt được lưu trong hồ sơ thực nghiệm. Cả vị trí và hướng đều tham gia xác định nhiệm vụ. Robot tới gần tọa độ đích vẫn cần đáp ứng sai số hướng và điều kiện dừng để được ghi nhận hoàn thành.

Năm tuyến khảo sát các dạng chuyển hướng trên cùng một bố trí vật cản: R01 xét chuyển lối tại giao cắt; R02 xét hành trình dài với các đoạn gần thẳng; R03 xét chuỗi góc liên tiếp; R04 bổ sung chiều tiếp cận và chuỗi chuyển hướng khác; R05 tập trung chuyển hướng và tiếp cận đích trên tuyến ngắn. Các tuyến dùng chung một số hành lang. R04 khác R02 cả điểm đầu, điểm đích và hình học đường, nên không tạo thành thí nghiệm tách riêng ảnh hưởng của chiều tiếp cận.

4) Bộ lập kế hoạch và các phương án đối chứng

Năm bộ lập kế hoạch gồm NavFn A*, NavFn Dijkstra, Theta*, Smac 2D và Smac Hybrid. Hai cấu hình NavFn tạo đường trên lưới; Theta* bổ sung đường theo hướng tự do. Smac 2D và Smac Hybrid cung cấp thêm các cấu trúc đường đầu vào. Smac Hybrid sử dụng mô hình Dubins, bán kính quay nhỏ nhất 0,35 m và 72 ô hướng; chức năng làm mượt nội bộ được tắt. Giới hạn bán kính này thuộc bộ lập kế hoạch, không thay thế mô hình quay tại chỗ của PSTMO.

Tài liệu Smoother Server mô tả giao diện tác vụ và cơ chế nạp nhiều plugin cùng dùng tài nguyên bản đồ, hình bao và biến đổi tọa độ [70]. Ở cấp thuật toán, Simple cân bằng độ bám đường đầu vào với giảm dao động [71]; Savitzky–Golay sử dụng lọc đa thức cục bộ để xử lý các nhiễu nhỏ của đường [72]; Constrained tối ưu nhiều thành phần liên quan độ trơn, vật cản và độ cong [73]. Bốn nguồn này là snapshot tài liệu Rolling, dùng để giải thích vai trò các thành phần, không thay thế cấu hình Jazzy trong Bảng IV hoặc chứng minh các giá trị mặc định đã được dùng. Nền tảng bộ lọc và tài liệu Jazzy của đối chứng được trích riêng dưới đây.

Năm phương án gồm đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO. Đường gốc giữ nguyên đầu ra của bộ lập kế hoạch để đối chiếu thay đổi do hậu xử lý. Simple làm mượt lặp; Savitzky–Golay, viết tắt SG trong bảng và hình, dùng bộ lọc bình phương tối thiểu cục bộ [74]; Constrained tối ưu số các thành phần chất lượng đường [56]. Ba đối chứng làm mượt cho phép so sánh các cách xử lý khác nhau dưới cùng bộ điều khiển. Bảng IV trình bày cấu hình từng phương án.

BẢNG IV. CẤU HÌNH CÁC PHƯƠNG ÁN HẬU XỬ LÝ

| Phương án | Cấu hình đáng chú ý |
| Đường gốc | Không hậu xử lý; giữ đường của bộ lập kế hoạch |
| Simple | Tối đa 1.000 vòng; dung sai 10⁻¹⁰; tinh chỉnh hai lần |
| SG | Savitzky–Golay; tinh chỉnh hai lần |
| Constrained | Trọng số làm trơn 200.000; chi phí vật cản 0,015; trọng số khoảng cách và độ cong đều bằng 0; tối đa 70 vòng; hệ số giảm/tăng mẫu 1; giữ hướng đầu–cuối; tắt đi lùi |
| PSTMO | Tham số trong Bảng II; cùng RPP và đường bao robot |


Constrained được thử với trọng số độ cong và bán kính quay nhỏ nhất bằng không. Kết quả vì vậy chỉ đại diện cho cấu hình này. Các phương án dùng chung bản đồ và đường bao robot, nhưng có cơ chế kiểm tra nội bộ khác nhau. Nghiên cứu chưa tối ưu tham số cho từng phương án với cùng giới hạn số lần đánh giá tham số. Các khác biệt này được xem xét khi diễn giải so sánh.

5) Hai bộ dữ liệu và kiểm soát đầu vào

Ma trận khảo sát gồm 125 lượt, tương ứng 5 tuyến × 5 bộ lập kế hoạch × 5 phương án. Mỗi tổ hợp được thực thi một lần. Hình và chỉ số được lấy từ cùng bản ghi thực thi. Mã kiểm tra của đường gốc được đối chiếu trong từng nhóm để xác nhận các phương án nhận cùng đầu vào hình học.

Ma trận 125 lượt được dùng để phân tích mô tả theo tuyến và bộ lập kế hoạch. Năm bộ lập kế hoạch tạo năm đầu vào khác nhau, không phải năm lần lặp của một đầu vào. Do đó, các bảng theo tuyến báo trung bình mô tả, không gán khoảng tin cậy từ việc gộp các bộ lập kế hoạch.

Một lượt cho mỗi tổ hợp chưa xác định được biến thiên khi thực thi lại cùng nhiệm vụ. Vì vậy, bộ kiểm chứng lặp đánh giá mức độ duy trì của chênh lệch hiệu năng khi giữ cố định đầu vào và cấu hình. Constrained cung cấp đối chứng tối ưu số trong cùng Nav2 để đối chiếu với cách xây dựng và lựa chọn chuỗi chuyển tiếp của PSTMO. Phạm vi lặp chỉ gồm hai phương án này, không thay thế so sánh đủ năm phương án hoặc xác lập ưu thế so với các đối chứng còn lại.

Bộ kiểm chứng lặp gồm 250 lượt PSTMO và Constrained: 5 tuyến × 5 bộ lập kế hoạch × 2 phương án × 5 lần lặp. Bộ này dùng một phiên bản phần mềm và cấu hình cố định. Mỗi lượt khởi động mới hệ thống; bốn tiến trình chạy trong các miền ROS và phân vùng Gazebo tách biệt. Các lần lặp đo biến thiên thực thi với đầu vào cố định, không khảo sát nhiều giá trị khởi tạo của bộ sinh số giả ngẫu nhiên. Kết quả được phân tích riêng tại Mục III-H.

6) Tiêu chí hoàn thành và xử lý lỗi

Một lượt đạt khi tác vụ điều hướng thành công, sai số vị trí không vượt 0,10 m và sai số hướng không vượt 0,15 rad. Robot đồng thời phải dừng với |v| ≤ 0,01 m/s và |ω| ≤ 0,02 rad/s. Điều kiện dừng được kiểm tra qua ba vòng quan sát liên tiếp trong thời gian chờ tối đa 3 s. Sai số được tính từ trạng thái mô phỏng. Dung sai đánh giá sau thực thi được phân biệt với dung sai bộ kiểm tra đích Nav2 trong Bảng I.

Các lượt không hoàn thành nhiệm vụ được giữ trong mẫu số tính tỷ lệ thành công. Chính sách thử lại chỉ áp dụng cho lỗi khởi tạo hạ tầng. Bộ kiểm chứng lặp có 14 lượt cần ít nhất một lần thử lại loại này; hồ sơ lưu riêng các lần gián đoạn và kết quả đã chốt. Các lượt thất bại sau khi thực thi không được chạy lại để thay kết quả.

Phân tích ghép cặp các chỉ số liên tục chỉ sử dụng các cặp mà cả hai phương án đều đạt. Các lượt còn lại được báo riêng bằng trạng thái kết thúc và sai số đích. Quy tắc này phân biệt thời gian hoàn thành với thời gian bị hủy trước đích. Mỗi bảng và phép so sánh đều nêu rõ tập lượt được sử dụng.

7) Chỉ số đo và cách diễn giải

Toàn bộ 125 bản ghi có đủ đường tham chiếu, quỹ đạo thực thi, độ giật và chuỗi trạng thái. Các chỉ số của cùng lượt được trích từ cùng bản ghi. Quỹ đạo thực thi được lấy mẫu lại theo chiều dài cung với bước 0,05 m, rồi làm mượt bằng trung bình trượt đối xứng với hai mẫu mỗi phía, giữ hai đầu đường. Quy trình này được áp dụng thống nhất trước khi tính RMSE và tích phân bình phương độ cong của quỹ đạo thực thi.

Chiều dài đường tham chiếu L là tổng khoảng cách giữa các điểm đầu ra. Quãng đường thực thi D được tính từ các vị trí robot trước bước xử lý quỹ đạo. Ký hiệu Eκ,ref và Eκ,exec lần lượt chỉ tích phân bình phương độ cong của đường tham chiếu và quỹ đạo thực thi đã xử lý. Mức giảm Eκ,ref không mặc nhiên dẫn đến mức giảm tương ứng của Eκ,exec.

Giá trị xấp xỉ của tích phân bình phương độ cong được tính từ các bộ ba điểm liên tiếp. Với độ dài ba cạnh a, b, c và diện tích tam giác có dấu A, độ cong mẫu bằng 4A/(abc). Mỗi giá trị κ² được nhân với nửa tổng chiều dài hai cạnh kề rồi cộng trên toàn đường; các bộ ba suy biến được bỏ qua theo ngưỡng số học. Độ cong cực đại κmax là cực đại trị tuyệt đối độ cong ước lượng trên các điểm đầu ra chưa lấy mẫu lại. Các chỉ số này phụ thuộc mật độ điểm.

Để kiểm tra ảnh hưởng lấy mẫu, phần tịnh tiến của đường tham chiếu được tách tại các tư thế quay tại chỗ và lấy mẫu lại với bước 0,05 m, cho các chỉ số κt,max và Eκ,t. Dấu quay tại chỗ được nhận diện khi dịch chuyển không quá 0,0001 m và thay đổi hướng ít nhất 5°. Đây là dấu trong đường tham chiếu, không phải số lần robot thực sự xoay khi điều khiển.

Sai số bám đường là khoảng cách nhỏ nhất từ vị trí xₖ trên quỹ đạo thực thi đã xử lý đến các đoạn của đường tham chiếu P. Với N vị trí, căn trung bình bình phương sai số, viết tắt RMSE, được xác định trong công thức 8. Mỗi phương án dùng đường tham chiếu riêng; RMSE phản ánh khả năng bám đường, không phải sai số định vị. Do lấy mẫu theo chiều dài cung, đây là chỉ số trên không gian đường đi, không phải trung bình theo thời gian; nó không mô tả riêng sai số hướng khi quay tại chỗ.

[Office Math được bảo toàn trong Word] (8)

Sai số bám cực đại emax dùng cùng phép đo khoảng cách và quỹ đạo đã xử lý như RMSE. Sai số cuối ep và eψ lần lượt là khoảng cách tới vị trí đích và trị tuyệt đối sai lệch hướng đã quy về [0; π], tính từ trạng thái cuối của mô phỏng. Sai số cuối phản ánh kết quả dừng tại đích, khác với sai số bám trên hành trình.

Thời gian thực thi T được đo bằng đồng hồ mô phỏng, gồm thời gian điều hướng và thời gian chờ robot dừng. Thời lượng hậu xử lý do tác vụ làm mượt Nav2 trả về, được tính bằng đồng hồ của nút phần mềm [75]. Khi dùng thời gian mô phỏng, đại lượng này không phải thời gian CPU hoặc thời gian thực độc lập của máy tính.

Khoảng hở c là khoảng cách nhỏ nhất giữa đường bao đã định hướng và vật cản trên bản đồ tĩnh. Phép đo trên đường tham chiếu nội suy tư thế với bước tịnh tiến 0,05 m và bước hướng 5°. Đường bao được lấy mẫu với bước không quá 0,025 m; trường khoảng cách được hiệu chỉnh nửa đường chéo ô. Ô chưa biết và ô có mức chiếm chỗ từ 65 trở lên được xem là vật cản. Độ phân giải bản đồ giới hạn cách diễn giải những chênh lệch khoảng hở rất nhỏ.

Phân vị 95% của trị tuyệt đối độ giật tịnh tiến mô tả biến thiên gia tốc. Độ giật được ước lượng từ chuỗi vận tốc mô phỏng bằng sai phân và phụ thuộc nhịp lấy mẫu. Vận tốc bánh tương đương được suy từ v ± bω/2 với b = 0,2548 m; gia tốc ngang là |vω|. Gia tốc và độ giật dùng bước thời gian hợp lệ từ 0,0001 đến 0,5 s. Cực đại chỉ mô tả các mẫu đã ghi, chưa chứng minh ràng buộc liên tục giữa các mẫu. Hệ thống còn ghi số lần giám sát va chạm can thiệp và số mẫu đường bao va chạm; chưa đo điện năng, rung tải hàng hoặc lực tiếp xúc.

Các chỉ số hình học dùng toàn bộ 25 đường của mỗi phương án; trung bình thực thi dùng tập lượt đạt và ghi rõ mẫu số. So sánh ghép cặp chỉ dùng các lượt cùng đầu vào mà cả hai phương án đạt. Trong phân tích bộ lặp, thời gian, RMSE và Eκ,exec là ba chỉ số chính được hiệu chỉnh đa kiểm định; các chỉ số còn lại bổ sung thông tin về đánh đổi. Phương pháp thống kê được trình bày tại Mục III-H.

B. R01 Chuyển lối tại giao cắt

1) Mục tiêu và hình học của tuyến

R01 chuyển từ hành lang phía dưới tại x = −2 m sang hành lang phía trên tại x = 2 m qua hành lang ngang. Tuyến này khảo sát các mối nối giữa đoạn thẳng và vùng rẽ, nơi đường theo lưới có thể chứa nhiều đoạn ngắn với độ cong cục bộ lớn.

Hình 5 so sánh năm đường tham chiếu của Theta*, bộ lập kế hoạch dùng thống nhất trong các hình R01–R05. Vùng phóng to được chọn tại điểm trên đường PSTMO cách xa nhất các đỉnh đường gốc. Đây là quy tắc minh họa khác biệt hình học, không phải lựa chọn theo mức cải thiện thực thi.

BẢNG V. KẾT QUẢ MÔ TẢ R01 TRÊN NĂM BỘ LẬP KẾ HOẠCH

| Phương án | Đạt | T (s) | e (mm) | Eκ,ref (m⁻¹) | c (cm) |
| Đường gốc | 5/5 | 43,50 | 22,7 | 146,25 | 19,4 |
| Simple | 5/5 | 41,49 | 23,2 | 13,91 | 21,4 |
| SG | 5/5 | 42,88 | 25,3 | 35,65 | 20,9 |
| Constrained | 5/5 | 43,21 | 24,4 | 18,62 | 23,5 |
| PSTMO | 5/5 | 39,74 | 21,9 | 2,38 | 15,7 |


T và e là trung bình các lượt đạt; Eκ,ref và c lấy trên năm đường tham chiếu. Eκ,ref có đơn vị m⁻¹; c là trung bình của khoảng hở nhỏ nhất từng đường, không phải khoảng hở nhỏ nhất của cả nhóm. SG viết tắt Savitzky–Golay.

[Hình được bảo toàn trong Word]

Hình 5. R01 với Theta*: (a) đường tham chiếu trên toàn tuyến; (b) vùng phóng to. Cùng màu và kiểu nét được dùng xuyên suốt các tuyến. Đây là đường tham chiếu, không phải quỹ đạo thực thi của robot.

2) Hình học đường và kết quả thực thi

Bảng V cho thấy cả 25 lượt R01 đều hoàn thành. PSTMO có thời gian trung bình 39,74 s, trong khi đường gốc có giá trị 43,50 s. Simple đạt 41,49 s, Savitzky–Golay đạt 42,88 s và Constrained đạt 43,21 s. RMSE của PSTMO là 21,9 mm, so với 24,4 mm của Constrained. PSTMO có thời gian và RMSE trung bình thấp hơn Constrained trên tuyến này.

Tích phân bình phương độ cong của đường tham chiếu PSTMO là 2,38 m⁻¹; các giá trị tương ứng của Constrained và đường gốc là 18,62 m⁻¹ và 146,25 m⁻¹. Kết quả xác nhận thay đổi hình học rõ rệt, trong khi mức giảm thời gian nhỏ hơn về tỷ lệ. Hình 6 đối chiếu trực tiếp thời gian và RMSE theo từng bộ lập kế hoạch. Cách so sánh này cho phép đánh giá phần thay đổi hình học đi kèm thay đổi thực thi.

BẢNG VI. CHIỀU DÀI, ĐỘ CONG CỰC ĐẠI VÀ CHỈ SỐ BỔ SUNG TẠI R01

| Phương án | L (m) | κmax (m⁻¹) | D (m) | emax (mm) | τ (ms) |
| Đường gốc | 7,711 | 24,46 | 7,566 | 62,8 | 0,0 |
| Simple | 7,640 | 10,13 | 7,526 | 57,4 | 0,6 |
| SG | 7,688 | 16,88 | 7,559 | 63,1 | 0,0 |
| Constrained | 7,703 | 9,82 | 7,570 | 68,2 | 25,8 |
| PSTMO | 7,541 | 2,03 | 7,464 | 53,3 | 102,0 |


L: chiều dài đường tham chiếu; κmax: độ cong tuyệt đối cực đại chưa lấy mẫu lại; D: quãng đường thực thi; emax: sai số bám đường cực đại; τ: thời lượng hậu xử lý. L, κmax và τ dùng năm lượt; D và emax chỉ dùng lượt đạt.

Bảng VI cho thấy PSTMO có chiều dài tham chiếu và quãng đường thực thi thấp nhất. Độ cong cực đại 2,03 m⁻¹ thấp hơn cả bốn đối chứng; sai số bám cực đại 53,3 mm cũng thấp nhất. Các thay đổi này cùng xuất hiện, nên chưa thể quy riêng mức giảm thời gian cho độ cong.

[Hình được bảo toàn trong Word]

Hình 6. R01: thời gian thực thi và RMSE của đủ năm phương án trên từng bộ lập kế hoạch. Các cột là một lượt thực thi, không phải trung bình nhiều lần lặp; không vẽ thanh sai số.

Theo thứ tự đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO, Eκ,exec lần lượt là 2,659; 2,208; 2,517; 2,673; 1,613 m⁻¹; phân vị 95% của trị tuyệt đối độ giật lần lượt là 2,521; 2,428; 2,404; 2,471; 2,485 m/s³. PSTMO có Eκ,exec thấp nhất nhưng độ giật cao hơn cả ba đối chứng làm mượt. Như vậy, cải thiện hình học thực thi chưa đi kèm giảm tương ứng biến thiên gia tốc.

3) Đánh đổi khoảng hở và diễn giải

Trong năm cặp theo bộ lập kế hoạch, PSTMO nhanh hơn đường gốc, Savitzky–Golay và Constrained ở cả năm cặp; nhanh hơn Simple ở bốn cặp. Tuy nhiên, so sánh ghép cặp cho thấy RMSE tăng so với Constrained khi dùng Theta* và Smac Hybrid, lần lượt 1,87 và 0,97 mm. Như vậy, lợi thế thời gian nhất quán hơn lợi thế bám đường ở R01.

Khoảng hở nhỏ nhất trung bình của PSTMO là 15,7 cm, so với 23,5 cm của Constrained. Đường có tích phân bình phương độ cong thấp hơn nhưng sử dụng phần hành lang gần vật cản hơn. Đây là đánh đổi hình học quan sát được; mức đóng góp của từng trọng số mục tiêu chưa được tách riêng.

Thời lượng hậu xử lý của PSTMO ở R01 khoảng 0,102 s, so với 0,026 s của Constrained. Lợi ích thời gian thực thi đi kèm chi phí hậu xử lý cao hơn. Hai đại lượng có phạm vi đo riêng theo Mục III-A, nên không lấy hiệu trực tiếp để xác định mức giảm độ trễ toàn hệ thống.

Sai số vị trí cuối trung bình của năm phương án theo thứ tự trên là 35,7; 36,1; 37,0; 33,6; 32,5 mm; sai số hướng tương ứng là 0,075; 0,083; 0,072; 0,076; 0,079 rad, chỉ tính các lượt đạt. PSTMO có sai số vị trí thấp nhất nhưng sai số hướng không thấp nhất. R01 vì vậy cho thấy lợi ích về thời gian, bám đường và vị trí dừng, đi kèm khoảng hở nhỏ hơn; chưa có ưu thế đồng thời trên mọi tiêu chí.

C. R02 Di chuyển chéo dài

1) Tác động của chiều dài và các đoạn thẳng

R02 nối (−4,7; −2,8) m với (4,7; 2,8) m qua hành lang ngang, gồm các đoạn gần thẳng dài hơn R01. Tuyến này khảo sát hiệu quả hậu xử lý khi chuyển hướng chỉ chiếm một phần hành trình; thời gian toàn nhiệm vụ còn bao gồm di chuyển thẳng và tiếp cận đích.

Hình 7 cho thấy các đường gần chồng lên nhau trên toàn tuyến nhưng tách nhau tại vùng chuyển hướng. Vì khác biệt hình học tập trung cục bộ, Bảng VII và Hình 8 được dùng để đối chiếu với kết quả thực thi trên cả năm bộ lập kế hoạch.

BẢNG VII. KẾT QUẢ MÔ TẢ R02 TRÊN NĂM BỘ LẬP KẾ HOẠCH

| Phương án | Đạt | T (s) | e (mm) | Eκ,ref (m⁻¹) | c (cm) |
| Đường gốc | 5/5 | 59,57 | 21,3 | 192,21 | 19,5 |
| Simple | 5/5 | 56,40 | 21,1 | 5,76 | 23,3 |
| SG | 5/5 | 58,37 | 19,9 | 20,86 | 22,6 |
| Constrained | 5/5 | 59,06 | 21,7 | 36,48 | 22,9 |
| PSTMO | 5/5 | 55,39 | 24,2 | 2,48 | 19,3 |


T và e lấy trên lượt đạt; Eκ,ref và c lấy trên đường tham chiếu. Cả 25 lượt R02 đều đạt. Các giá trị c là trung bình của cực tiểu từng đường.

[Hình được bảo toàn trong Word]

Hình 7. R02 với Theta*: đường tham chiếu toàn tuyến và vùng phóng to theo cùng quy tắc chọn vùng như R01. Trục tọa độ có đơn vị mét; tỷ lệ hiển thị toàn tuyến và vùng phóng to khác nhau.

2) Hiệu quả thời gian không đồng nghĩa cải thiện mọi chỉ số

PSTMO đạt thời gian trung bình 55,39 s, thấp nhất trong năm phương án ở R02. Đường gốc đạt 59,57 s, Savitzky–Golay 58,37 s và Constrained 59,06 s. Simple đạt 56,40 s, gần PSTMO hơn các đối chứng còn lại. Vì vậy, phần lợi ích bổ sung so với Simple nhỏ hơn lợi ích so với đường chưa xử lý.

Tuy nhiên, RMSE trung bình của PSTMO là 24,2 mm, trong khi đường gốc là 21,3 mm; ba đối chứng làm mượt cũng có RMSE thấp hơn PSTMO. Simple, Savitzky–Golay và Constrained lần lượt đạt 21,1; 19,9 và 21,7 mm. Tích phân bình phương độ cong của đường tham chiếu PSTMO là 2,48 m⁻¹. Trường hợp này cho thấy giảm tích phân bình phương độ cong chưa bảo đảm giảm sai số bám đường dưới cấu hình RPP đã dùng.

BẢNG VIII. CHIỀU DÀI, ĐỘ CONG CỰC ĐẠI VÀ CHỈ SỐ BỔ SUNG TẠI R02

| Phương án | L (m) | κmax (m⁻¹) | D (m) | emax (mm) | τ (ms) |
| Đường gốc | 11,970 | 20,97 | 11,847 | 58,7 | 0,0 |
| Simple | 11,888 | 2,91 | 11,810 | 49,1 | 1,2 |
| SG | 11,934 | 6,89 | 11,841 | 53,5 | 0,0 |
| Constrained | 11,957 | 10,27 | 11,846 | 58,7 | 30,0 |
| PSTMO | 11,816 | 2,07 | 11,763 | 51,5 | 114,0 |


Ký hiệu như Bảng VI. L, κmax và τ dùng năm lượt; D và emax chỉ dùng lượt đạt.

Trong Bảng VIII, PSTMO có đường tham chiếu và quãng đường thực thi ngắn nhất. Tuy nhiên, sai số cực đại 51,5 mm vẫn cao hơn Simple, 49,1 mm. Vì vậy, đường ngắn hơn và độ cong cực đại thấp hơn chưa đồng nghĩa bám đường chính xác hơn.

[Hình được bảo toàn trong Word]

Hình 8. R02: thời gian và RMSE theo bộ lập kế hoạch. Sự khác biệt giữa hai hàng chỉ số cho phép nhận diện trường hợp nhanh hơn nhưng bám đường kém hơn; không suy ra ưu thế toàn diện từ một hàng duy nhất.

So sánh theo bộ lập kế hoạch cho thấy PSTMO nhanh hơn Constrained ở bốn trong năm cặp. Mức giảm lớn nhất là 8,32 s với NavFn A*, trong khi Smac Hybrid chậm hơn 0,49 s. RMSE tăng với Theta*, Smac 2D và Smac Hybrid. Khi so với Simple, PSTMO chỉ nhanh hơn ở 2/5 cặp dù thời gian trung bình thấp hơn; RMSE cao hơn ở cả năm cặp. Như vậy, thời gian trung bình thấp hơn không đồng nghĩa nhanh hơn trên đa số đầu vào.

Theo thứ tự đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO, Eκ,exec lần lượt là 2,976; 2,020; 2,761; 2,913; 1,536 m⁻¹; phân vị 95% của trị tuyệt đối độ giật lần lượt là 2,349; 2,157; 2,294; 2,276; 2,120 m/s³. PSTMO có Eκ,exec và độ giật thấp nhất, trong khi RMSE cao nhất. Các chỉ số phản ánh những khía cạnh khác nhau; chuyển động có độ giật thấp hơn vẫn có thể lệch xa đường tham chiếu hơn.

3) Khoảng hở và ý nghĩa vận hành

Khoảng hở nhỏ nhất trung bình của PSTMO là 19,3 cm, so với 22,9 cm của Constrained. Mức giảm khoảng hở đi cùng giảm thời gian nhưng tăng RMSE. Khi lựa chọn phương án cho hành lang hẹp, cần đối chiếu cả khoảng hở và sai số bám đường. Trong thử nghiệm này, phép kiểm tra đường tham chiếu không phát hiện mẫu đường bao giao cắt vật cản.

Tại R02, PSTMO giảm thời gian trung bình nhưng tăng RMSE so với cả bốn đối chứng. Do đó, kết quả chưa hỗ trợ lựa chọn PSTMO nếu yêu cầu ưu tiên là sai số bám đường trung bình nhỏ nhất. Dữ liệu chưa tách được ảnh hưởng riêng của hình dạng đường và đáp ứng điều khiển.

Sai số vị trí cuối trung bình của năm phương án theo thứ tự trên là 37,1; 36,4; 37,5; 42,4; 37,0 mm; sai số hướng tương ứng là 0,086; 0,081; 0,083; 0,074; 0,083 rad, chỉ tính các lượt đạt. PSTMO gần Simple về sai số vị trí nhưng có sai số hướng lớn hơn Constrained. Sai số cuối và RMSE phản ánh hai khía cạnh khác nhau. Trên R02, thời gian trung bình giảm nhưng sai số bám đường trung bình tăng.

D. R03 Chuyển lối hình chữ U

1) Chuỗi góc liên tiếp và khả năng tương thích

R03 bắt đầu tại (−2,0; −2,8) m và kết thúc tại (2,0; −2,8) m, với hướng đích −π/2. Robot đi vào hệ hành lang rồi trở về hàng ngang xuất phát ở một lối khác. Các góc rẽ liên tiếp dùng chung chiều dài của những đoạn nối. Vì vậy, tăng chiều dài cắt ở một góc có thể hạn chế lựa chọn tại góc kế tiếp.

[Hình được bảo toàn trong Word]

Hình 9. R03 với Theta*: đường chữ U và vùng chuyển hướng được phóng to. Hình biểu diễn một bộ lập kế hoạch cố định; trường hợp không đạt ở tuyến này thuộc Smac Hybrid và được báo riêng.

Hình 9 minh họa việc ghép các chuyển tiếp liên tiếp trên đường chữ U của Theta*. Kết quả thuộc toàn bộ PSTMO; chưa thể quy riêng cho quy hoạch động vì chưa có thí nghiệm loại bỏ thành phần.

BẢNG IX. KẾT QUẢ MÔ TẢ R03 TRÊN NĂM BỘ LẬP KẾ HOẠCH

| Phương án | Đạt | T (s) | e (mm) | Eκ,ref (m⁻¹) | c (cm) |
| Đường gốc | 5/5 | 43,69 | 30,1 | 111,06 | 16,5 |
| Simple | 5/5 | 42,34 | 27,0 | 5,78 | 19,6 |
| SG | 5/5 | 43,68 | 27,6 | 16,54 | 19,8 |
| Constrained | 5/5 | 44,15 | 29,0 | 22,93 | 21,4 |
| PSTMO | 4/5 | 41,94 | 27,2 | 5,47 | 18,8 |


PSTMO có 4/5 lượt đạt; các phương án còn lại có 5/5. T và e của PSTMO chỉ lấy trên bốn lượt đạt. Eκ,ref và c vẫn lấy trên đủ năm đường; không diễn giải chênh lệch T giữa các trung bình này như một ước lượng ghép cặp.

2) Thời gian thực thi và trường hợp không đạt dung sai đích

Bảng IX ghi nhận PSTMO đạt 4/5 lượt, trong khi các phương án còn lại đạt 5/5. Thời gian trung bình của các lượt PSTMO đạt là 41,94 s; Simple đạt 42,34 s và Constrained 44,15 s. Các trung bình này dùng số lượt khác nhau. Vì vậy, so sánh ghép cặp với PSTMO chỉ dùng bốn đầu vào có cả hai phương án đạt; lượt Smac Hybrid được giữ trong tỷ lệ thành công. Bảng X bổ sung chỉ số hình học và thực thi của cả năm phương án.

Lượt PSTMO với Smac Hybrid hoàn tất tác vụ điều hướng nhưng có sai số vị trí cuối 0,102385 m. Giá trị này vượt ngưỡng 0,10 m khoảng 2,385 mm, nên lượt được phân loại không đạt. Hình 10 đánh dấu riêng kết quả này. Việc báo đồng thời trạng thái và sai số cho thấy mức vượt ngưỡng cụ thể, thay vì chỉ ghi một nhãn thất bại.

[Hình được bảo toàn trong Word]

Hình 10. R03: cột gạch và dấu × đánh dấu PSTMO/Smac Hybrid không đạt. Giá trị ghi ở cột đó chỉ mô tả trạng thái kết thúc; không được tính vào trung bình hoàn thành hoặc dùng để kết luận thời gian tốt hơn.

BẢNG X. CHIỀU DÀI, ĐỘ CONG CỰC ĐẠI VÀ CHỈ SỐ BỔ SUNG TẠI R03

| Phương án | L (m) | κmax (m⁻¹) | D (m) | emax (mm) | τ (ms) |
| Đường gốc | 7,096 | 21,14 | 6,898 | 76,0 | 0,0 |
| Simple | 7,005 | 2,89 | 6,848 | 63,9 | 1,8 |
| SG | 7,068 | 6,73 | 6,902 | 68,9 | 0,0 |
| Constrained | 7,109 | 10,12 | 6,929 | 74,7 | 27,0 |
| PSTMO | 6,981 | 3,17 | 6,840 | 62,7 | 103,2 |


Ký hiệu như Bảng VI. L, κmax và τ dùng năm lượt; D và emax chỉ dùng lượt đạt.

Bảng X cho thấy chiều dài tham chiếu PSTMO ngắn nhất, nhưng độ cong cực đại 3,17 m⁻¹ cao hơn Simple, 2,89 m⁻¹. Quãng đường thực thi và sai số cực đại của PSTMO chỉ dùng bốn lượt đạt; chênh lệch với trung bình năm lượt của đối chứng cần được đọc cùng giới hạn mẫu này.

Theo thứ tự đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO, Eκ,exec lần lượt là 4,511; 4,037; 4,577; 4,659; 4,023 m⁻¹; phân vị 95% của trị tuyệt đối độ giật lần lượt là 2,533; 2,530; 2,421; 2,418; 2,540 m/s³. Eκ,exec của PSTMO gần Simple, còn độ giật cao hơn các đối chứng trên tập lượt đạt riêng. Kết quả không hỗ trợ nhận định PSTMO cải thiện đồng thời hình học và biến thiên gia tốc tại R03.

3) So sánh các phương án và đánh đổi

Tích phân bình phương độ cong trung bình của đường tham chiếu là 5,47 m⁻¹ với PSTMO, 5,78 m⁻¹ với Simple và 22,93 m⁻¹ với Constrained. Chênh lệch hình học giữa PSTMO và Simple nhỏ hơn rõ rệt so với R01. RMSE trên các lượt đạt cũng gần nhau: 27,2 mm với PSTMO và 27,0 mm với Simple. Kết quả này cho thấy mức cải thiện của toàn bộ PSTMO thay đổi theo cấu trúc đường đầu vào; dữ liệu chưa tách riêng tác dụng của chuyển tiếp Bézier.

Trên bốn cặp có cả hai phương án đạt, PSTMO nhanh hơn đường gốc, Savitzky–Golay và Constrained ở cả bốn cặp; nhanh hơn Simple ở ba cặp. RMSE chỉ thấp hơn Simple ở một cặp, trong khi thấp hơn Constrained ở ba cặp. Ngoại lệ khi so với Constrained là Theta*, có RMSE tăng 5,36 mm. Tốc độ hoàn thành và độ bám đường tiếp tục cho thứ hạng khác nhau.

Khoảng hở nhỏ nhất trung bình của PSTMO là 18,8 cm, so với Constrained 21,4 cm. Lượt không đạt dung sai đích cho thấy kiểm tra hình học không thay thế đánh giá chuyển động thực. Khả năng duy trì hiệu năng khi lặp được xét riêng tại Mục III-H.

Sai số vị trí cuối trung bình của năm phương án theo thứ tự trên là 86,8; 86,5; 82,3; 83,3; 89,6 mm; sai số hướng tương ứng là 0,080; 0,069; 0,056; 0,056; 0,039 rad, chỉ tính các lượt đạt. Trên tập lượt đạt riêng, PSTMO có sai số hướng thấp nhất nhưng sai số vị trí cao nhất. Cùng với lượt vượt dung sai, kết quả R03 cho thấy lợi ích thời gian cần được cân nhắc với yêu cầu dừng chính xác, thay vì chỉ dựa vào độ cong.

E. R04 Chuyển lối chéo ngược

1) Đổi chiều tiếp cận và vai trò của tư thế đích

R04 nối (−4,6; 2,8) m với (2,0; −2,8) m và kết thúc ở hướng −π/2. Tuyến này bổ sung trường hợp tiếp cận hành lang theo chiều khác, với chuỗi chuyển hướng và tư thế đích khác R02. Kết quả mô tả toàn bộ nhiệm vụ, không tách riêng ảnh hưởng của thứ tự các góc rẽ.

Hình 11 thể hiện khác biệt cục bộ của năm đường tham chiếu với Theta*. Bảng XI và Hình 12 mở rộng đối chiếu sang cả năm bộ lập kế hoạch, nhằm kiểm tra liệu khác biệt nhỏ trên hình toàn tuyến có đi kèm thay đổi thời gian và RMSE hay không.

[Hình được bảo toàn trong Word]

Hình 11. R04 với Theta*: toàn tuyến chéo ngược và vùng phóng to. Việc đảo chiều tiếp cận không làm tuyến này trở thành bản sao đối xứng của R02.

BẢNG XI. KẾT QUẢ MÔ TẢ R04 TRÊN NĂM BỘ LẬP KẾ HOẠCH

| Phương án | Đạt | T (s) | e (mm) | Eκ,ref (m⁻¹) | c (cm) |
| Đường gốc | 4/5 | 52,12 | 31,3 | 83,51 | 15,8 |
| Simple | 5/5 | 49,06 | 28,4 | 5,99 | 19,9 |
| SG | 5/5 | 50,46 | 28,8 | 18,00 | 20,3 |
| Constrained | 5/5 | 50,83 | 30,9 | 28,31 | 21,6 |
| PSTMO | 5/5 | 47,71 | 24,4 | 2,89 | 19,3 |


Đường gốc có 4/5 lượt đạt; bốn phương án hậu xử lý có 5/5. T và e của đường gốc dùng bốn lượt đạt, còn Eκ,ref và c vẫn dùng đủ năm đường. So sánh PSTMO–Constrained không bị thiếu cặp ở R04.

2) Kết quả thời gian và bám đường

PSTMO hoàn thành cả năm lượt R04, đạt thời gian trung bình 47,71 s và RMSE 24,4 mm. Các giá trị tương ứng của Simple là 49,06 s và 28,4 mm; Savitzky–Golay là 50,46 s và 28,8 mm. Constrained đạt 50,83 s và 30,9 mm. Khác với R02, PSTMO có cả thời gian và RMSE trung bình thấp hơn các đối chứng làm mượt.

Tích phân bình phương độ cong của đường PSTMO là 2,89 m⁻¹, so với Simple 5,99 m⁻¹ và Constrained 28,31 m⁻¹. Khoảng hở nhỏ nhất trung bình giảm còn 19,3 cm, trong khi Constrained đạt 21,6 cm. Như vậy, cải thiện đồng thời thời gian và RMSE vẫn đi kèm giảm khoảng cách tới vật cản.

[Hình được bảo toàn trong Word]

Hình 12. R04: thời gian và RMSE trên từng bộ lập kế hoạch. Đường gốc/Theta* được đánh dấu không đạt do sai số vị trí cuối vượt ngưỡng; các đường đã làm mượt đều hoàn thành trong ma trận một lượt này.

BẢNG XII. CHIỀU DÀI, ĐỘ CONG CỰC ĐẠI VÀ CHỈ SỐ BỔ SUNG TẠI R04

| Phương án | L (m) | κmax (m⁻¹) | D (m) | emax (mm) | τ (ms) |
| Đường gốc | 9,737 | 19,98 | 9,595 | 64,9 | 0,0 |
| Simple | 9,662 | 2,80 | 9,531 | 57,4 | 1,2 |
| SG | 9,716 | 5,94 | 9,564 | 58,3 | 0,0 |
| Constrained | 9,753 | 8,42 | 9,576 | 65,3 | 29,4 |
| PSTMO | 9,618 | 2,29 | 9,491 | 52,3 | 93,6 |


Ký hiệu như Bảng VI. L, κmax và τ dùng năm lượt; D và emax chỉ dùng lượt đạt.

Bảng XII cho thấy PSTMO có chiều dài tham chiếu và độ cong cực đại thấp nhất. Trong bốn phương án làm mượt đều đạt 5/5 lượt, PSTMO cũng có quãng đường thực thi và sai số cực đại nhỏ nhất. Kết quả củng cố nhận xét về bám đường, nhưng chưa phản ánh độ chính xác cuối.

Theo thứ tự đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO, Eκ,exec lần lượt là 3,607; 2,681; 3,140; 3,283; 2,049 m⁻¹; phân vị 95% của trị tuyệt đối độ giật lần lượt là 2,374; 2,265; 2,335; 2,325; 2,271 m/s³. PSTMO có Eκ,exec thấp nhất, nhưng độ giật 2,271 m/s³ nhỉnh hơn Simple, 2,265 m/s³. Chênh lệch nhỏ này chỉ là mô tả một lượt mỗi đầu vào, chưa xác định khác biệt có tính lặp lại.

So sánh theo bộ lập kế hoạch cho thấy PSTMO nhanh hơn Constrained ở bốn cặp; với Smac Hybrid, thời gian tăng 0,05 s. RMSE thấp hơn Constrained ở cả năm cặp. So với Savitzky–Golay, PSTMO cũng giảm cả thời gian và RMSE ở năm cặp. So với Simple, PSTMO chỉ nhanh hơn ở hai cặp nhưng có RMSE thấp hơn ở bốn cặp. Trung bình toàn tuyến cần được đọc cùng phân bố theo bộ lập kế hoạch.

3) Trường hợp sát ngưỡng và tiêu chí hoàn thành

Đường gốc với Theta* có sai số vị trí cuối 0,100234 m, vượt ngưỡng 0,10 m khoảng 0,234 mm. Lượt này được phân loại không đạt theo cùng tiêu chí với PSTMO tại R03. Mức vượt nhỏ mô tả độ gần ngưỡng của kết quả, chưa chứng minh một khác biệt cơ học đáng kể. Tỷ lệ thành công của ma trận vẫn là 123/125.

Hai trường hợp sát ngưỡng ở R03 và R04 cho thấy cần báo sai số liên tục bên cạnh tỷ lệ hoàn thành. Dữ liệu xác định được mức vượt dung sai nhưng chưa đủ để quy nguyên nhân cho hình dạng đường hoặc một thành phần điều khiển riêng.

Sai số vị trí cuối trung bình của năm phương án theo thứ tự trên là 82,6; 85,2; 81,8; 85,6; 87,4 mm; sai số hướng tương ứng là 0,075; 0,066; 0,065; 0,067; 0,076 rad, chỉ tính các lượt đạt. PSTMO có cả hai sai số cuối lớn nhất trên tập lượt đạt riêng. R04 cho thấy giảm thời gian và sai số bám đường không bảo đảm cải thiện độ chính xác dừng, dù cả năm lượt PSTMO đều hoàn thành.

F. R05 Rẽ từ hành lang ngang vào hành lang dọc

1) Nhiệm vụ ngắn với chuyển hướng tập trung

R05 bắt đầu tại (−5,0; 0,0) m và kết thúc tại (0,0; 3,0) m, với hướng đích π/2. Tuyến tập trung chuyển hướng từ hành lang ngang sang hành lang dọc. Thời gian nhiệm vụ phụ thuộc cả đoạn rẽ lẫn quá trình giảm tốc gần đích. Đây là trường hợp bổ sung cho tuyến chéo dài R02.

Hình 13 minh họa đường tham chiếu theo quy tắc chọn vùng tại R01; Bảng XIII và Hình 14 đối chiếu cả năm bộ lập kế hoạch. Toàn bộ 25 lượt khảo sát R05 đều đạt, nhưng kết quả này không thay thế kiểm chứng lặp tại Mục III-H.

BẢNG XIII. KẾT QUẢ MÔ TẢ R05 TRÊN NĂM BỘ LẬP KẾ HOẠCH

| Phương án | Đạt | T (s) | e (mm) | Eκ,ref (m⁻¹) | c (cm) |
| Đường gốc | 5/5 | 40,27 | 21,2 | 141,50 | 18,7 |
| Simple | 5/5 | 38,25 | 23,0 | 12,25 | 21,2 |
| SG | 5/5 | 40,02 | 22,1 | 32,12 | 20,9 |
| Constrained | 5/5 | 39,33 | 24,4 | 25,35 | 22,2 |
| PSTMO | 5/5 | 37,69 | 20,8 | 2,44 | 20,8 |


Cả 25 lượt của ma trận khảo sát R05 đều đạt. Eκ,ref và c là chỉ số đường tham chiếu; T và e là chỉ số thực thi. Kết quả này không được thay thế cho tỷ lệ thành công của bộ thử nghiệm lặp.

2) So sánh thời gian và chất lượng đường

PSTMO đạt thời gian trung bình 37,69 s. Các giá trị tương ứng của đường gốc, Savitzky–Golay và Constrained là 40,27 s, 40,02 s và 39,33 s. Simple đạt 38,25 s, gần PSTMO nhất về thời gian. RMSE của PSTMO là 20,8 mm; Constrained và đường gốc lần lượt có giá trị 24,4 mm và 21,2 mm. Mức chênh lệch về thời gian và RMSE phụ thuộc phương án đối chứng.

Tích phân bình phương độ cong của PSTMO là 2,44 m⁻¹, so với Constrained 25,35 m⁻¹ và đường gốc 141,50 m⁻¹. Tỷ lệ giảm Eκ,ref lớn hơn tỷ lệ giảm thời gian thực thi. Robot thực thi qua bộ điều khiển và các tầng giới hạn vận tốc, nên tổng thời gian không chỉ phụ thuộc chỉ số độ cong. Hình 14 thể hiện sự khác biệt giữa các bộ lập kế hoạch.

BẢNG XIV. CHIỀU DÀI, ĐỘ CONG CỰC ĐẠI VÀ CHỈ SỐ BỔ SUNG TẠI R05

| Phương án | L (m) | κmax (m⁻¹) | D (m) | emax (mm) | τ (ms) |
| Đường gốc | 6,983 | 23,51 | 6,863 | 54,9 | 0,0 |
| Simple | 6,920 | 9,58 | 6,820 | 58,9 | 1,8 |
| SG | 6,960 | 16,15 | 6,861 | 59,5 | 0,0 |
| Constrained | 6,975 | 11,56 | 6,853 | 67,1 | 22,2 |
| PSTMO | 6,871 | 2,13 | 6,796 | 50,1 | 77,4 |


Ký hiệu như Bảng VI. L, κmax và τ dùng năm lượt; D và emax chỉ dùng lượt đạt.

Trong Bảng XIV, PSTMO có đường tham chiếu, quãng đường thực thi và sai số cực đại thấp nhất trên cùng năm đầu vào. Tuy vậy, thời lượng xử lý 77,4 ms cao hơn Constrained, 22,2 ms, và Simple, 1,8 ms. Lợi ích khi di chuyển đi kèm chi phí hậu xử lý cao hơn.

[Hình được bảo toàn trong Word]

Hình 13. R05 với Theta*: rẽ từ hành lang ngang vào hành lang dọc. Hình sử dụng ma trận khảo sát một lượt; các thất bại của bộ thử nghiệm lặp được báo riêng ở Mục III-I.

Theo thứ tự đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO, Eκ,exec lần lượt là 2,864; 2,117; 2,587; 2,545; 1,900 m⁻¹; phân vị 95% của trị tuyệt đối độ giật lần lượt là 2,364; 2,425; 2,432; 2,384; 2,331 m/s³. PSTMO có Eκ,exec và độ giật thấp nhất trên tuyến này. Khác R01 và R03, cải thiện hình học thực thi đi kèm giảm chỉ số biến thiên gia tốc, nhưng vẫn cần đánh giá riêng việc dừng tại đích.

3) Khoảng hở và độ chính xác cuối

So sánh theo bộ lập kế hoạch cho thấy PSTMO nhanh hơn Constrained ở bốn cặp, nhưng với Theta* tăng 0,01 s và tăng RMSE 0,33 mm. So với Simple, PSTMO giảm thời gian ở ba cặp và giảm RMSE ở cả năm cặp. So với đường gốc và Savitzky–Golay, số cặp nhanh hơn lần lượt là bốn và năm. Các chênh lệch rất nhỏ là kết quả mô tả của một lượt, chưa xác định tính ổn định khi lặp.

[Hình được bảo toàn trong Word]

Hình 14. R05: kết quả một lượt cho từng tổ hợp bộ lập kế hoạch–phương án. Tất cả các lượt trong hình đều đạt, nhưng điều đó chưa mô tả tính lặp lại của nhiệm vụ.

Khoảng hở nhỏ nhất trung bình của PSTMO là 20,8 cm, so với 22,2 cm của Constrained; chênh lệch nhỏ hơn tại R01. Thời lượng hậu xử lý tương ứng là khoảng 0,077 s và 0,022 s. Trên R05, PSTMO giảm thời gian thực thi và RMSE trung bình so với Constrained nhưng có thời lượng hậu xử lý dài hơn. Các chỉ số này được diễn giải theo phạm vi đo tại Mục III-A.

Sai số vị trí cuối trung bình của năm phương án theo thứ tự trên là 40,3; 45,6; 38,3; 44,8; 46,5 mm; sai số hướng tương ứng là 0,081; 0,079; 0,077; 0,063; 0,063 rad, chỉ tính các lượt đạt. PSTMO có sai số vị trí cao nhất; sai số hướng gần Constrained ở độ chính xác báo cáo. Lợi ích thời gian và bám đường tại R05 vì vậy không đồng nghĩa sai số vị trí cuối nhỏ hơn; độ tin cậy cần được đối chiếu thêm với bộ lặp.

G. Tổng hợp so sánh năm phương án

1) Kết quả chung và điều kiện ghép cặp

Ma trận khảo sát có 123/125 lượt hoàn thành. Bảng XV cho thấy PSTMO có thời gian, RMSE và tích phân bình phương độ cong tham chiếu trung bình thấp nhất. Tuy nhiên, thời gian và RMSE dùng tập lượt đạt riêng của mỗi phương án, nên hiệu giữa hai trung bình chưa phải chênh lệch trên cùng một tập đầu vào.

BẢNG XV. TỔNG HỢP MÔ TẢ MA TRẬN 125 LƯỢT

| Phương án | Đạt | T (s) | e (mm) | Eκ,ref (m⁻¹) |
| Đường gốc | 24/25 | 47,65 | 25,1 | 134,91 |
| Simple | 25/25 | 45,51 | 24,5 | 8,74 |
| SG | 25/25 | 47,08 | 24,7 | 24,63 |
| Constrained | 25/25 | 47,32 | 26,1 | 26,34 |
| PSTMO | 24/25 | 44,60 | 23,6 | 3,13 |


T và e lấy trên 24 lượt đạt của đường gốc/PSTMO và 25 lượt đạt của mỗi phương án còn lại. Eκ,ref dùng toàn bộ 25 đường mỗi phương án. Năm bộ lập kế hoạch là năm đầu vào khác nhau, không phải năm lần lặp.

[Hình được bảo toàn trong Word]

Hình 15. So sánh PSTMO với bốn đối chứng. Chấm là chênh lệch trung bình; đoạn thẳng là khoảng từ nhỏ nhất đến lớn nhất, không phải khoảng tin cậy. Hai hàng đầu dùng các cặp cùng đạt; hàng cuối dùng đủ 25 cặp đường tham chiếu. Số phía trên mỗi cột là số cặp có chỉ số giảm trên tổng số cặp tương ứng.

Bảng XVI ghép PSTMO với từng đối chứng cùng tuyến và bộ lập kế hoạch, chỉ giữ cặp cùng hoàn thành. Có 23 cặp với đường gốc và 24 cặp với mỗi phương án làm mượt. PSTMO giảm thời gian, RMSE và Eκ,exec trung bình trên các cặp này; mức giảm thời gian nhỏ nhất khi so với Simple.

BẢNG XVI. CHÊNH LỆCH PSTMO SO VỚI BỐN ĐỐI CHỨNG TRÊN CÁC CẶP CÙNG HOÀN THÀNH

| Đối chứng | n | ΔT (s) | Δe (mm) | ΔEκ,exec (m⁻¹) |
| Đường gốc | 23 | -3,328 | -0,829 | -1,117 |
| Simple | 24 | -1,207 | -0,861 | -0,453 |
| SG | 24 | -2,790 | -0,856 | -0,955 |
| Constrained | 24 | -3,009 | -2,314 | -1,055 |


n là số cặp; Δ = PSTMO − đối chứng. Mỗi cặp dùng cùng đường gốc. Các chênh lệch là thống kê mô tả của một lượt cho mỗi cấu hình, không kèm kiểm định ý nghĩa thống kê.

PSTMO nhanh hơn đường gốc ở 21/23 cặp, Simple ở 14/24, Savitzky–Golay ở 23/24 và Constrained ở 21/24. Số cặp có RMSE thấp hơn tương ứng là 12/23, 14/24, 15/24 và 17/24. Như vậy, lợi ích thời gian xuất hiện thường xuyên hơn lợi ích bám đường, đặc biệt khi so với Savitzky–Golay. Không có cơ sở để kết luận PSTMO giảm đồng thời mọi chỉ số trên mọi đầu vào.

Hình 15 cho thấy chênh lệch thời gian và RMSE có thể đổi dấu giữa các đầu vào dù trung bình âm. Hàng cuối của hình dùng Eκ,ref trên đủ 25 cặp đường, khác với Eκ,exec trên các cặp cùng đạt trong Bảng XVI. Mục III-B–F phân tích theo tuyến; phần tiếp theo kiểm tra ảnh hưởng mật độ điểm tới chỉ số độ cong tham chiếu.

2) Chiều dài và độ cong của đường tham chiếu

Các bảng theo tuyến cho thấy PSTMO đồng thời thay đổi chiều dài và độ cong đường tham chiếu. Vì vậy, thời gian thực thi thấp hơn không thể được quy hoàn toàn cho một yếu tố hình học. Bảng XVII kiểm tra thêm độ cong sau khi thống nhất bước lấy mẫu của phần tịnh tiến.

BẢNG XVII. HÌNH HỌC PHẦN TỊNH TIẾN SAU LẤY MẪU LẠI VÀ DẤU QUAY TẠI CHỖ

| Phương án | κt,max (m⁻¹) | Eκ,t (m⁻¹) | N | Quay |
| Đường gốc | 11,06 | 35,84 | 223,6 | 0 |
| Simple | 3,29 | 6,01 | 223,6 | 0 |
| SG | 5,97 | 15,44 | 223,6 | 0 |
| Constrained | 6,74 | 19,76 | 223,6 | 0 |
| PSTMO | 2,17 | 2,96 | 473,6 | 2 |


κt,max và Eκ,t là trung bình trên 25 đường sau khi tách phần tịnh tiến và lấy mẫu lại với bước 0,05 m. N là số điểm đầu ra trung bình trước lấy mẫu lại; “Quay” là tổng số dấu quay tại chỗ trong 25 đường. Đây không phải số lần xoay hướng do bộ điều khiển thực hiện.

Trong Bảng XVII, Eκ,t của PSTMO sau lấy mẫu lại vẫn thấp nhất: 2,96 m⁻¹, so với Simple 6,01 m⁻¹ và Constrained 19,76 m⁻¹. Thứ hạng trung bình được duy trì dù đã thống nhất bước lấy mẫu; kết quả rời rạc này không chứng minh tính liên tục độ cong của đường gấp khúc.

Hai dấu quay tại chỗ được ghi trong 25 đường PSTMO, với tổng góc khoảng 1,608 rad. Các đối chứng không chứa dấu theo tiêu chí đã định; tuy nhiên, RPP vẫn có thể yêu cầu xoay hướng. Số dấu không thay thế số lần quay quan sát được khi thực thi.

Sai lệch vị trí đầu–cuối do hậu xử lý không vượt 2,8 × 10⁻¹⁷ m, ở mức sai số số học. Kết quả chỉ kiểm tra vị trí của đường, không thay cho kiểm tra hướng, định vị hoặc sai số robot dừng tại đích.

3) Quỹ đạo thực thi và độ chính xác cuối

BẢNG XVIII. CHỈ SỐ CHUYỂN ĐỘNG VÀ SAI SỐ CUỐI CỦA NĂM PHƯƠNG ÁN

| Phương án | Eκ,exec (m⁻¹) | j95 (m/s³) | ep (mm) | eψ (rad) |
| Đường gốc | 3,311 | 2,430 | 55,4 | 0,080 |
| Simple | 2,613 | 2,361 | 58,0 | 0,076 |
| SG | 3,116 | 2,377 | 55,4 | 0,071 |
| Constrained | 3,215 | 2,375 | 57,9 | 0,067 |
| PSTMO | 2,149 | 2,341 | 57,3 | 0,069 |


Trung bình trên các lượt đạt: 24 lượt cho đường gốc/PSTMO, 25 lượt cho mỗi phương án còn lại. j95 là phân vị 95% của trị tuyệt đối độ giật tịnh tiến trong từng lượt; bảng lấy trung bình các phân vị này. ep và eψ là sai số vị trí và hướng cuối. Eκ,exec được tính trên quỹ đạo đã xử lý như Mục III-A.

PSTMO có Eκ,exec trung bình thấp nhất trong Bảng XVIII. Trên các cặp cùng hoàn thành, chỉ số này giảm ở 23/23 cặp với đường gốc, 21/24 với Simple và toàn bộ 24 cặp với mỗi đối chứng Savitzky–Golay và Constrained. Mức cải thiện hình học thực thi nhất quán hơn mức giảm RMSE trong ma trận khảo sát.

Quãng đường thực thi trung bình là 8,510; 8,507; 8,545; 8,555 và 8,539 m theo cùng thứ tự năm phương án. Giá trị gộp của PSTMO lớn hơn Simple, nhưng chênh lệch ghép cặp là −0,040 m trên 24 cặp cùng đạt. Sự khác nhau xuất phát từ việc Simple có thêm một lượt hoàn thành mà PSTMO không đạt. Đây là lý do cần báo mẫu số và dùng đầu vào tương ứng khi đánh giá chênh lệch.

Trong Bảng XVIII, PSTMO có độ giật phân vị 95% trung bình thấp nhất, nhưng sai số vị trí cuối 57,3 mm vẫn lớn hơn đường gốc và Savitzky–Golay, cùng đạt 55,4 mm. Sai số hướng 0,069 rad thấp hơn ba đối chứng nhưng cao hơn Constrained, 0,067 rad. Các trung bình dùng tập lượt đạt riêng; thứ hạng bám đường không chuyển nguyên sang độ chính xác dừng.

Hình 16 minh họa tín hiệu của R02 với Theta*, bộ lập kế hoạch cũng được dùng trong các hình đường tham chiếu theo tuyến. Trường hợp này giúp đối chiếu đường tham chiếu, chuyển động thực thi và vận tốc góc trong cùng lượt; không được dùng để ước lượng hiệu năng chung của năm tuyến.

[Hình được bảo toàn trong Word]

Hình 16. Minh họa R02/Theta* cho đủ năm phương án. Trên: đường tham chiếu nét đứt và quỹ đạo thực thi nét liền tại vùng rẽ; giữa: vận tốc góc của đường gốc, Simple và SG; dưới: Constrained và PSTMO. Hai đồ thị vận tốc dùng cùng thang đo, tín hiệu không lọc và thời gian riêng tính từ đầu từng lượt. Đây là một trường hợp minh họa, không phải kết quả trung bình toàn ma trận.

Các tín hiệu trong Hình 16 đều có đoạn vận tốc góc dương gần 0,25 rad/s, nhưng diễn biến theo thời gian khác nhau. Ở lượt R02/Theta* này, PSTMO hoàn thành trong 55,764 s, nhanh hơn Constrained 0,939 s nhưng chậm hơn Simple 0,636 s. Vùng quỹ đạo phóng to chỉ biểu diễn một phần hành trình, còn đồ thị vận tốc bao quát cả lượt. Biên độ vận tốc góc gần nhau chưa giải thích chênh lệch thời gian toàn nhiệm vụ; các tín hiệu không tách được tác động riêng của hình học đường và bộ điều khiển.

4) Biên độ chuyển động, thời lượng xử lý và an toàn

BẢNG XIX. CỰC ĐẠI QUAN SÁT ĐƯỢC CỦA CÁC ĐẠI LƯỢNG CHUYỂN ĐỘNG

| Phương án | |ω|max (rad/s) | |vb|max (m/s) | |a⊥|max (m/s²) | |a|max (m/s²) | |α|max (rad/s²) |
| Đường gốc | 0,249 | 0,315 | 0,062 | 0,281 | 0,697 |
| Simple | 0,249 | 0,315 | 0,062 | 0,287 | 0,789 |
| SG | 0,249 | 0,315 | 0,062 | 0,258 | 0,768 |
| Constrained | 0,249 | 0,315 | 0,062 | 0,285 | 0,656 |
| PSTMO | 0,249 | 0,315 | 0,062 | 0,302 | 0,732 |


Mỗi ô là giá trị lớn nhất trong toàn bộ 25 lượt, bao gồm không đạt. vb là vận tốc bánh tương đương suy từ vận tốc thân xe với khoảng cách bánh 0,2548 m; a⊥ = vω; a và α lần lượt là gia tốc tịnh tiến và gia tốc góc. Đây là đại lượng ước lượng tại các mẫu đã ghi, không phải phép đo trực tiếp vận tốc từng bánh.

Bảng XIX ghi nhận vận tốc góc cực đại khoảng 0,249 rad/s cho cả năm phương án, thấp hơn giới hạn 0,80 rad/s trong mô hình PSTMO. Vận tốc bánh tương đương không vượt khoảng 0,316 m/s và gia tốc ngang không vượt 0,063 m/s²; các giới hạn mô hình tương ứng là 0,36 m/s và 0,18 m/s². Vận tốc tịnh tiến cực đại xấp xỉ 0,30 m/s; phần vượt nhỏ hơn 10⁻⁷ m/s ở dữ liệu lưu được xem là sai số số học của mô phỏng.

Gia tốc tịnh tiến cực đại của PSTMO là 0,302 m/s², cao hơn bốn đối chứng trong ma trận dù vẫn thấp hơn giới hạn tăng tốc 0,35 m/s². Cực đại gia tốc góc là 0,732 rad/s², thấp hơn giới hạn 1,20 rad/s² nhưng cao hơn Constrained. Các kết quả này mô tả đáp ứng của toàn hệ thống dưới cùng bộ điều khiển; chúng chưa tách được đóng góp riêng của các ràng buộc nội bộ PSTMO hoặc bảo đảm rằng mọi trạng thái giữa hai mẫu đều thỏa giới hạn.

BẢNG XX. THỜI LƯỢNG HẬU XỬ LÝ VÀ CÁC CHỈ SỐ GIÁM SÁT TRONG MA TRẬN KHẢO SÁT

| Phương án | τ (ms) | Dừng | Can thiệp | Mẫu va chạm |
| Đường gốc | — | 25/25 | 0 | 0 |
| Simple | 1,32 | 25/25 | 0 | 0 |
| SG | 0,00 | 25/25 | 0 | 0 |
| Constrained | 26,88 | 25/25 | 0 | 0 |
| PSTMO | 98,04 | 25/25 | 0 | 0 |


τ là trung bình trên 25 lượt; đường gốc không gọi hậu xử lý. “Dừng” là số lượt đáp ứng điều kiện dừng vật lý; “Can thiệp” là tổng số lần bộ giám sát va chạm can thiệp; “Mẫu va chạm” là tổng mẫu đường bao giao cắt vật cản trên đường tham chiếu. Các số đếm bao gồm cả lượt không đạt.

Bảng XX ghi nhận thời lượng hậu xử lý trung bình 98,04 ms với PSTMO, 26,88 ms với Constrained và 1,32 ms với Simple. PSTMO có chi phí thời gian xử lý cao nhất trong phép đo này, nên cần cân nhắc khi yêu cầu cập nhật đường thường xuyên. Giá trị 0,00 ms của Savitzky–Golay theo đồng hồ mô phỏng không chứng minh chi phí tính toán bằng không; các số liệu chưa phải thời gian CPU.

Toàn bộ 125 lượt đáp ứng điều kiện dừng vật lý; hai lượt vẫn không đạt do sai số vị trí. Không ghi nhận can thiệp của bộ giám sát va chạm hoặc mẫu đường bao giao cắt vật cản trên đường tham chiếu. Khoảng hở ở các bảng theo tuyến cho thấy PSTMO thường đi gần vật cản hơn Constrained. Phép kiểm tra trên bản đồ tĩnh không thay thế phép đo tiếp xúc vật lý và không xác định được khoảng hở nhỏ nhất của toàn bộ chuyển động thực thi.

H. Kiểm chứng bổ sung về tính lặp lại

1) Phương pháp phân tích thống kê

Trong toàn bộ 125 cặp, hai phương án nhận cùng đường gốc trong từng cặp. Constrained đạt 122/125 và PSTMO đạt 121/125; có 118 cặp cùng đạt để phân tích chỉ số liên tục. Mỗi tổ hợp tuyến–bộ lập kế hoạch tạo một nhóm b. Gọi Rb là tập lần lặp cùng đạt trong nhóm; db là trung bình hiệu PSTMO trừ Constrained. Công thức 9 tính chênh lệch chung Δ bằng trung bình đều của 25 nhóm, tránh coi các lần lặp cùng đầu vào là các đầu vào độc lập. Các đại lượng yP,br và yC,br là giá trị chỉ số của hai phương án tại lần lặp r trong nhóm b.

[Office Math được bảo toàn trong Word] 

[Office Math được bảo toàn trong Word] (9)

Khoảng tin cậy 95% của Δ dùng bootstrap phân vị trên 25 nhóm, lấy mẫu lại 20.000 lần [76]. Kiểm định Wilcoxon hạng có dấu hai phía dùng db, loại các hiệu có trị tuyệt đối không quá 10⁻¹² [77]; xét phân bố chênh lệch đối xứng quanh không, không trực tiếp kiểm định trung bình Δ. Giá trị p của thời gian, RMSE và Eκ,exec được hiệu chỉnh Holm [78], nhưng khoảng tin cậy chỉ ở mức từng chỉ số. Các nhóm chia sẻ tuyến, bộ lập kế hoạch và bản đồ; gom nhóm chưa loại hết phụ thuộc. Suy luận giới hạn trong thiết kế đầu vào cố định.

2) Kết quả chính và các đánh đổi

BẢNG XXI. CHÊNH LỆCH PSTMO SO VỚI CONSTRAINED TRONG BỘ KIỂM CHỨNG LẶP

| Chỉ số | Δ trung bình | Khoảng tin cậy 95% | p Holm |
| T (s) | -2,857 | [-3,716; -2,039] | 1,19 × 10⁻⁶ |
| e (mm) | -2,985 | [-4,487; -1,506] | 4,89 × 10⁻⁴ |
| Eκ,exec (m⁻¹) | -1,038 | [-1,332; -0,767] | 5,36 × 10⁻⁷ |


Δ = PSTMO − Constrained, trung bình đều trên 25 nhóm. Khoảng tin cậy dùng bootstrap nhóm; p Holm hiệu chỉnh ba chỉ số chính. Phân tích chỉ dùng những cặp cả hai phương án cùng đạt và không gộp dữ liệu với ma trận 125 lượt.

PSTMO giảm thời gian trung bình theo nhóm 2,857 s, RMSE 2,985 mm và Eκ,exec 1,038 m⁻¹. Khoảng tin cậy 95% của cả ba chênh lệch đều nằm dưới không; các giá trị p sau hiệu chỉnh Holm đều nhỏ hơn 0,05. Kết quả hỗ trợ mức cải thiện trung bình trong thiết kế khảo sát, không bảo đảm mọi lần thực thi đều tốt hơn. Số nhóm có chỉ số giảm lần lượt là 24/25, 19/25 và 24/25, cho thấy lợi ích RMSE ít nhất quán hơn hai chỉ số còn lại.

Hình 17 cho thấy RMSE tăng ở một số đầu vào dù chênh lệch chung âm. Trung bình đều theo công thức 9 tránh để nhóm nhiều cặp cùng đạt hơn chi phối kết quả, nhưng không hiệu chỉnh thiên lệch do loại cặp không cùng đạt. Ý nghĩa vận hành còn phụ thuộc yêu cầu thời gian, sai số và khoảng hở; p nhỏ không thay thế các tiêu chí đó. Bảng XXII trình bày các đánh đổi.

BẢNG XXII. CÁC CHỈ SỐ BỔ SUNG TRONG BỘ KIỂM CHỨNG LẶP

| Chỉ số phụ | Δ trung bình | Khoảng tin cậy 95% |
| Eκ,ref (m⁻¹) | -23,202 | [-34,866; -13,197] |
| Phân vị 95% của |độ giật| (m/s³) | -0,078 | [-0,131; -0,032] |
| Quãng đường (m) | -0,089 | [-0,111; -0,069] |
| Sai số đích (mm) | +3,380 | [+1,077; +5,649] |
| Khoảng hở (mm) | -35,161 | [-58,420; -10,997] |
| Thời lượng hậu xử lý (ms) | +71,220 | [+58,392; +85,230] |


Các khoảng tin cậy là phân tích bổ sung; hiệu chỉnh Holm chỉ áp dụng cho ba chỉ số trong Bảng XXI. Dấu âm thuận lợi cho quãng đường, độ giật, tích phân bình phương độ cong và thời lượng xử lý; đối với khoảng hở, dấu âm biểu thị robot được dẫn gần vật cản hơn.

[Hình được bảo toàn trong Word]

Hình 17. Chênh lệch trung bình của 25 nhóm trong bộ kiểm chứng lặp. Trong mỗi tuyến, thứ tự là A*, Dijkstra, Theta*, Smac 2D và Smac Hybrid. Mỗi điểm dùng các lần lặp cùng đạt; đường đứt là trung bình đều của 25 điểm. Hình thể hiện mức thay đổi giữa các đầu vào, không biểu diễn 125 quan sát độc lập.

Bảng XXII ghi nhận quãng đường giảm 0,089 m và Eκ,ref giảm 23,202 m⁻¹, đồng thời khoảng hở giảm 35,161 mm, sai số vị trí cuối tăng 3,380 mm và thời lượng hậu xử lý tăng 71,220 ms. Lợi ích hình học và thời gian vì vậy đi kèm bất lợi về khoảng cách vật cản và độ chính xác dừng. Các khoảng tin cậy bổ sung không chứa không nhưng chưa được hiệu chỉnh đa kiểm định như ba chỉ số chính. Mức giảm độ giật 0,078 m/s³ chỉ mô tả biến thiên gia tốc, không xác định độ rung tải hàng hoặc điện năng tiêu thụ.

Trong 125 cặp, bốn cặp chỉ Constrained đạt và ba cặp chỉ PSTMO đạt. Kiểm định McNemar chính xác hai phía cho p = 1,0. Dữ liệu chưa cho thấy khác biệt tỷ lệ thành công; kết quả cũng không chứng minh tương đương hoặc không kém hơn. Năm lần lặp ở mỗi cấu hình còn ít để đánh giá các thất bại hiếm. Các lượt không đạt được giữ đầy đủ trong Mục III-I.

I. Phân tích các trường hợp không đạt và giới hạn nghiên cứu

1) Các lượt vượt dung sai vị trí

BẢNG XXIII. CÁC LƯỢT KHÔNG ĐẠT TRONG HAI BỘ DỮ LIỆU

| Tuyến/bộ lập kế hoạch | Phương án/lần | Trạng thái | Sai số đích (m) |
| R03 / Hybrid | P / KS | Vượt ngưỡng vị trí | 0,102385 |
| R04 / Theta* | G / KS | Vượt ngưỡng vị trí | 0,100234 |
| R05 / A* | P / 4 | Kết thúc trước đích | 1,580262 |
| R05 / Dijkstra | P / 4 | Kết thúc trước đích | 0,685907 |
| R05 / Smac 2D | C / 3 | Kết thúc trước đích | 3,465366 |
| R05 / Smac 2D | P / 1 | Kết thúc trước đích | 1,148750 |
| R05 / Hybrid | C / 4 | Kết thúc trước đích | 3,655099 |
| R04 / Hybrid | P / 4 | Vượt ngưỡng vị trí | 0,101583 |
| R04 / Theta* | C / 4 | Vượt ngưỡng vị trí | 0,100358 |


KS là ma trận khảo sát 125 lượt; số 1–5 là lần lặp trong bộ 250 lượt. G: đường gốc; P: PSTMO; C: Constrained. Hai bộ được báo riêng, không cộng mẫu số để tính một tỷ lệ thành công chung. “Kết thúc trước đích” mô tả trạng thái quan sát được, chưa xác định nguyên nhân và không đồng nghĩa đã dừng vật lý.

Trong ma trận khảo sát, PSTMO/Smac Hybrid tại R03 có sai số vị trí cuối 0,102385 m; đường gốc/Theta* tại R04 có sai số 0,100234 m. Cả hai tác vụ đều hoàn tất và robot đã dừng, nhưng vượt ngưỡng đánh giá 0,10 m. Giữ nguyên phân loại này tránh việc điều chỉnh dung sai sau khi biết kết quả. Những lượt này được tính vào tỷ lệ thất bại và không được dùng như thời gian hoàn thành hợp lệ khi so sánh ghép cặp.

Bộ lặp ghi thêm hai trường hợp R04 vượt ngưỡng: PSTMO/Smac Hybrid ở lần 4 đạt sai số 0,101583 m; Constrained/Theta* ở lần 4 đạt 0,100358 m. Phần vượt lần lượt khoảng 1,6 mm và 0,4 mm, nhỏ hơn nhiều so với sai số của các lượt kết thúc xa đích. Các con số phản ánh trạng thái mô phỏng, không biểu thị độ chính xác đo trên robot thực ở mức dưới milimét.

2) Các lượt kết thúc trước đích

Năm thất bại còn lại tập trung tại R05 trong bộ lặp, gồm ba lượt PSTMO và hai lượt Constrained. Sai số vị trí còn lại từ 0,686 đến 3,655 m. Các lượt này kết thúc trước khi hoàn thành hành trình, khác về bản chất với những trường hợp chỉ vượt nhẹ dung sai. Bảng XXIII giữ rõ tuyến, bộ lập kế hoạch, phương án và lần chạy để đối chiếu dữ liệu.

Hình 18 cho thấy phần lớn sai số nằm dưới ngưỡng, nhưng một số lượt R05 dừng phép thử khi còn xa đích. Thời gian trung bình trên các cặp cùng đạt chưa bao gồm chi phí phục hồi những thất bại này. Vì chưa có một chính sách phục hồi chung được đo, không thể quy đổi kết quả thành thời gian chu kỳ vận hành thực tế có tính đến thử lại.

Dữ liệu hiện tại xác định được trạng thái kết thúc, chưa chứng minh nguyên nhân cụ thể của từng thất bại. Việc quy lỗi cho hình dạng đường, bộ điều khiển hoặc định vị cần đối chiếu đồng bộ nhật ký và tín hiệu ở thời điểm sự kiện. Các thất bại xa đích xuất hiện ở cả PSTMO và Constrained; số lượng hiện có chưa đủ để gán ưu thế về độ tin cậy cho một phương án.

[Hình được bảo toàn trong Word]

Hình 18. Sai số vị trí cuối của toàn bộ 250 lượt kiểm chứng lặp. Trục đứng dùng thang log để hiển thị đồng thời sai số cỡ centimét và các lượt kết thúc xa đích. Ngưỡng 0,10 m chỉ là một phần của tiêu chí hoàn thành; các lượt thuộc ma trận 125 lượt không nằm trong hình này.

3) Phạm vi kết luận

Khảo sát dùng một bản đồ tĩnh, một robot mô phỏng và một cấu hình RPP; chưa xét vật cản động, nhiều mức khởi tạo ngẫu nhiên hay nhiễu. R01 khác đợt thu R02–R05; bộ lặp chỉ gồm PSTMO và Constrained. Chưa xác nhận ngẫu nhiên hóa thứ tự chạy; bốn tiến trình dùng chung tài nguyên máy dù tách miền truyền thông. Điều này hạn chế suy rộng và diễn giải thời lượng xử lý.

Kết quả chỉ đại diện cho các cấu hình ở Mục III-A, chưa có tinh chỉnh với cùng ngân sách đánh giá tham số trên tập hiệu chỉnh tách biệt. Phân tích cặp cùng đạt có thể thiên lệch về các nhiệm vụ dễ hoàn thành; vì vậy phải đọc cùng tỷ lệ thành công và các trường hợp không đạt.

Mở rộng sang môi trường động cần giải quyết thêm thông tin dự đoán vật cản và cập nhật quỹ đạo. Ahn và cộng sự [79] tích hợp dự đoán với trường thế nhân tạo và Bézier cho xe tự động; Xu và cộng sự [80] dùng thị giác và tối ưu B-spline để tránh vật cản động cho UAV. Hai ví dụ thuộc mô hình phương tiện khác, nên chỉ chỉ ra những yêu cầu nghiên cứu bổ sung, không chứng minh PSTMO sẽ giữ hiệu quả khi chuyển sang môi trường động. Khảo sát hiện tại chưa đánh giá các thành phần cảm nhận và dự đoán đó.

Lợi ích hình học và thực thi của PSTMO đi kèm đánh đổi về khoảng hở, sai số cuối và thời lượng xử lý. Chưa kiểm chứng trên robot thực hoặc loại bỏ từng thành phần trên cùng phiên bản để tách đóng góp của tiền xử lý, tìm hình dạng và quy hoạch động. Kiểm tra hình bao phụ thuộc bản đồ và lấy mẫu. Dữ liệu chưa hỗ trợ kết luận về điện năng, rung tải hàng hay ưu thế tỷ lệ thành công.

IV KẾT LUẬN

Bài báo đã trình bày PSTMO để hậu xử lý đường đi cho robot vi sai trong môi trường kho, với trọng tâm là lựa chọn có phối hợp giữa chuyển tiếp Bézier và quay tại chỗ. Việc kiểm tra hình bao, xét giới hạn chuyển động và so sánh thời gian trên cửa sổ chung tạo cơ sở sàng lọc ứng viên; quy hoạch động chọn chuỗi có tổng chi phí nhỏ nhất trong tập hữu hạn dưới ràng buộc cạnh chung. Cách tổ chức này đưa quan hệ giữa các góc liên tiếp vào quyết định hậu xử lý, thay vì chỉ tối ưu từng góc độc lập.

Trong bộ kiểm chứng lặp 250 lượt, phân tích 118 cặp cùng đạt bằng trung bình đều của 25 nhóm tuyến–bộ lập kế hoạch ghi nhận PSTMO giảm thời gian thực thi 2,857 s, RMSE bám đường 2,985 mm và tích phân bình phương độ cong quỹ đạo thực thi 1,038 m−1 so với Constrained. Khoảng tin cậy bootstrap nhóm 95% của cả ba chênh lệch nằm dưới không; các kiểm định Wilcoxon tương ứng có p sau hiệu chỉnh Holm nhỏ hơn 0,05. Đây là bằng chứng về lợi ích trung bình có điều kiện trong thiết kế khảo sát, không phải sự cải thiện ở mọi đầu vào. Ma trận 125 lượt bổ sung so sánh với các phương án còn lại nhưng không thay thế kiểm chứng lặp cho từng đối chứng.

Lợi ích không xuất hiện đồng thời trên mọi tiêu chí. Trong cùng phân tích theo nhóm, khoảng hở trên đường tham chiếu giảm 35,161 mm, sai số vị trí cuối tăng 3,380 mm và thời lượng hậu xử lý tăng 71,220 ms. Đại lượng thời lượng này được đo theo đồng hồ mô phỏng, không phải thời gian CPU. PSTMO đạt 121/125 lượt và Constrained đạt 122/125 lượt trong bộ lặp; dữ liệu chưa xác lập ưu thế tỷ lệ thành công, tương đương hoặc không kém hơn. Do đó, kết quả hỗ trợ việc cân nhắc PSTMO khi ưu tiên thời gian và chất lượng bám đường trong cấu hình đã khảo sát, đồng thời yêu cầu đánh giá khoảng hở và độ chính xác dừng cho ứng dụng cụ thể.

Phạm vi chứng cứ hiện giới hạn ở một bản đồ tĩnh, một robot mô phỏng và một cấu hình điều khiển. Phân tích cặp cùng đạt có thể chịu thiên lệch chọn mẫu; các nhóm còn chia sẻ tuyến, bộ lập kế hoạch và bản đồ. Chưa có thí nghiệm loại bỏ thành phần trên cùng phiên bản để quy riêng lợi ích cho quy hoạch động, cũng chưa kiểm chứng trên robot thực. Các bước tiếp theo cần ưu tiên đối chiếu lựa chọn chuỗi với lựa chọn cục bộ trong cùng cấu hình, khảo sát độ nhạy tham số trên tập hiệu chỉnh tách biệt, rồi mở rộng sang nhiều bản đồ và robot thực có hiệu chuẩn mô hình. Đánh giá tiếp theo cũng cần đo khoảng hở khi thực thi và chi phí phục hồi thất bại. Những kiểm chứng này sẽ xác định rõ hơn khi nào lợi ích của lựa chọn chuyển tiếp bù được chi phí hậu xử lý và mức giảm khoảng hở.

TÀI LIỆU THAM KHẢO

[1] T. Le-Anh and M. B. M. de Koster, “A review of design and control of automated guided vehicle systems,” European Journal of Operational Research 171(1):1–23, 2006.

[2] G. Fragapane, R. de Koster, F. Sgarbossa, and J. O. Strandhagen, “Planning and control of autonomous mobile robots for intralogistics: Literature review and research agenda,” European Journal of Operational Research, vol. 294, no. 2, pp. 405–426, 2021, doi: 10.1016/j.ejor.2021.01.019.

[3] R. de Koster, T. Le-Duc, K. J. Roodbergen, “Design and control of warehouse order picking: A literature review,” European Journal of Operational Research 182(2):481–501, 2007.

[4] N. Correll et al., “Analysis and Observations from the First Amazon Picking Challenge,” IEEE Trans. Automation Science and Engineering 15:172–188, 2018.

[5] S. Al Mahmud, A. Kamarulariffin, A. M. Ibrahim, and A. J. H. Mohideen, “Advancements and Challenges in Mobile Robot Navigation: A Comprehensive Review of Algorithms and Potential for Self-Learning Approaches,” Journal of Intelligent & Robotic Systems 110:120, 2024, doi: 10.1007/s10846-024-02149-5.

[6] L. Yang et al., “Path Planning Technique for Mobile Robots: A Review,” Machines 11(10):980, 2023, doi: 10.3390/machines11100980.

[7] A. Ravankar, A. A. Ravankar, Y. Kobayashi, Y. Hoshino, and C.-C. Peng, “Path smoothing techniques in robot navigation: State-of-the-art, current and future challenges,” Sensors, vol. 18, no. 9, art. 3170, 2018, doi: 10.3390/s18093170.

[8] E. W. Dijkstra, “A Note on Two Problems in Connexion with Graphs,” Numerische Mathematik 1:269–271, 1959, doi: 10.1007/BF01386390.

[9] P. Hart, N. Nilsson, and B. Raphael, “A Formal Basis for the Heuristic Determination of Minimum Cost Paths,” IEEE Transactions on Systems Science and Cybernetics 4:100–107, 1968, doi: 10.1109/TSSC.1968.300136.

[10] A. Stentz, “The Focussed D* Algorithm for Real-Time Replanning,” International Joint Conference on Artificial Intelligence, pp. 1652–1659, 1995.

[11] A. Nash, K. Daniel, S. Koenig, and A. Felner, “Theta*: Any-angle path planning on grids,” in Proc. AAAI, 2007. Available: https://idm-lab.org/bib/abstracts/papers/aaai07a.pdf.

[12] L. E. Kavraki, P. Svestka, J.-C. Latombe, M. H. Overmars, “Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces,” IEEE Transactions on Robotics and Automation 12:566–580, 1996.

[13] S. M. LaValle and J. J. Kuffner, “Rapidly-exploring random trees: Progress and prospects,” in Algorithmic and Computational Robotics: New Directions, B. R. Donald, K. M. Lynch, and D. Rus, Eds., A K Peters, 2001, pp. 293–308.

[14] S. Karaman and E. Frazzoli, “Sampling-based algorithms for optimal motion planning,” arXiv:1105.1186v1, 2011.

[15] D. Dolgov, S. Thrun, M. Montemerlo, and J. Diebel, “Practical Search Techniques in Path Planning for Autonomous Driving,” in Proc. AAAI Workshop on Search in Artificial Intelligence and Robotics, 2008.

[16] S. Macenski, M. Booker, J. Wallace, and T. Fischer, “Open-source, cost-aware kinematically feasible planning for mobile and surface robotics,” arXiv:2401.13078v2, 2025. Available: https://arxiv.org/abs/2401.13078v2.

[17] Z. Chen, H. Shao, B. Liu, S. Qiao, Y. Zhou, and Y. Li, “FDSPC: Fast and direct smooth motion planning via continuous curvature integration,” arXiv:2405.03281v2, 2025.

[18] K. Komoriya and K. Tanie, “Trajectory Control of a Wheel-type Mobile Robot Using a B-spline Curve,” Journal of the Robotics Society of Japan 8(2):133–143, 1990, doi: 10.7210/jrsj.8.133.

[19] A. Piazzi, C. Guarino Lo Bianco, M. Bertozzi, A. Fascioli, and A. Broggi, “Quintic G²-splines for the iterative steering of vision-based autonomous vehicles,” IEEE Transactions on Intelligent Transportation Systems, vol. 3, pp. 27–36, 2002, doi: 10.1109/6979.994793.

[20] Phan Gia Luan and Nguyen Truong Thinh, “C2 piecewise cubic Bezier curve based smoothing path for mobile robot,” International Journal of Mechanical Engineering and Robotics Research, vol. 10, no. 9, pp. 519–525, 2021, doi: 10.18178/ijmerr.10.9.519-525.

[21] M. Brezak and I. Petrović, “Path smoothing using clothoids for differential drive mobile robots,” IFAC Proceedings Volumes, vol. 44, no. 1, pp. 1133–1138, 2011, doi: 10.3182/20110828-6-IT-1002.02944.

[22] S. Fleury, P. Souères, J.-P. Laumond, and R. Chatila, “Primitives for Smoothing Mobile Robot Trajectories,” IEEE ICRA, 1993, doi: 10.1109/ROBOT.1993.292211.

[23] A. M. Lekkas, A. R. Dahl, M. Breivik, and T. I. Fossen, “Continuous-Curvature Path Generation Using Fermat’s Spiral,” Modeling, Identification and Control 34(4):183–198, 2013, doi: 10.4173/mic.2013.4.3.

[24] T. Tuttle and J. P. Wilhelm, “Minimal length multi-segment clothoid return paths for vehicles with turn rate constraints,” Frontiers in Aerospace Engineering 1:982808, 2022, doi: 10.3389/fpace.2022.982808.

[25] V. Parque and T. Miyashita, “Smooth curve fitting of mobile robot trajectories using differential evolution,” IEEE Access, vol. 8, pp. 82855–82866, 2020, doi: 10.1109/ACCESS.2020.2991003.

[26] L. Xu, D. Wang, B. Song, and M. Cao, “Global Smooth Path Planning for Mobile Robots based on Continuous Bezier Curve,” Chinese Automation Congress, 2017, doi: 10.1109/CAC.2017.8243114.

[27] B. Z. Türkkol, N. Altuntaş, and S. Çekirdek Yavuz, “A smooth global path planning method for unmanned surface vehicles using a novel combination of rapidly exploring random tree and Bézier curves,” Sensors, vol. 24, no. 24, art. 8145, 2024, doi: 10.3390/s24248145.

[28] J. Pan, L. Zhang, and D. Manocha, “Collision-free and curvature-continuous path smoothing in cluttered environments,” in Robotics: Science and Systems, 2011. Available: https://www.roboticsproceedings.org/rss07/p32.pdf.

[29] I. Andrei, M. Kloetzer, C. Mahulea, and C. Dosoftei, “Smooth path planning with safety margins using Piece-Wise Bezier curves,” in Proc. IEEE ETFA, 2025, pp. 1–4, doi: 10.1109/ETFA65518.2025.11205537.

[30] Q. Serdel, J. Marzat, and J. Moras, “Smooth Path Planning Using a Gaussian Process Regression Map for Mobile Robot Navigation,” arXiv:2407.05779, 2024.

[31] S. Quinlan and O. Khatib, “Elastic Bands: Connecting Path Planning and Control,” IEEE ICRA 1993, pp. 802–807, 1993.

[32] A. Richardson and E. Olson, “Iterative path optimization for practical robot planning,” in Proc. IEEE/RSJ IROS, 2011, pp. 3881–3886. Available: https://april.eecs.umich.edu/pdfs/richardson2011.pdf.

[33] H. Andreasson, J. Saarinen, M. Cirillo, T. Stoyanov, and A. J. Lilienthal, “Drive the Drive: From Discrete Motion Plans to Smooth Drivable Trajectories,” Robotics 3(4):400–416, 2014, doi: 10.3390/robotics3040400.

[34] H. Andreasson, J. Saarinen, M. Cirillo, T. Stoyanov, and A. J. Lilienthal, “Fast, continuous state path smoothing to improve navigation accuracy,” in Proc. IEEE ICRA, 2015, pp. 662–669, doi: 10.1109/ICRA.2015.7139250.

[35] E. Heiden, L. Palmieri, S. Koenig, K. O. Arras, and G. S. Sukhatme, “Gradient-informed path smoothing for wheeled mobile robots,” in Proc. IEEE ICRA, 2018. Available: https://eric-heiden.com/publication/2018-grips-icra/.

[36] N. Ratliff, M. Zucker, J. A. Bagnell, and S. Srinivasa, “CHOMP: Gradient Optimization Techniques for Efficient Motion Planning,” IEEE ICRA, 2009.

[37] M. Kalakrishnan, S. Chitta, E. Theodorou, P. Pastor, and S. Schaal, “STOMP: Stochastic Trajectory Optimization for Motion Planning,” IEEE ICRA 2011, 2011.

[38] C. Park, J. Pan, and D. Manocha, “ITOMP: Incremental Trajectory Optimization for Real-Time Replanning in Dynamic Environments,” ICAPS 2012, pp. 207–215, 2012.

[39] Z. Zhu, E. Schmerling, and M. Pavone, “A convex optimization approach to smooth trajectories for motion planning with car-like robots,” in Proc. IEEE CDC, 2015, pp. 835–842. Available: https://arxiv.org/abs/1506.01085.

[40] C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann, and T. Bertram, “Trajectory Modification Considering Dynamic Constraints of Autonomous Robots,” ROBOTIK 2012, pp. 1–6, 2012.

[41] C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann, and T. Bertram, “Efficient Trajectory Optimization Using a Sparse Model,” ECMR 2013, pp. 138–143, 2013.

[42] C. Rösmann, F. Hoffmann, and T. Bertram, “Timed-Elastic-Bands for Time-Optimal Point-to-Point Nonlinear Model Predictive Control,” European Control Conference, 2015, doi: 10.1109/ECC.2015.7331052.

[43] C. Rösmann, F. Hoffmann, and T. Bertram, “Kinodynamic Trajectory Optimization and Control for Car-Like Robots,” IEEE/RSJ IROS 2017, pp. 5681–5686, 2017.

[44] C. Rösmann, F. Hoffmann, and T. Bertram, “Integrated online trajectory planning and optimization in distinctive topologies,” Robotics and Autonomous Systems, vol. 88, pp. 142–153, 2017, doi: 10.1016/j.robot.2016.11.007.

[45] M. Zhang et al., “Universal trajectory optimization framework for differential drive robot class,” arXiv:2409.07924v3, 2025. Available: https://arxiv.org/abs/2409.07924v3.

[46] J.-W. Choi, R. E. Curry, and G. H. Elkaim, “Obstacle Avoiding Real-Time Trajectory Generation of Omnidirectional Vehicles,” in Proc. IEEE ICRA (author manuscript), 2009.

[47] M. Benko Loknar, G. Klančar, and S. Blažič, “Minimum-time trajectory generation for wheeled mobile systems using Bézier curves with constraints on velocity, acceleration and jerk,” Sensors, vol. 23, no. 4, art. 1982, 2023, doi: 10.3390/s23041982.

[48] S. Blažič, G. Klančar, M. Benko Loknar, and I. Škrjanc, “Warehouse path planning using low-order Bézier curves with minimum-time optimization,” IFAC-PapersOnLine, vol. 56, no. 2, pp. 11815–11820, 2023, doi: 10.1016/j.ifacol.2023.10.578.

[49] P. Dorpmüller, T. Schmitz, N. Bejagam, and T. Bertram, “Time-Optimal Trajectory Planning in Highway Scenarios using Basis-Spline Parameterization,” IEEE ITSC; arXiv:2310.03359, 2023.

[50] D. Fox, W. Burgard, and S. Thrun, “The Dynamic Window Approach to Collision Avoidance,” IEEE Robotics & Automation Magazine 4:23–33, 1997.

[51] C. Tao, S. Cheng, Y. Zhao, F. Wang, and N. Hovakimyan, “An optimization-based planner with B-spline parameterized continuous-time reference signals,” arXiv:2404.00133v1, 2024.

[52] Phan Gia Luan and Nguyen Truong Thinh, “Real-time hybrid navigation system-based path planning and obstacle avoidance for mobile robots,” Applied Sciences, vol. 10, no. 10, art. 3355, 2020, doi: 10.3390/app10103355.

[53] I.-A. Șomîtcă, S. Brad, V. Florian, and Ș.-E. Deaconu, “Improving Path Accuracy of Mobile Robots in Uncertain Environments by Adapted Bézier Curves,” Electronics 11(21):3568, 2022, doi: 10.3390/electronics11213568.

[54] N. Persson, M. C. Ekström, M. Ekström, and A. V. Papadopoulos, “On the initialization problem for Timed-Elastic Bands,” IFAC-PapersOnLine, vol. 56, no. 2, pp. 11802–11807, 2023, doi: 10.1016/j.ifacol.2023.10.574.

[55] J. Choi, “Kinodynamic motion planning for autonomous vehicles,” International Journal of Advanced Robotic Systems, 2014, doi: 10.5772/58683.

[56] Nav2, “Constrained Smoother,” Nav2 Jazzy API documentation. Available: https://api.nav2.org/nav2-jazzy/html/md_nav2_constrained_smoother_README.html. Accessed: Oct. 22, 2026.

[57] S. Macenski, S. Singh, F. Martín, and J. Ginés, “Regulated pure pursuit for robot path tracking,” Autonomous Robots, vol. 47, pp. 685–694, 2023, doi: 10.1007/s10514-023-10097-6.

[58] R. T. Farouki, “The Bernstein polynomial basis: A centennial retrospective,” Computer Aided Geometric Design, vol. 29, no. 6, pp. 379–419, 2012, doi: 10.1016/j.cagd.2012.03.001.

[59] C. de Boor, “On Calculating with B-Splines,” Journal of Approximation Theory 6:50–62, 1972, doi: 10.1016/0021-9045(72)90080-9.

[60] L. Piegl, “On NURBS: A Survey,” IEEE Computer Graphics and Applications 11(1):55–71, 1991, doi: 10.1109/38.67702.

[61] Z. Habib and M. Sakai, “Circle to Circle Transition with a Single PH Quintic Spiral,” Scientiae Mathematicae Japonicae Online e-2007:361–371, 2007.

[62] H. Delingette, M. Hebert, and K. Ikeuchi, “Trajectory Generation with Curvature Constraint Based on Energy Minimization,” IEEE/RSJ IROS 1991, pp. 206–211, 1991.

[63] K. M. Lynch and F. C. Park, “Modern Robotics: Mechanics, Planning, and Control,” Cambridge University Press (preprint), 2017.

[64] A. M. Shkel and V. Lumelsky, “Classification of the Dubins set,” Robotics and Autonomous Systems 34(4):179–202, 2001, doi: 10.1016/S0921-8890(00)00127-5.

[65] J. Ayala, D. Kirszenblat, and H. Rubinstein, “A Geometric Approach to Shortest Bounded Curvature Paths,” arXiv:1403.4899, 2015.

[66] J. A. Reeds and L. A. Shepp, “Optimal Paths for a Car That Goes Both Forwards and Backwards,” Pacific Journal of Mathematics 145:367–393, 1990.

[67] S. M. LaValle, “Planning Algorithms,” Cambridge University Press, 2006.

[68] S. Macenski, F. Martín, R. White, and J. Ginés Clavero, “The Marathon 2: A navigation system,” arXiv:2003.00368, 2020. Available: https://arxiv.org/abs/2003.00368.

[69] S. Macenski, T. Moore, D. V. Lu, A. Merzlyakov, and M. Ferguson, “From the desks of ROS maintainers: A survey of modern & capable mobile robotics algorithms in the Robot Operating System 2,” Robotics and Autonomous Systems, vol. 168, art. 104493, 2023, doi: 10.1016/j.robot.2023.104493.

[70] Nav2, “Smoother Server,” Rolling configuration documentation, snapshot Oct. 9, 2026. [Online]. Available: https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_smoother_server/.

[71] Nav2, “Simple Smoother,” Rolling configuration documentation, snapshot Oct. 9, 2026. [Online]. Available: https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/smoother_plugins/configuring_simple_smoother/.

[72] Nav2, “Savitzky-Golay Smoother,” Rolling configuration documentation, snapshot Oct. 9, 2026. [Online]. Available: https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/smoother_plugins/savitzky_golay_smoother/configuring_savitzky_golay_smoother/.

[73] Nav2, “Constrained Smoother,” Rolling configuration documentation, snapshot Oct. 9, 2026. [Online]. Available: https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/smoother_plugins/constrained_smoother/configuring_constrained_smoother/.

[74] A. Savitzky and M. J. E. Golay, “Smoothing and Differentiation of Data by Simplified Least Squares Procedures,” Analytical Chemistry 36(8):1627–1639, 1964, doi: 10.1021/ac60214a047.

[75] Nav2, “Smoother server implementation,” navigation2, branch jazzy, nav2_smoother/src/smoother_server.cpp. [Online]. Available: https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_smoother/src/smoother_server.cpp. Accessed: Oct. 13, 2026.

[76] B. Efron, “Bootstrap methods: Another look at the jackknife,” The Annals of Statistics, vol. 7, no. 1, pp. 1–26, 1979, doi: 10.1214/aos/1176344552.

[77] F. Wilcoxon, “Individual comparisons by ranking methods,” Biometrics Bulletin, vol. 1, no. 6, pp. 80–83, 1945, doi: 10.2307/3001968.

[78] S. Holm, “A simple sequentially rejective multiple test procedure,” Scandinavian Journal of Statistics, vol. 6, no. 2, pp. 65–70, 1979.

[79] S. Ahn, T. Oh, and J. Yoo, “Collision Avoidance Path Planning for Automated Vehicles Using Prediction Information and Artificial Potential Field,” Sensors 24(22):7292, 2024, doi: 10.3390/s24227292.

[80] Z. Xu, Y. Xiu, X. Zhan, B. Chen, and K. Shimada, “Vision-aided UAV navigation and dynamic obstacle avoidance using gradient-based B-spline trajectory optimization,” arXiv:2209.07003v3, 2024.


