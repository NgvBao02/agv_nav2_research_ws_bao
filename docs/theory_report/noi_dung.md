# Cơ sở lý thuyết và đối chiếu triển khai dự án AGV PSTMO

Báo cáo này hệ thống hóa cơ sở lý thuyết của robot vi sai, mô hình 3D, mô phỏng Gazebo, ROS 2 Navigation2, các bộ lập kế hoạch, các bộ làm mượt và quy trình đánh giá đang có trong dự án. Mục đích là tạo một tài liệu tham chiếu để đọc công thức, tìm đúng nơi triển khai và kiểm tra giới hạn của từng kết luận. Phiên bản đối chiếu ngày 06 tháng 10 năm 2026; mã nguồn và cấu hình được khóa bằng SHA-256 trong phụ lục.

Kết luận xuyên suốt là: một đường đi trơn về hình học chưa tự bảo đảm robot thực thi đúng, an toàn hoặc nhanh hơn. PSTMO xây dựng và lựa chọn các chuyển tiếp khả thi trong mô hình; Nav2, định vị, mô hình tiếp xúc và giới hạn điều khiển quyết định chuyển động vòng kín. Vì vậy báo cáo tách ba tầng: lý thuyết toán học, hiện thực trong chương trình và bằng chứng mô phỏng. Các hình chụp và đồ thị thực nghiệm được sử dụng lại từ hồ sơ dự án, không phải các lượt chạy mới của lần biên soạn này.

Tài liệu không có trang bìa. Đọc chương 1 để nắm kiến trúc; chương 2–7 cung cấp nền tảng; chương 8–11 đi sâu vào PSTMO và điều khiển; chương 12–15 hướng dẫn đối chiếu, đánh giá và chuyển sang robot thật. Phụ lục chứa thông số, danh mục nguồn và kiểm tra. Các giả thiết chưa được đo trên phần cứng được ghi rõ, không thay bằng số ước đoán.

@TOC

## Chương 1 Phạm vi và kiến trúc của hệ thống

### 1.1 Đối tượng nghiên cứu và các mức biểu diễn

Đối tượng nghiên cứu là một robot di động dẫn động vi sai có hai bánh chủ động, thân hình chữ nhật và các phần tử đỡ tiếp xúc sàn trong SDF. Robot di chuyển trong môi trường phẳng, chủ yếu là bản đồ kho tĩnh. Chuỗi xử lý chính gồm đọc bản đồ, ước lượng tư thế, tìm đường toàn cục, làm mượt, điều khiển bám đường và giám sát va chạm. PSTMO nằm ở tầng hậu xử lý đường toàn cục, không thay thế bộ định vị hoặc bộ điều khiển động cơ.

Một cấu hình robot q gồm vị trí x, y và góc hướng ψ. Đường đi P mô tả thứ tự các cấu hình trong không gian; quỹ đạo q(t) bổ sung thời gian. Hai robot có thể đi trên cùng P nhưng có vận tốc, thời gian hoàn thành, độ trượt và điện năng khác nhau. Trong Nav2, nav_msgs/Path không phải một hợp đồng vận tốc theo thời gian: các PoseStamped không mang đầy đủ vận tốc và gia tốc cần cho một trajectory controller.

$$ q = (x, y, ψ),    P = {q₀, q₁, …, qₙ},    q(t) = P(s(t))

PSTMO dùng thời gian nội bộ để so sánh thao tác tại góc. Hồ sơ này không được công bố thành lệnh SpeedLimit hay được buộc bộ điều khiển RPP phải bám. Khi viết bài báo, cần gọi đầu ra là đường đã làm mượt kèm kiểm tra khả thi theo mô hình, không khẳng định đó là quỹ đạo thời gian được thực thi chính xác.

### 1.2 Năm gói phần mềm và trách nhiệm

| Gói | Trách nhiệm | Dữ liệu hoặc giao diện chính |
| adaptive_pivot_g2 | Hình học, tìm ứng viên, tham số hóa thời gian, chọn trạng thái | Cấu trúc C++ Vec2, PathSample, TransitionCandidate, TimedProfile |
| adaptive_pivot_g2_nav2 | Plugin PSTMO và SafetyGatedHybrid, kiểm tra footprint | nav2_core::Smoother, costmap, footprint, diagnostics |
| adaptive_pivot_g2_benchmark | So sánh hình học, thực thi, đo chỉ số và ma trận thử | ROS actions, JSON, CSV, path hash |
| adaptive_pivot_g2_rviz | Bảng chọn môi trường, planner và phương pháp | Panel RViz2 và topic lựa chọn |
| vacuum_robot_gazebo | URDF, SDF, cảm biến, map, world và launch | Gazebo, ros_gz_bridge, Nav2, robot_state_publisher |

Các thư mục build, install và log là sản phẩm xây dựng, không phải nguồn lý thuyết độc lập. Khi src đã thay đổi nhưng install chưa cập nhật, chương trình có thể chạy bản cũ. Vì vậy phải phân biệt hash nguồn với bằng chứng thư viện thực sự đã được nạp. Các thử nghiệm MATLAB và báo cáo lịch sử là nguồn tham khảo tiến hóa thuật toán, không mặc nhiên là nhánh runtime hiện tại.

### 1.3 Hai luồng thực thi cần phân biệt

Luồng benchmark tính một đường Raw bằng planner đã chọn, gửi bản sao chính xác của nó tới từng smoother, rồi dùng FollowPath để đánh giá bám đường. Luồng NavigateToPose do behavior tree điều phối có thể tự tính lại đường, phục hồi và sử dụng planner mặc định. Chỉ khai báo một smoother trong YAML không chứng minh mọi NavigateToPose đều gọi smoother đó. Cần kiểm tra XML của behavior tree, action SmoothPath và diagnostics trước khi gán kết quả cho PSTMO.

Hồ sơ mô hình 3D có một lượt NavigateToPose/ThetaStar thành công trong kho. Lượt này là minh họa tích hợp, không phải phép chứng minh PSTMO được thực thi. Lần khởi tạo trước đó chọn GridBased không tồn tại nên abort; hồ sơ đã giữ cả lỗi và lần chạy chọn đúng ThetaStar. Đây là ví dụ vì sao phải kiểm tra luồng thực thi thay vì chỉ nhìn đường màu trên RViz.

![R/figures/model_pipeline.png|Các mức STEP, URDF, SDF và dữ liệu chuyển động trong hồ sơ 3D. Sơ đồ kỹ thuật của dự án; không phải ảnh đo phần cứng.]

### 1.4 Quy ước bằng chứng và phạm vi đầy đủ

Trong báo cáo, “cơ sở lý thuyết” là phương trình và điều kiện áp dụng; “cấu hình hiện tại” là giá trị đọc trực tiếp từ tệp; “kết quả lưu trữ” là số liệu đã có trong JSON/CSV hoặc báo cáo kèm nguồn; “cần kiểm chứng” là điều chưa có phép đo xác nhận. Phạm vi bao phủ các mô-đun hiện có trong src, mô hình và công cụ đánh giá liên quan. Không có tài liệu hữu hạn nào bao phủ mọi lý thuyết của robot học; các nền tảng được chọn theo những gì hệ thống này sử dụng hoặc cần để diễn giải đúng kết quả.

## Chương 2 Hình học phẳng và hệ tọa độ

### 2.1 Vectơ hướng và góc có dấu

Với vectơ a và b trong mặt phẳng, tích vô hướng xác định độ tương đồng hướng; tích có hướng vô hướng xác định chiều quay. Dùng atan2 thay vì arccos giúp giữ dấu góc và ổn định hơn khi góc gần 0 hoặc π. Trước khi dùng hướng cạnh để dựng đường cong phải chuẩn hóa vectơ, loại cạnh có độ dài gần 0 và kiểm tra giá trị hữu hạn.

$$ a·b = aₓbₓ + aᵧbᵧ,    a×b = aₓbᵧ − aᵧbₓ
$$ θ = atan2(a×b, a·b),    wrap(θ) = atan2(sin θ, cos θ)

Quy ước chuyển động trong mặt phẳng là x hướng trước, y sang trái, z lên trên ở hệ thân robot. Góc yaw dương quay ngược chiều kim đồng hồ khi nhìn từ trên. Đổi trục CAD không đúng sẽ đổi dấu quay, đảo trái/phải hoặc đặt cảm biến sai vị trí mặc dù hình nhìn gần giống nhau.

### 2.2 Biến đổi tư thế và chuỗi TF

Phép quay phẳng và tịnh tiến đưa điểm f trong hệ thân sang hệ bản đồ. Ma trận thuần nhất thuộc SE(2) cho phép ghép nhiều phép biến đổi; thứ tự nhân không được đổi tùy ý. Đảo biến đổi cần quay ngược cả phần tịnh tiến, không chỉ đổi dấu x và y.

$$ xₘ = x + fₓ cos ψ − fᵧ sin ψ,    yₘ = y + fₓ sin ψ + fᵧ cos ψ
$$ Tₐ꜀ = Tₐᵦ Tᵦ꜀,    T⁻¹ = [Rᵀ, −Rᵀt; 0, 1]

Trong hệ thống, map là hệ tham chiếu định vị toàn cục, odom là hệ tích phân chuyển động cục bộ, base_link gắn với robot. AMCL cung cấp hiệu chỉnh map→odom; odometry cung cấp odom→base_link; robot_state_publisher tạo các quan hệ từ base_link tới bánh và cảm biến. base_footprint là link cố định dưới base_link trong URDF này, không được tự đổi cây TF thành kiến trúc của một robot mẫu khác.

![R/figures/tf_tree.png|Cây TF đọc từ mô hình dự án. Phải phân biệt link cố định, khớp bánh quay và phép biến đổi định vị.]
![R/screenshots/rviz_tf.png|Ảnh RViz2 đã lưu trong hồ sơ 3D hiển thị hệ tọa độ của robot. Đây là ảnh ứng dụng, không phải sơ đồ sinh từ công thức.]

### 2.3 Quaternion và thời gian của phép biến đổi

Với chuyển động phẳng, quaternion yaw có qx=qy=0, qz=sin(ψ/2), qw=cos(ψ/2). Hai quaternion q và −q biểu diễn cùng phép quay nên không so sánh hướng bằng hiệu từng thành phần. Sai số yaw phải được bọc về khoảng góc chính trước khi tính trị tuyệt đối hoặc RMSE.

$$ qz = sin(ψ/2),    qw = cos(ψ/2),    eψ = atan2(sin(ψ̂−ψ), cos(ψ̂−ψ))

TF là dữ liệu theo thời gian. Một scan phát tại t phải được biến đổi bằng tư thế tương ứng t; lấy tư thế mới nhất thay cho t gây sai lệch khi robot đang quay. use_sim_time phải thống nhất trong Gazebo, Nav2, RViz và các recorder. Dữ liệu world của Gazebo chỉ so sánh trực tiếp với map khi đã xác nhận phép đăng ký hai hệ. Khi teleport reset robot, odom có thể vẫn tích lũy; phép đánh giá phải tái căn chỉnh hoặc khởi động lại hệ phù hợp.

### 2.4 Khoảng cách từ điểm tới đoạn và tham số chiều dài cung

Chiếu điểm p lên đoạn [a,b] bằng tỷ lệ λ bị chặn trong [0,1]. Khoảng cách tới đường gấp khúc là giá trị nhỏ nhất qua các đoạn, không chỉ qua các đỉnh. Công thức này được dùng trong điều kiện hóa đường và sai số bám. Tuy nhiên ở lối giao cắt, phép chiếu gần nhất toàn cục có thể nhảy sang nhánh khác; chỉ số theo tiến độ cần ràng buộc miền tìm hoặc dùng logic chiếu đơn điệu.

$$ λ = clip([frac:(p−a)·(b−a)|‖b−a‖²], 0, 1),    d(p,[a,b]) = ‖p−a−λ(b−a)‖
$$ L = Σ ‖pᵢ₊₁−pᵢ‖,    s(u) = ∫₀ᵘ ‖B′(ξ)‖ dξ

Lấy mẫu đều theo tham số u không đồng nghĩa đều theo chiều dài cung s. Khi đối chiếu các đường có mật độ điểm khác nhau, cần công bố bước lấy mẫu và cách nội suy. Ngay cả khi mọi hình có vẻ trơn giống nhau, một chỉ số dựa trên ba điểm có thể thay đổi đáng kể khi bước mẫu thay đổi.

## Chương 3 Mô hình cơ khí và động lực học của robot

### 3.1 Vai trò khác nhau của STEP STL URDF và SDF

STEP là biểu diễn CAD chứa các mặt và khối có cấu trúc; STL là lưới tam giác phục vụ hiển thị; URDF mô tả cây link/joint và hình học robot cho ROS; SDF mô tả mô hình vật lý, collision, inertia, cảm biến và plugin Gazebo. Việc các tệp mang cùng tên robot không bảo đảm chúng cùng revision. Đối chiếu cần kiểm tra trục, đơn vị, kích thước, vị trí khớp và hình học va chạm.

Hồ sơ 3D đo file_3D/Xe.step có 542 solid và 5.287 mặt không nằm trong solid. Số 168 PRODUCT record không phải số lượng linh kiện thực đã xác nhận. Bao toàn STEP khoảng 440 × 340 × 166,374 mm sau quy trục; hình bao hiển thị thân URDF khoảng 440 × 340 × 160 mm. Các mặt ngoài solid có thể mang bề mặt lốp nên không được bỏ qua khi đo bao hình học.

![R/figures/cad_iso_front.png|Mô hình STEP dựng từ tệp Xe.step. Hình CAD giải thích cấu tạo, không phải ảnh chụp robot thật.]
![R/figures/cad_exploded.png|Hình tách lớp CAD để đọc cấu trúc. Dịch chuyển bộ phận chỉ phục vụ trình bày, không thay vị trí trong tệp nguồn.]
![R/figures/cad_urdf_overlay.png|Đối chiếu hình học CAD và URDF sau quy trục. Trùng bao thân không có nghĩa mọi chi tiết hay vị trí bánh đều trùng.]

### 3.2 Khoảng cách bánh và ý nghĩa hiệu chuẩn

Có ba đại lượng khác nhau cần giữ nguyên tên. Tâm phần lốp STEP cho khoảng cách 230,8 mm; URDF/SDF hình học và giới hạn PSTMO dùng 254,8 mm; plugin DiffDrive dùng 283,4 mm làm khoảng cách hiệu dụng hiệu chuẩn. Số cuối bù hành vi quay trong mô phỏng, không phải kích thước cơ khí đo được. Không được dùng ba số thay thế lẫn nhau khi suy ra vận tốc bánh hoặc sai số quay.

| Đại lượng | Giá trị | Nguồn và cách sử dụng |
| Vệt bánh theo STEP | 0,2308 m | Hồ sơ CAD; kiểm tra revision cơ khí |
| Vệt bánh hình học URDF/SDF | 0,2548 m | Hai tâm bánh y=±0,1274 m |
| Vệt bánh trong giới hạn PSTMO | 0,2548 m | types.hpp và nav2_params.yaml |
| Vệt bánh hiệu dụng DiffDrive | 0,2834 m | model.sdf, hiệu chuẩn mô phỏng |
| Bán kính lăn khai báo | 0,0425 m | Collision bánh và plugin DiffDrive |

![R/figures/track_difference.png|Ba ý nghĩa của khoảng cách hai bánh trong hồ sơ dự án. Không diễn giải giá trị hiệu dụng là đo kích thước vật lý.]
![R/figures/geometry_layout.png|Kích thước thân, tâm bánh và cảm biến từ mô hình robot.]

### 3.3 Cây khớp và cách đặt hình học

URDF có 8 link và 7 joint: base_link là gốc; base_footprint, laser, imu và hai động cơ gắn bằng fixed joint; hai bánh dùng continuous joint với trục quay theo +Y. Tâm cảm biến laser ở z=0,10892 m so với base_link; IMU ở z=−0,0128 m. base_footprint thấp hơn base_link 0,0425 m. Các mesh dùng scale=0,001 để chuyển mm sang m. Origin của visual không phải origin của joint; bù tọa độ mesh sai có thể làm mô hình đẹp trên RViz nhưng sai cơ học trong Gazebo.

URDF gốc không có inertia tại base_link trong cấu hình này để tránh cảnh báo KDL; khối lượng mô phỏng được xác định trong SDF. Phải đọc đúng tệp vật lý đang spawn. robot_state_publisher không tự tạo contact, mô-men hay động lực học; nó công bố quan hệ hình học theo joint_states.

![R/figures/urdf_exploded.png|Phân rã các thành phần hiển thị URDF để đọc vị trí tương đối và khớp.]
![R/screenshots/rviz_collision.png|RViz2 hiển thị mô hình va chạm đã khai báo; collision là mô hình đơn giản hóa, không phải toàn bộ bề mặt STEP.]

### 3.4 Khối lượng tâm khối và tensor quán tính

Khối lượng là đầu vào mô phỏng, không thể suy ra từ thể tích STEP nếu chưa biết vật liệu, phần rỗng và các khối chồng lấn. SDF khai báo thân 4,6 kg và mỗi bánh 0,2 kg, tổng 5,0 kg. Tâm khối tổng hợp trong hệ sàn khoảng (−0,005588; −0,000493; 0,057241) m theo hồ sơ tính từ SDF. Đây không phải kết quả cân và đo tâm khối robot thật.

$$ M = Σ mⱼ,    r꜀ = [frac:Σ mⱼrⱼ|M]
$$ I꜀ = Σ {RⱼIⱼRⱼᵀ + mⱼ[(dⱼ·dⱼ)I − dⱼdⱼᵀ]},    dⱼ = rⱼ − r꜀

Định lý trục song song cộng quán tính từng bộ phận tại cùng tâm và cùng hệ trục. Tensor phải đối xứng, có trị riêng dương và thỏa điều kiện vật lý thích hợp; một ma trận chỉ có số dương trên đường chéo chưa đủ. Những kiểm tra đó xác nhận tính nhất quán của dữ liệu khai báo, không xác nhận vật liệu hay phân bố khối lượng thực.

![R/figures/mass_inertia.png|Khối lượng, tâm khối và quán tính tính từ SDF. Các số là thuộc tính mô hình, không phải phép đo thực.]

### 3.5 Contact ma sát và sai khác mô hình lý tưởng

Mô hình collision dùng các hộp thân, hai cylinder bánh và bốn cầu đỡ nhỏ. Phần thân collision gồm một hộp giữa, hai hộp đầu và một hộp trên, nên không chứa mọi chi tiết CAD. Ma sát bánh khai báo mu=1,2, mu2=0,05 và slip2=0,02; các cầu đỡ có ma sát nhỏ. Các tham số có ý nghĩa phụ thuộc backend vật lý; không được khẳng định hệ đang dùng engine ODE chỉ vì một trường cấu hình nhắc ODE. Hồ sơ chạy trước quan sát DART với collision detector ODE.

Quan hệ lực tổng quát là tổng lực bằng khối lượng nhân gia tốc và tổng mô-men bằng tốc độ biến thiên động lượng góc. Với mô hình phẳng lý tưởng, lực dọc do hai bánh và chênh lệch lực tạo mô-men quay. Trượt ngang, lực đỡ và động lực học bánh làm robot thực hoặc Gazebo lệch khỏi mô hình vi sai không trượt. Do đó hiệu chuẩn theo ground truth là bước riêng, không phải lý do để đổi kích thước CAD cho khớp dữ liệu.

$$ M v̇ = Fᴸ + Fᴿ − F꜀,    Izz ω̇ = (b/2)(Fᴿ−Fᴸ) − τ꜀

F꜀ và τ꜀ ký hiệu lực cản và mô-men cản tương đương, không phải hai đại lượng đã được nhận dạng trong dự án. Hệ chưa có bộ nhận dạng đầy đủ ma sát, mô-men tải và hiệu suất truyền động; các công thức này là cơ sở để thiết kế phép đo tiếp theo.

## Chương 4 Động học vi sai và các giới hạn chuyển động

### 4.1 Động học thuận và nghịch

Giả thiết cơ bản là hai bánh có cùng bán kính r, tiếp xúc một mặt phẳng, lăn không trượt theo phương dọc và robot không có vận tốc ngang trong hệ thân. Gọi ΩL, ΩR là tốc độ góc bánh; vL=rΩL và vR=rΩR là vận tốc tiếp tuyến. Vận tốc tịnh tiến tâm trục bằng trung bình hai bánh, vận tốc yaw bằng chênh lệch chia cho khoảng cách bánh b.

$$ v = [frac:vᴿ+vᴸ|2],    ω = [frac:vᴿ−vᴸ|b]
$$ vᴸ = v − bω/2,    vᴿ = v + bω/2,    Ωᴸ = vᴸ/r,    Ωᴿ = vᴿ/r
$$ ẋ = v cos ψ,    ẏ = v sin ψ,    ψ̇ = ω

Hai bánh cùng dấu và bằng nhau tạo chuyển động thẳng; cùng dấu nhưng khác nhau tạo cung; trái dấu tạo quay tại chỗ khi trung bình bằng 0. Khả năng pivot là đặc tính của robot vi sai, nhưng không có nghĩa pivot luôn an toàn ở một vị trí: cả thân robot quét một vùng lớn hơn bề rộng thân khi xoay.

### 4.2 Tích phân odometry và hiệu chuẩn

Trong một khoảng lấy mẫu, từ hai quãng lăn ΔsL và ΔsR suy ra Δs và Δψ. Xấp xỉ midpoint đưa robot tiến theo góc trung bình ψ+Δψ/2. Khi cần chính xác hơn, tích phân cung tròn dùng sin(Δψ/2)/(Δψ/2); khi Δψ tiến về 0 dùng giới hạn 1 để tránh chia gần 0. Encoder thiếu xung, bán kính hữu hiệu thay đổi và trượt bánh gây sai số tích lũy; AMCL chỉ hiệu chỉnh định vị toàn cục chứ không làm các phép đo encoder trở thành đúng.

$$ Δs = (Δsᴿ+Δsᴸ)/2,    Δψ = (Δsᴿ−Δsᴸ)/b
$$ xₖ₊₁ ≈ xₖ + Δs cos(ψₖ+Δψ/2),    yₖ₊₁ ≈ yₖ + Δs sin(ψₖ+Δψ/2)

Sai số thẳng thường nhạy với tỷ lệ bán kính; sai số quay nhạy với tỷ số bán kính/vệt bánh. Không thể hiệu chuẩn tốt cả hai chỉ từ một đoạn thẳng. Cần thẳng, lùi, quay hai chiều và cung hai chiều, giữ dữ liệu độc lập để kiểm chứng. Hồ sơ 21 lượt trước đã có bảy dạng chuyển động, nhưng chưa bao phủ nhiều tải và mặt sàn.

![R/figures/motion_forward.png|Phép thử đi thẳng trong Gazebo từ hồ sơ 21 lượt. Dữ liệu mô phỏng được dùng để minh họa cách đối chiếu odometry và ground truth.]
![R/figures/motion_pivot_ccw.png|Phép thử quay trái tại chỗ, minh họa vai trò của góc quay và vệt bánh hiệu dụng.]
![R/figures/motion_arc_left.png|Phép thử chạy cung trái, kiểm tra đồng thời phần tịnh tiến và phần quay của mô hình.]

### 4.3 Độ cong và điều kiện không đảo bánh trong

Với v>0, độ cong quỹ đạo κ=ω/v. Thay vào động học nghịch cho các hệ số 1±bκ/2. Trong pha chuyển tiếp tiến của PSTMO, bánh trong không được đảo chiều; điều kiện này cho |κ|≤2/b. Với b=0,2548 m, giới hạn hình học tương ứng khoảng 7,8493 m⁻¹, bán kính tối thiểu tương ứng 0,1274 m. Đây là giới hạn không đảo bánh, không phải bán kính thiết kế 0,35 m của SmacHybrid và cũng không phải giới hạn pivot của robot.

$$ κ = ω/v,    vᴸ = v(1−bκ/2),    vᴿ = v(1+bκ/2),    |κ| ≤ 2/b

Tại pivot, v=0 và ω khác 0 nên κ=ω/v không xác định. Không được cho pivot độ cong bằng 0 rồi tuyên bố đường có độ cong rất thấp. Phải tách số pivot và tổng góc pivot khỏi chỉ số hình học của phần tịnh tiến.

### 4.4 Miền vận tốc khả thi và ví dụ số

Giới hạn v, ω, vận tốc bánh và gia tốc ngang tạo các ràng buộc đồng thời. Một cặp lệnh nằm trong hình chữ nhật |v|≤0,30 và |ω|≤0,80 vẫn có thể vượt giới hạn bánh 0,36 m/s. Cụ thể v=0,30, ω=0,80, b=0,2548 cho bánh ngoài 0,40192 m/s. Vì vậy giới hạn từng thành phần Twist độc lập không tương đương giới hạn actuator.

$$ v꜀ₐₚ(κ) = min(vₘₐₓ, [frac:ωₘₐₓ||κ|], √([frac:aᵧₘₐₓ||κ|]), [frac:v_wmax|max(|1−bκ/2|,|1+bκ/2|)])

Ở κ=2 m⁻¹, ràng buộc bánh cho v≤0,36/1,2548≈0,2869 m/s; ràng buộc yaw cho 0,40 m/s và gia tốc ngang cho 0,30 m/s. Ràng buộc bánh là chặt nhất. Khi κ=0, các hạng chia |κ| được bỏ qua bằng nhánh điều kiện, không tính trực tiếp chia 0.

![R/figures/velocity_envelope.png|Miền vận tốc và giới hạn bánh trong hồ sơ mô hình. Các ràng buộc thân và bánh không tương đương.]

### 4.5 Dừng xe và khác biệt giữa dự báo với phép đo

Mô hình giảm tốc hằng a cho quãng dừng v²/(2a), cộng quãng đi trong độ trễ τ. Từ 0,30 m/s với a=0,45 m/s², phần giảm tốc lý tưởng là 0,10 m. Hồ sơ mô phỏng trước đo khoảng 0,10078 m sau lệnh zero, nhưng tiêu chí của phép thử đó là lần đầu vận tốc thấp hơn ngưỡng, không phải một dwell ổn định dài. Không dùng kết quả này để tuyên bố khoảng dừng an toàn của phần cứng.

$$ dₛₜₒₚ ≈ vτ + [frac:v²|2a],    tₛₜₒₚ ≈ τ + v/a

![R/figures/braking_calibration.png|Đối chiếu phanh trong mô phỏng. Phải giữ nguyên định nghĩa thời điểm zero, ngưỡng dừng và cửa sổ đo khi so sánh.]

## Chương 5 Cảm biến định vị và truyền thông ROS 2

### 5.1 LiDAR IMU và joint states

SDF khai báo LiDAR GPU 1.440 mẫu trên miền góc −π đến π, tần số 5,5 Hz, dải 0,15–12 m, độ phân giải khoảng cách 0,01 m và nhiễu Gaussian σ=0,01 m. Một scan chứa nhiều tia nhưng AMCL hiện lấy tối đa 180 beam; không được đồng nhất số tia sensor với số tia dùng trong định vị. LiDAR nằm khoảng 0,15142 m trên sàn theo hình học SDF, nên chướng ngại ngoài mặt phẳng quét có thể không được quan sát.

IMU khai báo 100 Hz, nhiễu gyro σ=0,002 và gia tốc σ=0,03 theo các trường sensor. Gyro đo tốc độ góc, accelerometer đo đại lượng liên quan gia tốc riêng; dữ liệu orientation phụ thuộc mô hình/plugin. Các covariance và đơn vị phải được kiểm tra trước khi fuse. Joint states cung cấp góc/vận tốc khớp, không tự thay thế wheel odometry với covariance phù hợp.

![R/screenshots/rviz_scan.png|LaserScan trong RViz2, lấy từ hồ sơ 3D. Mây tia là dữ liệu cảm biến mô phỏng đã biến đổi qua TF.]
![R/figures/paths_scan.png|Đường đi và scan trong hồ sơ robot, minh họa liên hệ giữa hình học môi trường và cảm biến.]
![R/figures/sample_rates.png|Nhịp dữ liệu đã ghi của các topic. Tần số cấu hình và tần số timestamp quan sát cần báo riêng.]

### 5.2 Bộ lọc Bayes và AMCL

Định vị Bayes duy trì phân bố xác suất của tư thế. Bước dự đoán dùng mô hình chuyển động và điều khiển/odometry; bước cập nhật nhân likelihood quan sát. AMCL biểu diễn phân bố bằng các hạt có trọng số và thay đổi số hạt thích nghi. Mô hình likelihood field đánh giá mức phù hợp của đầu tia scan với vật cản trên bản đồ; đây không phải SLAM vì bản đồ đã biết. Launch hiện đặt slam=False và tải map có sẵn. [T1]

$$ p(qₜ|z₁:ₜ,u₁:ₜ) ∝ p(zₜ|qₜ) ∫ p(qₜ|qₜ₋₁,uₜ) p(qₜ₋₁|z₁:ₜ₋₁,u₁:ₜ₋₁) dqₜ₋₁

Các giá trị hiện tại gồm 1.000–3.000 particles, random_seed=42, alpha1…alpha5=0,005, sigma_hit=0,15 m, z_hit=0,65 và z_rand=0,35. Đây là tuning mô phỏng với odom đã hiệu chuẩn; không bê nguyên sang phần cứng nhiều trượt. Một alpha nhỏ biểu thị tin mô hình chuyển động hơn, không có nghĩa odometry đã đạt một độ chính xác được bảo đảm.

Kệ kho lặp lại tạo khả năng nhầm vị trí giữa các hành lang tương tự. Khởi tạo sai yaw có thể chiếu scan qua tường và tạo lỗi cục bộ. simulation.launch.py ghi lại initial_pose của AMCL theo x_pose, y_pose, yaw spawn để tránh hai hệ khởi tạo bất nhất. Khi đánh giá localization, cần so AMCL với ground truth theo timestamp và hệ tọa độ, không dùng độ nhỏ của covariance làm thay thế sai số thật.

### 5.3 EKF dành cho robot thật và trạng thái triển khai

EKF tuyến tính hóa mô hình phi tuyến quanh ước lượng hiện tại, truyền covariance qua Jacobian và hiệu chỉnh theo innovation. Tệp ekf_real.yaml dự kiến chạy 30 Hz, two_d_mode=true, world_frame=odom. Nó fuse vx, vy, yaw rate của /wheel/odometry và chỉ yaw rate từ /imu/data. Không fuse hướng từ kế tuyệt đối trong cấu hình này. Chưa có bằng chứng trong baseline Gazebo rằng EKF thật được launch cùng mô phỏng; đây là cấu hình chuẩn bị cho phần cứng.

$$ x̂⁻ = f(x̂,u),    P⁻ = FPFᵀ + Q
$$ K = P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹,    x̂ = x̂⁻ + K(z−h(x̂⁻))

Q mô tả bất định mô hình, R mô tả bất định đo; không chọn cực nhỏ chỉ để đồ thị trông ổn định. Nếu cùng thông tin encoder xuất hiện ở nhiều đầu vào tương quan mà bị xem độc lập, bộ lọc có thể quá tự tin. Khi EKF sở hữu odom→base_link, node phần cứng không được phát thêm cùng TF; hai nguồn tranh quyền gây giật tư thế và sai thời gian transform.

### 5.4 Topic service action QoS và lifecycle

Topic là luồng dữ liệu liên tục như scan, odom, cmd_vel; service phù hợp tác vụ yêu cầu–đáp ứng ngắn; action cho tác vụ dài có feedback, cancel và result như ComputePathToPose, SmoothPath, FollowPath và NavigateToPose. Một action SUCCEEDED phản ánh tiêu chí server, không tự đồng nghĩa robot đã dừng vật lý tại ground truth goal.

QoS quy định reliability, durability và lịch sử bản tin. Reliable không chữa được dữ liệu chậm; best effort có thể phù hợp cảm biến tần số cao nhưng recorder cần phát hiện thiếu mẫu. Transient local cho phép nhận giá trị đã lưu từ publisher còn sống, thường hữu ích cho map và lựa chọn planner. QoS không tương thích có thể làm topic có publisher nhưng subscriber không nhận. ROS_DOMAIN_ID tách discovery DDS; GZ_PARTITION tách kênh Gazebo. Khi chạy nhiều ma trận, cần tách cả hai chứ không chỉ tên node.

Lifecycle quản lý configure, activate, deactivate, cleanup. Một node tồn tại trong graph chưa chứng minh action server sẵn sàng. Launch có các khoảng chờ và điều kiện khởi động; benchmark phải chờ clock, TF, map và server hợp lệ. Timeout hạ tầng cần phân biệt với timeout thuật toán hoặc robot mắc kẹt, tránh loại lỗi có lợi cho một phương pháp.

## Chương 6 Bản đồ chi phí và hình bao an toàn

### 6.1 Bản đồ chiếm chỗ và phép đổi tọa độ

Map kho giao cắt dùng PGM trinary, resolution=0,05 m/cell, origin=(−6,−4,0), negate=0, occupied_thresh=0,65 và free_thresh=0,25. PGM là ảnh lưu theo hàng từ trên xuống, trong khi tọa độ map tăng theo chiều y toán học; phải đảo hàng đúng khi chuyển ảnh sang world. Đọc map bằng ảnh có nội suy hoặc anti-alias rồi dùng làm ground truth va chạm có thể thay đổi biên vật cản.

$$ x = x₀ + (i+1/2)r,    y = y₀ + (j+1/2)r

Công thức trên áp dụng khi i,j là chỉ số ô theo hệ map và origin yaw=0. Với origin quay, phải áp dụng thêm R(ψ₀). Khi dùng chỉ số hàng ảnh h, j=H−1−h. Ranh giới ô gây sai số lượng tử nên clearance từ PGM không phải một phép đo hình học liên tục chính xác vô hạn.

![W/map_and_footprint_dimensions.png|Kích thước bản đồ kho giao cắt và footprint từ hồ sơ năm tuyến. Đây là hình phân tích map, không phải bản đồ SLAM mới.]

### 6.2 Costmap và inflation

OccupancyGrid và Costmap2D không dùng cùng thang ý nghĩa. Costmap có giá trị tự do, chi phí tăng gần vật cản, inscribed inflated obstacle=253, lethal=254 và unknown=255. Inflation tạo trường chi phí giúp planner ưu tiên khoảng hở. Một dạng mô hình bên ngoài bán kính nội tiếp là chi phí suy giảm mũ theo khoảng cách; quantization và cách lấy khoảng cách của hiện thực quyết định giá trị nguyên thực tế.

$$ C(d) ≈ 252 exp[−k(d−rᵢₙₛ)]    khi d > rᵢₙₛ trong miền inflation

Cấu hình hiện tại có inflation_radius=0,45 m, cost_scaling_factor=5. Global costmap chỉ kích hoạt static_layer và inflation_layer để giữ đầu vào toàn cục ổn định giữa các phép thử. Obstacle_layer còn một khối cấu hình nhưng không nằm trong danh sách plugins đang bật. Local costmap dùng voxel_layer và inflation_layer, kích thước 4 × 4 m, cập nhật 5 Hz và công bố 2 Hz; nó tiếp nhận scan sống để kiểm tra nguy cơ cục bộ.

### 6.3 Hình bao và không gian cấu hình

Footprint hình chữ nhật có các góc (±0,22;±0,17) m. Vật cản đối với robot có kích thước được hiểu trong không gian cấu hình: một vị trí tâm tự do chưa đủ vì thân có thể đè lên kệ. Với hướng cố định, có thể hình dung nở vật cản bằng hình phản chiếu footprint; khi hướng thay đổi, vùng cấm phụ thuộc ψ và trở thành bài toán SE(2).

$$ F(q) = {t + R(ψ)f : f ∈ Fᵦ},    S = ⋃ₜ F(q(t))

Vùng quét S bao gồm cả tịnh tiến và quay. Bán kính từ tâm tới góc thân là √(0,22²+0,17²)≈0,27803 m, đường kính quét tròn khoảng 0,55606 m. Con số này giúp hiểu tại sao robot rộng 0,34 m chưa chắc quay tại chỗ được trong lối rộng 0,40 m; nó không phải điều kiện cần cho mọi thao tác quay không trọn vòng.

![R/figures/swept_footprint.png|Vùng quét hình bao thân khi di chuyển hoặc pivot. Phải kiểm tra toàn bộ thân, không chỉ tâm hay hai bánh.]

### 6.4 Hiện thực footprint safety và giới hạn lấy mẫu

pose_is_safe kiểm tra cost ở tâm không vượt max_footprint_cost=252; kiểm tra polygon được tô kín không chứa ô lethal hoặc unknown; sau đó kiểm tra biên footprint bằng collision checker. Việc quét nội thất polygon tránh bỏ sót vật cản nhỏ nằm hoàn toàn bên trong thân. Inflation là chính sách chi phí tại tâm; không biến mọi ô inflated trong footprint thành lethal lần thứ hai.

line_is_safe chia đoạn theo bước max(0,005;0,5×resolution), tức 0,025 m ở map hiện tại. pivot_is_safe chọn bước góc từ cùng bước quét tuyến tính chia bán kính footprint. Đây là phép kiểm tra rời rạc theo độ phân giải, không phải chứng minh va chạm liên tục cho mọi hình vật cản. Sai số định vị, thời gian và tiếp xúc thật vẫn cần lớp kiểm tra khi thực thi.

Ở điểm bắt đầu, output PSTMO thường đặt yaw theo cạnh tịnh tiến đầu, không giữ nguyên yaw đầu vào. Do đó cần kiểm tra riêng thao tác xoay từ yaw robot hiện tại sang hướng đường. Mô-đun initial_heading và path_contract phục vụ hợp đồng đầu vào/kiểm tra của benchmark; không được coi hậu kiểm đường đã bao phủ mọi thao tác phát sinh bên ngoài đường xuất ra.

## Chương 7 Cơ sở của năm bộ lập kế hoạch

### 7.1 Đồ thị tìm kiếm và Dijkstra

Bản đồ lưới được diễn giải thành đồ thị với ô hoặc trạng thái là đỉnh và chuyển động hợp lệ là cạnh. Hàm chi phí cạnh có thể chứa chiều dài và chi phí đi gần vật cản. Dijkstra mở rộng đỉnh có chi phí tích lũy g nhỏ nhất; tính tối ưu của nó yêu cầu cạnh không âm và đúng tiêu chuẩn chi phí đang xét. “Ngắn nhất” không nhất thiết là ngắn nhất Euclid nếu chi phí đi qua vùng inflation cũng được cộng.

$$ g(v) ← min[g(v), g(u)+c(u,v)]

NavFnDijkstra trong dự án là NavfnPlanner với use_astar=false, tolerance=0,10 m, allow_unknown=false. NavFn làm việc với trường thế trên lưới và truy xuất đường; không nên mô tả đầu ra phần mềm là nguyên xi một chuỗi đỉnh của thuật toán sách giáo khoa. Raw của nó có thể có mật độ điểm hoặc dao động cần điều kiện hóa trước khi đo độ cong.

### 7.2 A star và vai trò heuristic

A* dùng f=g+h; h ước lượng phần chi phí còn lại. Heuristic admissible không vượt chi phí thật hỗ trợ bảo đảm tối ưu trong điều kiện thuật toán phù hợp; consistency giúp xử lý đóng/mở đỉnh thuận lợi. Tăng trọng số heuristic để tìm nhanh hơn là đánh đổi riêng, không được gán cho cấu hình nếu mã không thực hiện. NavFnAStar dùng cùng NavfnPlanner nhưng use_astar=true, giúp so sánh trong cùng họ planner.

$$ f(n)=g(n)+h(n),    h(n)=‖pₙ−p_goal‖    trong mô hình khoảng cách phù hợp

### 7.3 Theta star và đường nhìn trực tiếp

Theta* thử nối một đỉnh với cha của đỉnh hiện tại nếu có line of sight, nhờ đó đường không bị giới hạn hoàn toàn theo các hướng cạnh lưới. Trong dự án how_many_corners=8, trọng số Euclid 1,0 và traversal cost 2,0. Tính hợp lệ của line of sight trong planner và tính hợp lệ của swept footprint trong smoother là hai tầng khác nhau; đoạn thẳng hợp lệ theo tiêu chuẩn planner không bảo đảm mọi hướng thân khi quay đều an toàn.

Kết quả ThetaStar thường có ít góc neo và đoạn dài, nhưng đây là xu hướng phụ thuộc map và cost, không phải định lý hiệu năng. Khi đánh giá smoother cần giữ nguyên đường Raw của đúng planner; không so PSTMO trên ThetaStar với Simple trên NavFn rồi gán toàn bộ khác biệt cho smoothing.

### 7.4 Smac2D và SmacHybrid

Smac2D là planner tìm kiếm trên lưới 2D có chi phí và các tối ưu triển khai. Trong bản Jazzy đang cấu hình, hậu xử lý nhẹ tích hợp của Smac2D được coi là một phần planner, vì YAML đã ghi không có công tắc tắt tương đương trong giao diện đang dùng. Do đó Raw của Smac2D có thể đã qua bước làm mượt nội bộ, không phải “chưa xử lý dưới mọi hình thức”.

SmacHybrid tìm trên trạng thái (x,y,ψ), với 72 ô góc và motion_model_for_search=DUBIN, minimum_turning_radius=0,35 m. Dubins ràng buộc chuyển động tiến và bán kính cong; robot vi sai thật có thể pivot nên đây là một giả thiết động học khác. smooth_path=false tắt hậu xử lý của Hybrid trong cấu hình này. reverse_penalty tồn tại trong YAML không có nghĩa nhánh DUBIN hiện tại cho phép lùi.

| Planner ID | Không gian chính | Điểm cần giữ khi diễn giải |
| NavFnDijkstra | Lưới 2D | Không ràng buộc trực tiếp hướng thân và bán kính quay |
| NavFnAStar | Lưới 2D có heuristic | Cùng họ NavFn nhưng cách tìm trường thế khác |
| ThetaStar | Lưới với nối any angle | Ít phụ thuộc hướng lưới; vẫn cần kiểm tra thân robot |
| Smac2D | Tìm kiếm 2D có chi phí | Có hậu xử lý nội bộ thuộc planner |
| SmacHybrid | SE(2) lượng tử góc | Dubins tiến, Rmin 0,35 m, không tương đương mô hình pivot |

![P/case_C23_warehouse_cross_aisles_ThetaStar.png|Ví dụ lịch sử C23 trong kho giao cắt với ThetaStar. Dùng để đọc quan hệ Raw và các đường hậu xử lý, không coi là lượt chạy mới.]

## Chương 8 Các phương pháp làm mượt đối chứng

### 8.1 Vì sao cần các đối chứng khác nhau

Raw là đầu ra planner, không phải thuật toán làm mượt. Simple đại diện cập nhật hình học lặp nhẹ; Savitzky–Golay đại diện lọc tín hiệu cục bộ; Constrained đại diện tối ưu phi tuyến có trọng số; PSTMO kết hợp lựa chọn thao tác với dựng đường G². Adaptive Hybrid là chính sách chọn giữa hai ứng viên ở tầng cao hơn. Mỗi phương pháp có hợp đồng khác nhau, nên phải báo số thất bại, thời gian và độ an toàn cùng chất lượng đường. [T2]

### 8.2 Simple Smoother

Một cập nhật điển hình kéo điểm yᵢ về điểm gốc xᵢ đồng thời làm nhỏ sai phân bậc hai của các điểm lân cận. Thành phần fidelity giữ hình dạng ban đầu; thành phần smoothness làm giảm gãy khúc. Tăng smoothness quá mạnh có thể cắt góc gần vật cản nếu kiểm tra khả thi không đủ. Kết quả phụ thuộc điểm đầu, điểm cuối, số vòng lặp, tolerance và xử lý chỗ đổi chiều.

$$ yᵢ ← yᵢ + w_d(xᵢ−yᵢ) + w_s(yᵢ₋₁+yᵢ₊₁−2yᵢ)

YAML đặt tolerance=10⁻¹⁰, max_its=1000, do_refinement=true và refinement_num=2, enforce_path_inversion=false. Không tự gán w_data hay w_smooth từ một phiên bản tài liệu khác khi YAML không ghi; muốn trích giá trị mặc định phải khóa đúng phiên bản thư viện cài. Simple nhanh không có nghĩa có cùng điều kiện G² và giới hạn bánh như PSTMO.

### 8.3 Savitzky Golay

Savitzky–Golay fit đa thức trên một cửa sổ các mẫu theo bình phương tối thiểu rồi lấy giá trị ở tâm. Với bộ lọc 7 điểm thường được dùng trong tài liệu dự án, hệ số là (−2,3,6,7,6,3,−2)/21. Việc lọc x và y giảm nhiễu cục bộ nhưng không tự giải bài toán swept footprint hay thời gian. Các điểm biên phải được xử lý riêng; hệ số chỉ có ý nghĩa cùng quy ước cửa sổ và thứ tự đa thức.

$$ ŷᵢ = [frac:−2yᵢ₋₃+3yᵢ₋₂+6yᵢ₋₁+7yᵢ+6yᵢ₊₁+3yᵢ₊₂−2yᵢ₊₃|21]

Cấu hình dự án bật refinement hai lần và tắt enforce_path_inversion. Bộ lọc trên chỉ số mẫu không tương đương lọc theo khoảng cách vật lý nếu điểm Raw không đều. Vì vậy phải công bố cách lấy mẫu, tránh quy kết mọi giảm độ cong cho ưu thế hình học độc lập với độ phân giải.

### 8.4 Constrained Smoother

Constrained Smoother sử dụng tối ưu Ceres với các thành phần như độ trơn, khoảng cách tới đường gốc, chi phí vật cản và độ cong. Một cách viết tổng quát là tổng các hạng có trọng số; đây là biểu diễn khái niệm, không khẳng định từng residual bằng đúng một công thức đơn giản trong mọi phiên bản. [T3]

$$ J = w_s J_s + w_d J_d + w_c J_cost + w_κ J_κ

Trong cấu hình hiện tại, w_smooth=200000, w_cost=0,015, w_dist=0, w_curve=0 và minimum_turning_radius=0. Do đó không được nói đối chứng này đang ép cứng bán kính cong 0,35 m. Giữ mọi mẫu bằng downsampling_factor=1 tránh việc chuỗi A–B–A thành A–A làm residual độ cong suy biến. keep_start_orientation và keep_goal_orientation đều true; reversing_enabled=false; optimizer tối đa 70 vòng.

### 8.5 Hợp đồng so sánh công bằng

Một nhóm so sánh phải dùng cùng map, start/goal/yaw, cùng Raw và cùng controller. Thời gian planner, thời gian smoother và thời gian robot cần tách riêng. Một phương pháp trả lỗi không được ghi chiều dài hoặc độ cong bằng 0 rồi đưa vào trung bình. So sánh hình học trên các cặp thành công phải đi kèm mẫu số tỷ lệ thành công toàn bộ; nếu không, phương pháp hay thất bại ở ca khó có thể có trung bình đẹp giả tạo.

## Chương 9 PSTMO từ hình học tới tìm kiếm ứng viên

### 9.1 Pipeline hiện tại và trạng thái thuật toán

Plugin PSTMO độc lập hiện dùng condition_only và hierarchical_alpha_two_trim: điều kiện hóa, phát hiện góc, sinh tối đa hai khoảng cắt d, tìm tỷ lệ α=q/d, kiểm tra khả thi, so thời gian trên cửa sổ chung, lựa chọn tổ hợp bằng quy hoạch động, ghép và hậu kiểm. LOS tham lam có trong thư viện nhưng không nằm trong tiền xử lý mặc định này. Adaptive Hybrid giữ nhánh tìm legacy_joint_d_q riêng; không được gộp hai nhánh thành một mô tả chung.

![P/figure_01_pipeline.png|Pipeline PSTMO trong tài liệu nguồn. Sơ đồ thuật toán; các giới hạn và ngoại lệ được giải thích trong chương này.]

### 9.2 Điều kiện hóa đường và loại dao động an toàn

condition_polyline thực hiện giản lược theo sai lệch hình học với predicate an toàn đoạn. Chỉ khi dây cung đạt dung sai và footprint an toàn mới nhận shortcut; nếu không phải chia miền hoặc giữ điểm. Ngưỡng path_conditioning_max_deviation=0 trong YAML kích hoạt giá trị theo resolution_ratio=1,5 ở plugin, tức 0,075 m cho map 0,05 m. Không diễn giải giá trị 0 này là tắt hoàn toàn điều kiện hóa.

$$ δ(i,j) = max d(pₖ,[pᵢ,pⱼ])    với i<k<j,    δ(i,j) ≤ ε

Nhánh khử dao động kiểm tra số lần đổi dấu góc có ý nghĩa, độ dài miền và độ lệch lớn nhất tới dây cung, rồi mới gọi predicate an toàn. Cấu hình span=2,0 m, minimum_turn_angle=0,20 rad, minimum_sign_changes=2 và deviation theo 3×resolution, tức 0,15 m. Đây là điều kiện hóa có giới hạn, không phải xóa mọi đỉnh nằm trong tầm nhìn xa nhất.

![P/figure_03_conditioning_actual.png|Ví dụ điều kiện hóa từ dữ liệu lưu trữ. Các điểm neo giữ lại quyết định bài toán góc ở bước sau.]
![P/figure_08_preprocessing_choice.png|Lý do phân biệt điều kiện hóa với LOS tham lam. Hình giải thích kiến trúc, không phải kết quả đo mới.]

### 9.3 Phát hiện góc và ba trạng thái

Từ ba điểm liên tiếp tính hướng vào u, hướng ra v̂ và góc θ có dấu. Dưới ngưỡng corner_angle_threshold≈5°, plugin có thể giữ pass-through. Với góc đủ lớn, xét transition và pivot. Góc transition bị giới hạn bởi miền dựng đường, mặc định cực đại 170°; quay gần ngược hướng cần xử lý bằng trạng thái khác hoặc bị từ chối nếu không an toàn.

Pass-through giữ một góc nhỏ không đồng nghĩa toàn đường liên tục G² tại mọi vị trí. Tính G² được chứng minh cho mối nối giữa đoạn thẳng và transition Bézier được dựng đúng; pivot là đoạn chuyển hướng dừng, còn đường xuất dạng polyline là lấy mẫu rời rạc. Bài báo cần phát biểu đúng phạm vi này thay vì gọi toàn bộ nav_msgs/Path là một đường liên tục G² ở mọi điểm.

### 9.4 Dựng Bézier bậc năm và chứng minh tại mối nối

Gọi V là đỉnh, d là khoảng cắt bằng nhau trên hai cạnh, A=V−du và B=V+dv̂ là hai điểm nối. Đặt q=αd với α>0. Sáu điểm điều khiển cách đều theo tiếp tuyến ở mỗi đầu. Ba điểm đầu và ba điểm cuối thẳng hàng và cách đều là điều kiện mạnh hơn chỉ cùng tiếp tuyến.

$$ A = V−du,    B = V+dv̂,    q = αd
$$ P₀=A,    P₁=A+qu,    P₂=A+2qu
$$ P₃=B−2qv̂,    P₄=B−qv̂,    P₅=B
$$ B(u) = Σᵢ₌₀⁵ C(5,i)(1−u)⁵⁻ⁱuⁱPᵢ,    0 ≤ u ≤ 1

Đạo hàm thứ nhất tại đầu là 5qu, tại cuối là 5qv̂. Đạo hàm thứ hai tại hai đầu bằng 20 lần sai phân bậc hai của bộ ba điểm điều khiển, do đó bằng 0. Vì q>0 và hướng đơn vị không triệt tiêu, tốc độ tham số ở đầu/cuối khác 0, nên độ cong tại hai đầu bằng 0. Đoạn thẳng kế cận có độ cong 0 và cùng tiếp tuyến, suy ra nối G². Tính C² theo một tham số chung còn đòi hỏi khớp độ lớn đạo hàm của đoạn thẳng, không tự có chỉ từ lập luận G².

$$ B′(0)=5qu,    B′(1)=5qv̂,    B″(0)=B″(1)=0
$$ κ(u) = [frac:B′ₓB″ᵧ−B′ᵧB″ₓ|‖B′‖³],    κ(0)=κ(1)=0

![P/figure_04_bezier_g2.png|Điểm điều khiển Bézier bậc năm và độ cong tại mối nối. Sơ đồ toán học, không phải ảnh camera.]

### 9.5 Ý nghĩa của d alpha và bán kính thiết kế

d xác định phạm vi chiếm chỗ trên hai cạnh; α điều chỉnh độ dài tiếp tuyến điều khiển so với d. Bán kính thiết kế R=d/tan(|θ|/2) chỉ là đại lượng suy từ hình học cắt góc. Đường Bézier không phải cung tròn, do đó 1/R không phải độ cong không đổi và R không đồng nghĩa 1/κmax. Khi thay α, phân bố κ và Eκ thay đổi dù d và θ giữ nguyên.

### 9.6 Hai khoảng cắt được suy từ hình học

Khoảng ưu tiên d_pref là min(dmax,Lin,Lout). Nếu có góc trước cùng dùng đoạn vào, ngân sách đoạn vào giảm còn nửa phần chiều dài sau khi trừ margin; nếu không có góc trước thì được dùng chiều dài vào. Tương tự cho đoạn ra. d_compat là min của d_pref và hai ngân sách. Các khoảng nhỏ hơn dmin hoặc trùng nhau trong độ phân giải khử trùng bị bỏ.

$$ d_pref = min(dₘₐₓ,Lᵢₙ,Lₒᵤₜ),    d_compat = min(d_pref,βᵢₙ,βₒᵤₜ)
$$ βᵢₙ = (Lᵢₙ−m)/2    nếu có góc trước và Lᵢₙ≥m;    βᵢₙ = Lᵢₙ    nếu không có

Trong mã, trường hợp L<m được chặn về 0 trước khi chia hai. Với dmin=0,02 m, dmax=0,8 m, Lin=1,2 m, Lout=1,0 m và có góc hai bên, m=0,05 m cho d_pref=0,8 m và d_compat=0,475 m. Ví dụ này chỉ minh họa phép tính, không khẳng định hai đường cong tương ứng đều an toàn. Margin tự động là max(output_spacing,2×sample_spacing,resolution)=0,05 m ở cấu hình hiện tại.

![P/figure_06_two_trim_dp.png|Hai khoảng cắt và ràng buộc chồng lấn giữa các góc. Quy hoạch động cần lựa chọn tương thích trên cả chuỗi.]

### 9.7 Tìm alpha thô phục hồi và tinh

Với mỗi d, lưới thô là {0,1;0,2;0,3;0,4;0,5}. Nếu không có mẫu khả thi, thử phục hồi {0,15;0,25;0,35;0,45}. Chỉ khi đã có mẫu khả thi mới tinh chỉnh quanh mẫu thắng. Nếu thắng ở một mốc thô, khoảng tinh lấy hai hàng xóm phù hợp; nếu thắng ở điểm phục hồi thì lấy hai mốc thô bao nó. Khoảng được chia thành 10 phần, xét 11 nút và không tính lặp mẫu đã có.

Trong tập khả thi của một d, tiêu chuẩn chọn α là Eκ nhỏ nhất; hòa trong epsilon thì ưu tiên α nhỏ hơn. Sau đó mỗi d chỉ đưa ứng viên α thắng sang so sánh cấp tiếp theo. Điều này giảm số trạng thái, nhưng cũng có nghĩa quy hoạch động phía sau không duyệt mọi α khả thi. Không được tuyên bố tìm tối ưu liên tục toàn cục theo d và q.

![P/figure_05_alpha_search.png|Lưới tìm alpha và điều kiện chuyển sang phục hồi hoặc tinh chỉnh.]

### 9.8 Lấy mẫu và loại ứng viên

Đánh giá B, B′, B″ dùng đa thức Bernstein. Số đoạn khởi tạo là max(16,ceil(2d/sample_spacing)); nếu dây cung dài nhất vượt bước yêu cầu, số đoạn được nhân đôi có giới hạn. Đường có đạo hàm gần 0, giá trị không hữu hạn, đổi dấu độ cong không chủ ý hoặc yêu cầu đảo bánh trong bị loại. Tiếp theo kiểm tra footprint, giới hạn tốc độ và hồ sơ thời gian. Việc đường nhìn trơn không thay thế các cổng loại này.

Chiều dài và Eκ của ứng viên được tích lũy trên các mẫu. Giảm sample_spacing thường tăng độ phân giải kiểm tra nhưng tăng CPU; đây là đánh đổi số học. Độ cong cực đại tính trên mẫu vẫn có thể khác cực đại liên tục giữa mẫu. Nếu muốn một bảo đảm toán học toàn miền, cần thêm chặn đạo hàm, interval arithmetic hoặc phương pháp kiểm chứng đường cong liên tục, chưa được chứng minh ở hiện thực hiện tại.

## Chương 10 Tham số hóa thời gian và lựa chọn toàn đường

### 10.1 Quét tiến lùi theo gia tốc

Từ speed_limit ở mỗi mẫu, thuật toán tạo một dãy tốc độ không vượt cap. Quét tiến chặn khả năng tăng tốc, quét lùi chặn khả năng giảm tốc; hiện thực lặp cặp quét ba lần. start_speed và end_speed là cap biên, không phải luôn là giá trị cưỡng bức bằng cruise. Điều này quan trọng với cửa sổ ngắn không đủ khoảng cách đạt vận tốc lớn.

$$ vᵢ ≤ √(vᵢ₋₁² + 2aₐ꜀꜀Δsᵢ₋₁),    vᵢ₋₁ ≤ √(vᵢ² + 2a_dₑ꜀Δsᵢ₋₁)
$$ Δtᵢ = [frac:2Δsᵢ|vᵢ+vᵢ₊₁],    ωᵢ = vᵢκᵢ

Nếu đoạn có chiều dài dương nhưng tổng hai tốc độ bằng 0, thời gian không hữu hạn và profile bị loại. Mẫu vị trí trùng không được đưa vào profile tịnh tiến; pivot phải tách riêng. Tổng thời gian là tổng Δt, dựa trên xấp xỉ vận tốc tuyến tính theo thời gian trong mỗi đoạn, không phải tích phân động lực học contact của Gazebo.

### 10.2 Ràng buộc gia tốc góc và điều kiện hội tụ

Tính gia tốc góc sai phân |ωᵢ−ωᵢ₋₁|/Δt. Nếu vượt giới hạn, hạ hai cap kề bằng hệ số xấp xỉ 0,995√(αmax/αđo), chặn hệ số tối thiểu 0,1. Tối đa 40 vòng lặp rồi tính lại, từ chối nếu giới hạn vẫn bị vi phạm ngoài tolerance. Cách giảm này dựa trên nhận xét giảm v theo hệ số f thì ω giảm f và thời gian tăng khoảng 1/f, nên gia tốc góc giảm khoảng f².

$$ aωᵢ = [frac:abs(ωᵢ−ωᵢ₋₁)|Δtᵢ₋₁],    f ≈ 0,995√([frac:aωₘₐₓ|aωᵢ])

Đây là phép xây dựng profile khả thi rời rạc, không phải chứng minh thời gian tối thiểu toàn cục. Giới hạn jerk không nằm trong bộ ràng buộc lõi đang dùng; một profile đạt giới hạn gia tốc vẫn có thể có thay đổi gia tốc gắt. Gia tốc góc liên tục còn chứa cả κv̇ và v²dκ/ds, vì vậy G² không tự bảo đảm jerk nhỏ.

$$ ω̇ = κv̇ + v²[frac:dκ|ds]

### 10.3 Cửa sổ chung và trạng thái biên chung

So sánh một đường cong ngắn chỉ trên phần cong với pivot trên một miền dài hơn sẽ thiên lệch. parameterize_transition_window bổ sung đoạn thẳng hai bên để mọi phương án đi từ cùng vị trí vào tới cùng vị trí ra quanh đỉnh. common_trim bằng khoảng cắt lớn nhất của nhóm được xét. Tiếp đó parameterize_common_window tìm điểm cố định của cap vận tốc đầu/cuối chung: bắt đầu từ vận tốc thẳng tối đa, giảm về giá trị nhỏ nhất mà các profile hợp lệ đạt được, lặp tối đa 40 lần.

Sau khi có cap chung, mọi profile được tính lại; nếu vận tốc đầu/cuối thực tế không khớp trong tolerance, ứng viên bị loại. Nhờ đó thời gian so sánh có cùng điều kiện biên. Tuy nhiên sự nhất quán cục bộ không tự tạo một profile thời gian thực thi toàn hành trình được RPP tuân thủ; đó là hai vấn đề khác nhau.

### 10.4 Thời gian pivot với profile tam giác hoặc hình thang

Pivot trên cửa sổ chung gồm tiếp cận d tới đỉnh với vận tốc cuối 0, quay θ từ trạng thái dừng, rồi rời đỉnh tới biên ra. Với quay có tốc độ cực đại ωmax và gia tốc góc αmax, góc ngưỡng ωmax²/αmax phân biệt profile tam giác và hình thang. Với dịch chuyển thẳng, khoảng cách đủ dài cho phép đạt vmax; nếu ngắn, tính tốc độ đỉnh từ gia tốc và giảm tốc.

$$ T_rot = 2√(|θ|/αₘₐₓ)    nếu |θ| ≤ ωₘₐₓ²/αₘₐₓ
$$ T_rot = 2ωₘₐₓ/αₘₐₓ + (|θ|−ωₘₐₓ²/αₘₐₓ)/ωₘₐₓ    nếu vượt ngưỡng
$$ v_peak² = [frac:2aₐ꜀꜀a_dₑ꜀L + a_dₑ꜀v₀² + aₐ꜀꜀v₁²|aₐ꜀꜀+a_dₑ꜀]
$$ T_pivot = T_approach + T_rot + T_departure

minimum_translation_time kiểm tra cả khả năng đạt trạng thái biên trên chiều dài cho trước; nếu không đủ khoảng phanh/tăng tốc thì trả vô hạn. Đây là phép ước lượng kinematic, không tính thời gian settle do định vị hoặc ma sát trượt trong Gazebo. Chỉ nên so nó với thời gian mô hình khác cùng giả thiết, không thay cho thời gian action đo được.

### 10.5 Cổng thời gian và hàm mục tiêu ổn định

Nhánh transition mở khi có thời gian hữu hạn và pivot không an toàn, hoặc khi transition nhanh nhất cộng delta_time_selection=0,15 s vẫn nhỏ hơn thời gian pivot. Nếu pivot an toàn, time_competitive_slack=10 s được rút lại bởi lợi thế thời gian còn dư. Không phải mọi ứng viên trong vòng 10 s đều được nhận bất kể pivot.

$$ T_fast + ΔT < T_pivot,    ΔT = 0,15 s
$$ slack_eff = min(10, max(0,T_pivot−0,15−T_fast))    khi pivot an toàn
$$ Tⱼ ≤ T_fast + slack_eff

Sau cổng thời gian, stable_candidate_cost dùng peak cost chuẩn hóa theo 252, peak angular speed chuẩn hóa theo ωmax và Eκ bão hòa theo Eref=1 m⁻¹. Trọng số lần lượt 0,15;0,10;0,75. Peak cost là đại lượng rủi ro gần vật cản theo costmap, không phải khoảng hở tính bằng mét. Hàm có thang cố định giúp chi phí so được giữa các góc; hàm select_competitive_candidate chuẩn hóa min–max cũng tồn tại trong thư viện nhưng không được nhầm với chi phí ổn định mà pipeline DP hiện tại dùng.

$$ Jⱼ = 0,15 min(1,c_peak/252) + 0,10 min(1,|ω|_peak/ωₘₐₓ) + 0,75[frac:Eκ|Eκ+Eref]

### 10.6 Quy hoạch động và điều kiện tối ưu đúng phạm vi

Mỗi góc có tập trạng thái đã qua cổng: transition với d>0 hoặc pivot an toàn với d=0. Hai trạng thái kề tương thích khi tổng khoảng cắt cộng margin không vượt chiều dài cạnh chung. Quy hoạch động lưu chi phí tốt nhất kết thúc ở mỗi trạng thái và truy vết predecessor. Hòa chi phí ưu tiên ít pivot hơn, rồi thứ tự chỉ số ổn định.

$$ d(zᵢ)+d(zᵢ₊₁)+mᵢ ≤ Lᵢ
$$ Dᵢ(z) = Jᵢ(z) + min Dᵢ₋₁(z′)    trên các z′ tương thích với z

Độ phức tạp điển hình O(NK²) với N góc và tối đa K trạng thái mỗi góc, chưa gồm chi phí sinh ứng viên và footprint. DP tối ưu tổng chi phí trong đồ thị trạng thái đã giữ, không tối ưu trực tiếp thời gian toàn hành trình, không tối ưu mọi đường trong map và không bảo đảm tối ưu liên tục theo α. Chi phí pivot là 1 khi nhánh transition đã mở, nếu không là 0; không phải cộng nguyên thời gian pivot vào J.

### 10.7 Ghép đường bất biến và lỗi có ý nghĩa

build_output_path ghép đoạn thẳng, các mẫu đường cong và marker pivot tại vị trí trùng nhưng yaw đổi. Đầu và đích vị trí được bảo toàn; goal orientation giữ từ đầu vào; yaw đầu theo hướng cạnh đầu. output_spacing=0,05 m cho phần nội suy thẳng, còn đường cong xuất các mẫu của ứng viên. Hậu kiểm kiểm tra tính hữu hạn, cấu trúc hợp lệ và vùng quét theo các chuyển động được biểu diễn.

Nếu không có trạng thái an toàn hoặc hậu kiểm thất bại, PSTMO báo lỗi thay vì lặng lẽ trả đường Raw để giả thành công. Fallback Raw thuộc hợp đồng của SafetyGatedHybrid khi Raw an toàn, không phải hành vi chung mặc định của PSTMO độc lập. Diagnostics ghi mode tiền xử lý, mode tìm, số lần pipeline và lý do loại; các trường này là bằng chứng quan trọng để xác nhận mô tả trong bài báo đúng nhánh đã chạy.

## Chương 11 Adaptive Hybrid và điều khiển vòng kín

### 11.1 Nhánh tìm cũ trong Adaptive Hybrid

SafetyGatedHybridSmoother sinh ứng viên Simple và ứng viên Pivot nội bộ từ cùng đường đầu vào. Nhánh Pivot dùng condition_only và legacy_joint_d_q, không dùng pipeline hai d của PSTMO độc lập. Bộ tìm thích nghi trong adaptive_search.cpp lấy mẫu ban đầu trên miền d, chia khoảng ưu tiên theo biên trạng thái khả thi và biến thiên mục tiêu, dừng theo tolerance hoặc ngân sách. Cấu hình tối đa 20 đánh giá d mỗi góc, initial_search_samples=6 và giữ tối đa 5 ứng viên; không được chuyển các con số này sang mô tả PSTMO hai d.

Ngân sách thời gian được phân cho hai nhánh để giảm thiên lệch thứ tự thực hiện. Nếu một nhánh hết thời gian hoặc không an toàn, chính sách có thể chọn nhánh còn lại. Nếu cả hai không hợp lệ, chỉ được trả Raw khi Raw qua cùng kiểm tra an toàn. Thất bại của toàn bộ ba phương án phải được công bố, không bị che bằng label fallback.

### 11.2 Chính sách so sánh đối xứng

Khi cả hai ứng viên an toàn, chính sách so peak proximity cost: chênh ít nhất 20 ưu tiên bên thấp hơn. Trong deadband đó, xét maneuver effort với deadband tương đối 5% và effort_floor=0,25. Effort cộng độ cong tích lũy phần tịnh tiến với tổng góc quay tại chỗ chia chiều dài đặc trưng pivot=0,2548 m, để pivot không bị xem là chuyển động “miễn phí”.

$$ E_maneuver = Eκ_translation + [frac:Σ abs(Δψ_pivot)|ℓ_pivot]

Nếu chưa phân thắng bại, so chi phí dư rồi chiều dài với tolerance 10⁻⁶ m; hòa toàn bộ thì chọn Simple theo quy tắc ổn định. Tính đối xứng có nghĩa cả Simple lẫn Pivot đều có thể thắng theo cùng tiêu chuẩn, không phải ưu tiên mặc định Pivot. Đây vẫn là heuristic chọn ứng viên, không phải tối ưu đầy đủ điện năng hoặc xác suất va chạm.

### 11.3 Pure Pursuit và cấu hình RPP đang dùng

Pure Pursuit chọn điểm nhìn trước trong hệ robot. Với tọa độ carrot (xc,yc), độ cong cung điều khiển đi tới điểm đó là 2yc/(xc²+yc²); lệnh yaw là vκ. Lookahead lớn làm phản ứng êm hơn nhưng có thể cắt góc và bỏ chi tiết đường; lookahead nhỏ tăng độ nhạy với sai số hướng và nhiễu. RPP bổ sung các cơ chế điều tiết và kiểm tra va chạm, nhưng việc có tính năng trong plugin không có nghĩa tính năng đó đang được bật. [T4]

$$ κ_cmd = [frac:2y꜀|x꜀²+y꜀²],    ω_cmd = v_cmd κ_cmd

Cấu hình chung cho mọi phương pháp hiện đặt desired_linear_vel=0,30 m/s, lookahead cố định 0,50 m và tắt cả velocity-scaled lookahead, curvature speed scaling, cost speed scaling. Vẫn có approach ramp trong 1 m cuối, min_approach_linear_velocity=0,02 m/s, quay hướng đầu ở 0,25 rad/s và ngưỡng 0,10 rad. Vì thế phát biểu “RPP luôn giảm tốc theo độ cong của PSTMO” là sai cho cấu hình này.

![R/screenshots/rviz_warehouse_navigation.png|Ảnh RViz2 của lượt minh họa NavigateToPose trong kho. Không dùng ảnh này để khẳng định action đã gọi PSTMO.]

### 11.4 Chuỗi lệnh vận tốc và các lớp an toàn

Controller chạy 20 Hz, velocity_smoother 50 Hz, feedback=CLOSED_LOOP, odom_duration=0,1 s, timeout lệnh=0,15 s. Các giới hạn thân là v trong ±0,30 m/s, ω trong ±0,80 rad/s, gia tốc dọc 0,35 và giảm tốc 0,45 m/s², gia tốc yaw 1,20 rad/s². scale_velocities=false có thể làm tỷ số v/ω thay đổi khi từng thành phần bị chặn; do đó bán kính thực tế không nhất thiết đúng lệnh lý tưởng trước chặn.

Collision Monitor nhận cmd_vel_smoothed và xuất cmd_vel; FootprintApproach dự báo 1,5 s với bước 0,05 s, min_points=6 và scan sống. RPP cũng có dự báo va chạm lên tới carrot trong giới hạn thời gian cấu hình. Các cơ chế này giảm rủi ro theo cảm biến và mô hình, nhưng chưa phải hệ dừng khẩn cấp phần cứng hoặc chứng nhận an toàn công nghiệp.

### 11.5 Đích logic đích vật lý và kiểm tra tiến triển

SimpleGoalChecker có xy_goal_tolerance=0,06 m, yaw_goal_tolerance=0,10 rad và stateful=true. PoseProgressChecker xét cả dịch chuyển 0,05 m và quay 0,02 rad trong 30 s, tránh coi quay chỉnh hướng cuối là đứng yên. README còn nhắc StoppedGoalChecker là mô tả cũ; cấu hình YAML hiện tại là thẩm quyền cho báo cáo này.

SimpleGoalChecker không bắt buộc vận tốc thực đã về 0. Runner thực nghiệm vì thế chờ dừng và kiểm tra đích từ ground truth độc lập sau action. Dung sai điều khiển 0,06 m không thay thế tiêu chí nghiên cứu 0,10 m trong bộ năm tuyến: hai ngưỡng có mục đích và nguồn tư thế khác nhau. Một action thành công vẫn có thể không đạt tiêu chí nghiên cứu.

## Chương 12 Mô phỏng Gazebo và quan sát RViz2

### 12.1 Mô phỏng vật lý khác với trực quan hóa

Gazebo tích phân trạng thái vật lý, contact, sensor và plugin điều khiển theo bước thời gian. RViz2 hiển thị những bản tin ROS và TF nhận được; nó không chứng minh thân đã tránh va chạm vật lý chỉ vì hình đường không chạm tường. RobotModel trong RViz đọc URDF, trong khi robot Gazebo được spawn bằng SDF. Sự khác nhau giữa hai tệp có thể tạo tình huống “RViz đúng hình nhưng Gazebo sai va chạm”.

![R/screenshots/gz_iso_front.png|Ảnh Gazebo cận cảnh robot trong phiên mô phỏng đã lưu.]
![R/screenshots/rviz_iso.png|Ảnh RViz2 của RobotModel. So với Gazebo để đọc hình học và trục, không đồng nhất hai công cụ.]
![R/screenshots/gz_warehouse_top.png|Góc nhìn từ trên trong Gazebo của kho giao cắt.]
![R/screenshots/rviz_warehouse_overview.png|Bản đồ, costmap và robot trong RViz2 của cùng loại môi trường.]

### 12.2 Clock bridge và ground truth

bridge.yaml nối các luồng clock, joint_states, odom, tf, scan, imu, cmd_vel và ground_truth/odom. Ground truth có frame world/base_link và phải tách khỏi TF chuẩn của định vị; đưa ground truth vào map→odom sẽ làm đánh giá AMCL mất độc lập. Khi sim chậm hơn thời gian thực, thời gian mô phỏng của robot và thời gian CPU của thuật toán phải được báo theo hai đồng hồ khác nhau.

Real-time factor thấp không nhất thiết làm quỹ đạo mô phỏng sai theo sim time, nhưng có thể gây timeout wall-clock, nghẽn recorder và thay đổi lịch callback. Chạy nhiều Gazebo cùng lúc tăng nhiễu đo CPU. Phân tích phải giữ dấu thời gian, số mẫu, tỷ lệ mất dữ liệu và timeout, thay vì chỉ báo một con số thời gian trung bình.

### 12.3 Chuyển môi trường và quản lý trạng thái

environment_manager và switchable_simulation quản lý chuyển giữa bảy môi trường. Khi đổi world, phải xét reset clock, TF cũ, map mới, localization, các action còn hoạt động và lifecycle costmap. Không đổi file map trong khi vẫn giữ toàn bộ trạng thái của world cũ rồi coi đó là lượt thử sạch. Các bài test environment_contract, environment_manager và robot_description kiểm tra một phần hợp đồng này.

Các môi trường hiện có gồm research_warehouse, warehouse_long_aisles, warehouse_cross_aisles, warehouse_dispatch, narrow_aisles, office_maze và open_arena. Báo cáo lý thuyết bao phủ kiến trúc chung nhưng các minh họa nghiên cứu trọng tâm dùng warehouse_cross_aisles. Sự có mặt của bảy map không cho phép suy rộng kết quả một map thành mọi môi trường.

## Chương 13 Chỉ số đánh giá và thiết kế thực nghiệm

### 13.1 Chỉ số hình học và độ cong rời rạc

Chiều dài polyline là tổng độ dài các đoạn. Độ cong ba điểm dùng hai cạnh kề a,b và dây cung c; 2(a×b)/(‖a‖‖b‖‖c‖) là độ cong có dấu của đường tròn qua ba điểm khi không suy biến. compare_paths.py tích lũy bình phương độ cong với trọng số nửa tổng độ dài hai cạnh. Đây là phép ước lượng số, không đồng nhất chính xác với tích phân giải tích trên Bézier.

$$ κᵢ = [frac:2(a×b)|‖a‖‖b‖‖a+b‖],    Eκ ≈ Σ κᵢ² (‖a‖+‖b‖)/2

Khi điểm trùng, mẫu số bằng 0 và phải xử lý theo hợp đồng; không tùy tiện thêm epsilon lớn làm kết quả có vẻ hữu hạn. Khi cần so các đường khác mật độ, dùng cùng resampling spacing. Với quỹ đạo thực thi, jitter định vị được condition trước khi đo độ cong; phải ghi phương pháp và stride vì bộ lọc có thể làm nhỏ dao động thật nếu chọn quá mạnh.

### 13.2 Chỉ số pivot và độ cong tịnh tiến

calculate_maneuver_metrics nhận pivot khi hai vị trí cách nhau không quá 10⁻⁴ m và yaw đổi ít nhất 5°. Nó tách đường thành các đoạn tịnh tiến, resample từng đoạn rồi tính L, κmax và Eκ. Số marker pivot phản ánh cách biểu diễn Path, không tự đếm chính xác số lần robot dừng quay ngoài thực tế. Muốn đo pivot thực thi cần tiêu chí từ v(t), ω(t) và thời gian dwell.

Eκ có đơn vị m⁻¹, không phải joule. Năng lượng điện phải tích phân điện áp nhân dòng; effort của Hybrid cũng không phải điện năng. Không dùng việc Eκ giảm để suy ra trực tiếp pin tiết kiệm một tỷ lệ phần trăm nếu không có mô hình hoặc phép đo điện độc lập.

$$ E_electric = ∫ V(t)I(t) dt,    Eκ = ∫ κ(s)² ds

### 13.3 Sai số bám định vị và sai số đích

Sai số bám là khoảng cách giữa vị trí thực thi và đường tham chiếu, thường dùng điểm tới đoạn. RMSE nhấn mạnh lỗi lớn; max phát hiện đỉnh lỗi; percentile 95 ít nhạy với một mẫu ngoại lai hơn max. Sai số định vị là khác biệt AMCL so ground truth, không phải khác biệt robot so đường. Sai số đích là khoảng cách và yaw ở điều kiện kết thúc; ba loại không được gộp cùng tên accuracy.

$$ RMSE = √([frac:Σ eᵢ²|N]),    e_goal = √[(x_f−x_g)²+(y_f−y_g)²]

RMSE tính đều theo mẫu sẽ đặt trọng số theo thời gian khi tần số cố định, trong khi resample theo chiều dài đặt trọng số theo quãng đường. Hai kết quả có thể khác khi robot dừng lâu. Khi căn chỉnh odom với ground truth đầu lượt, phải ghi phép biến đổi cố định và cách nội suy thời gian; không tối ưu căn chỉnh bằng toàn bộ trajectory rồi gọi là sai số odometry tuyệt đối chưa hiệu chỉnh.

### 13.4 Khoảng hở và các tiêu chuẩn va chạm

clearance_metrics.py lấy mẫu footprint và poses rồi dùng bản đồ khoảng cách để ước lượng khoảng hở tới ô vật cản. Báo min, percentile 5 và số mẫu bất hợp lệ giúp hiểu đuôi phân bố. Khoảng hở tâm, khoảng hở thân, costmap proximity cost và contact vật lý Gazebo là bốn đại lượng khác nhau. Chúng không thể thay thế nhau dù đều liên quan “an toàn”.

Một hậu kiểm footprint không ghi nhận mẫu đè vật cản không chứng minh không có contact giữa các mẫu hay va chạm với chi tiết 3D không xuất hiện ở PGM. Nếu muốn tuyên bố không va chạm vật lý, cần log contact hoặc một cơ chế quan sát tương đương được kiểm chứng. Nếu muốn tuyên bố an toàn với người, cần đánh giá ngoài phạm vi một bài mô phỏng path smoothing.

### 13.5 Chỉ số vận tốc gia tốc và jerk

velocity_metrics.py suy vận tốc bánh từ lệnh Twist, dùng sai phân thời gian cho gia tốc và jerk, tính peak và p95. Phải gắn nhãn đây là chỉ số lệnh hay đo từ odometry. Lệnh trước velocity_smoother, lệnh sau Collision Monitor và vận tốc thân ground truth có thể khác nhau. Sai phân khuếch đại nhiễu timestamp; không tính qua khoảng thiếu dữ liệu dài như thể là chuỗi lấy mẫu đều.

$$ aᵢ ≈ [frac:vᵢ−vᵢ₋₁|tᵢ−tᵢ₋₁],    jᵢ ≈ [frac:aᵢ−aᵢ₋₁|tᵢ−tᵢ₋₁],    aᵧ = |vω|

closed_loop_metrics.py còn phân vùng cong và đoạn thoát cua bằng độ cong đường tham chiếu, mặc định threshold=0,40 m⁻¹ và cửa sổ hình học 0,10 m. Các chỉ số này nhạy với định nghĩa đoạn cong và phép chiếu tiến độ; khi viết bài phải nêu ngưỡng thay vì chỉ đưa tên “curve exit error”.

### 13.6 Đơn vị thử ghép cặp và thất bại

Đơn vị thử là một cấu hình start/goal/yaw, map, planner và phương pháp, kèm seed và lần lặp nếu có. Năm planner tạo năm đầu vào khác nhau, không phải năm lần lặp độc lập của cùng thuật toán. Năm route cũng không thay thế lặp nhiều seed trên một route. Một lần mỗi ô không đủ để suy ra phương sai ngẫu nhiên ổn định hoặc p-value đáng tin cho ưu thế tổng quát.

Tỷ lệ thành công có mẫu số là mọi lượt được định nghĩa trước. Khi so chỉ số liên tục, báo rõ dùng toàn bộ thành công riêng từng phương pháp hay tập ghép cặp cùng thành công. Phân biệt hạ tầng không khởi động được với robot thực thi nhưng không đạt mục tiêu; chỉ lỗi hạ tầng mới được retry theo chính sách đã định, giữ log cả lần thử.

### 13.7 Hồ sơ năm tuyến trong kho giao cắt

Bộ đã lưu gồm 5 tuyến × 5 planner × 5 phương án = 125 lượt, một lượt mỗi ô. R01 là dữ liệu lịch sử, R02–R05 là 100 lượt bổ sung. README của hồ sơ ghi 123/125 đạt; hai lượt không đạt là R04/ThetaStar/Raw với sai số đích 0,100234 m và R03/SmacHybrid/PSTMO với 0,102385 m, vượt ngưỡng 0,100000 m. Các số được trích như kết quả lưu trữ, không phải kiểm nghiệm lại trong lần viết này.

![W/overview_5_routes.png|Năm tuyến kho giao cắt trong hồ sơ 125 lượt. Các tuyến là năm tình huống, không phải năm seed của một tình huống.]
![W/R01_route_overview.png|Tuyến R01 giữ bài toán gốc của báo cáo PSTMO.]
![W/R02_route_overview.png|Tuyến R02 chuyển chéo dài, tạo chuỗi quyết định qua các giao cắt.]
![W/R03_route_overview.png|Tuyến R03 dạng chữ U phía Nam, dùng để đọc ảnh hưởng của nhiều góc liên tiếp.]
![W/R04_route_overview.png|Tuyến R04 chuyển chéo ngược, kiểm tra hướng tiếp cận khác trong cùng bản đồ.]
![W/R05_route_overview.png|Tuyến R05 từ hành lang ngang vào lối lấy hàng, nhấn mạnh trạng thái đích và tiếp cận.]

Không nhập 21 lượt động học của báo cáo 3D vào mẫu số 125 lượt smoothing. Cũng không cộng các kiểm tra hình học preflight thành lượt robot di chuyển. Phân lớp bộ dữ liệu là điều kiện cần để tái lập và tránh số mẫu bị thổi phồng.

## Chương 14 Cách đọc dữ liệu và đối chiếu một ca cụ thể

### 14.1 Từ yêu cầu đường đi tới hồ sơ bằng chứng

Trước khi nhìn biểu đồ, cần xác định scenario, start/goal đầy đủ yaw, tên planner, phương pháp, hash Raw, hash cấu hình và loại clock. Sau đó đọc action status, lý do thất bại và tiêu chí hậu kiểm. Chỉ khi đầu vào và điều kiện kết thúc đã rõ mới diễn giải L, κmax, Eκ, thời gian CPU hoặc tracking error. Màu đẹp và đường ít rung không đủ chứng minh chất lượng.

Ở một ca PSTMO, đọc diagnostics theo thứ tự: preprocessing_mode, candidate_search_mode, pipeline_execution_count, conditioning, số góc, d_pref/d_compat, số α đã đánh giá, các lý do loại, thời gian cửa sổ chung, trạng thái được DP chọn và final_invariants_verified. Các trường không tồn tại trong phiên bản JSON cũ phải đánh dấu thiếu, không suy từ phiên bản mới rồi điền vào lịch sử.

### 14.2 Ba góc nhìn của cùng một tình huống

Hình học kế hoạch trả lời đường đầu ra thay đổi thế nào; quỹ đạo thực thi trả lời thân robot thật trong mô phỏng đã đi đâu; đồ thị động học trả lời lệnh hoặc vận tốc biến đổi ra sao. Cần đọc cả ba để tránh nhầm một đường có Eκ thấp với một robot có tracking tốt. Hình dưới lấy từ cùng nhóm R02/ThetaStar của hồ sơ cũ, phục vụ minh họa cách đối chiếu.

![W/R02_ThetaStar_01_geometry.png|Hình học R02 và ThetaStar trong hồ sơ năm tuyến, dùng để đọc Raw và các phương án hậu xử lý.]
![W/R02_ThetaStar_02_execution.png|Đường thực thi tương ứng. Độ lệch so đường kế hoạch chịu ảnh hưởng của controller, định vị và contact.]
![W/R02_ThetaStar_03_dynamics.png|Đồ thị động học của cùng nhóm. Cần đọc nhãn luồng đo để phân biệt lệnh và vận tốc thực thi.]

### 14.3 Quy trình đối chiếu khi một kết quả khác báo cáo

Nếu hình khác tài liệu, trước hết kiểm tra revision và hash, không vội kết luận thuật toán sai. Tiếp theo kiểm tra tên plugin có thực sự được nạp, bản src có được build vào install, world và PGM có đúng cặp, initial pose và yaw có giống, Raw có cùng SHA-256 hay không. Sau đó kiểm tra sampling, cửa sổ thời gian, tiêu chí dừng và xử lý thất bại. Chỉ khi các điều kiện này khớp mới truy ngược tới khác biệt số học hoặc logic.

Lỗi thường gặp là so curvature của Raw chưa resample với smoother đã resample; lấy thời gian mô phỏng so wall time; tính localization error trước khi đăng ký map/world; coi mọi code có mặt trong repository là code đang chạy; hoặc suy tính năng từ YAML không nằm trong danh sách plugin được kích hoạt. Báo cáo này ưu tiên truy vết những điểm đó hơn việc gắn một tên thuật toán cho mọi kết quả.

## Chương 15 Từ mô phỏng tới phần cứng và giới hạn kết luận

### 15.1 Động cơ encoder và các trường chưa xác nhận

real_robot_profile.yaml ghi động cơ GA25, nguồn danh định 12 V, tỷ số truyền 45:1, tốc độ không tải 130 rpm và định mức 100 rpm. Với r=0,0425 m, công thức v=2πrn/60 cho khoảng 0,5786 và 0,4451 m/s. Đây là tốc độ lý tưởng từ thông số, không phải tốc độ robot dưới tải. Giới hạn điều hướng 0,30 m/s là lựa chọn cấu hình riêng.

$$ v_ideal = [frac:2πrn|60],    Δs = [frac:2πrΔN|N_rev]

Độ phân giải encoder, cách giải mã, driver và bộ điều khiển phần cứng còn chưa được xác nhận đầy đủ trong profile. Không được tự điền pulses per revolution từ một sản phẩm GA25 khác có cùng tên. N_rev phải là số count hiệu dụng trên một vòng bánh sau khi tính vị trí encoder, tỷ số truyền và chế độ giải mã. Nếu nhầm encoder trước/sau hộp số, odometry có thể sai một hệ số lớn dù chiều quay đúng.

![R/figures/cad_drive_a.png|Cụm truyền động thể hiện trong CAD. Tên và hình học bộ phận không thay bằng chứng thiết bị thật đã lắp và hiệu chuẩn.]
![R/figures/cad_electronics.png|Bố trí điện tử trong CAD. Không suy từ nhãn CAD rằng driver ROS hoặc kết nối điện đã hoạt động.]

### 15.2 Nguồn điện tải trọng và điều kiện thử thật

Profile nêu pin 4S4P gồm 16 cell 2.600 mAh; với giả thiết 3,7 V danh định/cell, điện áp danh định 14,8 V, đầy 16,8 V, dung lượng 10,4 Ah và năng lượng danh định 153,92 Wh. Đây là phép cộng nối tiếp/song song lý tưởng, không xác nhận năng lượng sử dụng, giới hạn dòng toàn hệ hay độ an toàn điện. Báo cáo không thay tài liệu cell, BMS, cầu chì, bộ hạ áp và kiểm định dây dẫn.

Thêm tải làm đổi tâm khối và quán tính, tăng lực kéo cần thiết và thay quãng phanh. Việc hình chiếu COM nằm trong đa giác đỡ chỉ là điều kiện tĩnh lý tưởng; động lực học quay, dốc và lực quán tính có thể làm giảm ổn định. Không có tải trọng định mức được chứng minh chỉ từ SDF 5 kg hoặc hình CAD. Trước thử thật cần quy trình riêng về giới hạn vận tốc, vùng thử, dừng khẩn cấp và đo contact/điện; báo cáo này không cấp chứng nhận an toàn.

![R/figures/support_payload.png|Minh họa tâm khối và vùng đỡ từ hồ sơ mô hình. Đây là cơ sở phân tích, không phải kết luận tải trọng cho phép.]

### 15.3 Những điều có thể và chưa thể khẳng định

Có thể khẳng định cấu trúc công thức Bézier đáp ứng tiếp tuyến và độ cong 0 tại hai đầu dưới các giả thiết đã nêu; hiện thực có các cổng khả thi, DP và hậu kiểm; các tệp mô hình có những giá trị được liệt kê; các bộ dữ liệu lưu trữ cho phép đối chiếu nhiều tình huống trong mô phỏng. Có thể dùng chúng làm căn cứ giải thích thiết kế và tái lập các phép đánh giá theo đúng revision.

Chưa thể khẳng định tối ưu liên tục toàn cục của PSTMO, bảo đảm collision-free liên tục trong thế giới thật, tối ưu điện năng, an toàn công nghiệp, hiệu năng với vật cản động hoặc tải/sàn bất kỳ. Chưa thể đồng nhất STEP với revision đã lắp thực. Chưa có chứng minh RPP thực thi đúng profile thời gian nội bộ PSTMO. Các giới hạn này cần được nêu cạnh kết luận liên quan, không chỉ giấu ở cuối bài.

### 15.4 Chương trình kiểm chứng tiếp theo

Một chương trình nghiên cứu tiếp theo nên tách kiểm chứng toán học, kiểm thử triển khai, mô phỏng độ nhạy và thử phần cứng. Kiểm chứng toán học kiểm tra mối nối, miền không đảo bánh và chặn sai số lấy mẫu. Kiểm thử triển khai thêm các góc suy biến, unknown/out-of-map, đường quá ngắn, đầu vào trùng, hết thời gian và xung đột góc kề. Mô phỏng độ nhạy thay resolution, lookahead, tốc độ, tải, nhiễu và seed, nhưng mỗi lần phải lưu cấu hình và giữ giao thức ghép cặp.

Phần cứng cần xác nhận kích thước bánh/vệt bánh, encoder, timestamp, latency, giới hạn vận tốc đo được và quãng dừng trước khi đánh giá hiệu năng điều hướng. Khi đã có nhiều lần lặp độc lập, báo phân bố và khoảng bất định thay vì chỉ trung bình. Các phép thử đề xuất ở đây chưa được chạy trong lần biên soạn tài liệu này.

## Phụ lục A Từ điển ký hiệu và đơn vị

| Ký hiệu | Ý nghĩa | Đơn vị hoặc miền |
| x y ψ | Tư thế robot phẳng | m, m, rad |
| r | Bán kính bánh khai báo | m |
| b | Khoảng cách hai bánh trong mô hình đang xét | m; phải nêu hình học hay hiệu dụng |
| v ω | Vận tốc tịnh tiến và góc thân | m/s, rad/s |
| vL vR | Vận tốc tiếp tuyến bánh | m/s |
| ΩL ΩR | Vận tốc góc bánh | rad/s |
| κ | Độ cong tịnh tiến | m⁻¹; không xác định khi pivot |
| Eκ | Tích phân bình phương độ cong | m⁻¹ |
| θ | Góc rẽ có dấu tại đỉnh | rad |
| d q | Khoảng cắt và độ dài điều khiển Bézier | m |
| α | Tỷ số q/d | Không thứ nguyên; khác gia tốc góc |
| m | Margin giữa hai vùng cắt trong công thức DP | m; không phải ký hiệu khối lượng ở công thức này |
| c_peak | Chi phí proximity cao nhất | Giá trị costmap, không phải mét |
| aᵧ | Gia tốc ngang | m/s² |
| J | Chi phí cục bộ chuẩn hóa | Không thứ nguyên |
| T | Thời gian ước lượng hoặc đo | s; phải nêu clock và cách đo |
| F S | Footprint và vùng quét footprint | Tập điểm trong không gian |

## Phụ lục B Bảng đối chiếu lý thuyết với mã nguồn

@TRACE

## Phụ lục C Thông số cấu hình nguyên bản để tra cứu

Phụ lục này giữ các dòng cấu hình có giá trị của nav2_params.yaml, bridge.yaml và real_robot_profile.yaml, bỏ dòng trống và chú thích để dễ tra. Số dòng ở cột đầu trỏ tới bản nguồn khóa trong manifest. Đây là bản cấu hình, không phải bản dump tham số của node đang chạy. Các khối như docking hoặc route server chỉ chứng minh đã có cấu hình; việc sử dụng còn phụ thuộc launch, plugin, graph và phần cứng liên quan. Không dùng phụ lục này để suy rằng mọi tính năng đều đã kiểm nghiệm.

@CONFIG

## Phụ lục D Truy vết phiên bản kiểm thử và nguồn tham khảo

@MANIFEST

### D1 Kiểm thử và cách đọc phạm vi bảo đảm

Các bài test lõi bao phủ dựng quintic, conditioning, line of sight, tìm thích nghi, tìm alpha, time parameterization, candidate selection, DP và hybrid selection. Các test Nav2 kiểm tra footprint; benchmark kiểm tra metrics, localization và initial heading; RViz kiểm tra catalog; Gazebo kiểm tra URDF và hợp đồng môi trường. Unit test đạt chỉ chứng minh các trường hợp đã định trong test, không thay mọi kiểm chứng vật lý.

Số test 308 từng xuất hiện trong báo cáo lịch sử là kết quả của một môi trường và thời điểm cụ thể, không được coi là số test vừa chạy của lần biên soạn này. Tài liệu mới khóa danh mục test hiện có nhưng không thực hiện lại build hay mô phỏng. Trước tái lập nên build workspace, chạy colcon test, đọc colcon test-result và lưu cả lỗi/bỏ qua; khi chỉ sửa tài liệu không nên tự sửa thuật toán để làm kết quả đẹp hơn.

@TESTS

### D2 Nguồn trực tiếp của dự án

[S1] src/adaptive_pivot_g2/src và include: hình học, tìm kiếm, thời gian, lựa chọn và quy hoạch động. Đây là nguồn chính cho chương 9–10.

[S2] src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp, safety_gated_hybrid_smoother.cpp và footprint_safety.cpp: cách plugin thực sự nối các mô-đun và xử lý lỗi.

[S3] src/vacuum_robot_gazebo/config, urdf, models, launch, maps và worlds: cấu hình và mô hình đang được dự án cung cấp. Các giá trị runtime cần kiểm tra thêm bằng dump khi chạy.

[S4] src/adaptive_pivot_g2_benchmark/adaptive_pivot_g2_benchmark: định nghĩa chỉ số, hợp đồng Path, runner và ma trận thực thi.

[S5] docs/PSTMO.docx và docs/PSTMO.pdf: tài liệu phương pháp và hình minh họa lịch sử. Tài liệu mới giữ cách trình bày học thuật, mở rộng nền tảng và ưu tiên mã nguồn hiện tại khi có khác biệt.

[S6] docs/warehouse_cross_aisles_5_routes/README.md, all_125_trials.csv và audit_manifest.json: giao thức, kết quả và truy vết bộ năm tuyến.

[S7] docs/robot_3d_report/README.md, geometry.json, cad_audit.json, mass_properties.json, motion và warehouse_demo.json: phân tích hình học và phép thử mô hình. Hình ảnh trong báo cáo có đường dẫn gốc lưu trong figure_manifest.json cạnh mã biên soạn.

### D3 Tài liệu chính thức để tra cứu nền tảng

[T1] Navigation2, AMCL, tài liệu nhánh Jazzy. https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/ . Truy cập 05/10/2026. Dùng cho khái niệm bộ định vị và ý nghĩa giao diện tham số; giá trị dự án lấy từ YAML.

[T2] Navigation2, Navigation Plugins, nhánh Jazzy. https://docs.nav2.org/jazzy/configuration_and_development/navigation_plugins/ . Truy cập 05/10/2026. Dùng để phân loại planner, smoother và các server; không suy ngược tính năng được bật.

[T3] Navigation2, Constrained Smoother, nhánh Jazzy. https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/smoother_plugins/constrained_smoother/configuring_constrained_smoother/ . Truy cập 05/10/2026. Đối chiếu vai trò các trọng số tối ưu và ràng buộc độ cong.

[T4] Navigation2, Regulated Pure Pursuit, nhánh Jazzy. https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/ . Truy cập 05/10/2026. Đối chiếu lookahead, điều tiết và quay hướng; không dùng tên tham số Rolling thay cho Jazzy.

### D4 Kết luận sử dụng tài liệu

Để dùng báo cáo làm căn cứ đối chiếu, trước hết chọn đúng chương theo thành phần, đọc giả thiết của công thức, sau đó tìm tệp và giá trị tại phụ lục. Khi sửa mô hình hoặc thuật toán, tạo revision mới của báo cáo thay vì tiếp tục gọi các thông số cũ là hiện tại. Các hình lịch sử luôn giữ vai trò minh họa có nguồn; kết luận về phiên bản mới chỉ được cập nhật sau khi có dữ liệu tương ứng.
