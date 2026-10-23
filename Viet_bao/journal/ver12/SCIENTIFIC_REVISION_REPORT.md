# Báo cáo hiệu đính khoa học PSTMO ver12

Ngày hoàn tất kiểm tra: 24/10/2026. Phạm vi: major revision về lập luận và trình bày, không thay đổi phương pháp đã triển khai hoặc kết quả thí nghiệm.

## Kết luận hiệu đính

Ver12 đặt trọng tâm vào bài toán lựa chọn **chuỗi** thao tác Bézier–quay tại chỗ trên đường đầu vào, với khả thi theo mô hình và tương thích chiều dài cắt ở cạnh chung. Điểm cần làm nổi bật là không gian quyết định và cách phối hợp các quyết định, không phải phát minh riêng Bézier G², DP hoặc tối ưu thời gian. Bằng chứng thực nghiệm tiếp tục thuộc toàn bộ PSTMO; chưa chứng minh riêng tác dụng DP hay pivot.

Phần III của ver11 đã mô tả thận trọng phần lớn các giới hạn. Không tìm thấy lý do để viết lại hoặc thay số liệu phần này. Ver12 giữ nguyên toàn bộ III, chỉ đánh lại số trích dẫn. II giữ toàn bộ nội dung và công thức, thêm hai đoạn giải thích ở II-D/E. Introduction được tổ chức lại sâu; Tóm tắt và Kết luận được đồng bộ theo lập luận mới.

## A Vấn đề phát hiện và phân loại

### Research positioning

Introduction ver11 dành nhiều dung lượng cho các nhánh không trực tiếp kiểm chứng trong bài: lấy hàng, PRM/RRT, phương tiện mặt nước, đường cao tốc, tối ưu trong môi trường động. Các đoạn đó không nhất thiết sai, nhưng phân tán câu hỏi về phối hợp chuyển hướng. Đối chiếu gần với Piazzi, Brezak và Blažič còn ngắn so với mức độ liên quan. Ver12 tập trung vào các khác biệt có thể truy vết: đầu vào, biến quyết định, mục tiêu, quan hệ giữa các đoạn và tầng tích hợp.

### Novelty clarification

Ver11 đã tránh tuyên bố “đầu tiên DP + Bézier”; cần giữ điểm đúng này. Blažič 2023 không chỉ ghép góc độc lập: hướng và độ cong tại các điểm trung gian có thể được tối ưu cho cả đường. Piazzi 2002 đã có nội suy bậc năm G². Brezak 2011 đã xét robot vi sai, không gian và khả thi. Vì vậy, ver12 không dùng sự thiếu hụt giả định của các công trình này để tạo tính mới. Khác biệt của PSTMO được phát biểu tương đối với các phương pháp đã đối chiếu, không như một bằng chứng ưu tiên tuyệt đối trên toàn bộ văn liệu.

### Mathematical ambiguity

Ba cấp mục tiêu dễ bị đọc lẫn: Eκ để tìm hình dạng cho từng chiều dài cắt; thời gian dự đoán để sàng lọc; J để tối thiểu hóa tổng chi phí của chuỗi hữu hạn. Ver11 đã nêu phần lớn nội dung nhưng chưa gom chúng thành một lời giải thích trực tiếp. Trạng thái DP không mang biến vận tốc toàn đường; không thể suy tối ưu thời gian thực thi từ truy hồi (7). Không phát hiện yêu cầu sửa chín công thức đã có.

### Implementation consistency

Trong C++, J được tính trong quá trình đánh giá ứng viên, trước khi hoàn thành chọn tập cạnh tranh. “Lọc trước khi so sánh chi phí” trong ver11 mô tả vai trò quyết định, không có nghĩa chỉ sau bộ lọc mới tính J. Ver12 bổ sung điểm này, không đổi thứ tự xử lý. Đỉnh đi qua dưới 5° có trạng thái cố định, chi phí và phần cắt bằng không; cờ nội bộ phục vụ lựa chọn không đồng nghĩa robot thực hiện pivot tại đỉnh đó.

### Experimental interpretation

Phân biệt hình học tham chiếu, quỹ đạo thực thi, sai số bám, sai số cuối, khoảng hở và tỷ lệ đạt đã có trong III. Giữ cả các tuyến PSTMO có RMSE cao hơn, các trường hợp không nhanh hơn Simple, chi phí hậu xử lý tăng và các lượt thất bại. Cấu hình Constrained đã thử có trọng số độ cong và khoảng cách tới đường gốc bằng không; không được coi nó là hiệu năng tối ưu của thuật toán. Điểm này được đưa lên Introduction và Conclusion, không thay cấu hình hoặc bảng.

### Statistical interpretation

Giữ 25 nhóm tuyến–planner và 118 cặp cùng đạt; không coi 250 lượt là 250 quan sát độc lập. Khóa `independent_block_count` trong aggregate là nhãn triển khai, không phải bằng chứng các nhóm độc lập hoàn toàn. Ver11 đã ghi rõ các nhóm chia sẻ tuyến, planner và bản đồ. Wilcoxon không trực tiếp kiểm định trung bình Δ; Holm chỉ hiệu chỉnh ba p-value chính, không hiệu chỉnh toàn bộ CI. McNemar p = 1 không chứng minh tương đương. Không sửa các đoạn đúng này và không chạy lại các phép kiểm định.

### Writing and presentation

Word nguồn thực tế có hash khác provenance dựng ver11 và không còn Abstract tiếng Anh. Dựng lại từ nguồn Markdown cũ sẽ có nguy cơ khôi phục phần tác giả đã bỏ. Ver12 dùng package Word hiện tại làm nguồn thiết kế và đối tượng khoa học, chỉ thay `word/document.xml`. Theo xác nhận của tác giả, giữ Tóm tắt/từ khóa tiếng Việt và tên tiếng Anh, không có Abstract/Keywords tiếng Anh.

## B Thay đổi quan trọng theo trước và sau

### 1 Mở đầu và I-A

- Trước: mở từ tổ chức logistics, tổng quan lấy hàng và Amazon Picking Challenge; tiếp theo liệt kê nhiều họ tìm kiếm.
- Sau: mở bằng việc hai góc cùng chiếm dụng một cạnh ngắn; I-A giới hạn vào tầng hậu xử lý và các họ planner liên quan khảo sát.
- Lý do: đưa vấn đề nghiên cứu lên trước phần nền; không suy năng suất kho từ chuyển động đơn robot.
- Bằng chứng: pipeline II-A, ràng buộc (7), phạm vi dữ liệu III-A; R001/R002/R005/R006 và các nguồn planner được giữ.
- Ảnh hưởng: thay tổ chức lập luận, không thay phát biểu thuật toán hoặc dữ liệu.

### 2 I-B đối chiếu G² và clothoid

- Trước: Piazzi được dẫn trong một câu về spline G²; Brezak trong một câu về clothoid cho robot vi sai.
- Sau: nêu điều kiện biên và mô hình của Piazzi; nêu chuyển tiếp clothoid, giới hạn phần cắt/độ lệch, cập nhật đường và kiểm chứng robot của Brezak. Phân biệt với cấu trúc zero-curvature Bézier–đoạn thẳng và lựa chọn chuỗi hữu hạn của PSTMO.
- Lý do: làm rõ những gì đã có và những khác biệt thực chất; không lấy bậc đa thức hoặc G² làm tuyên bố mới.
- Bằng chứng: R031 toàn văn 10 trang, đặc biệt trang PDF 2–4; R043 toàn văn 6 trang. Xem đối chiếu ở phần tiếp theo.
- Ảnh hưởng: tăng độ chính xác định vị; không bổ sung bảo đảm toán học mới cho PSTMO.

### 3 I-C đối chiếu Blažič 2023

- Trước: đã nêu Bézier bậc ba, hồ sơ vận tốc và mục tiêu độ cong của ví dụ, nhưng chưa giải thích rõ phối hợp toàn đường hay không gian quyết định.
- Sau: hai đoạn riêng về chuỗi điểm nối, hướng/độ cong, điểm dừng, hành lang và gia tốc; phân biệt bước hình học ở Mục 4.1 với bước vận tốc ở Mục 3; so sánh trực tiếp với tập Bézier/pivot, cạnh chung và Nav2.
- Lý do: đây là công trình gần; không mô tả sai thành chỉ làm trơn từng góc, bỏ kích thước xe hoặc trực tiếp tối ưu thời gian trong ví dụ hình học.
- Bằng chứng: R010, Mục 2.1–2.2, 3, 4–4.1, 5; đọc toàn bộ 6 trang và xem ảnh các trang PDF 2–5.
- Ảnh hưởng: thay chiều sâu so sánh; không tuyên bố PSTMO tốt hơn nghiên cứu đó vì không có benchmark đối đầu.

### 4 I-C/D/E và phạm vi đóng góp

- Trước: DP–Bézier đã được phân biệt với Choi; đóng góp phương pháp và thực nghiệm đã tách, nhưng nằm sau phần tổng quan rộng.
- Sau: nêu rõ trạng thái DP của Choi là điểm trên miền hành lang trước làm trơn, còn PSTMO chọn thao tác tại góc trên đường đã có. Khoảng trống được giới hạn bằng phát biểu cụ thể, không tuyên bố chưa từng có phối hợp nguyên thủy. Đưa cấu hình Constrained vào cách diễn giải so sánh ngay từ phần đầu.
- Lý do: phân biệt đối tượng tối ưu và tầng phần mềm; không đánh đồng hiệu quả của toàn bộ PSTMO với hiệu quả nhân quả của DP.
- Bằng chứng: [Choi 2014, Mục 2/2.2 và Summary](https://journals.sagepub.com/doi/10.5772/58683); [Nav2 Jazzy Constrained Smoother](https://api.nav2.org/nav2-jazzy/html/md_nav2_constrained_smoother_README.html); mã/config cohort và Bảng IV.
- Ảnh hưởng: làm rõ nhận định, không mở rộng mức bằng chứng.

### 5 Hai đoạn bổ sung ở II-D/E

- Trước II-D: “Điều kiện thời gian và khoảng cho phép lọc tập phương án trước khi so sánh chi phí…” đã đúng về vai trò bộ lọc.
- Sau: giữ câu cũ, thêm phân biệt Eκ/thời gian/J và thời điểm tính J trong triển khai.
- Trước II-E: có truy hồi, tập tương thích, tie-break và độ phức tạp; chưa giải thích trực tiếp vì sao một trạng thái cuối đủ để lưu tiền tố tốt nhất, hoặc trạng thái đi qua dưới ngưỡng.
- Sau: thêm lời giải thích tính cộng/quan hệ cạnh kề, giới hạn biến trạng thái và trường hợp đi qua.
- Bằng chứng: `candidate_selection.cpp:137–167`, `adaptive_pivot_g2_smoother.cpp:895–978,1274–1282,1467–1590`, `path_optimization.cpp:38–194`; đầu ra đi qua trong `build_output_path`.
- Ảnh hưởng: giải thích triển khai, không thay J, bộ lọc, truy hồi, pivot cost hoặc tham số.

### 6 Tóm tắt và Kết luận

- Trước: Tóm tắt chưa nhấn rõ lấy mẫu footprint hoặc đánh đổi sai số vị trí cuối; Kết luận chưa liệt kê riêng kiểm chứng pivot và lặp với Simple trong hướng tiếp theo.
- Sau: đồng bộ vấn đề cạnh chung; nêu lấy mẫu, lợi ích có điều kiện so với cấu hình đã thử và cả ba hướng bất lợi; hướng tiếp theo phân biệt DP, pivot và Simple.
- Bằng chứng: III-H/I, Bảng XXI–XXIII và aggregate đã lưu. Không thêm số liệu vào Tóm tắt ngoài các số đã có.
- Ảnh hưởng: tăng độ cân bằng và phạm vi kết luận, không thay kết quả.

### 7 Danh mục tham khảo và pipeline

- Trước: 80 mục, bao phủ toàn bộ 74 mục corpus và 6 nguồn bổ sung; Word thực tế khác bản dựng cũ.
- Sau: 54 mục có vị trí dẫn thực tế, 26 mục chỉ ở Introduction bị bỏ có lý do riêng; giữ mọi nguồn II/III. Danh mục đánh số lại theo lần xuất hiện đầu tiên.
- Bằng chứng: [REFERENCE_COVERAGE.md](REFERENCE_COVERAGE.md), [reference_decisions.json](reference_decisions.json), [citation_map.json](citation_map.json). Mỗi mục giữ có câu dẫn cụ thể, trang/phạm vi nguồn và hash nếu là nguồn cục bộ.
- Ảnh hưởng: tổ chức học thuật và tái lập artifact; không xóa bất kỳ tệp gốc nào trong REFERENCES.

## C Đối chiếu các công trình trọng tâm

### Piazzi 2002

Nguồn cục bộ: `REFERENCES/031_2002_Piazzi_Quintic_G2_Splines.pdf`, trang in 27–36. Mục II/III phát biểu nội suy vị trí, tiếp tuyến và độ cong hai đầu, xây dựng họ η-spline bậc năm với bốn tham số. Tính tối thiểu bậc năm áp dụng cho dữ liệu biên tùy ý; không chuyển kết quả đó thành chứng minh bậc năm là cần thiết cho trường hợp zero-curvature hạn chế của PSTMO. Mô hình xe có góc lái và vận tốc được giả thiết hằng trong lập luận điều khiển; không đồng nhất với robot vi sai quay tại chỗ. Bản thảo chỉ kế thừa tiền lệ G², không nhận là phát minh mới và không tuyên bố G² kéo theo trơn gia tốc/jerk thời gian.

### Brezak và Petrović 2011

Nguồn cục bộ: `REFERENCES/043_2011_Brezak_Clothoid_Path_Smoothing.pdf`, trang in 1133–1138. Công trình xuất phát từ đường gấp khúc, dùng clothoid có độ cong biến thiên tuyến tính theo chiều dài cung; phần đường tĩnh dùng cặp chuyển tiếp đối xứng. Giới hạn phần cắt và độ lệch điều chỉnh mức nắn đường trong không gian hạn chế. Phần cập nhật đường xét trạng thái có độ cong khác không và các tổ hợp clothoid. Có kiểm tra va chạm/khả thi và thử nghiệm robot vi sai; cũng bàn thao tác dừng/đổi hướng khi cần. Không được viết rằng phương pháp này bỏ qua khả thi hoặc không bao giờ cho phép dừng quay. PSTMO khác ở không gian lựa chọn đã giới hạn của bộ hậu xử lý và quan hệ phần cắt giữa các góc, không ở việc đầu tiên làm trơn cho robot vi sai.

### Blažič và cộng sự 2023

Nguồn cục bộ: `REFERENCES/010_2023_Blazic_Warehouse_Low_Order_Bezier_Minimum_Time.pdf`, trang in 11815–11820. Toàn văn đã đọc; các công thức/hình trọng tâm được xem trực quan. Bảng dưới ghi sự khác nhau của bài toán, không phải xếp hạng hiệu năng.

| Khía cạnh | Blažič 2023 | PSTMO đã triển khai |
|---|---|---|
| Đầu vào | Hành lang và các điểm cần đi qua/dừng; điểm trung gian tại vùng đổi hướng | Đường gấp khúc từ planner, rồi tiền xử lý |
| Biểu diễn | Nguyên thủy Bézier bậc ba xác định bằng vị trí, hướng, độ cong biên; có thể tách nguyên thủy khi cần | Bézier bậc năm nối với cạnh thẳng, hai chiều dài cắt và tìm α phân cấp; thêm quay tại chỗ |
| Phối hợp chuỗi | Hướng và độ cong tại điểm trung gian được điều chỉnh để tối ưu tiêu chí của đường | DP chọn trong tập hữu hạn, tương thích chiều dài cắt trên cạnh chung |
| Không gian | Hành lang cho trước; vùng cấm được nới trong ví dụ để biểu diễn AGV như điểm | Costmap, hình bao có hướng, kiểm tra lấy mẫu; không bảo đảm va chạm liên tục |
| Vận tốc/gia tốc | Hồ sơ vận tốc trên đường, giới hạn tốc độ và gia tốc kết hợp tiếp tuyến–hướng tâm dạng elip; điểm dừng có vận tốc bằng không | Giới hạn thân/bánh, gia tốc dự đoán; so sánh trên cửa sổ chung, không tối ưu biến vận tốc trong DP |
| Mục tiêu | Bước vận tốc hướng thời gian nhỏ nhất trên đường đã có; ví dụ hình dạng Mục 4.1 dùng tích phân κ² và phạt ra ngoài hành lang | Eκ chọn hình dạng, thời gian sàng lọc, tổng J chọn chuỗi; không cực tiểu trực tiếp thời gian thực thi |
| Bằng chứng | Ví dụ mô phỏng đường và hồ sơ chuyển động trong kho | Đường tham chiếu và thực thi Nav2/RPP trong Gazebo, hai bộ dữ liệu riêng |

Không kết luận Blažič thiếu xử lý toàn chuỗi hoặc bỏ kích thước AGV. Mô tả phương pháp trong toàn văn không đưa ra bài toán DP chọn Bézier/pivot với cạnh chung như PSTMO; đây là đối chiếu phát biểu cụ thể, không phải kết luận mọi biến thể của phương pháp đều không thể mở rộng theo hướng đó. Không so sánh chéo các con số thời gian giữa hai bài vì thiết kế khác nhau.

### Choi 2014

Đã đối chiếu các phần liên quan trên [trang nhà xuất bản](https://journals.sagepub.com/doi/10.5772/58683), gồm phần mô tả và định nghĩa các giai đoạn DP ở Mục 2.2. Không ghi là đã tải/đọc toàn bộ PDF. Quyết định DP ở đây chọn các điểm rời rạc tạo đường gấp khúc trước bước làm trơn, khác trạng thái thao tác ở PSTMO. “Pivot point” trong phần phân chia hành lang là điểm hình học, không phải bằng chứng thao tác quay tại chỗ của robot. Nội dung này đủ để bác bỏ tuyên bố “đầu tiên kết hợp DP và Bézier”, không dùng để khẳng định thiếu mọi cơ chế điều phối khác.

### Nav2 Constrained Smoother

Đối chiếu [README Jazzy](https://api.nav2.org/nav2-jazzy/html/md_nav2_constrained_smoother_README.html) và [mã hàm chi phí Jazzy](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_constrained_smoother/include/nav2_constrained_smoother/smoother_cost_function.hpp). Các residual về độ trơn, độ cong, lệch đường gốc và costmap không tạo cùng bài toán với bộ giới hạn vận tốc/thời gian PSTMO. Ràng buộc được thực hiện bằng các thành phần phạt có cấu hình; không suy bảo đảm khả thi cứng từ tên plugin. Cấu hình cohort được kiểm tra riêng bằng hash, không lấy giá trị mặc định trên tài liệu để thay cho cấu hình thực nghiệm. Nghiên cứu chưa có tinh chỉnh công bằng với cùng ngân sách, nên không xác lập ưu thế thuật toán tuyệt đối.

## D Những điểm cố ý không sửa

- Mã ROS/C++, plugin và cấu hình: chỉ đọc, đối chiếu commit/hash; không có thay đổi runtime.
- Công thức (1)–(9): giữ nguyên XML OMML; không đổi định nghĩa, trọng số hoặc điều kiện thời gian/DP.
- Tất cả 23 bảng, 696 ô và 18 hình: giữ nguyên XML, media và relationships. Không vẽ lại kết quả hoặc thêm hình dễ bị hiểu nhầm là thực nghiệm DP.
- II-A/B/C: giữ nguyên sau khi đối chiếu. Tiền xử lý vẫn có thể đổi hình học; G² không đồng nghĩa liên tục theo thời gian; footprint vẫn là kiểm tra mẫu.
- III: giữ nguyên cả thuận lợi, bất lợi, thất bại, chính sách retry và giới hạn. Số citation là thay đổi cơ học duy nhất.
- Thống kê: chỉ đọc aggregate và so định dạng số/đơn vị trong Bảng XXI–XXII; không chạy bootstrap, Wilcoxon, Holm hay McNemar.
- Tên bài, lề, cột, styles, hình học trang: giữ từ Word thực tế; không giảm font để ép số trang.

Không thêm hình hai góc: truy hồi và hai đoạn giải thích đã làm rõ quan hệ cạnh chung, trong khi yêu cầu ưu tiên bảo toàn 18 hình. Có thể bổ sung minh họa lý thuyết riêng ở vòng sửa sau nếu tác giả thấy cần; không dùng hình đó thay ablation.

## E Các hạn chế chưa thể giải quyết bằng biên tập

1. Chưa có DP so với lựa chọn cục bộ trên cùng tập ứng viên và cùng phiên bản.
2. Chưa tách vai trò của lựa chọn pivot–Bézier hoặc các thành phần tiền xử lý/tìm hình dạng.
3. Chưa có bộ kiểm chứng lặp với Simple, dù ma trận khảo sát có Simple.
4. Chưa có nhiều môi trường độc lập, vật cản động hoặc khảo sát nhiễu/seed rộng.
5. Chưa kiểm chứng robot thật; hai khoảng cách bánh trong mô hình và mô phỏng vẫn phải được phân biệt.
6. Chưa đo chi phí phục hồi thất bại để suy thời gian chu kỳ vận hành có retry.
7. Chưa chứng minh an toàn liên tục giữa mẫu hoặc đo đầy đủ khoảng hở thực thi.
8. Chưa xác lập bộ tham số tối ưu hoặc so đối chứng với cùng ngân sách tuning.
9. Phân tích cặp cùng đạt còn thiên lệch chọn mẫu; các nhóm dùng chung bản đồ/tuyến/planner, không thể suy rộng như mẫu môi trường độc lập.
10. Chưa đo điện năng hoặc rung tải. Eκ và jerk không thay các phép đo đó.

Những hạn chế này được giữ hoặc nhấn rõ trong bài; không đánh dấu là đã giải quyết. Việc chỉnh sửa làm lập luận chính xác hơn, không tự tạo đủ bằng chứng để bảo đảm bài được chấp nhận đăng.

## F Kiểm chứng và hồ sơ

Nguồn có thẩm quyền là Word ver11 SHA-256 `debafc4f83a9b7a4dfd52230908cb37fac42ccfb5c23bebea45be996ce556fbc`, không phải hash dựng cũ. [baseline.json](baseline.json) giữ hash tất cả tệp ver11, nguồn tham khảo và các đầu vào được bảo vệ. [provenance.json](provenance.json) lưu nguồn dựng và hash đầu ra; [evidence_audit.json](evidence_audit.json) lưu kiểm tra cohort/commit, số liệu và phạm vi kiểm tra lịch sử.

Builder so sánh trực tiếp XML bảng, ô, hình, OMML, section và khối tên bài; các phần ZIP ngoài `word/document.xml` nguyên byte. Toàn bộ văn bản II/III được đối chiếu sau quy hồi citation về khóa, chỉ cho phép đúng hai đoạn thêm. Citation có thứ tự xuất hiện liên tục 1–54 và không có nguồn giữ mà không dẫn.

Bản Word được render bằng Documents runtime/LibreOffice và kiểm tra từng trang; xem [visual_review.json](visual_review.json) và [quality_checks.json](quality_checks.json) cho số trang, hash và ghi chú từng trang. Chỉ kiểm tra bằng renderer này, không tuyên bố đã thử mọi phiên bản Microsoft Word. Ảnh/PDF render là artifact QA nội bộ, không phải một bản thảo độc lập.

Phạm vi đọc nguồn còn lại được kế thừa từ hồ sơ ver11 và ghi rõ trong `references.json`; không tuyên bố đã đọc lại toàn văn cả 54 tài liệu. Các nguồn trọng tâm nêu trên được đọc sâu cho lần sửa này. Kiểm tra hash không được dùng thay cho kiểm tra nội dung.
