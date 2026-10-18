## G. Tổng hợp so sánh năm phương án

### 1) Kết quả chung và điều kiện ghép cặp

Ma trận khảo sát có 123/125 lượt hoàn thành. Bảng {table:pooled} cho thấy PSTMO có thời gian, RMSE và tích phân bình phương độ cong tham chiếu trung bình thấp nhất. Tuy nhiên, thời gian và RMSE dùng tập lượt đạt riêng của mỗi phương án, nên hiệu giữa hai trung bình chưa phải chênh lệch trên cùng một tập đầu vào.

@table pooled Tổng hợp mô tả ma trận 125 lượt

@note T và e lấy trên 24 lượt đạt của đường gốc/PSTMO và 25 lượt đạt của mỗi phương án còn lại. Eκ,ref dùng toàn bộ 25 đường mỗi phương án. Năm bộ lập kế hoạch là năm đầu vào khác nhau, không phải năm lần lặp.

Bảng {table:all_pairs} ghép PSTMO với từng đối chứng cùng tuyến và bộ lập kế hoạch, chỉ giữ cặp cùng hoàn thành. Có 23 cặp với đường gốc và 24 cặp với mỗi phương án làm mượt. PSTMO giảm cả ba chỉ số trung bình trên các cặp này; mức giảm thời gian nhỏ nhất khi so với Simple.

@table all_pairs Chênh lệch PSTMO so với bốn đối chứng trên các cặp cùng hoàn thành

@note n là số cặp; Δ = PSTMO − đối chứng. Mỗi cặp dùng cùng đường gốc. Các chênh lệch là thống kê mô tả của một lượt cho mỗi cấu hình, không kèm kiểm định ý nghĩa thống kê.

PSTMO nhanh hơn đường gốc ở 21/23 cặp, Simple ở 14/24, Savitzky–Golay ở 23/24 và Constrained ở 21/24. Số cặp có RMSE thấp hơn tương ứng là 12/23, 14/24, 15/24 và 17/24. Như vậy, lợi ích thời gian xuất hiện thường xuyên hơn lợi ích bám đường, đặc biệt khi so với Savitzky–Golay. Không có cơ sở để kết luận PSTMO giảm đồng thời mọi chỉ số trên mọi đầu vào.

@figure matrix_delta So sánh PSTMO với bốn đối chứng. Chấm là chênh lệch trung bình; đoạn thẳng là khoảng từ nhỏ nhất đến lớn nhất, không phải khoảng tin cậy. Hai hàng đầu dùng các cặp cùng đạt; hàng cuối dùng đủ 25 cặp đường tham chiếu. Số phía trên mỗi cột là số cặp có chỉ số giảm trên tổng số cặp tương ứng.

Hình {figure:matrix_delta} cho thấy chênh lệch thời gian và RMSE có thể đổi dấu giữa các đầu vào dù trung bình âm. Phân tích theo tuyến tại Mục III-B–F xác định những trường hợp này. Mức giảm độ cong tham chiếu còn cần được xem xét theo mật độ điểm.

### 2) Chiều dài và độ cong của đường tham chiếu

Các bảng theo tuyến cho thấy PSTMO đồng thời thay đổi chiều dài và độ cong đường tham chiếu. Vì vậy, thời gian thực thi thấp hơn không thể được quy hoàn toàn cho một yếu tố hình học. Bảng {table:translation} kiểm tra thêm độ cong sau khi thống nhất bước lấy mẫu của phần tịnh tiến.

@table translation Hình học phần tịnh tiến sau lấy mẫu lại và dấu quay tại chỗ

@note κt,max và Eκ,t là trung bình trên 25 đường sau khi tách phần tịnh tiến và lấy mẫu lại với bước 0,05 m. N là số điểm đầu ra trung bình trước lấy mẫu lại; “Quay” là tổng số dấu quay tại chỗ trong 25 đường. Đây không phải số lần xoay hướng do bộ điều khiển thực hiện.

Bảng {table:translation} cho thấy số điểm đầu ra của PSTMO lớn hơn các đối chứng. Sau khi thống nhất bước lấy mẫu phần tịnh tiến, Eκ,t của PSTMO vẫn thấp hơn ở mức trung bình. Phép so sánh này không khẳng định các đường gấp khúc có độ cong liên tục theo nghĩa giải tích.

Hai dấu quay tại chỗ được ghi trong 25 đường PSTMO, với tổng góc khoảng 1,608 rad. Các đối chứng không chứa dấu theo tiêu chí đã định; tuy nhiên, RPP vẫn có thể yêu cầu xoay hướng. Số dấu không thay thế số lần quay quan sát được khi thực thi.

Sai lệch vị trí đầu–cuối do hậu xử lý không vượt 2,8 × 10⁻¹⁷ m, ở mức sai số số học. Kết quả chỉ kiểm tra vị trí của đường, không thay cho kiểm tra hướng, định vị hoặc sai số robot dừng tại đích.

### 3) Quỹ đạo thực thi và độ chính xác cuối

@table execution_extra Chỉ số chuyển động và sai số cuối của năm phương án

@note Trung bình trên các lượt đạt: 24 lượt cho đường gốc/PSTMO, 25 lượt cho mỗi phương án còn lại. j95 là phân vị 95% của trị tuyệt đối độ giật tịnh tiến trong từng lượt; bảng lấy trung bình các phân vị này. ep và eψ là sai số vị trí và hướng cuối. Eκ,exec được tính trên quỹ đạo đã xử lý như Mục III-A.

PSTMO có Eκ,exec trung bình thấp nhất trong Bảng {table:execution_extra}. Trên các cặp cùng hoàn thành, chỉ số này giảm ở 23/23 cặp với đường gốc, 21/24 với Simple và toàn bộ 24 cặp với mỗi đối chứng Savitzky–Golay và Constrained. Mức cải thiện hình học thực thi nhất quán hơn mức giảm RMSE trong ma trận khảo sát.

Quãng đường thực thi trung bình là 8,510; 8,507; 8,545; 8,555 và 8,539 m theo cùng thứ tự năm phương án. Giá trị gộp của PSTMO lớn hơn Simple, nhưng chênh lệch ghép cặp là −0,040 m trên 24 cặp cùng đạt. Sự khác nhau xuất phát từ việc Simple có thêm một lượt hoàn thành mà PSTMO không đạt. Đây là lý do cần báo mẫu số và dùng đầu vào tương ứng khi đánh giá chênh lệch.

PSTMO có sai số bám đường cực đại trung bình thấp hơn bốn đối chứng, nhưng sai số cuối không giảm theo cùng thứ tự. Độ chính xác dừng còn phụ thuộc cách tiếp cận đích, ngưỡng bộ kiểm tra đích và quá trình giảm tốc sau khi tác vụ hoàn tất.

@figure execution_trace Minh họa R02/Theta* cho đủ năm phương án. Trên: đường tham chiếu nét đứt và quỹ đạo thực thi nét liền tại vùng rẽ; giữa: vận tốc góc của đường gốc, Simple và SG; dưới: Constrained và PSTMO. Hai đồ thị vận tốc dùng cùng thang đo, tín hiệu không lọc và thời gian riêng tính từ đầu từng lượt. Đây là một trường hợp minh họa, không phải kết quả trung bình toàn ma trận.

Hình {figure:execution_trace} đối chiếu hình dạng đường với đáp ứng góc mà không căn chỉnh các đỉnh tín hiệu. Do RPP tắt điều tiết vận tốc theo độ cong và chi phí, không thể dùng hai chức năng này để giải thích mức giảm thời gian. Các phương án cũng chịu cùng tần số điều khiển và giới hạn vận tốc.

### 4) Biên độ chuyển động, thời lượng xử lý và an toàn

@table dynamics Cực đại quan sát được của các đại lượng chuyển động

@note Mỗi ô là giá trị lớn nhất trong toàn bộ 25 lượt, bao gồm không đạt. vb là vận tốc bánh tương đương suy từ vận tốc thân xe với khoảng cách bánh 0,2548 m; a⊥ = vω; a và α lần lượt là gia tốc tịnh tiến và gia tốc góc. Đây là đại lượng ước lượng tại các mẫu đã ghi, không phải phép đo trực tiếp vận tốc từng bánh.

Bảng {table:dynamics} ghi nhận vận tốc góc cực đại khoảng 0,249 rad/s cho cả năm phương án, thấp hơn giới hạn 0,80 rad/s trong mô hình PSTMO. Vận tốc bánh tương đương không vượt khoảng 0,316 m/s và gia tốc ngang không vượt 0,063 m/s²; các giới hạn mô hình tương ứng là 0,36 m/s và 0,18 m/s². Vận tốc tịnh tiến cực đại xấp xỉ 0,30 m/s; phần vượt nhỏ hơn 10⁻⁷ m/s ở dữ liệu lưu được xem là sai số số học của mô phỏng.

Gia tốc tịnh tiến cực đại của PSTMO là 0,302 m/s², cao hơn bốn đối chứng trong ma trận dù vẫn thấp hơn giới hạn tăng tốc 0,35 m/s². Cực đại gia tốc góc là 0,732 rad/s², thấp hơn giới hạn 1,20 rad/s² nhưng cao hơn Constrained. Các kết quả này mô tả đáp ứng của toàn hệ thống dưới cùng bộ điều khiển; chúng chưa tách được đóng góp riêng của các ràng buộc nội bộ PSTMO hoặc bảo đảm rằng mọi trạng thái giữa hai mẫu đều thỏa giới hạn.

@table safety Thời lượng hậu xử lý và các chỉ số giám sát trong ma trận khảo sát

@note τ là trung bình trên 25 lượt; đường gốc không gọi hậu xử lý. “Dừng” là số lượt đáp ứng điều kiện dừng vật lý; “Can thiệp” là tổng số lần bộ giám sát va chạm can thiệp; “Mẫu va chạm” là tổng mẫu đường bao giao cắt vật cản trên đường tham chiếu. Các số đếm bao gồm cả lượt không đạt.

Bảng {table:safety} cho thấy PSTMO có thời lượng hậu xử lý cao nhất. Giá trị bằng không của Savitzky–Golay được ghi theo đồng hồ mô phỏng, không chứng minh chi phí tính toán bằng không. Các phép đo này chưa phải thời gian CPU hoặc độ trễ trên phần cứng độc lập.

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

@figure repeat_blocks Chênh lệch trung bình của 25 nhóm trong bộ kiểm chứng lặp. Trong mỗi tuyến, thứ tự là A*, Dijkstra, Theta*, Smac 2D và Smac Hybrid. Mỗi điểm dùng các lần lặp cùng đạt; đường đứt là trung bình đều của 25 điểm. Hình thể hiện mức thay đổi giữa các đầu vào, không biểu diễn 125 quan sát độc lập.

Hình {figure:repeat_blocks} cho thấy RMSE tăng ở nhiều nhóm hơn hai chỉ số còn lại. PSTMO có lợi ích trung bình nhưng không cải thiện đồng thời mọi đầu vào. Bảng {table:secondary} bổ sung quãng đường, độ giật, sai số đích, khoảng hở và thời lượng hậu xử lý để đánh giá những mặt bất lợi cùng với ba chỉ số chính.

@table secondary Các chỉ số bổ sung trong bộ kiểm chứng lặp

@note Các khoảng tin cậy là phân tích bổ sung; hiệu chỉnh Holm chỉ áp dụng cho ba chỉ số trong Bảng {table:primary}. Dấu âm thuận lợi cho quãng đường, độ giật, tích phân bình phương độ cong và thời lượng xử lý; đối với khoảng hở, dấu âm biểu thị robot được dẫn gần vật cản hơn.

Khoảng hở giảm trung bình 35,161 mm, sai số vị trí cuối tăng 3,380 mm và thời lượng hậu xử lý tăng 71,220 ms. Vì vậy, lợi ích thời gian và bám đường cần được cân nhắc cùng yêu cầu khoảng hở và dung sai đích. Mức giảm độ giật 0,078 m/s³ chưa xác định độ rung tải hàng hay điện năng tiêu thụ, do nghiên cứu chưa đo các đại lượng đó.

Trong 125 cặp, bốn cặp chỉ Constrained đạt và ba cặp chỉ PSTMO đạt. Kiểm định McNemar chính xác hai phía cho p = 1,0. Dữ liệu chưa cho thấy khác biệt tỷ lệ thành công; kết quả cũng không chứng minh tương đương hoặc không kém hơn. Năm lần lặp ở mỗi cấu hình còn ít để đánh giá các thất bại hiếm. Các lượt không đạt được giữ đầy đủ trong Mục III-I.

## I. Phân tích các trường hợp không đạt và giới hạn nghiên cứu

### 1) Các lượt vượt dung sai vị trí

@table failures Các lượt không đạt trong hai bộ dữ liệu

@note KS là ma trận khảo sát 125 lượt; số 1–5 là lần lặp trong bộ 250 lượt. G: đường gốc; P: PSTMO; C: Constrained. Hai bộ được báo riêng, không cộng mẫu số để tính một tỷ lệ thành công chung. “Kết thúc trước đích” mô tả trạng thái quan sát được, chưa xác định nguyên nhân và không đồng nghĩa đã dừng vật lý.

Trong ma trận khảo sát, PSTMO/Smac Hybrid tại R03 có sai số vị trí cuối 0,102385 m; đường gốc/Theta* tại R04 có sai số 0,100234 m. Cả hai tác vụ đều hoàn tất và robot đã dừng, nhưng vượt ngưỡng đánh giá 0,10 m. Giữ nguyên phân loại này tránh việc điều chỉnh dung sai sau khi biết kết quả. Những lượt này được tính vào tỷ lệ thất bại và không được dùng như thời gian hoàn thành hợp lệ khi so sánh ghép cặp.

Bộ lặp ghi thêm hai trường hợp R04 vượt ngưỡng: PSTMO/Smac Hybrid ở lần 4 đạt sai số 0,101583 m; Constrained/Theta* ở lần 4 đạt 0,100358 m. Phần vượt lần lượt khoảng 1,6 mm và 0,4 mm, nhỏ hơn nhiều so với sai số của các lượt kết thúc xa đích. Các con số phản ánh trạng thái mô phỏng, không biểu thị độ chính xác đo trên robot thực ở mức dưới milimét.

### 2) Các lượt kết thúc trước đích

Năm thất bại còn lại tập trung tại R05 trong bộ lặp, gồm ba lượt PSTMO và hai lượt Constrained. Sai số vị trí còn lại từ 0,686 đến 3,655 m. Các lượt này kết thúc trước khi hoàn thành hành trình, khác về bản chất với những trường hợp chỉ vượt nhẹ dung sai. Bảng {table:failures} giữ rõ tuyến, bộ lập kế hoạch, phương án và lần chạy để đối chiếu dữ liệu.

@figure endpoints Sai số vị trí cuối của toàn bộ 250 lượt kiểm chứng lặp. Trục đứng dùng thang log để hiển thị đồng thời sai số cỡ centimét và các lượt kết thúc xa đích. Ngưỡng 0,10 m chỉ là một phần của tiêu chí hoàn thành; các lượt thuộc ma trận 125 lượt không nằm trong hình này.

Hình {figure:endpoints} cho thấy phần lớn sai số nằm dưới ngưỡng, nhưng một số lượt R05 dừng phép thử khi còn xa đích. Thời gian trung bình trên các cặp cùng đạt chưa bao gồm chi phí phục hồi những thất bại này. Vì chưa có một chính sách phục hồi chung được đo, không thể quy đổi kết quả thành thời gian chu kỳ vận hành thực tế có tính đến thử lại.

Dữ liệu hiện tại xác định được trạng thái kết thúc, chưa chứng minh nguyên nhân cụ thể của từng thất bại. Việc quy lỗi cho hình dạng đường, bộ điều khiển hoặc định vị cần đối chiếu đồng bộ nhật ký và tín hiệu ở thời điểm sự kiện. Các thất bại xa đích xuất hiện ở cả PSTMO và Constrained; số lượng hiện có chưa đủ để gán ưu thế về độ tin cậy cho một phương án.

### 3) Phạm vi kết luận

Nghiên cứu đánh giá năm dạng tuyến trên một bản đồ tĩnh, một robot mô phỏng và một cấu hình RPP. R01 của ma trận khảo sát được thu ở đợt khác R02–R05; dữ liệu một lượt chỉ hỗ trợ so sánh mô tả. Bộ lặp dùng cấu hình cố định nhưng chỉ có PSTMO và Constrained. Thứ tự chạy chưa được xác nhận ngẫu nhiên hóa; bốn tiến trình có miền truyền thông riêng vẫn dùng chung tài nguyên máy tính. Các giới hạn này đặc biệt ảnh hưởng cách diễn giải thời lượng xử lý và khả năng suy rộng.

Constrained được đánh giá với trọng số độ cong bằng không. Các phương án chưa được tinh chỉnh theo cùng ngân sách đánh giá tham số trên tập hiệu chỉnh tách biệt, nên kết quả đại diện cho các cấu hình đã báo. Phân tích cặp cùng đạt còn có thể thiên lệch về các nhiệm vụ dễ hoàn thành; tỷ lệ thành công và từng trường hợp không đạt phải được đọc cùng các chỉ số liên tục.

PSTMO cho thấy lợi ích về hình học và một số chỉ số thực thi trong phạm vi khảo sát, đồng thời có đánh đổi về khoảng hở, sai số cuối và thời lượng xử lý. Kiểm tra robot thực, nhiều bố trí bản đồ và thí nghiệm loại bỏ từng thành phần sẽ giúp xác định phạm vi áp dụng và đóng góp riêng của thuật toán. Dữ liệu hiện có chưa hỗ trợ kết luận về điện năng, rung tải hàng hoặc ưu thế tỷ lệ thành công.
