# ĐỀ CƯƠNG SƠ BỘ GỬI GIẢNG VIÊN HƯỚNG DẪN

## Tên đề tài dự kiến

**Adaptive Hybrid Pivot–G2: thuật toán hậu xử lý và làm mượt đường đi cho robot vi sai trong ROS 2/Nav2**

Tên tiếng Anh dự kiến: **Adaptive Hybrid Pivot–G2 Path Smoothing for Differential-Drive Robots in ROS 2/Nav2**

> **Phạm vi hiện tại:** Chỉ trình bày thuật toán smoother và đánh giá hình học đường đi; các phần thực thi robot không nằm trong phạm vi abstract này. Các tham số và luật chọn nhánh vẫn đang được hiệu chỉnh, vì vậy số liệu hiện có chỉ dùng để thiết kế thí nghiệm, chưa coi là kết quả cuối.

## 1. Đoạn nhắn ngắn có thể gửi cô

Thưa cô, em dự kiến nghiên cứu một thuật toán hậu xử lý đường đi cho robot vi sai trong ROS 2/Nav2. Đường do global planner sinh ra thường là một polyline gồm nhiều đoạn thẳng nối nhau tại các góc gấp. Nếu giữ nguyên, robot có thể phải quay nhiều; nếu làm tròn tất cả các góc, đường mới có thể cắt gần hoặc đi vào vật cản. Vì vậy, tại mỗi góc em dự kiến tạo hai loại trạng thái: giữ góc và đánh dấu thao tác quay tại chỗ (Pivot), hoặc thay góc bằng một đoạn Bézier bậc năm liên tục hình học G² khi còn đủ không gian.

Với mỗi góc, thuật toán tìm kiếm thích nghi giá trị cắt góc/bán kính, sinh nhiều ứng viên và loại các ứng viên không an toàn bằng kiểm tra toàn footprint trên costmap. Các ứng viên còn lại được ghép bằng quy hoạch động trên toàn đường để tránh hai đoạn cong kề nhau chồng lấn và chọn chuỗi có chi phí hình học thấp. Sau đó một cổng Hybrid dự kiến so sánh nhánh Pivot–G² với một smoother nền; nếu các đường làm mượt không bảo đảm an toàn thì hệ thống trả về đường Raw.

Em dự kiến thử nghiệm trên 7 môi trường mô phỏng Gazebo/Nav2, tổng cộng 60 cặp điểm đầu–đích, 5 global planner, 8 biến thể đường/phương pháp và 3 lần lặp. Như vậy có 900 nhóm cùng đầu vào Raw và 7.200 bản ghi đường hình học để so sánh. Các điểm đánh giá gồm tỷ lệ smoother tạo được đường, va chạm footprint, khoảng hở tới vật cản, chiều dài đường, độ lệch khỏi Raw, độ cong cực đại, tích phân bình phương độ cong, số trạng thái Pivot/G² và thời gian xử lý. Kết quả kỳ vọng là giảm góc gấp và dao động độ cong nhưng vẫn giữ an toàn và khả năng trả về đường gốc khi làm mượt không phù hợp. Hiện em chưa chốt phần trăm cải thiện vì một số bước lựa chọn và xử lý ứng viên vẫn cần sửa và chạy lại.

## 2. Bản tóm tắt sơ bộ để cô phát triển thành abstract

Đường đi do các bộ lập kế hoạch toàn cục sinh ra thường có dạng polyline với các góc gấp, gây bất lợi cho chuyển động của robot vi sai. Các phương pháp làm mượt thuần túy có thể giảm độ gấp khúc nhưng đồng thời làm đường lệch khỏi hành lang an toàn hoặc cắt gần vật cản. Nghiên cứu này đề xuất Adaptive Hybrid Pivot–G2, một thuật toán hậu xử lý đường đi tích hợp trong ROS 2/Nav2. Tại mỗi điểm đổi hướng, thuật toán xem xét trạng thái Pivot, giữ nguyên đỉnh góc để robot có thể đổi hướng tại chỗ, và trạng thái G², thay góc bằng đoạn chuyển tiếp Bézier bậc năm liên tục hình học với hai đoạn thẳng lân cận. Miền tham số cắt góc được xác định từ hình học cục bộ và được lấy mẫu thích nghi. Mỗi ứng viên được kiểm tra độ hợp lệ hình học và vùng quét footprint trên costmap. Các ứng viên an toàn sau đó được ghép bằng quy hoạch động để tránh chồng lấn giữa các góc kề và giảm một hàm chi phí gồm độ cong, chiều dài, độ lệch đường và phạt Pivot. Một cổng Hybrid so sánh các nhánh làm mượt và sử dụng đường Raw làm phương án dự phòng khi không có ứng viên an toàn. Kế hoạch đánh giá gồm 7 môi trường mô phỏng, 60 tình huống điểm đầu–đích, 5 global planner, 8 phương pháp và 3 lần lặp, tạo 7.200 bản ghi đường hình học trong 900 nhóm đầu vào ghép cặp. Các đặc tính được đánh giá gồm tỷ lệ thành công, an toàn footprint, khoảng hở, chiều dài, độ lệch, độ cong và thời gian xử lý. Nghiên cứu kỳ vọng xác định được sự đánh đổi giữa độ trơn, mức thay đổi đường và độ an toàn; kết quả định lượng sẽ được cập nhật sau khi thuật toán được ổn định và ma trận thử nghiệm được chạy lại.

**Từ khóa dự kiến:** robot vi sai; làm mượt đường đi; Bézier bậc năm; liên tục G²; quay tại chỗ; Nav2; Gazebo.

## 3. Kiến thức nền tảng và giải thích ký hiệu

### 3.1 Đường đi polyline và góc đổi hướng

Đường Raw từ global planner được xem là một dãy điểm:

`P = {p₀, p₁, ..., pₙ}, với pᵢ = [xᵢ, yᵢ]ᵀ`

Trong đó:

- `P` là toàn bộ đường đi dạng polyline;
- `pᵢ` là điểm thứ `i` trên đường;
- `xᵢ, yᵢ` là tọa độ của điểm `pᵢ` trong hệ tọa độ map, đơn vị mét;
- `n + 1` là tổng số điểm của đường.

Vector chỉ phương và chiều dài đoạn thứ `i` được tính bởi:

`eᵢ = pᵢ₊₁ − pᵢ`

`lᵢ = ||eᵢ||₂`

Trong đó `eᵢ` là vector từ `pᵢ` đến `pᵢ₊₁`; `lᵢ` là chiều dài Euclid của đoạn; ký hiệu `||·||₂` là chuẩn Euclid.

Hướng của đoạn được tính bằng:

`ψᵢ = atan2(yᵢ₊₁ − yᵢ, xᵢ₊₁ − xᵢ)`

Góc đổi hướng tại đỉnh `pᵢ` là:

`Δψᵢ = wrapToPi(ψᵢ − ψᵢ₋₁)`

Trong đó `ψᵢ₋₁` và `ψᵢ` lần lượt là hướng đi vào và hướng đi ra khỏi đỉnh; `wrapToPi(·)` đưa góc về khoảng `[-π, π]`; `Δψᵢ > 0` biểu diễn rẽ trái và `Δψᵢ < 0` biểu diễn rẽ phải theo quy ước hệ tọa độ.

### 3.2 Liên tục G⁰, G¹ và G²

- **G⁰ – liên tục vị trí:** hai đoạn gặp nhau tại cùng một điểm, không bị đứt đường.
- **G¹ – liên tục tiếp tuyến:** hai đoạn có cùng hướng tiếp tuyến tại điểm nối, vì vậy không còn góc gãy về hướng.
- **G² – liên tục độ cong hình học:** ngoài vị trí và hướng tiếp tuyến, độ cong ở hai phía của điểm nối cũng bằng nhau.

Trong đề tài, đoạn trước và sau góc là đường thẳng nên có độ cong bằng 0. Vì vậy đoạn Bézier chuyển tiếp cần thỏa:

`κ(0) = 0 và κ(1) = 0`

để nối G² với hai đoạn thẳng. G² là liên tục hình học, không đồng nghĩa hoàn toàn với C². C² yêu cầu đạo hàm theo cùng một tham số phải bằng nhau; G² chỉ yêu cầu hình dạng, hướng tiếp tuyến và độ cong phù hợp sau khi cho phép đổi tham số.

### 3.3 Đường Bézier bậc năm

Đường Bézier bậc năm được xác định bởi sáu điểm điều khiển `B₀, B₁, ..., B₅`:

`r(u) = Σᵢ₌₀⁵ bᵢ,₅(u)Bᵢ, 0 ≤ u ≤ 1`

với đa thức Bernstein:

`bᵢ,₅(u) = C(5,i)(1 − u)⁵⁻ⁱuⁱ`

Trong đó:

- `r(u) = [x(u), y(u)]ᵀ` là một điểm trên đường cong;
- `u` là tham số chuẩn hóa, chạy từ 0 ở đầu vào đến 1 ở đầu ra;
- `Bᵢ` là điểm điều khiển thứ `i`, đơn vị mét;
- `bᵢ,₅(u)` là hàm cơ sở Bernstein;
- `C(5,i) = 5!/[i!(5−i)!]` là hệ số tổ hợp.

Hai đạo hàm đầu của đường cong là:

`r′(u) = 5Σᵢ₌₀⁴ bᵢ,₄(u)(Bᵢ₊₁ − Bᵢ)`

`r″(u) = 20Σᵢ₌₀³ bᵢ,₃(u)(Bᵢ₊₂ − 2Bᵢ₊₁ + Bᵢ)`

`r′(u)` xác định hướng tiếp tuyến; `r″(u)` mô tả mức thay đổi của tiếp tuyến. Sáu điểm điều khiển của bậc năm cung cấp đủ bậc tự do để ràng buộc vị trí, hướng tiếp tuyến và độ cong ở cả hai đầu đoạn chuyển tiếp.

### 3.4 Khoảng cắt góc và bán kính tham chiếu

Giả sử hai điểm nối được đặt cách đỉnh góc một khoảng bằng nhau `d`. Với góc đổi hướng có độ lớn `|Δψ|`, bán kính cung tròn tiếp xúc tương đương dùng làm đại lượng tham chiếu là:

`R = d/tan(|Δψ|/2)`

Tương đương:

`d = R tan(|Δψ|/2)`

Trong đó:

- `d` là khoảng trim/cắt tính từ đỉnh góc dọc theo mỗi cạnh, đơn vị mét;
- `R` là bán kính hình học tham chiếu, đơn vị mét;
- `Δψ` là góc đổi hướng, đơn vị radian;
- `tan` là hàm tang.

Công thức này mô tả quan hệ hình học của cung tròn tiếp xúc và được dùng để diễn giải hoặc giới hạn miền tìm kiếm. Đoạn chuyển tiếp thực tế vẫn là Bézier bậc năm nên bán kính tức thời có thể thay đổi dọc đường cong.

### 3.5 Độ cong, chiều dài và năng lượng độ cong

Với `r(u) = [x(u), y(u)]ᵀ`, độ cong có dấu của đường phẳng là:

`κ(u) = [x′(u)y″(u) − y′(u)x″(u)]/[x′(u)² + y′(u)²]³ᐟ²`

Trong đó:

- `x′, y′` là đạo hàm bậc nhất theo `u`;
- `x″, y″` là đạo hàm bậc hai theo `u`;
- `κ` là độ cong, đơn vị `m⁻¹`;
- dấu của `κ` cho biết chiều rẽ; `|κ|` càng lớn thì góc cua càng gắt;
- khi `κ ≠ 0`, bán kính cong tức thời là `ρ = 1/|κ|`.

Phần tử chiều dài cung là:

`ds = ||r′(u)||₂ du`

Chiều dài của đoạn cong:

`L = ∫₀¹ ||r′(u)||₂ du`

Trong đó `s` là tọa độ chiều dài cung, đơn vị mét; `L` là tổng chiều dài đoạn.

Chỉ tiêu năng lượng độ cong được định nghĩa:

`Eκ = ∫ κ(s)² ds = ∫₀¹ κ(u)²||r′(u)||₂ du`

`Eκ` có đơn vị `m⁻¹`. Giá trị nhỏ thường biểu diễn đường ít gấp và ít dao động độ cong hơn. Đây chỉ là thước đo hình học, không phải năng lượng điện hoặc mức tiêu thụ pin.

### 3.6 Footprint, swept-footprint và clearance

Gọi `F` là tập điểm thuộc footprint của robot trong hệ tọa độ thân xe. Khi robot được đặt tại pose `q(s) = [x(s), y(s), θ(s)]`, footprint trong map là:

`F(q(s)) = Q(θ(s))F + [x(s), y(s)]ᵀ`

Trong đó `Q(θ)` là ma trận quay phẳng, được ký hiệu khác `R` để không nhầm với bán kính tham chiếu:

`Q(θ) = [[cosθ, −sinθ], [sinθ, cosθ]]`

Vùng quét của robot dọc đường là:

`S = ⋃ₛ F(q(s))`

Ứng viên chỉ được xem là an toàn nếu `S` không giao vùng vật cản `O`:

`S ∩ O = ∅`

Clearance nhỏ nhất có thể viết:

`cmin = minₛ min_{a∈F(q(s)), b∈O} ||a − b||₂`

Trong đó `θ(s)` là hướng tiếp tuyến của đường; `O` là tập vật cản; `S` là swept-footprint; `cmin` là khoảng hở nhỏ nhất, đơn vị mét. Trong chương trình, các đại lượng này được xấp xỉ bằng cách lấy mẫu đường và truy vấn costmap.

### 3.7 Hàm chi phí và quy hoạch động

Một hàm chi phí dự kiến cho toàn đường là:

`J = wκEκ + wL L + wdev D + wpivot Npivot + wobs Jobs`

Trong đó:

- `J` là tổng chi phí cần giảm;
- `Eκ` là năng lượng độ cong;
- `L` là chiều dài đường;
- `D` là độ lệch của đường làm mượt so với Raw;
- `Npivot` là số trạng thái Pivot;
- `Jobs` là thành phần phạt gần vật cản;
- `wκ, wL, wdev, wpivot, wobs` là các trọng số không âm.

Vì các đại lượng có đơn vị và độ lớn khác nhau, trước khi cộng cần chuẩn hóa hoặc chọn trọng số phù hợp. Công thức trên là cấu trúc thiết kế; trọng số chưa phải kết quả cố định.

Gọi `Cᵢ(j)` là chi phí cục bộ của ứng viên `j` tại góc `i`; `Tᵢ(k,j)` là chi phí chuyển hoặc giá trị vô cùng nếu ứng viên `k` và `j` chồng lấn. Truy hồi quy hoạch động là:

`Vᵢ(j) = Cᵢ(j) + minₖ[Vᵢ₋₁(k) + Tᵢ(k,j)]`

Trong đó:

- `Vᵢ(j)` là chi phí nhỏ nhất từ góc đầu tiên tới trạng thái `j` tại góc `i`;
- `k` duyệt các trạng thái của góc `i−1`;
- phép truy vết từ trạng thái có `V` nhỏ nhất ở góc cuối cho chuỗi Pivot/G² được chọn;
- với `N` góc và nhiều nhất `K` trạng thái mỗi góc, độ phức tạp là `O(NK²)`.

### 3.8 Bảng ký hiệu tổng hợp

| Ký hiệu | Ý nghĩa | Đơn vị |
|---|---|---|
| `pᵢ=[xᵢ,yᵢ]ᵀ` | Điểm thứ `i` của polyline | m |
| `eᵢ`, `lᵢ` | Vector và chiều dài đoạn thứ `i` | m đối với `lᵢ` |
| `ψᵢ`, `Δψᵢ` | Hướng đoạn và góc đổi hướng | rad |
| `u` | Tham số chuẩn hóa của Bézier | không đơn vị |
| `B₀...B₅` | Sáu điểm điều khiển Bézier | m |
| `d` | Khoảng cắt/trim tại góc | m |
| `R`, `ρ` | Bán kính tham chiếu và bán kính cong tức thời | m |
| `κ` | Độ cong có dấu | m⁻¹ |
| `s`, `L` | Tọa độ chiều dài cung và tổng chiều dài | m |
| `Eκ` | Tích phân bình phương độ cong | m⁻¹ |
| `q(s)`, `θ(s)`, `Q(θ)` | Pose trên đường, hướng tiếp tuyến và ma trận quay phẳng | m, rad, không đơn vị |
| `F`, `S`, `O` | Footprint, vùng quét và vùng vật cản | tập hình học |
| `cmin` | Clearance nhỏ nhất | m |
| `D` | Độ lệch so với đường Raw | m sau khi chọn định nghĩa cụ thể |
| `Npivot` | Số trạng thái Pivot | số đếm |
| `J`, `Cᵢ`, `Tᵢ`, `Vᵢ` | Các chi phí trong tối ưu và DP | phụ thuộc chuẩn hóa |
| `N`, `K` | Số góc và số trạng thái tối đa mỗi góc | số đếm |

## 4. Ý tưởng thuật toán

### 4.1 Đầu vào và điều kiện hóa đường

Đầu vào là một `nav_msgs/Path` dạng polyline từ global planner. Thuật toán loại điểm trùng hoặc đoạn quá ngắn, chuẩn hóa hướng của từng đoạn và xác định các đỉnh có góc đổi hướng đáng kể. Đường Raw ban đầu được giữ nguyên để làm mốc so sánh và phương án fallback.

### 4.2 Hai trạng thái tại mỗi góc

- **Pivot:** giữ đỉnh góc, không chèn đoạn cong. Trạng thái này phù hợp khi hành lang hẹp hoặc không đủ chiều dài để cắt góc.
- **G²:** cắt một đoạn có độ dài `d` trên hai cạnh kề và nối hai điểm cắt bằng đường Bézier bậc năm. Các điểm điều khiển được bố trí để tiếp tuyến và độ cong nối êm với hai đoạn thẳng, tức độ cong ở hai đầu chuyển tiếp tiến về 0.

Với góc đổi hướng `Δψ` và khoảng cắt `d`, bán kính hình học tham chiếu có thể viết:

`R = d / tan(|Δψ|/2)`

Đoạn chuyển tiếp được biểu diễn:

`r(u) = Σ Bᵢ,₅(u)Pᵢ, 0 ≤ u ≤ 1`

Độ cong được tính từ đạo hàm của đường:

`κ(u) = (x′y″ − y′x″)/(x′² + y′²)^(3/2)`

### 4.3 Sinh và kiểm tra ứng viên

Miền khả thi của `d` được giới hạn bởi chiều dài hai đoạn kề, khoảng cách tới các góc lân cận và vùng trống trên costmap. Thuật toán lấy mẫu từ thô đến tinh để tìm thêm ứng viên gần biên an toàn và vùng có chi phí thấp. Một ứng viên bị loại nếu:

- đoạn chuyển tiếp tự cắt hoặc tạo hình học bất thường;
- dấu độ cong thay đổi ngoài ý muốn;
- độ cong vượt ngưỡng cấu hình;
- hai đoạn chuyển tiếp kề nhau chồng lấn;
- bất kỳ mẫu nào của footprint quét dọc đường đi vào ô chiếm dụng hoặc vùng không xác định không được phép.

### 4.4 Ghép quyết định trên toàn đường

Mỗi góc giữ một số ứng viên G² an toàn và một trạng thái Pivot. Các ứng viên của hai góc kề chỉ được nối nếu tổng khoảng cắt và biên an toàn không vượt chiều dài đoạn chung. Bài toán được mô hình hóa thành các lớp trạng thái và giải bằng quy hoạch động. Với `N` góc và tối đa `K` trạng thái mỗi góc, độ phức tạp ghép dự kiến là `O(NK²)`.

Hàm chi phí hiện mới ở mức thiết kế, có thể gồm:

`J = wκ∫κ²ds + wL L + wdev D(raw, smooth) + wpivot Npivot + Jobs`

Trong đó `∫κ²ds` đo mức gấp/dao động độ cong, `L` là chiều dài, `D` là độ lệch so với Raw, `Npivot` là số thao tác Pivot và `Jobs` là phạt gần vật cản. Trọng số sẽ được chốt trước khi chạy thử nghiệm chính thức.

### 4.5 Cổng Hybrid và fallback

Nhánh Hybrid chỉ nhận một đường sau khi đường đó vượt kiểm tra footprint và các điều kiện hình học. Việc so sánh các nhánh dự kiến dựa trên mức chiếm dụng/clearance và effort hình học. Nếu không nhánh làm mượt nào hợp lệ, thuật toán trả về Raw thay vì cố tạo một đường cong không an toàn. Phần luật chọn nhánh này đang được hiệu chỉnh và chưa nên mô tả như một kết quả đã hoàn thiện.

## 5. Kế hoạch thử nghiệm

| Thành phần | Số lượng dự kiến | Ý nghĩa |
|---|---:|---|
| Môi trường Gazebo/Nav2 | 7 | Kho nghiên cứu, lối hẹp, văn phòng, không gian mở, kho giao cắt, kho điều phối và kho lối dài |
| Cặp điểm đầu–đích | 60 | Gồm đường thẳng, rẽ gấp, chữ S, hành lang hẹp và đường vòng vật cản |
| Global planner | 5 | NavFn A*, NavFn Dijkstra, Theta*, Smac 2D và Smac Hybrid |
| Đường/phương pháp so sánh | 8 | Raw, Simple, Savitzky–Golay, Constrained, Pivot–G2 fixed/adaptive và Hybrid fixed/adaptive |
| Lần lặp | 3 | Kiểm tra khả năng lặp lại; mọi phương pháp trong một nhóm nhận cùng Raw |
| Nhóm đầu vào ghép cặp | 900 | 60 tình huống × 5 planner × 3 lần lặp |
| Bản ghi đường hình học | 7.200 | 900 nhóm × 8 phương pháp |

**Cách gọi chính xác:** 7.200 là số bản ghi đường đi/quỹ đạo hình học sau lập kế hoạch và làm mượt, không phải 7.200 lượt robot thực thi trong Gazebo.

## 6. Điểm đánh giá và đặc tính dự kiến thu được

| Nhóm | Chỉ tiêu | Đặc tính cần rút ra |
|---|---|---|
| Khả năng tạo đường | Tỷ lệ planner có Raw; tỷ lệ smoother trả kết quả; timeout | Độ bền của thuật toán trên nhiều dạng đầu vào |
| An toàn | Số đường va chạm footprint; clearance nhỏ nhất/trung bình; peak cost | Khả năng không cắt vật cản khi làm tròn góc |
| Độ trơn | Độ cong cực đại; tích phân bình phương độ cong; biến thiên độ cong | Mức giảm góc gấp và dao động hình học |
| Mức thay đổi đường | Chiều dài; độ lệch cực đại/RMS so với Raw | Sự đánh đổi giữa làm mượt và giữ hành lang ban đầu |
| Cấu trúc quyết định | Số góc Pivot/G²; bán kính hoặc `d` được chọn; nhánh Hybrid | Thuật toán thích nghi như thế nào theo không gian |
| Chi phí tính toán | Thời gian trung bình, P95, cực đại | Khả năng dùng như plugin smoother trong Nav2 |
| Tính công bằng | Hash của Raw; cùng planner/scenario/repetition | Bảo đảm các smoother nhận đúng cùng một đầu vào |

Tích phân bình phương độ cong chỉ là một thước đo effort hình học; không gọi đây là năng lượng điện hoặc mức tiêu thụ pin.

## 7. Kết quả dự kiến và cách phát biểu an toàn

Kết quả kỳ vọng là nhánh G² giảm độ gấp và dao động độ cong tại những góc có đủ không gian; nhánh Pivot giữ khả năng xử lý các góc hẹp; kiểm tra swept-footprint loại các đoạn cong có nguy cơ cắt vật cản; và Raw fallback giúp hệ thống vẫn trả đường khi các ứng viên làm mượt không hợp lệ. Dữ liệu cũng dự kiến cho thấy sự đánh đổi: độ trơn có thể tốt hơn nhưng độ lệch khỏi Raw hoặc thời gian tính toán có thể tăng.

Không nên tuyên bố thuật toán luôn tạo đường ngắn nhất, nhanh nhất hoặc tối ưu toàn cục. Abstract cuối chỉ nên đưa phần trăm cải thiện sau khi khóa phiên bản code, chốt trọng số, chạy lại đầy đủ ma trận và báo cáo cả các trường hợp thất bại.

## 8. Việc cần hoàn thiện trước abstract cuối

- Chốt tiêu chí góc cần xử lý và miền tìm kiếm `d`.
- Chốt cách lấy mẫu footprint và xử lý ô unknown.
- Chốt công thức chi phí và trọng số của DP.
- Chốt luật so sánh các nhánh trong Hybrid.
- Khóa code/config/seed rồi chạy lại toàn bộ dữ liệu.
- Không trộn dữ liệu pilot của các phiên bản thuật toán khác nhau.
- Báo cáo cả failure và timeout, không chỉ các đường thành công.
