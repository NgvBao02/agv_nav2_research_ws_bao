# So sánh PSTMO với các smoother chuẩn của ROS 2/Nav2

Benchmark này dùng phiên bản hiện tại của workspace và so sánh:

- Raw;
- Nav2 `SimpleSmoother` (`simple`);
- Nav2 `SavitzkyGolaySmoother` (`savitzky_golay`);
- Nav2 `ConstrainedSmoother` (`constrained`);
- PSTMO (`pivot_g2`).

`adaptive_hybrid` không được chạy.

## Thiết kế

- 7 môi trường, mỗi môi trường một tình huống đại diện;
- 5 global planner;
- 1 lần đánh giá cho mỗi tổ hợp tình huống–planner;
- 35 nhóm ghép cặp, mỗi nhóm có cùng `raw_path_sha256`;
- 5 phương pháp, tổng cộng 175 bản ghi đường.

Ba smoother đối chứng là plugin Nav2 stock được cài trong ROS 2 Jazzy. Mọi
smoother nhận cùng một đường Raw đã canonicalize trong từng nhóm.

## Tính hợp lệ

- Raw, Savitzky–Golay, Constrained và PSTMO: 35/35 thành công;
- Simple: 34/35 thành công;
- lỗi Simple duy nhất là `SmacHybrid/full_replenishment`, action trả
  `FAILED_TO_SMOOTH_PATH` (mã 503);
- 35/35 nhóm có cùng raw hash giữa các phương pháp;
- 35/35 raw hash trùng với benchmark PSTMO rút gọn trước đó;
- không có mẫu va chạm footprint trong 174 đường đầu ra thành công;
- không có bản ghi `adaptive_hybrid`.

## So sánh công bằng trên 34 nhóm cùng thành công

| Phương pháp | Năng lượng độ cong tịnh tiến (1/m) | So với Raw | Chiều dài (m) | So với Raw | Khoảng hở footprint nhỏ nhất (m) | So với Raw |
|---|---:|---:|---:|---:|---:|---:|
| Raw | 50,207 | — | 10,314 | — | 0,161 | — |
| Simple | 9,418 | −81,24% | 10,240 | −0,72% | 0,191 | +18,29% |
| Savitzky–Golay | 22,699 | −54,79% | 10,296 | −0,18% | 0,191 | +18,58% |
| Constrained | 28,065 | −44,10% | 10,332 | +0,17% | **0,213** | **+32,12%** |
| **PSTMO** | **4,511** | **−91,01%** | **10,195** | **−1,16%** | 0,176 | +9,42% |

PSTMO có năng lượng độ cong thấp nhất và đường ngắn nhất. So với smoother
Nav2 stock tốt nhất về độ cong là Simple, PSTMO giảm thêm 52,10% năng lượng độ
cong và rút ngắn thêm 0,45% chiều dài. Đổi lại, Constrained đạt khoảng hở lớn
nhất; khoảng hở trung bình của PSTMO thấp hơn Constrained 17,18%.

## Mức độ đồng đều

Số nhóm có khoảng hở tối thiểu tăng/bằng/giảm so với Raw:

| Phương pháp | Tăng | Bằng | Giảm |
|---|---:|---:|---:|
| Simple | 20 | 4 | 10 |
| Savitzky–Golay | 18 | 12 | 4 |
| Constrained | 28 | 6 | 0 |
| PSTMO | 19 | 3 | 12 |

Vì chỉ có một tình huống trên mỗi môi trường và một lần đánh giá, các số liệu
này là so sánh mô tả ghép cặp, chưa phải kiểm định thống kê độc lập.

## Thời gian xử lý

Trên 34 nhóm cùng thành công, smoothing duration trung bình của PSTMO là
33,71 ms, runtime nội bộ là 33,51 ms và wall time trung bình là 46,21 ms. Wall
time trung bình của Simple,
Savitzky–Golay và Constrained lần lượt là 9,03, 7,45 và 29,50 ms. PSTMO chậm
nhất trong nhóm so sánh này nhưng vẫn dưới 50 ms trung bình.

## Tệp

- `*.csv`: 25 bản ghi của từng môi trường;
- `*_summary.json`: summary gốc do benchmark sinh;
- `aggregate_summary.json`: tổng hợp và kiểm định chung.
