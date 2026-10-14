# Bản sửa REV-ECIT 2026 phiên bản 5

Bản này tiếp nối [phiên bản 4](../ver4/README.md), giữ nguyên các bản trước
và dữ liệu thực nghiệm. Không chạy lại mô phỏng và không sửa mã ROS.

## Tệp chính

- [Bản Word](REV_ECIT2026_PSTMO_VER5.docx).
- [Bản PDF](REV_ECIT2026_PSTMO_VER5.pdf): sáu trang A4 hai cột.
- [Nội dung nguồn](manuscript.md), [hình khoa học](figures/) gồm bảy hình
  dưới dạng PNG 600 dpi, SVG và PDF vector.
- [Đối chiếu số liệu](data_audit.json), [số đo ghép cặp](paired_measurements.csv),
  [hồ sơ nguồn](provenance.json) và [kiểm tra bàn giao](quality_checks.json).
- [Quy cách trình bày](template_contract.md).

## Nội dung sửa

Thêm hình Bézier bậc năm tính trực tiếp từ sáu điểm điều khiển, có đỉnh góc,
hai điểm nối, vectơ tiếp tuyến và chiều dài cắt. Hình này là sơ đồ minh họa
toán học, không phải một phép đo thực nghiệm. Các công thức vẫn là toán học
Word chỉnh sửa được.

Mở rộng phân tích III-C–F, thay hai hình tổng hợp bằng so sánh với đủ bốn đối
chứng. Hình 6 biểu diễn chênh lệch thời gian và RMSE; Hình 7 tách bốn phép
so sánh độ cong–thời gian. Không gộp 136 điểm có chung kết quả PSTMO thành
136 quan sát độc lập. Nêu cả các trường hợp PSTMO chậm hơn hoặc có sai số lớn
hơn, cùng chi phí làm mượt cao hơn.

Phần III-E tập trung vào năm thất bại, đối chiếu JSON và nhật ký từng lượt.
Simple bị từ chối đường ứng viên do phát hiện va chạm ở bước làm mượt; bốn
phương án còn lại dừng sau cảnh báo dự đoán va chạm của bộ điều khiển và vượt
thời gian cho phép. Không diễn giải cảnh báo này thành va chạm vật lý đã xảy
ra; không dùng thời gian ngắn hoặc RMSE nhỏ của ca bị hủy làm bằng chứng tốt.

Hiệu chỉnh biên tập tiếp theo làm rõ đầu vào và đầu ra của PSTMO, phạm vi của
tính liên tục G2, điều kiện ghép các chuyển tiếp và giới hạn diễn giải cấu hình
Constrained. Ba quỹ đạo M2, M4 và M6 được đặt trong một mục chung; phân tích
định lượng được trình bày sau hình và bảng tương ứng. Phần thất bại nêu rõ
PSTMO chưa cải thiện tỷ lệ hoàn thành trong tập thử. Các thay đổi này không
thay đổi dữ liệu, số hình, số bảng hoặc danh mục 16 nguồn.

## Quy ước môi trường và nguồn số liệu

Trong bài chỉ dùng M1–M7. Tên nội bộ chỉ được giữ trong hồ sơ tái lập:

| Mã bài báo | Mã dataset |
| --- | --- |
| M1 | `open_arena` |
| M2 | `research_warehouse` |
| M3 | `narrow_aisles` |
| M4 | `office_maze` |
| M5 | `warehouse_cross_aisles` |
| M6 | `warehouse_dispatch` |
| M7 | `warehouse_long_aisles` |

Nguồn chính là [ma trận 175 lượt](../../../results/pstmo_execution_full_20260803/),
qua chỉ mục `execution_175_cases.csv`; các bản ghi Theta* được đọc theo đường
dẫn trong chỉ mục. Mỗi tổ hợp chỉ chạy một lần. Có 170 lượt thành công thuộc
34 nhóm đầy đủ và năm thất bại cùng nhóm M6/Smac Hybrid. Mỗi phương án đạt
34/35 lượt. Trung bình chỉ tính trên 34 nhóm đầy đủ, không gộp dữ liệu journal.

Độ cong được tính trên đường lấy mẫu lại cách đều 0,05 m; RMSE dùng quỹ đạo
đã lấy mẫu lại và lọc, đo tới đường tham chiếu riêng của mỗi phương án. Các
giới hạn diễn giải từ phiên bản 4 vẫn áp dụng. Chênh lệch phần trăm là tỷ lệ
của hai trung bình, không phải trung bình các phần trăm từng nhóm.

## Tái lập và kiểm tra

Môi trường Python cần `python-docx`, `matplotlib`, `numpy`, `scipy`, `Pillow`,
`PyYAML`, `lxml`, `latex2mathml`, `mathml2omml`, `pdfplumber` và `pypdf`.

```bash
python3 tools/build_rev_ecit_v5.py --audit-only
python3 tools/build_rev_ecit_v5.py
```

Lệnh thứ hai tái tạo Word, hình và hồ sơ nguồn của ver5. PDF cần xuất lại từ
Word sau mỗi thay đổi, sử dụng `render_docx.py` của kỹ năng Documents hoặc
Word. Cần có Times New Roman thật và kiểm tra font nhúng, không chấp nhận
thay thế font âm thầm. Xem đủ sáu trang trước khi xác nhận kiểm tra trực quan:

```bash
python3 tools/check_rev_ecit_v5.py --pdf Viet_bao/bao_Rev_ecit_2026/ver5/REV_ECIT2026_PSTMO_VER5.pdf --visual-reviewed
python3 tools/check_repository_docs.py
git diff --check
```

Trình kiểm tra đối chiếu mã kiểm tra nguồn, số hình/bảng, cách dẫn công thức,
hình học trang, các phần mẫu được giữ nguyên và giới hạn văn bản PDF. Kiểm tra
tự động không thay thế việc xem từng trang. Khi dữ liệu hoặc bộ sinh thay đổi,
phải tái sinh và kiểm tra lại; không dùng kết quả QA cũ cho bản mới.

Danh mục gồm 16 nguồn: 14 công trình đã xuất bản và hai tài liệu kỹ thuật
Nav2. Bài không sử dụng nguồn chưa công bố; PSTMO được trình bày như phương
pháp đề xuất trong chính bài này. Các lưu ý riêng
về nộp bài ở [phiên bản trước](../ver4/submission_notes.md) vẫn cần được tác
giả kiểm tra. Chất lượng trình bày không bảo đảm quyết định chấp nhận.

Tiêu đề bài báo được viết hoàn toàn bằng tiếng Việt và không dùng chữ viết
tắt PSTMO. Tên PSTMO được giới thiệu lần đầu trong câu mở đầu của Tóm tắt,
sau mô tả tiếng Việt về phương pháp hậu xử lý bằng chuyển tiếp Bézier bậc năm.
