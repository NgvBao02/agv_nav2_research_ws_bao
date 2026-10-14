# Bản sửa REV-ECIT 2026 phiên bản 4

Bản này được biên tập từ [bản Word phiên bản 3](../ver3/SuaREV_ECIT2026_PSTMO.docx).
Không ghi đè bản gốc và không chạy thêm mô phỏng.

## Tệp chính

- [Bản Word](REV_ECIT2026_PSTMO_VER4.docx): nội dung tiếng Việt, hai cột theo
  [mẫu Word chính thức lưu trong repository](../ecit_word_template.docx).
- [Bản PDF đã kiểm tra sáu trang](REV_ECIT2026_PSTMO_VER4.pdf).
- [Nội dung để đối chiếu](manuscript.md): văn bản và công thức nguồn; không dùng
  thay bản Word khi nộp bài.
- [Hình khoa học](figures/): PNG 600 dpi, SVG và PDF vector của sáu hình.
- [Đối chiếu số liệu](data_audit.json) và [nguồn dữ liệu](provenance.json).
- [Số đo của 170 lượt thuộc 34 nhóm đầy đủ](paired_measurements.csv).
- [Quy cách trình bày](template_contract.md).
- [Kết quả kiểm tra bàn giao](quality_checks.json).
- [Lưu ý riêng trước khi nộp](submission_notes.md): không thuộc nội dung bài báo.

## Những thay đổi chính

Viết lại phần giới thiệu và lý thuyết theo trọng tâm đánh giá thực thi, không
trình bày lại phương pháp như một đóng góp hoàn toàn mới. Sáu hình được dựng
lại; ba hình đường tham chiếu có bản đồ, vị trí đầu–đích và hai vùng phóng đại.
Công thức là đối tượng toán học Word chỉnh sửa được, không phải ảnh chụp.
Tên trường dữ liệu và mã kiểm tra được giữ trong hồ sơ tái lập, không đưa vào
nội dung khoa học.

Các bảng hình học và thực thi cùng sử dụng 34 nhóm có đủ năm phương án thành
công, thay vì trộn kết quả của những đợt đo khác nhau. Độ cong được tính lại
trên các đường tham chiếu đã lấy mẫu theo chiều dài cung với bước 0,05 m.
Do thay đổi quy tắc đánh giá độ cong, các trị số và phần trăm liên quan không
thể so trực tiếp với những bảng dùng mật độ điểm nguyên thủy.

## Dữ liệu và giới hạn diễn giải

Nguồn chính là [ma trận thực thi 175 lượt](../../../results/pstmo_execution_full_20260803/)
được lập chỉ mục bởi `execution_175_cases.csv`. Một số bản ghi Theta* nằm tại
`results/pstmo_execution_theta_star_20260802/`, theo đường dẫn trong chỉ mục.
Có 170 lượt đạt và 34 nhóm môi trường–bộ lập kế hoạch có đủ năm phương án
thành công, cùng đường đầu vào. Cả năm thất bại được giữ trong tỷ lệ 34/35 của
từng phương án; không đưa lượt thất bại vào trung bình thời gian hoàn thành.

Đây là một lượt cho mỗi tổ hợp, không phải nghiên cứu lặp nhiều lần. Không
gộp dữ liệu của bài journal hoặc các đợt hình học khác vào bài này. RMSE đo
khoảng cách đến đường tham chiếu riêng của từng phương án, trên chuỗi vị trí
đã lấy mẫu lại và lọc; không phải sai số định vị hoặc sai số tức thời chưa lọc.
Kiểm tra va chạm rời rạc không chứng minh an toàn liên tục.

## Tái lập

Chạy từ thư mục gốc repository, trong môi trường Python có `python-docx`,
`matplotlib`, `numpy`, `scipy`, `Pillow`, `PyYAML`, `lxml`, `latex2mathml` và
`mathml2omml`:

```bash
python3 tools/build_rev_ecit_v4.py --audit-only
python3 tools/build_rev_ecit_v4.py
```

Lệnh đầu kiểm tra lại số liệu và ghi hồ sơ kiểm tra. Lệnh sau còn tái tạo
Word, nội dung và hình trong thư mục này; không sửa bản ghi thực nghiệm.
Các hàm hình học thuần số được lấy từ mã benchmark và được ghi mã kiểm tra
trong hồ sơ nguồn. Nếu mã hoặc dataset thay đổi, phải kiểm tra lại các kết
quả trước khi dùng bản tái tạo.

Quy cách hiện công bố trên [trang hướng dẫn REV-ECIT](https://rev-ecit.vn/en/content/huong-dan-nop-bai)
là tiếng Việt, A4, mẫu IEEE và không quá sáu trang. Việc kiểm tra bố cục phải
gồm xuất Word thành PDF và xem từng trang, không chỉ kiểm tra văn bản XML.

Bản bàn giao đã xuất và kiểm tra bằng kỹ năng Documents/PDF. Những kiểm tra
này dẫn đến việc sửa hiển thị căn bậc hai, tổ hợp, số công thức, ngắt bảng,
chú thích và cấu hình font PDF. Không sửa mã ROS hoặc dữ liệu nguồn. Tài liệu
root và README này đã được đồng bộ; `tools/check_repository_docs.py` và
`git diff --check` đều đạt.
