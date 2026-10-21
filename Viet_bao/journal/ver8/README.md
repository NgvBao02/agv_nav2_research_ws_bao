# Bản thảo journal PSTMO phiên bản 8

Bản biên tập mạch lập luận từ [ver7](../ver7/README.md). Không chạy mô phỏng
mới, không thay số liệu, không ghi đè bản cũ. Giữ 18 ảnh và 23 bảng của ma
trận 125 lượt cùng bộ kiểm chứng lặp 250 lượt.

- [Bản Word](PSTMO_JOURNAL_VER8_PHAN_II_III.docx).
- [Nội dung đã sinh](manuscript.md), [nhật ký thay đổi](revision_log.json).
- [Số liệu bảng](table_values.json), [hồ sơ nguồn](provenance.json).
- [Kết quả kiểm tra](quality_checks.json), [quy cách biên tập](template_contract.md).
- [Script dựng](../../../tools/build_journal_sections_v8.py) và
  [script kiểm tra](../../../tools/verify_journal_sections_v8.py).

## Nội dung sửa

1. II-D trình bày điều kiện thời gian trước hàm chi phí; hoán đổi số công
   thức 5/6 và dẫn chiếu, giữ nguyên các đối tượng Office Math. Phân biệt
   điều kiện mở nhánh chuyển tiếp với giới hạn thời gian của các phương án
   được giữ. Đối chiếu trực tiếp `parameterize_common_window` và điều kiện
   `transition_branch_open` trong plugin; SHA-256 source khớp hồ sơ ver6.
2. III-A giải thích mục đích năm tuyến, vai trò đối chứng và lý do bộ lặp
   trước các kết quả. Chuyển lý do từ III-H lên III-A.5, không gán cho thiết
   kế lịch sử những giả định về đăng ký trước hoặc chọn đối chứng tốt nhất.
3. III-A.7 đặt xử lý quỹ đạo trước định nghĩa chỉ số; phân biệt đường tham
   chiếu, quỹ đạo đã xử lý, quãng đường từ trạng thái gốc và sai số cuối.
4. Chỉnh tiêu đề R01/R03/R05; chuyển so sánh thời gian–RMSE ở R04 lên đúng
   mục. Không mô tả R04 là phép thử tách riêng ảnh hưởng của chiều tiếp cận.
5. Bỏ cách diễn đạt “bù” giữa sai số cuối và RMSE; thay cách gọi “định vị
   cuối” bằng sai số vị trí cuối. Giữ nguyên các dãy số đủ năm phương án.
6. Kết quả thất bại của bộ lặp chỉ được phân tích tại III-H/III-I, không chen
   giữa số liệu khảo sát R05. Giữ nguyên các ca thất bại và mẫu số.
7. Phân biệt Eκ,ref trong Hình 15 với Eκ,exec trong Bảng XVI. Hình 16 giữ
   nguyên ảnh và thời gian toàn nhiệm vụ; bỏ so sánh ngưỡng 0,24 rad/s trong
   cửa sổ 34–40 s vì hồ sơ biên tập không xác lập cơ sở lựa chọn. Không coi
   một hình minh họa là bằng chứng đại diện hoặc phân tích nhân quả.
8. Rút phần giới hạn lặp lại giao thức, giữ các giới hạn về cấu hình,
   ngẫu nhiên hóa, tài nguyên dùng chung và phân tích cặp cùng đạt.
9. Đặt Hình 8 ngay trước phân tích theo bộ lập kế hoạch, sau Bảng VIII và
   đoạn diễn giải bảng; đặt Hình 18 sau hai đoạn phân tích thất bại, trước
   mục giới hạn, để giảm khoảng trắng khi dàn hai cột.
10. Bỏ section rỗng một cột ở cuối để tránh trang trắng, giữ một section hai
    cột. Đoạn rỗng cuối dùng khoảng dòng và dấu đoạn 1 pt. Không giảm cỡ chữ,
    giãn dòng thân bài hoặc lề.
11. Giữ chỉnh sửa mới nhất của người dùng: bỏ câu ngày thu dữ liệu R01 tại
    III-A.5. Vẫn nêu giới hạn khác đợt thu dữ liệu trong phần giới hạn nghiên cứu.

## Tái tạo và kiểm tra

Nguồn biên tập là script dựng, bản Word ver7 gốc được lưu tại
[bản tham chiếu](reference_ver7_original.docx) và manuscript ver7. Không sửa trực tiếp
`manuscript.md` ver8 rồi chạy script, vì đây là đầu ra sinh. Lưu riêng các
chỉnh sửa Word thủ công trước khi tái tạo.

Trong lúc biên tập, Word ver7 được người dùng lưu lại và bỏ câu ngày thu dữ
liệu. Bản tham chiếu gốc được tái dựng ở thư mục tạm từ script ver7, xác nhận
SHA-256 trùng hoàn toàn với nguồn trước chỉnh sửa rồi lưu riêng; không ghi
đè Word ver7 hiện tại. Hồ sơ nguồn theo dõi cả hai file. Ver8 đã tiếp nhận
việc bỏ câu ngày thu dữ liệu, không lấy thay đổi định dạng do lưu lại làm mẫu mới.

Script nhập các hàm ánh xạ XML và định dạng từ bộ dựng ver7 nhưng không gọi
quy trình dựng ver7. Chỉ `word/document.xml` và `docProps/core.xml` được đổi;
các thành phần gói khác giữ nguyên byte. Giữ nguyên bảng, ảnh và công thức,
chỉ hoán đổi vị trí và nhãn hai công thức đã nêu, đồng thời chuyển vị trí
Hình 8 và Hình 18 trong cùng tiểu mục mà không đổi thứ tự đánh số hình.

Dùng Python trong runtime Documents với lxml, python-docx và PyMuPDF. Phiên
biên tập dùng môi trường ảo tạm `/tmp/pstmo-v8-env`, PyMuPDF 1.26.7 và font
Times New Roman tại `~/.local/share/fonts/pstmo-times`. Tạo lại môi trường
tạm nếu đổi phiên. Render bằng `render_docx.py` và LibreOffice đóng gói của
Documents; không dùng LibreOffice desktop để ghi lại tài liệu.

```bash
PYTHONNOUSERSITE=1 /tmp/pstmo-v8-env/bin/python tools/build_journal_sections_v8.py
# Render Word vào một thư mục QA mới bằng render_docx.py, kèm --emit_pdf.
PYTHONNOUSERSITE=1 /tmp/pstmo-v8-env/bin/python tools/verify_journal_sections_v8.py --render-dir /tmp/pstmo-v8-final-3 --accept-visual-review
python3 tools/check_repository_docs.py
git diff --check
```

Chỉ dùng `--accept-visual-review` sau khi xem toàn bộ ảnh trang của đúng bản
cuối. Bộ kiểm tra xác nhận mã nguồn và dữ liệu nguồn không đổi, bảng và các
dãy số trong văn bản giữ nguyên, dẫn chiếu công thức đúng thứ tự, nội dung
thất bại không mất và các phần của gói Word được bảo toàn. Kiểm tra số liệu
thống kê kế thừa từ ver7 qua hồ sơ nguồn và đối chiếu nguyên vẹn; không tuyên
bố thực hiện lại thử nghiệm hoặc kiểm định. Không cần ROS test cho đợt sửa này.

Bản cuối gồm 15 trang hai cột; đã xem toàn bộ ảnh trang ngày 21/10/2026.
Số trang và xác nhận trực quan cuối cùng nằm trong `quality_checks.json`.
PDF/PNG tại `/tmp` chỉ dùng kiểm tra, không phải đầu ra bàn giao. Bản này vẫn
chỉ gồm phần II–III và bảy tài liệu tham khảo cục bộ, kế thừa mẫu IEEE A4 hai
cột; cần ghép toàn bài, đánh số lại tài liệu tham khảo và đối chiếu mẫu của
tạp chí đích trước khi nộp.
