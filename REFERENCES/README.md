# Tài liệu tham khảo PSTMO

Tra cứu bài journal hiện tại bằng [bộ VER12](VER12/README.md) và
[danh mục 54 nguồn theo số trong bài](TAI_LIEU_THAM_KHAO_VER12.md).
Các số `001–054` bên trong `VER12/` khớp `[1]–[54]` trong Word ver12.
Thư mục được đồng bộ ngày 26-10-2026 từ danh mục của bản thảo đã hoàn tất
ngày 24-10-2026.

## Bộ nguồn dùng cho ver12

- [VER12/](VER12/README.md): 54 mục đánh số theo bài, gồm 44 liên kết PDF,
  bốn liên kết snapshot HTML và sáu phiếu trích dẫn kèm URL.
- [TAI_LIEU_THAM_KHAO_VER12.md](TAI_LIEU_THAM_KHAO_VER12.md): trích dẫn nguyên
  văn, số mới, khóa nguồn, mã corpus cũ, tệp và vị trí sử dụng trong bài.
- [REFERENCE_MANIFEST_VER12.csv](REFERENCE_MANIFEST_VER12.csv): cùng danh mục
  ở dạng CSV, có trạng thái lưu trữ, đường dẫn gốc, URL và hash nguồn cục bộ.
- [CORPUS_STATUS_VER12.csv](CORPUS_STATUS_VER12.csv): phân loại cả 74 nguồn cũ;
  48 được dẫn trong ver12, 26 không được dẫn, kèm lý do của lần hiệu đính.
- [VER12_SYNC_AUDIT.json](VER12_SYNC_AUDIT.json): hash đầu vào và kết quả kiểm
  tra đồng bộ với Word.

Sáu nguồn chỉ có phiếu trích dẫn và URL là Choi 2014 `[28]`, Constrained
Smoother Jazzy `[29]`, Smoother Server Jazzy `[49]`, Efron `[50]`, Wilcoxon
`[51]` và Holm `[52]`. Đây là các nguồn ngoài corpus 74 đã có trong ver12.
Phiếu Markdown không thay cho toàn văn và không được tính là PDF hay snapshot.
Lần đồng bộ này không tải thêm hoặc tái thẩm định nguồn trực tuyến.

## Phân biệt số bài báo và mã thư viện

Ví dụ Piazzi 2002 là nguồn `[10]` của ver12, khóa `R031`, còn PDF gốc có tiền
tố `031_`. Trong `VER12/` nó được truy cập bằng tiền tố `010_`. Tên tệp giữ
phần năm/tác giả/từ khóa từ tệp nguồn; trích dẫn chuẩn lấy từ ver12, không suy
phiên bản xuất bản chỉ từ năm trong tên tệp.

Các tệp đánh số ở `VER12/` trỏ tương đối về tệp gốc trong thư mục cha bằng
symlink, tránh nhân đôi các PDF lớn. Khi sao chép bộ tham khảo, giữ **cả thư
mục REFERENCES** để các liên kết còn hoạt động. Sáu phiếu URL là tệp Markdown
thông thường.

## Corpus và hồ sơ lịch sử

74 tài liệu gốc ở cấp này gồm 70 PDF và bốn HTML, đánh mã `001–074` theo ngân
hàng ứng viên ban đầu. [REFERENCE_MANIFEST.csv](REFERENCE_MANIFEST.csv) và
[TAI_LIEU_THAM_KHAO_VER2.md](TAI_LIEU_THAM_KHAO_VER2.md) tiếp tục mô tả corpus
đó. **Số trong hai tệp cũ không phải số trích dẫn của ver12.**

Tệp gốc và hai danh mục lịch sử được giữ nguyên byte để các bản báo cáo cũ và
hash trong [baseline ver12](../Viet_bao/journal/ver12/baseline.json) tiếp tục
hợp lệ. Các nguồn không dùng ở ver12 vẫn có thể phục vụ nghiên cứu khác.

Nguồn có thẩm quyền cho thứ tự và trích dẫn mới là
[Word ver12](../Viet_bao/journal/ver12/PSTMO_JOURNAL_VER12_FULL_MANUSCRIPT.docx)
và [citation_map.json](../Viet_bao/journal/ver12/citation_map.json).
[Báo cáo hiệu đính](../Viet_bao/journal/ver12/SCIENTIFIC_REVISION_REPORT.md)
ghi phạm vi đọc và giới hạn bằng chứng; đồng bộ thư viện không mở rộng phạm
vi đã đọc của các tài liệu.

## Kiểm tra và tái tạo

Chạy từ root repository, dùng Python 3 với thư viện chuẩn:

```bash
python3 tools/sync_references_ver12.py --check
python3 tools/check_repository_docs.py
git diff --check
```

[Script đồng bộ](../tools/sync_references_ver12.py) so nguyên văn danh mục
với Word, kiểm tra 54 số liên tục, hash nguồn, symlink và 109 tệp đầu vào đã
khóa trong ver12. Chạy không có `--check` để tạo lại những tệp sinh còn thiếu;
script từ chối ghi đè nội dung khác hoặc liên kết sai. Nếu tác giả đổi bài
báo, phải đối chiếu lại danh mục và tạo phiên bản thư viện tương ứng.

Thay đổi lần này chỉ bổ sung lớp tra cứu theo ver12 và hướng dẫn liên quan;
không thay bài báo, mã ROS hoặc kết quả thực nghiệm.
