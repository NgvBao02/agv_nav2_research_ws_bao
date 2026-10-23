# PSTMO Journal ver12

Bản hiệu đính khoa học hoàn chỉnh từ Word ver11 **thực tế sau khi tác giả xóa
Abstract tiếng Anh**. Bản giao ngày 24-10-2026 có **21 trang, 54 tài liệu tham
khảo, 23 bảng, 18 hình và chín công thức đánh số**. Chỉ giữ Tóm tắt và từ khóa
tiếng Việt; khối tên bài Việt/Anh được giữ nguyên. Không sửa ver11 hoặc nguồn
dữ liệu, không chạy thí nghiệm hay tái phân tích thống kê.

## Tệp chính

- [Bài báo Word hoàn chỉnh](PSTMO_JOURNAL_VER12_FULL_MANUSCRIPT.docx).
- [Báo cáo hiệu đính khoa học](SCIENTIFIC_REVISION_REPORT.md): thay đổi, lý do,
  bằng chứng, đối chiếu công trình và hạn chế chưa giải quyết.
- [So sánh ver11 với ver12](VER11_VS_VER12.md).
- [Vai trò và phạm vi tài liệu tham khảo](REFERENCE_COVERAGE.md): 54 nguồn giữ,
  26 nguồn bỏ khỏi bài có lý do riêng. Không xóa tệp nào trong REFERENCES.
- [Kiểm chứng cấu trúc và render](quality_checks.json),
  [ghi chú kiểm tra 21 trang](visual_review.json),
  [đối chiếu số liệu và cohort](evidence_audit.json).

## Phạm vi thay đổi

Introduction tập trung vào bài toán phối hợp chuyển hướng trên các cạnh chung,
đối chiếu sâu Piazzi 2002, Brezak 2011, Blažič 2023, Choi 2014 và Nav2
Constrained. Hai đoạn bổ sung tại II-D/E làm rõ ba vai trò khác nhau của Eκ,
điều kiện thời gian và tổng J; giải thích trạng thái DP và điểm đi qua.
Không thay truy hồi, tập ứng viên, ràng buộc hoặc mã thuật toán.

II giữ nguyên văn bản gốc ngoài hai đoạn thêm và đánh số citation. III giữ
nguyên ngoài số citation, gồm cả kết quả bất lợi và chín ca không đạt. Tóm tắt
và Kết luận được đồng bộ với phạm vi bằng chứng. Không tuyên bố tối ưu thời
gian toàn cục, tính mới đầu tiên của DP kết hợp Bézier, hay hiệu quả nhân quả
của từng thành phần khi chưa có ablation.

Các giới hạn còn lại gồm thiếu ablation DP/pivot, thiếu robot thật và môi
trường độc lập, khoảng hở chỉ kiểm tra theo mẫu, và đối chứng theo cấu hình đã
thử chứ chưa tinh chỉnh cùng ngân sách. Nguồn tham khảo ngoài nhóm trọng tâm
kế thừa phạm vi đọc từ ver11; không tuyên bố đọc lại toàn văn cả 54 nguồn.
Chi tiết nằm trong báo cáo khoa học và [metadata nguồn](references.json).

## Nguồn có thẩm quyền

Nguồn Word là [ver11](../ver11/PSTMO_JOURNAL_VER11_FULL_MANUSCRIPT.docx), SHA-256
`debafc4f83a9b7a4dfd52230908cb37fac42ccfb5c23bebea45be996ce556fbc`.
Không dựng lại ver11 bằng script cũ: việc đó có thể khôi phục Abstract tiếng
Anh mà tác giả đã xóa. Hồ sơ dựng lịch sử trong thư mục ver11 được giữ nguyên
như snapshot; hash Word thực tế mới là nguồn cho lần hiệu đính này.

[baseline.json](baseline.json) khóa 109 tệp đầu vào được bảo vệ;
[provenance.json](provenance.json) ghi hash nguồn dựng và đầu ra.
SHA-256 Word ver12 đã kiểm tra:
`91b5ef3caeee63c9ac5cc5b27dd8d3ef1687a47fb19e8b83bd1ea9c6acc30b82`.

Nguồn biên tập là [introduction.md](introduction.md),
[front_conclusion.md](front_conclusion.md), [insertions.md](insertions.md),
[references.json](references.json) và
[reference_decisions.json](reference_decisions.json).
[citation_map.json](citation_map.json) ánh xạ số dẫn theo lần xuất hiện;
[revision_log.json](revision_log.json) ghi đúng hai đoạn thêm.
[manuscript_numbered.md](manuscript_numbered.md) chỉ là bản trích xuất đọc nhanh,
**không phải nguồn tái tạo công thức/hình/bảng**.
[artifact.md](artifact.md) ghi hợp đồng bảo toàn mẫu trình bày.

Builder lấy bảng, hình và OMML trực tiếp từ Word nguồn. Mọi phần ZIP ngoài
`word/document.xml` được giữ nguyên byte; bảng, ô, drawing, OMML, section và
khối tên bài được so XML. Kiểm tra văn bản II/III quy số citation về khóa nguồn
trước khi so sánh, không dùng sự giống nhau của số trang để chứng minh bảo toàn.

## Dựng và kiểm tra lại

Chạy từ root workspace. Dùng Python của Documents runtime, không dùng Python
hệ thống để dựng DOCX. Hai script mới chỉ ghi trong thư mục ver12:
[builder](../../../tools/build_journal_full_v12.py) và
[audit](../../../tools/audit_journal_full_v12.py).

```bash
PSTMO_DOC_PY=/home/linh-pham/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python
PYTHONNOUSERSITE=1 "$PSTMO_DOC_PY" tools/build_journal_full_v12.py --check
PYTHONNOUSERSITE=1 "$PSTMO_DOC_PY" tools/audit_journal_full_v12.py
```

Lệnh `--check` không có render chỉ kiểm tra cấu trúc và ghi trạng thái
`structural_pass_visual_pending`; muốn khôi phục trạng thái `passed` phải thực
hiện kiểm tra render tương ứng ở dưới. Audit chỉ so hash, số liệu đã lưu và
đổi đơn vị/định dạng số, không chạy lại bootstrap, Wilcoxon, Holm hay McNemar.

Khi chủ động sửa các nguồn biên tập, dựng lại bằng:

```bash
PYTHONNOUSERSITE=1 "$PSTMO_DOC_PY" tools/build_journal_full_v12.py --replace-generated
```

Chỉ dùng cờ này cho bản đã sinh: builder từ chối ghi đè nếu Word đã bị sửa thủ
công so với hash ghi trong provenance. Không chạy lại `--init`. Sau khi dựng
lại phải audit và render lại, không tái sử dụng kết luận QA của bản cũ.

### Kiểm tra trình bày

Môi trường đã dùng là Documents renderer với LibreOffice 25.2 đi kèm runtime.
Font Times New Roman nằm tại
`/home/linh-pham/.local/share/fonts/pstmo-times`; cấu hình được lưu trong
[render_fonts.conf](render_fonts.conf). Trên máy khác phải cung cấp font tương
ứng và điều chỉnh đường dẫn, không âm thầm thay font. Công thức/ký hiệu có thể
dùng font bổ sung; danh sách font thực tế nằm trong `quality_checks.json`.

```bash
PSTMO_QA_DIR=$(mktemp -d /tmp/pstmo-ver12-qa.XXXXXX)
PSTMO_DOC_SKILL=/home/linh-pham/.codex/plugins/cache/openai-primary-runtime/documents/26.1020.11127/skills/documents
FONTCONFIG_FILE="$PWD/Viet_bao/journal/ver12/render_fonts.conf" PYTHONNOUSERSITE=1 \
  "$PSTMO_DOC_PY" "$PSTMO_DOC_SKILL/render_docx.py" \
  Viet_bao/journal/ver12/PSTMO_JOURNAL_VER12_FULL_MANUSCRIPT.docx \
  --output_dir "$PSTMO_QA_DIR/render" --emit_pdf
```

Mở **từng PNG ở độ phân giải gốc**, kiểm tra cột, caption, bảng, ký hiệu và công
thức. `visual_review.json` là chứng nhận kiểm tra thủ công gắn với hash Word,
PDF và từng PNG; sau render mới phải đối chiếu/cập nhật chứng nhận đúng với
những trang thực sự đã xem, không tự động đặt `accepted=true`.

Kiểm tra render cần PyMuPDF. Lần này dùng venv tạm dựa trên bundled Python với
`PyMuPDF==1.26.7`; không sửa môi trường Python của repo:

```bash
"$PSTMO_DOC_PY" -m venv --system-site-packages "$PSTMO_QA_DIR/env"
"$PSTMO_QA_DIR/env/bin/python" -m pip install PyMuPDF==1.26.7
PYTHONNOUSERSITE=1 "$PSTMO_QA_DIR/env/bin/python" \
  tools/build_journal_full_v12.py --check --render-dir "$PSTMO_QA_DIR/render"
```

Ảnh/PDF là QA nội bộ trong thư mục tạm, không phải bản giao riêng và có thể
không còn sau phiên làm việc. Hash cùng ghi chú kiểm tra được lưu bền trong
ver12. Ngày truy cập tài liệu 23-10 phản ánh ngày đối chiếu nguồn; ngày hoàn
tất QA là 24-10. Kiểm tra này không thay thế việc mở trên Microsoft Word cụ
thể của tác giả hoặc kiểm tra template của tạp chí đích.

## Kiểm tra repository và tác động tài liệu

```bash
python3 tools/check_repository_docs.py
git diff --check
```

Đã bổ sung README này và mục ver12 tại [README root](../../../README.md), đồng
thời sửa mô tả Abstract của ver11 tại danh mục root cho đúng Word thực tế.
Không thay cấu trúc runtime, chính sách dữ liệu, lệnh ROS hoặc pipeline mặc
định; vì vậy không cần đổi AGENTS, docs/agent hay results/README.
Không chạy build/test ROS vì không sửa mã ROS; không commit và không đụng
thay đổi `.vscode/` có sẵn của tác giả.
