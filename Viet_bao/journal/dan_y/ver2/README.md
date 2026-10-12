# Dàn ý journal PSTMO — phiên bản 2

Thư mục này chứa dàn ý publication-oriented được dựng từ dữ liệu đã audit,
không phải bản thảo sẵn sàng nộp và không phải cam kết bài sẽ được nhận.

- `DAN_Y_JOURNAL_PSTMO_Q2_VER2.docx`: dàn ý 12 trang đã render và kiểm tra
  trực quan toàn bộ.
- Nguồn bố cục ban đầu:
  `../ver1/DAN_Y_JOURNAL_PSTMO_Q2_CHUYEN_SAU.docx`; tệp ver1 không bị ghi đè.
- Script tái tạo: `../../../../tools/revise_journal_outline_q2.py`.
- Nguồn thống kê:
  `../../../../results/pstmo_journal_repeatability_20261011/analysis/aggregate.json`.
- Báo cáo kỹ thuật đi kèm: `../../../../docs/PSTMO_KHO_GIAO_CAT_5_QUY_DAO.pdf`.

Dàn ý dùng 25 khối route–planner làm đơn vị suy luận cho ba outcome liên tục
chính, giữ toàn bộ bảy lượt không đạt trong mẫu số, và tách success analysis
khỏi metric có điều kiện trên trial thành công. Những phần chưa có bằng chứng
được đánh dấu rõ: ablation nhân quả trên cùng commit, multi-seed/noise,
dynamic obstacles, payload/energy và robot thật.

Tái tạo bằng:

```bash
python3 tools/revise_journal_outline_q2.py
```

Kiểm tra phát hành ngày 12-10-2026: DOCX mở hợp lệ, render bằng LibreOffice ra
12 trang Letter; toàn bộ trang đã được kiểm tra về tràn chữ, gãy bảng, header,
footer và khả năng đọc.
