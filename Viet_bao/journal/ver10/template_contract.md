# Quy cách bản thảo riêng ver10

Mẫu giữ nguyên tại `../ver9/PSTMO_JOURNAL_VER9_PHAN_II_III.docx`, SHA-256
`4f5743cc02139a23da557da46ee84704f7d5228bb371b55e7b0a19b0b78a53dc`.
Đã render và xem cả 15 trang mẫu trong phiên này. Hợp đồng dựng tạm được
viết trước authoring tại `/tmp/pstmo-v10.hjsixc/artifact.md`.

- A4 dọc 11906 × 16838 twip; lề trên/dưới 1247, hai bên 907 twip.
- Giữ BodyText Times New Roman 10 pt, dòng 254/240, sau đoạn 80 twip,
  thụt đầu 288 twip. Heading1/Heading2 lấy từ mẫu, tắt đánh số tự động để
  không sinh thêm số khi nội dung đã có I/IV hoặc A/B/C.
- Thân hai cột, khoảng cột 360 twip. Mở rộng một section liên tục toàn
  chiều ngang cho tên bài VN/EN mới; không tạo trang bìa hay header/footer.
- Tên VN dùng style Title mới, đen, đậm, 14 pt, căn giữa, hai dòng cân đối;
  tên EN 11 pt. Không đường trang trí. Từ khóa giữ cùng cỡ thân.
- Tài liệu tham khảo dùng ScientificReference có sẵn, 8 pt, thụt treo;
  keepLines giữ mỗi mục liền nhau. Nội dung ký hiệu m⁻¹ dùng run chỉ số trên
  chỉnh sửa được, tránh thiếu glyph của font Times New Roman.
- Toàn bộ slot thân cũ được thay, không giữ bảng/hình/công thức II–III.
  Chỉ document.xml, styles.xml, core.xml được sửa; các ZIP part khác,
  relationship, numbering, theme và opaque XML giữ nguyên byte. Inventory
  hash nguồn/đích của từng part ở provenance.json.
- Không field/SDT/footnote mới; không tác giả hoặc cơ quan tự suy đoán.
- Gate: nguồn ver9 không đổi; citation đủ và liên tục; đúng phần được yêu
  cầu; số liệu đối chiếu bảng nguồn; render, xem từng PNG, gắn hash vào
  visual_review.json. PDF/PNG chỉ để QA, không bàn giao như sản phẩm mới.

Khác biệt bố cục có chủ đích là khối tên bài mới và thay toàn bộ thân bằng
phần mở/kết theo yêu cầu. Những trang bảng/hình của mẫu không được dựng
lại. Không tuyên bố template này đã được xác nhận cho tạp chí đích.
