# Quy cách ghép ver11

Mẫu là Word ver9 chưa sửa; khối tên bài toàn chiều ngang và Title style
lấy nguyên từ ver10. Hợp đồng authoring được lập trước khi dựng tại
`/tmp/pstmo-v11.qyc5Nf/artifact.md`.

A4 11906 × 16838 twip; lề hai bên 907, trên/dưới 1247. Thân hai cột,
khoảng cột 360; Times New Roman 10 pt, line 254, after 80, first-line 288.
Tên Việt 14 pt, Anh 11 pt. Không trang bìa, header/footer hay tác giả tự tạo.
Giữ nguyên các style bảng, hình, chú thích và công thức bản nguồn.

Ghép ở mức XML trên package ver9 để bảo toàn relationship, media và Office
Math. Không dùng helper sao chép drawing mà bỏ rId. Chỉ sửa document.xml,
styles.xml và core.xml. Thân II–III được giữ nguyên ngoài số trích dẫn và
các đoạn chèn ghi trong revision_log.json. Thêm hai tóm tắt, Introduction,
Conclusion và bibliography chung; bỏ danh mục riêng cũ của II–III.

Tài liệu tham khảo 8 pt, hanging indent và keepLines. Khác biệt có chủ đích:
căn trái các mục tham khảo mới để tránh khoảng trắng quá lớn khi có URL
dài; không thu font/lề để ép trang. OMML và các chỉ số trong nguồn được
giữ nguyên, kể cả renderer dùng font dự phòng cho ký hiệu.

QA bắt buộc: hash nguồn, nguyên XML đối tượng khoa học, số liệu bảng,
nguồn–vị trí dẫn, render và xem mọi PNG của chính bản cuối. PDF/PNG chỉ là
artifact kiểm tra, không bàn giao. Đây không phải xác nhận mẫu của tạp chí đích.
