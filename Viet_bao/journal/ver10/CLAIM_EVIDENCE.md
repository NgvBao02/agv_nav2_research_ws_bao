# Đối chiếu khoa học của Abstract Introduction và Conclusion

## Mạch lập luận

Mở từ nhu cầu chuyển hướng và tiếp cận tư thế đích trong hành lang kho;
phân biệt hình học đường với chuyển động do bộ điều khiển thực hiện. Tổng
quan đi từ biểu diễn đường cong, tối ưu đường, các nghiên cứu Bézier gần
nhất đến tối ưu quỹ đạo có biến thời gian. Bài toán của PSTMO được đặt ở
lựa chọn dạng chuyển tiếp và phối hợp sử dụng cạnh chung. Kết luận trả
lời hiệu quả thực thi có điều kiện, không chuyển các tiền đề thiết kế thành
chứng minh nhân quả cho quy hoạch động.

## Khẳng định và căn cứ

| Khẳng định trong bản viết | Căn cứ có thể kiểm tra | Ranh giới diễn giải |
| --- | --- | --- |
| Đường trơn không tự xác lập chất lượng thực thi | Nguồn R006, R009, R051, R054; ver9 III-B–G | Không nói mọi thuật toán cũ chỉ quan tâm hình học |
| Bézier bậc năm và G² đã có trước | R031 trang PDF 1, R007 trang 1/6 | Không dùng chúng riêng lẻ làm claim novelty |
| Bézier và thời gian trong kho đã có trước | R010 PDF trang 3–5, mục 2.2, 3, 4.1 và 5 | Ví dụ tối ưu κ²/phạt hành lang, không mô tả sai là trực tiếp tối ưu thời gian ở bước hình dạng |
| DP cùng Bézier có tiền lệ | RCHOI, Abstract của nhà xuất bản | Không tuyên bố DP là thuật toán mới hoặc đây là kết hợp đầu tiên |
| Tối ưu trạng thái chuyển động cho robot vi sai | R063 bản v3, PDF trang 1, 3–4 | Không tuyên bố chưa ai xét quay tại chỗ/vận tốc thấp |
| Đóng góp PSTMO là tổ chức lựa chọn chuỗi | ver9 II-D–E, source_validation.json ver9 | Tối ưu trên ứng viên được giữ, không tối ưu liên tục toàn cục |
| Hình bao và G² có phạm vi rõ | ver9 II-B–C, II-E | Kiểm tra rời rạc; G² ở mối nối đã xây dựng, không bao gồm pivot |
| Hai bộ thử nghiệm độc lập về vai trò | ver9 III-A; README cohort lặp | Ma trận 125 mô tả, cohort 250 kiểm chứng lặp; không gộp mẫu |
| Hiệu quả thời gian/RMSE/độ cong thực thi | table_values.json ver9, bảng primary; cohort analysis/aggregate.json | Δ là trung bình đều 25 nhóm, 118 cặp cùng đạt |
| Đánh đổi khoảng hở/sai số cuối/thời lượng | table_values.json ver9, bảng secondary | Không dùng chỉ số phụ làm claim confirmatory, không gọi thời lượng là CPU |
| 121/125 so với 122/125 lượt đạt | README cohort; ver9 III-H–I | Không chứng minh hơn, tương đương hoặc không kém hơn |
| Cần ablation và robot thực | ver9 III-I, giới hạn cohort | Đây là việc tương lai, không phải thí nghiệm đã có |

Các đối chiếu mã nguồn đã được ghi trong source_validation.json của ver9;
bản này sử dụng mô tả ver9 cùng hash Word giữ nguyên, không sửa mã runtime.
Không tự dựng kết quả thực nghiệm từ ví dụ trong tài liệu tham khảo.

## Kiểm tra số liệu

Hiệu PSTMO trừ Constrained, theo bảng đã làm tròn của ver9:

- Thời gian: −2,857 s, CI [−3,716; −2,039] s.
- RMSE: −2,985 mm, CI [−4,487; −1,506] mm.
- Tích phân bình phương độ cong quỹ đạo thực thi: −1,038 m⁻¹.
- Khoảng hở đường tham chiếu: −35,161 mm.
- Sai số vị trí cuối: +3,380 mm.
- Thời lượng hậu xử lý: +71,220 ms.

Không dùng phần trăm suy từ hai trung bình gộp. CI theo nhóm không được
gọi là CI đã hiệu chỉnh Holm. Kiểm định Wilcoxon không được gọi là kiểm
định trực tiếp trung bình Δ. Chỉ ba chỉ số chính có hiệu chỉnh Holm.
Mốc 118 là số cặp cùng đạt, không phải 118 đầu vào độc lập. Các giới hạn
chọn mẫu và phụ thuộc còn lại được ghi trong Conclusion.

## Rà soát tài liệu

Danh mục nguồn đã chọn và các vị trí đọc nằm trong references.json. Đây là
rà soát có mục tiêu, không phải systematic review hay chứng minh vắng mặt
mọi công trình tương tự. Các nguồn chỉ được xem qua metadata, nguồn trùng
vai trò hoặc không có đoạn chứng cứ được đọc không được đưa vào References.
Nguồn 050 không trích vì bản trích xuất chữ gần như rỗng; cần OCR/xem ảnh
nếu dùng về sau, không suy nội dung từ tên tệp.

Điều chỉnh metadata được ghi tại từng mục: Smac dùng v2 năm 2025 dù tên
tệp mang 2024; RPP đối chiếu bản xuất bản Springer; bài Piece-Wise Bezier
có venue ETFA và DOI trên arXiv; Zhang dùng đúng v3 đã đọc thay vì điền
trang/volume chưa xác minh; Piazzi bỏ issue mâu thuẫn giữa các bản.
Nav2 dùng tài liệu Jazzy, không dùng snapshot Rolling làm bằng chứng
phiên bản thực nghiệm. Không sửa ngân hàng REFERENCES trong nhiệm vụ này.

## Những việc còn lại trước khi nộp

1. Ghép với II–III do giảng viên đang sửa; rà lại các mô tả và số liệu nếu
   phần đó thay đổi, đặc biệt các cấu hình đối chứng và định nghĩa metric.
2. Bổ sung nguồn có vai trò rõ cho toàn bài. Hiện 21 nguồn ở đây cộng bảy
   nguồn ver9, trừ hai mục trùng, bằng 26 nguồn: chưa đáp ứng mốc 50–100.
3. Chọn tạp chí và kiểm tra template, ngôn ngữ, số từ Abstract, kiểu citation.
   Nếu yêu cầu tiếng Anh, dịch và hiệu đính toàn bộ Abstract theo bản cuối.
4. Không tăng mức khẳng định về đóng góp độc lập của DP khi chưa có ablation;
   không cam kết an toàn liên tục, tiết kiệm điện, giảm rung tải hoặc tăng
   năng suất kho từ dữ liệu hiện có.
5. Bổ sung thông tin tác giả, đơn vị, funding/data availability và các mục
   bắt buộc theo tạp chí bằng thông tin thật do nhóm cung cấp.
