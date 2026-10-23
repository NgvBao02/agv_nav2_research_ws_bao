# Đối chiếu ver11 và ver12

Nguồn so sánh là Word ver11 thực tế sau chỉnh sửa của tác giả, không phải tái dựng từ nguồn Markdown cũ. Nội dung gốc trong ver11 không bị ghi đè.

| Thành phần | Ver11 thực tế | Ver12 | Tính chất thay đổi |
|---|---|---|---|
| Tên bài | Việt và Anh | Giữ nguyên khối XML | Không đổi |
| Phần tóm tắt | Chỉ tiếng Việt sau khi tác giả xóa tiếng Anh | Chỉ tiếng Việt; nêu cạnh chung, lấy mẫu và đủ hướng đánh đổi | Hiệu đính nội dung |
| I mở đầu | Logistics và tổng quan rộng | Vấn đề chuyển hướng trên cạnh chung trước, bối cảnh sau | Đổi tổ chức lập luận |
| I-A | Lịch sử nhiều họ tìm kiếm | Tầng hậu xử lý và planner liên quan khảo sát | Tập trung phạm vi |
| I-B | Nhiều biểu diễn đường được giới thiệu ngắn | Đối chiếu sâu Piazzi/Brezak; phân biệt ràng buộc không gian | Tăng chiều sâu |
| I-C | Nhiều nhánh tối ưu/điều khiển | Nhấn Blažič, Choi và cấu hình Constrained | Làm rõ khác biệt bài toán |
| I-D | Đã có vấn đề cạnh chung và giới hạn DP | Nối trực tiếp từ đối chiếu nguồn đến câu hỏi nghiên cứu; không tuyên bố ưu tiên tuyệt đối | Củng cố định vị |
| I-E | Hai loại đóng góp | Giữ hai loại, giới hạn rõ ở toàn bộ PSTMO và thiết kế mô phỏng | Làm rõ phạm vi |
| II-A/B/C | Pipeline, Bézier, giới hạn chuyển động và footprint | Nguyên văn trừ số citation | Bảo toàn |
| II-D | Tìm hình dạng, thời gian, J, pivot cost | Giữ tất cả; thêm một đoạn phân biệt ba cấp tiêu chí và thời điểm tính J | Giải thích, không đổi thuật toán |
| II-E | DP, tương thích, tie-break, giới hạn | Giữ tất cả; thêm một đoạn giải thích trạng thái/tiền tố và điểm đi qua | Giải thích, không đổi truy hồi |
| III-A–I | 125 lượt khảo sát và 250 lượt kiểm chứng riêng | Nguyên văn trừ số citation; giữ cả bất lợi và thất bại | Bảo toàn |
| IV | Kết quả chính và giới hạn | Nhấn cấu hình Constrained đã thử; bổ sung hướng kiểm chứng pivot và Simple | Đồng bộ kết luận |
| Tham khảo | 80 | 54, đánh lại số theo lần xuất hiện | Bỏ 26 nguồn có lý do riêng |
| Bảng / ô | 23 / 696 | 23 / 696, XML giống hệt | Bảo toàn |
| Hình | 18 | 18, drawing/media giống hệt | Bảo toàn |
| Công thức | 9 số thứ tự, 12 đối tượng OMML | Giữ nguyên XML và số thứ tự | Bảo toàn |
| Định dạng | IEEE A4 hai cột, Times New Roman | Giữ styles, lề, cột và khối tên | Không đổi |
| Kết quả mới | Không áp dụng | Không chạy thí nghiệm hay tái phân tích | Không phát sinh |

## Các thay đổi có thể kiểm tra trực tiếp

Trước II-D đã có câu “thời gian không phải một hạng bù trừ trong J”. Đoạn bổ sung diễn giải rõ Eκ chọn hình dạng, thời gian lọc tập, J chọn chuỗi; đồng thời lưu ý J được tính khi đánh giá ứng viên chứ không bắt buộc đợi sau bộ lọc.

Trước II-E đã có truy hồi và quan hệ tương thích. Đoạn bổ sung giải thích vì sao trạng thái cuối quyết định lựa chọn kế tiếp trong đồ thị, đồng thời nói rõ DP không mang biến vận tốc toàn đường và điểm đi qua dưới 5° không tạo quay tại chỗ vật lý.

Trước I-C đã nói đúng rằng ví dụ Blažič dùng tích phân bình phương độ cong và phạt ra ngoài hành lang. Ver12 **không sửa nhận định đúng đó thành một lỗi của ver11**; nó mở rộng đối chiếu về biến quyết định, phối hợp chuỗi, kích thước xe, gia tốc và tầng tích hợp.

## Những giá trị giữ nguyên

Chênh lệch PSTMO trừ Constrained trong phân tích theo 25 nhóm:

| Chỉ số | Δ giữ nguyên | Ý nghĩa |
|---|---:|---|
| Thời gian thực thi | −2,857 s | Lợi ích trung bình có điều kiện |
| RMSE bám đường | −2,985 mm | Không giảm ở mọi đầu vào |
| Tích phân độ cong bình phương thực thi | −1,038 m⁻¹ | Không phải điện năng |
| Khoảng hở tham chiếu | −35,161 mm | Đánh đổi bất lợi |
| Sai số vị trí cuối | +3,380 mm | Đánh đổi bất lợi |
| Thời lượng hậu xử lý | +71,220 ms | Đồng hồ mô phỏng, không phải CPU |

Giữ toàn bộ CI và p-value, 118 cặp cùng đạt, tỷ lệ 121/125 và 122/125 của hai phương án trong bộ lặp. Giữ chín dòng không đạt ở Bảng XXIII (hai khảo sát, bảy bộ lặp), không gộp cohort hoặc đổi dung sai.

## Truy vết thay đổi

- [Báo cáo khoa học](SCIENTIFIC_REVISION_REPORT.md): trước/sau, lý do, bằng chứng và mức ảnh hưởng theo từng thay đổi.
- [Nhật ký hai đoạn thêm](revision_log.json): vị trí neo và nguyên văn.
- [Nguồn Introduction](introduction.md), [Tóm tắt và Kết luận](front_conclusion.md), [đoạn bổ sung](insertions.md): các phần sửa được bằng văn bản.
- [Quyết định tài liệu](REFERENCE_COVERAGE.md): tất cả nguồn giữ và lý do từng nguồn bỏ.
- [Kiểm tra cấu trúc](quality_checks.json), [đối chiếu số liệu](evidence_audit.json), [kiểm tra trang](visual_review.json): bằng chứng bảo toàn và trình bày.

Builder không dùng Markdown để tái tạo bảng, hình hoặc phương trình; những đối tượng đó lấy trực tiếp từ Word ver11 đã khóa hash. Nhờ vậy, sự khác nhau về phần trình bày không bị lẫn với tái sinh dữ liệu khoa học.
