@before E. Lựa chọn chuỗi chuyển tiếp và tạo đường tham chiếu

Như vậy, ba tiêu chí được sử dụng ở những cấp khác nhau: Eκ chọn hình dạng cho từng chiều dài cắt; thời gian dự đoán xác định tập chuyển tiếp được giữ; J cung cấp chi phí cho lựa chọn chuỗi. Trong triển khai, J được tính khi đánh giá ứng viên trên cửa sổ chung, nhưng chỉ ứng viên qua bộ lọc mới tham gia so sánh ở bước lựa chọn chuỗi. DP không quay lại tìm các giá trị α đã bị loại và không tối ưu trực tiếp tổng thời gian chuyển động.

@before Sau khi truy vết,

Trạng thái tại một góc xác định phương án và chiều dài cắt, không chứa biến vận tốc dọc toàn đường. Khi trạng thái cuối của một chuỗi tiền tố đã cố định, các lựa chọn tương thích ở góc kế tiếp phụ thuộc trạng thái đó qua cạnh chung; do chi phí cộng, chỉ cần giữ tiền tố có chi phí thấp nhất cho mỗi trạng thái. Đây là cơ sở của truy hồi (7), đồng thời xác định giới hạn của tính tối ưu. Các đỉnh đi qua dưới ngưỡng 5° tham gia chuỗi như trạng thái cố định có chiều dài cắt và chi phí bằng không; chúng không tạo thao tác quay tại chỗ vật lý.
