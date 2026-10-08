# Chính sách dataset và artifact

## Nguyên tắc

- `results/` chứa nhiều cohort khác cấu hình; tên `current` hoặc `final` chỉ có
  nghĩa tại thời điểm tạo.
- Source/config hiện tại là nguồn sự thật cho hành vi đang chạy. Dataset là
  bằng chứng của đúng commit/config/hash đã ghi cùng nó.
- Chỉ so sánh ghép cặp khi planner, scenario, repetition, protocol và Raw path
  hash tương thích.
- Không loại, sửa hoặc chạy lại một trial thuật toán chỉ để thay kết quả xấu.
  Retry hạ tầng phải được ghi riêng.
- Không ghi đè dataset hoặc artifact lịch sử.

## Trình tự đọc tối thiểu

1. Đọc `results/README.md` để xác định cohort.
2. Đọc README của cohort nếu có.
3. Đọc aggregate/manifest/provenance trước dữ liệu từng trial.
4. Chỉ mở JSON/log/CSV chi tiết cần cho câu hỏi đang xử lý.
5. Với báo cáo, đối chiếu script tạo báo cáo và hash nguồn nếu kết luận phụ
   thuộc phiên bản.

## Nguồn báo cáo chính hiện có

- Hình học PSTMO hiện hành: `docs/pstmo_bao_cao_toan_dien_assets/`.
- Ma trận thực thi 175 lượt: `results/pstmo_execution_full_20260803/`.
- Năm tuyến kho giao cắt: `docs/warehouse_cross_aisles_5_routes/`.
- Audit mô hình 3D: `docs/robot_3d_report/`.

Các nguồn trên vẫn là snapshot. Nếu source/config thay đổi sau khi sinh dữ
liệu, tài liệu phải nói rõ rằng kết quả chưa được chạy lại.

## Tạo experiment mới

- Dùng thư mục mới dạng `results/<experiment>_<YYYYMMDD>/` hoặc tên nghiên cứu
  có ngày tương đương.
- Lưu scenario, method/planner, repetition, commit hoặc source hash, config
  hash, Raw path hash, seed nếu có và thông tin môi trường.
- Giữ summary nhỏ ở cấp experiment; nén lossless trace/log lớn khi phù hợp.
- Thêm README khi dataset được dùng để báo cáo, có protocol riêng, chứa failure
  quan trọng hoặc dễ bị nhầm với cohort khác.
- Cập nhật `results/README.md` khi experiment trở thành nguồn chính, thay thế
  nguồn cũ hoặc cần được phân loại là snapshot/ablation/pilot.

## Artifact lớn

PDF, DOCX, STEP, NPZ, ảnh, log và trace không được đọc hàng loạt. Dùng manifest,
SHA-256, metadata hoặc bản tổng hợp trước. Không chuyển artifact sang Git LFS,
xóa lịch sử Git hoặc tách repository nếu chưa có yêu cầu và kế hoạch migration
rõ ràng; các thao tác đó ảnh hưởng cách clone và khả năng tái lập.

## Đồng bộ README/AGENTS

Thay đổi chính sách, nguồn báo cáo chính, cách đặt tên hoặc giao thức experiment
phải cập nhật đồng thời tệp này, `results/README.md` và các README cohort bị ảnh
hưởng. Thay đổi vị trí artifact phải cập nhật mọi link liên quan và chạy
`python3 tools/check_repository_docs.py`.
