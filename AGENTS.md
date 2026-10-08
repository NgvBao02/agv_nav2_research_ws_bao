# Quy ước làm việc trong repository

## Phạm vi

Đây là workspace ROS 2 Jazzy/Nav2/Gazebo cho PSTMO. Mã nguồn chạy nằm chủ yếu
trong `src/`; `docs/`, `results/`, `REFERENCES/` và các thư mục bài báo chứa
nhiều artifact lớn hoặc snapshot lịch sử.

## Đọc context theo nhu cầu

Mục tiêu là giảm context không liên quan, không phải đọc ít bằng mọi giá. Bộ
định tuyến dưới đây chỉ là điểm bắt đầu; không phải giới hạn phạm vi điều tra.
Nếu chưa đủ bằng chứng để hiểu luồng chạy hoặc ảnh hưởng của thay đổi, phải mở
rộng context trước khi sửa.

Không quét hoặc đọc toàn bộ repo trước mỗi task. Bắt đầu bằng tài liệu phù hợp:

- Cần định vị package hoặc nguồn sự thật: đọc `docs/agent/REPO_MAP.md`.
- Cần build, test, launch hoặc benchmark: đọc `docs/agent/COMMANDS.md`.
- Cần dùng, tạo hoặc diễn giải dataset/artifact: đọc
  `docs/agent/DATA_POLICY.md` và `results/README.md`.
- Sửa lõi PSTMO: ưu tiên `src/adaptive_pivot_g2/`.
- Sửa plugin/safety/config Nav2: ưu tiên `src/adaptive_pivot_g2_nav2/` và
  `src/vacuum_robot_gazebo/config/nav2_params.yaml`.
- Sửa benchmark: ưu tiên `src/adaptive_pivot_g2_benchmark/` và đúng scenario
  YAML được yêu cầu.
- Sửa RViz: ưu tiên `src/adaptive_pivot_g2_rviz/` và
  `src/vacuum_robot_gazebo/rviz/`.
- Sửa robot/mô phỏng: ưu tiên `src/vacuum_robot_gazebo/`.
- Sửa báo cáo: đọc README trong đúng thư mục báo cáo và script tạo báo cáo
  tương ứng; không mặc định đọc mọi PDF/DOCX.

### Context tối thiểu trước khi sửa code

Với file sẽ thay đổi, phải kiểm tra đủ các lớp có liên quan:

1. File đích và header/interface trực tiếp của nó.
2. Call site, import, plugin registration hoặc nơi tiêu thụ cấu hình; dùng
   `rg` trên `src/` để tìm, rồi chỉ mở kết quả liên quan.
3. Test gần nhất và build/package manifest của package bị ảnh hưởng.
4. Cấu hình runtime và README gần nhất nếu hành vi công khai có thể đổi.
5. `git status`/`git diff` để không bỏ qua thay đổi sẵn có của người dùng.

Phải mở rộng phạm vi ngay khi gặp một trong các tín hiệu: symbol hoặc tham số
không rõ nguồn; API đi qua nhiều package; ID method/planner/scenario xuất hiện ở
nhiều nơi; test/build lỗi ngoài file đích; source, config và tài liệu mâu thuẫn;
hoặc thay đổi ảnh hưởng dữ liệu/báo cáo đã sinh. Với refactor kiến trúc hay thay
đổi giao diện liên package, đọc toàn bộ chuỗi producer → consumer và test tích
hợp liên quan trước khi kết luận.

Không đọc đệ quy `results/`, `docs/`, file PDF, DOCX, STEP, NPZ, log hoặc JSON
lớn theo mặc định. Khi các artifact này có khả năng quyết định câu trả lời, phải
đọc chúng có mục tiêu qua manifest, `rg`, truy vấn trường JSON hoặc script tổng
hợp; không được bỏ qua chỉ để tiết kiệm token. Tránh in output dài không liên
quan vào context.

## Hợp đồng đồng bộ tài liệu — bắt buộc

README và AGENTS là một phần của definition of done, không phải việc bổ sung
sau. Với mọi thay đổi trong repo:

1. Trước khi kết thúc, đánh giá thay đổi có ảnh hưởng tới hướng dẫn sử dụng,
   cấu trúc repo, hành vi, tham số, lệnh, dữ liệu, artifact hoặc quy trình dành
   cho agent hay không.
2. Nếu có ảnh hưởng, cập nhật README/AGENTS/tài liệu trong `docs/agent/` ngay
   trong cùng change/commit; không để tài liệu mô tả trạng thái cũ.
3. Cập nhật `README.md` ở root khi thay đổi cài đặt, package, pipeline mặc định,
   launch, planner/smoother, lệnh chính hoặc danh mục tài liệu.
4. Cập nhật README gần nhất của thư mục khi thay đổi dataset, báo cáo, artifact,
   cách tái lập hoặc giới hạn diễn giải của thư mục đó.
5. Cập nhật `AGENTS.md` và tài liệu `docs/agent/` khi thay đổi cấu trúc, nguồn sự
   thật, quy trình làm việc, lệnh chuẩn, chính sách dữ liệu hoặc quy tắc agent.
6. Khi tạo hoặc sửa bất kỳ README/AGENTS nào, kiểm tra các link liên quan và
   đối chiếu thông tin trùng lặp ở root README, `results/README.md` và
   `docs/agent/`; sửa tất cả chỗ bị tác động.
7. Không chạm cơ học vào README không liên quan chỉ để đổi ngày hoặc tạo diff.
   Nếu không cần cập nhật tài liệu, ghi rõ `Documentation impact: none` trong
   phần bàn giao.
8. Chạy `python3 tools/check_repository_docs.py` sau mọi thay đổi README/AGENTS.

## Thay đổi và kiểm chứng

- Giữ nguyên thay đổi sẵn có của người dùng; không sửa/xóa artifact ngoài phạm
  vi task.
- Ưu tiên test package bị ảnh hưởng. Chỉ chạy toàn workspace khi thay đổi giao
  diện liên package, cấu hình tích hợp hoặc khi được yêu cầu.
- Thay đổi chỉ liên quan Markdown không cần chạy toàn bộ ROS test; phải chạy
  trình kiểm tra tài liệu và `git diff --check`.
- Không ghi đè dataset cũ. Kết quả mới phải dùng thư mục mới và tuân theo
  `docs/agent/DATA_POLICY.md`.
- Khi bàn giao, nêu rõ mã nguồn, tài liệu và kiểm tra nào đã thay đổi/chạy.
