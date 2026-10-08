# Context dành cho coding agent

Thư mục này là bộ định tuyến context ngắn cho Codex và các coding agent khác.
Nó không thay thế mã nguồn, test, cấu hình runtime, tài liệu khoa học, README
chính hoặc dữ liệu gốc; đây là điểm bắt đầu khám phá, không phải trần context.

| Tệp | Chỉ đọc khi |
| --- | --- |
| [`REPO_MAP.md`](REPO_MAP.md) | Cần biết package, entry point hoặc nguồn sự thật của một phần hệ thống |
| [`COMMANDS.md`](COMMANDS.md) | Cần build, test, launch, benchmark hoặc kiểm tra tài liệu |
| [`DATA_POLICY.md`](DATA_POLICY.md) | Cần đọc, tạo, so sánh, di chuyển hoặc trích dẫn dataset/artifact |

Quy tắc bắt buộc cho toàn repo nằm trong [`AGENTS.md`](../../AGENTS.md). Không
đọc cả ba tệp theo thói quen; chọn đúng tệp theo task để giảm context. Sau đó
vẫn phải đọc file đích, dependency/call site, test và cấu hình liên quan. Nếu
source, config, test và tài liệu không khớp, mở rộng điều tra và ưu tiên bằng
chứng runtime/source thay vì tin bản đồ này.

## Đồng bộ

Khi cấu trúc, lệnh chuẩn hoặc chính sách dữ liệu thay đổi, cập nhật tệp tương
ứng trong thư mục này cùng với README/AGENTS bị ảnh hưởng. Sau đó chạy:

```bash
python3 tools/check_repository_docs.py
git diff --check
```
