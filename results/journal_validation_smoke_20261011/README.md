# Pilot kiểm tra pipeline journal

Đây là pilot ba lượt dùng để xác nhận runner, recorder và ghép cặp Raw trước
khi chạy cohort lặp chính. Pilot không được gộp vào thống kê của bài journal.

## Phạm vi

- Scenario: `cross_aisle_transfer` (R01).
- Planner: `ThetaStar`.
- Phương án: Raw, Constrained và PSTMO; một lượt cho mỗi phương án.
- Cả ba lượt dùng cùng Raw-path SHA-256
  `86c0fec4036278df23f064844c1f0f7af6c0b0c992699d18869cf0c976ddbf72`.
- Cả ba lượt đạt tiêu chí thực thi.

| Phương án | Thời gian (s) | Tracking RMSE (m) | Planned Eκ (1/m) |
| --- | ---: | ---: | ---: |
| Raw | 43.359 | 0.0138665 | 34.9876 |
| Constrained | 41.724 | 0.0191758 | 13.4857 |
| PSTMO | 38.601 | 0.0174431 | 1.26986 |

Các số trên chỉ là smoke test một lượt. Chúng không phải estimate thống kê,
không thay thế 250 trial trong
[`../pstmo_journal_repeatability_20261011/`](../pstmo_journal_repeatability_20261011/)
và không được chọn làm case chính chỉ vì kết quả thuận lợi.
