#!/usr/bin/env python3

"""Generate the theory-complete, image-audited PSTMO condition-only report."""

from __future__ import annotations

import html
import json
import math
import statistics
import subprocess
from pathlib import Path

import generate_pstmo_current_full_report as base


ROOT = base.ROOT
DOCS = base.DOCS
ASSETS = DOCS / "pstmo_no_los_toan_dien_assets"
RVIZ_DIR = ASSETS / "rviz_cases"
GAZEBO_DIR = ASSETS / "gazebo"
FIG_DIR = ASSETS / "figures"
OUTPUT_HTML = DOCS / "BAO_CAO_TOAN_DIEN_PSTMO_TAT_LOS.html"
OUTPUT_CSV = ASSETS / "benchmark_live_35_cases_no_los.csv"
OUTPUT_JSON = ASSETS / "benchmark_live_aggregate_no_los.json"
FAILURE_JSON = ASSETS / "failure_evidence_C30_no_los.json"
FAILURE_LOG = Path(
    "/home/linh-pham/.ros/log/smoother_server_249871_1785664096626.log"
)
NO_LOS_ENV_DESCRIPTION = dict(base.ENV_DESCRIPTION)
NO_LOS_ENV_DESCRIPTION["warehouse_long_aisles"] = (
    "Các hành lang song song dài kiểm tra tích lũy độ cong, nhiều góc liên tiếp "
    "và khả năng giữ corridor planner khi không dùng shortcut LOS."
)


def load_evidence():
    """Load the exact no-LOS live matrix and reject mixed-mode evidence."""
    items = [
        json.loads(path.read_text(encoding="utf-8"))
        for path in RVIZ_DIR.glob("*.json")
    ]
    items.sort(key=base.evidence_order)
    if len(items) != 35:
        raise RuntimeError(f"Expected 35 no-LOS records, found {len(items)}")
    seen = {(item["environment"], item["planner"]) for item in items}
    expected = {
        (environment, planner)
        for environment in base.ENVIRONMENTS
        for planner in base.PLANNERS
    }
    if seen != expected:
        raise RuntimeError(
            f"Evidence matrix mismatch: missing={sorted(expected-seen)} "
            f"extra={sorted(seen-expected)}"
        )
    for index, item in enumerate(items, 1):
        item["case_id"] = f"C{index:02d}"
        diag = item["pstmo_diagnostics"]
        valid = (
            len(item["start"]) == 3
            and len(item["goal"]) == 3
            and item.get("expected_preprocessing") == "condition_only"
            and diag.get("search_mode") == "hierarchical_alpha_two_trim"
            and diag.get("preprocessing_mode") == "condition_only"
            and diag.get("los_executed") is False
            and diag.get("los_runtime_s") == 0
            and diag.get("los_attempted_shortcuts") == 0
            and diag.get("los_accepted_shortcuts") == 0
            and diag.get("pipeline_execution_count") == 1
            and diag.get("final_invariants_verified") is True
            and "pstmo" in item["metrics"]
            and "pstmo" in item["paths"]
        )
        if not valid:
            raise RuntimeError(f"Mixed or invalid no-LOS evidence: {item['case_id']}")
        screenshot = ROOT / item["rviz_screenshot"]
        if not screenshot.is_file():
            raise FileNotFoundError(screenshot)
    return items


def configure_base_paths():
    """Route reusable metric/composite helpers to this report's asset tree."""
    base.ASSETS = ASSETS
    base.RVIZ_DIR = RVIZ_DIR
    base.GAZEBO_DIR = GAZEBO_DIR
    base.FIG_DIR = FIG_DIR
    base.OUTPUT_CSV = OUTPUT_CSV
    base.OUTPUT_JSON = OUTPUT_JSON
    base.FAILURE_JSON = FAILURE_JSON
    base.FAILURE_LOG = FAILURE_LOG


def algorithm_figures(items):
    """Draw explanatory figures; live evidence is kept visually distinct."""
    base.flow_figure(
        FIG_DIR / "figure_01_pipeline_no_los.png",
        "PSTMO độc lập hiện tại — một pipeline, LOS tắt thật",
        [
            "Planner path",
            "Condition\npolyline",
            "Hai d\nhình học",
            "Tìm α=q/d\nthô–tinh",
            "Footprint +\nđộng học",
            "Time gate\n+ DP",
            "Ghép + invariant\ncuối",
        ],
        colors=[
            "#334155", "#2563eb", "#0f766e", "#16a34a",
            "#ea580c", "#7c3aed", "#be123c",
        ],
        subtitle=(
            "Không gọi prune_line_of_sight; diagnostics bắt buộc "
            "los_executed=false, attempts=0, runtime=0."
        ),
    )
    base.flow_figure(
        FIG_DIR / "figure_02_hard_gates.png",
        "Chuỗi điều kiện loại cứng của một transition",
        [
            "Bézier hữu hạn\n0<α≤0,5",
            "Không đảo dấu κ\nngoài ý muốn",
            "Bánh trong\nkhông quay lùi",
            "Swept-footprint\nkhông va chạm",
            "v, ω, a, aω, ay\nhợp lệ",
            "Không chồng lấn\ntrong DP",
            "Output cuối\nquét lại",
        ],
        colors=[
            "#0284c7", "#0e7490", "#0f766e", "#15803d",
            "#ca8a04", "#c2410c", "#b91c1c",
        ],
        subtitle=(
            "Clearance là số đo hậu kiểm; vật cản lethal/unknown/outside và "
            "giao cắt footprint là điều kiện loại cứng."
        ),
    )

    first = items[0]
    diag = first["pstmo_diagnostics"]
    raw = first["paths"]["raw"]["poses"]
    raw_xy = base.np.asarray([[pose["x"], pose["y"]] for pose in raw])
    conditioned = base.np.asarray(diag["conditioned_polyline"])
    preprocessed = base.np.asarray(diag["preprocessed_polyline"])
    pixels, extent = base.map_image(first["environment"])
    fig, axes = base.plt.subplots(1, 3, figsize=(16, 4.8))
    for axis in axes:
        axis.imshow(
            pixels, cmap="gray", extent=extent, origin="upper", vmin=0, vmax=255
        )
        axis.set_aspect("equal")
        axis.grid(alpha=0.16)
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
    axes[0].plot(raw_xy[:, 0], raw_xy[:, 1], color="#111827", linewidth=1.6)
    axes[0].set_title(f"Raw planner: {len(raw_xy)} pose", fontweight="bold")
    axes[1].plot(
        conditioned[:, 0], conditioned[:, 1], "o-", color="#2563eb", linewidth=2
    )
    axes[1].set_title(
        f"Conditioning: {len(raw_xy)} → {len(conditioned)} neo", fontweight="bold"
    )
    axes[2].plot(
        conditioned[:, 0], conditioned[:, 1], "o--", color="#94a3b8",
        label="conditioned",
    )
    axes[2].plot(
        preprocessed[:, 0], preprocessed[:, 1], "x-", color="#16a34a",
        linewidth=2.2, label="đầu vào transition",
    )
    axes[2].set_title("Không có LOS: hai chuỗi trùng nhau", fontweight="bold")
    axes[2].legend(fontsize=8)
    fig.suptitle(
        "Dữ liệu thật C01: conditioning vẫn chạy, LOS không chạy",
        fontsize=14, fontweight="bold",
    )
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_03_conditioning_actual.png", dpi=170,
        bbox_inches="tight",
    )
    base.plt.close(fig)

    vertex = base.np.array([0.0, 0.0])
    incoming = base.np.array([1.0, 0.0])
    outgoing = base.np.array([0.0, 1.0])
    trim = 1.0
    alpha = 0.32
    control = alpha * trim
    entry = vertex - incoming * trim
    exit_point = vertex + outgoing * trim
    cps = base.np.asarray([
        entry,
        entry + incoming * control,
        entry + incoming * 2.0 * control,
        exit_point - outgoing * 2.0 * control,
        exit_point - outgoing * control,
        exit_point,
    ])
    parameter = base.np.linspace(0.0, 1.0, 401)
    curve = base.np.zeros((len(parameter), 2))
    for index in range(6):
        bernstein = (
            math.comb(5, index)
            * (1.0 - parameter) ** (5 - index)
            * parameter ** index
        )
        curve += bernstein[:, None] * cps[index]
    fig, axes = base.plt.subplots(1, 2, figsize=(13, 5.2))
    axes[0].plot([-1.35, 0.0, 0.0], [0.0, 0.0, 1.35], color="#64748b")
    axes[0].plot(cps[:, 0], cps[:, 1], "o--", color="#f59e0b", label="control polygon")
    axes[0].plot(curve[:, 0], curve[:, 1], color="#16a34a", linewidth=3, label="Bézier bậc 5")
    for index, point in enumerate(cps):
        axes[0].text(point[0] + 0.025, point[1] + 0.025, f"P{index}", fontsize=9)
    axes[0].set_aspect("equal")
    axes[0].grid(alpha=0.2)
    axes[0].legend()
    axes[0].set_title("Hình học chuẩn hóa d=1, α=0,32", fontweight="bold")
    first_derivative = base.np.gradient(curve, parameter, axis=0)
    second_derivative = base.np.gradient(first_derivative, parameter, axis=0)
    numerator = (
        first_derivative[:, 0] * second_derivative[:, 1]
        - first_derivative[:, 1] * second_derivative[:, 0]
    )
    denominator = base.np.maximum(
        (first_derivative[:, 0] ** 2 + first_derivative[:, 1] ** 2) ** 1.5,
        1.0e-12,
    )
    curvature = numerator / denominator
    axes[1].plot(parameter, curvature, color="#7c3aed", linewidth=2.4)
    axes[1].axhline(0.0, color="#64748b", linewidth=0.8)
    axes[1].scatter([0.0, 1.0], [curvature[0], curvature[-1]], color="#dc2626")
    axes[1].set_xlabel("u")
    axes[1].set_ylabel("κ(u) (chuẩn hóa)")
    axes[1].set_title("κ tiến về 0 ở hai đầu — nối G² với đoạn thẳng", fontweight="bold")
    axes[1].grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_04_bezier_g2.png", dpi=170, bbox_inches="tight"
    )
    base.plt.close(fig)

    fig, axis = base.plt.subplots(figsize=(13, 4.8))
    coarse = base.np.asarray([0.1, 0.2, 0.3, 0.4, 0.5])
    recovery = base.np.asarray([0.15, 0.25, 0.35, 0.45])
    fine = base.np.linspace(0.2, 0.4, 11)
    axis.scatter(coarse, base.np.full_like(coarse, 3.0), s=110, color="#2563eb", label="coarse")
    axis.scatter(recovery, base.np.full_like(recovery, 2.0), s=90, color="#dc2626", label="recovery nếu coarse thất bại hết")
    axis.scatter(fine, base.np.full_like(fine, 1.0), s=70, color="#16a34a", label="ví dụ refine winner 0,3")
    for value in coarse:
        axis.text(value, 3.12, f"{value:.1f}", ha="center", fontsize=9)
    axis.annotate(
        "chia [0,2; 0,4] thành 10 khoảng",
        xy=(0.31, 1.0), xytext=(0.34, 1.65),
        arrowprops={"arrowstyle": "->", "color": "#334155"},
    )
    axis.set_xlim(0.06, 0.54)
    axis.set_ylim(0.55, 3.55)
    axis.set_yticks([1, 2, 3], ["Fine", "Recovery", "Coarse"])
    axis.set_xlabel("α=q/d")
    axis.set_title("Tìm hình dạng α theo lưới thô–phục hồi–tinh", fontweight="bold")
    axis.grid(axis="x", alpha=0.2)
    axis.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_05_alpha_search.png", dpi=170, bbox_inches="tight"
    )
    base.plt.close(fig)

    fig, axes = base.plt.subplots(1, 2, figsize=(14, 5.1))
    axes[0].plot([0, 4], [0, 0], color="#475569", linewidth=4)
    axes[0].scatter([0, 2, 4], [0, 0, 0], s=100, color="#111827", zorder=4)
    axes[0].annotate("dᵢ", xy=(2, 0), xytext=(1.25, 0.62), arrowprops={"arrowstyle": "<->"})
    axes[0].annotate("dᵢ₊₁", xy=(2, 0), xytext=(2.75, 0.62), arrowprops={"arrowstyle": "<->"})
    axes[0].text(2, -0.42, "dᵢ+dᵢ₊₁+m ≤ Lᵢ", ha="center", fontsize=14, color="#7c3aed")
    axes[0].set_xlim(-0.3, 4.3)
    axes[0].set_ylim(-0.8, 1.1)
    axes[0].axis("off")
    axes[0].set_title("Ngân sách đoạn dùng chung", fontweight="bold")
    x_positions = [0, 2.6, 5.2, 7.8]
    for column, x_value in enumerate(x_positions):
        axes[1].text(x_value, 3.0, f"Góc {column+1}", ha="center", fontweight="bold")
        for row, label in enumerate(["d_pref", "d_compat", "pivot"]):
            y_value = 2.25 - 0.75 * row
            axes[1].scatter(x_value, y_value, s=170, color=["#16a34a", "#2563eb", "#f59e0b"][row])
            axes[1].text(x_value + 0.18, y_value, label, va="center", fontsize=8)
        if column:
            for left_row in range(3):
                for right_row in range(3):
                    axes[1].plot(
                        [x_positions[column-1] + 0.25, x_value - 0.25],
                        [2.25 - 0.75 * left_row, 2.25 - 0.75 * right_row],
                        color="#94a3b8", linewidth=0.7, alpha=0.45,
                    )
    axes[1].set_xlim(-0.6, 8.8)
    axes[1].set_ylim(0.35, 3.45)
    axes[1].axis("off")
    axes[1].set_title("DP chỉ giữ cạnh tương thích; chi phí cộng dồn", fontweight="bold")
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_06_two_trim_dp.png", dpi=170, bbox_inches="tight"
    )
    base.plt.close(fig)

    fig, axes = base.plt.subplots(1, 2, figsize=(14, 4.8))
    arc = base.np.linspace(0.0, 1.0, 240)
    kappa_demo = 2.2 * base.np.sin(base.np.pi * arc) ** 2
    speed_cap = base.np.minimum.reduce([
        base.np.full_like(arc, 0.30),
        base.np.divide(0.80, base.np.maximum(kappa_demo, 1.0e-8)),
        base.np.sqrt(base.np.divide(0.18, base.np.maximum(kappa_demo, 1.0e-8))),
        base.np.divide(0.36, 1.0 + 0.5 * 0.2548 * kappa_demo),
    ])
    axes[0].plot(arc, kappa_demo, color="#7c3aed", linewidth=2.5, label="|κ|")
    axes[0].set_xlabel("s/L")
    axes[0].set_ylabel("|κ| (chuẩn hóa)")
    axes[0].grid(alpha=0.2)
    axes[0].set_title("Độ cong làm giảm trần vận tốc", fontweight="bold")
    axes[1].plot(arc, speed_cap, color="#16a34a", linewidth=2.5, label="v_cap")
    axes[1].fill_between(arc, 0.0, speed_cap, color="#bbf7d0", alpha=0.55)
    axes[1].set_xlabel("s/L")
    axes[1].set_ylabel("v (m/s)")
    axes[1].set_ylim(0.0, 0.33)
    axes[1].grid(alpha=0.2)
    axes[1].set_title("Trần v từ v, ω, a_y và tốc độ bánh", fontweight="bold")
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_07_kinematic_time_gate.png", dpi=170,
        bbox_inches="tight",
    )
    base.plt.close(fig)

    fig, axis = base.plt.subplots(figsize=(14, 5.4))
    boxes = [
        (0.2, 2.55, 3.8, 1.45, "LOS bật", "Chọn chord xa nhất còn an toàn\n→ có thể bỏ neo trung gian\n→ đường hợp lệ hình học nhưng khó bám với controller"),
        (5.0, 2.55, 3.8, 1.45, "LOS tắt", "Giữ corridor sau conditioning\n→ chỉ thay lân cận góc bằng transition\n→ bảo toàn cấu trúc bám cục bộ"),
    ]
    for x_value, y_value, width, height, title, body in boxes:
        axis.add_patch(base.Rectangle((x_value, y_value), width, height, facecolor="#e0f2fe" if x_value < 1 else "#dcfce7", edgecolor="#334155", linewidth=1.5))
        axis.text(x_value + width/2, y_value + 1.08, title, ha="center", fontweight="bold", fontsize=13)
        axis.text(x_value + width/2, y_value + 0.52, body, ha="center", va="center", fontsize=10)
    axis.annotate("quyết định kiến trúc", xy=(5.0, 3.28), xytext=(4.05, 3.28), arrowprops={"arrowstyle": "->", "linewidth": 2})
    axis.text(4.5, 1.55, "Điều không được suy ra chỉ từ benchmark path", ha="center", fontweight="bold", color="#b91c1c")
    axis.text(4.5, 0.92, "35 ảnh chứng minh output hình học; muốn khẳng định tỷ lệ chạy tới goal phải có benchmark execute=true riêng.", ha="center", fontsize=10)
    axis.set_xlim(0, 9)
    axis.set_ylim(0.45, 4.45)
    axis.axis("off")
    axis.set_title("Vì sao tắt LOS: ưu tiên tính tương thích với controller", fontweight="bold", fontsize=15)
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_08_why_no_los.png", dpi=170, bbox_inches="tight"
    )
    base.plt.close(fig)


def result_figures(items, rows, aggregate):
    case_figures = {}
    for item in items:
        selected = [row for row in rows if row["case_id"] == item["case_id"]]
        case_figures[item["case_id"]] = base.plot_case_composite(item, selected)

    methods = list(base.METHODS)
    paired = aggregate["complete_five_method_cases"]
    fig, axes = base.plt.subplots(2, 2, figsize=(14, 9))
    specs = [
        ("paired_mean_path_length_m", "Chiều dài L (m)"),
        ("paired_mean_max_abs_curvature_1pm", "Kmax (1/m)"),
        ("paired_mean_curvature_energy_1pm", "Eκ (1/m)"),
        ("paired_mean_algorithm_time_s", "Thời gian SmoothPath (ms)"),
    ]
    for axis, (key, title) in zip(axes.ravel(), specs):
        values = [
            aggregate["methods"][method][key] * (1000 if key.endswith("time_s") else 1)
            for method in methods
        ]
        bars = axis.bar(
            [base.METHOD_LABEL[method] for method in methods], values,
            color=[base.METHOD_COLOR[method] for method in methods],
        )
        axis.set_title(title, fontweight="bold")
        axis.grid(axis="y", alpha=0.2)
        axis.tick_params(axis="x", rotation=18)
        for bar, value in zip(bars, values):
            axis.text(
                bar.get_x() + bar.get_width()/2, bar.get_height(),
                base.fnum(value, 2), ha="center", va="bottom", fontsize=8,
            )
    fig.suptitle(
        f"No-LOS: so sánh ghép cặp trên {paired} ca đủ năm phương pháp",
        fontsize=15, fontweight="bold",
    )
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_09_aggregate_metrics.png", dpi=170,
        bbox_inches="tight",
    )
    base.plt.close(fig)

    diagnostics = [item["pstmo_diagnostics"] for item in items]
    fig, axes = base.plt.subplots(1, 3, figsize=(15, 4.8))
    success = [aggregate["methods"][method]["success_count"] for method in methods]
    axes[0].bar(
        [base.METHOD_LABEL[method] for method in methods], success,
        color=[base.METHOD_COLOR[method] for method in methods],
    )
    axes[0].set_ylim(0, 37)
    axes[0].set_title("Thành công / 35", fontweight="bold")
    axes[0].tick_params(axis="x", rotation=20)
    for index, value in enumerate(success):
        axes[0].text(index, value + 0.3, str(value), ha="center")
    axes[1].bar(
        ["Raw input", "Sau conditioning"],
        [
            statistics.fmean(value["raw_input_points"] for value in diagnostics),
            statistics.fmean(value["conditioning_output_points"] for value in diagnostics),
        ],
        color=["#94a3b8", "#2563eb"],
    )
    axes[1].set_title("Số điểm trung bình — không có LOS", fontweight="bold")
    axes[1].grid(axis="y", alpha=0.2)
    axes[2].bar(
        ["LOS", "Toàn PSTMO"],
        [
            1000 * statistics.fmean(value["los_runtime_s"] for value in diagnostics),
            1000 * statistics.fmean(value["runtime_s"] for value in diagnostics),
        ],
        color=["#cbd5e1", "#0f766e"],
    )
    axes[2].set_title("Runtime nội bộ TB (ms)", fontweight="bold")
    axes[2].grid(axis="y", alpha=0.2)
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_10_success_points_runtime.png", dpi=170,
        bbox_inches="tight",
    )
    base.plt.close(fig)

    corners = [
        corner
        for item in items
        for corner in item["pstmo_diagnostics"]["corner_search"]
        if not corner.get("pass_through")
    ]
    selected = [corner for corner in corners if corner.get("selected_trim", 0) > 0]
    fig, axes = base.plt.subplots(2, 2, figsize=(13, 8))
    axes[0, 0].hist(
        [corner["selected_control_fraction"] for corner in selected],
        bins=base.np.arange(0.075, 0.526, 0.025), color="#16a34a", edgecolor="white",
    )
    axes[0, 0].set_title(f"α=q/d được chọn ({len(selected)} transition)", fontweight="bold")
    axes[0, 0].set_xlabel("α")
    axes[0, 1].hist(
        [corner["selected_trim"] for corner in selected],
        bins=base.np.linspace(0.02, 0.82, 17), color="#2563eb", edgecolor="white",
    )
    axes[0, 1].set_title("d được chọn", fontweight="bold")
    axes[0, 1].set_xlabel("d (m)")
    scatter = axes[1, 0].scatter(
        [corner["turn_angle"] for corner in selected],
        [corner["selected_control_fraction"] for corner in selected],
        c=[corner["selected_curvature_energy"] for corner in selected],
        cmap="viridis", s=28,
    )
    axes[1, 0].set_xlabel("Góc rẽ (rad)")
    axes[1, 0].set_ylabel("α")
    axes[1, 0].set_title("Mỗi góc có α riêng", fontweight="bold")
    fig.colorbar(scatter, ax=axes[1, 0], label="Eκ")
    axes[1, 1].bar(
        ["Coarse", "Recovery", "Fine"],
        [
            sum(value["coarse_shape_evaluations"] for value in diagnostics),
            sum(value["recovery_shape_evaluations"] for value in diagnostics),
            sum(value["refinement_shape_evaluations"] for value in diagnostics),
        ],
        color=["#2563eb", "#dc2626", "#16a34a"],
    )
    axes[1, 1].set_title("Tổng đánh giá hình dạng / 35 ca", fontweight="bold")
    for axis in axes.ravel():
        axis.grid(alpha=0.18)
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_11_dq_live_diagnostics.png", dpi=170,
        bbox_inches="tight",
    )
    base.plt.close(fig)

    fig, axis = base.plt.subplots(figsize=(13, 5.5))
    grid = base.np.ones((len(base.ENVIRONMENTS), len(base.PLANNERS), 3))
    grid[:] = base.np.array([0.82, 0.96, 0.86])
    axis.imshow(grid, aspect="auto")
    for env_index, environment in enumerate(base.ENVIRONMENTS):
        for planner_index, planner in enumerate(base.PLANNERS):
            case = next(
                item for item in items
                if item["environment"] == environment and item["planner"] == planner
            )
            axis.text(
                planner_index, env_index, f"{case['case_id']}\nOK\nLOS=0",
                ha="center", va="center", fontweight="bold", color="#14532d",
                fontsize=8,
            )
    axis.set_xticks(range(len(base.PLANNERS)), base.PLANNERS)
    axis.set_yticks(
        range(len(base.ENVIRONMENTS)),
        [base.ENV_LABEL[environment] for environment in base.ENVIRONMENTS],
    )
    axis.set_title(
        "Ma trận bằng chứng no-LOS: PNG + JSON + final invariant",
        fontweight="bold",
    )
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_12_test_matrix.png", dpi=170, bbox_inches="tight"
    )
    base.plt.close(fig)
    return case_figures


def build_report(items, rows, aggregate, complete_cases, case_figures):
    paired = aggregate["complete_five_method_cases"]
    method_table = []
    for method in base.METHODS:
        values = aggregate["methods"][method]
        method_table.append([
            f"<b>{base.METHOD_LABEL[method]}</b>",
            f"{values['success_count']}/35",
            base.fnum(values["paired_mean_path_length_m"], 4),
            base.fnum(values["paired_mean_max_abs_curvature_1pm"], 4),
            base.fnum(values["paired_mean_curvature_energy_1pm"], 4),
            base.fnum(1000 * values["paired_mean_algorithm_time_s"], 2),
            base.fnum(values["paired_mean_footprint_clearance_min_m"], 4),
            str(int(values["paired_mean_footprint_collision_sample_count"])),
        ])
    pstmo = aggregate["methods"]["pstmo"]
    comparison_rows = []
    for method in ("raw", "simple", "savitzky_golay", "constrained"):
        reference = aggregate["methods"][method]
        comparison_rows.append([
            base.METHOD_LABEL[method],
            base.fnum(base.pct_reduction(
                pstmo["paired_mean_path_length_m"],
                reference["paired_mean_path_length_m"],
            ), 2) + "%",
            base.fnum(base.pct_reduction(
                pstmo["paired_mean_max_abs_curvature_1pm"],
                reference["paired_mean_max_abs_curvature_1pm"],
            ), 2) + "%",
            base.fnum(base.pct_reduction(
                pstmo["paired_mean_curvature_energy_1pm"],
                reference["paired_mean_curvature_energy_1pm"],
            ), 2) + "%",
            base.fnum(1000 * (
                pstmo["paired_mean_algorithm_time_s"]
                - reference["paired_mean_algorithm_time_s"]
            ), 2) + " ms",
            base.fnum(base.pct_reduction(
                pstmo["paired_mean_footprint_clearance_min_m"],
                reference["paired_mean_footprint_clearance_min_m"],
            ), 2) + "%",
        ])
    diagnostics = aggregate["pstmo_diagnostics"]
    all_diag = [item["pstmo_diagnostics"] for item in items]
    selected_corners = [
        corner
        for item in items
        for corner in item["pstmo_diagnostics"]["corner_search"]
        if corner.get("selected_trim", 0) > 0
    ]
    alpha_values = [corner["selected_control_fraction"] for corner in selected_corners]
    trim_values = [corner["selected_trim"] for corner in selected_corners]
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, capture_output=True
    ).stdout.strip()
    sources = [
        ROOT / "src/adaptive_pivot_g2/src/path_conditioning.cpp",
        ROOT / "src/adaptive_pivot_g2/src/hierarchical_shape_search.cpp",
        ROOT / "src/adaptive_pivot_g2/src/quintic_transition.cpp",
        ROOT / "src/adaptive_pivot_g2/src/time_parameterization.cpp",
        ROOT / "src/adaptive_pivot_g2/src/path_optimization.cpp",
        ROOT / "src/adaptive_pivot_g2/src/candidate_selection.cpp",
        ROOT / "src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp",
        ROOT / "src/adaptive_pivot_g2_nav2/include/adaptive_pivot_g2_nav2/adaptive_pivot_g2_smoother.hpp",
        ROOT / "src/vacuum_robot_gazebo/config/nav2_params.yaml",
    ]
    source_rows = [
        [str(path.relative_to(ROOT)), base.sha256_file(path)[:20] + "…"]
        for path in sources
    ]
    content = []
    content.append(
        '<div class="cover"><h1>BÁO CÁO TOÀN DIỆN PSTMO — CẤU HÌNH TẮT LOS</h1>'
        '<div class="subtitle">Conditioning + hai d hình học + tìm α=q/d thô–tinh '
        '+ time gate + DP + swept-footprint</div>'
        '<div class="meta">Lý thuyết, công thức, mã nguồn và 35 ca Gazebo/RViz2 live • 02/08/2026</div></div>'
    )
    content.append(
        '<div class="evidence"><b>Kỷ luật bằng chứng.</b> Mọi số so sánh chính '
        'được tính lại từ 35 JSON thu cùng 35 ảnh RViz2. Mỗi ca lưu start/goal '
        'đủ x,y,yaw, exact ROS Path và SHA-256 của Raw. Bảy ảnh Gazebo xác nhận '
        'đúng world. Hình mang nhãn “sơ đồ” dùng để giảng lý thuyết; hình case '
        'là bằng chứng live và không dịch chuyển tọa độ đường.</div>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_09_aggregate_metrics.png",
        f"Kết quả ghép cặp công bằng trên {paired} ca đủ cả năm phương pháp.",
    ))
    content.append('<h2>1. Kết luận điều hành</h2>')
    content.append(
        '<p>PSTMO độc lập trong bản này chạy đúng một pipeline: '
        '<b>planner path → condition_polyline → sinh hai d → tìm α=q/d → '
        'cổng footprint/động học → time gate → DP → ghép → invariant cuối</b>. '
        'LOS không bị “chọn nhưng không có shortcut”; nó hoàn toàn không được gọi. '
        'Cả 35/35 diagnostics đều có <code>preprocessing_mode=condition_only</code>, '
        '<code>los_executed=false</code>, số lần thử shortcut bằng 0 và thời gian LOS bằng 0.</p>'
    )
    content.append(base.table_html(
        ["Phương pháp", "Thành công", "L (m)", "Kmax (1/m)", "Eκ (1/m)", "T (ms)", "Clr min (m)", "Mẫu va chạm TB"],
        method_table, "compact",
    ))
    content.append(
        '<p>L, Kmax, Eκ, thời gian và clearance trong bảng dùng đúng 34 nhóm '
        'có đủ năm phương pháp; tỷ lệ thành công dùng toàn bộ 35 ca. Simple '
        'thất bại thật ở C30 nên không được điền 0 hoặc âm thầm loại khỏi tỷ lệ thành công.</p>'
    )
    content.append(base.table_html(
        ["PSTMO so với", "ΔL giảm", "ΔKmax giảm", "ΔEκ giảm", "T chênh tuyệt đối", "Clr min giảm"],
        comparison_rows, "compact",
    ))
    content.append(
        '<div class="note"><b>Cách đọc dấu.</b> “Giảm” âm nghĩa là PSTMO tăng '
        'chỉ số đó so với đối chứng. Với thời gian, số dương là PSTMO chậm hơn. '
        'Clearance giảm không đồng nghĩa collision; nó là khoảng dự phòng nhỏ hơn '
        'sau khi footprint thật vẫn qua cổng an toàn.</div>'
    )

    content.append('<h2>2. Phạm vi và lý do tắt LOS</h2>')
    content.append(
        '<p>LOS tham lam tối ưu tiêu chí “điểm xa nhất còn an toàn hình học”. '
        'Một chord có thể không giao cắt footprint nhưng bỏ nhiều điểm neo mà '
        'controller dùng để biểu diễn hành lang, hướng tiếp cận hoặc sự đổi hướng '
        'từng bước. Vì vậy tính hợp lệ của chord trên costmap không tự suy ra '
        'khả năng controller bám ổn định trong mọi đoạn. Cấu hình no-LOS ưu tiên '
        'giữ corridor sau conditioning và chỉ thay đổi cục bộ quanh góc.</p>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_08_why_no_los.png",
        "Sơ đồ lý do kiến trúc; đây không phải số liệu thực thi robot.",
    ))
    content.append(
        '<div class="warning"><b>Giới hạn tuyên bố.</b> Bộ 35 ca hiện tại gọi '
        'SmoothPath với <code>execute=false</code>. Nó chứng minh planner/smoother '
        'tạo đường, invariant và footprint hậu kiểm; nó chưa đo tỷ lệ robot chạy '
        'tới goal. Nhận xét “LOS làm xe có đoạn không đi được” là động cơ thiết kế '
        'do người dùng quan sát. Muốn lượng hóa phải chạy thêm benchmark execute=true '
        'với timeout, sai số bám và trạng thái Nav2 được ghi lại.</div>'
    )
    content.append(
        '<p>Adaptive Hybrid không nằm trong so sánh hội nghị này và không bị đổi '
        'hành vi: Pivot nội bộ của Hybrid vẫn dùng <code>condition_only + '
        'legacy_joint_d_q</code>. Chỉ plugin PSTMO độc lập dùng bộ tìm mới.</p>'
    )

    content.append('<h2>3. Pipeline và bất biến</h2>')
    content.append(base.figure_html(
        FIG_DIR / "figure_01_pipeline_no_los.png", "Sơ đồ pipeline no-LOS hiện tại."
    ))
    content.append(
        '<p>Input là chuỗi pose của global planner. Output phải bảo toàn vị trí '
        'start/goal và orientation của goal, hữu hạn, không tạo vị trí trùng ngoài '
        'marker quay chủ ý, không giao cắt footprint trong các chuyển động nằm '
        'trong output, có time profile hợp lệ và qua kiểm tra độc lập sau khi ghép. '
        'Orientation tại pose đầu output được đặt theo cạnh tịnh tiến đầu tiên, '
        'không phải sao chép yaw start đầu vào.</p>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_02_hard_gates.png", "Sơ đồ các cổng loại cứng."
    ))

    content.append('<h2>4. Biểu diễn đường và metric cơ bản</h2>')
    content.append(
        '<p>Với polyline P={p₀,…,pₙ}, độ dài được tính trực tiếp trên cùng chuỗi '
        'tọa độ mà RViz2 hiển thị:</p>'
    )
    content.append('<div class="eq">L(P)=Σ<sub>i=0</sub><sup>n−1</sup> ‖p<sub>i+1</sub>−p<sub>i</sub>‖₂</div>')
    content.append(
        '<p>Curvature rời rạc được suy ra từ ba điểm liên tiếp; với đường tham số '
        'liên tục r(u)=(x(u),y(u)) dùng công thức chuẩn:</p>'
    )
    content.append('<div class="eq">κ(u)=[x′(u)y″(u)−y′(u)x″(u)]/[x′(u)²+y′(u)²]<sup>3/2</sup></div>')
    content.append('<div class="eq">K<sub>max</sub>=max<sub>s</sub>|κ(s)|,&nbsp;&nbsp; E<sub>κ</sub>=∫<sub>0</sub><sup>L</sup>κ(s)²ds</div>')
    content.append(
        '<p>Kmax phản ánh đỉnh uốn khó nhất; Eκ phạt độ cong trên toàn đường. '
        'Hai đường có cùng Kmax vẫn có thể khác Eκ nếu một đường duy trì cong '
        'lâu hơn. Eκ có đơn vị 1/m vì κ² có 1/m² và ds có m.</p>'
    )
    content.append(
        '<p>Với các marker quay tại chỗ, báo cáo còn ghi số pivot và tổng góc '
        'quay tuyệt đối:</p>'
    )
    content.append('<div class="eq">N<sub>pivot</sub>=#marker,&nbsp;&nbsp;Θ<sub>pivot</sub>=Σ<sub>j=1</sub><sup>N<sub>pivot</sub></sup>|wrap(ψ<sub>j</sub><sup>+</sup>−ψ<sub>j</sub><sup>−</sup>)|</div>')
    content.append(
        '<p>Npivot và Θpivot thấp hơn thường là lợi thế cho chuyển động liên tục '
        'vì giảm dừng–quay–đi, nhưng chỉ khi các transition thay thế vẫn an toàn '
        'và khả thi. Không được giảm pivot bằng cách ép một đường cong không thể chạy.</p>'
    )

    content.append('<h2>5. Conditioning: giảm nhiễu nhưng giữ corridor</h2>')
    content.append(
        '<p>Conditioning dùng Ramer–Douglas–Peucker lặp. Một chord (pᵢ,pⱼ) chỉ '
        'thay dải điểm khi độ lệch lớn nhất không vượt ngưỡng và swept-footprint '
        'của chord vẫn an toàn:</p>'
    )
    content.append('<div class="eq">δ(i,j)=max<sub>i&lt;k&lt;j</sub>dist(p<sub>k</sub>,segment(p<sub>i</sub>,p<sub>j</sub>))≤ε<sub>RDP</sub></div>')
    content.append('<div class="eq">ε<sub>RDP</sub>=1,5·resolution=1,5·0,05=0,075 m</div>')
    content.append(
        '<p>Bộ triệt dao động cục bộ sau đó yêu cầu span ≤2,0 m, ít nhất hai '
        'lần đổi dấu góc, góc tối thiểu 0,20 rad, độ lệch ≤3·resolution=0,15 m '
        'và chord vẫn an toàn. Đây không phải LOS xa nhất: nó có ngưỡng lệch '
        'nhỏ để giữ gần corridor planner.</p>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_03_conditioning_actual.png",
        "C01 live: đầu vào transition trùng đúng output conditioning vì LOS tắt.",
    ))

    content.append('<h2>6. Phát hiện góc</h2>')
    content.append(
        '<p>Tại ba neo (pᵢ₋₁,pᵢ,pᵢ₊₁), đặt vector đơn vị u theo cạnh vào '
        'và v theo cạnh ra. Góc có dấu được tính ổn định bằng atan2:</p>'
    )
    content.append('<div class="eq">θ<sub>i</sub>=atan2(u<sub>x</sub>v<sub>y</sub>−u<sub>y</sub>v<sub>x</sub>,u·v)</div>')
    content.append(
        '<p>Nếu |θᵢ|≤0,0872664626 rad (5°), neo được đi xuyên qua. Góc '
        'lớn hơn tạo tập trạng thái transition hoặc quay tại chỗ. Miền transition '
        'hình học kết thúc trước 170°; các góc quá gắt dựa vào pivot an toàn.</p>'
    )

    content.append('<h2>7. Bézier bậc năm và điều kiện G²</h2>')
    content.append(
        '<p>Với đỉnh V, trim d tạo entry A=V−du và exit B=V+dv. Đặt q=αd. '
        'Sáu control point:</p>'
    )
    content.append('<div class="eq">P₀=A; P₁=A+qu; P₂=A+2qu; P₃=B−2qv; P₄=B−qv; P₅=B</div>')
    content.append('<div class="eq">B(t)=Σ<sub>i=0</sub><sup>5</sup>C(5,i)(1−t)<sup>5−i</sup>t<sup>i</sup>P<sub>i</sub>,&nbsp;0≤t≤1</div>')
    content.append(
        '<p>Đạo hàm đầu B′(0)=5(P₁−P₀)=5qu và B′(1)=5(P₅−P₄)=5qv, '
        'nên tiếp tuyến khớp hai đoạn thẳng. Đạo hàm hai đầu:</p>'
    )
    content.append('<div class="eq">B″(0)=20(P₂−2P₁+P₀)=0,&nbsp;&nbsp;B″(1)=20(P₅−2P₄+P₃)=0</div>')
    content.append(
        '<p>Vì κ phụ thuộc tích có hướng B′×B″, κ(0)=κ(1)=0. Đoạn '
        'thẳng kề cũng có κ=0, nên vị trí, tiếp tuyến và curvature nối liên tục: '
        'đây là G² theo hình học. Điều này không tự khẳng định jerk thời gian liên '
        'tục; time parameterization xử lý ràng buộc vận tốc/gia tốc riêng.</p>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_04_bezier_g2.png", "Sơ đồ Bézier và profile curvature chuẩn hóa."
    ))

    content.append('<h2>8. Vai trò riêng của d, q và α=q/d</h2>')
    content.append(
        '<ul><li><b>d</b> quyết định transition chiếm bao nhiêu chiều dài ở hai '
        'phía đỉnh — tức kích thước hình học.</li><li><b>q</b> là khoảng cách '
        'control dọc tiếp tuyến.</li><li><b>α=q/d</b> là đại lượng không thứ '
        'nguyên điều khiển hình dạng khi d cố định.</li></ul>'
    )
    content.append(
        '<p>Thuật toán không còn dùng q/d=0,35 như hằng thiết kế. Miền cho phép '
        '0&lt;α≤0,5; α&gt;0,5 bị loại. α=0,5 vẫn phải qua kiểm tra suy biến, '
        'đổi dấu curvature, bánh trong, timing và footprint như mọi ứng viên khác.</p>'
    )

    content.append('<h2>9. Hai d có cơ sở hình học</h2>')
    content.append('<div class="eq">d<sub>pref</sub>=min(0,8,L<sub>in</sub>,L<sub>out</sub>)</div>')
    content.append(
        '<p>Với đoạn chung L và margin m, mỗi góc được cấp trước một nửa phần '
        'còn lại: b=(L−m)/2. Ở cạnh nối start hoặc goal không có góc kề nên dùng '
        'toàn bộ chiều dài cạnh. Khi đó:</p>'
    )
    content.append('<div class="eq">d<sub>compat</sub>=min(d<sub>pref</sub>,b<sub>in</sub>,b<sub>out</sub>)</div>')
    content.append(
        '<p>dcompat bị bỏ nếu nhỏ hơn max(0,02;½min(sample spacing,resolution)) '
        'hoặc gần trùng dpref trong dung sai khử trùng. Margin tự động hiện tại '
        'm=max(output spacing,2·sample spacing,resolution)=max(0,05;0,04;0,05)=0,05 m.</p>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_06_two_trim_dp.png", "Sơ đồ hai d và đồ thị trạng thái DP."
    ))

    content.append('<h2>10. Tìm α thô–phục hồi–tinh</h2>')
    content.append(
        '<ol><li>Đánh giá coarse {0,1;0,2;0,3;0,4;0,5}.</li>'
        '<li>Chỉ ứng viên qua tất cả cổng cứng mới có Eκ hợp lệ.</li>'
        '<li>Chọn Eκ nhỏ nhất; hòa trong 10⁻¹² chọn α nhỏ hơn.</li>'
        '<li>Winner coarse nội được tinh từ hàng xóm trái tới hàng xóm phải; '
        'winner biên tinh ô biên. Khoảng được chia thành 10 phần, tức 11 nút kể '
        'cả hai biên, và điểm trùng bị bỏ.</li>'
        '<li>Nếu toàn bộ coarse thất bại mới thử midpoint recovery '
        '{0,15;0,25;0,35;0,45}; winner recovery được tinh trong coarse cell chứa nó.</li></ol>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_05_alpha_search.png", "Sơ đồ lưới tìm α; 0,35 chỉ là một midpoint recovery."
    ))
    content.append(
        f'<p>Dữ liệu live chọn {len(alpha_values)} transition: α từ '
        f'{base.fnum(min(alpha_values),3)} tới {base.fnum(max(alpha_values),3)}, '
        f'trung bình {base.fnum(statistics.fmean(alpha_values),4)}; d từ '
        f'{base.fnum(min(trim_values),3)} m tới {base.fnum(max(trim_values),3)} m. '
        'Các con số này lấy từ diagnostics, không suy ra bằng mắt từ ảnh.</p>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_11_dq_live_diagnostics.png", "Phân bố d, α và số đánh giá live của 35 ca."
    ))

    content.append('<h2>11. Động học vi sai và điều kiện không đảo bánh</h2>')
    content.append(
        '<p>Với khoảng cách hai vệt bánh b=0,2548 m, vận tốc thân v và '
        'curvature κ, vận tốc góc ω=vκ. Vận tốc hai bánh:</p>'
    )
    content.append('<div class="eq">v<sub>L</sub>=v(1−bκ/2),&nbsp;&nbsp;v<sub>R</sub>=v(1+bκ/2)</div>')
    content.append(
        '<p>PSTMO loại transition nếu một hệ số bánh nhỏ hơn 0 (ngoài dung sai), '
        'vì transition tịnh tiến này không được yêu cầu bánh trong chạy lùi. '
        'Quay tại chỗ là trạng thái riêng và được kiểm tra swept-footprint xoay.</p>'
    )

    content.append('<h2>12. Giới hạn vận tốc theo curvature</h2>')
    content.append(
        '<p>Tại mỗi mẫu, trần tốc độ là min của giới hạn thân, giới hạn ω, '
        'gia tốc ngang và tốc độ bánh:</p>'
    )
    content.append('<div class="eq">v<sub>cap</sub>(κ)=min[v<sub>max</sub>,ω<sub>max</sub>/|κ|,√(a<sub>y,max</sub>/|κ|),v<sub>w,max</sub>/max(|1−bκ/2|,|1+bκ/2|)]</div>')
    content.append(
        '<p>Thông số hiện tại: vmax=0,30 m/s, ωmax=0,80 rad/s, '
        'vw,max=0,36 m/s, ay,max=0,18 m/s². Khi κ→0, hai hạng chia '
        'cho κ được xem là không ràng buộc.</p>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_07_kinematic_time_gate.png", "Sơ đồ ảnh hưởng của curvature lên trần vận tốc."
    ))

    content.append('<h2>13. Time parameterization</h2>')
    content.append(
        '<p>Từ vcap, thuật toán quét tiến/lùi để áp giới hạn gia tốc tịnh tiến '
        'amax=0,35 m/s² và giảm tốc dmax=0,45 m/s²:</p>'
    )
    content.append('<div class="eq">v<sub>i</sub>≤√(v<sub>i−1</sub>²+2a<sub>max</sub>Δs<sub>i−1</sub>),&nbsp;&nbsp;v<sub>i−1</sub>≤√(v<sub>i</sub>²+2d<sub>max</sub>Δs<sub>i−1</sub>)</div>')
    content.append('<div class="eq">Δt<sub>i</sub>=2Δs<sub>i</sub>/(v<sub>i</sub>+v<sub>i+1</sub>),&nbsp;&nbsp;ω<sub>i</sub>=v<sub>i</sub>κ<sub>i</sub></div>')
    content.append(
        '<p>Nếu |ωᵢ−ωᵢ₋₁|/Δt vượt aω,max=1,20 rad/s², cap ở hai '
        'đầu interval được thu theo xấp xỉ căn bậc hai và lặp tối đa 40 lần. '
        'Không hội tụ là lỗi cứng. Các d khác nhau được mở rộng bằng đoạn thẳng '
        'để so timing trên cùng common window.</p>'
    )

    content.append('<h2>14. Thời gian quay tại chỗ và time gate</h2>')
    content.append(
        '<p>Với góc quay |θ|, profile quay tối thiểu là tam giác nếu chưa chạm '
        'ωmax và hình thang nếu đã chạm:</p>'
    )
    content.append('<div class="eq">T<sub>rot</sub>=2√(|θ|/a<sub>ω</sub>) nếu |θ|≤ω<sub>max</sub>²/a<sub>ω</sub></div>')
    content.append('<div class="eq">T<sub>rot</sub>=2ω<sub>max</sub>/a<sub>ω</sub>+(|θ|−ω<sub>max</sub>²/a<sub>ω</sub>)/ω<sub>max</sub> nếu ngược lại</div>')
    content.append(
        '<p>Thời gian pivot window bằng thời gian đi tới đỉnh và dừng + quay + '
        'khởi hành. Nhánh transition chỉ mở khi:</p>'
    )
    content.append('<div class="eq">T<sub>fastest transition</sub>+ΔT&lt;T<sub>pivot</sub>,&nbsp;&nbsp;ΔT=0,15 s</div>')
    content.append(
        '<p>Nếu pivot không an toàn, điều kiện thời gian không đóng nhánh transition. '
        'Candidate trong slack cạnh tranh mới được giữ cho bước chọn cục bộ/DP.</p>'
    )

    content.append('<h2>15. Chi phí cục bộ giữa các d</h2>')
    content.append('<div class="eq">risk=min(1,C<sub>peak</sub>/252),&nbsp; angular=min(1,|ω|<sub>max</sub>/0,80),&nbsp; energy=Eκ/(Eκ+1)</div>')
    content.append('<div class="eq">J=(0,15·risk+0,10·angular+0,75·energy)/(0,15+0,10+0,75)</div>')
    content.append(
        '<p>Điểm này chỉ chọn giữa các trạng thái đã an toàn và đủ nhanh. '
        'Cpeak là cost inflation lớn nhất tại tâm trên transition; swept-footprint '
        'thật vẫn được kiểm tra riêng. Clearance tối thiểu toàn đường không nằm '
        'trong J. Riêng tại cùng một d, α được chọn bằng Eκ chứ không bằng J.</p>'
    )

    content.append('<h2>16. DP chống chồng lấn</h2>')
    content.append(
        '<p>Mỗi góc i có tập trạng thái Zᵢ: transition ở dpref, transition ở '
        'dcompat, pass-through hoặc pivot tùy khả thi. Cạnh giữa zᵢ và zᵢ₊₁ '
        'chỉ tồn tại khi:</p>'
    )
    content.append('<div class="eq">d(z<sub>i</sub>)+d(z<sub>i+1</sub>)+m≤L<sub>i</sub></div>')
    content.append('<div class="eq">D<sub>i</sub>(z)=J<sub>i</sub>(z)+min<sub>z′∈Z<sub>i−1</sub>, compatible(z′,z)</sub>D<sub>i−1</sub>(z′)</div>')
    content.append(
        '<p>Tie-break: tổng J nhỏ hơn; nếu bằng trong 10⁻¹² thì ít pivot hơn; '
        'nếu vẫn bằng chọn index nhỏ hơn. Vì trạng thái chứa d cụ thể, DP không '
        'gộp nhầm hai lời giải cùng góc nhưng chiếm đoạn khác nhau.</p>'
    )

    content.append('<h2>17. Ghép output, yaw và kiểm tra footprint</h2>')
    content.append(
        '<p>Đoạn thẳng được nội suy theo output spacing 0,05 m; transition chèn '
        'mẫu Bézier; pivot tạo hai pose cùng vị trí nhưng yaw trước/sau khác nhau. '
        'Với footprint body F và pose (x,y,ψ), footprint trong map là:</p>'
    )
    content.append('<div class="eq">F<sub>map</sub>(x,y,ψ)={ [x;y]+R(ψ)f | f∈F },&nbsp;R(ψ)=[[cosψ,−sinψ],[sinψ,cosψ]]</div>')
    content.append(
        '<p>Footprint thật là hình chữ nhật 0,44×0,34 m. Mỗi khoảng pose được '
        'nội suy cả vị trí và yaw với bước không lớn hơn max(0,005;½resolution); '
        'mẫu lethal, unknown, ngoài map hoặc polygon giao vật cản đều bị loại. '
        'Output đã ghép được quét lại độc lập; không fallback sang Raw hay pipeline khác.</p>'
    )
    content.append(
        '<div class="warning"><b>Khoảng xoay đầu chưa nằm trong invariant no-LOS.</b> '
        'Code đặt yaw pose đầu output theo heading cạnh đầu. Khi LOS tắt, phép xoay '
        'từ yaw start thực của robot sang heading này chưa được gọi qua một cổng '
        'pivot_is_safe riêng. Các yaw benchmark được chọn gần hướng khởi hành an '
        'toàn, nhưng đây vẫn là giới hạn triển khai phải sửa nếu muốn tuyên bố '
        'swept-footprint từ trạng thái robot hiện tại.</div>'
    )

    content.append('<h2>18. Clearance footprint hậu kiểm</h2>')
    content.append(
        '<p>Clearance hậu kiểm dùng distance transform của PGM. Đường được nội '
        'suy mỗi 0,05 m hoặc 5°; hình chữ nhật footprint được lấy mẫu với bước '
        'không lớn hơn 0,025 m. Tại mỗi mẫu footprint, khoảng cách tới ô occupied '
        'hoặc unknown được hiệu chỉnh trừ nửa đường chéo ô; clearance pose là min '
        'các mẫu. Vì vậy đây là xấp xỉ raster bảo thủ, không phải khoảng cách '
        'polygon liên tục chính xác. Collision sample count đếm mẫu có '
        'clearance≤0. Chỉ số này không tham gia chọn α:</p>'
    )
    content.append('<div class="eq">c(P)=min<sub>pose∈P</sub> min<sub>o∈Obstacle</sub> dist(F<sub>map</sub>(pose),o)</div>')
    content.append(
        f'<p>Trên {paired} ca ghép cặp, clearance min trung bình PSTMO là '
        f'{base.fnum(pstmo["paired_mean_footprint_clearance_min_m"],4)} m; '
        f'collision sample trung bình bằng '
        f'{base.fnum(pstmo["paired_mean_footprint_collision_sample_count"],2)}. '
        'Không được diễn giải clearance dương rất nhỏ là “dư địa vận hành lớn”; '
        'nó chỉ chứng minh không giao cắt trên bản đồ tĩnh và mẫu đã kiểm.</p>'
    )

    content.append('<h2>19. Thiết kế benchmark công bằng</h2>')
    content.append(
        '<p>Ma trận gồm 7 world × 5 planner. Trong mỗi ô, Raw, Simple, '
        'Savitzky–Golay, Constrained và PSTMO nhận cùng exact Raw path. Điểm '
        'start/goal và yaw giống nhau trong cùng ô; hash Raw được lưu. Không so '
        'hai phương pháp từ hai lần planner khác nhau rồi gọi là ghép cặp.</p>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_12_test_matrix.png", "35 ô no-LOS đều có output PSTMO và invariant cuối."
    ))
    content.append(base.table_html(
        ["Planner", "Đặc trưng", "Vai trò trong phép thử"],
        [
            ["NavFn A*", "Grid A*", "Đường grid cost-aware"],
            ["NavFn Dijkstra", "Grid Dijkstra", "Cùng costmap, chiến lược tìm kiếm khác"],
            ["ThetaStar", "Any-angle", "Nhiều chord dài"],
            ["Smac 2D", "Cost-aware 2D", "Có light smoother nội tại của planner"],
            ["Smac Hybrid", "Dubins tiến-only", "Motion model khác robot vi sai pivot; baseline planner"],
        ],
        "compact",
    ))
    content.append(
        '<div class="warning"><b>Baseline thất bại C30.</b> Simple bị Nav2 '
        'SmoothPath hủy với status=6, code=503 do collision tại '
        'x=−4,741290 m, y=3,482165 m, yaw=0,352672 rad. PSTMO vẫn hợp lệ. '
        'Log và SHA-256 được lưu trong <code>failure_evidence_C30_no_los.json</code>.</div>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_10_success_points_runtime.png",
        "Thành công, mức giảm điểm do conditioning và runtime; LOS bằng 0.",
    ))

    content.append('<h2>20. Phân tích kết quả tổng hợp</h2>')
    content.append(
        f'<p>Trên {paired} nhóm đầy đủ, PSTMO no-LOS có L='
        f'{base.fnum(pstmo["paired_mean_path_length_m"],4)} m, Kmax='
        f'{base.fnum(pstmo["paired_mean_max_abs_curvature_1pm"],4)} 1/m, Eκ='
        f'{base.fnum(pstmo["paired_mean_curvature_energy_1pm"],4)} 1/m và thời '
        f'gian SmoothPath={base.fnum(1000*pstmo["paired_mean_algorithm_time_s"],2)} ms. '
        'Bảng đầu báo phần trăm so với từng baseline ROS 2; không so với phiên '
        'thuật toán cũ làm kết luận hội nghị.</p>'
    )
    content.append(
        f'<p>Runtime nội bộ độ phân giải cao trên 35 ca là '
        f'{base.fnum(1000*diagnostics["runtime_s"]["mean"],2)} ms trung bình, '
        f'từ {base.fnum(1000*diagnostics["runtime_s"]["min"],2)} tới '
        f'{base.fnum(1000*diagnostics["runtime_s"]["max"],2)} ms. '
        f'Conditioning giảm trung bình {base.fnum(diagnostics["raw_input_points"]["mean"],1)} '
        f'pose xuống {base.fnum(diagnostics["conditioning_output_points"]["mean"],2)} '
        'điểm neo. LOS runtime=0, attempts=0 trong toàn bộ 35 ca.</p>'
    )
    content.append(
        f'<p>PSTMO tạo tổng cộng {int(diagnostics["g2_transitions"]["sum"])} '
        f'transition và {int(diagnostics["pivots"]["sum"])} pivot. Trung bình '
        f'mỗi ca có {base.fnum(pstmo["all_success_mean_pivot_marker_count"],4)} '
        f'pivot và tổng góc quay tại chỗ '
        f'{base.fnum(pstmo["all_success_mean_pivot_total_angle_rad"],4)} rad. '
        'Đây là chi phí thao tác, nên chỉ được xem là tốt khi giảm cùng lúc với '
        'việc giữ an toàn/khả thi.</p>'
    )
    content.append(
        '<p>Timeout 3 s là thời hạn client cấp cho cùng action SmoothPath, không '
        'phải “thời gian của PSTMO” và không xóa bất lợi tương đối. So sánh thời '
        'gian phải dùng chênh lệch ms trong bảng, đồng thời phân biệt duration '
        'trên topic và runtime nội bộ diagnostics.</p>'
    )

    content.append('<div class="page-break"></div><h2>21. Bằng chứng từng môi trường và planner</h2>')
    for environment in base.ENVIRONMENTS:
        section = base.ENVIRONMENTS.index(environment) + 1
        content.append(
            f'<h3>21.{section}. {base.ENV_LABEL[environment]}</h3>'
            f'<p>{NO_LOS_ENV_DESCRIPTION[environment]}</p>'
        )
        content.append(base.figure_html(
            GAZEBO_DIR / f"{environment}.png",
            f"Ảnh Gazebo live — {base.ENV_LABEL[environment]}.",
        ))
        for item in [value for value in items if value["environment"] == environment]:
            rows_for_case = [row for row in rows if row["case_id"] == item["case_id"]]
            content.append(
                f'<h4>{item["case_id"]} — {item["planner"]} — '
                f'{base.SCENARIO_LABEL.get(item["scenario"], item["scenario"])}</h4>'
            )
            content.append(base.figure_html(
                case_figures[item["case_id"]],
                f"{item['case_id']}: RViz2 gốc + exact ROS paths + bảng metric cùng ca.",
                "case-figure",
            ))
            content.append(
                '<ul>'
                + ''.join(
                    f'<li>{html.escape(note)}</li>'
                    for note in base.case_observation(item, rows_for_case)
                )
                + '</ul>'
            )
            diag = item["pstmo_diagnostics"]
            content.append(
                f'<p>Diagnostics: Raw {diag["raw_input_points"]} → conditioned '
                f'{diag["conditioning_output_points"]} điểm; LOS không chạy; '
                f'{diag["g2_transitions"]} transition, {diag["pivots"]} pivot, '
                f'{diag["evaluations"]} đánh giá, {diag["dp_states"]} DP states, '
                f'runtime nội bộ {1000*diag["runtime_s"]:.2f} ms.</p>'
            )

    content.append('<div class="page-break"></div><h2>22. Bảng đầy đủ 35 ca PSTMO no-LOS</h2>')
    pstmo_rows = []
    for item in items:
        row = next(
            value for value in rows
            if value["case_id"] == item["case_id"] and value["method"] == "pstmo"
        )
        diag = item["pstmo_diagnostics"]
        pstmo_rows.append([
            item["case_id"], base.ENV_LABEL[item["environment"]], item["planner"],
            base.fnum(row["path_length_m"], 3),
            base.fnum(row["max_abs_curvature_1pm"], 3),
            base.fnum(row["curvature_energy_1pm"], 3),
            base.fnum(1000 * diag["runtime_s"], 2),
            base.fnum(row["footprint_clearance_min_m"], 3),
            str(diag["conditioning_output_points"]),
            str(diag["g2_transitions"]), str(diag["pivots"]),
            base.fnum(row["pivot_total_angle_rad"], 3),
        ])
    content.append(base.table_html(
        ["ID", "Môi trường", "Planner", "L", "Kmax", "Eκ", "T nội bộ ms", "Clr min", "Neo", "G²", "Pivot", "Θpivot"],
        pstmo_rows, "tiny",
    ))

    content.append('<h2>23. Kiểm thử phần mềm, giới hạn và việc cần làm tiếp</h2>')
    content.append(
        '<ul><li>Build hai gói thay đổi thành công. Plugin Nav2: 37 test, '
        '0 error, 0 failure, 7 skip cppcheck theo môi trường; Gazebo: 28 test, '
        '0 error, 0 failure.</li><li>35/35 PSTMO có một pipeline và invariant '
        'cuối; 35/35 LOS không chạy.</li><li>Đây là 35 ca đại diện, không phủ '
        'toàn bộ không gian start–goal.</li><li>PGM tĩnh không mô hình hóa sai số '
        'localization, trượt bánh, tải, độ trễ controller hay vật cản động.</li>'
        '<li>Hình RViz/Gazebo chứng minh dữ liệu path nhưng không thay robot thật.</li>'
        '<li>PSTMO tối ưu rời rạc trên hai d và lưới α; không chứng minh optimum '
        'liên tục toàn cục.</li><li>Khi condition_only, yaw đầu output theo cạnh '
        'đầu; cần bổ sung kiểm tra xoay từ yaw robot hiện tại nếu dùng trong hành '
        'lang sát vật cản.</li><li>Bước tiếp theo phù hợp là benchmark execute=true '
        'ghép cặp LOS/no-LOS bằng cùng Raw hash, đo success-to-goal, cross-track '
        'error, thời gian hành trình và số recovery.</li></ul>'
    )

    content.append('<h2>24. Tái lập và tệp bằng chứng</h2>')
    content.append(
        '<ol><li>Build workspace và source <code>install/setup.bash</code>.</li>'
        '<li>Launch từng world với start pose đủ số thực x,y,yaw.</li>'
        '<li>Gọi <code>capture_pstmo_rviz_evidence.py '
        '--expected-preprocessing condition_only</code> cho từng planner.</li>'
        '<li>Xác minh preprocessing, los_executed, attempts, runtime, pipeline '
        'count và final invariant trước khi nhận ca.</li><li>Chạy script này để '
        'tính clearance từ exact path, sinh 35 composite, CSV/JSON và HTML.</li>'
        '<li>Chuyển HTML sang DOCX và xuất PDF bằng LibreOffice.</li></ol>'
    )
    content.append(base.table_html(
        ["Tệp nguồn khóa", "SHA-256 (rút gọn)"], source_rows, "compact"
    ))
    content.append(
        f'<p>Git HEAD khi tạo: <code>{html.escape(commit or "không có HEAD")}</code>. '
        'Worktree có thể chứa thay đổi chưa commit; hash tệp ở trên định danh '
        'trực tiếp phiên bản thuật toán được báo cáo.</p>'
    )
    content.append(
        '<ul><li><code>pstmo_no_los_toan_dien_assets/rviz_cases/</code>: '
        '35 PNG + 35 JSON exact paths.</li><li><code>gazebo/</code>: 7 ảnh world.</li>'
        '<li><code>figures/</code>: sơ đồ lý thuyết, chart và 35 composite.</li>'
        '<li><code>benchmark_live_35_cases_no_los.csv</code>: 175 hàng '
        'case×method, gồm thất bại.</li><li><code>benchmark_live_aggregate_no_los.json</code>: '
        'thống kê ghép cặp và diagnostics.</li><li><code>failure_evidence_C30_no_los.json</code>: '
        'log collision baseline Simple.</li></ul>'
    )

    style = '''
    @page { size:A4; margin:16mm; }
    body{font-family:"DejaVu Serif",serif;color:#172033;max-width:1120px;margin:auto;line-height:1.58;background:white}
    h1{font-size:30px;color:#0f3c5f}h2{font-size:22px;color:#0f3c5f;border-bottom:2px solid #9dc3e6;padding-bottom:4px;margin-top:32px}
    h3{font-size:18px;color:#155e75}h4{font-size:15px;color:#166534}.cover{text-align:center;padding:55px 25px 30px;border:3px solid #0f3c5f;margin:18px 0}
    .subtitle{font-size:18px;color:#0f766e;font-weight:bold;margin:18px}.meta{color:#475569}.evidence{background:#ecfeff;border-left:6px solid #0891b2;padding:14px 18px;margin:18px 0}
    .note{background:#eff6ff;border-left:6px solid #2563eb;padding:14px 18px;margin:18px 0}.warning{background:#fff7ed;border-left:6px solid #ea580c;padding:14px 18px;margin:18px 0}
    .eq{text-align:center;background:#f8fafc;border:1px solid #cbd5e1;padding:11px;margin:12px;font-family:"DejaVu Sans",sans-serif;page-break-inside:avoid}
    figure{text-align:center;margin:18px auto;page-break-inside:avoid}figure img{max-width:100%;height:auto;border:1px solid #cbd5e1}figcaption{font-size:12px;color:#475569;font-style:italic;margin-top:6px}.case-figure img{width:100%}
    table{border-collapse:collapse;width:100%;font-size:12px;margin:14px 0;page-break-inside:auto}th,td{border:1px solid #94a3b8;padding:5px;text-align:center}th{background:#dbeafe}.tiny{font-size:9px}.compact{font-size:11px}
    code{background:#f1f5f9;padding:1px 4px}.page-break{page-break-before:always}li{margin:4px 0}p{text-align:justify}
    '''
    document = (
        '<!doctype html><html lang="vi"><head><meta charset="utf-8">'
        '<title>Báo cáo toàn diện PSTMO tắt LOS</title><style>'
        + style + '</style></head><body>' + ''.join(content) + '</body></html>'
    )
    OUTPUT_HTML.write_text(document, encoding="utf-8")


def main():
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    configure_base_paths()
    items = load_evidence()
    rows = base.build_rows(items)
    aggregate, complete_cases = base.aggregate_rows(rows)
    base.persist_data(rows, aggregate, items)
    algorithm_figures(items)
    case_figures = result_figures(items, rows, aggregate)
    build_report(items, rows, aggregate, complete_cases, case_figures)
    print(json.dumps({
        "html": str(OUTPUT_HTML),
        "csv": str(OUTPUT_CSV),
        "json": str(OUTPUT_JSON),
        "cases": len(items),
        "case_figures": len(case_figures),
        "complete_groups": len(complete_cases),
    }, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
