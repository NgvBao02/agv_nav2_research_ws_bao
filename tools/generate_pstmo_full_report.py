#!/usr/bin/env python3

"""Generate the theory-complete, image-audited PSTMO condition-only report."""

from __future__ import annotations

import csv
import html
import json
import math
import shutil
import statistics
import subprocess
from pathlib import Path

import pstmo_report_common as base


ROOT = base.ROOT
DOCS = base.DOCS
ASSETS = DOCS / "pstmo_bao_cao_toan_dien_assets"
SOURCE_ASSETS = ASSETS
SOURCE_RVIZ_DIR = SOURCE_ASSETS / "rviz_cases"
RVIZ_DIR = ASSETS / "rviz_cases"
GAZEBO_DIR = ASSETS / "gazebo"
FIG_DIR = ASSETS / "figures"
EXECUTION_FIG_DIR = ASSETS / "execution_cases"
EXECUTION_MAP_MATRIX_DIR = ASSETS / "execution_map_matrices"
OUTPUT_HTML = DOCS / "BAO_CAO_TOAN_DIEN_PSTMO.html"
OUTPUT_CSV = ASSETS / "benchmark_hinh_hoc_175_luot.csv"
OUTPUT_JSON = ASSETS / "benchmark_hinh_hoc_tong_hop.json"
FAILURE_JSON = ASSETS / "bang_chung_loi_C30_simple.json"
EXECUTION_RESULTS = ROOT / "results" / "pstmo_execution_full_20260803"
EXECUTION_JSON = EXECUTION_RESULTS / "execution_aggregate_5planners_7env.json"
EXECUTION_CSV = EXECUTION_RESULTS / "execution_175_cases.csv"
FAILURE_LOG = Path(
    "/home/linh-pham/.ros/log/smoother_server_249871_1785664096626.log"
)
CURRENT_ENV_DESCRIPTION = dict(base.ENV_DESCRIPTION)
CURRENT_ENV_DESCRIPTION["warehouse_long_aisles"] = (
    "Các hành lang song song dài kiểm tra tích lũy độ cong, nhiều góc liên tiếp "
    "và khả năng giữ cấu trúc hình học cục bộ của đường do bộ lập kế hoạch tạo."
)


def load_evidence():
    """Load the exact current-PSTMO live matrix and reject mixed evidence."""
    items = [
        json.loads(path.read_text(encoding="utf-8"))
        for path in RVIZ_DIR.glob("*.json")
    ]
    items.sort(key=base.evidence_order)
    if len(items) != 35:
        raise RuntimeError(f"Expected 35 current-PSTMO records, found {len(items)}")
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
            and diag.get("pipeline_execution_count") == 1
            and diag.get("final_invariants_verified") is True
            and "pstmo" in item["metrics"]
            and "pstmo" in item["paths"]
        )
        if not valid:
            raise RuntimeError(f"Mixed or invalid PSTMO evidence: {item['case_id']}")
        screenshot = ROOT / item["rviz_screenshot"]
        if not screenshot.is_file():
            screenshot = SOURCE_RVIZ_DIR / Path(item["rviz_screenshot"]).name
        if not screenshot.is_file():
            raise FileNotFoundError(screenshot)
    return items


def prepare_assets():
    """Create the neutral report tree from the already-audited live evidence."""
    if not SOURCE_ASSETS.is_dir():
        raise FileNotFoundError(SOURCE_ASSETS)
    if SOURCE_ASSETS.resolve() != ASSETS.resolve():
        shutil.copytree(SOURCE_ASSETS, ASSETS, dirs_exist_ok=True)
    if EXECUTION_CSV.is_file():
        shutil.copy2(EXECUTION_CSV, ASSETS / EXECUTION_CSV.name)
    if EXECUTION_JSON.is_file():
        shutil.copy2(EXECUTION_JSON, ASSETS / EXECUTION_JSON.name)
    for legacy_relative in (
        "benchmark_live_35_cases_no_los.csv",
        "benchmark_live_aggregate_no_los.json",
        "failure_evidence_C30_no_los.json",
        "figures/figure_01_pipeline_no_los.png",
        "figures/figure_08_why_no_los.png",
    ):
        (ASSETS / legacy_relative).unlink(missing_ok=True)
    for evidence_path in RVIZ_DIR.glob("*.json"):
        evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
        screenshot_name = Path(evidence["rviz_screenshot"]).name
        evidence["rviz_screenshot"] = str(
            (RVIZ_DIR / screenshot_name).relative_to(ROOT)
        )
        if evidence.get("gazebo_screenshot"):
            gazebo_name = Path(evidence["gazebo_screenshot"]).name
            evidence["gazebo_screenshot"] = str(
                (GAZEBO_DIR / gazebo_name).relative_to(ROOT)
            )
        diagnostics = evidence.get("pstmo_diagnostics", {})
        for key in [key for key in diagnostics if key.startswith("los_")]:
            del diagnostics[key]
        evidence_path.write_text(
            json.dumps(evidence, ensure_ascii=False, indent=2, sort_keys=True)
            + "\n",
            encoding="utf-8",
        )


def load_execution_evidence():
    """Load the independently audited 175-run travel-time matrix."""
    if not EXECUTION_JSON.is_file() or not EXECUTION_CSV.is_file():
        raise FileNotFoundError(
            "Run tools/summarize_pstmo_full_execution.py before this report"
        )
    aggregate = json.loads(EXECUTION_JSON.read_text(encoding="utf-8"))
    with EXECUTION_CSV.open(encoding="utf-8", newline="") as stream:
        records = list(csv.DictReader(stream))
    audit = aggregate.get("audit", {})
    required = (
        audit.get("trial_count") == 175
        and audit.get("success_count")
        == audit.get("ground_truth_goal_reached_count")
        and audit.get("success_count")
        == audit.get("controller_success_count")
        and audit.get("collision_monitor_intervention_count") == 0
        and audit.get("planned_footprint_collision_sample_count") == 0
        and audit.get("exact_raw_hash_complete_group_count") == 34
        and audit.get("all_methods_successful_group_count") == 34
        and len(records) == 175
    )
    if not required:
        raise RuntimeError("The travel-time execution audit is not clean")
    return records, aggregate


def execution_figures(records, aggregate):
    """Draw travel-time figures from the audited Gazebo execution matrix."""
    methods = list(base.METHODS)
    planners = [
        "NavFnAStar", "NavFnDijkstra", "ThetaStar", "Smac2D", "SmacHybrid"
    ]
    environments = list(base.ENVIRONMENTS)
    labels = [base.METHOD_LABEL[method] for method in methods]
    colors = [base.METHOD_COLOR[method] for method in methods]

    overall = aggregate["overall_by_method"]
    values = [overall[method]["execution_time_s_mean"] for method in methods]
    fig, axis = base.plt.subplots(figsize=(11.5, 5.8))
    bars = axis.bar(labels, values, color=colors)
    axis.set_ylabel("Thời gian di chuyển (s)")
    axis.set_title(
        "Thời gian di chuyển trung bình — 35 cặp môi trường × bộ lập kế hoạch",
        fontweight="bold",
    )
    axis.grid(axis="y", alpha=0.22)
    for bar, value, method in zip(bars, values, methods):
        axis.text(
            bar.get_x() + bar.get_width() / 2, bar.get_height(),
            f'{base.fnum(value, 2)}\n'
            f'{overall[method]["success_count"]}/35 thành công',
            ha="center", va="bottom", fontsize=8,
        )
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_13_execution_overall.png", dpi=180,
        bbox_inches="tight",
    )
    base.plt.close(fig)

    x = base.np.arange(len(planners))
    width = 0.155
    fig, axis = base.plt.subplots(figsize=(15, 6.5))
    for index, method in enumerate(methods):
        planner_values = [
            aggregate["by_planner_and_method"][planner][method][
                "execution_time_s_mean"
            ]
            for planner in planners
        ]
        axis.bar(
            x + (index - 2) * width, planner_values, width,
            label=base.METHOD_LABEL[method], color=base.METHOD_COLOR[method],
        )
    axis.set_xticks(x, planners)
    axis.set_ylabel("Thời gian di chuyển (s)")
    axis.set_title(
        "So sánh theo năm bộ lập kế hoạch toàn cục — trung bình bảy môi trường",
        fontweight="bold",
    )
    axis.legend(ncol=5, fontsize=9, loc="upper center")
    axis.grid(axis="y", alpha=0.22)
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_14_execution_by_planner.png", dpi=180,
        bbox_inches="tight",
    )
    base.plt.close(fig)

    baseline_methods = methods[:-1]
    row_keys = [
        (environment, planner)
        for environment in environments
        for planner in planners
    ]
    matrix = base.np.full(
        (len(row_keys), len(baseline_methods)), base.np.nan, dtype=float
    )
    for row_index, (environment, planner) in enumerate(row_keys):
        selected = [
            record for record in records
            if record["benchmark_environment"] == environment
            and record["planner"] == planner
        ]
        pstmo_record = next(
            record for record in selected if record["method"] == "pstmo"
        )
        for column_index, method in enumerate(baseline_methods):
            baseline_record = next(
                record for record in selected if record["method"] == method
            )
            if (
                pstmo_record["success"] == "True"
                and baseline_record["success"] == "True"
            ):
                matrix[row_index, column_index] = (
                    float(pstmo_record["execution_time_s"])
                    - float(baseline_record["execution_time_s"])
                )
    limit = max(1.0, float(base.np.nanmax(base.np.abs(matrix))))
    color_map = base.plt.get_cmap("RdYlGn_r").copy()
    color_map.set_bad("#64748b")
    fig, axis = base.plt.subplots(figsize=(10.5, 15.5))
    image = axis.imshow(
        matrix, cmap=color_map, vmin=-limit, vmax=limit, aspect="auto"
    )
    axis.set_xticks(
        range(len(baseline_methods)),
        [base.METHOD_LABEL[method] for method in baseline_methods],
    )
    axis.set_yticks(
        range(len(row_keys)),
        [
            f"{base.ENV_LABEL[environment]} — {planner}"
            for environment, planner in row_keys
        ],
        fontsize=8,
    )
    for row_index in range(matrix.shape[0]):
        for column_index in range(matrix.shape[1]):
            value = matrix[row_index, column_index]
            if not base.np.isfinite(value):
                axis.text(
                    column_index, row_index, "Không đạt",
                    ha="center", va="center", fontsize=6, color="white",
                )
                continue
            axis.text(
                column_index, row_index, base.fnum(value, 2),
                ha="center", va="center", fontsize=7,
                color="white" if abs(value) > 0.58 * limit else "#111827",
            )
    axis.set_title(
        "Chênh lệch từng cặp: thời gian PSTMO trừ phương pháp đối chứng (s)",
        fontweight="bold",
    )
    colorbar = fig.colorbar(image, ax=axis, shrink=0.66)
    colorbar.set_label("Âm: PSTMO nhanh hơn; dương: PSTMO chậm hơn (s)")
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_15_execution_pairwise_heatmap.png", dpi=190,
        bbox_inches="tight",
    )
    base.plt.close(fig)

    EXECUTION_MAP_MATRIX_DIR.mkdir(parents=True, exist_ok=True)
    map_matrix_figures = {}
    for environment in environments:
        pixels, extent = base.map_image(environment)
        fig, axes = base.plt.subplots(
            len(planners), len(methods), figsize=(22, 22),
            squeeze=False,
        )
        for planner_index, planner in enumerate(planners):
            for method_index, method in enumerate(methods):
                axis = axes[planner_index, method_index]
                matches = [
                    record for record in records
                    if record["benchmark_environment"] == environment
                    and record["planner"] == planner
                    and record["method"] == method
                ]
                if len(matches) != 1:
                    raise RuntimeError(
                        "Execution matrix is not one-to-one for "
                        f"{environment}/{planner}/{method}: {len(matches)} rows"
                    )
                record = matches[0]
                trial = json.loads(
                    (ROOT / record["trial_json"]).read_text(encoding="utf-8")
                )
                planned = base.np.asarray(
                    trial.get("selected_path_xy") or [], dtype=float
                ).reshape((-1, 2))
                executed = base.np.asarray(
                    trial.get("executed_path_xy") or [], dtype=float
                ).reshape((-1, 2))
                axis.imshow(
                    pixels, cmap="gray", extent=extent, origin="upper",
                    vmin=0, vmax=255,
                )
                if len(planned):
                    axis.plot(
                        planned[:, 0], planned[:, 1], "--", color="#2563eb",
                        linewidth=2.5, zorder=3,
                    )
                    axis.scatter(
                        [planned[0, 0], planned[-1, 0]],
                        [planned[0, 1], planned[-1, 1]],
                        c=["#16a34a", "#7c3aed"], s=58,
                        edgecolors="white", linewidths=0.8, zorder=5,
                    )
                if len(executed):
                    axis.plot(
                        executed[:, 0], executed[:, 1], color="#dc2626",
                        linewidth=3.0, zorder=4,
                    )
                if record["success"] == "True":
                    outcome = f'{float(record["execution_time_s"]):.2f} s'
                    outcome_color = "#065f46"
                else:
                    outcome = "THẤT BẠI"
                    outcome_color = "#991b1b"
                axis.text(
                    0.5, 0.025, outcome, transform=axis.transAxes,
                    ha="center", va="bottom", fontsize=20,
                    fontweight="bold", color=outcome_color,
                    bbox={
                        "facecolor": "white", "edgecolor": outcome_color,
                        "alpha": 0.92, "boxstyle": "round,pad=0.18",
                    },
                    zorder=7,
                )
                if not len(planned):
                    axis.text(
                        0.5, 0.55, "KHÔNG TẠO ĐƯỜNG",
                        transform=axis.transAxes, ha="center", va="center",
                        fontsize=17, fontweight="bold", color="#991b1b",
                        bbox={"facecolor": "white", "alpha": 0.9},
                        zorder=8,
                    )
                if planner_index == 0:
                    axis.set_title(
                        base.METHOD_LABEL[method], fontsize=24,
                        fontweight="bold", pad=10,
                    )
                if method_index == 0:
                    axis.set_ylabel(
                        planner, fontsize=22, fontweight="bold", labelpad=10,
                    )
                else:
                    axis.set_ylabel("")
                axis.set_aspect("equal")
                axis.set_xticks([])
                axis.set_yticks([])
                for spine in axis.spines.values():
                    spine.set_color("#64748b")
                    spine.set_linewidth(1.3)
        fig.suptitle(
            f"{base.ENV_LABEL[environment]}: quỹ đạo thực thi của "
            "5 bộ lập kế hoạch × 5 phương pháp",
            fontsize=28, fontweight="bold", y=0.995,
        )
        fig.text(
            0.5, 0.012,
            "Xanh nét đứt: đường giao cho bộ điều khiển   |   "
            "Đỏ: quỹ đạo Gazebo   |   Xanh lá: điểm đầu   |   "
            "Tím: điểm đích",
            ha="center", fontsize=19, fontweight="bold",
        )
        fig.tight_layout(rect=(0.015, 0.035, 1.0, 0.975), h_pad=1.1, w_pad=0.8)
        output = EXECUTION_MAP_MATRIX_DIR / f"{environment}_matrix_5x5.png"
        fig.savefig(output, dpi=185, bbox_inches="tight", facecolor="white")
        base.plt.close(fig)
        map_matrix_figures[environment] = output

    EXECUTION_FIG_DIR.mkdir(parents=True, exist_ok=True)
    case_figures = {}
    for environment in environments:
        pixels, extent = base.map_image(environment)
        for planner in planners:
            selected_records = [
                record for record in records
                if record["benchmark_environment"] == environment
                and record["planner"] == planner
            ]
            selected_records.sort(
                key=lambda record: methods.index(record["method"])
            )
            fig, axes = base.plt.subplots(2, 3, figsize=(15.8, 9.8))
            for axis, record in zip(axes.ravel()[:5], selected_records):
                trial = json.loads(
                    (ROOT / record["trial_json"]).read_text(encoding="utf-8")
                )
                planned = base.np.asarray(
                    trial.get("selected_path_xy") or [], dtype=float
                ).reshape((-1, 2))
                executed = base.np.asarray(
                    trial.get("executed_path_xy") or [], dtype=float
                ).reshape((-1, 2))
                axis.imshow(
                    pixels, cmap="gray", extent=extent, origin="upper",
                    vmin=0, vmax=255,
                )
                if len(planned):
                    axis.plot(
                        planned[:, 0], planned[:, 1], "--", color="#2563eb",
                        linewidth=1.5, label="Đường giao cho bộ điều khiển",
                    )
                    axis.scatter(
                        [planned[0, 0], planned[-1, 0]],
                        [planned[0, 1], planned[-1, 1]],
                        c=["#16a34a", "#7c3aed"], s=36, zorder=5,
                    )
                if len(executed):
                    axis.plot(
                        executed[:, 0], executed[:, 1], color="#dc2626",
                        linewidth=2.0, label="Quỹ đạo chân thực Gazebo",
                    )
                if not len(planned):
                    axis.text(
                        0.5, 0.5, "Bộ làm mượt không tạo được đường hợp lệ",
                        transform=axis.transAxes, ha="center", va="center",
                        bbox={"facecolor": "white", "alpha": 0.88},
                    )
                axis.set_aspect("equal")
                axis.grid(alpha=0.16)
                if record["success"] == "True":
                    outcome = f'{float(record["execution_time_s"]):.3f} s'
                    detail = (
                        f'RMSE bám {base.fnum(record["tracking_rmse_m"], 3)} m; '
                        f'sai số đích '
                        f'{base.fnum(record["final_position_error_m"], 3)} m'
                    )
                else:
                    outcome = "THẤT BẠI"
                    detail = "Không đạt đích nên không tính thời gian di chuyển"
                axis.set_title(
                    f'{base.METHOD_LABEL[record["method"]]} — {outcome}\n{detail}',
                    fontsize=10, fontweight="bold",
                    color="#991b1b" if record["success"] != "True" else "#111827",
                )
                axis.set_xlabel("x (m)")
                axis.set_ylabel("y (m)")
            legend_axis = axes.ravel()[5]
            legend_axis.axis("off")
            handles, legend_labels = axes.ravel()[0].get_legend_handles_labels()
            legend_axis.legend(handles, legend_labels, loc="upper left")
            legend_axis.text(
                0.02, 0.68,
                "Cùng điểm đầu/đích và cùng SHA-256 đường Raw\n"
                "Mỗi phương pháp chạy trong một Gazebo/Nav2 mới\n"
                "Kết thúc: ground truth vào đích + xe dừng ổn định\n"
                "Chấm xanh: start; chấm tím: goal",
                va="top", fontsize=11, linespacing=1.6,
            )
            fig.suptitle(
                f"{base.ENV_LABEL[environment]} — {planner}: "
                "đường kế hoạch và quỹ đạo chân thực",
                fontsize=15, fontweight="bold",
            )
            fig.tight_layout()
            output = EXECUTION_FIG_DIR / f"{environment}_{planner.lower()}.png"
            fig.savefig(output, dpi=175, bbox_inches="tight")
            base.plt.close(fig)
            case_figures[(environment, planner)] = output
    return case_figures, map_matrix_figures


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
        FIG_DIR / "figure_01_pipeline.png",
        "PSTMO hiện tại — một quy trình xử lý duy nhất",
        [
            "Đường từ bộ\nlập kế hoạch",
            "Điều kiện hóa\nđường gấp khúc",
            "Hai d\nhình học",
            "Tìm α=q/d\nthô–tinh",
            "Hình bao +\nđộng học",
            "Ưu thế thời gian\n+ quy hoạch động",
            "Ghép + kiểm tra\nbất biến cuối",
        ],
        colors=[
            "#334155", "#2563eb", "#0f766e", "#16a34a",
            "#ea580c", "#7c3aed", "#be123c",
        ],
        subtitle=(
            "Sau điều kiện hóa, thuật toán giữ chuỗi điểm neo và tối ưu cục bộ "
            "tại các góc bằng hai d hình học cùng lưới α thô–tinh."
        ),
    )
    base.flow_figure(
        FIG_DIR / "figure_02_hard_gates.png",
        "Chuỗi điều kiện bắt buộc của một đoạn chuyển tiếp",
        [
            "Bézier hữu hạn\n0<α≤0,5",
            "Không đảo dấu κ\nngoài ý muốn",
            "Bánh trong\nkhông quay lùi",
            "Vùng quét hình bao\nkhông va chạm",
            "v, ω, a, aω, ay\nhợp lệ",
            "Không chồng lấn\ntrong quy hoạch động",
            "Đường cuối\nđược quét lại",
        ],
        colors=[
            "#0284c7", "#0e7490", "#0f766e", "#15803d",
            "#ca8a04", "#c2410c", "#b91c1c",
        ],
        subtitle=(
            "Khoảng hở là số đo hậu kiểm; ô gây va chạm, ô chưa biết, ngoài bản đồ và "
            "giao cắt hình bao là điều kiện loại bắt buộc."
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
    axes[0].set_title(f"Đường gốc: {len(raw_xy)} tư thế", fontweight="bold")
    axes[1].plot(
        conditioned[:, 0], conditioned[:, 1], "o-", color="#2563eb", linewidth=2
    )
    axes[1].set_title(
        f"Điều kiện hóa: {len(raw_xy)} → {len(conditioned)} điểm neo", fontweight="bold"
    )
    axes[2].plot(
        conditioned[:, 0], conditioned[:, 1], "o--", color="#94a3b8",
        label="đã điều kiện hóa",
    )
    axes[2].plot(
        preprocessed[:, 0], preprocessed[:, 1], "x-", color="#16a34a",
        linewidth=2.2, label="đầu vào đoạn chuyển tiếp",
    )
    axes[2].set_title("Chuỗi neo đi thẳng vào bước tạo đoạn chuyển tiếp", fontweight="bold")
    axes[2].legend(fontsize=8)
    fig.suptitle(
        "Dữ liệu C01: đầu ra điều kiện hóa là đầu vào của bước tạo đoạn chuyển tiếp",
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
    axes[0].plot(cps[:, 0], cps[:, 1], "o--", color="#f59e0b", label="đa giác điều khiển")
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
    axis.scatter(coarse, base.np.full_like(coarse, 3.0), s=110, color="#2563eb", label="lưới thô")
    axis.scatter(recovery, base.np.full_like(recovery, 2.0), s=90, color="#dc2626", label="phục hồi nếu toàn bộ lưới thô thất bại")
    axis.scatter(fine, base.np.full_like(fine, 1.0), s=70, color="#16a34a", label="ví dụ tinh chỉnh quanh α=0,3")
    for value in coarse:
        axis.text(value, 3.12, f"{value:.1f}", ha="center", fontsize=9)
    axis.annotate(
        "chia [0,2; 0,4] thành 10 khoảng",
        xy=(0.31, 1.0), xytext=(0.34, 1.65),
        arrowprops={"arrowstyle": "->", "color": "#334155"},
    )
    axis.set_xlim(0.06, 0.54)
    axis.set_ylim(0.55, 3.55)
    axis.set_yticks([1, 2, 3], ["Tinh", "Phục hồi", "Thô"])
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
    axes[1].set_title("Giới hạn v từ v, ω, a_y và vận tốc bánh", fontweight="bold")
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_07_kinematic_time_gate.png", dpi=170,
        bbox_inches="tight",
    )
    base.plt.close(fig)

    fig, axis = base.plt.subplots(figsize=(14, 5.4))
    boxes = [
        (0.2, 2.55, 3.8, 1.45, "Rút gọn xa", "Bỏ nhiều neo trung gian\n→ giảm số đoạn\n→ có thể thay đổi mạnh cấu trúc bám"),
        (5.0, 2.55, 3.8, 1.45, "Cấu hình hiện tại", "Giữ chuỗi neo sau điều kiện hóa\n→ chỉ thay lân cận góc bằng đoạn chuyển tiếp\n→ duy trì cấu trúc bám cục bộ"),
    ]
    for x_value, y_value, width, height, title, body in boxes:
        axis.add_patch(base.Rectangle((x_value, y_value), width, height, facecolor="#e0f2fe" if x_value < 1 else "#dcfce7", edgecolor="#334155", linewidth=1.5))
        axis.text(x_value + width/2, y_value + 1.08, title, ha="center", fontweight="bold", fontsize=13)
        axis.text(x_value + width/2, y_value + 0.52, body, ha="center", va="center", fontsize=10)
    axis.annotate("quyết định kiến trúc", xy=(5.0, 3.28), xytext=(4.05, 3.28), arrowprops={"arrowstyle": "->", "linewidth": 2})
    axis.text(4.5, 1.55, "Điều không được suy ra chỉ từ phép thử đường hình học", ha="center", fontweight="bold", color="#b91c1c")
    axis.text(4.5, 0.92, "35 ảnh xác nhận đường hình học; tỷ lệ tới đích phải được đo bằng phép thử thực thi riêng.", ha="center", fontsize=10)
    axis.set_xlim(0, 9)
    axis.set_ylim(0.45, 4.45)
    axis.axis("off")
    axis.set_title("Lựa chọn tiền xử lý: ưu tiên tính tương thích với bộ điều khiển", fontweight="bold", fontsize=15)
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_08_preprocessing_choice.png", dpi=170, bbox_inches="tight"
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
        f"So sánh ghép cặp trên {paired} ca đủ năm phương pháp",
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
    axes[1].set_title("Số điểm trung bình sau điều kiện hóa", fontweight="bold")
    axes[1].grid(axis="y", alpha=0.2)
    axes[2].bar(
        ["Toàn PSTMO"],
        [1000 * statistics.fmean(value["runtime_s"] for value in diagnostics)],
        color=["#0f766e"],
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
                planner_index, env_index, f"{case['case_id']}\nHỢP LỆ",
                ha="center", va="center", fontweight="bold", color="#14532d",
                fontsize=8,
            )
    axis.set_xticks(range(len(base.PLANNERS)), base.PLANNERS)
    axis.set_yticks(
        range(len(base.ENVIRONMENTS)),
        [base.ENV_LABEL[environment] for environment in base.ENVIRONMENTS],
    )
    axis.set_title(
        "Ma trận bằng chứng PSTMO: PNG + JSON + kiểm tra cuối",
        fontweight="bold",
    )
    fig.tight_layout()
    fig.savefig(
        FIG_DIR / "figure_12_test_matrix.png", dpi=170, bbox_inches="tight"
    )
    base.plt.close(fig)
    return case_figures


def build_report(
    items, rows, aggregate, complete_cases, case_figures,
    execution_records, execution_aggregate, execution_case_figures,
    execution_map_matrix_figures,
):
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
        '<div class="cover"><h1>BÁO CÁO TOÀN DIỆN THUẬT TOÁN PSTMO</h1>'
        '<div class="subtitle">Conditioning + hai d hình học + tìm α=q/d thô–tinh '
        '+ time gate + DP + swept-footprint</div>'
        '<div class="meta">Lý thuyết, công thức, mã nguồn, 35 ca hình học RViz2 và 175 lượt di chuyển Gazebo • 03/08/2026</div></div>'
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
        'Cả 35/35 chẩn đoán đều xác nhận <code>preprocessing_mode=condition_only</code>, '
        'một lần thực thi pipeline và kiểm tra cuối thành công.</p>'
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

    content.append('<h2>2. Phạm vi và lựa chọn tiền xử lý</h2>')
    content.append(
        '<p>Tiền xử lý xa nhất có thể bỏ nhiều điểm neo mà bộ điều khiển dùng để '
        'biểu diễn hành lang, hướng tiếp cận hoặc sự đổi hướng từng bước. Tính '
        'hợp lệ hình học của một dây cung dài trên costmap không tự suy ra khả '
        'năng bám ổn định. Vì vậy cấu hình hiện tại giữ chuỗi neo sau điều kiện '
        'hóa và chỉ thay đổi cục bộ quanh các góc.</p>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_08_preprocessing_choice.png",
        "Sơ đồ lý do kiến trúc; đây không phải số liệu thực thi robot.",
    ))
    content.append(
        '<div class="evidence"><b>Hai tầng kiểm thử độc lập.</b> Bộ 35 ca RViz2 '
        'đo hình học trên cùng đường Raw và lưu đầy đủ tọa độ. Bộ 175 lượt Gazebo '
        'thực thi robot thật trong mô phỏng, gồm 7 môi trường × 5 bộ lập kế hoạch '
        '× 5 phương pháp. Thời gian di chuyển được đo từ lúc bộ điều khiển nhận '
        'đường đến lúc robot đạt đích theo tọa độ Gazebo và dừng ổn định.</div>'
    )
    content.append(
        '<p>Adaptive Hybrid không nằm trong so sánh hội nghị này và không bị đổi '
        'hành vi: Pivot nội bộ của Hybrid vẫn dùng <code>condition_only + '
        'legacy_joint_d_q</code>. Chỉ plugin PSTMO độc lập dùng bộ tìm mới.</p>'
    )

    content.append('<h2>3. Pipeline và bất biến</h2>')
    content.append(base.figure_html(
        FIG_DIR / "figure_01_pipeline.png", "Sơ đồ pipeline PSTMO hiện tại."
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

    content.append('<h2>5. Điều kiện hóa: giảm nhiễu nhưng giữ cấu trúc đường</h2>')
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
        'và dây cung vẫn an toàn. Đây là rút gọn cục bộ có ngưỡng lệch '
        'nhỏ để giữ gần đường do bộ lập kế hoạch tạo.</p>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_03_conditioning_actual.png",
        "C01 thật: đầu vào transition trùng đúng đầu ra điều kiện hóa.",
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
        'đổi dấu độ cong, bánh trong, thời gian và hình bao robot như mọi phương án khác.</p>'
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
        '<li>Chỉ phương án đạt tất cả điều kiện bắt buộc mới có Eκ hợp lệ.</li>'
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
    content.append('<div class="eq">r=min(1,C<sub>peak</sub>/252),&nbsp; g=min(1,|ω|<sub>max</sub>/0,80),&nbsp; u=Eκ/(Eκ+1)</div>')
    content.append('<div class="eq">J=(0,15·r+0,10·g+0,75·u)/(0,15+0,10+0,75)</div>')
    content.append('<p>Trong đó r là điểm rủi ro theo costmap, g là điểm vận tốc góc và u là điểm tích phân bình phương độ cong đã chuẩn hóa.</p>')
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
        '<div class="warning"><b>Khoảng xoay đầu chưa nằm trong kiểm tra cuối.</b> '
        'Code đặt yaw pose đầu output theo hướng cạnh đầu. Phép xoay '
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
        FIG_DIR / "figure_12_test_matrix.png", "35 ô đều có output PSTMO và qua kiểm tra cuối."
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
        'Log và SHA-256 được lưu trong <code>bang_chung_loi_C30_simple.json</code>.</div>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_10_success_points_runtime.png",
        "Thành công, mức giảm điểm do điều kiện hóa và thời gian chạy nội bộ.",
    ))

    content.append('<h2>20. Phân tích kết quả tổng hợp</h2>')
    content.append(
        f'<p>Trên {paired} nhóm đầy đủ, PSTMO có L='
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
        'điểm neo trước khi tối ưu transition.</p>'
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

    content.append('<div class="page-break"></div><h2>21. Thời gian robot di chuyển trong Gazebo</h2>')
    content.append(
        '<p>Đây là <b>thời gian di chuyển</b>, không phải thời gian tính đường '
        'hay thời gian chạy bộ làm mượt. Đồng hồ bắt đầu khi action FollowPath '
        'được bộ điều khiển chấp nhận và kết thúc khi tọa độ chân thực của Gazebo '
        'đã vào miền đích, sai số hướng đạt yêu cầu và robot dừng ổn định. Mỗi '
        'lượt khởi động một mô phỏng Gazebo/Nav2 mới để trạng thái bộ điều khiển '
        'không truyền từ phương pháp trước sang phương pháp sau.</p>'
    )
    execution_audit = execution_aggregate["audit"]
    content.append(
        f'<div class="evidence"><b>Kết quả kiểm toán.</b> '
        f'{execution_audit["success_count"]}/{execution_audit["trial_count"]} '
        'lượt thành công và mọi lượt thành công đều đạt đích theo ground truth '
        'rồi dừng ổn định; '
        f'{execution_audit["collision_monitor_intervention_count"]} lần Collision '
        'Monitor can thiệp; '
        f'{execution_audit["planned_footprint_collision_sample_count"]} mẫu '
        'footprint kế hoạch va chạm. Có '
        f'{execution_audit["all_methods_successful_group_count"]}/35 nhóm mà '
        'cả năm phương pháp đều hoàn tất hành trình; nhóm còn lại được giữ nguyên '
        'trong báo cáo dưới dạng thất bại, không thay điểm đặt.</div>'
    )
    content.append(
        f'<p><b>Đối chiếu đường đầu vào.</b> '
        f'{execution_audit["exact_raw_hash_complete_group_count"]}/35 nhóm có đủ '
        'năm mã SHA-256 đường Raw trùng nhau. Ở nhóm thất bại Kho điều phối–'
        'SmacHybrid, bốn phương pháp có cùng mã băm; Simple bị từ chối trước khi '
        'bộ ghi thực thi nhận được đường nên JSON của riêng lượt đó không có mã '
        'băm. Nhóm này vẫn dùng cấu hình điểm đầu–đích đã khóa nhưng không được '
        'coi là nhóm ghép cặp hoàn chỉnh để tính trung bình.</p>'
    )
    content.append(
        '<div class="warning"><b>Ca Warehouse Dispatch–Smac Hybrid.</b> Cả '
        'năm phương pháp đều không hoàn tất. Raw, Savitzky–Golay, Constrained '
        'và PSTMO tạo đường không có mẫu footprint tĩnh va chạm, nhưng '
        'Regulated Pure Pursuit dự báo va chạm phía trước ngay khi xuất phát và '
        'hết thời gian kiên nhẫn; Simple bị smoother_server loại tại '
        'x=−4,741290 m, y=3,482165 m. Báo cáo giữ nguyên ca này để thể hiện '
        'giới hạn kết hợp planner–smoother–controller, không đổi điểm đặt và '
        'không dùng thời gian dừng sớm để làm đẹp trung bình.</div>'
    )
    content.append(base.figure_html(
        FIG_DIR / "figure_13_execution_overall.png",
        "Thời gian di chuyển trung bình trên đủ 35 cặp môi trường–bộ lập kế hoạch.",
    ))
    execution_overall_rows = []
    for method in base.METHODS:
        stats = execution_aggregate["overall_by_method"][method]
        execution_overall_rows.append([
            f"<b>{base.METHOD_LABEL[method]}</b>",
            f'{stats["success_count"]}/{stats["trial_count"]}',
            str(stats["successful_time_sample_count"]),
            base.fnum(stats["execution_time_s_mean"], 3),
            base.fnum(stats["execution_time_s_stdev"], 3),
            base.fnum(stats["execution_time_s_min"], 3),
            base.fnum(stats["execution_time_s_max"], 3),
        ])
    content.append(base.table_html(
        ["Phương pháp", "Thành công", "Số thời gian hợp lệ", "TB (s)", "Độ lệch chuẩn (s)", "Nhỏ nhất (s)", "Lớn nhất (s)"],
        execution_overall_rows, "compact",
    ))
    content.append(
        '<div class="note"><b>Giới hạn thống kê.</b> Mỗi ô '
        'môi trường–planner–phương pháp mới chạy một lần. Độ lệch chuẩn trong '
        'bảng mô tả sự khác nhau giữa 35 tuyến, không phải độ dao động khi lặp '
        'lại cùng một tuyến; vì vậy báo cáo không tuyên bố ý nghĩa thống kê.</div>'
    )

    content.append('<h3>21.1. So sánh theo năm bộ lập kế hoạch toàn cục</h3>')
    content.append(base.figure_html(
        FIG_DIR / "figure_14_execution_by_planner.png",
        "Mỗi cột là trung bình bảy môi trường của cùng một bộ lập kế hoạch.",
    ))
    planner_order = [
        "NavFnAStar", "NavFnDijkstra", "ThetaStar", "Smac2D", "SmacHybrid"
    ]
    planner_time_rows = []
    for planner in planner_order:
        by_method = execution_aggregate["by_planner_and_method"][planner]
        planner_time_rows.append(
            [planner]
            + [
                f'{base.fnum(by_method[method]["execution_time_s_mean"], 3)} '
                f'({by_method[method]["success_count"]}/7)'
                for method in base.METHODS
            ]
        )
    content.append(base.table_html(
        ["Bộ lập kế hoạch", "Raw", "Simple", "Savitzky–Golay", "Constrained", "PSTMO"],
        planner_time_rows, "compact",
    ))

    content.append('<h3>21.2. So sánh theo bảy môi trường</h3>')
    environment_time_rows = []
    for environment in base.ENVIRONMENTS:
        by_method = execution_aggregate["by_environment_and_method"][environment]
        environment_time_rows.append(
            [base.ENV_LABEL[environment]]
            + [
                f'{base.fnum(by_method[method]["execution_time_s_mean"], 3)} '
                f'({by_method[method]["success_count"]}/5)'
                for method in base.METHODS
            ]
        )
    content.append(base.table_html(
        ["Môi trường", "Raw", "Simple", "Savitzky–Golay", "Constrained", "PSTMO"],
        environment_time_rows, "compact",
    ))
    content.append(
        '<p>Trong hai bảng trên, số trong ngoặc là số lượt thành công dùng để '
        'tính trung bình. Thời gian của lượt thất bại không được coi là hành '
        'trình nhanh và không được đưa vào trung bình.</p>'
    )

    content.append('<h3>21.3. So sánh ghép cặp PSTMO với từng đối chứng</h3>')
    paired_rows = []
    for method in base.METHODS[:-1]:
        comparison = execution_aggregate["pstmo_paired_comparison"][method]
        paired_rows.append([
            base.METHOD_LABEL[method],
            base.fnum(comparison["paired_difference_s_mean"], 3),
            base.fnum(comparison["paired_relative_change_percent_mean"], 2) + "%",
            f'{comparison["pstmo_faster_pair_count"]}/{comparison["pair_count"]}',
            f'{comparison["pstmo_slower_pair_count"]}/{comparison["pair_count"]}',
        ])
    content.append(base.table_html(
        ["PSTMO so với", "TB PSTMO−đối chứng (s)", "TB thay đổi ghép cặp", "PSTMO nhanh hơn", "PSTMO chậm hơn"],
        paired_rows, "compact",
    ))
    content.append(base.figure_html(
        FIG_DIR / "figure_15_execution_pairwise_heatmap.png",
        "Bản đồ nhiệt cho thấy đầy đủ 140 phép so sánh cặp, không chỉ giá trị trung bình.",
    ))

    content.append('<h3>21.4. Bảng thời gian đầy đủ của 35 nhóm ghép cặp</h3>')
    detailed_execution_rows = []
    for environment in base.ENVIRONMENTS:
        for planner in planner_order:
            selected = [
                record for record in execution_records
                if record["benchmark_environment"] == environment
                and record["planner"] == planner
            ]
            detailed_execution_rows.append(
                [base.ENV_LABEL[environment], planner]
                + [
                    (
                        base.fnum(float(method_record["execution_time_s"]), 3)
                        if method_record["success"] == "True"
                        else "THẤT BẠI"
                    )
                    for method in base.METHODS
                    for method_record in [next(
                        record for record in selected
                        if record["method"] == method
                    )]
                ]
            )
    content.append(base.table_html(
        ["Môi trường", "Planner", "Raw", "Simple", "Savitzky–Golay", "Constrained", "PSTMO"],
        detailed_execution_rows, "tiny",
    ))
    content.append('<h3>21.5. Tổng quan trực quan theo từng bản đồ</h3>')
    content.append(
        '<p>Bảy ma trận dưới đây cho phép đối chiếu trực tiếp trên từng bản đồ. '
        'Mỗi hàng là một bộ lập kế hoạch toàn cục, mỗi cột là một phương pháp '
        'làm mượt; vì vậy một hình chứa đủ 25 lượt thực thi độc lập. Số trong '
        'từng ô là thời gian robot thực sự di chuyển đến đích. Ô '
        '<b>THẤT BẠI</b> không được đưa vào trung bình thời gian.</p>'
    )
    for environment in base.ENVIRONMENTS:
        content.append(base.figure_html(
            execution_map_matrix_figures[environment],
            f'{base.ENV_LABEL[environment]}: ma trận đầy đủ 5 bộ lập kế hoạch '
            '× 5 phương pháp trên cùng bản đồ và cùng cặp điểm đầu–đích.',
            "case-figure",
        ))

    content.append('<h3>21.6. Hình chi tiết của toàn bộ 35 nhóm thực thi</h3>')
    content.append(
        '<p>Mỗi hình dưới đây dùng trực tiếp <code>selected_path_xy</code> và '
        '<code>executed_path_xy</code> trong JSON của năm lượt Gazebo. Đường xanh '
        'nét đứt là đường Nav2 giao cho bộ điều khiển; đường đỏ là quỹ đạo chân '
        'thực của thân xe trong Gazebo. Vì vậy hình thể hiện được cả hình dạng '
        'đường và sai khác khi robot thực sự bám đường.</p>'
    )
    for environment in base.ENVIRONMENTS:
        content.append(f'<h4>{base.ENV_LABEL[environment]}</h4>')
        for planner in planner_order:
            content.append(base.figure_html(
                execution_case_figures[(environment, planner)],
                f'{base.ENV_LABEL[environment]} — {planner}: đủ năm phương pháp, '
                'cùng cấu hình điểm đầu–đích; trạng thái thất bại được giữ nguyên.',
                "case-figure",
            ))

    content.append('<div class="page-break"></div><h2>22. Bằng chứng từng môi trường và planner</h2>')
    for environment in base.ENVIRONMENTS:
        section = base.ENVIRONMENTS.index(environment) + 1
        content.append(
            f'<h3>22.{section}. {base.ENV_LABEL[environment]}</h3>'
            f'<p>{CURRENT_ENV_DESCRIPTION[environment]}</p>'
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
                f'{diag["conditioning_output_points"]} điểm neo; '
                f'{diag["g2_transitions"]} transition, {diag["pivots"]} pivot, '
                f'{diag["evaluations"]} đánh giá, {diag["dp_states"]} DP states, '
                f'runtime nội bộ {1000*diag["runtime_s"]:.2f} ms.</p>'
            )

    content.append('<div class="page-break"></div><h2>23. Bảng đầy đủ 35 ca hình học PSTMO</h2>')
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

    content.append('<h2>24. Kiểm thử phần mềm và giới hạn</h2>')
    content.append(
        '<ul><li>Build thành công năm gói PSTMO, Nav2 plugin, benchmark, RViz2 '
        'và Gazebo. Kết quả tổng hợp: 308 test, 0 error, 0 failure và 40 skip '
        'của cppcheck/static analysis theo môi trường.</li><li>35/35 ca hình học '
        'PSTMO dùng đúng một pipeline và qua kiểm tra cuối.</li><li>Đây là 35 ca đại diện, không phủ '
        'toàn bộ không gian start–goal.</li><li>PGM tĩnh không mô hình hóa sai số '
        'localization, trượt bánh, tải, độ trễ controller hay vật cản động.</li>'
        '<li>Hình RViz/Gazebo chứng minh dữ liệu path nhưng không thay robot thật.</li>'
        '<li>PSTMO tối ưu rời rạc trên hai d và lưới α; không chứng minh optimum '
        'liên tục toàn cục.</li><li>Khi condition_only, yaw đầu output theo cạnh '
        'đầu; cần bổ sung kiểm tra xoay từ yaw robot hiện tại nếu dùng trong hành '
        'lang sát vật cản.</li><li>Bộ thực thi hiện có một lần lặp cho mỗi ô; '
        'cần tăng số lần lặp và thử robot thật trước khi suy rộng kết luận.</li></ul>'
    )

    content.append('<h2>25. Tái lập và tệp bằng chứng</h2>')
    content.append(
        '<ol><li>Build workspace và source <code>install/setup.bash</code>.</li>'
        '<li>Launch từng world với start pose đủ số thực x,y,yaw.</li>'
        '<li>Gọi <code>capture_pstmo_rviz_evidence.py '
        '--expected-preprocessing condition_only</code> cho từng planner.</li>'
        '<li>Xác minh chế độ tiền xử lý, chế độ tìm kiếm, số lần chạy pipeline '
        'và kiểm tra cuối trước khi nhận ca.</li><li>Chạy script này để '
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
        '<ul><li><code>pstmo_bao_cao_toan_dien_assets/rviz_cases/</code>: '
        '35 PNG + 35 JSON exact paths.</li><li><code>gazebo/</code>: 7 ảnh world.</li>'
        '<li><code>figures/</code>: sơ đồ lý thuyết, chart và 35 composite.</li>'
        '<li><code>execution_map_matrices/</code>: 7 ma trận trực quan, mỗi ma trận '
        'chứa đủ 5 planner × 5 phương pháp trên một bản đồ.</li>'
        '<li><code>execution_cases/</code>: 35 hình thực thi chi tiết; mỗi hình '
        'đối chiếu năm đường kế hoạch với năm quỹ đạo Gazebo.</li>'
        '<li><code>benchmark_hinh_hoc_175_luot.csv</code>: 175 hàng '
        'case×method, gồm thất bại.</li><li><code>benchmark_hinh_hoc_tong_hop.json</code>: '
        'thống kê ghép cặp và diagnostics.</li><li><code>execution_175_cases.csv</code> '
        'và <code>execution_aggregate_5planners_7env.json</code>: toàn bộ dữ liệu '
        'di chuyển đã kiểm toán.</li><li><code>bang_chung_loi_C30_simple.json</code>: '
        'log va chạm của đối chứng Simple trong phép thử hình học.</li></ul>'
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
        '<title>Báo cáo toàn diện thuật toán PSTMO</title><style>'
        + style + '</style></head><body>' + ''.join(content) + '</body></html>'
    )
    OUTPUT_HTML.write_text(document, encoding="utf-8")


def main():
    prepare_assets()
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    configure_base_paths()
    items = load_evidence()
    rows = base.build_rows(items)
    aggregate, complete_cases = base.aggregate_rows(rows)
    base.persist_data(rows, aggregate, items)
    algorithm_figures(items)
    case_figures = result_figures(items, rows, aggregate)
    execution_records, execution_aggregate = load_execution_evidence()
    execution_case_figures, execution_map_matrix_figures = execution_figures(
        execution_records, execution_aggregate
    )
    build_report(
        items, rows, aggregate, complete_cases, case_figures,
        execution_records, execution_aggregate, execution_case_figures,
        execution_map_matrix_figures,
    )
    print(json.dumps({
        "html": str(OUTPUT_HTML),
        "csv": str(OUTPUT_CSV),
        "json": str(OUTPUT_JSON),
        "cases": len(items),
        "case_figures": len(case_figures),
        "complete_groups": len(complete_cases),
        "execution_trials": len(execution_records),
        "execution_map_matrices": len(execution_map_matrix_figures),
    }, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
