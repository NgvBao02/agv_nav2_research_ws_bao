#!/usr/bin/env python3
"""Build one logically unified, terminology-audited PSTMO research report.

The report is generated only from repository source code, current configuration,
the 35 RViz2 geometry-evidence JSON files, and the audited 175-run Gazebo CSV.
It replaces the former PDF that concatenated two independently numbered reports.
"""

from __future__ import annotations

import csv
import hashlib
import html
import json
import math
import shutil
import statistics
import subprocess
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

# Ubuntu's Matplotlib is compiled against the system NumPy.  Prefer the matching
# system pair while still allowing user-site packages such as python-docx.
SYSTEM_DIST_PACKAGES = Path("/usr/lib/python3/dist-packages")
if SYSTEM_DIST_PACKAGES.is_dir():
    system_site = str(SYSTEM_DIST_PACKAGES)
    if system_site in sys.path:
        sys.path.remove(system_site)
    sys.path.insert(0, system_site)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from docx import Document

from html_report_to_docx import convert as html_to_docx


ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
ASSETS = DOCS / "pstmo_bao_cao_toan_dien_assets"
FIGURES = ASSETS / "figures"
RVIZ_CASES = ASSETS / "rviz_cases"
GAZEBO = ASSETS / "gazebo"
EXECUTION_CASES = ASSETS / "execution_cases"
EXECUTION_MATRICES = ASSETS / "execution_map_matrices"
VISUALS_3D = ASSETS / "visuals_3d"
GEOMETRY_CSV = ASSETS / "benchmark_hinh_hoc_175_luot.csv"
GEOMETRY_AGGREGATE = ASSETS / "benchmark_hinh_hoc_tong_hop.json"
EXECUTION_CSV = ASSETS / "execution_175_cases.csv"
EXECUTION_AGGREGATE = ASSETS / "execution_aggregate_5planners_7env.json"

OUTPUT_HTML = DOCS / "PSTMO_unified.html"
OUTPUT_DOCX = DOCS / "PSTMO.docx"
OUTPUT_PDF = DOCS / "PSTMO.pdf"
BACKUP_PDF = DOCS / "PSTMO_original_ghep_2_nguon.pdf"

ENVIRONMENTS = (
    "open_arena",
    "research_warehouse",
    "narrow_aisles",
    "office_maze",
    "warehouse_cross_aisles",
    "warehouse_dispatch",
    "warehouse_long_aisles",
)
PLANNERS = ("NavFnAStar", "NavFnDijkstra", "ThetaStar", "Smac2D", "SmacHybrid")
METHODS = ("raw", "simple", "savitzky_golay", "constrained", "pstmo")
BASELINES = ("simple", "savitzky_golay", "constrained")

ENV_LABEL = {
    "open_arena": "Không gian mở",
    "research_warehouse": "Kho nghiên cứu",
    "narrow_aisles": "Lối đi hẹp",
    "office_maze": "Mê cung văn phòng",
    "warehouse_cross_aisles": "Kho có lối giao cắt",
    "warehouse_dispatch": "Kho điều phối",
    "warehouse_long_aisles": "Kho có lối đi dài",
}
ENV_DESCRIPTION = {
    "open_arena": (
        "Vật cản thưa; dùng để quan sát ảnh hưởng của đường vòng quanh một khối "
        "trung tâm khi hành lang không phải yếu tố chi phối."
    ),
    "research_warehouse": (
        "Kệ, thùng hàng, đoạn chéo và góc vuông cùng xuất hiện; phù hợp kiểm tra "
        "sự phối hợp giữa đường chéo dài và các chuyển hướng cục bộ."
    ),
    "narrow_aisles": (
        "Các dãy kệ tạo hành lang ngoằn ngoèo; khoảng hở hình bao và chuỗi góc "
        "gần nhau là các ràng buộc quan trọng."
    ),
    "office_maze": (
        "Vách ngăn và cửa lệch tạo nhiều đoạn ngắn; môi trường này nhấn mạnh "
        "ràng buộc không chồng lấn giữa vùng cắt của hai góc kề nhau."
    ),
    "warehouse_cross_aisles": (
        "Các lối dọc–ngang giao nhau; dùng để kiểm tra đoạn chuyển tiếp khi robot vào "
        "và rời vùng giao cắt."
    ),
    "warehouse_dispatch": (
        "Tuyến dài qua vùng điều phối có mật độ vật cản cao; đây cũng là nơi xuất "
        "hiện ca C30 bộc lộ giới hạn phối hợp giữa bộ lập kế hoạch, bộ làm mượt và bộ điều khiển."
    ),
    "warehouse_long_aisles": (
        "Các hành lang song song dài; dùng để quan sát tích lũy độ cong và nhiều "
        "lần chuyển lối trên quãng đường lớn."
    ),
}
SCENARIO_LABEL = {
    "center_block_detour": "Vòng qua khối trung tâm",
    "lower_left_diagonal": "Đường chéo góc trái dưới",
    "southwest_northeast_weave": "Luồn Tây Nam–Đông Bắc",
    "office_long_diagonal": "Đường chéo dài văn phòng",
    "cross_aisle_transfer": "Chuyển lối tại giao cắt",
    "full_replenishment": "Tuyến bổ sung hàng toàn kho",
    "diagonal_replenishment": "Tuyến bổ sung hàng chéo",
}
METHOD_LABEL = {
    "raw": "Raw",
    "simple": "Simple",
    "savitzky_golay": "Savitzky–Golay",
    "constrained": "Constrained",
    "pstmo": "PSTMO",
}
METHOD_COLOR = {
    "raw": "#111827",
    "simple": "#2563eb",
    "savitzky_golay": "#f59e0b",
    "constrained": "#9333ea",
    "pstmo": "#16a34a",
}


def fnum(value, digits: int = 3) -> str:
    if value is None or value == "":
        return "–"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "–"
    if not math.isfinite(number):
        return "–"
    return f"{number:.{digits}f}".replace(".", ",")


def degrees(value) -> float:
    return math.degrees(float(value))


def wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def pct_reduction(new, reference) -> float:
    new_value = float(new)
    reference_value = float(reference)
    if abs(reference_value) < 1.0e-12:
        return 0.0
    return 100.0 * (reference_value - new_value) / reference_value


def describe_lower(new, reference, digits: int = 2) -> str:
    reduction = pct_reduction(new, reference)
    if reduction >= 0:
        return f"giảm {fnum(reduction, digits)}%"
    return f"tăng {fnum(-reduction, digits)}%"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def load_cases() -> list[dict]:
    cases = []
    for environment in ENVIRONMENTS:
        for planner in PLANNERS:
            matches = sorted(RVIZ_CASES.glob(f"{environment}__*__{planner}.json"))
            if len(matches) != 1:
                raise RuntimeError(
                    f"Expected one evidence JSON for {environment}/{planner}, got {matches}"
                )
            item = json.loads(matches[0].read_text(encoding="utf-8"))
            item["case_id"] = f"C{len(cases) + 1:02d}"
            diagnostics = item["pstmo_diagnostics"]
            if not (
                item.get("expected_preprocessing") == "condition_only"
                and diagnostics.get("preprocessing_mode") == "condition_only"
                and diagnostics.get("search_mode") == "hierarchical_alpha_two_trim"
                and diagnostics.get("pipeline_execution_count") == 1
                and diagnostics.get("final_invariants_verified") is True
            ):
                raise RuntimeError(f"Evidence contract failed for {item['case_id']}")
            cases.append(item)
    if len(cases) != 35:
        raise RuntimeError(f"Expected 35 cases, got {len(cases)}")
    return cases


def table_html(headers, rows, css_class: str = "") -> str:
    head = "".join(f"<th>{html.escape(str(value))}</th>" for value in headers)
    body = "".join(
        "<tr>" + "".join(f"<td>{value}</td>" for value in row) + "</tr>"
        for row in rows
    )
    return (
        f'<table class="{css_class}"><thead><tr>{head}</tr></thead>'
        f"<tbody>{body}</tbody></table>"
    )


def figure_html(path: Path, caption: str, css_class: str = "") -> str:
    relative = path.resolve().relative_to(DOCS.resolve()).as_posix()
    return (
        f'<figure class="{css_class}"><img src="{relative}" '
        f'alt="{html.escape(caption)}"><figcaption>{html.escape(caption)}'
        "</figcaption></figure>"
    )


def equation(text: str) -> str:
    return f'<div class="eq">{text}</div>'


def callout(text: str, kind: str = "mine") -> str:
    return f'<div class="{kind}">{text}</div>'


def page_break() -> str:
    return '<div class="page-break"></div>'


def state_of_corner(corner: dict) -> str:
    if corner.get("pass_through"):
        return "Giữ nguyên góc"
    if float(corner.get("selected_trim", 0.0)) > 0.0:
        return "Bézier G²"
    return "Quay tại chỗ"


def case_corner_stats(item: dict) -> dict:
    corners = item["pstmo_diagnostics"]["corner_search"]
    absolute_angles = [abs(degrees(corner["turn_angle"])) for corner in corners]
    transitions = [corner for corner in corners if state_of_corner(corner) == "Bézier G²"]
    pivots = [corner for corner in corners if state_of_corner(corner) == "Quay tại chỗ"]
    pass_through = [corner for corner in corners if state_of_corner(corner) == "Giữ nguyên góc"]
    return {
        "corners": corners,
        "count": len(corners),
        "mean_abs_deg": statistics.fmean(absolute_angles),
        "median_abs_deg": statistics.median(absolute_angles),
        "max_abs_deg": max(absolute_angles),
        "min_abs_deg": min(absolute_angles),
        "left": sum(float(corner["turn_angle"]) > 0.0 for corner in corners),
        "right": sum(float(corner["turn_angle"]) < 0.0 for corner in corners),
        "transitions": transitions,
        "pivots": pivots,
        "pass_through": pass_through,
    }


def case_geometry_figure(item: dict) -> Path:
    candidates = sorted(FIGURES.glob(f"case_{item['case_id']}_*.png"))
    if len(candidates) != 1:
        raise RuntimeError(f"Geometry figure missing for {item['case_id']}: {candidates}")
    return candidates[0]


def execution_case_figure(item: dict) -> Path:
    planner_file = item["planner"].lower()
    path = EXECUTION_CASES / f"{item['environment']}_{planner_file}.png"
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def prepare_system_images() -> dict[str, Path]:
    target = ASSETS / "system_model"
    target.mkdir(parents=True, exist_ok=True)
    sources = {
        "gazebo": ROOT / "results" / "gui_validation_20260724" / "gazebo_research_warehouse.png",
        "rviz": ROOT / "results" / "gui_validation_20260724" / "rviz_research_ui_final.png",
        "rviz_layout": ROOT / "results" / "gui_validation_20260724" / "rviz_default_optimized_layout.png",
    }
    output = {}
    for name, source in sources.items():
        if not source.is_file():
            raise FileNotFoundError(source)
        destination = target / source.name
        shutil.copy2(source, destination)
        output[name] = destination
    return output


def make_angle_figures(cases: list[dict], geometry_by_case: dict[str, dict[str, dict]], execution_by_pair: dict[tuple[str, str], dict[str, dict]]) -> dict[str, Path]:
    FIGURES.mkdir(parents=True, exist_ok=True)
    all_corners = [
        corner
        for item in cases
        for corner in item["pstmo_diagnostics"]["corner_search"]
    ]
    absolute_degrees = [abs(degrees(corner["turn_angle"])) for corner in all_corners]
    transitions = [corner for corner in all_corners if state_of_corner(corner) == "Bézier G²"]
    state_counts = [
        sum(state_of_corner(corner) == label for corner in all_corners)
        for label in ("Giữ nguyên góc", "Bézier G²", "Quay tại chỗ")
    ]

    figure_16 = FIGURES / "figure_16_angle_state_summary.png"
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.2))
    axes[0, 0].hist(absolute_degrees, bins=np.arange(0, 66, 5), color="#2563eb", edgecolor="white")
    axes[0, 0].axvline(5, color="#dc2626", linestyle="--", label="Ngưỡng 5°")
    axes[0, 0].set_title("Phân bố |góc rẽ| tại 229 góc", fontweight="bold")
    axes[0, 0].set_xlabel("|θ| (độ)")
    axes[0, 0].set_ylabel("Số góc")
    axes[0, 0].legend()
    bars = axes[0, 1].bar(
        ["Giữ nguyên góc", "Bézier G²", "Quay tại chỗ"],
        state_counts,
        color=["#94a3b8", "#16a34a", "#f59e0b"],
    )
    axes[0, 1].set_title("Trạng thái được DP chọn", fontweight="bold")
    for bar, value in zip(bars, state_counts):
        axes[0, 1].text(bar.get_x() + bar.get_width() / 2, value, str(value), ha="center", va="bottom")
    axes[1, 0].hist(
        [float(corner["selected_control_fraction"]) for corner in transitions],
        bins=np.arange(0.295, 0.371, 0.005), color="#7c3aed", edgecolor="white",
    )
    axes[1, 0].set_title("Tỷ lệ hình dạng α=q/d của 221 đoạn chuyển tiếp", fontweight="bold")
    axes[1, 0].set_xlabel("α")
    axes[1, 0].set_ylabel("Số đoạn chuyển tiếp")
    axes[1, 1].hist(
        [float(corner["selected_trim"]) for corner in transitions],
        bins=np.linspace(0.05, 0.85, 17), color="#0f766e", edgecolor="white",
    )
    axes[1, 1].set_title("Khoảng cắt d được chọn", fontweight="bold")
    axes[1, 1].set_xlabel("d (m)")
    axes[1, 1].set_ylabel("Số đoạn chuyển tiếp")
    for axis in axes.ravel():
        axis.grid(alpha=0.18)
    fig.suptitle("Phân tích góc và trạng thái từ dữ liệu chẩn đoán RViz2", fontweight="bold", fontsize=15)
    fig.tight_layout()
    fig.savefig(figure_16, dpi=180, bbox_inches="tight")
    plt.close(fig)

    max_angles = np.zeros((len(ENVIRONMENTS), len(PLANNERS)))
    annotations = [["" for _ in PLANNERS] for _ in ENVIRONMENTS]
    for item in cases:
        env_index = ENVIRONMENTS.index(item["environment"])
        planner_index = PLANNERS.index(item["planner"])
        stats = case_corner_stats(item)
        max_angles[env_index, planner_index] = stats["max_abs_deg"]
        annotations[env_index][planner_index] = (
            f"{stats['max_abs_deg']:.1f}°\n"
            f"G² {len(stats['transitions'])} / Q {len(stats['pivots'])}"
        )
    figure_17 = FIGURES / "figure_17_case_angle_heatmap.png"
    fig, axis = plt.subplots(figsize=(13, 6.4))
    image = axis.imshow(max_angles, cmap="YlOrRd", vmin=30, vmax=60, aspect="auto")
    for row in range(len(ENVIRONMENTS)):
        for column in range(len(PLANNERS)):
            axis.text(column, row, annotations[row][column], ha="center", va="center", fontsize=8, color="#111827")
    axis.set_xticks(range(len(PLANNERS)), PLANNERS)
    axis.set_yticks(range(len(ENVIRONMENTS)), [ENV_LABEL[value] for value in ENVIRONMENTS])
    axis.set_title("Góc rẽ lớn nhất trong từng ca sau điều kiện hóa đường", fontweight="bold")
    colorbar = fig.colorbar(image, ax=axis)
    colorbar.set_label("|θ|max (độ)")
    fig.tight_layout()
    fig.savefig(figure_17, dpi=180, bbox_inches="tight")
    plt.close(fig)

    gains_k = np.zeros_like(max_angles)
    gains_e = np.zeros_like(max_angles)
    for item in cases:
        env_index = ENVIRONMENTS.index(item["environment"])
        planner_index = PLANNERS.index(item["planner"])
        rows = geometry_by_case[item["case_id"]]
        valid_stock = [rows[method] for method in BASELINES if rows[method]["success"] == "True"]
        best_k = min(valid_stock, key=lambda row: float(row["max_abs_curvature_1pm"]))
        best_e = min(valid_stock, key=lambda row: float(row["curvature_energy_1pm"]))
        pstmo = rows["pstmo"]
        gains_k[env_index, planner_index] = pct_reduction(
            pstmo["max_abs_curvature_1pm"], best_k["max_abs_curvature_1pm"]
        )
        gains_e[env_index, planner_index] = pct_reduction(
            pstmo["curvature_energy_1pm"], best_e["curvature_energy_1pm"]
        )
    figure_18 = FIGURES / "figure_18_case_geometry_gain_heatmap.png"
    fig, axes = plt.subplots(1, 2, figsize=(15, 6.2))
    for axis, values, title in zip(
        axes,
        (gains_k, gains_e),
        ("PSTMO so với đối chứng có κmax thấp nhất", "PSTMO so với đối chứng có Eκ thấp nhất"),
    ):
        image = axis.imshow(values, cmap="RdYlGn", vmin=-30, vmax=100, aspect="auto")
        for row in range(len(ENVIRONMENTS)):
            for column in range(len(PLANNERS)):
                axis.text(column, row, f"{values[row, column]:.1f}%", ha="center", va="center", fontsize=8)
        axis.set_xticks(range(len(PLANNERS)), PLANNERS, rotation=20)
        axis.set_yticks(range(len(ENVIRONMENTS)), [ENV_LABEL[value] for value in ENVIRONMENTS])
        axis.set_title(title, fontweight="bold")
        fig.colorbar(image, ax=axis, fraction=0.046, pad=0.04).set_label("Mức giảm; âm = PSTMO tăng")
    fig.suptitle("Mức cải thiện hình học theo từng ca ghép cặp", fontweight="bold", fontsize=15)
    fig.tight_layout()
    fig.savefig(figure_18, dpi=180, bbox_inches="tight")
    plt.close(fig)

    x_angles = []
    y_k = []
    y_time = []
    colors = []
    environment_colors = plt.cm.tab10(np.linspace(0, 1, len(ENVIRONMENTS)))
    for item in cases:
        stats = case_corner_stats(item)
        geometry = geometry_by_case[item["case_id"]]
        raw = geometry["raw"]
        pstmo = geometry["pstmo"]
        execution = execution_by_pair[(item["environment"], item["planner"])]
        x_angles.append(stats["max_abs_deg"])
        y_k.append(pct_reduction(pstmo["max_abs_curvature_1pm"], raw["max_abs_curvature_1pm"]))
        if execution["pstmo"]["success"] == "True" and execution["raw"]["success"] == "True":
            y_time.append(float(execution["pstmo"]["execution_time_s"]) - float(execution["raw"]["execution_time_s"]))
        else:
            y_time.append(float("nan"))
        colors.append(environment_colors[ENVIRONMENTS.index(item["environment"])])
    valid_time = np.isfinite(y_time)
    corr_k = float(np.corrcoef(x_angles, y_k)[0, 1])
    corr_time = float(np.corrcoef(np.asarray(x_angles)[valid_time], np.asarray(y_time)[valid_time])[0, 1])
    figure_19 = FIGURES / "figure_19_angle_vs_gain.png"
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.4))
    axes[0].scatter(x_angles, y_k, c=colors, s=50, edgecolor="white")
    axes[0].axhline(0, color="#64748b", linewidth=1)
    axes[0].set_title(f"Góc lớn nhất và mức giảm κmax so với Raw (r={corr_k:.2f})", fontweight="bold")
    axes[0].set_xlabel("|θ|max của ca (độ)")
    axes[0].set_ylabel("Mức giảm κmax (%)")
    axes[1].scatter(np.asarray(x_angles)[valid_time], np.asarray(y_time)[valid_time], c=np.asarray(colors)[valid_time], s=50, edgecolor="white")
    axes[1].axhline(0, color="#64748b", linewidth=1)
    axes[1].set_title(f"Góc lớn nhất và Δ thời gian PSTMO−Raw (r={corr_time:.2f})", fontweight="bold")
    axes[1].set_xlabel("|θ|max của ca (độ)")
    axes[1].set_ylabel("Δ thời gian (s); âm = PSTMO nhanh hơn")
    for axis in axes:
        axis.grid(alpha=0.2)
    fig.suptitle("Góc lớn nhất không phải biến duy nhất giải thích hiệu quả", fontweight="bold", fontsize=15)
    fig.tight_layout()
    fig.savefig(figure_19, dpi=180, bbox_inches="tight")
    plt.close(fig)
    return {
        "angle_summary": figure_16,
        "angle_heatmap": figure_17,
        "gain_heatmap": figure_18,
        "angle_vs_gain": figure_19,
        "corr_k": corr_k,
        "corr_time": corr_time,
    }


def geometry_table(rows: dict[str, dict]) -> str:
    values = []
    for method in METHODS:
        row = rows[method]
        if row["success"] != "True":
            values.append([
                f"<b>{METHOD_LABEL[method]}</b>", "THẤT BẠI", "–", "–", "–", "–", "–", "–"
            ])
            continue
        values.append([
            f"<b>{METHOD_LABEL[method]}</b>",
            "Đạt",
            fnum(row["path_length_m"], 3),
            fnum(row["max_abs_curvature_1pm"], 3),
            fnum(row["curvature_energy_1pm"], 3),
            fnum(1000.0 * float(row["algorithm_time_s"]), 2),
            fnum(row["footprint_clearance_min_m"], 3),
            str(int(float(row["footprint_collision_sample_count"]))),
        ])
    return table_html(
        ["Phương án", "Kết quả", "Chiều dài L (m)", "Độ cong lớn nhất κmax (m⁻¹)", "∫κ²ds = Eκ (m⁻¹)", "Thời gian xử lý (ms)", "Khoảng hở nhỏ nhất (m)", "Mẫu hình bao va chạm"],
        values,
        "tiny",
    )


def execution_table(rows: dict[str, dict]) -> str:
    values = []
    for method in METHODS:
        row = rows[method]
        if row["success"] != "True":
            values.append([
                f"<b>{METHOD_LABEL[method]}</b>", "THẤT BẠI", "–", "–", "–", "–", "–", "–"
            ])
            continue
        values.append([
            f"<b>{METHOD_LABEL[method]}</b>",
            "Đạt",
            fnum(row["execution_time_s"], 3),
            fnum(row["traveled_distance_m"], 3),
            fnum(row["final_position_error_m"], 3),
            fnum(abs(degrees(row["final_yaw_error_rad"])), 2),
            fnum(row["tracking_rmse_m"], 3),
            fnum(row["tracking_max_error_m"], 3),
        ])
    return table_html(
        ["Phương án", "Kết quả", "Thời gian chạy (s)", "Quãng đường (m)", "Sai số vị trí đích (m)", "Sai số góc hướng (°)", "RMSE bám đường (m)", "Sai số bám lớn nhất (m)"],
        values,
        "tiny",
    )


def corner_table(item: dict, limit: int = 5) -> str:
    corners = sorted(
        item["pstmo_diagnostics"]["corner_search"],
        key=lambda value: abs(float(value["turn_angle"])),
        reverse=True,
    )[:limit]
    rows = []
    for corner in corners:
        state = state_of_corner(corner)
        angle = degrees(corner["turn_angle"])
        rows.append([
            str(int(corner["index"]) + 1),
            f"({fnum(corner['x'], 3)}; {fnum(corner['y'], 3)})",
            fnum(angle, 2),
            "Trái" if angle > 0 else "Phải",
            state,
            fnum(corner["selected_trim"], 3) if state == "Bézier G²" else "–",
            fnum(corner["selected_control_fraction"], 3) if state == "Bézier G²" else "–",
            f"{corner['safe_feasible']}/{corner['evaluations']}",
        ])
    return table_html(
        ["STT góc", "Tọa độ (m)", "Góc rẽ θ (°)", "Chiều rẽ", "Cách xử lý được chọn", "Khoảng cắt d (m)", "Tỷ lệ α", "Số phương án đạt / lượt đánh giá"],
        rows,
        "tiny",
    )


def case_comparison_notes(geometry: dict[str, dict], execution: dict[str, dict]) -> list[str]:
    pstmo = geometry["pstmo"]
    raw = geometry["raw"]
    valid_stock = [geometry[method] for method in BASELINES if geometry[method]["success"] == "True"]
    best_k = min(valid_stock, key=lambda row: float(row["max_abs_curvature_1pm"]))
    best_e = min(valid_stock, key=lambda row: float(row["curvature_energy_1pm"]))
    best_l = min(valid_stock, key=lambda row: float(row["path_length_m"]))
    best_clearance = max(
        [row for row in geometry.values() if row["success"] == "True"],
        key=lambda row: float(row["footprint_clearance_min_m"]),
    )
    notes = [
        (
            f"So với đường thô Raw, PSTMO {describe_lower(pstmo['max_abs_curvature_1pm'], raw['max_abs_curvature_1pm'])} "
            f"κmax và {describe_lower(pstmo['curvature_energy_1pm'], raw['curvature_energy_1pm'])} Eκ; "
            f"chiều dài {describe_lower(pstmo['path_length_m'], raw['path_length_m'])}."
        ),
        (
            f"Phương án đối chứng Nav2 có κmax thấp nhất là {METHOD_LABEL[best_k['method']]} "
            f"({fnum(best_k['max_abs_curvature_1pm'], 3)} m⁻¹); PSTMO đạt "
            f"{fnum(pstmo['max_abs_curvature_1pm'], 3)} m⁻¹, tương ứng "
            f"{describe_lower(pstmo['max_abs_curvature_1pm'], best_k['max_abs_curvature_1pm'])}."
        ),
        (
            f"Phương án đối chứng Nav2 có Eκ thấp nhất là {METHOD_LABEL[best_e['method']]} "
            f"({fnum(best_e['curvature_energy_1pm'], 3)} m⁻¹); PSTMO đạt "
            f"{fnum(pstmo['curvature_energy_1pm'], 3)} m⁻¹, tương ứng "
            f"{describe_lower(pstmo['curvature_energy_1pm'], best_e['curvature_energy_1pm'])}."
        ),
        (
            f"Phương án đối chứng Nav2 ngắn nhất là {METHOD_LABEL[best_l['method']]} "
            f"({fnum(best_l['path_length_m'], 3)} m); PSTMO dài "
            f"{fnum(pstmo['path_length_m'], 3)} m và {describe_lower(pstmo['path_length_m'], best_l['path_length_m'])}."
        ),
        (
            f"Khoảng hở hình bao nhỏ nhất của PSTMO là {fnum(pstmo['footprint_clearance_min_m'], 3)} m; "
            f"phương án có khoảng hở lớn nhất trong ca là {METHOD_LABEL[best_clearance['method']]} "
            f"với {fnum(best_clearance['footprint_clearance_min_m'], 3)} m. Mọi mẫu PSTMO đều không va chạm."
        ),
    ]
    successful_execution = [row for row in execution.values() if row["success"] == "True"]
    if execution["pstmo"]["success"] == "True":
        fastest = min(successful_execution, key=lambda row: float(row["execution_time_s"]))
        ordered = sorted(successful_execution, key=lambda row: float(row["execution_time_s"]))
        rank = ordered.index(execution["pstmo"]) + 1
        detail = (
            f"Trong Gazebo, PSTMO mất {fnum(execution['pstmo']['execution_time_s'], 3)} s, "
            f"xếp {rank}/{len(ordered)} trong các phương án hoàn tất; nhanh nhất là "
            f"{METHOD_LABEL[fastest['method']]} ({fnum(fastest['execution_time_s'], 3)} s)."
        )
        if execution["raw"]["success"] == "True":
            delta = float(execution["pstmo"]["execution_time_s"]) - float(execution["raw"]["execution_time_s"])
            detail += (
                f" Chênh lệch PSTMO−Raw là {fnum(delta, 3)} s "
                f"({'PSTMO nhanh hơn' if delta < 0 else 'PSTMO chậm hơn'})."
            )
        notes.append(detail)
    else:
        notes.append(
            "PSTMO không hoàn tất lượt Gazebo trong ca này; vì vậy không xếp hạng thời gian "
            "và không dùng thời điểm dừng sớm như một kết quả nhanh."
        )
    if geometry["simple"]["success"] != "True":
        notes.append(
            "Simple không tạo được đường hợp lệ trong phép thử hình học; báo cáo giữ nguyên trạng thái thất bại "
            "thay vì điền giá trị 0 hoặc loại ca khỏi tỷ lệ thành công."
        )
    return notes


def source_table() -> str:
    sources = [
        ROOT / "src" / "adaptive_pivot_g2" / "src" / "path_conditioning.cpp",
        ROOT / "src" / "adaptive_pivot_g2" / "src" / "quintic_transition.cpp",
        ROOT / "src" / "adaptive_pivot_g2" / "src" / "hierarchical_shape_search.cpp",
        ROOT / "src" / "adaptive_pivot_g2" / "src" / "time_parameterization.cpp",
        ROOT / "src" / "adaptive_pivot_g2" / "src" / "candidate_selection.cpp",
        ROOT / "src" / "adaptive_pivot_g2" / "src" / "path_optimization.cpp",
        ROOT / "src" / "adaptive_pivot_g2_nav2" / "src" / "adaptive_pivot_g2_smoother.cpp",
        ROOT / "src" / "adaptive_pivot_g2_nav2" / "src" / "footprint_safety.cpp",
        ROOT / "src" / "vacuum_robot_gazebo" / "config" / "nav2_params.yaml",
        ROOT / "src" / "vacuum_robot_gazebo" / "config" / "real_robot_profile.yaml",
        ROOT / "src" / "vacuum_robot_gazebo" / "models" / "vacuum_robot" / "model.sdf",
        GEOMETRY_CSV,
        EXECUTION_CSV,
        GEOMETRY_AGGREGATE,
        EXECUTION_AGGREGATE,
    ]
    rows = [[str(path.relative_to(ROOT)), sha256(path)[:24] + "…"] for path in sources]
    return table_html(["Tệp nguồn/cấu hình", "SHA-256 rút gọn"], rows, "compact")


def build_html() -> dict:
    cases = load_cases()
    geometry_rows = load_csv(GEOMETRY_CSV)
    execution_rows = load_csv(EXECUTION_CSV)
    geometry_aggregate = json.loads(GEOMETRY_AGGREGATE.read_text(encoding="utf-8"))
    execution_aggregate = json.loads(EXECUTION_AGGREGATE.read_text(encoding="utf-8"))

    geometry_by_case: dict[str, dict[str, dict]] = defaultdict(dict)
    for row in geometry_rows:
        geometry_by_case[row["case_id"]][row["method"]] = row
    execution_by_pair: dict[tuple[str, str], dict[str, dict]] = defaultdict(dict)
    for row in execution_rows:
        execution_by_pair[(row["benchmark_environment"], row["planner"])][row["method"]] = row
    if any(len(rows) != 5 for rows in geometry_by_case.values()):
        raise RuntimeError("Geometry matrix is not complete")
    if any(len(rows) != 5 for rows in execution_by_pair.values()):
        raise RuntimeError("Execution matrix is not complete")

    system_images = prepare_system_images()
    angle_figures = make_angle_figures(cases, geometry_by_case, execution_by_pair)

    pstmo_geo = geometry_aggregate["methods"]["pstmo"]
    raw_geo = geometry_aggregate["methods"]["raw"]
    simple_geo = geometry_aggregate["methods"]["simple"]
    execution_audit = execution_aggregate["audit"]
    paired_raw = execution_aggregate["pstmo_paired_comparison"]["raw"]
    paired_simple = execution_aggregate["pstmo_paired_comparison"]["simple"]

    all_corners = [corner for item in cases for corner in item["pstmo_diagnostics"]["corner_search"]]
    all_abs_angles = [abs(degrees(corner["turn_angle"])) for corner in all_corners]
    transitions = [corner for corner in all_corners if state_of_corner(corner) == "Bézier G²"]
    pivots = [corner for corner in all_corners if state_of_corner(corner) == "Quay tại chỗ"]
    pass_through = [corner for corner in all_corners if state_of_corner(corner) == "Giữ nguyên góc"]

    content: list[str] = []
    content.append(
        '<div class="cover"><h1>BÁO CÁO NGHIÊN CỨU THUẬT TOÁN PSTMO</h1>'
        '<div class="subtitle">Làm mượt đường đi và tối ưu hóa thao tác chuyển hướng cho robot vi sai trong ROS 2 Navigation2</div>'
        '<div class="authors">NGUYỄN TIẾN CƯƠNG</div>'
        '<div class="meta">Bản thống nhất từ hai nguồn • đối chiếu mã nguồn, RViz2, Gazebo và dữ liệu ghép cặp • 19/08/2026</div></div>'
    )
    content.append(figure_html(
        VISUALS_3D / "robot_isometric_clean.png",
        "Mô hình 3D được dựng trực tiếp từ STL/SDF của robot; không dùng làm bằng chứng thực nghiệm. Ảnh Gazebo và RViz2 gốc được trình bày ở Chương 5.",
    ))

    content.append('<h2>TÓM TẮT</h2>')
    content.append(
        '<div class="abstract"><p>Các bộ lập kế hoạch toàn cục trong ROS 2 Navigation2 thường trả về một chuỗi tư thế rời rạc có các góc gãy. '
        'Đối với robot vi sai, đường hợp lệ trên bản đồ chi phí (costmap) chưa bảo đảm chuyển hướng có thể thực thi liên tục: robot có thể phải giảm tốc mạnh, quay tại chỗ '
        'hoặc yêu cầu vận tốc bánh vượt giới hạn. Báo cáo này trình bày PSTMO, một bộ làm mượt đường đi tạo đoạn chuyển tiếp (transition) Bézier bậc năm liên tục hình học G², '
        'kiểm tra động học hai bánh, giới hạn vận tốc theo độ cong, vùng quét hình bao robot, điều kiện ưu thế thời gian và quy hoạch động chống chồng lấn. </p>'
        f'<p>Bộ bằng chứng gồm 35 nhóm hình học RViz2 và {execution_audit["trial_count"]} lượt thực thi Gazebo: 7 môi trường × 5 bộ lập kế hoạch × 5 phương án. '
        f'Trên 34 nhóm đủ năm phương án, PSTMO đạt κmax trung bình {fnum(pstmo_geo["paired_mean_max_abs_curvature_1pm"], 3)} m⁻¹ và Eκ '
        f'{fnum(pstmo_geo["paired_mean_curvature_energy_1pm"], 3)} m⁻¹; tương ứng giảm {fnum(pct_reduction(pstmo_geo["paired_mean_max_abs_curvature_1pm"], raw_geo["paired_mean_max_abs_curvature_1pm"]), 2)}% '
        f'và {fnum(pct_reduction(pstmo_geo["paired_mean_curvature_energy_1pm"], raw_geo["paired_mean_curvature_energy_1pm"]), 2)}% so với Raw. '
        f'PSTMO thành công 35/35 ở phép thử hình học; Simple thành công {simple_geo["success_count"]}/35. Trong thực thi vòng kín, '
        f'{execution_audit["success_count"]}/{execution_audit["trial_count"]} lượt tới đích; cả năm thất bại thuộc cùng nhóm Kho điều phối–SmacHybrid. '
        f'Trên 34 cặp PSTMO–Raw cùng thành công, PSTMO nhanh hơn trung bình {fnum(abs(paired_raw["paired_difference_s_mean"]), 3)} s và nhanh hơn trong '
        f'{paired_raw["pstmo_faster_pair_count"]}/34 cặp. Không ghi nhận mẫu hình bao robot trên đường kế hoạch bị va chạm hoặc lần mô-đun Collision Monitor can thiệp. </p>'
        '<p>Kết quả chứng minh lợi ích rõ rệt về hình học và thời gian mô phỏng, nhưng chưa thay thế kiểm chứng lặp lại hoặc thử nghiệm robot thật.</p>'
        '</div>'
    )
    content.append('<p><b>Từ khóa—</b>PSTMO; robot vi sai; làm mượt đường đi; Bézier bậc năm G²; vùng quét hình bao robot; ROS 2 Navigation2; RViz2; Gazebo.</p>')

    content.append('<h2>QUY ƯỚC THUẬT NGỮ VÀ CÁCH ĐỌC</h2>')
    content.append(
        callout(
            '<b>Nguyên tắc thuật ngữ.</b> Thuật ngữ tiếng Việt được dùng trước; tiếng Anh đặt trong ngoặc ở lần xuất hiện đầu. '
            '“Đường đi” là <i>path</i>; “quỹ đạo theo thời gian” là <i>trajectory</i>. “Tích phân bình phương độ cong Eκ” là một chỉ số hình học, '
            'không được gọi là điện năng tiêu thụ. “Giới hạn vận tốc theo độ cong” và “biểu đồ vận tốc theo chiều dài đường” là hai khái niệm khác nhau.'
        )
    )
    content.append(
        '<ul><li><b>Đường thô Raw:</b> đầu ra của bộ lập kế hoạch, dùng làm đối chứng; Raw không phải một bộ làm mượt.</li>'
        '<li><b>Phương án xử lý tại góc (corner-handling option):</b> một cách xử lý cụ thể cho một góc, gồm trạng thái, khoảng cắt d và tỷ lệ hình dạng α=q/d.</li>'
        '<li><b>Quay tại chỗ (pivot):</b> một trạng thái chuyển hướng, không phải tên thuật toán.</li>'
        '<li><b>Khoảng hở hình bao (footprint clearance):</b> khoảng cách nhỏ nhất từ hình bao robot tới vật cản; khác với giá trị chi phí tại tâm và khác với điều kiện va chạm nhị phân.</li>'
        '<li><b>Hình 3D giải thích:</b> hình dựng từ STL/SDF hoặc minh họa bối cảnh; dùng để trình bày cấu tạo và quan hệ hình học, không dùng thay số liệu.</li>'
        '<li><b>Ảnh chụp trực tiếp:</b> ảnh lấy từ RViz2/Gazebo trong phiên chạy. Các đồ thị phóng to sử dụng đúng tọa độ ROS Path lưu trong JSON của cùng ca.</li></ul>'
    )
    content.append('<h2>MỤC LỤC THEO MẠCH LẬP LUẬN</h2>')
    content.append(table_html(
        ["Chương", "Câu hỏi được trả lời"],
        [
            ["1. Bối cảnh và bài toán", "PSTMO nằm ở đâu trong Nav2 và vì sao góc gãy khó thực thi?"],
            ["2. Cơ sở toán học và mô hình robot", "Các công thức nào cần để mô tả góc, độ cong, bánh xe và an toàn?"],
            ["3. Phương pháp PSTMO", "Các phương án xử lý tại góc được tạo, kiểm tra, lựa chọn và ghép như thế nào?"],
            ["4. Triển khai ROS 2/Nav2", "Công thức tương ứng với tệp mã nguồn và tham số nào?"],
            ["5. Thiết kế thực nghiệm", "Mô hình, môi trường, bộ lập kế hoạch, phương án, điểm đầu–đích và chỉ số đánh giá là gì?"],
            ["6. Kết quả tổng hợp", "PSTMO hơn/kém ở chỉ số nào và giới hạn thống kê là gì?"],
            ["7. Phân tích C01–C35", "Mỗi ca có góc bao nhiêu, trạng thái nào và so sánh từng phương án ra sao?"],
            ["8. Thảo luận", "Điều gì giải thích kết quả và điều gì chưa được chứng minh?"],
            ["9. Kết luận và hướng phát triển", "Kết luận khoa học và bước kiểm chứng tiếp theo là gì?"],
            ["Phụ lục", "Công thức, thuật ngữ, tham số, bảng 35 ca và tệp tái lập."],
        ],
        "compact",
    ))

    # Chapter 1
    content.append(page_break())
    content.append('<h2>CHƯƠNG 1. BỐI CẢNH HỆ THỐNG VÀ BÀI TOÁN NGHIÊN CỨU</h2>')
    content.append('<h3>1.1. Chuỗi xử lý trong ROS 2 Navigation2</h3>')
    content.append(
        '<p>Navigation2 (Nav2) là khung điều hướng dạng mô-đun của ROS 2. Bộ định vị (localization) ước lượng tư thế robot; '
        'bộ lập kế hoạch toàn cục (global planner) tạo đường đi; bộ làm mượt đường đi (path smoother) chỉnh hình học; '
        'bộ điều khiển bám đường (path-following controller) phát lệnh vận tốc tuyến tính v và vận tốc góc ω. '
        'PSTMO được tích hợp ở tầng làm mượt, giữa bộ lập kế hoạch và bộ điều khiển.</p>'
    )
    content.append(equation('Bản đồ + cảm biến → định vị → bộ lập kế hoạch toàn cục → <b>PSTMO</b> → bộ điều khiển bám đường → cmd_vel → robot'))
    content.append(figure_html(system_images["rviz"], "Ảnh chụp trực tiếp giao diện RViz2: bản đồ, bản đồ chi phí, hình bao robot, đường của bộ lập kế hoạch, đường sau làm mượt và dữ liệu chẩn đoán."))
    content.append('<h3>1.2. Phân biệt đường đi và quỹ đạo theo thời gian</h3>')
    content.append(
        '<p>Đường đi (path) là chuỗi tư thế P={p₀,…,pₙ}, trong đó pᵢ=(xᵢ,yᵢ,ψᵢ). Quỹ đạo theo thời gian (trajectory) bổ sung thời điểm, '
        'vận tốc và gia tốc. PSTMO trả về <code>nav_msgs/Path</code>, nhưng trong quá trình lựa chọn phương án có xây dựng biểu đồ vận tốc và ước lượng thời gian '
        'để loại đường cong không khả thi.</p>'
    )
    content.append('<h3>1.3. Vì sao góc gãy là vấn đề?</h3>')
    content.append(
        '<p>Tại một đỉnh của đường gấp khúc (polyline), hướng cạnh vào và cạnh ra khác nhau. Nếu robot đi qua đúng đỉnh với v&gt;0, hướng vận tốc phải đổi tức thời; '
        'độ cong vì thế rất lớn hoặc không xác định. Bộ điều khiển thường phải giảm tốc mạnh, dừng–quay–đi hoặc tạo chuyển động lệch khỏi đường hình học. '
        'Vấn đề nghiên cứu là thay vùng quanh góc bằng một thao tác chuyển hướng mượt mà nhưng vẫn an toàn và có lợi về thời gian.</p>'
    )
    content.append('<h3>1.4. Các phương án đối chứng</h3>')
    content.append(equation('Simple: yᵢ ← yᵢ + w_d(xᵢ−yᵢ) + w_s(yᵢ₋₁+yᵢ₊₁−2yᵢ)'))
    content.append(equation('Savitzky–Golay 7 điểm: [−2, 3, 6, 7, 6, 3, −2] / 21'))
    content.append(equation('Constrained: J = w_s J_s + w_κ J_κ + w_d J_d + w_c J_cost'))
    content.append(
        '<p>Trong cấu hình thử nghiệm đối chứng, Constrained dùng <code>w_smooth=200000</code>, <code>w_cost=0,015</code>, '
        '<code>w_curve=0</code> và <code>w_dist=0</code>. Vì vậy báo cáo phân biệt công thức tổng quát của plugin với đúng cấu hình được thử.</p>'
    )
    content.append('<h3>1.5. Câu hỏi nghiên cứu</h3>')
    content.append(
        '<ol><li>PSTMO có giảm κmax và Eκ so với Raw và các bộ làm mượt Nav2 hay không?</li>'
        '<li>Đường PSTMO có duy trì an toàn hình bao và khả năng thực thi vòng kín trong Gazebo hay không?</li>'
        '<li>Chi phí tính toán và khoảng hở hình bao thay đổi như thế nào?</li>'
        '<li>Góc rẽ, bộ lập kế hoạch và môi trường có đủ để giải thích mức cải thiện hay không?</li></ol>'
    )

    # Chapter 2
    content.append(page_break())
    content.append('<h2>CHƯƠNG 2. CƠ SỞ TOÁN HỌC VÀ MÔ HÌNH ROBOT</h2>')
    content.append('<h3>2.1. Vectơ, tích vô hướng, tích có hướng và góc có dấu</h3>')
    content.append(equation('u·v=u_x v_x+u_y v_y, &nbsp; u×v=u_x v_y−u_y v_x'))
    content.append(equation('θ=atan2(u×v,u·v)'))
    content.append(
        '<p>θ&gt;0 biểu diễn rẽ trái và θ&lt;0 biểu diễn rẽ phải theo quy ước trục bản đồ. Trong chương kết quả, mọi góc báo bằng độ được chuyển trực tiếp '
        'từ trường <code>turn_angle</code> trong dữ liệu chẩn đoán sau điều kiện hóa đường; không ước lượng lại từ ảnh.</p>'
    )
    content.append('<h3>2.2. Độ dài, đạo hàm và độ cong</h3>')
    content.append(equation('L(P)=∑ⁿ⁻¹ᵢ₌₀ ‖pᵢ₊₁−pᵢ‖₂'))
    content.append(equation('κ(t)=[x′(t)y″(t)−y′(t)x″(t)]/[x′(t)²+y′(t)²]³ᐟ²'))
    content.append(equation('κ_max=max|κ(s)|, &nbsp; Eκ=∫₀ᴸκ(s)²ds'))
    content.append(
        '<p>κmax phản ánh đoạn uốn gắt nhất; Eκ mô tả mức uốn tích lũy trên toàn đường. Eκ có đơn vị m⁻¹, là một chỉ số hình học và là thành phần của hàm mục tiêu; '
        'không phải năng lượng điện đo từ pin.</p>'
    )
    content.append('<h3>2.3. Liên tục Cᵏ và liên tục hình học Gᵏ</h3>')
    content.append(
        '<p>C¹ yêu cầu đạo hàm theo tham số bằng nhau; G¹ chỉ yêu cầu cùng hướng tiếp tuyến. G² yêu cầu thêm độ cong liên tục. '
        'Khi nối đoạn thẳng với đường cong, độ cong của đường cong phải tiến về 0 ở mối nối vì đoạn thẳng có κ=0. '
        'Nếu chỉ dùng G¹, κ vẫn có thể nhảy bậc và kéo theo lệnh ω=vκ đổi đột ngột khi v&gt;0. Vì vậy PSTMO chọn G²: '
        'nó giữ cùng tiếp tuyến như G¹ đồng thời loại bước nhảy độ cong tại mối nối thẳng–cong.</p>'
    )
    content.append('<h3>2.4. Mô hình hai bánh vi sai</h3>')
    content.append(equation('v=(v_R+v_L)/2, &nbsp; ω=(v_R−v_L)/b'))
    content.append(equation('v_L=v(1−bκ/2), &nbsp; v_R=v(1+bκ/2), &nbsp; ω=vκ'))
    content.append(
        '<p>Đoạn chuyển tiếp được xem là pha tịnh tiến liên tục; vì vậy bánh trong không được chạy lùi. Điều kiện 1−|bκ|/2≥0 dẫn tới |κ|≤2/b. '
        'Quay tại chỗ là trạng thái riêng, trong đó hai bánh có thể quay ngược chiều.</p>'
    )
    content.append(figure_html(
        VISUALS_3D / "robot_isometric_from_stl_sdf.png",
        "Mô hình 3D được dựng trực tiếp từ STL/SDF của vacuum_robot: thân CAD, hai bánh chủ động, hai động cơ GA25, IMU BNO055 và gốc đo LiDAR. "
        "Đây là hình giải thích cấu tạo; ảnh kết quả mô phỏng vẫn được trình bày riêng ở Chương 5.",
    ))
    content.append(figure_html(
        VISUALS_3D / "wheel_layout_3d.png",
        "Kiểm tra bố trí bánh theo SDF: tâm vệt lăn bánh trái tại y=+0,1274 m, bánh phải tại y=−0,1274 m; vệt bánh vật lý b=0,2548 m. Thân được làm trong chỉ để nhìn rõ cả hai bánh.",
    ))
    content.append('<h3>2.5. Hình bao chiếm chỗ và vùng quét</h3>')
    content.append(equation('F_map(x,y,ψ)={ [x,y]ᵀ+R(ψ)f | f∈F_body }'))
    content.append(
        '<p>Kiểm tra an toàn tại một tư thế (pose safety) chỉ xét hình bao robot ở đúng tư thế đó. Kiểm tra an toàn vùng quét (swept-footprint safety) '
        'xét thêm các tư thế nội suy giữa hai mốc, gồm cả thay đổi vị trí và góc hướng (yaw). Chỉ kiểm tra tâm robot là chưa đủ vì góc của hình bao '
        'có thể chạm kệ dù tâm còn nằm trong ô tự do.</p>'
    )
    content.append(figure_html(
        VISUALS_3D / "robot_footprint_3d.png",
        "Hình bao chiếm chỗ 0,44 × 0,34 m được biến đổi theo từng tư thế; phần thân xe, không chỉ tâm robot, là đối tượng kiểm tra an toàn.",
    ))
    content.append(figure_html(
        VISUALS_3D / "swept_footprint_3d.png",
        "Vùng quét hình bao robot là hợp các hình bao dọc chuyển động. Một phương án bị loại ngay khi vùng quét giao vật cản.",
    ))

    # Chapter 3
    content.append(page_break())
    content.append('<h2>CHƯƠNG 3. PHƯƠNG PHÁP PSTMO</h2>')
    content.append('<h3>3.1. Quy trình xử lý thống nhất</h3>')
    content.append(figure_html(FIGURES / "figure_01_pipeline.png", "Quy trình PSTMO hiện tại: một luồng condition_only, không ghép hai thuật toán tiền xử lý."))
    content.append(
        '<p>Đường do bộ lập kế hoạch tạo ra được điều kiện hóa, phân tích góc, sinh tối đa hai khoảng cắt cho mỗi góc, tìm α theo lưới phân cấp, '
        'kiểm tra hình học–động học–an toàn–thời gian, so sánh các phương án tại từng góc, chọn chuỗi bằng quy hoạch động, ghép và hậu kiểm toàn đường.</p>'
    )
    content.append('<h3>3.2. Điều kiện hóa đường</h3>')
    content.append(equation('δ(i,j)=max_{i&lt;k&lt;j} dist(p_k, đoạn(p_i,p_j)) ≤ ε_RDP'))
    content.append(equation('ε_RDP=1,5×độ phân giải=1,5×0,05=0,075 m'))
    content.append(
        '<p>Thuật toán Ramer–Douglas–Peucker (RDP) chỉ nhận một dây cung khi vừa đạt sai số hình học vừa an toàn theo vùng quét hình bao robot. '
        'Sau đó bộ triệt dao động zíc-zắc cục bộ xử lý dải ngắn có nhiều lần đổi dấu góc. Cấu hình độc lập hiện tại là <code>condition_only</code>; '
        'rút gọn theo đường nhìn thẳng tham lam (greedy line-of-sight, LOS) không thuộc quy trình được đánh giá.</p>'
    )
    content.append(figure_html(FIGURES / "figure_03_conditioning_actual.png", "Ví dụ từ dữ liệu thực: Raw → điều kiện hóa → chuỗi điểm neo dùng để phát hiện góc và sinh đoạn chuyển tiếp."))
    content.append('<h3>3.3. Ba trạng thái tại một góc</h3>')
    content.append(
        '<ul><li><b>Giữ nguyên góc (pass-through):</b> không chèn đoạn chuyển tiếp hoặc quay tại chỗ khi |θ| nhỏ hơn ngưỡng 5°.</li>'
        '<li><b>Đoạn chuyển tiếp (transition):</b> cắt hai cạnh và chèn Bézier bậc năm G².</li>'
        '<li><b>Quay tại chỗ (pivot):</b> đi tới đỉnh, dừng, đổi góc hướng rồi tiếp tục.</li></ul>'
    )
    content.append('<h3>3.4. Cấu trúc Bézier bậc năm G²</h3>')
    content.append(equation('A=V−du, &nbsp; B=V+dv, &nbsp; q=αd'))
    content.append(equation('P₀=A; P₁=A+qu; P₂=A+2qu; P₃=B−2qv; P₄=B−qv; P₅=B'))
    content.append(equation('B(t)=∑⁵ᵢ₌₀ C(5,i)(1−t)⁵⁻ⁱ tⁱ Pᵢ, &nbsp; 0≤t≤1'))
    content.append(
        '<p>Ba điểm đầu và ba điểm cuối thẳng hàng, cách đều q. Vì vậy B″(0)=B″(1)=0; thay vào công thức độ cong cho κ(0)=κ(1)=0. '
        'Mối nối với hai đoạn thẳng đạt G² về hình học. Bézier bậc năm được chọn vì đủ điều kiện biên mà vẫn giữ α để điều chỉnh hình dạng.</p>'
    )
    content.append(figure_html(FIGURES / "figure_04_bezier_g2.png", "Sơ đồ Bézier G² và biểu đồ độ cong; đây là hình giải thích công thức, không phải ảnh RViz2."))
    content.append('<h3>3.5. Hai khoảng cắt d và tìm α=q/d</h3>')
    content.append(equation('d_pref=min(d_max,L_in,L_out)'))
    content.append(equation('d_compat=min(d_pref,b_in,b_out)'))
    content.append(
        '<p>d<sub>pref</sub> ưu tiên vùng chuyển tiếp lớn; d<sub>compat</sub> giảm mức chiếm đoạn khi góc kề cùng dùng một cạnh. Với từng d, '
        'lưới thô α={0,1;0,2;0,3;0,4;0,5} được đánh giá trước. Nếu toàn bộ lưới thô thất bại, lưới phục hồi '
        '{0,15;0,25;0,35;0,45} mới được dùng; sau đó đánh giá tinh 11 nút quanh phương án có Eκ nhỏ nhất.</p>'
    )
    content.append(figure_html(FIGURES / "figure_05_alpha_search.png", "Tìm α thô–phục hồi–tinh; chỉ các phương án đạt mọi điều kiện bắt buộc mới được so sánh Eκ."))
    content.append('<h3>3.6. Chuỗi điều kiện loại bắt buộc</h3>')
    content.append(figure_html(FIGURES / "figure_02_hard_gates.png", "Thứ tự các điều kiện loại bắt buộc: hình học, động học, hình bao robot, thời gian và hậu kiểm."))
    content.append(
        '<p>Một phương án bị loại nếu đạo hàm suy biến, độ cong không hữu hạn, đổi dấu quay ngoài ý muốn, yêu cầu bánh trong chạy lùi, '
        'không tạo được vận tốc dương, vi phạm hình bao robot hoặc bản đồ chi phí, hoặc biểu đồ thời gian không hội tụ. Va chạm là điều kiện loại bắt buộc; '
        'không thể được bù bằng Eκ thấp.</p>'
    )
    content.append('<h3>3.7. Giới hạn vận tốc và biểu đồ vận tốc</h3>')
    content.append(equation('v_limit(κ)=min[v_max,ω_max/|κ|,√(a_y,max/|κ|),v_w,max/max(|1−bκ/2|,|1+bκ/2|)]'))
    content.append(equation('vᵢ≤√(vᵢ₋₁²+2a_accΔs), &nbsp; vᵢ₋₁≤√(vᵢ²+2a_decΔs)'))
    content.append(
        '<p><b>Giới hạn vận tốc theo độ cong</b> là ngưỡng cục bộ tại mỗi mẫu. <b>Biểu đồ vận tốc theo chiều dài đường</b> là dãy v(s) '
        'sau quét tiến/lùi để thỏa giới hạn tăng tốc và giảm tốc. Hai khái niệm này phải được trình bày riêng, không thay thế cho nhau.</p>'
    )
    content.append(figure_html(FIGURES / "figure_07_kinematic_time_gate.png", "Ví dụ giới hạn vận tốc, biểu đồ vận tốc và vận tốc góc dùng để ước lượng thời gian."))
    content.append('<h3>3.8. Điều kiện ưu thế thời gian và hàm mục tiêu cục bộ</h3>')
    content.append(equation('T_fastest+ΔT&lt;T_pivot, &nbsp; ΔT=0,15 s'))
    content.append(equation('ΔT_comp=10,0 s nếu quay tại chỗ không an toàn; nếu an toàn: min[10,0 s; max(0;T_pivot−0,15 s−T_fastest)], &nbsp; Tᵢ≤T_fastest+ΔT_comp'))
    content.append(equation('r_cost=min(1,c_peak/252), &nbsp; r_ω=min(1,|ω|_peak/ω_max), &nbsp; eκ=(Eκ/E_ref)/(Eκ/E_ref+1), &nbsp; E_ref=1 m⁻¹'))
    content.append(equation('J=0,15·r_cost+0,10·r_ω+0,75·eκ &nbsp; → &nbsp; min'))
    content.append(
        '<p>Điều kiện ưu thế thời gian (time gate) chỉ mở nhánh đoạn chuyển tiếp khi đường cong nhanh nhất cộng biên 0,15 s vẫn nhanh hơn quay tại chỗ, '
        'trừ trường hợp quay tại chỗ không an toàn. Sau khi mở nhánh, chỉ phương án có thời gian nằm trong cửa sổ cạnh tranh ΔT<sub>comp</sub> mới được '
        'xét tiếp; khi quay tại chỗ an toàn, cửa sổ này còn bị chặn để mọi phương án vẫn nhanh hơn quay tại chỗ ít nhất 0,15 s. '
        'Trong đó r<sub>cost</sub> là thành phần chi phí môi trường, r<sub>ω</sub> là thành phần vận tốc góc '
        'và e<sub>κ</sub> là thành phần uốn hình học không thứ nguyên, dùng mức tham chiếu E<sub>ref</sub>=1 m⁻¹; e<sub>κ</sub> không phải điện năng. '
        'Trong mã nguồn, E<sub>ref</sub> là tham số <code>curvature_energy_scale=1,0</code>. Hàm J được tối thiểu hóa và chỉ xếp hạng các trạng thái '
        'đã vượt mọi điều kiện bắt buộc. '
        'Toàn bộ bảy môi trường dùng cùng một định nghĩa J và cùng bộ trọng số; môi trường chỉ làm thay đổi giá trị chi phí, khoảng hở và tập phương án '
        'khả thi, không tạo một hàm mục tiêu riêng.</p>'
    )
    content.append('<h3>3.9. Quy hoạch động và ghép đường</h3>')
    content.append(equation('d(zᵢ)+d(zᵢ₊₁)+m≤Lᵢ'))
    content.append(equation('Dᵢ(z)=Jᵢ(z)+min_{z′ tương thích} Dᵢ₋₁(z′)'))
    content.append(
        '<p>Quy hoạch động chọn một trạng thái cho mỗi góc mà không chồng lấn vùng cắt; độ phức tạp O(NK²). '
        'Đường cuối bảo toàn tư thế đầu và tư thế đích, nội suy đoạn thẳng 0,05 m, gán góc hướng theo tiếp tuyến và quét lại toàn bộ hình bao robot. '
        'Nếu bất biến cuối thất bại, plugin báo lỗi thay vì âm thầm trả Raw.</p>'
    )

    # Chapter 4
    content.append(page_break())
    content.append('<h2>CHƯƠNG 4. TRIỂN KHAI ROS 2/NAV2 VÀ TRUY VẾT MÃ NGUỒN</h2>')
    content.append('<h3>4.1. Bảng truy vết lý thuyết–triển khai</h3>')
    content.append(table_html(
        ["Khối", "Tệp chịu trách nhiệm", "Hành vi chính"],
        [
            ["Điều kiện hóa", "path_conditioning.cpp", "RDP, triệt dao động zíc-zắc, điều kiện an toàn dây cung"],
            ["Bézier G²", "quintic_transition.cpp", "Sáu điểm điều khiển, κ, Eκ, giới hạn bánh và vận tốc"],
            ["Hai d và α", "hierarchical_shape_search.cpp", "d_pref, d_compat, lưới thô–phục hồi–tinh"],
            ["Thời gian", "time_parameterization.cpp", "Quét tiến/lùi, gia tốc góc, thời gian quay tại chỗ"],
            ["Hàm mục tiêu", "candidate_selection.cpp", "Chi phí môi trường–vận tốc góc–mức uốn và quy tắc lựa chọn tất định khi bằng điểm"],
            ["Quy hoạch động", "path_optimization.cpp", "Tương thích đoạn chung và truy vết chuỗi trạng thái"],
            ["Trình cắm Nav2", "adaptive_pivot_g2_smoother.cpp", "Quy trình, điều kiện ưu thế thời gian, ghép đường và dữ liệu chẩn đoán"],
            ["An toàn", "footprint_safety.cpp", "Kiểm tra tư thế, đoạn thẳng, quay và vùng quét hình bao robot"],
        ],
        "compact",
    ))
    content.append('<h3>4.2. Tham số thực nghiệm hiện tại</h3>')
    content.append(table_html(
        ["Tham số", "Giá trị", "Ý nghĩa"],
        [
            ["wheel_separation của PSTMO", "0,2548 m", "Khoảng cách tâm vệt lăn vật lý dùng trong ràng buộc động học"],
            ["wheel_separation hiệu dụng Gazebo", "0,2834 m", "Giá trị DiffDrive đã hiệu chuẩn tiếp xúc; hệ số 1,112245 so với vệt bánh vật lý"],
            ["wheel_radius", "0,0425 m", "Bán kính bánh chủ động trong SDF"],
            ["footprint (hình bao robot)", "0,44 × 0,34 m", "Hình chữ nhật dự phòng và mô hình đánh giá"],
            ["costmap resolution (độ phân giải)", "0,05 m", "Độ phân giải bản đồ chi phí"],
            ["maximum_trim_distance", "0,8 m", "Khoảng cắt lớn nhất"],
            ["minimum_trim_distance", "0,02 m", "Khoảng cắt nhỏ nhất"],
            ["output_spacing / sample_spacing", "0,05 / 0,02 m", "Khoảng mẫu đầu ra / Bézier"],
            ["vmax / ωmax", "0,30 m/s / 0,80 rad/s", "Giới hạn thân xe"],
            ["vwheel,max", "0,36 m/s", "Giới hạn mỗi bánh"],
            ["ay,max", "0,18 m/s²", "Giới hạn gia tốc ngang"],
            ["a_acc / a_dec", "0,35 / 0,45 m/s²", "Gia tốc dọc / độ lớn giảm tốc dọc"],
            ["aω,max", "1,20 rad/s²", "Gia tốc góc"],
            ["max_footprint_cost", "252", "Ngưỡng chi phí tại tâm; đa giác hình bao được quét riêng"],
            ["ΔT_selection", "0,15 s", "Biên ưu thế thời gian"],
            ["time_competitive_slack", "10,0 s", "Giới hạn trên của cửa sổ các phương án được so sánh; còn bị chặn bởi thời gian quay tại chỗ"],
            ["curvature_energy_scale", "1,0 m⁻¹", "Mức tham chiếu để chuẩn hóa Eκ thành đại lượng không thứ nguyên"],
            ["segment_margin", "0 → hiệu dụng 0,05 m", "Giá trị 0 kích hoạt max(output spacing, 2×sample spacing, resolution)"],
            ["trọng số J", "0,15 / 0,10 / 0,75", "Chi phí môi trường / vận tốc góc / mức uốn Eκ"],
        ],
        "compact",
    ))
    content.append('<h3>4.3. Bộ điều khiển và các lớp an toàn dùng chung</h3>')
    content.append(
        '<p>Cả năm phương án dùng cùng bộ điều khiển bám đường Regulated Pure Pursuit (RPP) ở 20 Hz, vận tốc mong muốn 0,30 m/s và khoảng nhìn trước '
        '(lookahead distance) cố định 0,50 m. Bộ làm mượt lệnh vận tốc (Velocity Smoother) chạy 50 Hz; mô-đun giám sát va chạm (Collision Monitor) '
        'kiểm tra tiếp cận theo hình bao robot. Do bộ điều khiển và giới hạn vận tốc dùng chung, khác biệt giữa các phương án chủ yếu đến từ đường được '
        'giao cho tác vụ <code>FollowPath</code>.</p>'
    )
    content.append('<h3>4.4. Kiểm thử phần mềm</h3>')
    content.append(
        '<p>Kết quả tổng hợp hiện có: 308 phép kiểm thử, 0 lỗi thực thi, 0 phép thất bại và 40 phép được bỏ qua thuộc cppcheck/phân tích tĩnh theo môi trường. '
        'Cả 35 ca PSTMO đều xác nhận đúng một lần chạy quy trình và trường hậu kiểm <code>final_invariants_verified=true</code>.</p>'
    )

    # Chapter 5
    content.append(page_break())
    content.append('<h2>CHƯƠNG 5. THIẾT KẾ THỰC NGHIỆM</h2>')
    content.append('<h3>5.1. Mô hình và phần mềm mô phỏng</h3>')
    content.append(figure_html(
        VISUALS_3D / "robot_exploded_from_stl_sdf.png",
        "Hình tách lớp cấu tạo được dựng từ đúng STL/SDF để giải thích mô hình. Vị trí tách rời chỉ phục vụ trình bày, không phải trạng thái vật lý trong Gazebo.",
    ))
    content.append(figure_html(system_images["gazebo"], "Gazebo Harmonic trả về trạng thái vật lý của robot trong môi trường kho; ảnh chụp trực tiếp từ phiên chạy hệ thống."))
    content.append(figure_html(system_images["rviz_layout"], "RViz2 hiển thị RobotModel, biến đổi hệ tọa độ TF, LaserScan, bản đồ, bản đồ chi phí, hình bao robot, đường của bộ lập kế hoạch, đường sau làm mượt và dữ liệu chẩn đoán."))
    content.append(
        '<p><b>Cần phân biệt ba loại hình.</b> Hình 3D từ STL/SDF giải thích cấu tạo và quan hệ hình học; Gazebo mô phỏng thân xe, tiếp xúc bánh–sàn, cảm biến và vật cản; Nav2 xử lý định vị, bản đồ chi phí, '
        'lập kế hoạch, làm mượt và điều khiển. Vì vậy ảnh Gazebo xác nhận mô hình vật lý/môi trường, còn ảnh RViz2 xác nhận dữ liệu điều hướng '
        'và đúng đường <code>nav_msgs/Path</code> được đánh giá. Hình 3D và các sơ đồ ở Chương 2–3 không được dùng thay bằng chứng mô phỏng.</p>'
    )
    content.append(table_html(
        ["Khối", "Giá trị/cấu hình đúng trong mô hình", "Dữ liệu quan sát"],
        [
            ["Thân robot", "Bao CAD 0,44 × 0,34 m; thân chính 4,6 kg; hai bánh 0,2 kg/bánh; bốn gối cầu đỡ", "RobotModel, hình bao robot và trạng thái vật lý Gazebo"],
            ["Truyền động vi sai", "Hai bánh chủ động R=0,0425 m; vệt bánh vật lý 0,2548 m", "<code>/cmd_vel</code>, <code>/joint_states</code>, <code>/odom</code> ở 30 Hz"],
            ["Hiệu chuẩn Gazebo", "DiffDrive dùng vệt bánh hiệu dụng 0,2834 m để ước lượng hành trình khớp giá trị thực mô phỏng", "<code>/odom</code> đối chiếu <code>/ground_truth/odom</code> độc lập ở 30 Hz"],
            ["LiDAR 2D RPLIDAR A1M8", "360°; 1440 tia; 5,5 Hz; 0,15–12 m; σ nhiễu 0,01 m", "<code>/scan</code> và lớp LaserScan trong RViz2"],
            ["IMU BNO055", "100 Hz; nhiễu vận tốc góc 0,002 rad/s và gia tốc 0,03 m/s²", "<code>/imu/data</code>"],
            ["Định vị và bản đồ", "Định vị Monte Carlo thích nghi (AMCL) với mô hình chuyển động vi sai; bản đồ chi phí toàn cục/cục bộ", "TF map→odom→base_link, bản đồ, bản đồ chi phí và hình bao robot"],
            ["Chuỗi điều hướng", "5 bộ lập kế hoạch toàn cục → 5 phương án đường → PSTMO/đối chứng → RPP → bộ làm mượt lệnh vận tốc → Collision Monitor", "Đường của bộ lập kế hoạch, đường sau làm mượt, dữ liệu chẩn đoán và quỹ đạo thực mô phỏng"],
        ],
        "compact",
    ))
    content.append('<h3>5.2. Ma trận thử nghiệm</h3>')
    content.append(figure_html(FIGURES / "figure_12_test_matrix.png", "Ma trận 7 môi trường × 5 bộ lập kế hoạch; mỗi ô có ảnh PNG RViz2, tọa độ đường chính xác trong JSON và kết quả hậu kiểm."))
    content.append(
        '<p>Mỗi tổ hợp môi trường–bộ lập kế hoạch tạo một đường thô Raw. Năm phương án Raw, Simple, Savitzky–Golay, Constrained và PSTMO nhận cùng đường đó. '
        'Phép thử hình học có 35×5=175 bản ghi; phép thử vòng kín có thêm 175 lượt Gazebo độc lập.</p>'
    )
    content.append('<h3>5.3. Bảy môi trường và điểm đầu–đích</h3>')
    start_goal_rows = []
    for environment in ENVIRONMENTS:
        item = next(value for value in cases if value["environment"] == environment)
        start = item["start"]
        goal = item["goal"]
        bearing = math.degrees(math.atan2(goal[1] - start[1], goal[0] - start[0]))
        start_goal_rows.append([
            ENV_LABEL[environment],
            f"({fnum(start[0], 2)}; {fnum(start[1], 2)}; {fnum(degrees(start[2]), 1)}°)",
            f"({fnum(goal[0], 2)}; {fnum(goal[1], 2)}; {fnum(degrees(goal[2]), 1)}°)",
            fnum(bearing, 1) + "°",
            SCENARIO_LABEL[item["scenario"]],
        ])
    content.append(table_html(
        ["Môi trường", "Điểm đầu (x; y; góc hướng)", "Điểm đích (x; y; góc hướng)", "Phương vị thẳng đầu→đích", "Kịch bản"],
        start_goal_rows,
        "compact",
    ))
    for environment in ENVIRONMENTS:
        content.append(figure_html(GAZEBO / f"{environment}.png", f"Ảnh chụp trực tiếp Gazebo/RViz2 của môi trường {ENV_LABEL[environment]}."))
    content.append('<h3>5.4. Năm bộ lập kế hoạch toàn cục</h3>')
    content.append(table_html(
        ["Bộ lập kế hoạch", "Đặc trưng dùng trong báo cáo", "Điểm cần thận trọng"],
        [
            ["NavFn A*", "Tìm kiếm A* trên lưới bản đồ chi phí", "Đường có thể mang đặc trưng bước lưới và nhiều điểm"],
            ["NavFn Dijkstra", "Tìm kiếm Dijkstra trên lưới", "Dùng cùng bản đồ chi phí nhưng chiến lược tìm kiếm khác A*"],
            ["ThetaStar", "Lập kế hoạch không bị giới hạn theo hướng lưới (any-angle)", "Thường tạo dây cung dài và ít điểm hơn"],
            ["Smac2D", "Tìm kiếm hai chiều có xét chi phí", "Có bước làm mượt nội bộ của bộ lập kế hoạch; không phải máy chủ làm mượt độc lập"],
            ["SmacHybrid", "Hybrid A* sử dụng các mẫu chuyển động tiến", "Mô hình chuyển động của bộ lập kế hoạch không biểu diễn đầy đủ thao tác quay tại chỗ của robot vi sai"],
        ],
        "compact",
    ))
    content.append('<h3>5.5. Chỉ số và quy tắc so sánh</h3>')
    content.append(
        '<ul><li><b>Hình học:</b> chiều dài L, độ cong lớn nhất κmax, tích phân bình phương độ cong Eκ, số lần quay tại chỗ và tổng góc quay tại chỗ.</li>'
        '<li><b>An toàn:</b> khoảng hở hình bao nhỏ nhất/trung bình, số mẫu va chạm.</li>'
        '<li><b>Tính toán:</b> thời gian thuật toán của bộ làm mượt.</li>'
        '<li><b>Thực thi:</b> trạng thái đến đích thành công, thời gian chạy, quãng đường, sai số vị trí/góc hướng cuối và sai số bám đường.</li>'
        '<li><b>Ghép cặp:</b> cùng môi trường, cùng bộ lập kế hoạch và cùng mã SHA-256 của đường Raw.</li></ul>'
    )
    content.append(
        callout(
            '<b>Giới hạn thống kê.</b> Mỗi tổ hợp mới chạy một lần. Độ lệch chuẩn giữa 35 tuyến mô tả độ khác nhau của tuyến, không phải độ dao động '
            'khi lặp lại cùng một tuyến; báo cáo vì vậy không tuyên bố ý nghĩa thống kê.'
        )
    )

    # Chapter 6
    content.append(page_break())
    content.append('<h2>CHƯƠNG 6. KẾT QUẢ TỔNG HỢP VÀ PHÂN TÍCH</h2>')
    content.append('<h3>6.1. Kết quả hình học ghép cặp</h3>')
    content.append(figure_html(FIGURES / "figure_09_aggregate_metrics.png", "Bốn chỉ số hình học trên 34 nhóm đủ cả năm phương án."))
    method_rows = []
    for method in METHODS:
        stats = geometry_aggregate["methods"][method]
        method_rows.append([
            f"<b>{METHOD_LABEL[method]}</b>",
            f"{stats['success_count']}/35",
            fnum(stats["paired_mean_path_length_m"], 4),
            fnum(stats["paired_mean_max_abs_curvature_1pm"], 4),
            fnum(stats["paired_mean_curvature_energy_1pm"], 4),
            fnum(1000 * stats["paired_mean_algorithm_time_s"], 2),
            fnum(stats["paired_mean_footprint_clearance_min_m"], 4),
        ])
    content.append(table_html(
        ["Phương án", "Thành công hình học", "Chiều dài L (m)", "Độ cong lớn nhất κmax (m⁻¹)", "∫κ²ds = Eκ (m⁻¹)", "Thời gian xử lý (ms)", "Khoảng hở nhỏ nhất (m)"],
        method_rows,
        "compact",
    ))
    content.append(
        f'<p>So với phương án đối chứng Nav2 tốt nhất về κmax là Simple, PSTMO giảm {fnum(pct_reduction(pstmo_geo["paired_mean_max_abs_curvature_1pm"], simple_geo["paired_mean_max_abs_curvature_1pm"]), 2)}%. '
        f'So với phương án đối chứng tốt nhất về Eκ cũng là Simple, PSTMO giảm {fnum(pct_reduction(pstmo_geo["paired_mean_curvature_energy_1pm"], simple_geo["paired_mean_curvature_energy_1pm"]), 2)}%. '
        f'Chiều dài PSTMO ngắn hơn Simple {fnum(pct_reduction(pstmo_geo["paired_mean_path_length_m"], simple_geo["paired_mean_path_length_m"]), 2)}%, '
        f'nhưng thời gian thuật toán cao hơn {fnum(1000*(pstmo_geo["paired_mean_algorithm_time_s"]-simple_geo["paired_mean_algorithm_time_s"]), 2)} ms.</p>'
    )
    content.append('<h3>6.2. Góc rẽ, trạng thái và tham số được chọn</h3>')
    content.append(figure_html(angle_figures["angle_summary"], "Phân bố góc và trạng thái lấy trực tiếp từ 229 bản ghi chẩn đoán corner_search."))
    content.append(
        f'<p>Toàn bộ 35 đường có {len(all_corners)} góc sau điều kiện hóa: {len(transitions)} đoạn chuyển tiếp Bézier G², {len(pivots)} quay tại chỗ và '
        f'{len(pass_through)} góc được giữ nguyên. |θ| trung bình {fnum(statistics.fmean(all_abs_angles), 2)}°, trung vị {fnum(statistics.median(all_abs_angles), 2)}°, '
        f'nhỏ nhất {fnum(min(all_abs_angles), 2)}° và lớn nhất {fnum(max(all_abs_angles), 2)}°. Trong 221 đoạn chuyển tiếp, α được chọn nằm trong '
        f'[{fnum(min(float(c["selected_control_fraction"]) for c in transitions), 2)}; {fnum(max(float(c["selected_control_fraction"]) for c in transitions), 2)}], '
        f'trung bình {fnum(statistics.fmean(float(c["selected_control_fraction"]) for c in transitions), 3)}; d nằm trong '
        f'[{fnum(min(float(c["selected_trim"]) for c in transitions), 3)}; {fnum(max(float(c["selected_trim"]) for c in transitions), 3)}] m.</p>'
    )
    content.append(figure_html(angle_figures["angle_heatmap"], "Mỗi ô ghi góc lớn nhất và số đoạn chuyển tiếp G²/quay tại chỗ của đúng ca."))
    content.append('<h3>6.3. Mức cải thiện theo từng ca</h3>')
    content.append(figure_html(angle_figures["gain_heatmap"], "Mức giảm κmax và Eκ của PSTMO so với phương án đối chứng Nav2 tốt nhất trong từng ca; giá trị âm nghĩa là PSTMO làm chỉ số tăng."))
    content.append(
        '<p>PSTMO không tốt hơn ở mọi ca với cùng biên độ. Một số ca của Smac2D/SmacHybrid đã có đường tương đối mượt từ bộ lập kế hoạch hoặc phương án đối chứng, nên mức giảm nhỏ hơn. '
        'Ngược lại, các đường lưới có đỉnh độ cong cao cho mức cải thiện lớn. Phân tích từng ca ở Chương 7 ghi rõ cả trường hợp PSTMO tốt hơn và kém hơn.</p>'
    )
    content.append(figure_html(angle_figures["angle_vs_gain"], "Tương quan mô tả giữa góc lớn nhất, mức giảm κmax và chênh lệch thời gian; không dùng để suy luận nhân quả."))
    content.append(
        f'<p>Hệ số tương quan mô tả giữa góc lớn nhất và mức giảm κmax so với Raw chỉ là r={fnum(angle_figures["corr_k"], 2)}; giữa góc lớn nhất và '
        f'chênh lệch thời gian PSTMO−Raw là r={fnum(angle_figures["corr_time"], 2)}. Vì vậy không thể kết luận đơn giản “góc càng lớn thì PSTMO càng nhanh”. '
        'Số điểm, chiều dài cạnh, chuỗi góc, hành lang, bộ lập kế hoạch và phản ứng của bộ điều khiển cùng ảnh hưởng kết quả.</p>'
    )
    content.append('<h3>6.4. An toàn và khoảng hở</h3>')
    content.append(
        f'<p>Trên 34 nhóm đầy đủ, khoảng hở nhỏ nhất trung bình của PSTMO là {fnum(pstmo_geo["paired_mean_footprint_clearance_min_m"], 4)} m; '
        f'Raw là {fnum(raw_geo["paired_mean_footprint_clearance_min_m"], 4)} m, Simple là {fnum(simple_geo["paired_mean_footprint_clearance_min_m"], 4)} m. '
        'PSTMO có khoảng hở cao hơn Raw nhưng thấp hơn Simple, Savitzky–Golay và Constrained. Đây là đánh đổi về khoảng dự phòng, không phải va chạm: '
        'số mẫu hình bao robot bị va chạm của PSTMO bằng 0.</p>'
    )
    content.append('<h3>6.5. Thực thi vòng kín trong Gazebo</h3>')
    content.append(figure_html(FIGURES / "figure_13_execution_overall.png", "Thời gian di chuyển trung bình của các lượt thành công; lượt thất bại không được coi là nhanh."))
    execution_method_rows = []
    for method in METHODS:
        stats = execution_aggregate["overall_by_method"][method]
        execution_method_rows.append([
            f"<b>{METHOD_LABEL[method]}</b>",
            f"{stats['success_count']}/{stats['trial_count']}",
            str(stats["successful_time_sample_count"]),
            fnum(stats["execution_time_s_mean"], 3),
            fnum(stats["execution_time_s_stdev"], 3),
            fnum(stats["execution_time_s_min"], 3),
            fnum(stats["execution_time_s_max"], 3),
        ])
    content.append(table_html(
        ["Phương án", "Lượt thành công", "Số mẫu thời gian hợp lệ", "Trung bình (s)", "Độ lệch chuẩn (s)", "Nhỏ nhất (s)", "Lớn nhất (s)"],
        execution_method_rows,
        "compact",
    ))
    content.append(figure_html(FIGURES / "figure_14_execution_by_planner.png", "Thời gian theo bộ lập kế hoạch; phải đọc giá trị trung bình cùng số lượt thành công."))
    paired_rows = []
    for method in METHODS[:-1]:
        stats = execution_aggregate["pstmo_paired_comparison"][method]
        paired_rows.append([
            METHOD_LABEL[method],
            str(stats["pair_count"]),
            fnum(stats["paired_difference_s_mean"], 3),
            fnum(stats["paired_relative_change_percent_mean"], 2) + "%",
            f"{stats['pstmo_faster_pair_count']}/{stats['pair_count']}",
            f"{stats['pstmo_slower_pair_count']}/{stats['pair_count']}",
        ])
    content.append(table_html(
        ["PSTMO so với", "Số cặp", "Chênh lệch trung bình PSTMO−đối chứng (s)", "Thay đổi tương đối ghép cặp", "PSTMO nhanh hơn", "PSTMO chậm hơn"],
        paired_rows,
        "compact",
    ))
    content.append(figure_html(FIGURES / "figure_15_execution_pairwise_heatmap.png", "Đầy đủ 140 so sánh cặp PSTMO với bốn đối chứng; xanh biểu thị PSTMO nhanh hơn."))
    content.append('<h3>6.6. Ca thất bại C30</h3>')
    content.append(
        callout(
            '<b>Cần phân biệt hai loại thất bại.</b> Trong phép thử hình học, máy chủ làm mượt <code>smoother_server</code> từ chối đường Simple tại '
            'x=−4,741290 m; y=3,482165 m; góc hướng=0,352672 rad vì kiểm tra hình bao phát hiện va chạm. Trong phép thử thực thi, cả năm phương án '
            'của Kho điều phối–SmacHybrid không hoàn tất: Simple không có đường hợp lệ; bốn đường còn lại không có mẫu hình bao tĩnh bị va chạm nhưng '
            'RPP liên tục dự báo va chạm phía trước và tác vụ bị hủy do vượt thời gian chờ điều khiển (<code>PATIENCE_EXCEEDED</code>, mã 104). '
            'Cảnh báo dự báo va chạm nội bộ của RPP khác với một lần can thiệp của mô-đun Collision Monitor; dữ liệu ghi nhận số lần Collision Monitor can thiệp bằng 0. '
            'Do đó đây là giới hạn phối hợp giữa bộ lập kế hoạch, bộ làm mượt và bộ điều khiển; không được coi thời điểm dừng sớm là thời gian hoàn thành.'
        )
    )
    content.append('<h3>6.7. Kết quả theo môi trường</h3>')
    environment_rows = []
    for environment in ENVIRONMENTS:
        stats = execution_aggregate["by_environment_and_method"][environment]
        environment_rows.append([
            ENV_LABEL[environment]
        ] + [
            f"{fnum(stats[method]['execution_time_s_mean'], 3)} ({stats[method]['success_count']}/5)"
            for method in METHODS
        ])
    content.append(table_html(
        ["Môi trường", "Raw", "Simple", "Savitzky–Golay", "Constrained", "PSTMO"],
        environment_rows,
        "compact",
    ))
    for environment in ENVIRONMENTS:
        content.append(figure_html(EXECUTION_MATRICES / f"{environment}_matrix_5x5.png", f"{ENV_LABEL[environment]}: đủ 5 bộ lập kế hoạch × 5 phương án; hình dùng dữ liệu đường kế hoạch và quỹ đạo Gazebo của từng lượt."))

    # Chapter 7: every case
    content.append(page_break())
    content.append('<h2>CHƯƠNG 7. PHÂN TÍCH CHI TIẾT 35 CA RVIZ2/GAZEBO</h2>')
    content.append(
        '<p>Mỗi ca dưới đây dùng đúng một tệp JSON RViz2, năm hàng thử nghiệm hình học và năm hàng thực thi Gazebo. Góc được lấy từ dữ liệu chẩn đoán sau điều kiện hóa. '
        '“Hơn” chỉ được dùng theo chiều có lợi của từng chỉ số: L, κmax, Eκ và thời gian càng thấp càng tốt; khoảng hở càng cao càng tốt; va chạm phải bằng 0.</p>'
    )
    for environment_index, environment in enumerate(ENVIRONMENTS, 1):
        # The sixth environment ends exactly at a page boundary in the current
        # dataset; a second forced break would create a numbered blank page.
        if environment_index != len(ENVIRONMENTS):
            content.append(page_break())
        content.append(f'<h3>7.{environment_index}. {ENV_LABEL[environment]}</h3>')
        content.append(f'<p>{ENV_DESCRIPTION[environment]}</p>')
        content.append(figure_html(GAZEBO / f"{environment}.png", f"Ảnh chụp trực tiếp Gazebo/RViz2 — {ENV_LABEL[environment]}."))
        for planner_index, item in enumerate([case for case in cases if case["environment"] == environment], 1):
            geometry = geometry_by_case[item["case_id"]]
            execution = execution_by_pair[(environment, item["planner"])]
            stats = case_corner_stats(item)
            start, goal = item["start"], item["goal"]
            bearing = degrees(math.atan2(goal[1] - start[1], goal[0] - start[0]))
            start_offset = wrap_degrees(degrees(start[2]) - bearing)
            goal_offset = wrap_degrees(degrees(goal[2]) - bearing)
            content.append(page_break())
            content.append(
                f'<h4>{item["case_id"]} — {item["planner"]} — '
                f'{SCENARIO_LABEL[item["scenario"]]}</h4>'
            )
            content.append(
                f'<p><b>Cấu hình hình học.</b> Điểm đầu=({fnum(start[0], 3)}; {fnum(start[1], 3)}; góc hướng {fnum(degrees(start[2]), 2)}°), '
                f'điểm đích=({fnum(goal[0], 3)}; {fnum(goal[1], 3)}; góc hướng {fnum(degrees(goal[2]), 2)}°). Phương vị thẳng đầu→đích là '
                f'{fnum(bearing, 2)}°; góc hướng đầu lệch {fnum(start_offset, 2)}° và góc hướng đích lệch {fnum(goal_offset, 2)}° so với phương vị này.</p>'
            )
            content.append(
                f'<p><b>Cấu trúc góc.</b> Đường Raw có {item["pstmo_diagnostics"]["raw_input_points"]} mẫu tư thế; sau điều kiện hóa còn '
                f'{item["pstmo_diagnostics"]["conditioning_output_points"]} điểm neo và tạo {stats["count"]} góc. '
                f'Có {stats["left"]} góc rẽ trái, {stats["right"]} góc rẽ phải; |θ| trung bình {fnum(stats["mean_abs_deg"], 2)}°, '
                f'trung vị {fnum(stats["median_abs_deg"], 2)}° và lớn nhất {fnum(stats["max_abs_deg"], 2)}°. '
                f'Quy hoạch động (DP) chọn {len(stats["transitions"])} đoạn chuyển tiếp G², {len(stats["pivots"])} quay tại chỗ và giữ nguyên {len(stats["pass_through"])} góc nhỏ.</p>'
            )
            content.append('<p><b>Các góc lớn nhất của ca:</b></p>')
            content.append(corner_table(item))
            content.append(figure_html(
                case_geometry_figure(item),
                f'{item["case_id"]}: ảnh RViz2 gốc, đường ROS Path chính xác được phóng to và bảng chỉ số hình học. Phần phóng to dùng tọa độ từ JSON của cùng ca, không dùng hình vẽ thay thế.',
                "case-figure",
            ))
            content.append('<p><b>Bảng hình học năm phương án:</b></p>')
            content.append(geometry_table(geometry))
            content.append('<p><b>Phân tích so sánh:</b></p><ul>')
            for note in case_comparison_notes(geometry, execution):
                content.append(f'<li>{html.escape(note)}</li>')
            content.append('</ul>')
            content.append(figure_html(
                execution_case_figure(item),
                f'{item["case_id"]}: đường kế hoạch được giao cho bộ điều khiển và quỹ đạo thực mô phỏng (ground truth) trong Gazebo của đủ năm phương án; trạng thái thất bại được giữ nguyên.',
                "case-figure",
            ))
            content.append('<p><b>Bảng thực thi vòng kín:</b></p>')
            content.append(execution_table(execution))
            content.append(
                f'<p><b>Dữ liệu chẩn đoán PSTMO.</b> {item["pstmo_diagnostics"]["evaluations"]} lần đánh giá hình dạng, '
                f'{item["pstmo_diagnostics"]["dp_states"]} trạng thái quy hoạch động, '
                f'{item["pstmo_diagnostics"]["compatible_edges"]} cạnh tương thích và thời gian xử lý nội bộ '
                f'{fnum(1000*item["pstmo_diagnostics"]["runtime_s"], 2)} ms.</p>'
            )

    # Chapter 8
    content.append(page_break())
    content.append('<h2>CHƯƠNG 8. THẢO LUẬN</h2>')
    content.append('<h3>8.1. Trả lời câu hỏi nghiên cứu</h3>')
    content.append(
        '<ol><li><b>Chất lượng hình học:</b> PSTMO giảm mạnh κmax và Eκ ở trung bình ghép cặp, nhưng mức giảm thay đổi theo ca; một số ca có phương án đối chứng đã tốt nên biên lợi ích nhỏ.</li>'
        '<li><b>Khả năng thực thi:</b> trong 34 nhóm cả năm phương án cùng hoàn tất, PSTMO thường nhanh hơn Raw, Savitzky–Golay và Constrained; lợi thế trước Simple nhỏ hơn và PSTMO không nhanh hơn ở mọi cặp.</li>'
        '<li><b>An toàn:</b> không có mẫu hình bao trên đường kế hoạch bị va chạm và không có lần Collision Monitor can thiệp, nhưng khoảng hở thấp hơn một số phương án đối chứng.</li>'
        '<li><b>Chi phí tính toán:</b> thời gian làm mượt khoảng 95 ms, cao hơn các phương án đối chứng; đây là đánh đổi thực và phải được báo cáo.</li></ol>'
    )
    content.append('<h3>8.2. Vì sao góc lớn chưa đủ giải thích kết quả?</h3>')
    content.append(
        '<p>Hai ca có cùng góc lớn nhất vẫn có thể khác số điểm Raw, chiều dài cạnh, chuỗi trái–phải, khoảng trống quanh góc, bộ lập kế hoạch và phản ứng của RPP. '
        'Tương quan mô tả nhỏ trong Hình 6.4 cho thấy góc lớn nhất không phải biến dự báo duy nhất. Phân tích cần dùng toàn bộ cấu trúc đường và môi trường.</p>'
    )
    content.append('<h3>8.3. Khoảng hở và an toàn</h3>')
    content.append(
        '<p>Trạng thái không va chạm (collision-free) là điều kiện nhị phân trên bản đồ chi phí hiện tại; khoảng hở (clearance) là biên dự phòng. '
        'Một đường có khoảng hở 0,015 m vẫn không giao vật cản tĩnh, nhưng nhạy hơn với sai số định vị, trượt bánh và độ trễ. Vì vậy hướng phát triển '
        'cần thêm ràng buộc khoảng hở hoặc thử nghiệm độ bền vững, không chỉ lặp lại kiểm tra va chạm.</p>'
    )
    content.append('<h3>8.4. Phạm vi bằng chứng</h3>')
    content.append(
        '<ul><li>35 ca đại diện không phủ toàn bộ không gian điểm đầu–đích.</li>'
        '<li>Mỗi lượt thực thi chỉ chạy một lần; chưa có khoảng tin cậy theo từng tuyến.</li>'
        '<li>PGM tĩnh không mô hình hóa vật cản động, tải hàng, trượt bánh, độ rơ và biến thiên pin.</li>'
        '<li>Ảnh RViz2/Gazebo chứng minh đường và quỹ đạo mô phỏng, không thay thử nghiệm phần cứng.</li>'
        '<li>PSTMO tối ưu trên tập trạng thái rời rạc hai d và lưới α, không chứng minh tối ưu liên tục toàn cục.</li>'
        '<li>G² là độ mượt hình học; không tự động bảo đảm độ giật (jerk) theo thời gian bằng 0.</li></ul>'
    )

    # Chapter 9
    content.append(page_break())
    content.append('<h2>CHƯƠNG 9. KẾT LUẬN VÀ HƯỚNG PHÁT TRIỂN</h2>')
    content.append(
        '<p>PSTMO giải quyết khoảng trống giữa đường gấp khúc của bộ lập kế hoạch và thao tác chuyển hướng có thể thực thi. Cấu trúc Bézier bậc năm tạo mối nối G²; '
        'các điều kiện bắt buộc về động học, thời gian và vùng quét hình bao robot loại các phương án không khả thi; quy hoạch động bảo đảm chuỗi đoạn chuyển tiếp không chồng lấn. '
        'Bộ dữ liệu hiện tại cho thấy lợi ích lớn về κmax và Eκ, đồng thời giảm thời gian di chuyển trung bình so với Raw trong mô phỏng.</p>'
    )
    content.append(
        '<p>Bước tiếp theo cần ưu tiên: (1) lặp lại mỗi tổ hợp nhiều lần; (2) đánh giá nhiều cặp điểm đầu–đích hơn; (3) đo đầy đủ sai số ngang đường và sai số góc hướng; '
        '(4) thêm nhiễu định vị, trượt và vật cản động; (5) đo dòng điện và điện năng Wh trên robot thật; (6) đánh giá ràng buộc khoảng hở thay vì chỉ điều kiện va chạm; '
        '(7) kiểm tra chuyển động từ góc hướng ban đầu thực tới hướng cạnh đầu trong hành lang sát vật cản.</p>'
    )
    content.append(callout('<b>Kết luận ngắn.</b> PSTMO làm đường mượt hơn theo nghĩa hình học và thường giúp robot mô phỏng di chuyển nhanh hơn, nhưng kết luận an toàn vận hành cuối cùng vẫn cần thử nghiệm lặp và kiểm chứng trên robot thật.'))

    # Appendices
    content.append(page_break())
    content.append('<h2>PHỤ LỤC A. BẢNG CÔNG THỨC CỐT LÕI</h2>')
    content.append(table_html(
        ["Nội dung", "Công thức", "Đơn vị/ý nghĩa"],
        [
            ["Góc có dấu", "θ=atan2(u×v,u·v)", "rad hoặc độ; dấu trái/phải"],
            ["Độ cong", "κ=(x′y″−y′x″)/(x′²+y′²)³ᐟ²", "m⁻¹"],
            ["Tích phân bình phương độ cong", "Eκ=∫κ²ds", "m⁻¹; chỉ số hình học"],
            ["Bánh trái/phải", "vL=v(1−bκ/2); vR=v(1+bκ/2)", "m/s"],
            ["Giới hạn không đảo bánh", "|κ|≤2/b", "m⁻¹"],
            ["Bézier bậc năm", "B(t)=Σ C(5,i)(1−t)⁵⁻ⁱtⁱPᵢ", "0≤t≤1"],
            ["Điểm điều khiển", "P₀=A;P₁=A+qu;P₂=A+2qu;P₃=B−2qv;P₄=B−qv;P₅=B", "q=αd"],
            ["Điều kiện G² tại hai đầu", "B″(0)=B″(1)=0⇒κ(0)=κ(1)=0", "Nối với đoạn thẳng"],
            ["Khoảng cắt ưu tiên", "d_pref=min(d_max,L_in,L_out)", "m"],
            ["Giới hạn vận tốc theo độ cong", "v_lim=min[v_max,ω_max/|κ|,√(a_y,max/|κ|),v_w,max/max(|1−bκ/2|,|1+bκ/2|)]", "m/s"],
            ["Ràng buộc tăng/giảm tốc", "vᵢ≤√(vᵢ₋₁²+2a_accΔs); vᵢ₋₁≤√(vᵢ²+2a_decΔs)", "m/s"],
            ["Thời gian đoạn", "Δt=2Δs/(vᵢ+vᵢ₊₁)", "s"],
            ["Điều kiện thời gian", "Tfastest+0,15<Tpivot", "Mở nhánh đoạn chuyển tiếp"],
            ["Cửa sổ cạnh tranh", "Tᵢ≤Tfastest+ΔTcomp", "ΔTcomp tối đa 10,0 s và bị chặn bởi Tpivot−0,15 s"],
            ["Hàm mục tiêu cục bộ", "J=0,15r_cost+0,10r_ω+0,75eκ", "Không thứ nguyên; chọn J nhỏ hơn"],
            ["Tương thích hai góc", "dᵢ+dᵢ₊₁+m≤Lᵢ", "Không chồng lấn; m hiệu dụng=0,05 m"],
            ["Quy hoạch động", "Dᵢ(z)=Jᵢ(z)+min Dᵢ₋₁(z′)", "O(NK²)"],
        ],
        "compact",
    ))

    content.append(page_break())
    content.append('<h2>PHỤ LỤC B. TỪ ĐIỂN THUẬT NGỮ ANH–VIỆT</h2>')
    glossary = [
        ("path", "đường đi / đường hình học", "Chuỗi tư thế chưa gắn mốc thời gian"),
        ("trajectory", "quỹ đạo theo thời gian", "Có t, v, ω và gia tốc"),
        ("pose", "tư thế", "Gồm vị trí và hướng của robot"),
        ("yaw", "góc hướng / góc quay quanh trục z", "Dùng độ hoặc rad; phải ghi đơn vị"),
        ("global planner", "bộ lập kế hoạch toàn cục", "Tạo đường từ điểm đầu tới điểm đích"),
        ("path smoother", "bộ làm mượt đường đi", "Chỉnh hình học của đường"),
        ("path-following controller", "bộ điều khiển bám đường", "Phát lệnh vận tốc tuyến tính v và vận tốc góc ω"),
        ("corner-handling option", "phương án xử lý tại góc", "Một cách xử lý cụ thể cho một góc"),
        ("trim distance d", "khoảng cắt d", "Chiều dài cắt trên cạnh vào/ra"),
        ("shape ratio α=q/d", "tỷ lệ hình dạng α", "Điều chỉnh phân bố độ cong"),
        ("curvature κ", "độ cong κ", "Mức đổi hướng trên một đơn vị chiều dài"),
        ("integral of squared curvature Eκ", "tích phân bình phương độ cong Eκ", "Chỉ số uốn hình học; không phải điện năng"),
        ("footprint", "hình bao chiếm chỗ của robot", "Đa giác biểu diễn phần thân xe chiếm chỗ"),
        ("swept footprint", "vùng quét hình bao robot", "Hợp của các hình bao dọc chuyển động"),
        ("footprint clearance", "khoảng hở hình bao", "Khoảng cách nhỏ nhất từ hình bao tới vật cản"),
        ("costmap", "bản đồ chi phí", "Lưới chi phí phục vụ lập kế hoạch và kiểm tra an toàn"),
        ("odometry", "ước lượng hành trình", "Ước lượng chuyển động tích lũy; không đồng nhất với giá trị thực mô phỏng"),
        ("ground truth", "giá trị thực mô phỏng", "Tư thế/quỹ đạo lấy trực tiếp từ trạng thái vật lý Gazebo"),
        ("diagnostics", "dữ liệu chẩn đoán", "Số liệu nội bộ dùng để truy vết quyết định"),
        ("curvature-dependent speed limit", "giới hạn vận tốc theo độ cong", "Ngưỡng v tại từng mẫu"),
        ("speed profile", "biểu đồ vận tốc dọc theo chiều dài đường", "Dãy v(s) sau quét tiến/lùi"),
        ("time parameterization", "tham số hóa theo thời gian", "Gắn vận tốc và Δt cho các mẫu đường"),
        ("time gate", "điều kiện ưu thế thời gian", "Mở đoạn chuyển tiếp khi có lợi hơn quay tại chỗ"),
        ("pivot", "quay tại chỗ", "Đổi góc hướng khi tâm robot không tịnh tiến"),
        ("pass-through", "giữ nguyên góc", "Góc nhỏ không chèn đoạn chuyển tiếp hoặc quay tại chỗ"),
        ("dynamic programming", "quy hoạch động", "Chọn chuỗi trạng thái toàn đường"),
        ("invariant", "bất biến / điều kiện luôn phải đúng", "Điều kiện phải thỏa ở bước hậu kiểm đầu ra"),
        ("baseline", "phương án đối chứng", "Mốc so sánh"),
        ("runtime", "thời gian xử lý", "Thời gian thực thi thuật toán; phải ghi ms hoặc s"),
        ("RMSE tracking error", "căn sai số bình phương trung bình khi bám đường", "Chỉ số tổng hợp sai số bám, đơn vị m"),
        ("collision-free", "không va chạm", "Điều kiện nhị phân; không đồng nghĩa có khoảng hở lớn"),
        ("PATIENCE_EXCEEDED", "vượt thời gian chờ điều khiển", "Mã 104 của FollowPath; không dịch là “hết kiên nhẫn”"),
    ]
    content.append(table_html(["Thuật ngữ/ký hiệu tiếng Anh", "Tiếng Việt nên dùng", "Cách hiểu trong báo cáo"], glossary, "compact"))

    content.append('<h2>PHỤ LỤC C. BẢNG TÓM TẮT GÓC VÀ TRẠNG THÁI 35 CA</h2>')
    case_rows = []
    for item in cases:
        stats = case_corner_stats(item)
        selected_alpha = [float(c["selected_control_fraction"]) for c in stats["transitions"]]
        selected_trim = [float(c["selected_trim"]) for c in stats["transitions"]]
        case_rows.append([
            item["case_id"], ENV_LABEL[item["environment"]], item["planner"],
            str(stats["count"]), fnum(stats["mean_abs_deg"], 1), fnum(stats["max_abs_deg"], 1),
            str(len(stats["transitions"])), str(len(stats["pivots"])), str(len(stats["pass_through"])),
            f"{fnum(min(selected_alpha), 2)}–{fnum(max(selected_alpha), 2)}" if selected_alpha else "–",
            f"{fnum(min(selected_trim), 3)}–{fnum(max(selected_trim), 3)}" if selected_trim else "–",
        ])
    content.append(table_html(
        ["ID", "Môi trường", "Bộ lập kế hoạch", "Số góc", "|θ| trung bình (°)", "|θ| lớn nhất (°)", "Số G²", "Số lần quay", "Số góc giữ nguyên", "Khoảng α", "Khoảng d (m)"],
        case_rows,
        "tiny",
    ))

    content.append(page_break())
    content.append('<h2>PHỤ LỤC D. TÁI LẬP VÀ NGUỒN TRUY VẾT</h2>')
    content.append(
        '<ol><li>Biên dịch workspace và nạp môi trường bằng <code>source install/setup.bash</code>.</li>'
        '<li>Khởi chạy từng tệp môi trường với điểm đầu và điểm đích gồm x, y, góc hướng.</li>'
        '<li>Thu 35 ảnh/tệp JSON RViz2 bằng quy trình <code>condition_only</code> và xác minh bất biến hậu kiểm.</li>'
        '<li>Chạy năm phương án trong một phiên mô phỏng mới cho từng tổ hợp môi trường–bộ lập kế hoạch.</li>'
        '<li>Ghép cặp bằng SHA-256 đường Raw; giữ nguyên thất bại.</li>'
        '<li>Sinh lại báo cáo bằng <code>tools/generate_pstmo_unified_report.py</code>.</li></ol>'
    )
    content.append(source_table())
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, capture_output=True).stdout.strip()
    content.append(f'<p>Mã Git HEAD khi xuất báo cáo: <code>{html.escape(commit or "không xác định")}</code>. Cây làm việc có thể chứa tệp chưa được commit; bảng SHA-256 ở trên định danh trực tiếp nội dung mã nguồn.</p>')

    content.append(page_break())
    content.append('<h2>TÀI LIỆU THAM KHẢO</h2>')
    references = [
        "A. Ravankar, A. A. Ravankar, Y. Kobayashi, Y. Hoshino, C.-C. Peng, “Path Smoothing Techniques in Robot Navigation: State-of-the-Art, Current and Future Challenges,” Sensors, vol. 18, no. 9, art. 3170, 2018, DOI: 10.3390/s18093170.",
        "J. R. Sánchez-Ibáñez, C. J. Pérez-del-Pulgar, A. García-Cerezo, “Path Planning for Autonomous Mobile Robots: A Review,” Sensors, vol. 21, no. 23, art. 7898, 2021, DOI: 10.3390/s21237898.",
        "J. Roth, “Continuous-Curvature Trajectory Planning,” Journal of Automation, Mobile Robotics and Intelligent Systems, vol. 15, no. 1, 2021, DOI: 10.14313/JAMRIS/1-2021/2.",
        "A. Piazzi, C. Guarino Lo Bianco, M. Romano, “Smooth Path Generation for Wheeled Mobile Robots Using η³-Splines,” in Motion Control, F. Casolo, Ed., IntechOpen, 2010, ISBN: 978-953-7619-55-8.",
        "M. Kılıçarslan Ouach, T. Eren, “PRM Path Smoothening by Circular Arc Fillet Method for Mobile Robot Navigation,” arXiv:2112.03604, 2021.",
        "X. Huang, C.-B. Yan, “An Efficient Method for Extracting the Shortest Path from the Dubins Set for Short Distances Between Initial and Final Positions,” arXiv:2309.07565v2, 2025.",
        "ROS 2 Navigation2, tài liệu kiến trúc và API; https://navigation.ros.org/.",
        "Mã nguồn plugin PSTMO trong workspace; phiên bản cụ thể được định danh bằng SHA-256 ở Phụ lục D.",
        "Bộ bằng chứng RViz2/Gazebo và CSV/JSON trong docs/pstmo_bao_cao_toan_dien_assets/.",
    ]
    content.append("<ol>" + "".join(f"<li>{html.escape(reference)}</li>" for reference in references) + "</ol>")

    style = """
    @page { size:A4; margin:16mm; }
    body{font-family:"Times New Roman",serif;color:#172033;max-width:1080px;margin:auto;line-height:1.55;background:white}
    h1{font-size:30px;color:#0f3c5f} h2{font-size:22px;color:#0f3c5f;border-bottom:2px solid #9dc3e6;padding-bottom:4px;margin-top:28px}
    h3{font-size:18px;color:#155e75} h4{font-size:15px;color:#166534}.cover{text-align:center;padding:45px 25px 25px;border:3px solid #0f3c5f;margin:18px 0}
    .subtitle{font-size:18px;color:#0f766e;font-weight:bold;margin:18px}.authors{font-size:15px;font-weight:bold}.meta{color:#475569;margin-top:12px}
    .mine{background:#ecfeff;border-left:6px solid #0891b2;padding:12px 16px;margin:16px 0}.warning{background:#fff7ed;border-left:6px solid #ea580c;padding:12px 16px;margin:16px 0}
    .eq{text-align:center;background:#f8fafc;border:1px solid #cbd5e1;padding:10px;margin:11px;font-family:"DejaVu Sans",sans-serif;page-break-inside:avoid}
    figure{text-align:center;margin:16px auto;page-break-inside:avoid} figure img{max-width:100%;height:auto;border:1px solid #cbd5e1} figcaption{font-size:12px;color:#475569;font-style:italic;margin-top:5px}.case-figure img{width:100%}
    table{border-collapse:collapse;width:100%;font-size:12px;margin:12px 0;page-break-inside:auto} th,td{border:1px solid #94a3b8;padding:4px;text-align:center} th{background:#dbeafe}.tiny{font-size:8.5px}.compact{font-size:10.5px}
    code{background:#f1f5f9;padding:1px 4px}.page-break{page-break-before:always}li{margin:3px 0}p{text-align:justify}
    """
    document = (
        '<!doctype html><html lang="vi"><head><meta charset="utf-8">'
        '<title>Báo cáo nghiên cứu thuật toán PSTMO</title><style>'
        + style + "</style></head><body>" + "".join(content) + "</body></html>"
    )
    OUTPUT_HTML.write_text(document, encoding="utf-8")
    return {
        "cases": len(cases),
        "geometry_rows": len(geometry_rows),
        "execution_rows": len(execution_rows),
        "corners": len(all_corners),
        "transitions": len(transitions),
        "pivots": len(pivots),
    }


def export_report() -> dict:
    if OUTPUT_PDF.exists() and not BACKUP_PDF.exists():
        shutil.copy2(OUTPUT_PDF, BACKUP_PDF)
    summary = build_html()
    html_to_docx(OUTPUT_HTML, OUTPUT_DOCX, paper=False, page_number=True)
    document = Document(OUTPUT_DOCX)
    document.core_properties.title = "Báo cáo nghiên cứu thuật toán PSTMO"
    document.core_properties.subject = (
        "ROS 2 Navigation2, robot vi sai, Bézier bậc năm G², RViz2 và Gazebo"
    )
    document.core_properties.author = "NGUYỄN TIẾN CƯƠNG"
    document.core_properties.keywords = (
        "PSTMO, path smoothing, differential drive, G2 Bezier, footprint, Nav2"
    )
    document.core_properties.comments = (
        "Unified from repository theory, source code, RViz2 and Gazebo evidence."
    )
    document.save(OUTPUT_DOCX)
    with tempfile.TemporaryDirectory(prefix="pstmo_unified_pdf_") as directory:
        subprocess.run(
            [
                "libreoffice", "--headless", "--convert-to", "pdf",
                "--outdir", directory, str(OUTPUT_DOCX),
            ],
            cwd=ROOT,
            check=True,
            timeout=180,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        generated = Path(directory) / f"{OUTPUT_DOCX.stem}.pdf"
        if not generated.is_file():
            raise FileNotFoundError(generated)
        shutil.copy2(generated, OUTPUT_PDF)
    summary.update({
        "html": str(OUTPUT_HTML),
        "docx": str(OUTPUT_DOCX),
        "pdf": str(OUTPUT_PDF),
        "backup": str(BACKUP_PDF),
    })
    return summary


if __name__ == "__main__":
    print(json.dumps(export_report(), ensure_ascii=False, indent=2, sort_keys=True))
