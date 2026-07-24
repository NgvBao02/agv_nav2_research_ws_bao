#!/usr/bin/env python3

"""Generate data-driven REV-ECIT 2026 figures, paper, and full supplement.

The script deliberately prefers the system Python packages because a user-site
NumPy may be newer than the binary Matplotlib shipped by Ubuntu.
"""

from __future__ import annotations

import argparse
import ast
import collections
import csv
import html
import json
import math
from pathlib import Path
import site
import shutil
import statistics
import struct
import sys
from typing import Iterable

# ROS Jazzy's Ubuntu Matplotlib is compiled against Ubuntu's NumPy.  Excluding
# the user site before either is imported prevents an incompatible user-level
# NumPy from shadowing that matching pair.
_USER_SITE = site.getusersitepackages()
if isinstance(_USER_SITE, str) and _USER_SITE in sys.path:
    sys.path.remove(_USER_SITE)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import yaml


ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
ASSETS = DOCS / "rev_ecit_2026_assets"
GEOMETRY = ROOT / "results" / "conference_geometry_20260725"
EXECUTION = ROOT / "results" / "conference_execution_20260725"
AUDIT = ROOT / "results" / "closed_loop_audit_20260725"
GUI = ROOT / "results" / "gui_validation_20260724"
FONT_REGULAR = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

METHOD_ORDER = (
    "raw",
    "simple",
    "savitzky_golay",
    "constrained",
    "pivot_g2_fixed",
    "pivot_g2",
    "adaptive_hybrid_fixed",
    "adaptive_hybrid",
)
METHOD_LABEL = {
    "raw": "Raw",
    "simple": "Simple",
    "savitzky_golay": "Savitzky–Golay",
    "constrained": "Constrained",
    "pivot_g2_fixed": "Pivot–G2 cố định",
    "pivot_g2": "Pivot–G2 thích nghi",
    "adaptive_hybrid_fixed": "Hybrid cố định",
    "adaptive_hybrid": "Hybrid thích nghi",
}
METHOD_SHORT = {
    "raw": "Raw",
    "simple": "Simple",
    "savitzky_golay": "SG",
    "constrained": "Con.",
    "pivot_g2_fixed": "P–G2 F",
    "pivot_g2": "P–G2 A",
    "adaptive_hybrid_fixed": "Hyb. F",
    "adaptive_hybrid": "Hyb. A",
}
METHOD_COLOR = {
    "raw": "#ba3b46",
    "simple": "#f59e0b",
    "savitzky_golay": "#06b6d4",
    "constrained": "#22a06b",
    "pivot_g2_fixed": "#8b5cf6",
    "pivot_g2": "#c026d3",
    "adaptive_hybrid_fixed": "#64748b",
    "adaptive_hybrid": "#1d4ed8",
}
ENV_LABEL = {
    "research_warehouse": "Kho nghiên cứu",
    "narrow_aisles": "Lối hẹp",
    "office_maze": "Văn phòng",
    "open_arena": "Không gian mở",
    "warehouse_cross_aisles": "Kho giao cắt",
    "warehouse_dispatch": "Kho điều phối",
    "warehouse_long_aisles": "Kho lối dài",
}
PLANNER_ORDER = (
    "ThetaStar",
    "NavFnAStar",
    "NavFnDijkstra",
    "Smac2D",
    "SmacHybrid",
)


def as_float(value, default=math.nan):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def mean(values: Iterable[float]):
    finite = [value for value in values if math.isfinite(value)]
    return statistics.mean(finite) if finite else math.nan


def percent_change(new, old):
    return 100.0 * (new - old) / old if old and math.isfinite(old) else math.nan


def selected_radius_summary(rows):
    """Summarize radii selected by the adaptive Pivot-G2 DP.

    Diagnostics are serialized through ROS as a Python-compatible list of
    dictionaries. Only the standalone adaptive Pivot-G2 rows are used so one
    optimized path is not counted again through the Hybrid wrapper.
    """
    legacy_bank = (0.20, 0.30, 0.40, 0.50, 0.60, 0.75, 1.00, 1.25, 1.50)
    by_environment = collections.defaultdict(list)
    for row in rows:
        if row.get("method") != "pivot_g2" or not row.get("ok"):
            continue
        payload = row.get("pivot_corner_search", "")
        try:
            corners = ast.literal_eval(payload) if payload else []
        except (SyntaxError, ValueError):
            continue
        if not isinstance(corners, list):
            continue
        for corner in corners:
            if not isinstance(corner, dict):
                continue
            radius = as_float(corner.get("selected_radius"))
            if math.isfinite(radius) and radius > 0.0:
                by_environment[row["environment"]].append(radius)

    output = {}
    all_radii = []
    for environment in ENV_LABEL:
        values = sorted(by_environment.get(environment, []))
        all_radii.extend(values)
        if not values:
            continue
        outside_bank = sum(
            all(abs(radius - fixed) > 1.0e-6 for fixed in legacy_bank)
            for radius in values
        )
        output[environment] = {
            "count": len(values),
            "minimum": values[0],
            "median": statistics.median(values),
            "p90": float(np.percentile(values, 90)),
            "maximum": values[-1],
            "outside_legacy_fraction": outside_bank / len(values),
        }
    if all_radii:
        values = sorted(all_radii)
        outside_bank = sum(
            all(abs(radius - fixed) > 1.0e-6 for fixed in legacy_bank)
            for radius in values
        )
        output["all"] = {
            "count": len(values),
            "minimum": values[0],
            "median": statistics.median(values),
            "p90": float(np.percentile(values, 90)),
            "maximum": values[-1],
            "outside_legacy_fraction": outside_bank / len(values),
        }
    return output


def fmt(value, digits=3):
    if value is None or not math.isfinite(float(value)):
        return "–"
    return f"{float(value):.{digits}f}"


def pct(value, digits=1):
    if value is None or not math.isfinite(float(value)):
        return "–"
    return f"{float(value):.{digits}f}%"


def load_geometry():
    rows = []
    for path in sorted(GEOMETRY.glob("*.csv")):
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                row["environment"] = path.stem
                row["ok"] = row["success"].lower() == "true"
                rows.append(row)
    if not rows:
        raise RuntimeError(f"No conference geometry CSV files found in {GEOMETRY}")
    return rows


def load_execution():
    rows = []
    if not EXECUTION.exists():
        return rows
    for path in sorted(EXECUTION.rglob("*.json")):
        if path.name.endswith("_summary.json"):
            continue
        try:
            row = json.loads(path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            continue
        if "method" not in row or "success" not in row:
            continue
        row["_path"] = str(path.relative_to(ROOT))
        rows.append(row)
    return rows


def primary_execution_matrix(rows):
    """Return the controlled 8-smoother × 3-speed comparison matrix."""
    return [
        row
        for row in rows
        if Path(row.get("_path", "")).parent ==
        Path("results/conference_execution_20260725/lower_left_diagonal")
        and row.get("environment") == "research_warehouse"
        and row.get("scenario") == "lower_left_diagonal"
        and row.get("planner") == "ThetaStar"
        and row.get("method") in METHOD_ORDER
    ]


def load_map_validation():
    rows = []
    validation = AUDIT / "map_validation"
    if validation.exists():
        for path in sorted(validation.rglob("*_pivot_g2.json")):
            row = json.loads(path.read_text(encoding="utf-8"))
            row["_path"] = str(path.relative_to(ROOT))
            rows.append(row)
    research = AUDIT / "terminal_v2" / "lower_left_diagonal_pivot_g2.json"
    if research.exists():
        row = json.loads(research.read_text(encoding="utf-8"))
        row["_path"] = str(research.relative_to(ROOT))
        rows.append(row)
    return rows


def aggregate_geometry(rows, key_fields):
    groups = collections.defaultdict(list)
    for row in rows:
        groups[tuple(row[key] for key in key_fields)].append(row)
    result = {}
    for key, group in groups.items():
        ok = [row for row in group if row["ok"]]
        result[key] = {
            "attempts": len(group),
            "successes": len(ok),
            "energy": mean(
                as_float(row["translation_curvature_energy_1pm"]) for row in ok
            ),
            "clearance": mean(
                as_float(row["footprint_clearance_min_m"]) for row in ok
            ),
            "length": mean(as_float(row["path_length_m"]) for row in ok),
            "deviation": mean(as_float(row["deviation_rmse_m"]) for row in ok),
            "runtime_ms": 1000.0
            * mean(as_float(row["algorithm_time_s"]) for row in ok),
            "collisions": sum(
                int(as_float(row["footprint_collision_sample_count"], 0.0))
                for row in ok
            ),
        }
    return result


def validate_pairing(rows):
    groups = collections.defaultdict(set)
    for row in rows:
        groups[
            (
                row["environment"],
                row["scenario"],
                row["planner"],
                row["repetition"],
            )
        ].add(row["raw_path_sha256"])
    bad = [key for key, hashes in groups.items() if len(hashes) != 1]
    return len(groups), bad


def paired_geometry(rows, first_method, second_method, metric, higher_better=False):
    index = {}
    for row in rows:
        index[
            (
                row["environment"],
                row["scenario"],
                row["planner"],
                row["repetition"],
                row["method"],
            )
        ] = row
    common = {
        key[:-1]
        for key in index
        if key[-1] in {first_method, second_method}
    }
    differences = []
    ratios = []
    wins = 0
    for key in common:
        first = index.get((*key, first_method))
        second = index.get((*key, second_method))
        if not first or not second or not first["ok"] or not second["ok"]:
            continue
        first_value = as_float(first[metric])
        second_value = as_float(second[metric])
        if not math.isfinite(first_value) or not math.isfinite(second_value):
            continue
        differences.append(first_value - second_value)
        if abs(second_value) > 1.0e-12:
            ratios.append(first_value / second_value)
        wins += (
            first_value > second_value if higher_better else first_value < second_value
        )
    return {
        "n": len(differences),
        "wins": wins,
        "mean_difference": mean(differences),
        "mean_ratio": mean(ratios),
        "median_ratio": statistics.median(ratios) if ratios else math.nan,
    }


def set_plot_style():
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 8.5,
            "axes.titlesize": 10,
            "axes.labelsize": 8.5,
            "legend.fontsize": 7.5,
            "figure.dpi": 160,
            "savefig.dpi": 220,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "grid.alpha": 0.25,
        }
    )


def save_geometry_summary(overall):
    methods = [method for method in METHOD_ORDER if (method,) in overall]
    energy = [overall[(method,)]["energy"] for method in methods]
    clearance = [100.0 * overall[(method,)]["clearance"] for method in methods]
    success = [
        100.0
        * overall[(method,)]["successes"]
        / overall[(method,)]["attempts"]
        for method in methods
    ]
    figure, axes = plt.subplots(1, 3, figsize=(10.6, 3.25))
    x = np.arange(len(methods))
    colors = [METHOD_COLOR[method] for method in methods]
    labels = [METHOD_SHORT[method] for method in methods]
    axes[0].bar(x, energy, color=colors)
    axes[0].set_ylabel(r"$\int \kappa^2 ds$ (m$^{-1}$)")
    axes[0].set_title("(a) Năng lượng độ cong tịnh tiến")
    axes[1].bar(x, clearance, color=colors)
    axes[1].set_ylabel("Khoảng hở nhỏ nhất TB (cm)")
    axes[1].set_title("(b) Khoảng hở footprint")
    axes[2].bar(x, success, color=colors)
    axes[2].set_ylim(96, 100.2)
    axes[2].set_ylabel("Tỷ lệ sinh đường thành công (%)")
    axes[2].set_title("(c) Độ bền chuỗi xử lý")
    for axis in axes:
        axis.set_xticks(x)
        axis.set_xticklabels(labels, rotation=38, ha="right")
        axis.grid(axis="y")
    figure.suptitle("7.200 phép đo hình học: 7 map × 5 planner × 8 phương pháp × 3 lần")
    figure.tight_layout()
    figure.savefig(ASSETS / "figure_02_geometry_overview.png", bbox_inches="tight")
    plt.close(figure)


def save_map_ratios(map_method):
    environments = [env for env in ENV_LABEL if (env, "raw") in map_method]
    methods = ("simple", "pivot_g2_fixed", "pivot_g2", "adaptive_hybrid")
    ratios = np.array(
        [
            [
                map_method[(env, method)]["energy"]
                / map_method[(env, "raw")]["energy"]
                for method in methods
            ]
            for env in environments
        ]
    )
    figure, axis = plt.subplots(figsize=(8.9, 3.65))
    image = axis.imshow(ratios, cmap="YlGnBu_r", vmin=0.0, vmax=1.0, aspect="auto")
    axis.set_xticks(range(len(methods)))
    axis.set_xticklabels([METHOD_LABEL[method] for method in methods], rotation=20)
    axis.set_yticks(range(len(environments)))
    axis.set_yticklabels([ENV_LABEL[env] for env in environments])
    for row in range(ratios.shape[0]):
        for column in range(ratios.shape[1]):
            value = ratios[row, column]
            axis.text(
                column,
                row,
                f"{100.0 * (1.0 - value):.0f}%↓",
                ha="center",
                va="center",
                color="white" if value < 0.48 else "#102a43",
                fontweight="bold",
            )
    colorbar = figure.colorbar(image, ax=axis, pad=0.02)
    colorbar.set_label("Tỷ số năng lượng độ cong so với Raw")
    axis.set_title("Mức giảm năng lượng độ cong trên từng môi trường")
    figure.tight_layout()
    figure.savefig(ASSETS / "figure_03_map_energy_ratio.png", bbox_inches="tight")
    plt.close(figure)


def speed_label(value):
    return "thích nghi" if abs(value) < 1.0e-9 else f"{value:.2f} m/s"


def save_execution_summary(rows):
    if not rows:
        return
    speeds = sorted(
        {as_float(row.get("fixed_speed_limit_mps"), 0.0) for row in rows},
        key=lambda value: (value == 0.0, value),
    )
    methods = [method for method in METHOD_ORDER if any(
        row.get("method") == method for row in rows
    )]
    figure, axes = plt.subplots(1, 3, figsize=(11.0, 3.45))
    x = np.arange(len(methods))
    width = 0.82 / max(len(speeds), 1)
    for speed_index, speed in enumerate(speeds):
        selected = {
            row["method"]: row
            for row in rows
            if abs(as_float(row.get("fixed_speed_limit_mps"), 0.0) - speed)
            < 1.0e-9
            and row.get("success")
        }
        offsets = x + (speed_index - 0.5 * (len(speeds) - 1)) * width
        axes[0].bar(
            offsets,
            [as_float(selected.get(method, {}).get("execution_time_s")) for method in methods],
            width=width,
            label=speed_label(speed),
        )
        axes[1].bar(
            offsets,
            [
                100.0
                * as_float(selected.get(method, {}).get("tracking_rmse_m"))
                for method in methods
            ],
            width=width,
        )
        axes[2].bar(
            offsets,
            [
                100.0
                * as_float(
                    selected.get(method, {}).get("estimated_tracking_rmse_m")
                )
                for method in methods
            ],
            width=width,
        )
    titles = (
        "(a) Thời gian thực thi",
        "(b) RMSE theo ground truth Gazebo",
        "(c) RMSE robot/controller nhận biết",
    )
    ylabels = ("s", "cm", "cm")
    for axis, title, ylabel in zip(axes, titles, ylabels):
        axis.set_xticks(x)
        axis.set_xticklabels([METHOD_SHORT[m] for m in methods], rotation=38, ha="right")
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.grid(axis="y")
    axes[0].legend(frameon=False, ncol=min(3, len(speeds)))
    figure.suptitle("Chạy kín Gazebo: cùng planner ThetaStar và cùng bài lower_left_diagonal")
    figure.tight_layout()
    figure.savefig(ASSETS / "figure_04_speed_tracking.png", bbox_inches="tight")
    plt.close(figure)


def select_trace_record(rows):
    preferred = [
        row
        for row in rows
        if row.get("method") == "adaptive_hybrid"
        and abs(as_float(row.get("fixed_speed_limit_mps"), 0.0)) < 1.0e-9
        and row.get("success")
    ]
    if preferred:
        return preferred[0]
    preferred = [row for row in rows if row.get("method") == "pivot_g2" and row.get("success")]
    return preferred[0] if preferred else None


def save_path_tracking(rows):
    row = select_trace_record(rows)
    if not row:
        return
    reference = row.get("selected_path_xy", [])
    truth = row.get("ground_truth_state_trace", [])
    estimated = row.get("estimated_map_state_trace", [])
    figure, axes = plt.subplots(1, 2, figsize=(9.2, 3.65))
    if reference:
        axes[0].plot(
            [point[0] for point in reference],
            [point[1] for point in reference],
            "--",
            color="#111827",
            linewidth=2.0,
            label="Quỹ đạo tham chiếu",
        )
    if truth:
        axes[0].plot(
            [sample[1] for sample in truth],
            [sample[2] for sample in truth],
            color=METHOD_COLOR["adaptive_hybrid"],
            linewidth=1.6,
            label="Ground truth Gazebo",
        )
    if estimated:
        axes[0].plot(
            [sample[1] for sample in estimated],
            [sample[2] for sample in estimated],
            color="#f97316",
            linewidth=1.1,
            label="TF map→base_link",
        )
    axes[0].axis("equal")
    axes[0].set_title("(a) Quỹ đạo tham chiếu và quỹ đạo chạy")
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    axes[0].legend(frameon=False)
    telemetry = row.get("adaptive_speed_trace", [])
    if telemetry:
        axes[1].plot(
            [sample[0] for sample in telemetry],
            [sample[4] for sample in telemetry],
            label="v đo",
            color="#111827",
            linewidth=1.2,
        )
        axes[1].plot(
            [sample[0] for sample in telemetry],
            [sample[6] for sample in telemetry],
            label="v lệnh",
            color=METHOD_COLOR["adaptive_hybrid"],
            linewidth=1.2,
        )
        axes[1].plot(
            [sample[0] for sample in telemetry],
            [sample[7] for sample in telemetry],
            label="Trần profile",
            color="#f97316",
            linewidth=1.0,
        )
    axes[1].set_title("(b) Profile vận tốc vòng kín")
    axes[1].set_xlabel("Thời gian (s)")
    axes[1].set_ylabel("Vận tốc (m/s)")
    axes[1].grid()
    axes[1].legend(frameon=False)
    figure.tight_layout()
    figure.savefig(ASSETS / "figure_05_trace_speed.png", bbox_inches="tight")
    plt.close(figure)


def save_map_validation(rows):
    if not rows:
        return
    rows = sorted(rows, key=lambda row: list(ENV_LABEL).index(row["environment"]))
    labels = [ENV_LABEL[row["environment"]] for row in rows]
    x = np.arange(len(rows))
    figure, axes = plt.subplots(1, 3, figsize=(10.7, 3.4))
    axes[0].bar(
        x,
        [100.0 * as_float(row.get("tracking_rmse_m")) for row in rows],
        color="#2563eb",
    )
    axes[1].bar(
        x,
        [100.0 * as_float(row.get("estimated_tracking_rmse_m")) for row in rows],
        color="#0f766e",
    )
    axes[2].bar(
        x,
        [100.0 * as_float(row.get("localization_position_error_p95_m")) for row in rows],
        color="#d97706",
    )
    for axis, title in zip(
        axes,
        (
            "RMSE bám theo ground truth",
            "RMSE bám trong hệ điều khiển",
            "Sai số định vị P95",
        ),
    ):
        axis.set_xticks(x)
        axis.set_xticklabels(labels, rotation=42, ha="right")
        axis.set_ylabel("cm")
        axis.set_title(title)
        axis.grid(axis="y")
    figure.suptitle("Kiểm chứng chạy kín Pivot–G2 thích nghi trên cả 7 môi trường")
    figure.tight_layout()
    figure.savefig(ASSETS / "figure_06_all_map_closed_loop.png", bbox_inches="tight")
    plt.close(figure)


def save_all_map_path_overlays(rows):
    """Render selected paths and measured Gazebo traces on every map."""
    selected = {}
    for environment in ENV_LABEL:
        candidates = [
            row for row in rows
            if row.get("environment") == environment
            and row.get("success")
            and row.get("selected_path_xy")
            and row.get("ground_truth_state_trace")
        ]
        if environment in ("warehouse_dispatch", "research_warehouse"):
            preferred_method = "adaptive_hybrid"
        else:
            preferred_method = "pivot_g2"
        preferred = [
            row for row in candidates
            if row.get("method") == preferred_method
            and math.isclose(
                as_float(row.get("fixed_speed_limit_mps")), 0.22,
                abs_tol=1.0e-12,
            )
        ]
        if preferred or candidates:
            selected[environment] = (preferred or candidates)[0]

    figure, axes = plt.subplots(2, 4, figsize=(13.2, 6.3))
    for axis, environment in zip(axes.flat, ENV_LABEL):
        map_yaml = (
            ROOT / "src" / "vacuum_robot_gazebo" / "maps" /
            f"{environment}.yaml"
        )
        metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
        occupancy = np.asarray(Image.open(map_yaml.parent / metadata["image"]))
        resolution = float(metadata["resolution"])
        origin_x, origin_y = map(float, metadata["origin"][:2])
        height, width = occupancy.shape[:2]
        axis.imshow(
            occupancy,
            cmap="gray",
            vmin=0,
            vmax=255,
            origin="upper",
            extent=(
                origin_x,
                origin_x + width * resolution,
                origin_y,
                origin_y + height * resolution,
            ),
        )
        row = selected.get(environment)
        if row is not None:
            path = np.asarray(row["selected_path_xy"], dtype=float)
            trace = np.asarray(
                [[sample[1], sample[2]]
                 for sample in row["ground_truth_state_trace"]],
                dtype=float,
            )
            axis.plot(
                path[:, 0], path[:, 1],
                color="#1d4ed8", linewidth=1.8, label="đường chọn",
            )
            axis.plot(
                trace[:, 0], trace[:, 1],
                color="#f97316", linewidth=1.1, alpha=0.9,
                label="Gazebo GT",
            )
            axis.scatter(
                path[[0, -1], 0], path[[0, -1], 1],
                c=["#16a34a", "#dc2626"], s=18, zorder=4,
            )
            speed = speed_label(as_float(row.get("fixed_speed_limit_mps"), 0.0))
            method = METHOD_SHORT.get(row.get("method"), row.get("method", ""))
            axis.text(
                0.02, 0.02, f"{method}, {speed}",
                transform=axis.transAxes, fontsize=7,
                bbox={
                    "facecolor": "white",
                    "alpha": 0.82,
                    "edgecolor": "none",
                },
            )
        axis.set_title(ENV_LABEL[environment], fontsize=9, weight="bold")
        axis.set_aspect("equal", adjustable="box")
        axis.set_xticks([])
        axis.set_yticks([])
    axes.flat[-1].axis("off")
    handles, labels = axes.flat[0].get_legend_handles_labels()
    figure.legend(
        handles, labels, loc="lower right", bbox_to_anchor=(0.98, 0.08),
        frameon=False, fontsize=8,
    )
    figure.suptitle(
        "Đường được chọn và ground truth Gazebo trên toàn bộ bảy môi trường",
        fontsize=12,
        weight="bold",
    )
    figure.tight_layout(rect=(0, 0, 1, 0.95))
    figure.savefig(ASSETS / "figure_11_all_map_paths.png", bbox_inches="tight")
    plt.close(figure)


def read_binary_stl(path):
    with path.open("rb") as stream:
        stream.read(80)
        triangle_count = struct.unpack("<I", stream.read(4))[0]
        data = np.fromfile(stream, dtype=np.dtype([
            ("normal", "<f4", (3,)),
            ("vertices", "<f4", (3, 3)),
            ("attribute", "<u2"),
        ]), count=triangle_count)
    return np.asarray(data["vertices"], dtype=float)


def save_robot_render():
    mesh_dir = (
        ROOT
        / "src"
        / "vacuum_robot_gazebo"
        / "models"
        / "vacuum_robot"
        / "meshes"
    )
    components = (
        ("base_link.stl", (-0.035315, 0.200943, -0.211698), "#9aa4b2"),
        ("left_wheel_link_1.stl", (-0.0353034, 0.0735430, -0.2091883), "#1f2937"),
        ("right_wheel_link_1.stl", (-0.0353129, 0.3283430, -0.2091955), "#1f2937"),
    )
    figure = plt.figure(figsize=(8.5, 5.4))
    axis = figure.add_subplot(111, projection="3d")
    for name, translation, color in components:
        triangles = read_binary_stl(mesh_dir / name) * 0.001
        triangles += np.asarray(translation)
        # Do not subsample individual facets: STL triangles are not an ordered
        # surface strip, so dropping every nth face creates artificial holes.
        stride = 1
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        normals = np.cross(
            triangles[:, 1] - triangles[:, 0],
            triangles[:, 2] - triangles[:, 0],
        )
        normal_lengths = np.linalg.norm(normals, axis=1)
        normals /= np.maximum(normal_lengths[:, None], 1.0e-12)
        light_direction = np.asarray([0.35, -0.45, 0.82])
        light_direction /= np.linalg.norm(light_direction)
        illumination = 0.38 + 0.62 * np.abs(normals @ light_direction)
        base_color = np.asarray(to_rgb(color))
        face_colors = np.clip(illumination[:, None] * base_color, 0.0, 1.0)
        collection3d = Poly3DCollection(
            triangles[::stride],
            facecolor=face_colors[::stride],
            edgecolor="none",
            linewidth=0.0,
            alpha=1.0,
        )
        collection3d.set_rasterized(True)
        axis.add_collection3d(collection3d)
    axis.scatter([0], [0], [0.109], color="#dc2626", s=34, label="RPLIDAR A1M8")
    axis.scatter([0], [0], [0.030], color="#16a34a", s=24, label="BNO055")
    axis.set_xlim(-0.25, 0.25)
    axis.set_ylim(-0.22, 0.22)
    axis.set_zlim(0.0, 0.18)
    axis.set_box_aspect((0.5, 0.44, 0.18))
    axis.view_init(elev=27, azim=-55)
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_zlabel("z (m)")
    axis.set_title("Mô hình CAD 3D dùng trực tiếp trong Gazebo\n440 × 340 mm, bánh Ø85 mm, khối lượng 5,0 kg")
    axis.legend(frameon=False, loc="upper left")
    figure.tight_layout()
    figure.savefig(ASSETS / "figure_07_robot_3d.png", bbox_inches="tight")
    plt.close(figure)


def copy_gui_evidence():
    candidates = {
        "figure_08_rviz_gazebo_ui.png": GUI / "rviz_research_ui_final.png",
        "figure_09_gazebo_warehouse.png": GUI / "gazebo_research_warehouse.png",
        "figure_10_rviz_all_methods.png": GUI / "rviz_scrollable_panel_smoothers.png",
    }
    for target, source in candidates.items():
        if source.exists():
            shutil.copy2(source, ASSETS / target)


def font(size, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT_REGULAR, size)


def centered(draw, box, text, selected_font, fill="#102a43"):
    left, top, right, bottom = box
    bounds = draw.multiline_textbbox(
        (0, 0), text, font=selected_font, align="center", spacing=7
    )
    width = bounds[2] - bounds[0]
    height = bounds[3] - bounds[1]
    draw.multiline_text(
        ((left + right - width) / 2, (top + bottom - height) / 2),
        text,
        font=selected_font,
        fill=fill,
        align="center",
        spacing=7,
    )


def arrow(draw, start, end, fill="#475569", width=5):
    draw.line((start, end), fill=fill, width=width)
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = max(math.hypot(dx, dy), 1.0)
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    base = (end[0] - 18 * ux, end[1] - 18 * uy)
    draw.polygon(
        (
            end,
            (base[0] + 9 * px, base[1] + 9 * py),
            (base[0] - 9 * px, base[1] - 9 * py),
        ),
        fill=fill,
    )


def save_architecture():
    image = Image.new("RGB", (1900, 940), "white")
    draw = ImageDraw.Draw(image)
    centered(
        draw,
        (0, 20, image.width, 100),
        "CHUỖI ADAPTIVE HYBRID PIVOT–G2 VÀ VÒNG PHẢN HỒI",
        font(40, True),
    )
    nodes = (
        ((60, 180, 350, 380), "5 global\nplanner", "#e0f2fe"),
        ((440, 130, 800, 430), "Điều kiện hóa\n+ tìm kiếm trim\n+ DP toàn đường", "#fae8ff"),
        ((890, 130, 1250, 430), "Cổng an toàn Hybrid\nSimple / Pivot–G2\n/ Raw fallback", "#fef3c7"),
        ((1340, 180, 1640, 380), "Profile hai chiều\nv(s), a, jerk,\nω & bánh xe", "#dcfce7"),
    )
    for box, label, fill in nodes:
        draw.rounded_rectangle(box, radius=24, fill=fill, outline="#334155", width=4)
        centered(draw, box, label, font(29, True))
    for first, second in zip(nodes, nodes[1:]):
        arrow(
            draw,
            (first[0][2] + 16, 280),
            (second[0][0] - 16, 280),
        )
    draw.rounded_rectangle(
        (320, 610, 1580, 820),
        radius=25,
        fill="#f8fafc",
        outline="#475569",
        width=4,
    )
    centered(
        draw,
        (320, 610, 1580, 820),
        "Maneuver-aware RPP + servo đích hai chiều\n"
        "chiếu tiến độ có hướng, giới hạn sai số ngang/hướng/ω\n"
        "← odom + TF/AMCL + ground truth Gazebo → telemetry & benchmark",
        font(28, True),
    )
    arrow(draw, (1490, 400), (1490, 590))
    arrow(draw, (500, 590), (500, 450))
    image.save(ASSETS / "figure_01_architecture.png")


def table(headers, rows, classes=""):
    head = "".join(f"<th>{html.escape(str(item))}</th>" for item in headers)
    body = []
    for row in rows:
        body.append(
            "<tr>"
            + "".join(f"<td>{html.escape(str(item))}</td>" for item in row)
            + "</tr>"
        )
    return (
        f'<table class="{classes}"><thead><tr>{head}</tr></thead>'
        f"<tbody>{''.join(body)}</tbody></table>"
    )


def figure(filename, caption, css_class=""):
    path = ASSETS / filename
    if not path.exists():
        return ""
    return (
        f'<figure class="{css_class}"><img src="rev_ecit_2026_assets/{filename}" '
        f'alt="{html.escape(caption)}"><figcaption>{html.escape(caption)}</figcaption>'
        "</figure>"
    )


def overall_table(overall, compact=False):
    rows = []
    for method in METHOD_ORDER:
        value = overall[(method,)]
        rows.append(
            (
                METHOD_LABEL[method],
                f'{value["successes"]}/{value["attempts"]}',
                fmt(value["energy"], 2),
                fmt(value["clearance"], 3),
                fmt(value["length"], 3),
                fmt(value["deviation"], 3),
                fmt(value["runtime_ms"], 1),
            )
        )
    headers = (
        "Phương pháp",
        "OK",
        "Eκ",
        "clear. (m)",
        "L (m)",
        "dev. (m)",
        "ms",
    )
    return table(headers, rows, "compact" if compact else "")


def paper_geometry_table(overall):
    rows = []
    for method in METHOD_ORDER:
        value = overall[(method,)]
        rows.append(
            (
                METHOD_SHORT[method],
                f'{100.0 * value["successes"] / value["attempts"]:.1f}',
                fmt(value["energy"], 2),
                fmt(100.0 * value["clearance"], 1),
                fmt(value["length"], 2),
            )
        )
    return table(
        ("PP", "OK (%)", "Eκ", "clear. (cm)", "L (m)"),
        rows,
        "compact",
    )


def execution_table(rows):
    output = []
    for row in sorted(
        rows,
        key=lambda item: (
            as_float(item.get("fixed_speed_limit_mps"), 0.0) == 0.0,
            as_float(item.get("fixed_speed_limit_mps"), 0.0),
            METHOD_ORDER.index(item["method"]),
        ),
    ):
        output.append(
            (
                speed_label(as_float(row.get("fixed_speed_limit_mps"), 0.0)),
                METHOD_LABEL.get(row["method"], row["method"]),
                "Có" if row.get("success") else "Không",
                fmt(as_float(row.get("execution_time_s")), 2),
                fmt(100.0 * as_float(row.get("tracking_rmse_m")), 2),
                fmt(100.0 * as_float(row.get("estimated_tracking_rmse_m")), 2),
                fmt(100.0 * as_float(row.get("curve_exit_tracking_rmse_m")), 2),
                fmt(as_float(row.get("actual_max_linear_mps")), 3),
                fmt(as_float(row.get("adaptive_speed_nominal_p95_abs_jerk_mps3")), 2),
                fmt(100.0 * as_float(row.get("adaptive_speed_safety_override_fraction")), 1),
            )
        )
    return table(
        (
            "Tốc độ",
            "Smoother",
            "OK",
            "t (s)",
            "RMSE GT (cm)",
            "RMSE est. (cm)",
            "exit (cm)",
            "vmax",
            "jerk P95",
            "override %",
        ),
        output,
        "tiny",
    )


def paper_execution_table(rows):
    adaptive = [
        row
        for row in rows
        if abs(as_float(row.get("fixed_speed_limit_mps"), 0.0)) < 1.0e-9
    ]
    output = []
    for row in sorted(
        adaptive, key=lambda item: METHOD_ORDER.index(item["method"])
    ):
        output.append(
            (
                METHOD_SHORT[row["method"]],
                fmt(as_float(row.get("execution_time_s")), 1),
                fmt(100.0 * as_float(row.get("tracking_rmse_m")), 2),
                fmt(100.0 * as_float(row.get("estimated_tracking_rmse_m")), 2),
                fmt(100.0 * as_float(row.get("curve_exit_tracking_rmse_m")), 2),
                fmt(as_float(row.get("actual_max_linear_mps")), 2),
            )
        )
    return table(
        ("PP", "t (s)", "GT", "est.", "exit", "vmax"),
        output,
        "compact",
    )


def map_validation_table(rows):
    output = []
    for row in sorted(rows, key=lambda item: list(ENV_LABEL).index(item["environment"])):
        output.append(
            (
                ENV_LABEL[row["environment"]],
                row["scenario"],
                "Có" if row.get("success") else "Không",
                fmt(as_float(row.get("execution_time_s")), 2),
                fmt(100.0 * as_float(row.get("tracking_rmse_m")), 2),
                fmt(100.0 * as_float(row.get("tracking_max_error_m")), 2),
                fmt(100.0 * as_float(row.get("estimated_tracking_rmse_m")), 2),
                fmt(100.0 * as_float(row.get("localization_position_error_p95_m")), 2),
                str(row.get("planned_footprint_collision_sample_count", "–")),
            )
        )
    return table(
        (
            "Map",
            "Kịch bản",
            "OK",
            "t (s)",
            "GT RMSE (cm)",
            "GT max (cm)",
            "est. RMSE (cm)",
            "loc. P95 (cm)",
            "va chạm",
        ),
        output,
        "tiny",
    )


def paper_map_validation_table(rows):
    short_environment = {
        "research_warehouse": "Kho NC",
        "narrow_aisles": "Lối hẹp",
        "office_maze": "Văn phòng",
        "open_arena": "Mở",
        "warehouse_cross_aisles": "Giao cắt",
        "warehouse_dispatch": "Điều phối",
        "warehouse_long_aisles": "Lối dài",
    }
    output = []
    for row in sorted(
        rows, key=lambda item: list(ENV_LABEL).index(item["environment"])
    ):
        output.append(
            (
                short_environment[row["environment"]],
                fmt(100.0 * as_float(row.get("tracking_rmse_m")), 2),
                fmt(
                    100.0 * as_float(row.get("estimated_tracking_rmse_m")),
                    2,
                ),
                fmt(
                    100.0
                    * as_float(row.get("localization_position_error_p95_m")),
                    2,
                ),
            )
        )
    return table(
        ("Map", "GT RMSE", "est. RMSE", "loc. P95"),
        output,
        "compact",
    )


def stratified_execution_table(rows):
    output = []
    for row in sorted(
        rows,
        key=lambda item: (
            item.get("environment", ""),
            item.get("planner", ""),
            as_float(item.get("fixed_speed_limit_mps"), 0.0) == 0.0,
            as_float(item.get("fixed_speed_limit_mps"), 0.0),
            METHOD_ORDER.index(item["method"])
            if item.get("method") in METHOD_ORDER
            else len(METHOD_ORDER),
        ),
    ):
        output.append(
            (
                ENV_LABEL.get(row.get("environment"), row.get("environment", "–")),
                row.get("scenario", "–"),
                row.get("planner", "–"),
                METHOD_SHORT.get(row.get("method"), row.get("method", "–")),
                speed_label(as_float(row.get("fixed_speed_limit_mps"), 0.0)),
                "Có" if row.get("success") else "Không",
                fmt(as_float(row.get("execution_time_s")), 2),
                fmt(100.0 * as_float(row.get("tracking_rmse_m")), 2),
                fmt(100.0 * as_float(row.get("estimated_tracking_rmse_m")), 2),
                fmt(100.0 * as_float(row.get("curve_exit_tracking_rmse_m")), 2),
                str(row.get("planned_footprint_collision_sample_count", "–")),
            )
        )
    return table(
        (
            "Map",
            "Kịch bản",
            "Planner",
            "PP",
            "Tốc độ",
            "OK",
            "t (s)",
            "GT (cm)",
            "est. (cm)",
            "exit (cm)",
            "va chạm",
        ),
        output,
        "tiny",
    )


def radius_distribution_table(rows):
    summary = selected_radius_summary(rows)
    output = []
    for environment in (*ENV_LABEL, "all"):
        if environment not in summary:
            continue
        value = summary[environment]
        output.append(
            (
                "Toàn bộ" if environment == "all" else ENV_LABEL[environment],
                str(value["count"]),
                fmt(value["minimum"], 3),
                fmt(value["median"], 3),
                fmt(value["p90"], 3),
                fmt(value["maximum"], 3),
                pct(100.0 * value["outside_legacy_fraction"], 1),
            )
        )
    return table(
        ("Map", "n", "R min", "R trung vị", "R p90", "R max", "ngoài bank"),
        output,
        "compact",
    )


def dispatch_robustness_case(rows):
    pivot = next(
        (
            row for row in rows
            if row.get("environment") == "warehouse_dispatch"
            and row.get("scenario") == "full_replenishment"
            and row.get("planner") == "ThetaStar"
            and row.get("method") == "pivot_g2"
            and math.isclose(
                as_float(row.get("fixed_speed_limit_mps")), 0.22,
                abs_tol=1.0e-12,
            )
            and not row.get("success")
        ),
        None,
    )
    hybrid = next(
        (
            row for row in rows
            if row.get("environment") == "warehouse_dispatch"
            and row.get("scenario") == "full_replenishment"
            and row.get("planner") == "ThetaStar"
            and row.get("method") == "adaptive_hybrid"
            and math.isclose(
                as_float(row.get("fixed_speed_limit_mps")), 0.22,
                abs_tol=1.0e-12,
            )
            and row.get("success")
        ),
        None,
    )
    if pivot is None or hybrid is None:
        return ""
    return f"""
<h3>9.2 Ca phản ví dụ và vai trò của cổng Hybrid</h3>
<p>Ở <code>warehouse_dispatch/full_replenishment</code>, Pivot–G2 độc lập tại
0,22 m/s có clearance kế hoạch chỉ
{100.0 * as_float(pivot.get("planned_footprint_clearance_min_m")):.2f} cm.
Sai số bám ground-truth cực đại đạt
{100.0 * as_float(pivot.get("tracking_max_error_m")):.2f} cm; RPP phát hiện
va chạm dự báo liên tiếp và hủy action với mã
{pivot.get("controller_error_code")} (<code>PATIENCE_EXCEEDED</code>), trước
khi collision monitor phải can thiệp. Đây được giữ là một failure thực nghiệm,
không retry như lỗi hạ tầng. Trên đúng planner, scenario, trần 0,22 m/s và
cùng <code>raw_path_sha256</code>, Adaptive Hybrid chọn Simple có clearance kế hoạch
{100.0 * as_float(hybrid.get("planned_footprint_clearance_min_m")):.2f} cm và
hoàn thành trong {as_float(hybrid.get("execution_time_s")):.2f} s. Ca này cho
thấy Pivot–G2 thuần tối ưu độ cong không luôn là lựa chọn chạy kín tốt nhất,
và cổng Hybrid là thành phần an toàn cần thiết của phương pháp đề xuất.</p>
"""


def paper_html(geometry, execution, validation, overall, pairing_count, paired):
    raw = overall[("raw",)]
    pivot = overall[("pivot_g2",)]
    hybrid = overall[("adaptive_hybrid",)]
    simple = overall[("simple",)]
    geometry_claim = (
        f"Pivot–G2 thích nghi giảm {abs(percent_change(pivot['energy'], raw['energy'])):.1f}% "
        f"năng lượng độ cong trung bình so với Raw; Hybrid thích nghi giảm "
        f"{abs(percent_change(hybrid['energy'], raw['energy'])):.1f}% và đạt "
        f"{hybrid['successes']}/{hybrid['attempts']} lượt thành công."
    )
    execution_success = sum(bool(row.get("success")) for row in execution)
    execution_count = len(execution)
    estimated_rmse = mean(
        as_float(row.get("estimated_tracking_rmse_m"))
        for row in execution
        if row.get("success")
    )
    ground_truth_rmse = mean(
        as_float(row.get("tracking_rmse_m"))
        for row in execution
        if row.get("success")
    )
    max_nominal_jerk = max(
        (
            as_float(row.get("adaptive_speed_nominal_p95_abs_jerk_mps3"))
            for row in execution
            if row.get("success")
        ),
        default=math.nan,
    )
    radius_summary = selected_radius_summary(geometry).get("all", {})
    body = f"""<!doctype html>
<html lang="vi"><head><meta charset="utf-8">
<title>Adaptive Hybrid Pivot–G2 - REV-ECIT 2026</title>
<style>
@page {{ size: A4; margin: 17mm 16mm 18mm 16mm; }}
body {{ font-family: "Times New Roman", serif; font-size: 9pt; line-height: 1.04;
  color: #111; margin: 0; text-align: justify; }}
.title {{ text-align:center; font-size: 15pt; font-weight: bold; line-height:1.12; }}
.authors {{ text-align:center; font-size: 10pt; margin: 4pt 0 7pt; }}
.abstract {{ margin: 0 8mm 5pt; }}
.columns {{ column-count: 2; column-gap: 7mm; }}
h2 {{ font-size: 9.7pt; text-align:center; margin: 5pt 0 2pt; break-after: avoid; }}
h3 {{ font-size: 9.2pt; margin: 4pt 0 1pt; break-after: avoid; }}
p {{ margin: 1.2pt 0; }}
.eq {{ text-align:center; font-style:italic; margin: 2pt 0; break-inside: avoid; }}
figure {{ margin: 3pt 0; break-inside: avoid; text-align:center; }}
figure img {{ width:100%; max-height: 56mm; object-fit:contain; }}
figure.compact img {{ max-height: 43mm; }}
figcaption {{ font-size: 7.5pt; line-height:1.0; text-align:center; }}
table {{ border-collapse:collapse; width:100%; margin:2pt 0; font-size:6.7pt;
  break-inside:avoid; }}
th, td {{ border:0.3pt solid #555; padding:1.0pt; text-align:center; }}
th {{ font-weight:bold; background:#eee; }}
.compact {{ font-size:6.2pt; }}
.refs {{ font-size:7.2pt; line-height:1.0; }}
.note {{ font-size:7.3pt; }}
</style></head><body>
<div class="title">ADAPTIVE HYBRID PIVOT–G2 VỚI BAO RÀNG BUỘC VẬN TỐC HAI CHIỀU
CHO ROBOT VI SAI TRONG ROS 2/NAV2</div>
<div class="authors">[TÊN TÁC GIẢ] — [ĐƠN VỊ] — [EMAIL]</div>
<div class="abstract"><b>Tóm tắt—</b> Bài báo trình bày một chuỗi hậu xử lý đường đi
cho robot vi sai gồm điều kiện hóa polyline có kiểm tra swept-footprint, chuyển tiếp
Bézier bậc năm liên tục hình học G2, tìm kiếm bán kính thích nghi, tối ưu trạng thái
góc trên toàn đường, cổng an toàn Hybrid và profile vận tốc hai chiều có giới hạn
vận tốc bánh, gia tốc ngang, gia tốc góc và jerk. Bộ điều khiển mở rộng Regulated
Pure Pursuit bằng phép chiếu tiến độ có hướng, giới hạn phục hồi bám và servo đích
hai chiều. Đánh giá hình học dùng {len(geometry):,} mẫu trên 7 map, 5 planner, 8
phương pháp và 3 lần lặp; chạy kín dùng ground truth Gazebo, odom và TF/AMCL tách
biệt. {geometry_claim} Trong ma trận chạy kín hiện có {execution_success}/{execution_count}
lượt thành công; RMSE trung bình theo ground truth là {100*ground_truth_rmse:.2f} cm,
trong khi RMSE trong hệ điều khiển là {100*estimated_rmse:.2f} cm. Kết quả cho thấy
ưu thế chính của phương pháp là giảm độ gấp khúc có kiểm chứng an toàn và giữ khả
năng hoàn thành nhờ fallback, không phải tối ưu toàn cục tuyệt đối.</div>
<div class="abstract"><b>Từ khóa—</b> robot vi sai, làm mượt đường đi, G2,
Bézier bậc năm, Nav2, profile vận tốc, Gazebo.</div>
<div class="columns">
<h2>I. GIỚI THIỆU</h2>
<p>Đường đi trên costmap thường gồm các đoạn thẳng và góc gấp. Làm mượt thuần
hình học có thể cắt vào vật cản; làm tròn mọi góc có thể tăng thời gian hoặc tạo
đoạn cong robot không bám được. Công trình Bézier bậc năm của Simba và cộng sự
chỉ ra khả năng tạo quỹ đạo C2 cho robot phi holonomic [1], trong khi Regulated
Pure Pursuit (RPP) điều chỉnh vận tốc theo độ cong, khoảng cách vật cản và đích
[2]. Tuy nhiên, trong một chuỗi Nav2 thực tế còn cần ràng buộc footprint, các
góc buộc quay tại chỗ, tiến độ không nhảy sang nhánh giao cắt, động học tăng/giảm
tốc và sai số định vị.</p>
<p>Đóng góp của nghiên cứu gồm: (i) ứng viên Pivot–G2 thích nghi được đánh giá
trong cùng cửa sổ thời gian và ghép bằng quy hoạch động; (ii) Hybrid chọn
Simple/Pivot–G2/Raw bằng cổng an toàn; (iii) bao vận tốc S-curve hai chiều và
phục hồi vòng kín; (iv) benchmark cùng raw-path hash và tách ground truth,
odometry, TF/AMCL.</p>
{figure("figure_01_architecture.png", "Hình 1. Chuỗi thuật toán và vòng phản hồi đo lường.", "compact")}

<h2>II. PHƯƠNG PHÁP</h2>
<h3>A. Chuyển tiếp Pivot–G2</h3>
<p>Với góc đổi hướng Δψ và khoảng trim d, bán kính thiết kế là
R=d/tan(|Δψ|/2). Sáu điểm điều khiển Bézier bậc năm được đặt đối xứng trên hai
tiếp tuyến; đạo hàm bậc hai ở hai đầu bằng không nên độ cong nối với đoạn thẳng
liên tục. Với r(u)=Σ Bᵢ,₅(u)Pᵢ:</p>
<div class="eq">κ(u) = (x′y″−y′x″)/(x′²+y′²)<sup>3/2</sup>, &nbsp;
E<sub>κ</sub>=∫κ²ds.</div>
<p>Ứng viên bị loại nếu đổi dấu độ cong ngoài ý muốn, bánh trong phải đảo chiều,
footprint va chạm, hoặc vi phạm giới hạn. Tìm kiếm thích nghi ưu tiên biên
safe/unsafe và lân cận nghiệm tốt nhưng vẫn lấy mẫu toàn miền. Mỗi góc gồm các
trạng thái G2 và pivot; DP O(NK²) chọn chuỗi có tổng chi phí nhỏ nhất với điều
kiện trim hai góc kề không chồng lấn. Trong ma trận, DP chọn
{int(radius_summary.get("count", 0))} chuyển tiếp;
{100.0 * radius_summary.get("outside_legacy_fraction", 0.0):.1f}% bán kính
không thuộc bank cố định legacy, xác nhận tìm kiếm không bị lượng tử hóa.</p>
<h3>B. Cổng Hybrid và profile vận tốc</h3>
<p>Hybrid lấy Simple làm mặc định; Pivot–G2 chỉ được chọn khi cải thiện cost
an toàn đủ lớn và năng lượng độ cong nằm trong ngân sách; Raw là fallback cuối.
Mỗi điểm có trần tức thời:</p>
<div class="eq">v̄(s)=min[v<sub>max</sub>, √(a<sub>y,max</sub>/|κ|),
ω<sub>max</sub>/|κ|, v<sub>w,max</sub>/(1+L|κ|/2)].</div>
<p>Hai lượt truyền tiến/lùi giải vận tốc đạt được với profile gia tốc đối xứng
giới hạn jerk. Khoảng chuyển từ v₀ đến v₁ dùng profile tam giác khi
Δv≤a²/j và hình thang trong trường hợp còn lại. Sau đó, cặp nút được giảm đồng
thời đến khi |Δ(vκ)|/Δt≤α<sub>max</sub>. Cách này khắc phục lỗi cũ chỉ phanh
nhìn về phía trước nhưng tăng tốc ngay sau cong.</p>
<h3>C. Điều khiển bám và đích</h3>
<p>Tiến độ s được chiếu trong cửa sổ cục bộ bằng
J=e<sub>xy</sub>²+(w<sub>ψ</sub>e<sub>ψ</sub>)², giới hạn lùi 3 cm và ưu tiên
nghiệm phía trước khi bằng điểm. Các hàm smoothstep giảm v theo sai số ngang,
hướng và sai số ω. Khi gần đích, vị trí mục tiêu được chốt trong odom còn yaw
được biến đổi trực tiếp theo TF mới nhất; servo có thể chạy tiến hoặc lùi rồi
mới căn yaw. Điều này loại vòng quay sai do AMCL dịch chuyển vài cm sau khi
vừa qua dung sai vị trí.</p>

<h2>III. THIẾT KẾ THỬ NGHIỆM</h2>
<p>Robot Gazebo dùng CAD 440×340 mm, hai bánh Ø85 mm, khoảng cách tâm bánh
0,2548 m, RPLIDAR A1M8 và IMU BNO055. Giới hạn chính: v≤0,30 m/s,
|ω|≤0,80 rad/s, |v<sub>w</sub>|≤0,36 m/s, a<sub>y</sub>≤0,18 m/s²,
a<sub>tăng</sub>≤0,35 m/s², a<sub>giảm</sub>≤0,45 m/s²,
j≤0,90 m/s³. Bảy môi trường gồm kho nghiên cứu,
lối hẹp, văn phòng, không gian mở và ba cấu hình kho công nghiệp.</p>
<p>Thiết kế hình học ghép cặp giữ nguyên planner output SHA-256 giữa 8 smoother:
{pairing_count} nhóm, 0 sai ghép. Có 60 tình huống, 5 planner và 3 lần lặp,
tổng {len(geometry):,} dòng. Chạy kín ghi /world/.../pose ground truth, /odom,
TF map→base_link, cmd_vel và telemetry; vì vậy sai số điều khiển không bị đánh
đồng với sai số định vị. Giới hạn tốc độ thử là 0,15; 0,22 và chế độ thích nghi
đến 0,30 m/s.</p>
{figure("figure_07_robot_3d.png", "Hình 2. Mô hình CAD 3D thực thi trong Gazebo.", "compact")}

<h2>IV. KẾT QUẢ VÀ THẢO LUẬN</h2>
{figure("figure_02_geometry_overview.png", "Hình 3. Tổng hợp đánh giá hình học trên toàn bộ ma trận.")}
<p>Raw chỉ thất bại ở 3 ca planner không sinh được đường. Pivot–G2 độc lập đạt
{pivot['successes']}/{pivot['attempts']}; Hybrid đạt {hybrid['successes']}/{hybrid['attempts']}
và không có mẫu footprint va chạm trong các đường thành công. So với Raw,
Pivot–G2 thích nghi giảm trung bình {abs(percent_change(pivot['energy'],raw['energy'])):.1f}%
Eκ, rút ngắn {abs(percent_change(pivot['length'],raw['length'])):.2f}% và tăng
clearance trung bình {percent_change(pivot['clearance'],raw['clearance']):.1f}%.
Hybrid giảm {abs(percent_change(hybrid['energy'],raw['energy'])):.1f}% Eκ,
tăng clearance {percent_change(hybrid['clearance'],raw['clearance']):.1f}%;
so với Simple, Eκ trung bình giảm {abs(percent_change(hybrid['energy'],simple['energy'])):.1f}%.
Độ lệch khỏi raw tăng là đánh đổi chủ động, nhưng mọi ứng viên đều qua
swept-footprint.</p>
{paper_geometry_table(overall)}
{figure("figure_03_map_energy_ratio.png", "Hình 4. Mức giảm Eκ theo từng map.", "compact")}
{figure("figure_04_speed_tracking.png", "Hình 5. So sánh chạy kín theo bộ làm mượt và tốc độ.")}
<p>RMSE mà controller quan sát nhỏ hơn ground truth vì phần chênh lệch còn lại
đến từ AMCL/odom. Đây là lý do không dùng riêng TF để tuyên bố chất lượng bám.
Jerk danh định P95 lớn nhất là {fmt(max_nominal_jerk,2)} m/s³; các điểm chuyển
an toàn, pivot và servo được báo riêng là safety override. Không có can thiệp
collision monitor trong các lượt thành công.</p>
{paper_execution_table(execution)}
{figure("figure_05_trace_speed.png", "Hình 6. Quỹ đạo chạy thật trong Gazebo và profile vận tốc vòng kín.")}
<p>Kiểm chứng phân tầng trên cả bảy môi trường cho Pivot–G2 thích nghi đều hoàn
thành, không có mẫu footprint va chạm. RMSE controller quan sát nằm trong khoảng
{100*min(as_float(row.get("estimated_tracking_rmse_m")) for row in validation):.2f}–{100*max(as_float(row.get("estimated_tracking_rmse_m")) for row in validation):.2f}
cm; phần sai số ground truth tăng theo drift định vị, đặc biệt trên đường dài.</p>
{figure("figure_06_all_map_closed_loop.png", "Hình 7. Phân rã sai số chạy kín trên bảy môi trường.", "compact")}
{paper_map_validation_table(validation)}

<h2>V. KẾT LUẬN</h2>
<p>Adaptive Hybrid Pivot–G2 cho lợi thế rõ nhất ở ba mặt: giảm năng lượng độ
cong mạnh so với raw, kiểm tra an toàn footprint trước khi nhận ứng viên, và
không đánh đổi độ bền nhờ fallback. Bao vận tốc hai chiều cùng chiếu tiến độ có
hướng đã xử lý hiện tượng tăng tốc lệch ray sau cong; servo đích sửa hướng quay
sai do nhiễu định vị. Kết quả chưa chứng minh tối ưu toàn cục hay khả năng chạy
trên robot vật lý; bước tiếp theo là thí nghiệm phần cứng lặp nhiều lần và kiểm
định thống kê theo quãng đường dài.</p>

<h2>TÀI LIỆU THAM KHẢO</h2>
<div class="refs">
<p>[1] K. R. Simba, N. Uchiyama, S. Sano, “Real-time smooth trajectory generation
for nonholonomic mobile robots using Bézier curves,” <i>RCIM</i>, vol. 41,
pp. 31–42, 2016, doi:10.1016/j.rcim.2016.02.002.</p>
<p>[2] S. Macenski, S. Singh, F. Martín, J. Ginés, “Regulated pure pursuit for
robot path tracking,” <i>Autonomous Robots</i>, vol. 47, pp. 685–694, 2023,
doi:10.1007/s10514-023-10097-6.</p>
<p>[3] H. Pham, Q.-C. Pham, “A new approach to time-optimal path
parameterization based on reachability analysis,” arXiv:1707.07239, 2017.</p>
<p>[4] L. Xu, M. Cao, B. Song, “A new approach to smooth path planning of mobile
robot based on quartic Bézier transition curve and improved PSO,”
<i>Neurocomputing</i>, vol. 473, pp. 98–106, 2022.</p>
<p>[5] F. Dellaert, D. Fox, W. Burgard, S. Thrun, “Monte Carlo localization for
mobile robots,” <i>Proc. ICRA</i>, pp. 1322–1328, 1999.</p>
<p>[6] Navigation2, “Regulated Pure Pursuit Controller,” tài liệu chính thức,
truy cập 25-07-2026.</p>
</div>
</div></body></html>"""
    return body


def supplement_html(
    geometry,
    execution,
    stratified_execution,
    validation,
    overall,
    map_method,
    planner_method,
    pairing_count,
    paired,
):
    by_map_sections = []
    for environment in ENV_LABEL:
        rows = []
        for method in METHOD_ORDER:
            value = map_method[(environment, method)]
            rows.append(
                (
                    METHOD_LABEL[method],
                    f'{value["successes"]}/{value["attempts"]}',
                    fmt(value["energy"], 3),
                    fmt(value["clearance"], 3),
                    fmt(value["length"], 3),
                    fmt(value["deviation"], 4),
                    fmt(value["runtime_ms"], 2),
                    value["collisions"],
                )
            )
        by_map_sections.append(
            f"<h3>{html.escape(ENV_LABEL[environment])}</h3>"
            + table(
                (
                    "Phương pháp",
                    "OK",
                    "Eκ",
                    "clear. (m)",
                    "L (m)",
                    "dev. (m)",
                    "ms",
                    "collision",
                ),
                rows,
                "tiny",
            )
        )
    planner_rows = []
    for planner in PLANNER_ORDER:
        for method in METHOD_ORDER:
            value = planner_method[(planner, method)]
            planner_rows.append(
                (
                    planner,
                    METHOD_LABEL[method],
                    f'{value["successes"]}/{value["attempts"]}',
                    fmt(value["energy"], 3),
                    fmt(value["clearance"], 3),
                    fmt(value["length"], 3),
                    fmt(value["deviation"], 4),
                    fmt(value["runtime_ms"], 2),
                )
            )
    baseline_path = AUDIT / "baseline_lower_left_pivot_g2.json"
    current_path = AUDIT / "terminal_v2" / "lower_left_diagonal_pivot_g2.json"
    baseline = json.loads(baseline_path.read_text()) if baseline_path.exists() else {}
    current = json.loads(current_path.read_text()) if current_path.exists() else {}
    audit_rows = []
    for label, key, scale, digits in (
        ("Thời gian thực thi (s)", "execution_time_s", 1.0, 3),
        ("RMSE ground truth (cm)", "tracking_rmse_m", 100.0, 3),
        ("Sai số bám cực đại (cm)", "tracking_max_error_m", 100.0, 3),
        ("Sai số vị trí cuối (cm)", "final_position_error_m", 100.0, 3),
        ("Sai số yaw cuối (rad)", "final_yaw_error_rad", 1.0, 4),
    ):
        old = scale * as_float(baseline.get(key))
        new = scale * as_float(current.get(key))
        audit_rows.append((label, fmt(old, digits), fmt(new, digits), pct(percent_change(new, old), 1)))
    pair_rows = []
    for name, value in paired.items():
        pair_rows.append(
            (
                name,
                value["n"],
                value["wins"],
                fmt(value["mean_ratio"], 3),
                fmt(value["median_ratio"], 3),
            )
        )
    return f"""<!doctype html><html lang="vi"><head><meta charset="utf-8">
<title>Phụ lục đầy đủ Adaptive Hybrid Pivot–G2</title>
<style>
@page {{ size:A4; margin:16mm; }}
body {{ font-family:"Times New Roman",serif; font-size:10pt; line-height:1.18;
 color:#111; }}
h1 {{ text-align:center; font-size:18pt; }}
h2 {{ font-size:13pt; margin-top:10pt; border-bottom:0.5pt solid #666; }}
h3 {{ font-size:11pt; margin:7pt 0 2pt; }}
p {{ margin:3pt 0; text-align:justify; }}
figure {{ text-align:center; break-inside:avoid; margin:6pt 0; }}
figure img {{ max-width:100%; max-height:125mm; object-fit:contain; }}
figcaption {{ font-size:9pt; }}
table {{ border-collapse:collapse; width:100%; margin:4pt 0; }}
th,td {{ border:0.5pt solid #555; padding:2pt; text-align:center; }}
th {{ background:#eee; }}
.tiny {{ font-size:7.6pt; }}
.compact {{ font-size:8pt; }}
code {{ font-family:"DejaVu Sans Mono",monospace; font-size:8pt; }}
.warning {{ padding:5pt; border:1pt solid #b45309; background:#fff7ed; }}
</style></head><body>
<h1>BÁO CÁO KỸ THUẬT VÀ PHỤ LỤC THỰC NGHIỆM ĐẦY ĐỦ<br>
ADAPTIVE HYBRID PIVOT–G2 — REV-ECIT 2026</h1>
<p><b>Tác giả:</b> [TÊN TÁC GIẢ] &nbsp; <b>Đơn vị:</b> [ĐƠN VỊ] &nbsp;
<b>Email:</b> [EMAIL]</p>
<p class="warning"><b>Phạm vi bằng chứng:</b> toàn bộ số liệu trong tài liệu này
được sinh từ ROS 2 Jazzy/Nav2 và Gazebo Sim 8. “Ground truth” nghĩa là pose mô
phỏng Gazebo, không phải thí nghiệm robot vật lý. Không tuyên bố tối ưu toàn cục;
“tốt hơn” luôn gắn với metric và miền thử cụ thể.</p>

<h2>1. Mục tiêu và lỗi được tái hiện</h2>
<p>Hai lỗi thực tế được tái hiện từ trace: (1) gần đích, goal checker nhả điều
kiện vị trí khi AMCL dịch khoảng vài centimet, controller quay theo bearing nhiễu
rồi quay ngược về yaw đích; (2) profile cũ chỉ truyền giới hạn phanh theo chiều
lùi, nên ngay sau đoạn cong vận tốc có thể được trả nhanh hơn khả năng động học,
tạo sai số ngang. Ngoài ra, phép chiếu điểm gần nhất thuần khoảng cách có thể
nhảy nhánh tại đường tự giao.</p>
{table(("Metric","Trước","Sau","Thay đổi"), audit_rows, "compact")}
<p>Bản sửa chốt riêng tọa độ đích trong odom nhưng biến đổi yaw theo TF mới nhất;
servo cuối hai chiều; phép chiếu có hướng, cửa sổ tiến độ và giới hạn hồi quy;
bao tăng/giảm tốc S-curve hai chiều; giới hạn theo sai số ngang, heading và
residual ω. Kết quả trên đúng kịch bản lower_left_diagonal cho thấy thời gian,
RMSE và lỗi cuối đều giảm; biến thiên AMCL được báo riêng thay vì gán cho
controller.</p>

<h2>2. Chuỗi thuật toán và công thức</h2>
{figure("figure_01_architecture.png", "Hình 1. Luồng dữ liệu và vòng phản hồi.")}
<h3>2.1 Điều kiện hóa polyline</h3>
<p>Đường planner được loại điểm trùng, đơn giản hóa kiểu Ramer–Douglas–Peucker
có biên sai lệch, nhưng mọi shortcut phải vượt qua kiểm tra swept-footprint.
Một bộ phát hiện đổi dấu góc liên tiếp triệt dao động costmap nhỏ trong một cửa
sổ hữu hạn. Mỗi thao tác đều giữ liên kết về chỉ số raw path để benchmark được.</p>
<h3>2.2 Chuyển tiếp Bézier bậc năm G2 và pivot</h3>
<p>Với r(u)=Σᵢ₌₀⁵ C(5,i)(1−u)⁵⁻ⁱuⁱPᵢ, hai cặp điểm đầu và cuối thẳng hàng với
hai cạnh và được bố trí để r″(0)=r″(1)=0. Do đó κ tại điểm nối đoạn thẳng bằng
0 và liên tục hình học bậc hai. Góc không đủ miền hình học, footprint không an
toàn, bánh trong đảo chiều hoặc profile thời gian bất khả thi trở thành pivot
quay tại chỗ có marker trùng vị trí nhưng đổi yaw.</p>
<h3>2.3 Tìm kiếm thích nghi và DP</h3>
<p>Miền d=[Rmin tan(|Δψ|/2), min(Rmax tan(|Δψ|/2),dgeo)] được lấy mẫu quyết định;
refinement ưu tiên biên feasible/infeasible, safe/unsafe rồi lân cận objective
tốt. Chi phí ổn định chuẩn hóa risk costmap, |ω|max và Eκ. DP chọn một trạng
thái cho mỗi góc với ràng buộc dᵢ+dᵢ₊₁+m≤ℓᵢ, tránh quyết định tham lam làm hai
chuyển tiếp kề chồng nhau.</p>
<h3>2.4 Profile vận tốc và vòng kín</h3>
<p>Trần cục bộ là min của vận tốc thân, gia tốc ngang, ω và vận tốc bánh. Với
Δv≤a²/j, khoảng S-curve bằng (v₀+v₁)√(Δv/j); nếu đạt a cực đại, khoảng bằng
0,5(v₀+v₁)(Δv/a+a/j). Hai phép nghịch đảo bằng chia đôi tạo bao đạt được theo
cả chiều tiến và lùi; lặp với ràng buộc |Δ(vκ)|/Δt≤αmax. Vòng kín áp smoothstep
vào sai số bám và giới hạn jerk danh định; safety override được log riêng.</p>
{figure("figure_05_trace_speed.png", "Hình 2. Một trace quỹ đạo và profile vận tốc đo trực tiếp.")}

<h2>3. Mô hình robot, bản đồ và công cụ so sánh</h2>
{figure("figure_07_robot_3d.png", "Hình 3. Mô hình CAD 3D dùng trong Gazebo.")}
{figure("figure_08_rviz_gazebo_ui.png", "Hình 4. Gazebo và RViz2 với panel chọn map/planner/smoother.")}
{figure("figure_10_rviz_all_methods.png", "Hình 5. Các phương pháp được tách thành nút và lớp hiển thị độc lập.")}
<p>Panel RViz2 có 7 nút môi trường, 5 planner, 8 chế độ thực thi và công tắc hiển
thị từng đường. Environment manager khởi động lại world/map/Nav2 trong namespace
thích hợp; benchmark ghi hash raw path để bảo đảm các smoother cùng một đầu vào.
Mô hình robot gồm chassis CAD, hai bánh chủ động, footprint 440×340 mm, lidar và
IMU; collision geometry được kiểm tra độc lập với mesh trực quan.</p>

<h2>4. Thiết kế đánh giá và tính toàn vẹn dữ liệu</h2>
<p>Ma trận hình học: 7 môi trường, 60 kịch bản, 5 planner, 8 phương pháp, 3 lần
lặp = {len(geometry):,} dòng. Có {pairing_count} nhóm planner/kịch bản/lặp và
mọi nhóm có đúng một raw_path_sha256. Ma trận này đánh giá sinh đường, footprint,
Eκ, chiều dài, độ lệch và runtime; không được gọi là chạy động lực học.</p>
<p>Ma trận chạy kín tốn thời gian vật lý nên dùng thiết kế phân tầng: đủ 8
smoother × 3 tốc độ trên một bài có nhiều đoạn cong; Pivot–G2 đại diện trên cả
7 map; và Hybrid đại diện theo planner. Mỗi trial khởi tạo Gazebo cô lập, đợi
Nav2 active, lập kế hoạch, thực thi, chờ robot ổn định rồi mới kết luận. Các
trace ground truth, odom, TF/AMCL và command được lưu cùng JSON.</p>

<h2>5. Kết quả hình học toàn cục</h2>
{figure("figure_02_geometry_overview.png", "Hình 6. Tổng hợp 7.200 phép đo.")}
{overall_table(overall)}
<p>Pivot–G2 độc lập tối ưu trực tiếp hình dạng cong nên cho Eκ thấp nhất nhưng
có thể không sinh ứng viên ở góc quá hẹp; Hybrid ưu tiên độ bền và an toàn nên
thường dùng Simple, chỉ đổi sang Pivot–G2 khi có lợi ích costmap đủ lớn. Vì vậy
không nên kết luận Hybrid luôn có Eκ thấp nhất; ưu thế của nó là trade-off an
toàn–mượt–khả dụng.</p>
{figure("figure_03_map_energy_ratio.png", "Hình 7. Giảm Eκ theo từng map.")}
{table(("So sánh cặp","n","số thắng","tỷ số TB","tỷ số trung vị"), pair_rows, "compact")}
<h3>5.1 Phân bố bán kính thích nghi được DP chọn</h3>
{radius_distribution_table(geometry)}
<p>“Ngoài bank” là tỷ lệ R không trùng bất kỳ giá trị
0,20/0,30/0,40/0,50/0,60/0,75/1,00/1,25/1,50 m trong sai số 10⁻⁶ m. Pivot
có d=0 không được trộn vào phân bố bán kính chuyển tiếp.</p>

<h2>6. Kết quả theo từng map — không lược bỏ trường hợp</h2>
{''.join(by_map_sections)}

<h2>7. Kết quả theo từng planner và smoother</h2>
{table(("Planner","Smoother","OK","Eκ","clear. (m)","L (m)","dev. (m)","ms"), planner_rows, "tiny")}

<h2>8. Chạy kín theo tốc độ và smoother</h2>
{figure("figure_04_speed_tracking.png", "Hình 8. Thời gian và RMSE theo tốc độ.")}
{execution_table(execution)}
<p>RMSE ground truth gồm cả sai số robot so với map do localizer/odometry; RMSE
estimated là sai số đường mà controller nhận thấy. Khi hai giá trị chênh nhau,
không được “tuning controller” để che lỗi định vị. Jerk danh định dùng telemetry
trước safety override; finite-difference jerk của /cmd_vel có xung ở chuyển
trạng thái và được báo riêng.</p>

<h2>9. Chạy kín đại diện trên cả 7 map</h2>
{figure("figure_06_all_map_closed_loop.png", "Hình 9. Phân rã sai số chạy kín trên 7 map.")}
{map_validation_table(validation)}
{figure("figure_11_all_map_paths.png", "Hình 10. Đường được chọn và trace ground-truth Gazebo trên toàn bộ 7 map.")}
<h3>9.1 Ma trận phân tầng bổ sung theo map, planner và tốc độ</h3>
{stratified_execution_table(stratified_execution)}
{dispatch_robustness_case(stratified_execution)}

<h2>10. Điểm hơn, đánh đổi và giới hạn</h2>
<ul>
<li><b>So với Raw:</b> giảm Eκ và chiều dài trung bình, tăng khoảng hở; đổi lại
runtime hậu xử lý và độ lệch hình học tăng nhưng bị chặn bởi swept-footprint.</li>
<li><b>So với Simple:</b> Hybrid giữ đường Simple ở đa số ca, chỉ dùng Pivot–G2
khi lợi ích an toàn đủ lớn; vì thế giảm Eκ trung bình mà vẫn có fallback.</li>
<li><b>So với Savitzky–Golay/Constrained:</b> Pivot–G2 có điều kiện nối rõ ràng,
marker pivot và profile động học liên kết trực tiếp; các bộ kia là baseline
hình học, không mã hóa quyết định quay tại chỗ của robot vi sai.</li>
<li><b>Không phải bằng chứng tối ưu toàn cục:</b> adaptive search là tìm kiếm
hữu hạn đa tiêu chí, map mô phỏng không bao quát mọi nhiễu và chưa có robot vật
lý. Cần ít nhất 20–30 lần lặp/trường hợp quan trọng và kiểm định bootstrap hoặc
Wilcoxon trước camera-ready.</li>
</ul>

<h2>11. Tái lập</h2>
<p><code>colcon build --base-paths src --symlink-install</code><br>
<code>colcon test --base-paths src --event-handlers console_direct+</code><br>
<code>ros2 launch vacuum_robot_gazebo switchable_simulation.launch.py gui:=true</code><br>
<code>ros2 run adaptive_pivot_g2_benchmark execution_matrix -- --help</code></p>
<p>Dữ liệu nguồn: <code>results/conference_geometry_20260725/</code>,
<code>results/conference_execution_20260725/</code> và
<code>results/closed_loop_audit_20260725/</code>. Script hiện tại tái sinh toàn
bộ bảng và hình, tránh sao chép số liệu bằng tay.</p>

<h2>12. Tài liệu tham khảo</h2>
<ol>
<li>K. R. Simba, N. Uchiyama, S. Sano, “Real-time smooth trajectory generation
for nonholonomic mobile robots using Bézier curves,” RCIM 41, 31–42, 2016.</li>
<li>S. Macenski et al., “Regulated pure pursuit for robot path tracking,”
Autonomous Robots 47, 685–694, 2023.</li>
<li>H. Pham, Q.-C. Pham, “A new approach to time-optimal path parameterization
based on reachability analysis,” arXiv:1707.07239, 2017.</li>
<li>L. Xu, M. Cao, B. Song, “A new approach to smooth path planning ... quartic
Bézier ... PSO,” Neurocomputing 473, 98–106, 2022.</li>
<li>F. Dellaert et al., “Monte Carlo localization for mobile robots,” ICRA,
1322–1328, 1999.</li>
</ol>
</body></html>"""


def write_summary(
    geometry,
    execution,
    all_execution,
    validation,
    overall,
    map_method,
    planner_method,
    pairing_count,
    pairing_bad,
    paired,
):
    summary = {
        "geometry_row_count": len(geometry),
        "geometry_pairing_group_count": pairing_count,
        "geometry_pairing_bad_groups": [list(key) for key in pairing_bad],
        "geometry_overall": {
            method: overall[(method,)] for method in METHOD_ORDER
        },
        "geometry_by_map_method": {
            f"{environment}/{method}": map_method[(environment, method)]
            for environment in ENV_LABEL
            for method in METHOD_ORDER
        },
        "geometry_by_planner_method": {
            f"{planner}/{method}": planner_method[(planner, method)]
            for planner in PLANNER_ORDER
            for method in METHOD_ORDER
        },
        "paired_comparisons": paired,
        "selected_radius_distribution": selected_radius_summary(geometry),
        "execution_trial_count": len(execution),
        "execution_success_count": sum(bool(row.get("success")) for row in execution),
        "all_execution_trial_count": len(all_execution),
        "all_execution_success_count": sum(
            bool(row.get("success")) for row in all_execution
        ),
        "map_validation_count": len(validation),
        "map_validation_success_count": sum(bool(row.get("success")) for row in validation),
    }
    (ASSETS / "report_data_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def write_compact_execution_csv(rows):
    fields = (
        "environment",
        "scenario",
        "configuration_sha256",
        "planner",
        "method",
        "fixed_speed_limit_mps",
        "success",
        "execution_time_s",
        "tracking_rmse_m",
        "tracking_max_error_m",
        "curve_tracking_rmse_m",
        "curve_exit_tracking_rmse_m",
        "estimated_tracking_rmse_m",
        "estimated_tracking_max_error_m",
        "final_position_error_m",
        "final_yaw_error_rad",
        "localization_position_error_p95_m",
        "actual_max_linear_mps",
        "actual_max_angular_radps",
        "actual_max_wheel_linear_mps",
        "adaptive_speed_nominal_p95_abs_jerk_mps3",
        "adaptive_speed_safety_override_fraction",
        "planned_footprint_collision_sample_count",
        "collision_monitor_interventions",
        "raw_path_sha256",
        "selected_path_sha256",
    )
    output = EXECUTION / "conference_execution_compact.csv"
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fields, lineterminator="\n"
        )
        writer.writeheader()
        for row in sorted(
            rows,
            key=lambda item: (
                item.get("environment", ""),
                item.get("scenario", ""),
                item.get("planner", ""),
                item.get("method", ""),
                as_float(item.get("fixed_speed_limit_mps"), 0.0),
            ),
        ):
            writer.writerow({field: row.get(field, "") for field in fields})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--require-complete-execution",
        action="store_true",
        help="Require all 8 methods at all three speed settings.",
    )
    args = parser.parse_args()
    ASSETS.mkdir(parents=True, exist_ok=True)
    geometry = load_geometry()
    all_execution = load_execution()
    execution = primary_execution_matrix(all_execution)
    validation = load_map_validation()
    if args.require_complete_execution:
        combinations = {
            (row["method"], as_float(row.get("fixed_speed_limit_mps"), 0.0))
            for row in execution
        }
        missing = {
            (method, speed)
            for method in METHOD_ORDER
            for speed in (0.15, 0.22, 0.0)
        } - combinations
        if missing:
            raise RuntimeError(f"Execution matrix is incomplete: {sorted(missing)}")

    overall = aggregate_geometry(geometry, ("method",))
    map_method = aggregate_geometry(geometry, ("environment", "method"))
    planner_method = aggregate_geometry(geometry, ("planner", "method"))
    pairing_count, pairing_bad = validate_pairing(geometry)
    if pairing_bad:
        raise RuntimeError(f"Raw path pairing failed for {len(pairing_bad)} groups")
    paired = {
        "Hybrid thích nghi / Raw — Eκ": paired_geometry(
            geometry,
            "adaptive_hybrid",
            "raw",
            "translation_curvature_energy_1pm",
        ),
        "Pivot–G2 thích nghi / Raw — Eκ": paired_geometry(
            geometry,
            "pivot_g2",
            "raw",
            "translation_curvature_energy_1pm",
        ),
        "Hybrid thích nghi / Simple — Eκ": paired_geometry(
            geometry,
            "adaptive_hybrid",
            "simple",
            "translation_curvature_energy_1pm",
        ),
        "Hybrid thích nghi / Raw — clearance": paired_geometry(
            geometry,
            "adaptive_hybrid",
            "raw",
            "footprint_clearance_min_m",
            higher_better=True,
        ),
    }

    set_plot_style()
    save_architecture()
    save_geometry_summary(overall)
    save_map_ratios(map_method)
    save_execution_summary(execution)
    save_path_tracking(execution)
    save_map_validation(validation)
    save_all_map_path_overlays(all_execution)
    save_robot_render()
    copy_gui_evidence()
    write_summary(
        geometry,
        execution,
        all_execution,
        validation,
        overall,
        map_method,
        planner_method,
        pairing_count,
        pairing_bad,
        paired,
    )
    write_compact_execution_csv(all_execution)
    (DOCS / "REV_ECIT_2026_ADAPTIVE_HYBRID_PIVOT_G2_PAPER.html").write_text(
        paper_html(
            geometry,
            execution,
            validation,
            overall,
            pairing_count,
            paired,
        ),
        encoding="utf-8",
    )
    (DOCS / "REV_ECIT_2026_ADAPTIVE_HYBRID_PIVOT_G2_SUPPLEMENT.html").write_text(
        supplement_html(
            geometry,
            execution,
            [
                row for row in all_execution
                if row not in execution
            ],
            validation,
            overall,
            map_method,
            planner_method,
            pairing_count,
            paired,
        ),
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "geometry_rows": len(geometry),
                "execution_trials": len(execution),
                "validation_trials": len(validation),
                "assets": str(ASSETS),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
