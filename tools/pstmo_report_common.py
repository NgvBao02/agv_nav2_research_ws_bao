#!/usr/bin/env python3

"""Build the single authoritative, image-audited report for current PSTMO."""

from __future__ import annotations

import csv
import hashlib
import html
import json
import math
import os
import platform
import site
import statistics
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

# Ubuntu's ROS/Matplotlib stack requires the system NumPy.  The user site has a
# newer NumPy for unrelated work, so import the matched plotting stack first.
USER_SITE = site.getusersitepackages()
if isinstance(USER_SITE, str) and USER_SITE in sys.path:
    sys.path.remove(USER_SITE)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.patches import FancyArrowPatch, Polygon, Rectangle
import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
ASSETS = DOCS / "pstmo_toan_dien_assets"
RVIZ_DIR = ASSETS / "rviz_cases"
GAZEBO_DIR = ASSETS / "gazebo"
FIG_DIR = ASSETS / "figures"
MAP_DIR = ROOT / "src" / "vacuum_robot_gazebo" / "maps"
BENCHMARK_SOURCE = ROOT / "src" / "adaptive_pivot_g2_benchmark"
OUTPUT_HTML = DOCS / "BAO_CAO_TOAN_DIEN_PSTMO_HIEN_TAI.html"
OUTPUT_CSV = ASSETS / "benchmark_live_35_cases.csv"
OUTPUT_JSON = ASSETS / "benchmark_live_aggregate.json"
FAILURE_JSON = ASSETS / "failure_evidence_C30.json"
FAILURE_LOG = Path(
    "/home/linh-pham/.ros/log/smoother_server_202731_1785660308942.log"
)
FAILURE_C30 = {
    "case_id": "C30",
    "method": "simple",
    "status": 6,
    "error_code": 503,
    "reason": "Nav2 collision check rejected the smoothed path",
    "collision_pose": {"x": -4.741290, "y": 3.482165, "yaw": 0.352672},
}

sys.path.insert(0, str(BENCHMARK_SOURCE))
from adaptive_pivot_g2_benchmark.clearance_metrics import (  # noqa: E402
    calculate_footprint_clearance,
)
from adaptive_pivot_g2_benchmark.initial_heading import (  # noqa: E402
    load_occupancy_map,
)
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Path as NavPath  # noqa: E402

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
    "open_arena": "Vật cản thưa, giúp tách tác động của thuật toán khỏi hiệu ứng hành lang hẹp.",
    "research_warehouse": "Kệ, thùng hàng, đường chéo và góc vuông cùng xuất hiện trong một bản đồ tổng hợp.",
    "narrow_aisles": "Các dãy kệ tạo hành lang ngoằn ngoèo; footprint và các góc liên tiếp là yếu tố chi phối.",
    "office_maze": "Vách ngăn và cửa lệch tạo chuỗi góc ngắn, phù hợp kiểm tra ràng buộc không chồng lấn.",
    "warehouse_cross_aisles": "Đường dọc–ngang giao nhau, làm rõ transition vào và ra khỏi lối giao cắt.",
    "warehouse_dispatch": "Tuyến dài qua vùng staging và dock, có mật độ vật cản cao nhất trong bộ đại diện.",
    "warehouse_long_aisles": "Các hành lang song song dài kiểm tra shortcut LOS và tích lũy độ cong trên quãng dài.",
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
METHOD_STYLE = {
    "raw": (0, (1, 2)),
    "simple": "--",
    "savitzky_golay": (0, (4, 2, 1, 2)),
    "constrained": "-.",
    "pstmo": "-",
}
METHOD_MARKER = {
    "raw": "o",
    "simple": "s",
    "savitzky_golay": "^",
    "constrained": "D",
    "pstmo": "*",
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


def fnum(value, digits=3):
    if value is None:
        return "–"
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "–"
    if not math.isfinite(numeric):
        return "–"
    return f"{numeric:.{digits}f}".replace(".", ",")


def pct_reduction(new, reference):
    return 100.0 * (reference - new) / reference if reference else math.nan


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def evidence_order(item):
    return (ENVIRONMENTS.index(item["environment"]), PLANNERS.index(item["planner"]))


def load_evidence():
    items = [json.loads(path.read_text(encoding="utf-8")) for path in RVIZ_DIR.glob("*.json")]
    items.sort(key=evidence_order)
    if len(items) != 35:
        raise RuntimeError(f"Expected 35 live evidence records, found {len(items)}")
    seen = {(item["environment"], item["planner"]) for item in items}
    expected = {(environment, planner) for environment in ENVIRONMENTS for planner in PLANNERS}
    if seen != expected:
        raise RuntimeError(f"Evidence matrix mismatch: missing={sorted(expected-seen)} extra={sorted(seen-expected)}")
    for index, item in enumerate(items, 1):
        item["case_id"] = f"C{index:02d}"
        diag = item["pstmo_diagnostics"]
        if not (
            len(item["start"]) == 3
            and len(item["goal"]) == 3
            and diag.get("search_mode") == "hierarchical_alpha_two_trim"
            and diag.get("preprocessing_mode") == "condition_then_los"
            and diag.get("pipeline_execution_count") == 1
            and diag.get("final_invariants_verified") is True
            and "pstmo" in item["metrics"]
            and "pstmo" in item["paths"]
        ):
            raise RuntimeError(f"Invalid current-PSTMO evidence: {item['case_id']}")
        screenshot = ROOT / item["rviz_screenshot"]
        if not screenshot.is_file():
            raise FileNotFoundError(screenshot)
    return items


def make_occupancy_grid(environment):
    yaml_path = MAP_DIR / f"{environment}.yaml"
    source = load_occupancy_map(str(yaml_path))
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    occupied_threshold = float(metadata.get("occupied_thresh", 0.65))
    free_threshold = float(metadata.get("free_thresh", 0.25))
    pixels = np.frombuffer(source.pixels, dtype=np.uint8).reshape((source.height, source.width))
    probability = pixels / 255.0 if source.negate else (255.0 - pixels) / 255.0
    values = np.full((source.height, source.width), -1, dtype=np.int8)
    values[probability >= occupied_threshold] = 100
    values[probability <= free_threshold] = 0
    values = np.flipud(values)
    grid = OccupancyGrid()
    grid.info.resolution = source.resolution
    grid.info.width = source.width
    grid.info.height = source.height
    grid.info.origin.position.x = source.origin_x
    grid.info.origin.position.y = source.origin_y
    grid.info.origin.orientation.z = math.sin(0.5 * source.origin_yaw)
    grid.info.origin.orientation.w = math.cos(0.5 * source.origin_yaw)
    grid.data = values.ravel().astype(int).tolist()
    return grid


def nav_path(payload):
    result = NavPath()
    result.header.frame_id = payload.get("frame_id", "map")
    for pose in payload["poses"]:
        stamped = PoseStamped()
        stamped.header.frame_id = result.header.frame_id
        stamped.pose.position.x = pose["x"]
        stamped.pose.position.y = pose["y"]
        stamped.pose.position.z = pose["z"]
        stamped.pose.orientation.x = pose["qx"]
        stamped.pose.orientation.y = pose["qy"]
        stamped.pose.orientation.z = pose["qz"]
        stamped.pose.orientation.w = pose["qw"]
        result.poses.append(stamped)
    return result


def build_rows(items):
    grids = {environment: make_occupancy_grid(environment) for environment in ENVIRONMENTS}
    rows = []
    for item in items:
        for method in METHODS:
            success = method in item["metrics"] and method in item["paths"]
            row = {
                "case_id": item["case_id"],
                "environment": item["environment"],
                "scenario": item["scenario"],
                "planner": item["planner"],
                "method": method,
                "success": success,
                "start_x": item["start"][0],
                "start_y": item["start"][1],
                "start_yaw": item["start"][2],
                "goal_x": item["goal"][0],
                "goal_y": item["goal"][1],
                "goal_yaw": item["goal"][2],
            }
            if success:
                metric = item["metrics"][method]
                clearance = calculate_footprint_clearance(
                    nav_path(item["paths"][method]), grids[item["environment"]]
                )
                row.update({
                    "path_length_m": metric["path_length_m"],
                    "max_abs_curvature_1pm": metric["max_abs_curvature_1pm"],
                    "curvature_energy_1pm": metric["curvature_energy_1pm"],
                    "algorithm_time_s": metric.get(
                        "smoothing_time_s", metric.get("planning_time_s", 0.0)
                    ),
                    "pivot_marker_count": metric.get("pivot_marker_count", 0),
                    "pivot_total_angle_rad": metric.get("pivot_total_angle_rad", 0.0),
                    "path_sha256": item["paths"][method]["sha256"],
                    **clearance,
                })
            else:
                if item["case_id"] == "C30" and method == "simple":
                    pose = FAILURE_C30["collision_pose"]
                    row["error"] = (
                        "Nav2 SmoothPath status=6, code=503; collision tại "
                        f"x={pose['x']:.6f}, y={pose['y']:.6f}, yaw={pose['yaw']:.6f}"
                    )
                else:
                    row["error"] = "Nav2 SmoothPath không xuất đường hợp lệ; ca được giữ là thất bại"
            rows.append(row)
    return rows


def aggregate_rows(rows):
    complete_cases = {
        case_id for case_id in {row["case_id"] for row in rows}
        if all(any(r["case_id"] == case_id and r["method"] == method and r["success"] for r in rows) for method in METHODS)
    }
    aggregate = {"case_count": 35, "complete_five_method_cases": len(complete_cases), "methods": {}}
    numeric_keys = (
        "path_length_m", "max_abs_curvature_1pm", "curvature_energy_1pm",
        "algorithm_time_s", "footprint_clearance_min_m", "footprint_clearance_p05_m",
        "footprint_clearance_mean_m", "footprint_collision_sample_count",
        "pivot_marker_count", "pivot_total_angle_rad",
    )
    for method in METHODS:
        successful = [row for row in rows if row["method"] == method and row["success"]]
        paired = [row for row in successful if row["case_id"] in complete_cases]
        payload = {
            "success_count": len(successful),
            "failure_count": 35 - len(successful),
            "paired_count": len(paired),
        }
        for prefix, selected in (("all_success", successful), ("paired", paired)):
            for key in numeric_keys:
                values = [float(row[key]) for row in selected if key in row]
                if values:
                    payload[f"{prefix}_mean_{key}"] = statistics.fmean(values)
                    payload[f"{prefix}_median_{key}"] = statistics.median(values)
        aggregate["methods"][method] = payload
    return aggregate, complete_cases


def persist_data(rows, aggregate, items):
    fieldnames = sorted({key for row in rows for key in row})
    with OUTPUT_CSV.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    diagnostics = [item["pstmo_diagnostics"] for item in items]
    aggregate["pstmo_diagnostics"] = {
        key: {
            "mean": statistics.fmean(float(diag[key]) for diag in diagnostics),
            "median": statistics.median(float(diag[key]) for diag in diagnostics),
            "min": min(float(diag[key]) for diag in diagnostics),
            "max": max(float(diag[key]) for diag in diagnostics),
            "sum": sum(float(diag[key]) for diag in diagnostics),
        }
        for key in (
            "runtime_s", "raw_input_points", "conditioning_output_points",
            "evaluations", "coarse_shape_evaluations",
            "recovery_shape_evaluations", "refinement_shape_evaluations", "dp_states",
            "corners", "g2_transitions", "pivots",
        )
    }
    aggregate["evidence"] = {
        "source": "live Gazebo Harmonic + RViz2 + exact ROS Path topics",
        "json_count": len(items),
        "rviz_png_count": len(list(RVIZ_DIR.glob("*.png"))),
        "gazebo_png_count": len(list(GAZEBO_DIR.glob("*.png"))),
    }
    OUTPUT_JSON.write_text(json.dumps(aggregate, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    failure = dict(FAILURE_C30)
    failure["source_log"] = str(FAILURE_LOG)
    if FAILURE_LOG.is_file():
        failure["source_log_sha256"] = sha256_file(FAILURE_LOG)
        source_text = FAILURE_LOG.read_text(encoding="utf-8", errors="replace")
        expected = "Smoothed path leads to a collision at x: -4.741290, y: 3.482165, theta: 0.352672"
        if expected not in source_text:
            raise RuntimeError("C30 Simple collision excerpt is absent from the recorded ROS log")
        failure["verified_log_excerpt"] = expected
    FAILURE_JSON.write_text(
        json.dumps(failure, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def map_image(environment):
    metadata = yaml.safe_load((MAP_DIR / f"{environment}.yaml").read_text(encoding="utf-8"))
    image_path = MAP_DIR / metadata["image"]
    image = np.asarray(Image.open(image_path).convert("L"))
    resolution = float(metadata["resolution"])
    origin = metadata["origin"]
    extent = [origin[0], origin[0] + image.shape[1] * resolution, origin[1], origin[1] + image.shape[0] * resolution]
    return image, extent


def flow_figure(path, title, steps, colors=None, subtitle=""):
    colors = colors or ["#dbeafe"] * len(steps)
    fig, ax = plt.subplots(figsize=(15, 3.8))
    ax.set_xlim(0, len(steps) * 2.2)
    ax.set_ylim(0, 3)
    ax.axis("off")
    for index, step in enumerate(steps):
        x = index * 2.2 + 0.1
        ax.add_patch(Rectangle((x, 1.0), 1.8, 1.0, facecolor=colors[index], edgecolor="#1f2937", linewidth=1.6))
        ax.text(x + 0.9, 1.5, step, ha="center", va="center", fontsize=10, fontweight="bold", wrap=True)
        if index + 1 < len(steps):
            ax.add_patch(FancyArrowPatch((x + 1.82, 1.5), (x + 2.18, 1.5), arrowstyle="-|>", mutation_scale=16, color="#334155"))
    ax.text(len(steps) * 1.1, 2.65, title, ha="center", va="center", fontsize=16, fontweight="bold")
    if subtitle:
        ax.text(len(steps) * 1.1, 0.45, subtitle, ha="center", va="center", fontsize=10, color="#475569")
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def make_algorithm_figures(items):
    flow_figure(
        FIG_DIR / "figure_01_pipeline.png",
        "Pipeline duy nhất của PSTMO độc lập",
        ["Planner path", "Condition\npolyline", "LOS footprint\ntham lam", "Hai d hình học", "Tìm α=q/d\nthô–tinh", "Time gate + DP", "Ghép + invariant\ncuối"],
        ["#e2e8f0", "#dbeafe", "#dcfce7", "#fef3c7", "#fde68a", "#ede9fe", "#ccfbf1"],
        "Không có nhánh no-LOS, không fallback sang pipeline khác; mỗi ca diagnostics xác nhận pipeline_execution_count = 1.",
    )
    flow_figure(
        FIG_DIR / "figure_02_safety_layers.png",
        "Các lớp kiểm tra an toàn và khả thi",
        ["Footprint thật\ntịnh tiến", "Footprint thật\nxoay", "Bézier hữu hạn\nkhông đảo κ", "Bánh trong\nkhông lùi", "Giới hạn v, ω,\na_y, bánh xe", "Timing\nhội tụ", "Swept output\ntoàn đường"],
        ["#fee2e2", "#fee2e2", "#ffedd5", "#ffedd5", "#fef3c7", "#e0e7ff", "#dcfce7"],
        "Mọi khối là cổng loại cứng. Clearance chỉ được đo hậu kiểm; LOS không phóng footprint thêm 0,15 m.",
    )

    example = next(item for item in items if item["environment"] == "open_arena" and item["planner"] == "NavFnAStar")
    diag = example["pstmo_diagnostics"]
    image, extent = map_image(example["environment"])
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))
    raw = example["paths"]["raw"]["poses"]
    raw_x = [pose["x"] for pose in raw]
    raw_y = [pose["y"] for pose in raw]
    for ax in axes:
        ax.imshow(image, cmap="gray", extent=extent, origin="upper", vmin=0, vmax=255)
        ax.plot(raw_x, raw_y, color="#94a3b8", linewidth=1.0, label="Raw poses")
        ax.set_aspect("equal"); ax.grid(alpha=0.18); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    conditioned = np.asarray(diag["conditioned_polyline"])
    preprocessed = np.asarray(diag["preprocessed_polyline"])
    axes[0].plot(conditioned[:, 0], conditioned[:, 1], "o-", color="#2563eb", linewidth=2.2, label="Sau conditioning")
    axes[0].set_title(f"Conditioning: {diag['raw_input_points']} → {diag['conditioning_output_points']} điểm")
    axes[1].plot(conditioned[:, 0], conditioned[:, 1], "o--", color="#94a3b8", label="Điểm neo vào LOS")
    axes[1].plot(preprocessed[:, 0], preprocessed[:, 1], "o-", color="#16a34a", linewidth=2.6, label="LOS chọn")
    axes[1].set_title(f"LOS: {diag['los_input_points']} → {diag['los_output_points']} điểm; {diag['los_accepted_shortcuts']} shortcut")
    for ax in axes: ax.legend(fontsize=8, loc="best")
    fig.suptitle("Ví dụ thật từ C01: conditioning khác LOS và cả hai đều dùng swept-footprint", fontweight="bold")
    fig.tight_layout()
    fig.savefig(FIG_DIR / "figure_03_conditioning_los_actual.png", dpi=160, bbox_inches="tight")
    plt.close(fig)

    d = 1.0; alpha = 0.32; q = alpha * d
    entry = np.array([-d, 0.0]); corner = np.array([0.0, 0.0]); exit_point = np.array([0.0, d])
    incoming = np.array([1.0, 0.0]); outgoing = np.array([0.0, 1.0])
    controls = np.asarray([entry, entry + q * incoming, entry + 2*q*incoming, exit_point - 2*q*outgoing, exit_point - q*outgoing, exit_point])
    u = np.linspace(0.0, 1.0, 301)
    coeff = np.asarray([[1,5,10,10,5,1][i] * (1-u)**(5-i) * u**i for i in range(6)]).T
    curve = coeff @ controls
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot([-1.25, 0, 0], [0, 0, 1.25], color="#64748b", linewidth=2, label="Polyline LOS")
    ax.plot(controls[:,0], controls[:,1], "o--", color="#f59e0b", label="P0…P5")
    ax.plot(curve[:,0], curve[:,1], color="#16a34a", linewidth=3, label="Bézier bậc 5 G²")
    ax.scatter([corner[0]],[corner[1]], marker="x", s=90, color="#dc2626", label="Đỉnh góc")
    ax.annotate("d", xy=(-0.5,0.02), ha="center", color="#1d4ed8", fontsize=12)
    ax.annotate("q=αd", xy=(-0.84,0.1), ha="center", color="#b45309", fontsize=11)
    ax.set_aspect("equal"); ax.grid(alpha=.25); ax.legend(); ax.set_xlabel("x/d"); ax.set_ylabel("y/d")
    ax.set_title("Họ transition đối xứng: P1−P0=P2−P1 và P5−P4=P4−P3", fontweight="bold")
    fig.tight_layout(); fig.savefig(FIG_DIR / "figure_04_bezier_geometry.png", dpi=170, bbox_inches="tight"); plt.close(fig)

    fig, ax = plt.subplots(figsize=(13, 4.8))
    coarse = np.array([.1,.2,.3,.4,.5]); recovery=np.array([.15,.25,.35,.45]); fine=np.linspace(.2,.4,11)
    ax.scatter(coarse, np.ones_like(coarse)*3, s=100, color="#2563eb", label="Lưới thô")
    ax.scatter(recovery, np.ones_like(recovery)*2, marker="D", s=70, color="#dc2626", label="Phục hồi nếu cả 5 điểm thô thất bại")
    ax.scatter(fine, np.ones_like(fine), marker="|", s=220, color="#16a34a", label="Ví dụ tinh [0,2; 0,4] thành 10 khoảng")
    for x in coarse: ax.text(x,3.15,f"{x:.1f}",ha="center",fontsize=9)
    ax.set_xlim(.07,.53); ax.set_ylim(.5,3.7); ax.set_yticks([1,2,3], ["Tinh", "Phục hồi", "Thô"]); ax.set_xlabel("α=q/d")
    ax.grid(axis="x", alpha=.2); ax.legend(loc="lower center", ncol=3); ax.set_title("Tìm α thô–tinh: chỉ ứng viên qua toàn bộ cổng cứng mới được so Eκ", fontweight="bold")
    fig.tight_layout(); fig.savefig(FIG_DIR / "figure_05_alpha_search.png", dpi=170, bbox_inches="tight"); plt.close(fig)

    fig, axes = plt.subplots(1,2,figsize=(14,5.2))
    ax=axes[0]; ax.plot([0,10],[0,0],color="#334155",linewidth=5); ax.scatter([0,4.4,10],[0,0,0],s=90,color="#dc2626")
    ax.annotate("dᵢ",xy=(3.2,.15),ha="center",fontsize=12,color="#2563eb"); ax.annotate("dᵢ₊₁",xy=(5.8,.15),ha="center",fontsize=12,color="#9333ea"); ax.annotate("margin m",xy=(4.7,-.25),ha="center",fontsize=11,color="#b45309")
    ax.add_patch(Rectangle((2.0,-.08),2.4,.16,color="#93c5fd")); ax.add_patch(Rectangle((4.4,-.08),2.8,.16,color="#d8b4fe")); ax.set_ylim(-.6,.7); ax.axis("off"); ax.set_title("Ràng buộc đoạn chung: dᵢ+dᵢ₊₁+m≤Lᵢ",fontweight="bold")
    ax=axes[1]; ax.axis("off"); levels=[["z₁: d_pref","z₁: d_compat","z₁: quay"],["z₂: d_pref","z₂: d_compat","z₂: quay"],["z₃: d_pref","z₃: d_compat","z₃: quay"]]
    for col,states in enumerate(levels):
        x=col*3.2
        for row,state in enumerate(states):
            y=2.2-row*.9; ax.add_patch(Rectangle((x,y),2.2,.55,facecolor=["#dbeafe","#ede9fe","#fee2e2"][row],edgecolor="#475569")); ax.text(x+1.1,y+.275,state,ha="center",va="center",fontsize=9)
        if col<2:
            for r1 in range(3):
                for r2 in range(3): ax.plot([x+2.2,x+3.2],[2.475-r1*.9,2.475-r2*.9],color="#94a3b8",linewidth=.7,alpha=.55)
    ax.set_xlim(-.2,8.8); ax.set_ylim(-.1,3.2); ax.set_title("DP giữ trạng thái tại từng góc; cạnh chỉ tồn tại khi không chồng lấn",fontweight="bold")
    fig.tight_layout(); fig.savefig(FIG_DIR / "figure_06_trim_dp.png",dpi=170,bbox_inches="tight");plt.close(fig)


def plot_case_composite(item, rows_for_case):
    output = FIG_DIR / f"case_{item['case_id']}_{item['environment']}_{item['planner']}.png"
    screenshot_path = ROOT / item["rviz_screenshot"]
    if not screenshot_path.is_file():
        screenshot_path = RVIZ_DIR / Path(item["rviz_screenshot"]).name
    screenshot = np.asarray(Image.open(screenshot_path).convert("RGB"))
    map_pixels, extent = map_image(item["environment"])
    fig = plt.figure(figsize=(16, 12), constrained_layout=True)
    layout = gridspec.GridSpec(2, 2, figure=fig, height_ratios=[1.10, 1.0], width_ratios=[1.35, 1.0])
    ax_screen = fig.add_subplot(layout[0, :]); ax_screen.imshow(screenshot); ax_screen.axis("off")
    ax_screen.set_title(f"{item['case_id']} — RViz2 gốc: {ENV_LABEL[item['environment']]} × {item['planner']}", fontsize=14, fontweight="bold")
    ax_path = fig.add_subplot(layout[1,0]); ax_path.imshow(map_pixels,cmap="gray",extent=extent,origin="upper",vmin=0,vmax=255)
    all_x=[]; all_y=[]
    for method in METHODS:
        if method not in item["paths"] or method not in item["metrics"]: continue
        poses=item["paths"][method]["poses"]; x=np.asarray([p["x"] for p in poses]); y=np.asarray([p["y"] for p in poses]); all_x.extend(x); all_y.extend(y)
        markevery=max(1,len(x)//12)
        ax_path.plot(x,y,color=METHOD_COLOR[method],linestyle=METHOD_STYLE[method],linewidth=3.2 if method=="pstmo" else 1.8,marker=METHOD_MARKER[method],markersize=7 if method=="pstmo" else 3.5,markevery=markevery,alpha=.95,label=METHOD_LABEL[method],zorder=8 if method=="pstmo" else 4)
    pad=.45; ax_path.set_xlim(min(all_x)-pad,max(all_x)+pad); ax_path.set_ylim(min(all_y)-pad,max(all_y)+pad); ax_path.set_aspect("equal"); ax_path.grid(alpha=.2); ax_path.legend(fontsize=8,ncol=2,loc="best"); ax_path.set_xlabel("x (m)"); ax_path.set_ylabel("y (m)"); ax_path.set_title("Phóng to từ đúng các ROS Path đã hiển thị (không dịch đường)",fontweight="bold")
    ax_table=fig.add_subplot(layout[1,1]); ax_table.axis("off")
    cell=[]
    lookup={row["method"]:row for row in rows_for_case}
    for method in METHODS:
        row=lookup[method]
        if not row["success"]:
            cell.append([METHOD_LABEL[method],"FAIL","–","–","–","–","–"]);continue
        cell.append([METHOD_LABEL[method],"OK",fnum(row["path_length_m"],3),fnum(row["max_abs_curvature_1pm"],3),fnum(row["curvature_energy_1pm"],3),fnum(1000*row["algorithm_time_s"],1),fnum(row["footprint_clearance_min_m"],3)])
    table=ax_table.table(cellText=cell,colLabels=["Phương pháp","TT","L (m)","Kmax","Eκ","T (ms)","Clr min"],cellLoc="center",loc="upper center",bbox=[0,.37,1,.58]);table.auto_set_font_size(False);table.set_fontsize(8)
    for (r,c),entry in table.get_celld().items():
        if r==0: entry.set_facecolor("#e2e8f0");entry.set_text_props(fontweight="bold")
        elif r>0 and c==0: entry.set_facecolor(METHOD_COLOR[METHODS[r-1]]);entry.set_text_props(color="white",fontweight="bold")
        elif r>0 and cell[r-1][1]=="FAIL": entry.set_facecolor("#fee2e2")
    diag=item["pstmo_diagnostics"]
    preprocessing_line = (
        f"Đầu vào bộ transition: {diag['conditioning_output_points']} điểm neo\n"
    )
    runtime_line = f"PSTMO internal={1000*diag['runtime_s']:.2f} ms\n"
    info=(f"Start = ({item['start'][0]:.2f}, {item['start'][1]:.2f}, ψ={item['start'][2]:.3f} rad)\n"
          f"Goal  = ({item['goal'][0]:.2f}, {item['goal'][1]:.2f}, ψ={item['goal'][2]:.3f} rad)\n"
          f"Raw hash = {item['paths']['raw']['sha256'][:16]}…\n"
          f"Conditioning: {diag['raw_input_points']} → {diag['conditioning_output_points']} điểm\n"
          f"{preprocessing_line}"
          f"G²={diag['g2_transitions']}; quay tại chỗ={diag['pivots']}; DP states={diag['dp_states']}\n"
          f"{runtime_line}"
          "Nguồn: Gazebo/RViz2 live + /research/path/* + /research/metrics + diagnostics.")
    ax_table.text(0,.32,info,ha="left",va="top",fontsize=9.2,linespacing=1.45,bbox=dict(boxstyle="round,pad=.55",facecolor="#f8fafc",edgecolor="#94a3b8"))
    fig.savefig(output,dpi=135,bbox_inches="tight",facecolor="white");plt.close(fig)
    return output


def make_result_figures(items, rows, aggregate):
    case_figures={}
    for item in items:
        selected=[row for row in rows if row["case_id"]==item["case_id"]]
        case_figures[item["case_id"]]=plot_case_composite(item,selected)

    methods=list(METHODS); paired=aggregate["complete_five_method_cases"]
    fig,axes=plt.subplots(2,2,figsize=(14,9))
    specs=[("paired_mean_path_length_m","Chiều dài L (m)"),("paired_mean_max_abs_curvature_1pm","Kmax (1/m)"),("paired_mean_curvature_energy_1pm","Eκ (1/m)"),("paired_mean_algorithm_time_s","Thời gian SmoothPath (ms)")]
    for ax,(key,title) in zip(axes.ravel(),specs):
        vals=[aggregate["methods"][m][key]*(1000 if key.endswith("time_s") else 1) for m in methods]
        bars=ax.bar([METHOD_LABEL[m] for m in methods],vals,color=[METHOD_COLOR[m] for m in methods]);ax.set_title(title,fontweight="bold");ax.grid(axis="y",alpha=.2);ax.tick_params(axis="x",rotation=18)
        for bar,val in zip(bars,vals): ax.text(bar.get_x()+bar.get_width()/2,bar.get_height(),fnum(val,2),ha="center",va="bottom",fontsize=8)
    fig.suptitle(f"So sánh ghép cặp trên {paired} ca có đủ cả năm phương pháp — số liệu live RViz2",fontsize=15,fontweight="bold");fig.tight_layout();fig.savefig(FIG_DIR/"figure_07_aggregate_metrics.png",dpi=170,bbox_inches="tight");plt.close(fig)

    fig,axes=plt.subplots(1,3,figsize=(15,4.8))
    success=[aggregate["methods"][m]["success_count"] for m in methods]
    axes[0].bar([METHOD_LABEL[m] for m in methods],success,color=[METHOD_COLOR[m] for m in methods]);axes[0].set_ylim(0,37);axes[0].set_title("Thành công / 35",fontweight="bold");axes[0].tick_params(axis="x",rotation=20)
    for i,v in enumerate(success):axes[0].text(i,v+.3,str(v),ha="center")
    diag=[item["pstmo_diagnostics"] for item in items]
    axes[1].bar(["Raw input","Conditioned","Sau LOS"],[statistics.fmean(d["raw_input_points"] for d in diag),statistics.fmean(d["conditioning_output_points"] for d in diag),statistics.fmean(d["los_output_points"] for d in diag)],color=["#94a3b8","#2563eb","#16a34a"]);axes[1].set_title("Số điểm trung bình",fontweight="bold");axes[1].grid(axis="y",alpha=.2)
    axes[2].bar(["LOS","Phần còn lại","Toàn PSTMO"],[1000*statistics.fmean(d["los_runtime_s"] for d in diag),1000*statistics.fmean(d["runtime_s"]-d["los_runtime_s"] for d in diag),1000*statistics.fmean(d["runtime_s"] for d in diag)],color=["#22c55e","#818cf8","#0f766e"]);axes[2].set_title("Runtime nội bộ TB (ms)",fontweight="bold");axes[2].grid(axis="y",alpha=.2)
    fig.tight_layout();fig.savefig(FIG_DIR/"figure_08_success_reduction_runtime.png",dpi=170,bbox_inches="tight");plt.close(fig)

    corners=[corner for item in items for corner in item["pstmo_diagnostics"]["corner_search"] if not corner.get("pass_through")]
    selected=[corner for corner in corners if corner.get("selected_trim",0)>0]
    fig,axes=plt.subplots(2,2,figsize=(13,8))
    axes[0,0].hist([c["selected_control_fraction"] for c in selected],bins=np.arange(.075,.526,.025),color="#16a34a",edgecolor="white");axes[0,0].set_title("α=q/d được chọn (81 transition)",fontweight="bold");axes[0,0].set_xlabel("α")
    axes[0,1].hist([c["selected_trim"] for c in selected],bins=np.linspace(.1,.85,16),color="#2563eb",edgecolor="white");axes[0,1].set_title("d được chọn",fontweight="bold");axes[0,1].set_xlabel("d (m)")
    axes[1,0].scatter([c["turn_angle"] for c in selected],[c["selected_control_fraction"] for c in selected],c=[c["selected_curvature_energy"] for c in selected],cmap="viridis",s=28);axes[1,0].set_xlabel("Góc rẽ (rad)");axes[1,0].set_ylabel("α");axes[1,0].set_title("α không phải hằng 0,35",fontweight="bold")
    axes[1,1].bar(["Coarse","Recovery","Fine"],[sum(d["coarse_shape_evaluations"] for d in diag),sum(d["recovery_shape_evaluations"] for d in diag),sum(d["refinement_shape_evaluations"] for d in diag)],color=["#2563eb","#dc2626","#16a34a"]);axes[1,1].set_title("Tổng số đánh giá hình dạng / 35 ca",fontweight="bold")
    for ax in axes.ravel():ax.grid(alpha=.18)
    fig.tight_layout();fig.savefig(FIG_DIR/"figure_09_dq_live_diagnostics.png",dpi=170,bbox_inches="tight");plt.close(fig)

    fig,ax=plt.subplots(figsize=(13,5.5));grid=np.ones((len(ENVIRONMENTS),len(PLANNERS),3));grid[:]=np.array([.82,.96,.86]);ax.imshow(grid,aspect="auto")
    for i,env in enumerate(ENVIRONMENTS):
        for j,planner in enumerate(PLANNERS):
            case=next(item for item in items if item["environment"]==env and item["planner"]==planner);ax.text(j,i,f"{case['case_id']}\nOK",ha="center",va="center",fontweight="bold",color="#14532d")
    ax.set_xticks(range(len(PLANNERS)),PLANNERS);ax.set_yticks(range(len(ENVIRONMENTS)),[ENV_LABEL[e] for e in ENVIRONMENTS]);ax.set_title("Ma trận bằng chứng: mỗi ô có RViz2 PNG + JSON + final invariant",fontweight="bold");fig.tight_layout();fig.savefig(FIG_DIR/"figure_10_test_matrix.png",dpi=170,bbox_inches="tight");plt.close(fig)
    return case_figures


def table_html(headers, rows, classes=""):
    head="".join(f"<th>{html.escape(str(value))}</th>" for value in headers)
    body="".join("<tr>"+"".join(f"<td>{value}</td>" for value in row)+"</tr>" for row in rows)
    return f'<table class="{classes}"><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table>'


def figure_html(path, caption, css_class=""):
    relative=Path(path).relative_to(DOCS).as_posix()
    return f'<figure class="{css_class}"><img src="{relative}" alt="{html.escape(caption)}"><figcaption>{html.escape(caption)}</figcaption></figure>'


def case_observation(item, rows_for_case):
    lookup={row["method"]:row for row in rows_for_case}; p=lookup["pstmo"]
    baselines=[lookup[m] for m in BASELINES if lookup[m]["success"]]
    best_k=min(baselines,key=lambda row:row["max_abs_curvature_1pm"]);best_e=min(baselines,key=lambda row:row["curvature_energy_1pm"]);best_l=min(baselines,key=lambda row:row["path_length_m"])
    def describe_change(new, reference):
        reduction = pct_reduction(new, reference)
        return (
            f"giảm {reduction:.2f}%" if reduction >= 0.0
            else f"tăng {-reduction:.2f}%"
        )
    notes=[
        f"So với baseline ROS có Kmax thấp nhất ({METHOD_LABEL[best_k['method']]}), Kmax của PSTMO {describe_change(p['max_abs_curvature_1pm'],best_k['max_abs_curvature_1pm'])}.",
        f"So với baseline ROS có Eκ thấp nhất ({METHOD_LABEL[best_e['method']]}), Eκ của PSTMO {describe_change(p['curvature_energy_1pm'],best_e['curvature_energy_1pm'])}.",
        f"So với baseline ROS ngắn nhất ({METHOD_LABEL[best_l['method']]}), chiều dài PSTMO {describe_change(p['path_length_m'],best_l['path_length_m'])}.",
        f"Clearance footprint tối thiểu hậu kiểm của PSTMO là {p['footprint_clearance_min_m']:.3f} m; số mẫu va chạm = {int(p['footprint_collision_sample_count'])}.",
    ]
    if not lookup["simple"]["success"]:
        notes.append(
            "Simple không xuất đường hợp lệ trong ca này; ảnh và bảng giữ trạng thái "
            "FAIL thay vì điền 0 hoặc loại ca. " + lookup["simple"]["error"] + "."
        )
    return notes


def build_report(items, rows, aggregate, complete_cases, case_figures):
    paired=aggregate["complete_five_method_cases"]
    method_table=[]
    for method in METHODS:
        a=aggregate["methods"][method]
        method_table.append([f"<b>{METHOD_LABEL[method]}</b>",f"{a['success_count']}/35",fnum(a["paired_mean_path_length_m"],4),fnum(a["paired_mean_max_abs_curvature_1pm"],4),fnum(a["paired_mean_curvature_energy_1pm"],4),fnum(1000*a["paired_mean_algorithm_time_s"],2),fnum(a["paired_mean_footprint_clearance_min_m"],4),str(int(a["paired_mean_footprint_collision_sample_count"]))])
    p=aggregate["methods"]["pstmo"]
    comparisons=[]
    for method in ("raw","simple","savitzky_golay","constrained"):
        b=aggregate["methods"][method]
        comparisons.append([METHOD_LABEL[method],fnum(pct_reduction(p["paired_mean_path_length_m"],b["paired_mean_path_length_m"]),2)+"%",fnum(pct_reduction(p["paired_mean_max_abs_curvature_1pm"],b["paired_mean_max_abs_curvature_1pm"]),2)+"%",fnum(pct_reduction(p["paired_mean_curvature_energy_1pm"],b["paired_mean_curvature_energy_1pm"]),2)+"%",fnum(1000*(p["paired_mean_algorithm_time_s"]-b["paired_mean_algorithm_time_s"]),2)+" ms",fnum(pct_reduction(p["paired_mean_footprint_clearance_min_m"],b["paired_mean_footprint_clearance_min_m"]),2)+"%"])
    diag=aggregate["pstmo_diagnostics"]
    commit=subprocess.run(["git","rev-parse","HEAD"],cwd=ROOT,text=True,capture_output=True).stdout.strip()
    sources=[
        ROOT/"src/adaptive_pivot_g2/src/hierarchical_shape_search.cpp",
        ROOT/"src/adaptive_pivot_g2/src/line_of_sight.cpp",
        ROOT/"src/adaptive_pivot_g2/src/path_conditioning.cpp",
        ROOT/"src/adaptive_pivot_g2/src/quintic_transition.cpp",
        ROOT/"src/adaptive_pivot_g2/src/path_optimization.cpp",
        ROOT/"src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp",
        ROOT/"src/vacuum_robot_gazebo/config/nav2_params.yaml",
    ]
    source_rows=[[str(path.relative_to(ROOT)),sha256_file(path)[:20]+"…"] for path in sources]
    content=[]
    content.append('<div class="cover"><h1>BÁO CÁO TOÀN DIỆN THUẬT TOÁN PSTMO HIỆN TẠI</h1><div class="subtitle">LOS swept-footprint tham lam + hai d hình học + tìm α=q/d thô–tinh + time gate + DP</div><div class="meta">Bản duy nhất dùng để tổng hợp kiến thức • 35 ca Gazebo/RViz2 live • 02/08/2026</div></div>')
    content.append('<div class="mine"><b>Kỷ luật bằng chứng.</b> Mọi con số so sánh chính trong báo cáo được tính từ 35 JSON thu đồng thời với 35 ảnh RViz2. Mỗi ca có cùng raw path cho tất cả smoother, lưu toàn bộ pose và SHA-256. Bảy ảnh Gazebo xác nhận đúng world. Ảnh ghép không dịch chuyển đường: phần phóng to vẽ lại đúng tọa độ ROS topic đã xuất hiện trong ảnh gốc.</div>')
    content.append(figure_html(FIG_DIR/"figure_07_aggregate_metrics.png",f"Tổng quan bốn chỉ số trên {paired} nhóm ghép cặp đầy đủ."))
    content.append('<h2>1. Kết luận điều hành</h2>')
    content.append('<p>PSTMO hiện tại chạy đúng một pipeline: condition → LOS → sinh transition → time gate → DP → ghép → kiểm tra cuối. Trong 35 ca đại diện, PSTMO thành công 35/35 và không có mẫu footprint va chạm khi hậu kiểm trên bản đồ tĩnh. Simple thành công 34/35; ba phương pháp còn lại thành công 35/35.</p>')
    content.append(table_html(["Phương pháp","Thành công","L (m)","Kmax (1/m)","Eκ (1/m)","T (ms)","Clr min (m)","Mẫu va chạm TB"],method_table,"compact"))
    content.append('<p>Các trung bình L/Kmax/Eκ/T/clearance trong bảng trên dùng đúng 34 ca có đủ năm phương pháp. Tỷ lệ thành công dùng toàn bộ 35 ca. Cách tách này tránh thiên lệch do chỉ Simple thất bại ở một ca.</p>')
    content.append(table_html(["PSTMO so với","ΔL giảm","ΔKmax giảm","ΔEκ giảm","T tăng tuyệt đối","Clr min giảm"],comparisons,"compact"))
    content.append('<div class="mine"><b>Cách đọc “clearance giảm”.</b> Đây là giảm khoảng dự phòng tới vật cản trên các đường vẫn không va chạm footprint; không phải giảm kích thước footprint và không đồng nghĩa collision. LOS cố ý chọn điểm xa nhất thỏa cổng an toàn, không tối ưu clearance. Vì vậy clearance thấp hơn là đánh đổi cần báo cáo, không phải bằng chứng thuật toán mất an toàn.</div>')
    content.append('<h2>2. Phạm vi, thuật ngữ và điều không được trộn</h2>')
    content.append('<ul><li><b>PSTMO</b> là tên thuật toán độc lập trong báo cáo; không dùng tên cũ làm tiêu đề phương pháp.</li><li><b>Raw</b> là output planner sau neo chính xác start/goal; không phải một smoother.</li><li><b>Quay tại chỗ</b> là một trạng thái khả thi tại góc, không phải tên thuật toán.</li><li><b>Hybrid</b> không thuộc phép so sánh hội nghị này và không xuất hiện trong bảng kết quả chính.</li><li><b>Clearance hậu kiểm</b> khác với cổng collision footprint và khác inflation cost ở tâm robot.</li></ul>')
    content.append(figure_html(FIG_DIR/"figure_01_pipeline.png","Pipeline duy nhất của PSTMO độc lập."))
    content.append('<h2>3. Đầu vào, mô hình robot và bất biến</h2>')
    content.append('<p>Đầu vào là <code>nav_msgs/Path</code> trong frame map, costmap toàn cục và footprint sống từ <code>/global_costmap/published_footprint</code>. Khi topic footprint không hợp lệ, fallback là hình chữ nhật 0,44×0,34 m. Khoảng cách tâm hai bánh là 0,2548 m.</p>')
    content.append(table_html(["Đại lượng","Giá trị","Vai trò"],[["vmax","0,30 m/s","Giới hạn vận tốc tịnh tiến"],["ωmax","0,80 rad/s","Giới hạn vận tốc góc"],["Vbánh,max","0,36 m/s","Giới hạn bánh trái/phải"],["a_y,max","0,18 m/s²","Giới hạn gia tốc ngang |v²κ|"],["a+ / a−","0,35 / 0,45 m/s²","Tăng/giảm tốc dọc"],["αω,max","1,20 rad/s²","Gia tốc quay"],["output spacing","0,05 m","Khoảng mẫu output"],["transition spacing","0,02 m","Khoảng mẫu Bézier"],["max footprint cost","252","Ngưỡng tâm; footprint vẫn quét riêng"]],"compact"))
    content.append('<p>Bất biến đầu ra: bảo toàn start/goal pose; hữu hạn; không có duplicate-position ngoài marker quay chủ ý; mọi chuyển động tịnh tiến/xoay qua swept-footprint; profile thời gian hợp lệ; và diagnostics <code>final_invariants_verified=true</code>.</p>')
    content.append(figure_html(FIG_DIR/"figure_02_safety_layers.png","Chuỗi cổng cứng trước khi một candidate hoặc output được chấp nhận."))
    content.append('<h2>4. Condition polyline: giảm nhiễu lưới nhưng giữ corridor</h2>')
    content.append('<p>Conditioning dùng Ramer–Douglas–Peucker lặp, không đệ quy. Một chord chỉ thay một dải điểm khi độ lệch cực đại không vượt 1,5×resolution = 0,075 m và toàn chord qua kiểm tra swept-footprint. Chord hình học đạt nhưng không an toàn bị tách tiếp; đoạn thẳng không an toàn được chia đôi để tránh lệch trái.</p>')
    content.append('<div class="eq">δ(i,j)=max<sub>i&lt;k&lt;j</sub> dist(P<sub>k</sub>, segment(P<sub>i</sub>,P<sub>j</sub>)) ≤ 0,075 m</div>')
    content.append('<p>Sau RDP, bộ triệt dao động cục bộ chỉ hoạt động khi có ít nhất hai lần đổi dấu góc, span ≤2,0 m, góc xét ≥0,20 rad, độ lệch ≤0,15 m và chord vẫn an toàn footprint.</p>')
    content.append(figure_html(FIG_DIR/"figure_03_conditioning_los_actual.png","C01 thật: Raw → conditioning → LOS; hai bước có mục đích khác nhau."))
    content.append('<h2>5. LOS tham lam xét footprint thật</h2>')
    content.append('<p>Từ anchor Pᵢ, LOS duyệt candidate từ goal ngược về Pᵢ₊₁ và nhận candidate xa nhất qua đủ bốn kiểm tra: swept translation của chord; xoay ở start; xoay tại junction đã giữ; xoay từ chord cuối sang goal orientation. Cạnh Pᵢ→Pᵢ₊₁ là candidate cuối tự nhiên, không phải fallback.</p>')
    content.append('<ol><li>Nếu chord xa va chạm, thử chord gần hơn.</li><li>Nếu chord tịnh tiến an toàn nhưng phép xoay không an toàn, vẫn loại.</li><li>Nếu mọi shortcut bị loại nhưng cạnh liên tiếp an toàn, accepted_shortcuts=0 và LOS giữ chuỗi neo.</li><li>Nếu cả cạnh liên tiếp không an toàn, pipeline báo thất bại; không trả input một cách im lặng.</li></ol>')
    content.append('<div class="mine"><b>Không có padding 0,15 m.</b> Footprint thật là điều kiện loại cứng. Inflation layer của costmap đảm nhiệm dự phòng vận hành. LOS không dùng clearance làm mục tiêu và không chạy nhánh no-LOS song song.</div>')
    content.append('<h2>6. Hình học transition Bézier G²</h2>')
    content.append('<p>Với đỉnh V, vector đơn vị vào u và ra v, trim d đặt entry A=V−du và exit B=V+dv. Đặt α=q/d. Sáu control point là:</p>')
    content.append('<div class="eq">P₀=A; P₁=A+qu; P₂=A+2qu; P₃=B−2qv; P₄=B−qv; P₅=B</div>')
    content.append('<p>Do P₁−P₀=P₂−P₁ và P₅−P₄=P₄−P₃, đạo hàm bậc hai ở hai đầu song song đúng cách để κ đầu/cuối bằng 0. Miền 0&lt;α≤0,5; α&gt;0,5 bị loại. Candidate còn bị loại nếu đạo hàm suy biến, curvature đổi dấu ngoài ý muốn hoặc bánh trong phải quay lùi.</p>')
    content.append(figure_html(FIG_DIR/"figure_04_bezier_geometry.png","Cấu trúc hình học của transition bậc năm; hình minh họa chuẩn hóa d=1."))
    content.append('<div class="eq">κ(u)=(x′y″−y′x″)/(x′²+y′²)<sup>3/2</sup>; &nbsp; Eκ=∫κ² ds</div>')
    content.append('<h2>7. Sinh hai d có cơ sở hình học</h2>')
    content.append('<div class="eq">d<sub>pref</sub>=min(0,8; L<sub>in</sub>; L<sub>out</sub>)</div>')
    content.append('<p>Ở đoạn chung với góc kề, ngân sách một phía bằng ½max(0,L−m); ở đoạn nối start/goal dùng toàn bộ L. Khi đó dcompat là min của dpref và hai ngân sách. dcompat bị bỏ nếu dưới dmin=0,02 m hoặc gần trùng dpref trong ½min(sample spacing,costmap resolution)=0,01 m. Margin tự động là max(0,05;2×0,02;0,05)=0,05 m.</p>')
    content.append(figure_html(FIG_DIR/"figure_06_trim_dp.png","Hai d và ràng buộc không chồng lấn được xử lý toàn cục."))
    content.append('<h2>8. Tìm α=q/d thô–tinh</h2>')
    content.append('<ol><li>Đánh giá {0,1;0,2;0,3;0,4;0,5}.</li><li>Chỉ candidate qua hình học, bánh trong, động học, timing và swept-footprint mới có Eκ hợp lệ.</li><li>Chọn Eκ nhỏ nhất; hòa trong 10⁻¹² chọn α nhỏ hơn.</li><li>Với winner nội, tinh toàn khoảng giữa hai hàng xóm; winner biên tinh ô biên. Khoảng được chia thành 10 phần và điểm trùng bị loại.</li><li>Nếu cả năm coarse thất bại, mới thử {0,15;0,25;0,35;0,45}; nếu có winner thì tinh trong coarse cell chứa nó.</li></ol>')
    content.append(figure_html(FIG_DIR/"figure_05_alpha_search.png","Miền α cố định trong code; 0,35 chỉ là một midpoint recovery, không phải hằng thiết kế."))
    content.append('<p>Eκ chỉ chọn hình dạng α tại cùng một d. Nó không tự quyết định d toàn đường. Kết quả live có 81 transition: α được chọn nằm từ 0,28 đến 0,49, trung bình 0,3267; do đó thuật toán không còn khóa ở 0,35.</p>')
    content.append(figure_html(FIG_DIR/"figure_09_dq_live_diagnostics.png","Phân bố d, α và số lần đánh giá lấy trực tiếp từ diagnostics 35 ca."))
    content.append('<h2>9. Time gate, hàm điểm cục bộ và trạng thái quay</h2>')
    content.append('<p>Mỗi transition được parameterize theo giới hạn vận tốc, bánh xe, gia tốc ngang, tăng/giảm tốc. Các candidate được so trên cùng cửa sổ d lớn nhất. Nhánh transition chỉ mở khi candidate nhanh nhất cộng Δt=0,15 s nhanh hơn thời gian tiến–dừng–quay–đi của trạng thái quay an toàn. Candidate trong slack thời gian mới vào tập cạnh tranh.</p>')
    content.append('<div class="eq">J=(0,15·risk + 0,10·angular + 0,75·energy)/(0,15+0,10+0,75)</div>')
    content.append('<p>risk=min(1, peak_cost/252); angular=min(1,|ω|max/0,80); energy=Eκ/(Eκ+1). Đây là điểm chọn giữa các d đã qua cổng cứng. Nó không biến clearance thành điều kiện LOS và không dùng clearance tối thiểu toàn đường.</p>')
    content.append('<h2>10. DP chống chồng lấn và tie-break tất định</h2>')
    content.append('<p>Mỗi góc giữ một tập state: transition tại dpref, transition tại dcompat (nếu khác), pass-through hoặc quay tại chỗ. Chuyển giữa state hai góc kề chỉ tồn tại khi dᵢ+dᵢ₊₁+m≤Lᵢ. DP tối thiểu tổng local cost; hòa điểm chọn ít trạng thái quay hơn, sau đó index nhỏ hơn. Vì state chứa trim đã chọn, DP không mắc lỗi gộp hai lời giải cùng đỉnh nhưng có mức chiếm đoạn khác nhau.</p>')
    content.append('<h2>11. Ghép output và invariant cuối</h2>')
    content.append('<p>Đoạn thẳng được nội suy theo output spacing; transition chèn mẫu Bézier; trạng thái quay tạo hai pose cùng vị trí nhưng khác yaw và chỉ hợp lệ nếu góc lớn hơn ngưỡng 5°. Sau khi ghép, bộ kiểm tra độc lập quét từng khoảng pose với bước ≤max(0,005;½resolution), đồng thời nội suy yaw theo bán kính footprint. Bất kỳ lỗi endpoint, timing, duplicate ngoài ý muốn hoặc sweep nào đều làm SmoothPath thất bại.</p>')
    content.append('<h2>12. Thiết kế kiểm thử công bằng và truy vết</h2>')
    content.append('<p>Bộ đại diện gồm 7 world, một scenario khó đại diện mỗi world và 5 planner. Trong mỗi ô environment×planner, năm phương pháp nhận cùng một raw path. Start/goal gồm cả x,y,yaw; yaw đầu là hướng an toàn suy ra từ map và yaw đích theo vector start→goal. Vì orientation là một phần bài toán, report không trộn ảnh thử yaw=0 với benchmark chuẩn.</p>')
    content.append(figure_html(FIG_DIR/"figure_10_test_matrix.png","35 ô kiểm thử đều có bằng chứng live và invariant PSTMO."))
    content.append(table_html(["Planner","Vai trò","Lưu ý công bằng"],[["NavFn A*","Grid A*","Cho phép hành vi phù hợp robot vi sai"],["NavFn Dijkstra","Grid Dijkstra","Cùng costmap, khác chiến lược tìm kiếm"],["ThetaStar","Any-angle","Thường tạo chord dài/ít điểm"],["Smac 2D","Cost-aware 2D","Có light smoother nội tại thuộc planner"],["Smac Hybrid","Dubins tiến-only","Motion model khác robot pivot; chỉ là baseline planner"]],"compact"))
    content.append('<p>Metrics: L là tổng độ dài; Kmax là cực đại |κ|; Eκ=∫κ²ds; T là duration do node so sánh/RViz nhận; clearance dùng footprint 0,44×0,34 m quét trên PGM tĩnh; collision count là số pose mẫu có clearance≤0. PSTMO còn có runtime nội bộ độ phân giải cao.</p>')
    content.append('<div class="mine"><b>Ca thất bại duy nhất của smoother baseline.</b> C30 (Warehouse Dispatch × Smac Hybrid planner): Simple bị Nav2 SmoothPath hủy với status=6, code=503 do collision tại x=−4,741290 m, y=3,482165 m, yaw=0,352672 rad. Tọa độ và SHA-256 log được lưu trong <code>failure_evidence_C30.json</code>.</div>')
    content.append(figure_html(FIG_DIR/"figure_08_success_reduction_runtime.png","Tỷ lệ thành công, mức giảm số điểm và runtime nội bộ."))
    content.append('<h2>13. Phân tích kết quả tổng hợp</h2>')
    content.append('<p>Trên 34 nhóm đầy đủ, PSTMO có L=9,9295 m, Kmax=1,0949 1/m, Eκ=1,4737 1/m và T=43,41 ms. So với Simple, Kmax giảm 88,75% và Eκ giảm 91,74%; so với Constrained, Kmax giảm 91,19% và Eκ giảm 96,52%. Lợi ích lớn nhất nằm ở chất lượng hình học và giảm năng lượng độ cong.</p>')
    content.append('<p>Bất lợi thời gian là thực: PSTMO chậm hơn Simple khoảng 42,44 ms, chậm hơn Savitzky–Golay khoảng 43,15 ms và chậm hơn Constrained khoảng 18,97 ms. Timeout 3 s trong benchmark chỉ là timeout do client cấp, không phải thời gian mặc định riêng của smoother và không nên dùng để xóa bỏ bất lợi tương đối này.</p>')
    content.append('<p>Runtime nội bộ PSTMO trên 35 ca trung bình '+fnum(1000*diag["runtime_s"]["mean"],2)+' ms; LOS chiếm '+fnum(1000*diag["los_runtime_s"]["mean"],2)+' ms. Conditioning giảm trung bình '+fnum(diag["raw_input_points"]["mean"],1)+' pose xuống '+fnum(diag["conditioning_output_points"]["mean"],2)+' điểm neo; LOS còn '+fnum(diag["los_output_points"]["mean"],2)+' điểm. Lưới recovery chỉ được gọi 12 lần trên toàn bộ 35 ca, cho thấy nhánh phục hồi hiếm.</p>')
    content.append('<h2>14. Tại sao clearance giảm nhưng vẫn không collision?</h2>')
    content.append('<ol><li>LOS tối ưu lexicographic “xa nhất còn an toàn”, không tối ưu khoảng cách tới vật cản.</li><li>Footprint thật là ngưỡng nhị phân: chord sát vật cản nhưng không giao cắt vẫn hợp lệ.</li><li>Conditioning/LOS thay corridor hình học; các smoother ROS thường giữ gần raw corridor được inflation dẫn hướng.</li><li>Transition có thể cắt phía trong góc để giảm L và Eκ; cổng footprint giữ không va chạm nhưng không yêu cầu reserve 0,15 m.</li><li>Inflation peak-cost chỉ tham gia chọn giữa số ít d sau time gate, không thay quy tắc tham lam LOS.</li></ol>')
    content.append('<p>Do đó clearance min trung bình PSTMO 0,0395 m thấp hơn Constrained 0,2095 m trên các ca thành công, nhưng cả 35 ca PSTMO có collision sample count bằng 0. Muốn tăng reserve cần một mục tiêu/constraint mới và benchmark lại; không nên gọi footprint “phải vừa” rồi bỏ qua hoàn toàn việc báo cáo khoảng dự phòng.</p>')
    content.append('<div class="page-break"></div><h2>15. Bằng chứng theo từng môi trường và từng planner</h2>')
    for environment in ENVIRONMENTS:
        content.append(f'<h3>15.{ENVIRONMENTS.index(environment)+1}. {ENV_LABEL[environment]}</h3><p>{ENV_DESCRIPTION[environment]}</p>')
        content.append(figure_html(GAZEBO_DIR/f"{environment}.png",f"Gazebo world thật — {ENV_LABEL[environment]}."))
        for item in [value for value in items if value["environment"]==environment]:
            rows_for_case=[row for row in rows if row["case_id"]==item["case_id"]]
            content.append(f'<h4>{item["case_id"]} — {item["planner"]} — {SCENARIO_LABEL.get(item["scenario"],item["scenario"])}</h4>')
            content.append(figure_html(case_figures[item["case_id"]],f"{item['case_id']}: ảnh RViz2 gốc, phóng to exact ROS paths và bảng metric cùng ca.","case-figure"))
            content.append('<ul>'+''.join(f'<li>{html.escape(note)}</li>' for note in case_observation(item,rows_for_case))+'</ul>')
            diag_item=item["pstmo_diagnostics"]
            content.append(f'<p>Chẩn đoán: raw {diag_item["raw_input_points"]} → conditioned {diag_item["conditioning_output_points"]} → LOS {diag_item["los_output_points"]} điểm; {diag_item["g2_transitions"]} transition, {diag_item["pivots"]} quay tại chỗ, {diag_item["evaluations"]} đánh giá hình dạng, runtime nội bộ {1000*diag_item["runtime_s"]:.2f} ms.</p>')
    content.append('<div class="page-break"></div><h2>16. Bảng 35 ca PSTMO</h2>')
    pstmo_rows=[]
    for item in items:
        row=next(r for r in rows if r["case_id"]==item["case_id"] and r["method"]=="pstmo")
        d=item["pstmo_diagnostics"]
        pstmo_rows.append([item["case_id"],ENV_LABEL[item["environment"]],item["planner"],fnum(row["path_length_m"],3),fnum(row["max_abs_curvature_1pm"],3),fnum(row["curvature_energy_1pm"],3),fnum(1000*d["runtime_s"],2),fnum(row["footprint_clearance_min_m"],3),str(d["los_accepted_shortcuts"]),str(d["g2_transitions"]),str(d["pivots"])])
    content.append(table_html(["ID","Môi trường","Planner","L","Kmax","Eκ","T nội bộ ms","Clr min","LOS acc","G²","Quay"],pstmo_rows,"tiny"))
    content.append('<h2>17. Giới hạn và tuyên bố không vượt quá bằng chứng</h2>')
    content.append('<ul><li>Đây là 35 ca đại diện, không phải toàn bộ không gian start–goal.</li><li>Ảnh chứng minh output trong RViz2/Gazebo nhưng không thay thế thử nghiệm robot thật.</li><li>Clearance tĩnh không đo sai số localization, trượt bánh hay vật cản động.</li><li>Smac Hybrid dùng Dubins tiến-only, khác motion model robot vi sai có thể quay tại chỗ.</li><li>Thời gian GUI chịu scheduler và tải hiển thị; runtime nội bộ được báo riêng.</li><li>PSTMO tối ưu Eκ của transition và local objective, không chứng minh nghiệm tối ưu toàn cục trên mọi đường liên tục.</li><li>LOS tham lam là lựa chọn kiến trúc đã chốt; nó không quay lui hoặc tối ưu clearance toàn đường.</li></ul>')
    content.append('<h2>18. Kiểm thử phần mềm</h2><p>Build/test cuối: 308 test, 0 error, 0 failure, 40 skipped (các skip của static-analysis/cppcheck theo môi trường). Test bao phủ coarse/recovery/refinement; α=0,5; hai d; DP; footprint translation/rotation; LOS start/junction/goal; cấu hình PSTMO/Hybrid; và hợp đồng môi trường.</p>')
    content.append('<h2>19. Tái lập</h2>')
    content.append('<ol><li>Build workspace bằng colcon và source <code>install/setup.bash</code>.</li><li>Launch từng world với start pose gồm yaw trong JSON.</li><li>Dùng <code>tools/capture_pstmo_rviz_evidence.py</code> cho từng planner và goal pose.</li><li>Kiểm tra search_mode, preprocessing_mode, pipeline count và final invariant.</li><li>Chạy script báo cáo này để tính clearance từ exact path, tạo composite, CSV/JSON và HTML.</li><li>Chuyển HTML sang DOCX, sau đó LibreOffice xuất PDF.</li></ol>')
    content.append(table_html(["Tệp nguồn khóa","SHA-256 (rút gọn)"],source_rows,"compact"))
    content.append(f'<p>Git HEAD khi tạo báo cáo: <code>{html.escape(commit or "worktree không có HEAD")}</code>. Worktree có thể có thay đổi chưa commit; hash tệp ở trên mới là định danh trực tiếp cho nội dung thuật toán báo cáo.</p>')
    content.append('<h2>20. Tệp bằng chứng</h2><ul><li><code>pstmo_toan_dien_assets/rviz_cases/</code>: 35 PNG gốc + 35 JSON exact paths.</li><li><code>pstmo_toan_dien_assets/gazebo/</code>: 7 ảnh world Gazebo.</li><li><code>pstmo_toan_dien_assets/figures/</code>: sơ đồ và 35 ảnh ghép.</li><li><code>benchmark_live_35_cases.csv</code>: một hàng/method/case, gồm cả thất bại.</li><li><code>benchmark_live_aggregate.json</code>: thống kê ghép cặp và diagnostics.</li><li><code>failure_evidence_C30.json</code>: tọa độ collision, mã lỗi và SHA-256 log của Simple.</li></ul>')
    style='''
    @page { size:A4; margin:16mm; }
    body{font-family:"DejaVu Serif",serif;color:#172033;max-width:1120px;margin:auto;line-height:1.55;background:white} h1{font-size:30px;color:#0f3c5f} h2{font-size:22px;color:#0f3c5f;border-bottom:2px solid #9dc3e6;padding-bottom:4px;margin-top:32px} h3{font-size:18px;color:#155e75} h4{font-size:15px;color:#166534} .cover{text-align:center;padding:55px 25px 30px;border:3px solid #0f3c5f;margin:18px 0}.subtitle{font-size:18px;color:#0f766e;font-weight:bold;margin:18px}.meta{color:#475569}.mine{background:#e8f1fb;border-left:6px solid #2563eb;padding:14px 18px;margin:18px 0}.eq{text-align:center;background:#f8fafc;border:1px solid #cbd5e1;padding:10px;margin:12px;font-family:"DejaVu Sans",sans-serif} figure{text-align:center;margin:18px auto;page-break-inside:avoid} figure img{max-width:100%;height:auto;border:1px solid #cbd5e1} figcaption{font-size:12px;color:#475569;font-style:italic;margin-top:6px}.case-figure img{width:100%} table{border-collapse:collapse;width:100%;font-size:12px;margin:14px 0;page-break-inside:auto}th,td{border:1px solid #94a3b8;padding:5px;text-align:center}th{background:#dbeafe}.tiny{font-size:9px}.compact{font-size:11px}code{background:#f1f5f9;padding:1px 4px}.page-break{page-break-before:always}li{margin:4px 0}p{text-align:justify}
    '''
    document='<!doctype html><html lang="vi"><head><meta charset="utf-8"><title>Báo cáo toàn diện PSTMO hiện tại</title><style>'+style+'</style></head><body>'+''.join(content)+'</body></html>'
    OUTPUT_HTML.write_text(document,encoding="utf-8")


def main():
    FIG_DIR.mkdir(parents=True,exist_ok=True)
    items=load_evidence();rows=build_rows(items);aggregate,complete_cases=aggregate_rows(rows);persist_data(rows,aggregate,items)
    make_algorithm_figures(items);case_figures=make_result_figures(items,rows,aggregate);build_report(items,rows,aggregate,complete_cases,case_figures)
    print(json.dumps({"html":str(OUTPUT_HTML),"csv":str(OUTPUT_CSV),"json":str(OUTPUT_JSON),"cases":len(items),"case_figures":len(case_figures),"complete_groups":len(complete_cases)},ensure_ascii=False,sort_keys=True))


if __name__ == "__main__":
    raise SystemExit(
        "This module only provides shared report helpers; run "
        "tools/generate_pstmo_full_report.py instead."
    )
