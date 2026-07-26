#!/usr/bin/env python3

"""Generate the legacy compact HTML used to rebuild tutorial figures and data.

The authoritative detailed report is rebuilt from Linh's 53-page DOCX master
with ``rebuild_detailed_algorithm_report.py``.  This older renderer is retained
because it regenerates the project diagrams and provides a compact HTML view,
but it must not overwrite the detailed report linked from the README.
"""

from __future__ import annotations

import argparse
import collections
import csv
import gzip
import hashlib
import html
import json
import math
import shutil
import site
import statistics
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# ROS Jazzy's Ubuntu Matplotlib and NumPy are a matching pair.  Do not allow a
# newer user-site NumPy to shadow the system version before Matplotlib loads.
_USER_SITE = site.getusersitepackages()
if isinstance(_USER_SITE, str) and _USER_SITE in sys.path:
    sys.path.remove(_USER_SITE)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import yaml


ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
ASSETS = DOCS / "bao_cao_toan_dien_assets"
OUTPUT = DOCS / "BAO_CAO_TOAN_DIEN_ADAPTIVE_HYBRID_PIVOT_G2_LEGACY_COMPACT.html"
MAP_DIR = ROOT / "src" / "vacuum_robot_gazebo" / "maps"
WORLD_DIR = ROOT / "src" / "vacuum_robot_gazebo" / "worlds"
SCENARIO_DIR = ROOT / "src" / "adaptive_pivot_g2_benchmark" / "config"
GEOMETRY_DIR = ROOT / "results" / "conference_geometry_20260725"
EXECUTION_CSV = (
    ROOT
    / "results"
    / "conference_execution_20260725"
    / "conference_execution_compact.csv"
)
FONT_REGULAR = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

ENVIRONMENTS = (
    "research_warehouse",
    "narrow_aisles",
    "office_maze",
    "open_arena",
    "warehouse_cross_aisles",
    "warehouse_dispatch",
    "warehouse_long_aisles",
)
ENV_LABEL = {
    "research_warehouse": "Kho nghiên cứu",
    "narrow_aisles": "Lối đi hẹp",
    "office_maze": "Mê cung văn phòng",
    "open_arena": "Không gian mở",
    "warehouse_cross_aisles": "Kho có lối giao cắt",
    "warehouse_dispatch": "Kho điều phối–xuất hàng",
    "warehouse_long_aisles": "Kho có lối đi dài",
}
ENV_DESCRIPTIONS = {
    "research_warehouse": (
        "Môi trường tổng hợp gồm ba kệ khác hướng và một thùng hàng. Nó tạo cả "
        "góc vuông, đường chéo, lối hẹp cục bộ và đoạn thẳng dài; đây là map "
        "phát triển và kiểm tra hồi quy chính."
    ),
    "narrow_aisles": (
        "Bốn dãy kệ dọc đặt lệch nhau tạo hành lang ngoằn ngoèo. Map này nhấn "
        "mạnh clearance của footprint, khả năng đổi hướng liên tiếp và lỗi "
        "chọn nhầm nhánh khi các đoạn đường nằm gần nhau."
    ),
    "office_maze": (
        "Các vách ngăn đứt đoạn, cửa ra vào lệch nhau và ba bàn làm việc tạo "
        "đường chữ L/U. Đây là phép thử cho planner ở không gian nhiều ngõ cụt "
        "và cho smoother khi hai góc liên tiếp dùng chung một đoạn ngắn."
    ),
    "open_arena": (
        "Không gian thưa với cột, khối vuông và vách chắn rời rạc. Map tách "
        "ảnh hưởng của bản thân thuật toán khỏi ảnh hưởng của hành lang hẹp, "
        "phù hợp so sánh độ dài, thời gian và đường chéo."
    ),
    "warehouse_cross_aisles": (
        "Bốn dãy kệ được tách đôi ở giữa để tạo một lối ngang rộng. Robot phải "
        "đi từ lối dọc sang lối ngang rồi trở lại lối dọc; đây là bài thử rõ "
        "cho đoạn vào cong và thoát cong."
    ),
    "warehouse_dispatch": (
        "Kho hỗn hợp có vùng chứa hàng, pallet đầu vào, kệ staging, pallet đầu "
        "ra và vách dock. Đây là map khó nhất về mật độ vật cản và đã bộc lộ "
        "ca phản ví dụ ở 0,22 m/s khiến cổng Hybrid phải fallback."
    ),
    "warehouse_long_aisles": (
        "Bốn kệ dài song song tạo các lối picking hẹp, nối bằng hành lang "
        "chuyển tiếp ở hai đầu. Map kiểm tra quãng đường dài, tích lũy sai số "
        "định vị và việc tăng tốc lại sau nhiều đoạn cong."
    ),
}
SCENARIO_FILES = {
    "research_warehouse": "research_scenarios.yaml",
    "narrow_aisles": "narrow_aisles_scenarios.yaml",
    "office_maze": "office_maze_scenarios.yaml",
    "open_arena": "open_arena_scenarios.yaml",
    "warehouse_cross_aisles": "warehouse_cross_aisles_scenarios.yaml",
    "warehouse_dispatch": "warehouse_dispatch_scenarios.yaml",
    "warehouse_long_aisles": "warehouse_long_aisles_scenarios.yaml",
}
VALIDATION_PATHS = {
    "research_warehouse": (
        ROOT
        / "results"
        / "current_full_audit_20260726"
        / "lower_left_diagonal_pivot_g2_final.json.gz"
    ),
    "narrow_aisles": (
        ROOT
        / "results"
        / "current_full_audit_20260726"
        / "narrow_aisles_pivot_g2_final.json.gz"
    ),
    "office_maze": (
        ROOT
        / "results"
        / "current_full_audit_20260726"
        / "office_maze_pivot_g2_final.json.gz"
    ),
    "open_arena": (
        ROOT
        / "results"
        / "current_full_audit_20260726"
        / "open_arena_pivot_g2_final.json.gz"
    ),
    "warehouse_cross_aisles": (
        ROOT
        / "results"
        / "current_full_audit_20260726"
        / "warehouse_cross_aisles_pivot_g2_final.json.gz"
    ),
    "warehouse_dispatch": (
        ROOT
        / "results"
        / "current_full_audit_20260726"
        / "warehouse_dispatch_pivot_g2_final.json.gz"
    ),
    "warehouse_long_aisles": (
        ROOT
        / "results"
        / "current_full_audit_20260726"
        / "warehouse_long_aisles_pivot_g2_final.json.gz"
    ),
}
CURRENT_AUDIT_PATHS = {
    "lower_before": (
        ROOT
        / "results"
        / "current_full_audit_20260726"
        / "lower_left_diagonal_pivot_g2_baseline.json.gz"
    ),
    "lower_after": VALIDATION_PATHS["research_warehouse"],
    "rack_before": (
        ROOT
        / "results"
        / "current_full_audit_20260726"
        / "right_rack_detour_pivot_g2_baseline.json.gz"
    ),
    "rack_after": (
        ROOT
        / "results"
        / "current_full_audit_20260726"
        / "right_rack_detour_pivot_g2_final.json.gz"
    ),
    "narrow_before_angular_braking": (
        ROOT
        / "results"
        / "current_full_audit_20260726"
        / "narrow_aisles_pivot_g2_optimized.json.gz"
    ),
    "narrow_after": VALIDATION_PATHS["narrow_aisles"],
}
HARDWARE_PROFILE = (
    ROOT / "src" / "vacuum_robot_gazebo" / "config" / "real_robot_profile.yaml"
)

METHOD_LABEL = {
    "raw": "Raw (chưa làm mượt)",
    "simple": "Nav2 Simple",
    "savitzky_golay": "Savitzky–Golay",
    "constrained": "Nav2 Constrained",
    "pivot_g2_fixed": "Pivot–G2 bán kính cố định",
    "pivot_g2": "Pivot–G2 thích nghi",
    "adaptive_hybrid_fixed": "Hybrid dùng Pivot cố định",
    "adaptive_hybrid": "Adaptive Hybrid Pivot–G2",
}


def as_float(value, default=math.nan):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def fmt(value, digits=3):
    value = as_float(value)
    return "–" if not math.isfinite(value) else f"{value:.{digits}f}"


def percentage_reduction(new, reference):
    return 100.0 * (reference - new) / reference


def load_json(path):
    if path.suffix == ".gz":
        with gzip.open(path, "rt", encoding="utf-8") as stream:
            return json.load(stream)
    return json.loads(path.read_text(encoding="utf-8"))


def finite_mean(values):
    finite = [value for value in values if math.isfinite(value)]
    return statistics.mean(finite) if finite else math.nan


def load_geometry_summary():
    """Rebuild the compact geometry summary from the retained source CSVs.

    The old conference-report JSON was deliberately removed during repository
    cleanup.  Keeping this aggregation here makes the tutorial reproducible
    without restoring a redundant report-specific intermediate artifact.
    """
    rows = []
    for path in sorted(GEOMETRY_DIR.glob("*.csv")):
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                row["environment"] = path.stem
                row["ok"] = row.get("success", "").lower() == "true"
                rows.append(row)
    if not rows:
        raise RuntimeError(f"No geometry CSV files found in {GEOMETRY_DIR}")

    def aggregate(key_fields):
        groups = collections.defaultdict(list)
        for row in rows:
            groups[tuple(row[field] for field in key_fields)].append(row)
        output = {}
        for key, group in groups.items():
            successful = [row for row in group if row["ok"]]
            output[key] = {
                "attempts": len(group),
                "successes": len(successful),
                "energy": finite_mean(
                    as_float(row["translation_curvature_energy_1pm"])
                    for row in successful
                ),
                "clearance": finite_mean(
                    as_float(row["footprint_clearance_min_m"])
                    for row in successful
                ),
                "length": finite_mean(
                    as_float(row["path_length_m"]) for row in successful
                ),
                "deviation": finite_mean(
                    as_float(row["deviation_rmse_m"]) for row in successful
                ),
                "runtime_ms": 1000.0
                * finite_mean(
                    as_float(row["algorithm_time_s"]) for row in successful
                ),
                "collisions": sum(
                    int(as_float(row["footprint_collision_sample_count"], 0.0))
                    for row in successful
                ),
            }
        return output

    overall = aggregate(("method",))
    by_map = aggregate(("environment", "method"))
    pairing = collections.defaultdict(set)
    for row in rows:
        pairing[
            (
                row["environment"],
                row["scenario"],
                row["planner"],
                row["repetition"],
            )
        ].add(row["raw_path_sha256"])
    bad_pairing = [key for key, hashes in pairing.items() if len(hashes) != 1]
    return {
        "geometry_row_count": len(rows),
        "geometry_pairing_group_count": len(pairing),
        "geometry_pairing_bad_groups": bad_pairing,
        "geometry_overall": {
            method: overall[(method,)] for method in METHOD_LABEL
        },
        "geometry_by_map_method": {
            f"{environment}/{method}": by_map[(environment, method)]
            for environment in ENVIRONMENTS
            for method in METHOD_LABEL
        },
    }


def load_execution_rows():
    with EXECUTION_CSV.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def load_validation():
    return {
        environment: load_json(path)
        for environment, path in VALIDATION_PATHS.items()
    }


def load_current_audit():
    return {
        label: load_json(path)
        for label, path in CURRENT_AUDIT_PATHS.items()
    }


def load_hardware_profile():
    return yaml.safe_load(HARDWARE_PROFILE.read_text(encoding="utf-8"))


def load_scenarios(environment):
    data = yaml.safe_load(
        (SCENARIO_DIR / SCENARIO_FILES[environment]).read_text(encoding="utf-8")
    )
    return data["scenarios"]


def load_map(environment):
    metadata = yaml.safe_load(
        (MAP_DIR / f"{environment}.yaml").read_text(encoding="utf-8")
    )
    occupancy = np.asarray(Image.open(MAP_DIR / metadata["image"]))
    return metadata, occupancy


def parse_world_boxes(environment):
    root = ET.parse(WORLD_DIR / f"{environment}.sdf").getroot()
    boxes = []
    for visual in root.findall(".//visual"):
        size_text = visual.findtext("./geometry/box/size")
        pose_text = visual.findtext("pose")
        if not size_text or not pose_text:
            continue
        size = tuple(float(value) for value in size_text.split())
        pose = tuple(float(value) for value in pose_text.split())
        material = visual.findtext("./material/diffuse")
        rgba = (
            tuple(float(value) for value in material.split())
            if material else (0.35, 0.45, 0.60, 1.0)
        )
        boxes.append(
            {
                "name": visual.get("name", ""),
                "size": size,
                "pose": pose,
                "color": rgba[:3],
            }
        )
    return boxes


def set_plot_style():
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "axes.titlesize": 11,
            "axes.labelsize": 9,
            "figure.dpi": 150,
            "savefig.dpi": 180,
            "axes.grid": True,
            "grid.alpha": 0.22,
        }
    )


def font(size, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT_REGULAR, size)


def draw_box(draw, box, text, fill, outline="#334155", text_fill="#102a43", size=24):
    draw.rounded_rectangle(box, radius=18, fill=fill, outline=outline, width=3)
    selected = font(size, bold=True)
    bounds = draw.multiline_textbbox((0, 0), text, font=selected, align="center", spacing=6)
    width = bounds[2] - bounds[0]
    height = bounds[3] - bounds[1]
    x = (box[0] + box[2] - width) / 2
    y = (box[1] + box[3] - height) / 2
    draw.multiline_text(
        (x, y), text, font=selected, fill=text_fill, align="center", spacing=6
    )


def draw_arrow(draw, start, end, fill="#475569", width=5):
    draw.line((start, end), fill=fill, width=width)
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = max(math.hypot(dx, dy), 1.0)
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    base = (end[0] - 18 * ux, end[1] - 18 * uy)
    draw.polygon(
        [
            end,
            (base[0] + 9 * px, base[1] + 9 * py),
            (base[0] - 9 * px, base[1] - 9 * py),
        ],
        fill=fill,
    )


def save_learning_roadmap():
    image = Image.new("RGB", (1900, 1150), "white")
    draw = ImageDraw.Draw(image)
    draw.text(
        (950, 55),
        "TỪ GOAL TRONG RVIZ2 ĐẾN CHUYỂN ĐỘNG THẬT TRONG GAZEBO",
        anchor="ma",
        font=font(40, True),
        fill="#0f2744",
    )
    stages = [
        ("1. Nhận thức", "map + lidar + TF\n→ robot đang ở đâu?", "#dbeafe"),
        ("2. Planner", "tìm một đường thô\nkhông xuyên vật cản", "#dcfce7"),
        ("3. Phương pháp của bạn", "Adaptive Hybrid Pivot–G2\nlàm mượt + cổng an toàn", "#fce7f3"),
        ("4. Profile vận tốc", "giới hạn bánh, cong,\ngia tốc và jerk", "#fef3c7"),
        ("5. Controller", "RPP + projection có hướng\n+ Pivot/servo đích", "#e0e7ff"),
        ("6. Robot/Gazebo", "cmd_vel → hai bánh\n→ ground truth", "#ccfbf1"),
        ("7. Đánh giá", "RMSE, clearance, Eκ,\nthời gian, success", "#f3e8ff"),
    ]
    y = 150
    for index, (title, text, color) in enumerate(stages):
        box = (420, y, 1480, y + 105)
        draw_box(draw, box, f"{title}\n{text}", color, size=24)
        if index + 1 < len(stages):
            draw_arrow(draw, (950, y + 105), (950, y + 137))
        y += 137
    draw.text(
        (950, 1110),
        "Mỗi khối đều được giải thích từ khái niệm cơ bản đến công thức và dữ liệu đo.",
        anchor="ms",
        font=font(24),
        fill="#475569",
    )
    image.save(ASSETS / "figure_01_learning_roadmap.png")


def save_ros_nav2_flow():
    image = Image.new("RGB", (2450, 1200), "white")
    draw = ImageDraw.Draw(image)
    draw.text(
        (1225, 45), "HỆ ROS 2 / NAV2 CỦA DỰ ÁN", anchor="ma",
        font=font(42, True), fill="#0f2744",
    )
    boxes = {
        "rviz": (40, 145, 330, 335),
        "bt": (400, 145, 680, 335),
        "planner": (750, 120, 1070, 360),
        "smoother": (1140, 120, 1480, 360),
        "controller": (1550, 120, 1900, 360),
        "gazebo": (1970, 120, 2400, 360),
        "costmap": (750, 510, 1070, 700),
        "localization": (1140, 510, 1480, 700),
        "sensors": (1550, 510, 1900, 700),
        "benchmark": (850, 870, 1600, 1050),
    }
    labels = {
        "rviz": "RViz2\nGoal + panel chọn",
        "bt": "BT Navigator\nđiều phối nhiệm vụ",
        "planner": "Planner Server\n5 global planner",
        "smoother": "Smoother Server\n7 smoother\n(phương pháp của bạn ở đây)",
        "controller": "Controller Server\nManeuver-aware RPP\n+ profile vận tốc",
        "gazebo": "Gazebo Harmonic\nphysics + robot + world",
        "sensors": "Scan / IMU / odom\n/ ground truth",
        "localization": "AMCL + TF\nmap → odom → base_link",
        "costmap": "Global/Local Costmap\nstatic + voxel + inflation",
        "benchmark": "Benchmark\nhash + metrics + traces",
    }
    colors = {
        "rviz": "#dbeafe", "bt": "#e0e7ff", "planner": "#dcfce7",
        "smoother": "#fce7f3", "controller": "#fef3c7",
        "gazebo": "#ccfbf1", "sensors": "#cffafe",
        "localization": "#ede9fe", "costmap": "#fee2e2",
        "benchmark": "#f3e8ff",
    }

    def poly_arrow(points, fill="#475569", width=4):
        for start, end in zip(points[:-2], points[1:-1]):
            draw.line([start, end], fill=fill, width=width, joint="curve")
        draw_arrow(draw, points[-2], points[-1], fill=fill, width=width)

    # Vẽ đường trước hộp để các tuyến phản hồi đi "sau" node, dễ đọc hơn.
    for source, target in [
        ("rviz", "bt"),
        ("bt", "planner"),
        ("planner", "smoother"),
        ("smoother", "controller"),
        ("controller", "gazebo"),
    ]:
        source_box, target_box = boxes[source], boxes[target]
        draw_arrow(
            draw,
            (source_box[2], (source_box[1] + source_box[3]) // 2),
            (target_box[0], (target_box[1] + target_box[3]) // 2),
            width=5,
        )

    # Vòng phản hồi vật lý: Gazebo -> cảm biến -> định vị -> costmap -> Nav2.
    poly_arrow([(2185, 360), (2185, 430), (1725, 430), (1725, 510)])
    draw_arrow(draw, (1550, 605), (1480, 605), width=5)
    draw_arrow(draw, (1140, 605), (1070, 605), width=5)
    draw_arrow(draw, (910, 510), (910, 360), width=5)
    # Costmap còn cấp kiểm tra va chạm/giới hạn cho controller; tuyến đi ngoài các hộp.
    poly_arrow(
        [(910, 700), (910, 780), (1515, 780), (1515, 400), (1725, 400), (1725, 360)],
        fill="#64748b",
    )
    # Dữ liệu thực thi được ghi một lần tại benchmark, không trộn với luồng điều khiển.
    poly_arrow([(1725, 700), (1725, 820), (1225, 820), (1225, 870)])

    for key, box in boxes.items():
        draw_box(draw, box, labels[key], colors[key], size=25)

    draw.text(
        (1225, 460), "PHẢN HỒI TRẠNG THÁI / TOPIC + TF", anchor="ms",
        font=font(21, True), fill="#0f766e",
    )
    draw.text(
        (1990, 420), "physics → sensor", anchor="ls",
        font=font(19), fill="#475569",
    )
    draw.text(
        (1030, 755), "costmap/footprint → kiểm tra an toàn controller",
        anchor="ls", font=font(18), fill="#64748b",
    )
    draw.text(
        (1225, 840), "GHI DỮ LIỆU ĐỂ SO SÁNH", anchor="ms",
        font=font(20, True), fill="#7e22ce",
    )
    draw.text(
        (1225, 1140),
        "Action dùng cho nhiệm vụ dài; topic dùng cho dòng dữ liệu; TF mô tả quan hệ giữa các frame.",
        anchor="ms", font=font(25), fill="#475569",
    )
    image.save(ASSETS / "figure_02_ros_nav2_flow.png")


def save_diff_drive_kinematics():
    figure, axis = plt.subplots(figsize=(10.5, 6.0))
    axis.set_aspect("equal")
    axis.set_xlim(-1.6, 3.2)
    axis.set_ylim(-1.8, 1.8)
    axis.axis("off")
    body = Rectangle((-0.65, -0.48), 1.3, 0.96, fc="#cbd5e1", ec="#334155", lw=2)
    axis.add_patch(body)
    axis.add_patch(Rectangle((-0.45, 0.48), 0.9, 0.16, fc="#111827"))
    axis.add_patch(Rectangle((-0.45, -0.64), 0.9, 0.16, fc="#111827"))
    axis.arrow(0, 0, 1.15, 0, width=0.025, head_width=0.16, color="#2563eb")
    axis.text(1.22, 0.06, "v: vận tốc tiến", color="#2563eb", weight="bold")
    axis.add_patch(FancyArrowPatch(
        (0.45, -0.15), (0.45, 0.35), connectionstyle="arc3,rad=0.8",
        arrowstyle="-|>", mutation_scale=18, lw=2, color="#c026d3",
    ))
    axis.text(0.72, 0.42, "ω: vận tốc góc", color="#c026d3", weight="bold")
    axis.arrow(-0.2, 0.65, 1.0, 0, width=0.018, head_width=0.12, color="#16a34a")
    axis.arrow(-0.2, -0.75, 0.65, 0, width=0.018, head_width=0.12, color="#f97316")
    axis.text(0.88, 0.72, "vR: bánh phải", color="#16a34a")
    axis.text(0.53, -0.86, "vL: bánh trái", color="#f97316")
    axis.annotate(
        "", xy=(-0.9, 0.48), xytext=(-0.9, -0.48),
        arrowprops={"arrowstyle": "<->", "lw": 2, "color": "#475569"},
    )
    axis.text(-1.2, 0, "L\nkhoảng cách\nhai vệt lăn", ha="center", va="center")
    axis.text(
        1.35, -0.55,
        "v = (vR + vL)/2\nω = (vR − vL)/L\nκ = ω/v\n"
        "vL = v(1 − Lκ/2)\nvR = v(1 + Lκ/2)",
        fontsize=13,
        bbox={"boxstyle": "round,pad=0.5", "fc": "#f8fafc", "ec": "#94a3b8"},
    )
    axis.text(
        -1.45, 1.45,
        "Robot vi sai: không trượt ngang lý tưởng; muốn quay phải tạo chênh lệch tốc độ hai bánh.",
        fontsize=12, weight="bold",
    )
    figure.tight_layout()
    figure.savefig(ASSETS / "figure_03_diff_drive.png", bbox_inches="tight")
    plt.close(figure)


def save_costmap_footprint():
    metadata, occupancy = load_map("research_warehouse")
    resolution = float(metadata["resolution"])
    ox, oy = map(float, metadata["origin"][:2])
    height, width = occupancy.shape[:2]
    extent = (ox, ox + width * resolution, oy, oy + height * resolution)
    figure, axes = plt.subplots(1, 2, figsize=(11.6, 4.8))
    for axis in axes:
        axis.imshow(
            occupancy, cmap="gray", vmin=0, vmax=255, origin="upper", extent=extent
        )
        axis.set_aspect("equal")
        axis.set_xlim(-5.2, -2.0)
        axis.set_ylim(-3.1, -0.4)
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
    axes[0].scatter([-3.1], [-1.8], c="#2563eb", s=55)
    axes[0].set_title("Sai: chỉ kiểm tra tâm robot")
    footprint = np.asarray(
        [[0.22, 0.17], [0.22, -0.17], [-0.22, -0.17], [-0.22, 0.17]]
    )
    center = np.asarray([-3.1, -1.8])
    axes[1].add_patch(
        Polygon(footprint + center, closed=True, fc="#60a5fa", ec="#1d4ed8", alpha=0.7)
    )
    for ratio in np.linspace(0, 1, 8):
        c = (1 - ratio) * np.asarray([-3.1, -2.6]) + ratio * np.asarray([-3.1, -1.1])
        axes[1].add_patch(
            Polygon(footprint + c, closed=True, fc="none", ec="#f97316", alpha=0.55)
        )
    axes[1].set_title("Đúng: swept footprint trên toàn chuyển động")
    figure.suptitle(
        "Costmap cho biết ô bị chiếm; footprint mới cho biết toàn thân xe có va chạm hay không",
        weight="bold",
    )
    figure.tight_layout()
    figure.savefig(ASSETS / "figure_04_costmap_footprint.png", bbox_inches="tight")
    plt.close(figure)


def bezier_points(control, samples=301):
    u = np.linspace(0.0, 1.0, samples)
    coefficients = np.asarray([1, 5, 10, 10, 5, 1], dtype=float)
    curve = np.zeros((samples, 2), dtype=float)
    for index in range(6):
        basis = coefficients[index] * (1 - u) ** (5 - index) * u ** index
        curve += basis[:, None] * control[index]
    return curve


def save_pivot_g2_geometry():
    phi = math.pi / 2
    radius = 0.6
    trim = radius * math.tan(abs(phi) / 2)
    q = 0.35 * trim
    entry = np.asarray([-trim, 0.0])
    exit_point = np.asarray([0.0, trim])
    incoming = np.asarray([1.0, 0.0])
    outgoing = np.asarray([0.0, 1.0])
    control = np.asarray(
        [
            entry,
            entry + q * incoming,
            entry + 2 * q * incoming,
            exit_point - 2 * q * outgoing,
            exit_point - q * outgoing,
            exit_point,
        ]
    )
    curve = bezier_points(control)
    figure, axes = plt.subplots(1, 2, figsize=(11.8, 5.2))
    axis = axes[0]
    axis.plot([-1.2, 0], [0, 0], "--", color="#94a3b8")
    axis.plot([0, 0], [0, 1.2], "--", color="#94a3b8")
    axis.plot(curve[:, 0], curve[:, 1], color="#c026d3", lw=3, label="Bézier bậc 5")
    axis.plot(control[:, 0], control[:, 1], "o--", color="#2563eb", ms=5, label="P0…P5")
    axis.scatter([0], [0], marker="x", s=80, c="#dc2626", label="đỉnh góc")
    axis.annotate("", xy=entry, xytext=(0, 0), arrowprops={"arrowstyle": "<->"})
    axis.text(-trim / 2, -0.08, "d")
    axis.annotate("", xy=exit_point, xytext=(0, 0), arrowprops={"arrowstyle": "<->"})
    axis.text(0.05, trim / 2, "d")
    axis.set_aspect("equal")
    axis.set_title("Transition G2: xe đi qua một đường cong")
    axis.legend(loc="lower right")
    axis.grid(True)
    axis = axes[1]
    axis.plot([-1.2, 0], [0, 0], color="#2563eb", lw=3)
    axis.scatter([0], [0], s=130, c="#f97316", marker="o")
    axis.arrow(0, 0, 0, 0.9, width=0.018, head_width=0.09, color="#16a34a")
    axis.add_patch(Circle((0, 0), 0.16, fc="none", ec="#c026d3", lw=3))
    axis.text(0.18, 0.14, "Pivot: d = 0\nđứng yên và quay", fontsize=12)
    axis.set_xlim(-1.2, 1.0)
    axis.set_ylim(-0.5, 1.2)
    axis.set_aspect("equal")
    axis.set_title("Trạng thái Pivot: không có đoạn cong")
    axis.grid(True)
    figure.suptitle(
        "Hai cách xử lý một góc của phương pháp Pivot–G2", weight="bold"
    )
    figure.tight_layout()
    figure.savefig(ASSETS / "figure_05_pivot_g2_geometry.png", bbox_inches="tight")
    plt.close(figure)


def save_search_dp():
    image = Image.new("RGB", (2100, 1220), "white")
    draw = ImageDraw.Draw(image)
    draw.text(
        (1050, 45), "TÌM KIẾM THÍCH NGHI + QUY HOẠCH ĐỘNG TOÀN ĐƯỜNG",
        anchor="ma", font=font(40, True), fill="#0f2744",
    )
    draw_box(
        draw, (80, 150, 620, 330),
        "Mỗi góc\nmiền d khả thi từ hình học",
        "#dbeafe", size=27,
    )
    draw_box(
        draw, (780, 150, 1320, 330),
        "Coarse-to-fine\n6 mẫu đầu, tối đa 20 đánh giá",
        "#fce7f3", size=27,
    )
    draw_box(
        draw, (1480, 150, 2020, 330),
        "Giữ tối đa 5 ứng viên an toàn\n+ trạng thái Pivot d=0",
        "#dcfce7", size=26,
    )
    draw_arrow(draw, (620, 240), (780, 240))
    draw_arrow(draw, (1320, 240), (1480, 240))
    columns = [310, 760, 1210, 1660]
    states = [
        [("P", 0.0), ("G2", 0.22), ("G2", 0.37)],
        [("P", 0.0), ("G2", 0.18), ("G2", 0.44)],
        [("P", 0.0), ("G2", 0.25), ("G2", 0.52)],
        [("P", 0.0), ("G2", 0.16), ("G2", 0.41)],
    ]
    for corner, x in enumerate(columns):
        draw.text(
            (x, 430), f"Góc {corner + 1}", anchor="ma",
            font=font(28, True), fill="#334155",
        )
        for state_index, (kind, trim) in enumerate(states[corner]):
            y = 540 + 210 * state_index
            color = "#fef3c7" if kind == "P" else "#e0e7ff"
            draw_box(
                draw, (x - 145, y - 60, x + 145, y + 60),
                f"{kind}\nd={trim:.2f} m", color, size=23,
            )
    for corner in range(3):
        left_x, right_x = columns[corner], columns[corner + 1]
        for left_index in range(3):
            for right_index in range(3):
                ly = 540 + 210 * left_index
                ry = 540 + 210 * right_index
                compatible = (
                    states[corner][left_index][1]
                    + states[corner + 1][right_index][1]
                    + 0.05 <= 0.78
                )
                if compatible:
                    draw.line(
                        (left_x + 145, ly, right_x - 145, ry),
                        fill="#94a3b8", width=2,
                    )
    draw.line((455, 750, 615, 750), fill="#c026d3", width=8)
    draw.line((905, 750, 1065, 960), fill="#c026d3", width=8)
    draw.line((1355, 960, 1515, 750), fill="#c026d3", width=8)
    draw.text(
        (1050, 1160),
        "DP chỉ nối hai trạng thái nếu dᵢ + dᵢ₊₁ + margin ≤ chiều dài đoạn chung; "
        "đường màu tím là chuỗi có tổng cost thấp nhất.",
        anchor="ms", font=font(24), fill="#475569",
    )
    image.save(ASSETS / "figure_06_search_dp.png")


def save_map_detail(environment, validation):
    metadata, occupancy = load_map(environment)
    resolution = float(metadata["resolution"])
    origin_x, origin_y = map(float, metadata["origin"][:2])
    height, width = occupancy.shape[:2]
    extent = (
        origin_x,
        origin_x + width * resolution,
        origin_y,
        origin_y + height * resolution,
    )
    scenarios = load_scenarios(environment)
    boxes = parse_world_boxes(environment)
    figure = plt.figure(figsize=(13.2, 6.2))
    map_axis = figure.add_subplot(1, 2, 1)
    map_axis.imshow(
        occupancy, cmap="gray", vmin=0, vmax=255, origin="upper", extent=extent
    )
    for index, scenario in enumerate(scenarios):
        start = scenario["start"][:2]
        goal = scenario["goal"][:2]
        map_axis.plot(
            [start[0], goal[0]], [start[1], goal[1]],
            color="#94a3b8", alpha=0.18, lw=0.8,
        )
        map_axis.scatter(
            [start[0]], [start[1]], c="#16a34a", s=9, alpha=0.5
        )
        map_axis.scatter(
            [goal[0]], [goal[1]], c="#dc2626", s=9, alpha=0.5
        )
    selected = np.asarray(validation["selected_path_xy"], dtype=float)
    trace = np.asarray(
        [[sample[1], sample[2]] for sample in validation["ground_truth_state_trace"]],
        dtype=float,
    )
    map_axis.plot(
        selected[:, 0], selected[:, 1], color="#1d4ed8", lw=2.2,
        label="đường Pivot–G2 được chọn",
    )
    map_axis.plot(
        trace[:, 0], trace[:, 1], color="#f97316", lw=1.3,
        label="ground truth Gazebo",
    )
    map_axis.scatter(
        selected[[0, -1], 0], selected[[0, -1], 1],
        c=["#16a34a", "#dc2626"], s=45, zorder=5,
    )
    map_axis.set_aspect("equal")
    map_axis.set_xlabel("x trong frame map (m)")
    map_axis.set_ylabel("y trong frame map (m)")
    map_axis.set_title(
        f"RViz2 / occupancy grid\n{len(scenarios)} scenario, 0,05 m/ô"
    )
    map_axis.legend(loc="lower right", fontsize=7)

    world_axis = figure.add_subplot(1, 2, 2, projection="3d")
    for box in boxes:
        x, y, z = box["pose"][:3]
        sx, sy, sz = box["size"]
        color = box["color"]
        alpha = 0.35 if "floor" in box["name"] else 0.82
        world_axis.bar3d(
            x - sx / 2, y - sy / 2, z - sz / 2,
            sx, sy, sz, color=color, alpha=alpha, shade=True,
            edgecolor="#475569" if "floor" not in box["name"] else "none",
            linewidth=0.25,
        )
    world_axis.set_xlim(-6.2, 6.2)
    world_axis.set_ylim(-4.2, 4.2)
    world_axis.set_zlim(0, 1.5)
    world_axis.set_box_aspect((12.4, 8.4, 3.3))
    world_axis.view_init(elev=31, azim=-58)
    world_axis.set_xlabel("x (m)")
    world_axis.set_ylabel("y (m)")
    world_axis.set_zlabel("z (m)")
    world_axis.set_title(
        f"Gazebo / SDF\n{max(0, len(boxes) - 5)} vật thể bên trong"
    )
    figure.suptitle(
        f"{ENV_LABEL[environment]} — cùng một hình học trong RViz2 và Gazebo",
        fontsize=14, weight="bold",
    )
    figure.tight_layout(rect=(0, 0, 1, 0.94))
    figure.savefig(ASSETS / f"map_{environment}.png", bbox_inches="tight")
    plt.close(figure)


def copy_evidence_assets():
    sources = {
        "figure_07_robot_3d.png": (
            DOCS / "rev_ecit_2026_assets" / "figure_07_robot_3d.png"
        ),
        "figure_08_rviz_gazebo_ui.png": (
            DOCS / "rev_ecit_2026_assets" / "figure_08_rviz_gazebo_ui.png"
        ),
        "figure_09_rviz_methods.png": (
            DOCS / "rev_ecit_2026_assets" / "figure_10_rviz_all_methods.png"
        ),
        "figure_11_geometry_comparison.png": (
            DOCS / "rev_ecit_2026_assets" / "figure_02_geometry_overview.png"
        ),
        "figure_12_map_energy_comparison.png": (
            DOCS / "rev_ecit_2026_assets" / "figure_03_map_energy_ratio.png"
        ),
        "figure_13_speed_comparison.png": (
            DOCS / "rev_ecit_2026_assets" / "figure_04_speed_tracking.png"
        ),
    }
    for target, source in sources.items():
        if not source.exists():
            raise FileNotFoundError(source)
        shutil.copy2(source, ASSETS / target)


def save_current_speed_trace(validation):
    """Plot the current 2026-07-26 trace, not the older conference trace."""
    row = validation["research_warehouse"]
    selected = np.asarray(row["selected_path_xy"], dtype=float)
    ground_truth = np.asarray(row["ground_truth_state_trace"], dtype=object)
    trace = np.asarray(row["adaptive_speed_trace"], dtype=object)
    fields = {
        name: index for index, name in enumerate(row["adaptive_speed_trace_fields"])
    }

    # Giữ ba panel trên một hàng nhưng tăng chiều cao để nhãn trục, legend và
    # dao động nhỏ vẫn đọc được khi hình được đặt vừa chiều rộng trang A4.
    figure, axes = plt.subplots(1, 3, figsize=(14.8, 6.4))
    axes[0].plot(
        selected[:, 0], selected[:, 1], color="#1d4ed8", lw=2.4,
        label="selected path",
    )
    axes[0].plot(
        ground_truth[:, 1].astype(float),
        ground_truth[:, 2].astype(float),
        color="#f97316", lw=1.5, label="ground truth Gazebo",
    )
    axes[0].scatter(
        selected[[0, -1], 0], selected[[0, -1], 1],
        c=["#16a34a", "#dc2626"], s=42, zorder=5,
    )
    axes[0].set_aspect("equal")
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    axes[0].set_title("Quỹ đạo vật lý và đường tham chiếu")
    axes[0].legend(fontsize=8)
    axes[0].grid(True)

    time = trace[:, fields["time_s"]].astype(float)
    for field, label, color, width in (
        ("measured_speed_mps", "v đo", "#0f766e", 1.5),
        ("command_speed_mps", "v lệnh", "#2563eb", 1.8),
        ("profile_cap_mps", "trần profile", "#dc2626", 1.4),
    ):
        axes[1].plot(
            time, trace[:, fields[field]].astype(float),
            label=label, color=color, lw=width,
        )
    axes[1].set_xlabel("thời gian (s)")
    axes[1].set_ylabel("vận tốc (m/s)")
    axes[1].set_title("Profile vận tốc thực hiện")
    axes[1].legend(fontsize=8)
    axes[1].grid(True)

    cross_track_cm = 100.0 * np.abs(
        trace[:, fields["cross_track_error_m"]].astype(float)
    )
    heading_deg = np.degrees(
        np.abs(trace[:, fields["path_heading_error_rad"]].astype(float))
    )
    axes[2].plot(
        time, cross_track_cm, color="#7c3aed", lw=1.5,
        label="sai số ngang (cm)",
    )
    heading_axis = axes[2].twinx()
    heading_axis.plot(
        time, heading_deg, color="#ea580c", lw=1.2, alpha=0.8,
        label="sai số hướng (độ)",
    )
    axes[2].set_xlabel("thời gian (s)")
    axes[2].set_ylabel("sai số ngang (cm)", color="#7c3aed")
    heading_axis.set_ylabel("sai số hướng (độ)", color="#ea580c")
    axes[2].set_title("Sai số mà controller quan sát")
    axes[2].grid(True)
    lines = axes[2].lines + heading_axis.lines
    axes[2].legend(lines, [line.get_label() for line in lines], fontsize=8)

    figure.suptitle(
        "Trace cuối 26/07/2026 — research_warehouse/lower_left_diagonal\n"
        f"t = {row['execution_time_s']:.3f} s; "
        f"GT RMSE = {100.0 * row['tracking_rmse_m']:.3f} cm; "
        f"GT max = {100.0 * row['tracking_max_error_m']:.3f} cm",
        fontsize=13, weight="bold",
    )
    figure.tight_layout(rect=(0, 0, 1, 0.90))
    figure.savefig(ASSETS / "figure_10_speed_trace.png", bbox_inches="tight")
    plt.close(figure)


def save_current_all_map_error(validation):
    labels = [ENV_LABEL[name] for name in ENVIRONMENTS]
    metrics = (
        (
            "RMSE bám theo ground truth",
            [100.0 * validation[name]["tracking_rmse_m"] for name in ENVIRONMENTS],
            "#2563eb",
        ),
        (
            "RMSE bám trong hệ điều khiển",
            [
                100.0 * validation[name]["estimated_tracking_rmse_m"]
                for name in ENVIRONMENTS
            ],
            "#0f766e",
        ),
        (
            "Sai số định vị P95",
            [
                100.0 * validation[name]["localization_position_error_p95_m"]
                for name in ENVIRONMENTS
            ],
            "#d97706",
        ),
    )
    figure, axes = plt.subplots(1, 3, figsize=(14.8, 4.8))
    x = np.arange(len(labels))
    for axis, (title, values, color) in zip(axes, metrics):
        bars = axis.bar(x, values, color=color)
        axis.set_xticks(x, labels, rotation=43, ha="right")
        axis.set_ylabel("cm")
        axis.set_title(title)
        axis.grid(True, axis="y", alpha=0.35)
        for bar, value in zip(bars, values):
            axis.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height(),
                f"{value:.2f}",
                ha="center", va="bottom", fontsize=7,
            )
    figure.suptitle(
        "Kiểm chứng chạy kín Pivot–G2 thích nghi trên 7 môi trường — "
        "trace cuối 26/07/2026",
        fontsize=14, weight="bold",
    )
    figure.tight_layout(rect=(0, 0, 1, 0.93))
    figure.savefig(ASSETS / "figure_14_all_map_error.png", bbox_inches="tight")
    plt.close(figure)


def save_current_audit_comparison(current_audit):
    """Show improvements and regressions honestly on a common 100% baseline."""
    lower_before = current_audit["lower_before"]
    lower_after = current_audit["lower_after"]
    rack_before = current_audit["rack_before"]
    rack_after = current_audit["rack_after"]
    narrow_before = current_audit["narrow_before_angular_braking"]
    narrow_after = current_audit["narrow_after"]
    groups = (
        (
            "lower_left_diagonal",
            (
                ("GT RMSE", "tracking_rmse_m"),
                ("Odom vị trí P95", "odometry_position_error_p95_m"),
                ("Odom yaw P95", "odometry_yaw_error_p95_rad"),
                ("RMSE controller", "estimated_tracking_rmse_m"),
            ),
            lower_before,
            lower_after,
        ),
        (
            "right_rack_detour",
            (
                ("Thời gian", "execution_time_s"),
                ("GT RMSE", "tracking_rmse_m"),
                ("Exit max", "curve_exit_tracking_max_error_m"),
                ("Odom yaw P95", "odometry_yaw_error_p95_rad"),
            ),
            rack_before,
            rack_after,
        ),
        (
            "narrow_aisles",
            (
                ("Thời gian", "execution_time_s"),
                ("Initial alignment", "adaptive_speed_mode_sample_counts.initial_alignment"),
                ("GT RMSE", "tracking_rmse_m"),
                ("Exit max", "curve_exit_tracking_max_error_m"),
            ),
            narrow_before,
            narrow_after,
        ),
    )

    def get(row, key):
        if "." not in key:
            return float(row[key])
        first, second = key.split(".", 1)
        return float(row[first][second])

    figure, axes = plt.subplots(1, 3, figsize=(14.8, 4.6))
    for axis, (title, metrics, before, after) in zip(axes, groups):
        labels = [label for label, _ in metrics]
        ratios = [100.0 * get(after, key) / get(before, key) for _, key in metrics]
        y = np.arange(len(labels))
        axis.axvline(100.0, color="#475569", lw=1.2, ls="--")
        colors = ["#16a34a" if ratio < 100.0 else "#dc2626" for ratio in ratios]
        bars = axis.barh(y, ratios, color=colors, alpha=0.88)
        axis.set_yticks(y, labels)
        axis.invert_yaxis()
        axis.set_xlabel("sau / trước (%)")
        axis.set_title(title)
        axis.grid(True, axis="x", alpha=0.3)
        axis.set_xlim(0.0, max(115.0, max(ratios) + 10.0))
        for bar, ratio in zip(bars, ratios):
            axis.text(
                ratio + 1.0, bar.get_y() + bar.get_height() / 2,
                f"{ratio:.1f}%", va="center", fontsize=8,
            )
    figure.suptitle(
        "Rà soát 26/07/2026: 100% là giá trị trước; xanh là giảm, đỏ là tăng",
        fontsize=14, weight="bold",
    )
    figure.tight_layout(rect=(0, 0, 1, 0.92))
    figure.savefig(
        ASSETS / "figure_15_current_audit_comparison.png", bbox_inches="tight"
    )
    plt.close(figure)


def save_current_figure_manifest(validation, current_audit):
    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    generated_figures = (
        "figure_10_speed_trace.png",
        "figure_14_all_map_error.png",
        "figure_15_current_audit_comparison.png",
        *(f"map_{name}.png" for name in ENVIRONMENTS),
    )
    manifest = {
        "version": "current_full_audit_20260726",
        "validation_sources": {
            name: {
                "path": str(path.relative_to(ROOT)),
                "sha256": digest(path),
                "execution_time_s": validation[name]["execution_time_s"],
                "tracking_rmse_m": validation[name]["tracking_rmse_m"],
                "tracking_max_error_m": validation[name]["tracking_max_error_m"],
                "estimated_tracking_rmse_m": validation[name][
                    "estimated_tracking_rmse_m"
                ],
                "localization_position_error_p95_m": validation[name][
                    "localization_position_error_p95_m"
                ],
            }
            for name, path in VALIDATION_PATHS.items()
        },
        "before_after_sources": {
            name: {
                "path": str(path.relative_to(ROOT)),
                "sha256": digest(path),
            }
            for name, path in CURRENT_AUDIT_PATHS.items()
        },
        "generated_figures": generated_figures,
        "figure_sha256": {
            filename: digest(ASSETS / filename)
            for filename in generated_figures
        },
    }
    (ASSETS / "current_figure_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def table(headers, rows, class_name=""):
    heading = "".join(f"<th>{html.escape(str(value))}</th>" for value in headers)
    body = "".join(
        "<tr>" + "".join(f"<td>{value}</td>" for value in row) + "</tr>"
        for row in rows
    )
    return (
        f'<table class="{class_name}"><thead><tr>{heading}</tr></thead>'
        f"<tbody>{body}</tbody></table>"
    )


def figure(name, caption, class_name=""):
    return (
        f'<figure class="{class_name}"><img src="bao_cao_toan_dien_assets/{name}" '
        f'alt="{html.escape(caption)}"><figcaption>{caption}</figcaption></figure>'
    )


SYMBOLS = [
    ("x, y", "m", "Tọa độ phẳng của tâm robot hoặc một điểm trên đường."),
    ("θ hoặc ψ", "rad", "Góc hướng (yaw) của robot/đường so với trục x."),
    ("φ", "rad", "Góc đổi hướng tại một đỉnh polyline."),
    ("v", "m/s", "Vận tốc tuyến tính của tâm robot."),
    ("ω", "rad/s", "Vận tốc góc quanh trục z."),
    ("vL, vR", "m/s", "Vận tốc dài tại bánh trái và bánh phải."),
    ("L", "m", "Khoảng cách giữa hai mặt phẳng tâm vệt lăn; hiện là 0,2548 m."),
    ("κ", "1/m", "Độ cong; κ=ω/v khi v khác 0."),
    ("R", "m", "Bán kính thiết kế dùng để suy ra trim; không phải cung tròn đầu ra."),
    ("d", "m", "Khoảng cắt từ đỉnh góc đến điểm vào/ra transition."),
    ("q", "m", "Khoảng cách giữa các control point; q=0,35d."),
    ("P0…P5", "m", "Sáu điểm điều khiển của Bézier bậc năm."),
    ("u", "–", "Tham số Bézier chạy từ 0 đến 1."),
    ("B(u)", "m", "Vị trí trên đường Bézier tại tham số u."),
    ("B′, B″", "–", "Đạo hàm bậc một và hai theo u."),
    ("Eκ", "1/m", "Năng lượng độ cong: tích phân κ² theo chiều dài đường."),
    ("s", "m", "Tiến độ hoặc chiều dài cung tích lũy dọc đường."),
    ("Δs", "m", "Khoảng cách giữa hai mẫu liên tiếp."),
    ("Δt", "s", "Khoảng thời gian giữa hai mẫu."),
    ("a", "m/s²", "Gia tốc tuyến tính."),
    ("ay", "m/s²", "Gia tốc ngang xấp xỉ v²|κ|."),
    ("α", "rad/s²", "Gia tốc góc."),
    ("j", "m/s³", "Jerk: tốc độ thay đổi của gia tốc."),
    ("v̄", "m/s", "Trần vận tốc an toàn tại một điểm."),
    ("n", "rpm", "Tốc độ quay: số vòng trong một phút."),
    ("τ", "N·m", "Mô-men xoắn tại trục."),
    ("G", "–", "Tỷ số truyền hộp số; GA25 hiện dùng 45:1."),
    ("U", "V", "Điện áp giữa hai điểm."),
    ("I", "A", "Dòng điện chạy qua một nhánh."),
    ("Q", "Ah", "Dung lượng điện tích danh nghĩa của bộ pin."),
    ("E", "Wh", "Năng lượng điện danh nghĩa, xấp xỉ U·Q."),
    ("C-rate", "1/h", "Hệ số dòng xả so với dung lượng; 5C của 2,6 Ah là 13 A."),
    ("J", "–", "Cost/hàm mục tiêu cần tối thiểu hóa."),
    ("e_xy", "m", "Sai số ngang từ robot đến đường."),
    ("e_ψ", "rad", "Sai số giữa hướng robot và tiếp tuyến đường."),
    ("RMSE", "m", "Căn trung bình bình phương sai số bám."),
    ("P95", "m", "Phân vị 95%; 95% mẫu không lớn hơn giá trị này."),
    ("N", "–", "Số góc của đường."),
    ("K", "–", "Số trạng thái ứng viên trung bình tại mỗi góc."),
    ("λ", "–", "Trọng số của một thành phần trong hàm mục tiêu."),
    ("∞", "–", "Vô cùng; trong code thường biểu thị ứng viên không hợp lệ."),
    ("ε", "–", "Ngưỡng rất nhỏ để so sánh số thực và tránh chia cho 0."),
]

GLOSSARY = [
    ("4S4P", "Bốn cell nối tiếp trong mỗi nhánh và bốn nhánh song song; tổng 16 cell."),
    ("Action", "Cơ chế ROS cho tác vụ kéo dài, có goal, feedback, result và khả năng hủy."),
    ("Adaptive", "Thích nghi: tham số được chọn theo từng dữ liệu đầu vào thay vì một giá trị cố định."),
    ("AMCL", "Adaptive Monte Carlo Localization: định vị robot trên map bằng particle filter."),
    ("Baseline", "Phương pháp đối chứng dùng làm mốc so sánh."),
    ("Behavior Tree (BT)", "Cây hành vi điều phối tuần tự/rẽ nhánh các tác vụ Nav2."),
    ("Bézier curve", "Đường cong đa thức được xác định bởi các điểm điều khiển."),
    ("BMS", "Battery Management System: mạch bảo vệ/cân bằng cell; chưa chốt model trong dự án."),
    ("Buck converter", "Bộ nguồn hạ áp DC–DC; cần để hạ pack 4S tối đa 16,8 V xuống rail motor 12 V."),
    ("Camera-ready", "Bản cuối để xuất bản; không liên quan đến báo cáo giáo trình này."),
    ("Clearance", "Khoảng hở nhỏ nhất từ footprint robot đến vật cản."),
    ("Closed loop", "Vòng kín: lệnh được cập nhật bằng phản hồi trạng thái robot thực/mô phỏng."),
    ("Coarse-to-fine", "Tìm kiếm thô đến tinh: lấy mẫu rộng trước rồi chia nhỏ vùng đáng chú ý."),
    ("Collision", "Va chạm hoặc cấu hình hình học giao với vật cản."),
    ("Controller", "Bộ điều khiển biến đường tham chiếu thành lệnh vận tốc."),
    ("Cost", "Đại lượng số biểu thị mức xấu/rủi ro; nhỏ hơn thường tốt hơn."),
    ("Costmap", "Lưới chi phí 2D của Nav2; ô càng đắt càng gần/nguy hiểm với vật cản."),
    ("Curvature", "Độ cong; đo mức hướng tiếp tuyến thay đổi trên một mét đường."),
    ("Deterministic", "Xác định: cùng đầu vào và tham số cho cùng kết quả."),
    ("Differential drive", "Kiểu xe hai bánh chủ động độc lập; quay nhờ chênh lệch tốc độ bánh."),
    ("Dynamic Programming (DP)", "Quy hoạch động: ghép nghiệm tối ưu từ các trạng thái con và quan hệ tương thích."),
    ("Encoder", "Cảm biến tạo xung theo chuyển động trục; phải biết PPR và cách giải mã trước khi đổi xung ra quãng đường."),
    ("Fallback", "Phương án dự phòng an toàn khi phương án ưu tiên không hợp lệ."),
    ("Footprint", "Đa giác chiếu bằng đại diện toàn thân robot trong costmap."),
    ("Frame", "Hệ tọa độ có gốc và hướng riêng, ví dụ map, odom, base_link."),
    ("G1 continuity", "Liên tục vị trí và hướng tiếp tuyến, nhưng độ cong có thể nhảy."),
    ("G2 continuity", "Liên tục vị trí, hướng tiếp tuyến và độ cong theo hình học."),
    ("Gazebo", "Trình mô phỏng vật lý, cảm biến và thế giới 3D."),
    ("Global planner", "Thuật toán tìm đường từ start đến goal trên global costmap."),
    ("Goal checker", "Thành phần quyết định robot đã đến đích và dừng hay chưa."),
    ("Ground truth", "Pose lấy trực tiếp từ vật lý Gazebo, không qua odometry/AMCL."),
    ("Hard constraint", "Ràng buộc bắt buộc; vi phạm là loại ứng viên, không được metric khác bù."),
    ("Hash / SHA-256", "Dấu vân tay dữ liệu dùng xác nhận các phương pháp nhận cùng raw path."),
    ("Hybrid", "Lai: chọn giữa nhiều nhánh thuật toán theo luật định trước."),
    ("Inflation layer", "Lớp costmap lan chi phí ra quanh vật cản để phản ánh độ gần."),
    ("Interpolation", "Nội suy giá trị ở giữa các mẫu đã biết."),
    ("Jerk", "Đạo hàm của gia tốc; jerk lớn tạo thay đổi lệnh đột ngột."),
    ("Lifecycle node", "Node ROS có các trạng thái configure/activate/deactivate/cleanup."),
    ("Locked rotor / stall", "Trạng thái trục motor bị khóa; dòng và nhiệt tăng cao, không phải chế độ vận hành."),
    ("Localization", "Định vị: ước lượng robot đang ở đâu trong map."),
    ("Lookahead / carrot", "Điểm nhìn trước trên đường mà Pure Pursuit hướng tới."),
    ("Map server", "Node phát occupancy map cho Nav2."),
    ("Marker Pivot", "Hai pose cùng vị trí nhưng khác yaw, biểu diễn lệnh dừng và quay tại chỗ."),
    ("Nav2", "Navigation2: tập node điều hướng chuẩn của ROS 2."),
    ("Node", "Một tiến trình/thành phần ROS có publisher, subscriber, service hoặc action."),
    ("Nonholonomic", "Ràng buộc chuyển động khiến xe không thể trượt ngang tức thời."),
    ("Occupancy grid", "Ảnh lưới biểu diễn ô trống, ô bị chiếm và ô chưa biết."),
    ("Odometry", "Ước lượng chuyển động tương đối tích lũy từ bánh/IMU."),
    ("Open loop", "Vòng hở: không dùng phản hồi thực để sửa lệnh đang phát."),
    ("Path", "Chuỗi pose hình học; chưa nhất thiết có thời gian/vận tốc."),
    ("Planner", "Bộ tìm đường hình học từ điểm đầu đến điểm cuối."),
    ("Plugin", "Thành phần có thể nạp động vào server mà không sửa lõi Nav2."),
    ("Polyline", "Đường gấp khúc gồm nhiều đoạn thẳng nối tiếp."),
    ("Pose", "Vị trí cộng hướng của một vật thể."),
    ("Projection", "Phép tìm điểm tương ứng gần nhất của robot trên đường tham chiếu."),
    ("QoS", "Quality of Service: luật độ tin cậy, lưu mẫu và độ bền của topic ROS."),
    ("Quaternion", "Biểu diễn hướng 3D bằng bốn số; tránh singularity của Euler angle."),
    ("Raw path", "Đường đầu ra planner trước mọi bước làm mượt."),
    ("Regulated Pure Pursuit (RPP)", "Pure Pursuit có thêm giảm tốc theo cong, cost và va chạm dự báo."),
    ("Repetition", "Lần lặp thí nghiệm độc lập cùng cấu hình."),
    ("Rated load", "Điểm làm việc có tải định mức do nhà sản xuất công bố."),
    ("Resampling", "Lấy lại mẫu đường với khoảng cách đều hơn."),
    ("Robot Operating System 2", "Middleware và hệ sinh thái giao tiếp cho robot; không phải hệ điều hành kernel."),
    ("RViz2", "Công cụ trực quan hóa map, TF, robot, sensor, path và panel điều khiển."),
    ("Safety gate", "Cổng quyết định chỉ cho một nhánh đi qua khi thỏa điều kiện an toàn."),
    ("Scenario", "Một trường hợp thử gồm map, start, goal và cấu hình liên quan."),
    ("SDF", "Simulation Description Format: mô tả world, model, collision, sensor và plugin Gazebo."),
    ("Servo", "Điều khiển phản hồi cục bộ đưa sai số vị trí/hướng về gần 0."),
    ("Smoother", "Bộ hậu xử lý làm đường planner bớt gấp khúc và khả thi hơn."),
    ("Smoothstep", "Hàm 3r²−2r³ chuyển giá trị trơn với slope bằng 0 ở hai đầu."),
    ("Speed cap", "Giới hạn trên của vận tốc tại một điểm; controller có thể chạy chậm hơn."),
    ("No-load speed", "Tốc độ khi motor gần như không mang tải; không phải tốc độ xe an toàn dưới tải."),
    ("S-curve", "Profile gia tốc có các đoạn jerk hữu hạn, tránh bước nhảy gia tốc."),
    ("Swept footprint", "Hợp của footprint khi robot quét dọc toàn đoạn chuyển động."),
    ("Telemetry", "Dữ liệu trạng thái/giới hạn/metric phát ra để theo dõi và debug."),
    ("TF", "Thư viện/cây biến đổi hệ tọa độ của ROS."),
    ("Time parameterization", "Gắn vận tốc và thời gian vào đường hình học dưới các giới hạn động học."),
    ("Topic", "Kênh publish–subscribe truyền luồng message bất đồng bộ."),
    ("Trajectory", "Đường có thêm quy luật thời gian, vận tốc và thường cả gia tốc."),
    ("Transition", "Đoạn chuyển tiếp thay góc gấp bằng chuyển động cong hoặc Pivot."),
    ("URDF", "Unified Robot Description Format: mô tả link/joint robot cho ROS."),
    ("Voxel layer", "Lớp costmap 3D cục bộ tổng hợp dữ liệu sensor theo voxel."),
    ("Yaw", "Góc quay quanh trục z, tức hướng nhìn của robot trên mặt phẳng."),
]


def symbol_table():
    return table(("Ký hiệu", "Đơn vị", "Ý nghĩa"), SYMBOLS, "compact")


def glossary_table():
    return table(
        ("Thuật ngữ tiếng Anh", "Giải nghĩa trong dự án"),
        GLOSSARY,
        "compact",
    )


def parameters_table():
    rows = [
        ("Kích thước footprint", "0,44 × 0,34 m", "Bao thân robot trong costmap"),
        ("Khoảng cách vệt lăn L", "0,2548 m", "Công thức động học và giới hạn bánh"),
        ("Đường kính bánh", "0,085 m", "Mô hình Gazebo"),
        ("vmax", "0,30 m/s", "Vận tốc tuyến tính tối đa"),
        ("ωmax", "0,80 rad/s", "Vận tốc góc tối đa"),
        ("vw,max", "0,36 m/s", "Vận tốc dài tối đa của mỗi bánh"),
        ("ay,max", "0,18 m/s²", "Giới hạn gia tốc ngang trong cong"),
        ("atăng", "0,35 m/s²", "Gia tốc tiến tối đa"),
        ("agiảm", "0,45 m/s²", "Độ lớn giảm tốc tối đa"),
        ("αmax", "1,20 rad/s²", "Gia tốc góc tối đa"),
        ("jmax", "0,90 m/s³", "Jerk tuyến tính tối đa"),
        ("Khoảng mẫu transition", "0,02 m", "Kiểm tra hình học Bézier"),
        ("Khoảng output", "0,05 m", "Resample đường đầu ra"),
        ("R thích nghi", "0,10…1,50 m", "Miền bán kính thiết kế"),
        ("Đánh giá mỗi góc", "6 đầu, tối đa 20", "Ngân sách coarse-to-fine"),
        ("Ứng viên giữ lại", "5/góc + Pivot", "Trạng thái cho DP"),
        ("Footprint cost tối đa", "200", "Gate center cost trước kiểm tra footprint"),
    ]
    return table(("Tham số", "Giá trị", "Vai trò"), rows, "compact")


def hardware_tables(profile):
    drive = profile["drive"]
    power = profile["power"]
    cell = power["cell"]
    pack = power["pack"]
    motor_rows = [
        ("Số lượng / model", f"{drive['motor_count']} × {drive['motor_model']}", "Hai bánh chủ động độc lập"),
        ("Điện áp định mức", f"{fmt(drive['rated_voltage_v'], 1)} V DC", "Rail motor bắt buộc được điều áp"),
        ("Tỷ số truyền", f"{fmt(drive['gearbox_ratio'], 0)}:1", "Giảm tốc armature xuống trục ra"),
        ("Tốc độ armature", f"{fmt(drive['armature_speed_rpm'], 0)} rpm", "Giá trị kỹ thuật được cung cấp"),
        ("Không tải", f"{fmt(drive['nominal_output_rpm'], 0)} ± {fmt(drive['no_load_output_speed_tolerance_percent'], 0)}% rpm; {fmt(1000*drive['no_load_current_per_motor_a'], 0)} mA", "Không dùng làm tốc độ chạy liên tục dưới tải"),
        ("Tải định mức", f"{fmt(drive['rated_load_output_rpm'], 0)} ± {fmt(drive['rated_load_output_speed_tolerance_percent'], 0)}% rpm; {fmt(1000*drive['rated_load_current_per_motor_a'], 0)} mA", "Vùng làm việc danh nghĩa"),
        ("Mô-men định mức", f"{fmt(drive['rated_output_torque_kgf_cm'], 1)} kgf·cm = {fmt(drive['rated_output_torque_nm'], 7)} N·m", "Mô-men vận hành tham chiếu"),
        ("Stall", f"{fmt(drive['stall_current_per_motor_a'], 1)} A; {fmt(drive['stall_output_torque_kgf_cm'], 1)} kgf·cm = {fmt(drive['stall_output_torque_nm'], 7)} N·m", "Giới hạn tuyệt đối; không được giữ rotor khóa"),
        ("Kích thước thân motor", f"{fmt(1000*drive['motor_body_length_m'], 0)} × Ø{fmt(1000*drive['motor_body_diameter_m'], 0)} mm", "Visual GA25 trong URDF/SDF"),
        ("Vận tốc bánh lý thuyết", f"{fmt(drive['theoretical_rated_load_linear_speed_mps'], 3)} m/s rated; {fmt(drive['theoretical_no_load_linear_speed_mps'], 3)} m/s no-load", "Controller giới hạn bánh ở 0,36 m/s"),
    ]
    battery_rows = [
        ("Cấu hình", power["topology"], f"{power['series_cell_count']} nối tiếp × {power['parallel_cell_count']} song song = {power['total_cell_count']} cell"),
        ("Mỗi cell", f"{cell['type']}; {fmt(cell['capacity_mah'], 0)} mAh; {fmt(cell['discharge_rate_c'], 0)}C / {fmt(cell['stated_discharge_current_a'], 0)} A", "Thông số người dùng cung cấp"),
        ("Điện áp pack", f"{fmt(pack['nominal_voltage_assumption_v'], 1)} V nominal; {fmt(pack['full_voltage_assumption_v'], 1)} V full", "Suy ra từ giả định Li-ion 3,7/4,2 V mỗi cell"),
        ("Dung lượng pack", f"{fmt(pack['capacity_ah'], 1)} Ah", "Song song cộng dung lượng: 4 × 2,6 Ah"),
        ("Năng lượng danh nghĩa", f"{fmt(pack['nominal_energy_assumption_wh'], 2)} Wh", "14,8 V × 10,4 Ah"),
        ("Dòng xả lý thuyết", f"{fmt(pack['theoretical_continuous_discharge_current_a'], 0)} A", "4 nhánh × 13 A; không phải rating tự động của toàn xe"),
        ("Rail motor", f"{fmt(power['motor_rail']['regulated_voltage_v'], 1)} V", "Cần buck 12 V vì pack đầy là 16,8 V"),
        ("Các giá trị còn thiếu", "BMS, fuse, buck, driver, dây/giắc, nhiệt, cutoff", "Phải chốt theo phần tử có rating thấp nhất"),
    ]
    return (
        table(("Thông số GA25", "Giá trị", "Ý nghĩa và lý do sử dụng"), motor_rows, "compact"),
        table(("Thông số nguồn", "Giá trị", "Ý nghĩa và lý do sử dụng"), battery_rows, "compact"),
    )


def scenarios_table(environment):
    rows = []
    for scenario in load_scenarios(environment):
        start = scenario["start"]
        goal = scenario["goal"]
        rows.append(
            (
                f"<code>{html.escape(scenario['name'])}</code>",
                f"({start[0]:.2f}; {start[1]:.2f})",
                f"({goal[0]:.2f}; {goal[1]:.2f})",
            )
        )
    return table(("Scenario", "Start (m)", "Goal (m)"), rows, "compact")


def validation_table(validation):
    rows = []
    for environment in ENVIRONMENTS:
        row = validation[environment]
        rows.append(
            (
                ENV_LABEL[environment],
                html.escape(str(row.get("scenario", ""))),
                fmt(as_float(row.get("execution_time_s")), 2),
                fmt(100 * as_float(row.get("tracking_rmse_m")), 2),
                fmt(100 * as_float(row.get("tracking_max_error_m")), 2),
                fmt(100 * as_float(row.get("estimated_tracking_rmse_m")), 2),
                fmt(100 * as_float(row.get("localization_position_error_p95_m")), 2),
                fmt(100 * as_float(row.get("planned_footprint_clearance_min_m")), 2),
            )
        )
    return table(
        (
            "Map", "Scenario", "t (s)", "GT RMSE (cm)", "GT max (cm)",
            "RMSE điều khiển (cm)", "định vị P95 (cm)", "clearance (cm)",
        ),
        rows,
        "tiny",
    )


def geometry_overall_table(summary):
    rows = []
    for method, label in METHOD_LABEL.items():
        value = summary["geometry_overall"][method]
        rows.append(
            (
                label,
                f"{value['successes']}/{value['attempts']}",
                fmt(value["energy"], 3),
                fmt(100 * value["clearance"], 2),
                fmt(value["length"], 3),
                fmt(100 * value["deviation"], 2),
                fmt(value["runtime_ms"], 2),
            )
        )
    return table(
        (
            "Phương pháp", "Thành công", "Eκ (1/m)", "clearance (cm)",
            "chiều dài (m)", "lệch raw (cm)", "runtime (ms)",
        ),
        rows,
        "tiny",
    )


def map_method_table(summary):
    rows = []
    for environment in ENVIRONMENTS:
        raw = summary["geometry_by_map_method"][f"{environment}/raw"]
        pivot = summary["geometry_by_map_method"][f"{environment}/pivot_g2"]
        hybrid = summary["geometry_by_map_method"][f"{environment}/adaptive_hybrid"]
        rows.append(
            (
                ENV_LABEL[environment],
                fmt(raw["energy"], 2),
                fmt(pivot["energy"], 2),
                fmt(percentage_reduction(pivot["energy"], raw["energy"]), 1) + "%",
                fmt(hybrid["energy"], 2),
                fmt(100 * hybrid["clearance"], 2),
                f"{hybrid['successes']}/{hybrid['attempts']}",
            )
        )
    return table(
        (
            "Map", "Raw Eκ", "Pivot A Eκ", "Pivot giảm", "Hybrid A Eκ",
            "Hybrid clearance (cm)", "Hybrid OK",
        ),
        rows,
        "tiny",
    )


def primary_execution_table(execution):
    rows = []
    primary = [
        row for row in execution
        if row["environment"] == "research_warehouse"
        and row["scenario"] == "lower_left_diagonal"
        and row["planner"] == "ThetaStar"
        and not row["configuration_sha256"]
        and math.isclose(as_float(row["fixed_speed_limit_mps"]), 0.0, abs_tol=1e-12)
    ]
    for row in sorted(primary, key=lambda value: list(METHOD_LABEL).index(value["method"])):
        rows.append(
            (
                METHOD_LABEL[row["method"]],
                "Có" if row["success"] == "True" else "Không",
                fmt(row["execution_time_s"], 2),
                fmt(100 * as_float(row["tracking_rmse_m"]), 2),
                fmt(100 * as_float(row["tracking_max_error_m"]), 2),
                fmt(100 * as_float(row["curve_exit_tracking_rmse_m"]), 2),
                fmt(as_float(row["actual_max_linear_mps"]), 3),
                fmt(as_float(row["adaptive_speed_nominal_p95_abs_jerk_mps3"]), 3),
            )
        )
    return table(
        (
            "Phương pháp", "OK", "t (s)", "GT RMSE (cm)", "GT max (cm)",
            "exit RMSE (cm)", "vmax (m/s)", "jerk P95",
        ),
        rows,
        "tiny",
    )


def planner_table(execution):
    rows = []
    planner_rows = [
        row for row in execution
        if row["environment"] == "research_warehouse"
        and row["scenario"] == "lower_left_diagonal"
        and row["method"] == "adaptive_hybrid"
        and row["configuration_sha256"]
        and math.isclose(as_float(row["fixed_speed_limit_mps"]), 0.0, abs_tol=1e-12)
    ]
    for row in sorted(planner_rows, key=lambda value: value["planner"]):
        rows.append(
            (
                html.escape(row["planner"]),
                "Có" if row["success"] == "True" else "Không",
                fmt(row["execution_time_s"], 2),
                fmt(100 * as_float(row["tracking_rmse_m"]), 2),
                fmt(100 * as_float(row["estimated_tracking_rmse_m"]), 2),
                fmt(100 * as_float(row["curve_exit_tracking_rmse_m"]), 2),
            )
        )
    return table(
        ("Planner", "OK", "t (s)", "GT RMSE (cm)", "RMSE điều khiển (cm)", "exit (cm)"),
        rows,
        "compact",
    )


def map_sections(validation):
    output = []
    for index, environment in enumerate(ENVIRONMENTS, start=1):
        row = validation[environment]
        output.append(
            f"""
<h3>15.{index} {ENV_LABEL[environment]} — <code>{environment}</code></h3>
<p>{ENV_DESCRIPTIONS[environment]}</p>
{figure(
    f"map_{environment}.png",
    f"Hình 15.{index}. Bên trái là occupancy grid RViz2 cùng đường được chọn và "
    "ground truth; bên phải là phối cảnh tái dựng trực tiếp từ các visual box "
    "trong SDF Gazebo.",
)}
<p><b>Cách đọc số liệu đại diện.</b> Scenario
<code>{html.escape(str(row.get("scenario", "")))}</code> dùng ThetaStar +
Pivot–G2 thích nghi + vận tốc thích nghi; thời gian
{fmt(row.get("execution_time_s"), 2)} s, ground-truth RMSE
{fmt(100 * as_float(row.get("tracking_rmse_m")), 2)} cm, sai số cực đại
{fmt(100 * as_float(row.get("tracking_max_error_m")), 2)} cm, clearance kế hoạch
{fmt(100 * as_float(row.get("planned_footprint_clearance_min_m")), 2)} cm.
Sai số mà controller nhìn thấy là
{fmt(100 * as_float(row.get("estimated_tracking_rmse_m")), 2)} cm; chênh lệch
với ground truth cho thấy ảnh hưởng của định vị.</p>
<p><b>Các trường hợp thử có sẵn:</b></p>
{scenarios_table(environment)}
"""
        )
    return "\n".join(output)


def tutorial_html(summary, execution, validation, current_audit, hardware):
    raw = summary["geometry_overall"]["raw"]
    pivot = summary["geometry_overall"]["pivot_g2"]
    hybrid = summary["geometry_overall"]["adaptive_hybrid"]
    motor_table, battery_table = hardware_tables(hardware)
    lower_before = current_audit["lower_before"]
    lower_after = current_audit["lower_after"]
    rack_before = current_audit["rack_before"]
    rack_after = current_audit["rack_after"]
    narrow_before = current_audit["narrow_before_angular_braking"]
    narrow_after = current_audit["narrow_after"]
    return f"""<!doctype html>
<html lang="vi"><head><meta charset="utf-8">
<title>Báo cáo toàn diện Adaptive Hybrid Pivot–G2</title>
<style>
@page {{ size: A4; margin: 18mm 18mm 18mm 18mm; }}
body {{ font-family: "Times New Roman", serif; font-size: 10.5pt;
  line-height: 1.32; color: #172033; margin: 0; text-align: left; }}
.title {{ text-align:center; font-size: 24pt; font-weight:bold; line-height:1.16;
  color:#102a43; margin-top:25mm; }}
.subtitle {{ text-align:center; font-size:13pt; color:#475569; margin:8pt 0; }}
.meta {{ text-align:center; font-size:10pt; color:#64748b; }}
h1 {{ text-align:center; font-size:22pt; }}
h2 {{ font-size:16pt; color:#153e75; border-bottom:1pt solid #93c5fd;
  padding-bottom:3pt; margin-top:14pt; }}
h3 {{ font-size:13pt; color:#1e3a5f; margin-top:10pt; }}
h4 {{ font-size:11pt; color:#334155; margin-bottom:2pt; }}
p {{ margin:4pt 0; text-align:left; }}
li {{ margin-bottom:3pt; }}
.lead {{ font-size:12pt; line-height:1.45; background:#eff6ff;
  border-left:4pt solid #2563eb; padding:9pt; }}
.mine {{ background:#fdf2f8; border:1pt solid #f0abfc; padding:9pt;
  margin:8pt 0; }}
.warning {{ background:#fff7ed; border-left:4pt solid #f97316; padding:8pt; }}
.definition {{ background:#f8fafc; border-left:4pt solid #64748b; padding:7pt; }}
.eq {{ text-align:center; font-size:11pt; font-style:italic; background:#f8fafc;
  border:0.5pt solid #cbd5e1; padding:7pt; margin:7pt 8mm; }}
figure {{ margin:8pt 0; break-inside:avoid; text-align:center; }}
figure img {{ max-width:100%; max-height:170mm; object-fit:contain; }}
figure.compact img {{ max-height:105mm; }}
figcaption {{ font-size:9pt; color:#475569; text-align:center; margin-top:3pt; }}
table {{ border-collapse:collapse; width:100%; margin:7pt 0; break-inside:auto; }}
th, td {{ border:0.5pt solid #94a3b8; padding:3.5pt; vertical-align:top; }}
th {{ background:#dbeafe; font-weight:bold; text-align:left; }}
tr {{ break-inside:avoid; }}
.compact {{ font-size:9pt; }}
.tiny {{ font-size:7.5pt; }}
code {{ font-family:"DejaVu Sans Mono", monospace; font-size:8.5pt;
  background:#f1f5f9; }}
.toc td:first-child {{ width:12%; text-align:center; font-weight:bold; }}
.page-break {{ break-before:page; page-break-before:always; }}
.center {{ text-align:center; }}
</style></head><body>

<div class="title">BÁO CÁO TOÀN DIỆN<br>
ADAPTIVE HYBRID PIVOT–G2<br>
CHO ROBOT VI SAI TRONG ROS 2 / NAV2</div>
<div class="subtitle">Từ kiến thức nhập môn, phương trình và kiến trúc phần mềm<br>
đến mô phỏng Gazebo, RViz2, bảy bản đồ và so sánh thực nghiệm</div>
<div class="meta">Phiên bản thuật toán hiện tại — tự sinh từ source và dữ liệu ngày 26/07/2026</div>
<div class="meta">Workspace: <code>/home/linh-pham/agv_nav2_research_ws</code></div>
{figure("figure_01_learning_roadmap.png", "Lộ trình đọc báo cáo.", "compact")}
<p class="center"><b>Đối tượng:</b> người mới bắt đầu về robot di động, ROS 2 và
lập kế hoạch đường đi. Đây là tài liệu học và vận hành dự án, không phải bài báo
hội nghị và không có tài liệu supplement tách rời.</p>

<div class="page-break"></div>
<h2>Mục lục và cách đọc</h2>
{table(
    ("Phần", "Nội dung", "Nên đọc khi"),
    [
        ("1–5", "Bài toán, ký hiệu, robot vi sai, ROS 2/Nav2, Gazebo/RViz2", "Bạn mới bắt đầu"),
        ("6–14", "Toàn bộ thuật toán của bạn từ raw path đến cmd_vel", "Bạn muốn hiểu công thức/code"),
        ("15", "Từng map trong RViz2 và Gazebo, scenario và số liệu", "Bạn chuẩn bị chạy mô phỏng"),
        ("16–18", "Đo lường, so sánh và kết luận phương pháp của bạn mạnh ở đâu", "Bạn phân tích kết quả"),
        ("19–22", "Giới hạn, cách chạy/debug, glossary và tài liệu tham khảo", "Bạn phát triển tiếp"),
    ],
    "toc",
)}
<p class="lead"><b>Tóm tắt một câu.</b> Phương pháp của bạn nhận đường gấp khúc
từ một global planner, tìm tại mỗi góc một transition Bézier bậc năm G2 hoặc một
lệnh Pivot, ghép các lựa chọn bằng quy hoạch động, kiểm tra toàn bộ footprint,
dùng cổng Hybrid để fallback an toàn, rồi điều khiển xe bằng RPP có profile vận
tốc hai chiều và phản hồi sai số bám.</p>

<h2>1. Bài toán mà dự án đang giải quyết</h2>
<h3>1.1 Vì sao “đã tìm được đường” chưa có nghĩa là “xe đi tốt”?</h3>
<p>Global planner thường làm việc trên một lưới ô vuông. Kết quả là một
<i>path</i> — chuỗi pose hình học — có thể chứa góc gấp, bậc thang theo grid hoặc
đoạn rất sát vật cản. Robot vi sai không thể dịch ngang, bánh xe có giới hạn tốc
độ và thân xe có kích thước. Vì vậy robot có thể:</p>
<ul>
<li>phải dừng rồi quay ở một góc quá gắt;</li>
<li>cắt góc và để thân xe chạm kệ dù tâm robot vẫn ở ô trống;</li>
<li>vào cong quá nhanh, tạo sai số ngang;</li>
<li>tăng tốc ngay sau cong khi yaw response chưa ổn định, làm chệch ray;</li>
<li>chọn nhầm một nhánh gần đó khi đường tự cắt hoặc chạy song song;</li>
<li>đến gần goal nhưng quay sai hướng do pose ước lượng dịch chuyển.</li>
</ul>
<h3>1.2 Ba lớp của nghiệm hiện tại</h3>
<ol>
<li><b>Hình học:</b> điều kiện hóa raw polyline; tạo Pivot hoặc Bézier G2;
tìm d/R thích nghi; DP toàn đường; swept-footprint.</li>
<li><b>Động học–vận tốc:</b> trần vận tốc theo cong/bánh; gia tốc, giảm tốc,
gia tốc góc và jerk; truyền giới hạn về cả phía trước lẫn phía sau.</li>
<li><b>Điều khiển vòng kín:</b> Maneuver-aware RPP, projection có xét hướng,
giảm tốc theo sai số bám, thực thi Pivot và terminal servo.</li>
</ol>
<div class="mine"><b>PHƯƠNG PHÁP CỦA BẠN LÀ GÌ?</b><br>
Tên đầy đủ nên dùng trong tài liệu là <b>Adaptive Hybrid Pivot–G2 với
bidirectional jerk-limited speed envelope và maneuver-aware RPP</b>. “Pivot–G2”
là lõi hình học; “Adaptive” là tìm bán kính/trim riêng từng góc; “Hybrid” là cổng
chọn Simple/Pivot/Raw an toàn; profile vận tốc và controller là phần biến đường
hình học thành chuyển động bám được. Các phương pháp Raw, Simple,
Savitzky–Golay và Constrained là đối chứng, không phải đóng góp mới của bạn.</div>

<h2>2. Ký hiệu và toán học tối thiểu</h2>
<h3>2.1 Đơn vị SI và cách đọc ký hiệu</h3>
<p><code>m</code> là mét; <code>s</code> là giây; <code>rad</code> là radian.
Một vòng tròn là 2π rad. Dấu Δ đọc là “delta”, nghĩa là độ thay đổi.
Dấu κ đọc là “kappa”, nghĩa là độ cong. Dấu ∫ là tích phân — có thể hiểu như
cộng rất nhiều phần tử nhỏ. Dấu ′ là đạo hàm.</p>
{symbol_table()}
<h3>2.2 Pose, yaw và quaternion</h3>
<p>Trong bài toán phẳng, pose có thể hiểu là (x, y, ψ). ROS vẫn lưu orientation
bằng quaternion (qx, qy, qz, qw) để tổng quát cho 3D. Với robot không nghiêng,
quaternion yaw có dạng qz=sin(ψ/2), qw=cos(ψ/2). Hàm
<code>atan2(sin ψ, cos ψ)</code> đưa góc về khoảng −π…π, tránh lỗi nhảy từ
+179° sang −179°.</p>
<h3>2.3 Path khác trajectory</h3>
<p><b>Path</b> chỉ nói “đi qua đâu”. <b>Trajectory</b> còn nói “đến mỗi điểm lúc
nào, nhanh bao nhiêu”. Pivot–G2 trước hết tạo path; time-parameterization và
speed envelope biến nó thành quy luật vận tốc khả thi.</p>

<h2>3. Động học robot vi sai</h2>
{figure("figure_03_diff_drive.png", "Động học cơ bản của robot vi sai.")}
<div class="eq">v = (v<sub>R</sub> + v<sub>L</sub>)/2</div>
<div class="eq">ω = (v<sub>R</sub> − v<sub>L</sub>)/L</div>
<div class="eq">ẋ = v cos ψ;&nbsp;&nbsp; ẏ = v sin ψ;&nbsp;&nbsp; ψ̇ = ω</div>
<p>Nếu chuyển động theo một đường có độ cong κ thì ω=vκ. Từ đó:</p>
<div class="eq">v<sub>L</sub>=v(1−Lκ/2);&nbsp;&nbsp;
v<sub>R</sub>=v(1+Lκ/2)</div>
<p>Khi |Lκ/2|&gt;1, bánh trong phải đảo chiều. Lõi Pivot–G2 hiện loại transition
như vậy. Pivot là trường hợp đặc biệt v=0 nhưng ω≠0: hai bánh quay ngược chiều,
robot đổi yaw mà tâm gần như đứng yên.</p>
<p>Với κ không đổi và khác 0, bán kính quỹ đạo tâm là 1/|κ|. Tuy nhiên R trong
Pivot–G2 chỉ là <b>bán kính thiết kế</b> để tính trim; đường đầu ra là Bézier và
κ biến thiên, vì vậy không được gọi R là bán kính cung tròn thực.</p>

<h2>4. ROS 2, Nav2, TF, RViz2 và Gazebo</h2>
{figure("figure_02_ros_nav2_flow.png", "Luồng node và dữ liệu của dự án.")}
<h3>4.1 ROS 2 truyền dữ liệu như thế nào?</h3>
<ul>
<li><b>Topic:</b> luồng dữ liệu liên tục, ví dụ <code>/scan</code>,
<code>/odom</code>, <code>/cmd_vel</code>.</li>
<li><b>Service:</b> yêu cầu ngắn có request/response.</li>
<li><b>Action:</b> nhiệm vụ dài có feedback và hủy được, ví dụ
<code>ComputePathToPose</code>, <code>SmoothPath</code>,
<code>FollowPath</code>.</li>
<li><b>QoS:</b> quy định reliable/best-effort và có giữ mẫu cuối hay không.</li>
</ul>
<h3>4.2 Các frame quan trọng</h3>
{table(
    ("Frame", "Ai tạo", "Ý nghĩa"),
    [
        ("world", "Gazebo", "Hệ tuyệt đối của mô phỏng; nguồn ground truth."),
        ("map", "Map/AMCL", "Hệ bản đồ tĩnh mà goal và global path sử dụng."),
        ("odom", "Wheel odometry", "Hệ liên tục cục bộ nhưng có thể drift."),
        ("base_link", "Robot model", "Gốc thân xe ở cao độ tâm bánh."),
        ("base_footprint", "Fixed joint", "Hình chiếu base xuống mặt đất."),
        ("laser", "Robot model", "Hệ đo RPLIDAR."),
        ("imu_link", "Robot model", "Hệ đo BNO055."),
    ],
    "compact",
)}
<p>Chuỗi TF thường là <code>map → odom → base_link</code>. AMCL sửa quan hệ
map→odom; odometry cập nhật odom→base_link. Benchmark không dùng riêng TF để
tự chấm nó: pose vật lý Gazebo được ghi độc lập trong world rồi căn chỉnh về
map để tính ground-truth error.</p>
<h3>4.3 Nav2 làm gì?</h3>
<p>BT Navigator nhận goal; Planner Server sinh raw path; Smoother Server chạy
các plugin; Controller Server phát cmd_vel; costmap và Collision Monitor kiểm
tra vật cản. Các node dùng lifecycle để chỉ nhận nhiệm vụ khi đã active.</p>
<h3>4.4 RViz2 và Gazebo khác nhau</h3>
<p>RViz2 không mô phỏng vật lý. Nó hiển thị message ROS: map, TF, robot, sensor
và path. Gazebo mới tích phân lực/chuyển động, tạo va chạm và sensor. Hai cửa sổ
phải dùng đúng cùng world/map; environment manager trong dự án chịu trách nhiệm
dừng stack cũ và khởi động cặp mới.</p>
{figure(
    "figure_08_rviz_gazebo_ui.png",
    "Ảnh chạy thật của giao diện RViz2 và Gazebo trong dự án.",
)}
{figure(
    "figure_09_rviz_methods.png",
    "Panel RViz2: từng planner, map và phương pháp làm mượt được tách thành lựa chọn riêng.",
)}

<h2>5. Mô hình robot và cảm biến</h2>
{figure(
    "figure_07_robot_3d.png",
    "Mô hình CAD 3D được nạp trực tiếp trong Gazebo.",
)}
<h3>5.1 Hình học và khối lượng</h3>
<p>Chassis có envelope 440×340 mm, khối lượng 4,6 kg; hai bánh mỗi bánh 0,2 kg,
đường kính 85 mm. Tổng danh nghĩa khoảng 5,0 kg. Footprint Nav2 là hình chữ nhật
[(0,22;0,17), (0,22;−0,17), (−0,22;−0,17), (−0,22;0,17)]. Khoảng cách hai tâm
vệt lăn vật lý là 0,2548 m. Gazebo DiffDrive từng dùng 0,2809 m như hệ số hiệu
chuẩn tiếp xúc; phép fit bình phương tối thiểu có trọng số trên hai trace độc
lập ngày 26/07 đã cập nhật giá trị hiệu dụng thành <b>0,2834 m</b>, tương
đương multiplier 1,112245. Đây là hiệu chuẩn contact/odometry của mô phỏng,
không thay đổi kích thước động học vật lý 0,2548 m.</p>
<h3>5.2 Collision và visual</h3>
<p><i>Visual</i> là mesh nhìn thấy; <i>collision</i> là hình đơn giản dùng cho
vật lý. Thân dùng nhiều box để phủ đúng envelope nhưng chừa hốc bánh. Bốn bi đỡ
nhỏ có ma sát thấp. Tách visual/collision giúp mô phỏng ổn định mà vẫn hiển thị
CAD chi tiết.</p>
<h3>5.3 Cảm biến</h3>
{table(
    ("Cảm biến", "Cấu hình mô phỏng", "Topic", "Vai trò"),
    [
        ("RPLIDAR A1M8", "1440 tia, 5,5 Hz, 0,15–12 m, nhiễu 0,01 m", "<code>/scan</code>", "AMCL và local costmap"),
        ("BNO055 IMU", "100 Hz, nhiễu gyro/accel", "<code>/imu/data</code>", "Quan sát quay và tích hợp tương lai"),
        ("Wheel odometry", "30 Hz từ DiffDrive", "<code>/odom</code>", "odom→base_link"),
        ("Gazebo ground truth", "30 Hz, độc lập odometry", "<code>/ground_truth/odom</code>", "Chấm sai số vật lý"),
    ],
    "compact",
)}
<h3>5.4 Hai động cơ GA25 encoder 130 rpm</h3>
<p>Trước tiên, các giá trị nhà sản xuất được giữ nguyên trong profile phần
cứng. Sau đó mới đổi sang SI để kiểm tra với bánh xe. Cuối cùng, URDF/SDF chỉ
dùng no-load speed và stall torque làm <i>giới hạn tuyệt đối</i>; controller
vẫn chạy trong vùng bảo thủ hơn.</p>
<div class="eq">ω<sub>out</sub> = n·2π/60;&nbsp;&nbsp;
v<sub>wheel</sub> = r·ω<sub>out</sub></div>
<div class="eq">τ[N·m] = τ[kgf·cm]·0,0980665</div>
{motor_table}
<p class="warning"><b>Cảnh báo bản chất:</b> stall torque không phải mô-men
được phép giữ lâu. Khi rotor bị khóa, mỗi motor có thể hút 1,3 A và sinh nhiệt
nhanh. Phần mềm không thay thế fuse, giới hạn dòng driver và bảo vệ nhiệt.</p>

<h3>5.5 Bộ nguồn 16 cell 18650 mắc 4S4P</h3>
<p>Trong ký hiệu 4S4P, chữ <b>S</b> là <i>series</i> (nối tiếp, cộng điện áp);
chữ <b>P</b> là <i>parallel</i> (song song, cộng dung lượng và khả năng dòng).
Với giả định cell Li-ion chuẩn 3,7 V danh nghĩa, 4,2 V khi đầy:</p>
<div class="eq">U<sub>pack</sub>=4U<sub>cell</sub>;
&nbsp; Q<sub>pack</sub>=4Q<sub>cell</sub>;
&nbsp; E≈U<sub>nom</sub>Q</div>
{battery_table}
<p class="warning">Pack đầy 16,8 V <b>không được</b> cấp thẳng vào GA25 định
mức 12 V. Cần buck 12 V, BMS 4S, fuse, motor driver và dây dẫn có rating đã
xác minh. Giá trị 52 A chỉ là phép nhân lý tưởng của cell, không phải dòng xe
được phép khai thác.</p>

<h3>5.6 Encoder: điều gì chưa được phép giả định?</h3>
<p>Tên motor có chữ encoder nhưng dữ liệu được cung cấp chưa có PPR
(pulses per revolution), vị trí encoder trước/sau hộp số hoặc chế độ giải mã
x1/x2/x4. Vì vậy <code>ticks_per_rev</code>, rad/tick và m/tick được để
<code>null</code>. Điền một con số đoán sẽ trực tiếp làm sai odometry, sai yaw
và cuối cùng làm xe lệch khỏi đường sau cua.</p>

<h3>5.7 Các giới hạn đang dùng</h3>
{parameters_table()}

<h2>6. Map, costmap và footprint</h2>
<h3>6.1 Ba file tạo thành một môi trường</h3>
<ul>
<li><code>.pgm</code>: ảnh occupancy mà RViz2/Nav2 đọc.</li>
<li><code>.yaml</code>: resolution, origin và ngưỡng occupied/free.</li>
<li><code>.sdf</code>: sàn, tường, kệ, vật cản, ánh sáng và physics Gazebo.</li>
</ul>
<p>Cả bảy map có kích thước 12×8 m, resolution 0,05 m/ô và origin
(−6;−4;0). <code>occupied_thresh=0,65</code>, <code>free_thresh=0,25</code>.
PGM và SDF được sinh từ cùng hình học để vật cản RViz2 trùng vật cản Gazebo.</p>
<h3>6.2 Từ occupancy sang costmap</h3>
<p>Static layer nạp map; voxel layer đưa lidar vào local costmap; inflation
layer lan chi phí với radius 0,45 m. Center cost giúp đánh giá proximity nhưng
không thay cho collision: một tâm an toàn vẫn có thể khiến góc thân xe chạm kệ.</p>
{figure(
    "figure_04_costmap_footprint.png",
    "Lý do mọi shortcut và transition phải kiểm tra swept footprint.",
)}

<h2>7. Tổng quan chính xác về phương pháp của bạn</h2>
<div class="mine"><b>Đầu vào:</b> một <code>nav_msgs/Path</code> thô từ bất kỳ
planner đã cấu hình.<br><b>Đầu ra hình học:</b> một path an toàn chọn giữa
Simple, Adaptive Pivot–G2 và Raw.<br><b>Đầu ra điều khiển:</b> cmd_vel được giới
hạn theo đường phía trước, phản hồi bám và trạng thái robot.</div>
<ol>
<li>Neo chính xác cả start và goal liên tục vào path grid, bỏ pose trùng và
điều kiện hóa polyline trong hành lang an toàn.</li>
<li>Phát hiện góc bằng hướng hai đoạn và ngưỡng 5°.</li>
<li>Ở mỗi góc tạo trạng thái Pivot và tìm nhiều transition Bézier G2 an toàn.</li>
<li>Chấm cost ổn định; giữ tối đa năm G2 tốt nhất.</li>
<li>DP chọn một trạng thái mỗi góc, không cho hai trim chồng nhau.</li>
<li>Khâu toàn đường; kiểm tra lại endpoint, NaN, duplicate, động học và swept footprint.</li>
<li>Hybrid so sánh Simple/Pivot, fallback Raw nếu cần.</li>
<li>Controller căn yaw theo preview 0,30 m của path thật, xây speed envelope
hai chiều, bám path, thực thi marker Pivot và servo goal.</li>
</ol>
{figure(
    "figure_01_learning_roadmap.png",
    "Chuỗi xử lý nhìn từ góc độ người sử dụng.",
    "compact",
)}

<h2>8. Điều kiện hóa raw path</h2>
<h3>8.1 Vì sao cần bước này?</h3>
<p>Planner grid có thể tạo các đổi hướng trái–phải nhỏ liên tục. Nếu coi mỗi
đổi hướng là một góc thật, smoother sẽ sinh quá nhiều transition. Hàm
<code>condition_polyline</code> thử nối tắt một chuỗi điểm bằng chord.</p>
<h3>8.2 Điều kiện nhận shortcut</h3>
<ul>
<li>mọi điểm bị bỏ cách chord không quá ngưỡng deviation;</li>
<li>toàn chord qua predicate swept-footprint;</li>
<li>giữ nguyên thứ tự, start và goal;</li>
<li>oscillation shortcut chỉ áp dụng khi span/deviation/góc và số lần đổi dấu đạt ngưỡng.</li>
</ul>
<p>Trong benchmark chính, line-of-sight pruning bị tắt để Pivot–G2 và baseline
cùng nhận một raw path; nếu bật LOS chỉ cho Pivot thì lợi ích pruning sẽ bị
trộn với lợi ích smoothing.</p>

<h2>9. Hình học Pivot–G2</h2>
{figure(
    "figure_05_pivot_g2_geometry.png",
    "Một góc có thể được giải bằng transition G2 hoặc Pivot.",
)}
<h3>9.1 Tính góc đổi hướng</h3>
<p>Với vector đơn vị hướng vào e<sub>in</sub> và hướng ra
e<sub>out</sub>:</p>
<div class="eq">φ = atan2(cross(e<sub>in</sub>,e<sub>out</sub>),
dot(e<sub>in</sub>,e<sub>out</sub>))</div>
<p>Dấu φ cho biết rẽ trái/phải; |φ| là độ lớn. Góc dưới 5° không cần transition;
góc gần 180° nằm ngoài miền Bézier và thường phải Pivot.</p>
<h3>9.2 Trim và bán kính thiết kế</h3>
<div class="eq">d = R tan(|φ|/2)&nbsp;&nbsp;⇔&nbsp;&nbsp;
R = d / tan(|φ|/2)</div>
<p>Tối ưu d thuận lợi vì ràng buộc hai góc kề trở thành phép cộng tuyến tính.
Adaptive mode không dùng luật 45% làm điều kiện chính; fixed mode vẫn giữ
bank R=[0,20;0,30;…;1,50] m để ablation.</p>
<h3>9.3 Sáu control point</h3>
<div class="eq">q = 0,35d</div>
<div class="eq">P0=entry;&nbsp; P1=P0+q e<sub>in</sub>;&nbsp;
P2=P0+2q e<sub>in</sub></div>
<div class="eq">P5=exit;&nbsp; P4=P5−q e<sub>out</sub>;&nbsp;
P3=P5−2q e<sub>out</sub></div>
<h3>9.4 Phương trình Bézier bậc năm</h3>
<div class="eq">B(u)=Σ<sub>i=0…5</sub> C(5,i)(1−u)<sup>5−i</sup>
u<sup>i</sup>P<sub>i</sub>,&nbsp; 0≤u≤1</div>
<p>C(5,i) là hệ số nhị thức [1,5,10,10,5,1]. Do P0,P1,P2 thẳng hàng cách đều
và tương tự ở đầu kia, B″ tại hai endpoint bằng 0 theo hướng pháp tuyến; κ đầu
và cuối là 0. Transition nối với đoạn thẳng có vị trí, hướng và độ cong liên
tục theo hình học G2.</p>
<h3>9.5 Đạo hàm, độ cong và năng lượng</h3>
<div class="eq">κ(u) =
(x′y″−y′x″)/(x′²+y′²)<sup>3/2</sup></div>
<div class="eq">E<sub>κ</sub> = ∫ κ(s)<sup>2</sup> ds
≈ Σ 0,5(κ<sub>i−1</sub><sup>2</sup>+κ<sub>i</sub><sup>2</sup>)Δs</div>
<p>Eκ phạt đường cong gắt và kéo dài. Nó không trực tiếp là jerk hay thời gian;
vì vậy ứng viên còn phải qua time-parameterization và chạy kín.</p>

<h2>10. Ràng buộc cứng của một ứng viên</h2>
<p>Một ứng viên bị loại ngay, không được metric tốt khác “bù”, nếu:</p>
<ul>
<li>có NaN/Inf, đạo hàm suy biến hoặc đoạn chuyển động dài 0;</li>
<li>đổi dấu κ ngoài ý muốn;</li>
<li>bánh trong phải quay ngược trong một transition được định nghĩa là chạy tiến;</li>
<li>không có vận tốc dương thỏa vmax, ωmax, vw,max và ay,max;</li>
<li>time-parameterization không hội tụ dưới giới hạn gia tốc góc;</li>
<li>center cost vượt ngưỡng hoặc swept footprint chạm lethal/unknown không cho phép;</li>
<li>trim chồng với góc kế tiếp;</li>
<li>full stitched path không giữ start/goal hoặc thất bại kiểm tra cuối.</li>
</ul>
<h3>10.1 Trần vận tốc cục bộ của transition</h3>
<div class="eq">v̄ = min(v<sub>max</sub>,
ω<sub>max</sub>/|κ|,
√(a<sub>y,max</sub>/|κ|),
v<sub>w,max</sub>/max(|1−Lκ/2|,|1+Lκ/2|))</div>

<h2>11. Tìm kiếm thích nghi và hàm mục tiêu</h2>
<h3>11.1 Vì sao không dùng gradient descent?</h3>
<p>Collision checker và costmap là rời rạc; một thay đổi rất nhỏ có thể làm
footprint bước sang ô khác. Hàm objective không trơn và không chắc đơn đỉnh.
Thuật toán dùng coarse-to-fine xác định.</p>
<h3>11.2 Miền và thứ tự tinh chỉnh</h3>
<ol>
<li>Từ |φ| và chiều dài hai đoạn, tính miền d tương ứng R=0,10…1,50 m và hình học sẵn có.</li>
<li>Lấy sáu mẫu đầu phủ toàn miền.</li>
<li>Ưu tiên chia đôi interval có biên feasible/infeasible hoặc safe/unsafe.</li>
<li>Tiếp theo ưu tiên vùng kề objective tốt nhất và vùng objective còn biến đổi.</li>
<li>Dừng ở tối đa 20 đánh giá, tolerance R=0,01 m hoặc objective=0,01.</li>
<li>Xếp hạng ổn định theo objective rồi d nhỏ hơn; giữ tối đa năm ứng viên.</li>
</ol>
<h3>11.3 Cost không phụ thuộc mật độ lấy mẫu</h3>
<div class="eq">risk = min(1, peak_cost/252)</div>
<div class="eq">angular = min(1, |ω|<sub>peak</sub>/ω<sub>max</sub>)</div>
<div class="eq">energy = E<sub>κ</sub>/(E<sub>κ</sub>+1,0 m<sup>−1</sup>)</div>
<div class="eq">J = 0,15·risk + 0,10·angular + 0,75·energy</div>
<p>J nhỏ hơn tốt hơn. An toàn không nằm trong J; an toàn được kiểm tra trước.
Candidate còn phải nằm trong time gate so với Pivot/candidate nhanh nhất.</p>

<h2>12. Quy hoạch động giữa các góc</h2>
{figure(
    "figure_06_search_dp.png",
    "DP ghép ứng viên giữa các góc liên tiếp.",
)}
<h3>12.1 Ràng buộc đoạn chung</h3>
<div class="eq">d<sub>i,a</sub> + d<sub>i+1,b</sub> + m<sub>i</sub>
≤ L<sub>i</sub></div>
<p>m mặc định bằng max(output spacing, 2×sample spacing, costmap resolution)
=max(0,05;0,04;0,05)=0,05 m. Pivot có d=0 nên có thể giải xung đột.</p>
<h3>12.2 Truy hồi DP</h3>
<div class="eq">D<sub>i</sub>(k)=J<sub>i</sub>(k)+
min<sub>j tương thích k</sub>D<sub>i−1</sub>(j)</div>
<p>Độ phức tạp O(NK²). Khi bằng cost, code ưu tiên ít Pivot hơn rồi index nhỏ
hơn để deterministic. Backtracking lấy chuỗi trạng thái cuối cùng.</p>

<h2>13. Cổng an toàn Adaptive Hybrid</h2>
<h3>13.1 Vì sao pure Pivot–G2 chưa phải toàn bộ phương pháp?</h3>
<p>Giảm Eκ không đảm bảo controller chạy tốt trong hành lang sát vật cản.
Simple đôi khi cho clearance tốt hơn. Do đó phương pháp hoàn chỉnh lấy Simple
làm mặc định và chỉ nhận Pivot khi có safety gain đủ rõ.</p>
<div class="eq">Δcost = cost<sub>Simple</sub> − cost<sub>Pivot</sub> ≥ 20</div>
<div class="eq">E<sub>Pivot</sub> ≤ 2·(E<sub>Simple</sub> + 0,25)</div>
{table(
    ("Tình trạng", "Đường được chọn", "Lý do"),
    [
        ("Simple và Pivot an toàn; Pivot qua hai gate", "Pivot–G2", "Có lợi proximity mà không tăng Eκ quá ngân sách"),
        ("Cả hai an toàn nhưng Pivot không qua gate", "Simple", "Default bảo thủ"),
        ("Simple không an toàn, Pivot an toàn", "Pivot–G2", "simple_unsafe"),
        ("Pivot không an toàn, Simple an toàn", "Simple", "pivot_unsafe"),
        ("Cả hai không an toàn, Raw an toàn", "Raw", "raw fallback"),
        ("Raw/Simple/Pivot đều không an toàn", "Fail", "Không xuất đường nguy hiểm"),
    ],
    "compact",
)}

<h2>14. Profile vận tốc và điều khiển vòng kín</h2>
<h3>14.1 Hướng ban đầu: hai tầng thay vì đoán một lần</h3>
<p><b>Tầng trước planning.</b> Nếu scenario không ghi yaw, bộ resolver đọc
PGM/YAML, inflation theo footprint, tìm route A* bảo thủ và kiểm tra swept
footprint theo nhiều hướng. Yaw này làm tư thế spawn an toàn; nó chưa thể biết
chính xác planner nào sẽ chọn nhánh nào.</p>
<p><b>Tầng sau planning.</b> Khi path đã được smoother chọn, controller lấy
điểm preview cách start 0,30 m theo chiều dài cung:</p>
<div class="eq">ψ<sub>path,0</sub> =
atan2(y<sub>preview</sub>−y<sub>0</sub>,
x<sub>preview</sub>−x<sub>0</sub>)</div>
<p>Nếu |ψpath,0−ψrobot| ≥ 0,15 rad, xe giữ v=0 và căn hướng; chỉ bắt đầu tịnh
tiến khi sai số vào dải 0,035 rad và vận tốc góc đã dừng. Hysteresis
0,15/0,035 tránh bật–tắt quanh một ngưỡng.</p>
<p><b>Neo start/goal.</b> Planner grid thường trả tâm ô lệch pose yêu cầu
√(0,025²+0,025²)=0,03536 m. Benchmark và panel RViz2 nay phục hồi chính xác
pose start/goal trước và sau smoothing; correction lớn hơn 0,08 m bị từ chối.</p>

<h3>14.2 Trần tức thời theo độ cong</h3>
<div class="eq">v̄(s)=min[v<sub>max</sub>,
√(a<sub>y,max</sub>/|κ|),
ω<sub>max</sub>/|κ|,
v<sub>w,max</sub>/(1+L|κ|/2)]</div>
<p>Đường thẳng có κ≈0 nên chỉ bị vmax/bánh giới hạn. Cong gắt tăng |κ| làm ba
trần còn lại giảm.</p>
<h3>14.3 Khoảng chuyển vận tốc giới hạn jerk</h3>
<p>Đặt Δv=|v1−v0|, a là giới hạn tăng/giảm tốc, j là jerk. Khi
Δv≤a²/j, profile gia tốc tam giác:</p>
<div class="eq">S = (v0+v1)√(Δv/j)</div>
<p>Khi Δv&gt;a²/j, profile có đoạn giữ gia tốc:</p>
<div class="eq">S = 0,5(v0+v1)(Δv/a + a/j)</div>
<p>Code dùng tìm kiếm nhị phân để đảo công thức: biết khoảng cách còn lại thì
tìm vận tốc lớn nhất vẫn kịp tăng/giảm.</p>
<h3>14.4 Hai lượt truyền</h3>
<ul>
<li><b>Backward pass:</b> giới hạn tương lai được truyền ngược để phanh trước cong/goal.</li>
<li><b>Forward pass:</b> giới hạn quá khứ được truyền xuôi để không tăng tốc ngay sau cong nhanh hơn động lực học cho phép.</li>
<li>Hai pass lặp đến hội tụ vì một cap mới ở pass này có thể tác động pass kia.</li>
<li>Sau đó cặp điểm tiếp tục bị scale đến khi |Δ(vκ)|/Δt≤αmax.</li>
</ul>
<h3>14.5 Phép chiếu tiến độ có hướng</h3>
<div class="eq">score = e<sub>xy</sub><sup>2</sup> +
(0,20·e<sub>ψ</sub>)<sup>2</sup></div>
<p>Chỉ tìm trong cửa sổ 0,25 m sau và 0,80 m trước hint; tiến độ chỉ được lùi
tối đa 0,03 m. Khi hai điểm bằng score, chọn s lớn hơn. Đây là sửa lỗi chọn
nhầm nhánh gần về khoảng cách nhưng ngược hướng.</p>
<h3>14.6 RPP và giảm tốc phục hồi</h3>
<p>Pure Pursuit chọn carrot phía trước. Trong hệ robot, nếu carrot có tọa độ
(xL,yL) và khoảng lookahead ℓ thì κ xấp xỉ 2yL/ℓ²; sau đó ω=vκ. RPP của Nav2
còn giảm v theo cost, curvature và time-to-collision.</p>
<p>Wrapper của bạn lấy min giữa RPP và các cap. Khi cross-track, heading hoặc
angular-tracking error đi từ soft đến hard, cap giảm bằng smoothstep:</p>
<div class="eq">r=(|e|−e<sub>soft</sub>)/(e<sub>hard</sub>−e<sub>soft</sub>);
&nbsp; h(r)=3r²−2r³</div>
<div class="eq">v<sub>cap</sub>=(1−h)v<sub>max</sub>+h·v<sub>min</sub></div>
<h3>14.7 Bộ tạo lệnh jerk-limited</h3>
<div class="eq">a* = clamp((v<sub>target</sub>−v<sub>prev</sub>)/Δt,
−a<sub>dec</sub>,a<sub>acc</sub>)</div>
<div class="eq">a<sub>cmd</sub> =
clamp(a*, a<sub>prev</sub>−jΔt, a<sub>prev</sub>+jΔt)</div>
<div class="eq">v<sub>cmd</sub> =
min(v<sub>target</sub>, max(0,v<sub>prev</sub>+a<sub>cmd</sub>Δt))</div>
<p>Một safety cap thấp hơn được phép thắng ngay; mẫu đó được gắn
<code>safety_override</code> để không giả vờ rằng jerk vật lý luôn được giữ khi
an toàn đòi hỏi phanh khẩn. Biên v=0 cũng là một hard override: nếu quán tính
gia tốc của S-curve định đưa speed magnitude xuống âm, code clip về 0, gắn cờ
trung thực và khởi tạo lại gia tốc ở trạng thái khả thi. Trong bảy lượt cuối,
mọi mẫu <i>nominal</i> đều có |j|≤0,90 m/s³.</p>
<h3>14.8 Bao phanh góc, Pivot và terminal servo</h3>
<p>Trace Gazebo cho thấy giảm tốc góc hiệu dụng của mô hình bốn điểm tiếp xúc
chỉ khoảng 0,18 rad/s² trong khi giới hạn lệnh là 1,20 rad/s². Dùng riêng giới
hạn lệnh khiến xe quay quá yaw đích rồi phải quay ngược. Vì vậy lệnh quay nay
có thêm bao phanh:</p>
<div class="eq">ω<sub>brake</sub> =
min[ω<sub>max</sub>, √(2α<sub>eff</sub>
max(|e<sub>ψ</sub>|−e<sub>tol</sub>,0))]</div>
<div class="eq">ω* = sign(e<sub>ψ</sub>)
min(K<sub>ψ</sub>|e<sub>ψ</sub>|,ω<sub>brake</sub>)</div>
<p>Hai pose trùng vị trí nhưng khác yaw là marker Pivot. Controller dừng dịch
chuyển, quay với tốc độ tối đa 0,70 rad/s và deadband 0,015 rad. Tại goal, xe
phanh vào staging radius, chốt vị trí đích trong odom, cho phép servo vị trí
tiến/lùi tốc độ thấp, rồi căn yaw theo TF mới nhất. Nếu sai số vị trí tăng quá
0,04 m khi đang quay, controller quay lại pha lấy XY.</p>
{figure(
    "figure_10_speed_trace.png",
    "Trace đo từ Gazebo: quỹ đạo, profile vận tốc và tốc độ thực.",
)}
<p class="definition"><b>Điểm quan trọng:</b> tốc độ 0,30 m/s là trần, không
phải tốc độ cố định. Robot tự chạy chậm tại cong, gần vật cản, khi sai số bám
lớn hoặc khi cần phanh; sau cong chỉ tăng lại theo forward acceleration/jerk
envelope.</p>

<h2>15. Bảy môi trường trong RViz2 và Gazebo</h2>
<p>Mỗi hình dưới đây dùng trực tiếp PGM/YAML mà RViz2 hiển thị, SDF mà Gazebo
nạp và trace ground truth đã ghi. Các đường xám mờ là toàn bộ scenario start→goal
định nghĩa sẵn; xanh dương là path được chọn; cam là chuyển động vật lý.</p>
{map_sections(validation)}

<h2>16. Benchmark đo gì và tránh sai lệch như thế nào?</h2>
<h3>16.1 Hai tầng đánh giá</h3>
<ul>
<li><b>Hình học:</b> 7 map × 60 scenario × 5 planner × 8 method × 3 repetition
=7.200 dòng. Mỗi nhóm 8 method có cùng raw_path_sha256.</li>
<li><b>Chạy kín:</b> ma trận phân tầng 42 trial, có 24 trial chính
8 method × 3 tốc độ, kiểm tra thêm planner/map và ca robust. Đây không phải toàn
bộ tích 7×5×8×3. Sau thay đổi động học ngày 26/07, bảy lượt đại diện mới
(mỗi map một lượt) được chạy lại bằng Gazebo ground truth; hai ca lỗi chính có
thêm before/after và repetition.</li>
</ul>
<h3>16.2 Định nghĩa metric</h3>
<div class="eq">RMSE = √[(1/M) Σ e<sub>i</sub><sup>2</sup>]</div>
<ul>
<li><b>tracking RMSE ground truth:</b> khoảng cách từ pose vật lý Gazebo đến segment path gần nhất.</li>
<li><b>estimated RMSE:</b> sai số trong hệ map/TF mà controller quan sát.</li>
<li><b>localization error:</b> khoảng cách pose ước lượng với ground truth đã căn chỉnh.</li>
<li><b>curve/exit RMSE:</b> sai số riêng khi |κ| vượt ngưỡng và ngay sau cong.</li>
<li><b>clearance:</b> khoảng hở nhỏ nhất của toàn footprint.</li>
<li><b>success:</b> action, ground-truth goal, yaw và trạng thái dừng đều phải đạt.</li>
</ul>
{validation_table(validation)}
<h3>16.3 Sửa lỗi đã đo được</h3>
<h4>Ca 1 — lệch sau cong ở <code>right_rack_detour</code></h4>
{table(
    ("Metric", "Trước", "Sau", "Cải thiện"),
    [
        ("Action time (s)", fmt(rack_before["controller_action_time_s"], 3), fmt(rack_after["controller_action_time_s"], 3), f"{percentage_reduction(rack_after['controller_action_time_s'], rack_before['controller_action_time_s']):.1f}%"),
        ("GT RMSE (cm)", fmt(100*rack_before["tracking_rmse_m"], 3), fmt(100*rack_after["tracking_rmse_m"], 3), f"{percentage_reduction(rack_after['tracking_rmse_m'], rack_before['tracking_rmse_m']):.1f}%"),
        ("Sai số thoát cong max (cm)", fmt(100*rack_before["curve_exit_tracking_max_error_m"], 3), fmt(100*rack_after["curve_exit_tracking_max_error_m"], 3), f"{percentage_reduction(rack_after['curve_exit_tracking_max_error_m'], rack_before['curve_exit_tracking_max_error_m']):.1f}%"),
        ("Odometry yaw P95 (rad)", fmt(rack_before["odometry_yaw_error_p95_rad"], 5), fmt(rack_after["odometry_yaw_error_p95_rad"], 5), f"{percentage_reduction(rack_after['odometry_yaw_error_p95_rad'], rack_before['odometry_yaw_error_p95_rad']):.1f}%"),
        ("Sai số vị trí cuối (cm)", fmt(100*rack_before["final_position_error_m"], 3), fmt(100*rack_after["final_position_error_m"], 3), f"{percentage_reduction(rack_after['final_position_error_m'], rack_before['final_position_error_m']):.1f}%"),
    ],
    "compact",
)}
<p>Nguyên nhân chính không phải xe “chạy nhanh một cách cố định”. Controller
chỉ thấy estimated RMSE vài millimet nhưng ground truth lệch lớn hơn; fit
odometry yaw đã chỉ ra separation hiệu dụng bị thiếu. Sau khi hiệu chuẩn 0,2834
m, sai số yaw giảm và feedback speed envelope mới phản ánh đúng chuyển động.</p>

<h4>Ca 2 — hướng đầu ở <code>lower_left_diagonal</code></h4>
{table(
    ("Metric", "Trước", "Sau", "Cải thiện"),
    [
        ("Odometry vị trí P95 (cm)", fmt(100*lower_before["odometry_position_error_p95_m"], 3), fmt(100*lower_after["odometry_position_error_p95_m"], 3), f"{percentage_reduction(lower_after['odometry_position_error_p95_m'], lower_before['odometry_position_error_p95_m']):.1f}%"),
        ("Odometry yaw P95 (rad)", fmt(lower_before["odometry_yaw_error_p95_rad"], 5), fmt(lower_after["odometry_yaw_error_p95_rad"], 5), f"{percentage_reduction(lower_after['odometry_yaw_error_p95_rad'], lower_before['odometry_yaw_error_p95_rad']):.1f}%"),
        ("RMSE controller nhìn thấy (cm)", fmt(100*lower_before["estimated_tracking_rmse_m"], 3), fmt(100*lower_after["estimated_tracking_rmse_m"], 3), f"{percentage_reduction(lower_after['estimated_tracking_rmse_m'], lower_before['estimated_tracking_rmse_m']):.1f}%"),
        ("Jerk nominal max (m/s³)", fmt(lower_before["adaptive_speed_nominal_max_abs_jerk_mps3"], 3), fmt(lower_after["adaptive_speed_nominal_max_abs_jerk_mps3"], 3), "đúng trần 0,90"),
    ],
    "compact",
)}
<p>Yaw spawn map-aware là 0,050 rad, nhưng preview của path sau smoothing yêu
cầu quay thêm 0,245 rad. Tầng alignment sau planning đã đưa sai số vào dải
0,035 rad trước khi cho v khác 0; vì vậy vị trí không bị kéo lệch trong lúc xe
còn quay.</p>

<h4>Ca 3 — bỏ quay quá đích trong <code>narrow_aisles</code></h4>
{table(
    ("Metric", "Trước bao phanh góc", "Sau", "Cải thiện"),
    [
        ("Mẫu initial alignment", str(narrow_before["adaptive_speed_mode_sample_counts"]["initial_alignment"]), str(narrow_after["adaptive_speed_mode_sample_counts"]["initial_alignment"]), f"{percentage_reduction(narrow_after['adaptive_speed_mode_sample_counts']['initial_alignment'], narrow_before['adaptive_speed_mode_sample_counts']['initial_alignment']):.1f}%"),
        ("Thời gian alignment xấp xỉ (s)", fmt(0.05*narrow_before["adaptive_speed_mode_sample_counts"]["initial_alignment"], 2), fmt(0.05*narrow_after["adaptive_speed_mode_sample_counts"]["initial_alignment"], 2), f"{percentage_reduction(narrow_after['adaptive_speed_mode_sample_counts']['initial_alignment'], narrow_before['adaptive_speed_mode_sample_counts']['initial_alignment']):.1f}%"),
        ("Action time (s)", fmt(narrow_before["controller_action_time_s"], 3), fmt(narrow_after["controller_action_time_s"], 3), f"{percentage_reduction(narrow_after['controller_action_time_s'], narrow_before['controller_action_time_s']):.1f}%"),
        ("Jerk nominal max (m/s³)", fmt(narrow_before["adaptive_speed_nominal_max_abs_jerk_mps3"], 3), fmt(narrow_after["adaptive_speed_nominal_max_abs_jerk_mps3"], 3), "đúng trần 0,90"),
    ],
    "compact",
)}

<h2>17. So sánh cuối cùng với các phương pháp khác</h2>
<h3>17.1 Các phương pháp đối chứng là gì?</h3>
{table(
    ("Phương pháp", "Ý tưởng", "Điểm mạnh", "Điểm yếu trong bài toán này"),
    [
        ("Raw", "Giữ nguyên planner output", "Không thêm runtime/sai lệch", "Góc gấp, Eκ lớn"),
        ("Nav2 Simple", "Gradient smoothing cục bộ", "Nhanh, robust, clearance tốt", "Không mô hình Pivot/G2 riêng"),
        ("Savitzky–Golay", "Lọc đa thức theo cửa sổ", "Rất nhanh, giữ trend", "Có thể khó xử lý inversion/góc đặc biệt"),
        ("Constrained", "Tối ưu có cost/smooth constraints", "Bám costmap và điều kiện biên", "Runtime cao hơn, không có Pivot state của dự án"),
        ("Pivot–G2 fixed", "Thử bank R cố định", "Ablation rõ, Eκ thấp", "Lượng tử hóa bán kính, xử lý overlap kém linh hoạt"),
        ("Pivot–G2 adaptive", "Tìm d liên tục + DP", "Eκ thấp nhất, R riêng từng góc", "Pure branch không luôn robust chạy kín"),
        ("Adaptive Hybrid Pivot–G2", "Safety gate Simple/Pivot/Raw", "Hoàn thành tốt, clearance cao, giữ lợi ích G2 khi đáng chọn", "Không luôn có Eκ thấp bằng pure Pivot vì chủ động fallback"),
    ],
    "compact",
)}
{figure(
    "figure_11_geometry_comparison.png",
    "So sánh hình học trên toàn bộ 7.200 dòng.",
)}
{geometry_overall_table(summary)}
<p>Pivot–G2 thích nghi giảm
<b>{percentage_reduction(pivot['energy'], raw['energy']):.1f}%</b> Eκ so với
Raw. Adaptive Hybrid giảm
<b>{percentage_reduction(hybrid['energy'], raw['energy']):.1f}%</b>, đạt
<b>{hybrid['successes']}/{hybrid['attempts']}</b> và clearance trung bình
<b>{100*hybrid['clearance']:.2f} cm</b>. Pure Pivot có Eκ thấp hơn Hybrid vì
Hybrid cố ý giữ Simple ở ca mà Pivot không đủ safety gain.</p>
{figure(
    "figure_12_map_energy_comparison.png",
    "Mức giảm năng lượng độ cong theo từng môi trường.",
)}
{map_method_table(summary)}
<h3>17.2 So sánh chạy kín 8 smoother ở tốc độ thích nghi</h3>
{primary_execution_table(execution)}
<p>Trong scenario chính, Pivot–G2 thích nghi có GT RMSE/exit RMSE tốt nhất
trong nhóm nổi bật, nhưng kết luận không nên chỉ dựa một scenario. Toàn ma trận
phân tầng có 41/42 thành công. Ma trận 42 trial được ghi trước lượt hiệu chuẩn
động học ngày 26/07; nó vẫn hợp lệ để so sánh tương đối các smoother trên cùng
controller và cùng thời điểm, còn bảng bảy map ở Mục 16 là xác nhận riêng cho
phiên bản controller/odometry hiện tại. Không được trộn hai tầng dữ liệu thành
một tuyên bố tuyệt đối.</p>
{figure(
    "figure_13_speed_comparison.png",
    "Ảnh hưởng của smoother và tốc độ đến thời gian, RMSE và sai số thoát cong.",
)}
<h3>17.3 Tương thích với năm planner</h3>
{planner_table(execution)}
<p>Smoother của bạn là hậu xử lý: nó không thay planner. Raw path khác nhau theo
planner nên kết quả cuối cũng khác; benchmark tách planner ID và hash để không
so sánh nhầm input.</p>
<h3>17.4 Ca phản ví dụ phải nhớ</h3>
<p class="warning">Tại
<code>warehouse_dispatch/full_replenishment</code>, ThetaStar + pure Pivot–G2
ở 0,22 m/s thất bại do RPP liên tục dự báo collision và kết thúc
<code>PATIENCE_EXCEEDED</code>. Trên đúng cùng raw_path_sha256, Adaptive Hybrid
chọn Simple, tăng clearance kế hoạch từ 10,61 cm lên 14,49 cm và hoàn thành
trong 95,35 s. Vì vậy tên phương pháp hoàn chỉnh phải nhấn mạnh <b>Hybrid safety
gate</b>, không nên tuyên bố pure Pivot luôn tốt nhất.</p>
{figure(
    "figure_14_all_map_error.png",
    "Sai số chạy kín Pivot–G2 thích nghi trên bảy môi trường.",
)}

<h2>18. Phương pháp của bạn làm được gì?</h2>
<h3>18.1 Những khả năng đã có bằng code và test</h3>
<ul>
<li>Tự tìm R/d riêng từng góc, không bị giới hạn trong bank cũ.</li>
<li>Giữ Pivot như trạng thái thật và thực thi nó ở controller.</li>
<li>Giải xung đột trim giữa nhiều góc bằng DP thay vì greedy.</li>
<li>Giữ liên tục G2 giữa đoạn thẳng và transition Bézier.</li>
<li>Loại ứng viên đảo dấu độ cong, đảo bánh trong, không khả thi vận tốc hoặc va chạm footprint.</li>
<li>Fallback Simple/Raw theo luật công bố trước; không xuất đường không an toàn.</li>
<li>Tự điều chỉnh tốc độ đến 0,30 m/s theo cong, bánh, gia tốc, jerk, sai số bám và goal.</li>
<li>Neo start/goal liên tục vào path lưới và từ chối correction vượt 0,08 m.</li>
<li>Căn hướng hai tầng: route an toàn trước planning và tiếp tuyến path thật sau planning.</li>
<li>Phanh góc theo giảm tốc hiệu dụng đo từ Gazebo để không quay quá yaw đích.</li>
<li>Không nhảy nhánh projection tại đường gần tự giao; không tăng tốc sớm sau cong.</li>
<li>Tách ground truth, odom và AMCL khi đo; so sánh công bằng bằng path hash.</li>
<li>Chạy trên bảy môi trường, năm planner và hiển thị riêng từng phương pháp trong RViz2.</li>
<li>Giữ một hardware contract chung cho URDF, SDF, motor GA25 và pack 4S4P.</li>
</ul>
<h3>18.2 Đâu là đóng góp riêng, đâu là thành phần kế thừa?</h3>
{table(
    ("Thành phần", "Nguồn", "Vai trò trong hệ của bạn"),
    [
        ("ROS 2/Nav2/Gazebo/RViz2", "Nền tảng mã nguồn mở", "Middleware, navigation servers, physics và visualization"),
        ("NavFn/Theta*/Smac", "Planner Nav2", "Sinh raw path"),
        ("Simple/SG/Constrained", "Baseline Nav2", "Đối chứng và nhánh Simple trong Hybrid"),
        ("Bézier bậc năm G2", "Cơ sở toán học đã có trong nghiên cứu", "Dạng transition được hiện thực và ràng buộc theo robot"),
        ("Adaptive trim search + DP + diagnostics", "Phần phát triển của dự án", "Tự chọn candidate toàn đường"),
        ("Safety-gated Hybrid có Raw fallback", "Phần phát triển của dự án", "Độ robust của phương pháp hoàn chỉnh"),
        ("Bidirectional jerk-aware speed envelope", "Phần phát triển của dự án", "Phanh trước và tăng lại sau cong khả thi"),
        ("Continuous endpoint anchor + two-stage heading", "Phần phát triển của dự án", "Sửa offset tâm ô và chỉ chạy tiến sau khi khớp hướng path thật"),
        ("Measured angular-braking envelope", "Phần phát triển của dự án", "Giảm quay quá đích theo α hiệu dụng đo từ Gazebo"),
        ("Heading-aware projection + terminal servo", "Phần phát triển của dự án", "Sửa hướng sai, lệch sau cong và goal"),
        ("Ground-truth benchmark phân tầng", "Phần phát triển của dự án", "Đo vật lý, định vị và controller riêng"),
    ],
    "compact",
)}

<h2>19. Giới hạn và cách diễn giải trung thực</h2>
<ul>
<li>Không có bằng chứng tối ưu toàn cục liên tục; coarse-to-fine có evaluation budget.</li>
<li>Mô phỏng chưa thay thế robot thật: ma sát sàn, backlash, tải, pin và nhiễu sensor thực khác Gazebo.</li>
<li>Closed-loop là thiết kế phân tầng, chưa phải toàn bộ tích 7 map × 5 planner × 8 smoother × 3 tốc độ × nhiều seed.</li>
<li>Adaptive parameters được tuning trên cùng họ map; cần hold-out map và nhiều seed để kết luận tổng quát.</li>
<li>Footprint hiện là rectangle bảo thủ; robot thật cần đo envelope và khoảng cách vệt lăn lại.</li>
<li>RPP collision prediction có thể hủy ca mà footprint hình học tĩnh vẫn clear; đó là khác biệt giữa feasibility hình học và executability vòng kín.</li>
<li>Encoder chưa có PPR, vị trí gắn và chế độ x1/x2/x4; chưa thể chuyển tick sang mét một cách đúng đắn.</li>
<li>Điện áp 3,7/4,2 V mỗi cell là giả định Li-ion chuẩn. BMS, fuse, buck, driver và cutoff vẫn phải chọn và xác minh trên phần cứng thật.</li>
<li>Bảy lượt ngày 26/07 xác nhận phiên bản hiện tại trên từng map nhưng chưa phải so sánh lại đủ tám smoother trên mọi map sau hiệu chuẩn.</li>
</ul>

<h2>20. Cách chạy, quan sát và debug</h2>
<h3>20.1 Build và mở hệ chuyển map</h3>
<p><code>cd /home/linh-pham/agv_nav2_research_ws<br>
source /opt/ros/jazzy/setup.bash<br>
colcon build --symlink-install<br>
source install/setup.bash<br>
ros2 launch vacuum_robot_gazebo switchable_simulation.launch.py gui:=true</code></p>
<h3>20.2 Quy trình cho người mới</h3>
<ol>
<li>Trong panel RViz2 chọn map và nhấn đổi môi trường; đợi Nav2 active.</li>
<li>Chọn planner.</li>
<li>Dùng 2D Goal Pose click–drag để đặt vị trí và yaw goal.</li>
<li>Bật “Hiện tất cả”, sau đó ẩn từng smoother để nhìn riêng.</li>
<li>Chọn execute method là <code>adaptive_hybrid</code khi muốn chạy phương pháp hoàn chỉnh.</li>
<li>Quan sát đường trắng ground truth, topic telemetry và cửa sổ Gazebo.</li>
</ol>
<h3>20.3 Topic hữu ích</h3>
{table(
    ("Topic", "Nội dung"),
    [
        ("<code>/research/goal_pose</code>", "Goal nghiên cứu từ RViz2"),
        ("<code>/planner_selector</code>", "Planner đang chọn"),
        ("<code>/research/smoother_visibility</code>", "Các đường đang bật/ẩn"),
        ("<code>/research/environment_selector</code>", "Yêu cầu đổi world/map"),
        ("<code>/research/metrics</code>", "Metric từng đường"),
        ("<code>/research/adaptive_speed</code>", "Cap, phase, sai số và safety override"),
        ("<code>/cmd_vel</code>", "Lệnh vận tốc cuối"),
        ("<code>/odom</code>", "Wheel odometry"),
        ("<code>/ground_truth/odom</code>", "Pose vật lý Gazebo"),
        ("<code>/scan</code>", "Lidar"),
    ],
    "compact",
)}
<h3>20.4 Test</h3>
<p><code>colcon test --packages-select adaptive_pivot_g2
adaptive_pivot_g2_nav2 adaptive_pivot_g2_controller
adaptive_pivot_g2_benchmark adaptive_pivot_g2_rviz vacuum_robot_gazebo<br>
colcon test-result --all --verbose</code></p>

<h2>21. Glossary thuật ngữ tiếng Anh</h2>
<p>Bảng này giải nghĩa thuật ngữ theo đúng ngữ cảnh dự án; cùng một từ có thể có
nghĩa rộng hơn trong lĩnh vực khác.</p>
{glossary_table()}

<h2>22. Tài liệu tham khảo và đường dẫn source</h2>
<ol>
<li>K. R. Simba, N. Uchiyama, S. Sano, “Real-time smooth trajectory generation
for nonholonomic mobile robots using Bézier curves,” RCIM 41, 2016,
doi:10.1016/j.rcim.2016.02.002.</li>
<li>S. Macenski et al., “Regulated pure pursuit for robot path tracking,”
Autonomous Robots 47, 2023, doi:10.1007/s10514-023-10097-6.</li>
<li>H. Pham, Q.-C. Pham, “A new approach to time-optimal path parameterization
based on reachability analysis,” arXiv:1707.07239.</li>
<li>Navigation2 documentation: planner, smoother, controller, costmap, AMCL và
Behavior Tree.</li>
</ol>
{table(
    ("Nội dung", "File chính"),
    [
        ("Bézier G2", "<code>src/adaptive_pivot_g2/src/quintic_transition.cpp</code>"),
        ("Adaptive search", "<code>src/adaptive_pivot_g2/src/adaptive_search.cpp</code>"),
        ("DP", "<code>src/adaptive_pivot_g2/src/path_optimization.cpp</code>"),
        ("Hybrid gate", "<code>src/adaptive_pivot_g2/src/hybrid_selection.cpp</code>"),
        ("Nav2 smoother", "<code>src/adaptive_pivot_g2_nav2/src/</code>"),
        ("Speed envelope", "<code>src/adaptive_pivot_g2_controller/src/adaptive_speed_profile.cpp</code>"),
        ("Maneuver-aware RPP", "<code>src/adaptive_pivot_g2_controller/src/maneuver_aware_rpp_controller.cpp</code>"),
        ("Hướng ban đầu map-aware", "<code>src/adaptive_pivot_g2_benchmark/adaptive_pivot_g2_benchmark/initial_heading.py</code>"),
        ("Neo start/goal", "<code>src/adaptive_pivot_g2_benchmark/adaptive_pivot_g2_benchmark/path_contract.py</code>"),
        ("Benchmark", "<code>src/adaptive_pivot_g2_benchmark/</code>"),
        ("Cấu hình", "<code>src/vacuum_robot_gazebo/config/nav2_params.yaml</code>"),
        ("Profile robot thật", "<code>src/vacuum_robot_gazebo/config/real_robot_profile.yaml</code>"),
        ("URDF/SDF robot", "<code>src/vacuum_robot_gazebo/urdf/vacuum_robot.urdf</code> và <code>models/vacuum_robot/model.sdf</code>"),
        ("Map/world", "<code>src/vacuum_robot_gazebo/maps/</code> và <code>worlds/</code>"),
    ],
    "compact",
)}
<p class="lead"><b>Kết luận cuối.</b> Điểm riêng mạnh nhất của hệ không phải chỉ
là một đường Bézier đẹp. Đó là chuỗi logic kín: ứng viên G2/Pivot thích nghi →
DP chống overlap → swept-footprint → Hybrid fallback → speed envelope hai chiều
→ RPP có projection/servo → benchmark ground truth. Pure Pivot tối ưu độ cong;
Adaptive Hybrid Pivot–G2 mới là phương pháp hoàn chỉnh nên dùng để chạy xe.</p>
</body></html>"""


def validate_inputs(summary, execution, validation, current_audit, hardware):
    if summary.get("geometry_row_count") != 7200:
        raise RuntimeError("Expected the verified 7,200-row geometry matrix")
    if summary.get("geometry_pairing_group_count") != 900:
        raise RuntimeError("Expected 900 paired raw-path groups")
    if summary.get("geometry_pairing_bad_groups"):
        raise RuntimeError("Geometry methods did not receive paired raw paths")
    if len(execution) != 42:
        raise RuntimeError(f"Expected 42 compact execution rows, got {len(execution)}")
    if sum(row.get("success") == "True" for row in execution) != 41:
        raise RuntimeError("Expected the disclosed 41/42 closed-loop outcome")
    if set(validation) != set(ENVIRONMENTS):
        raise RuntimeError("Seven-map validation is incomplete")
    for environment, row in validation.items():
        if not row.get("success"):
            raise RuntimeError(f"Validation failed for {environment}")
        if not row.get("physically_settled"):
            raise RuntimeError(f"Robot did not physically settle for {environment}")
        if not row.get("selected_path_xy") or not row.get("ground_truth_state_trace"):
            raise RuntimeError(f"Missing path/ground-truth trace for {environment}")
        if as_float(row.get("adaptive_speed_nominal_max_abs_jerk_mps3")) > 0.900001:
            raise RuntimeError(f"Nominal jerk exceeded 0.9 m/s^3 for {environment}")
        for key in (
            "planner_start_anchor_adjustment_m",
            "selected_start_anchor_adjustment_m",
        ):
            if as_float(row.get(key), 0.0) > 0.080001:
                raise RuntimeError(f"Start anchoring exceeded contract in {environment}")

    if set(current_audit) != set(CURRENT_AUDIT_PATHS):
        raise RuntimeError("Before/after audit dataset is incomplete")
    for label, row in current_audit.items():
        if not row.get("success") or not row.get("physically_settled"):
            raise RuntimeError(f"Audit trace is not a settled success: {label}")

    drive = hardware["drive"]
    power = hardware["power"]
    expected_drive = {
        "rated_voltage_v": 12.0,
        "gearbox_ratio": 45.0,
        "armature_speed_rpm": 6000.0,
        "nominal_output_rpm": 130.0,
        "no_load_current_per_motor_a": 0.060,
        "rated_load_output_rpm": 100.0,
        "rated_load_current_per_motor_a": 0.300,
        "rated_output_torque_kgf_cm": 1.0,
        "stall_current_per_motor_a": 1.3,
        "stall_output_torque_kgf_cm": 3.6,
        "motor_body_length_m": 0.068,
        "motor_body_diameter_m": 0.025,
    }
    for key, expected in expected_drive.items():
        if not math.isclose(as_float(drive.get(key)), expected, rel_tol=1.0e-9):
            raise RuntimeError(f"Hardware profile mismatch: drive.{key}")
    encoder = drive["encoder"]
    if any(
        encoder.get(key) is not None
        for key in (
            "pulses_per_revolution",
            "quadrature_decode",
            "encoder_ticks_per_rev",
            "radians_per_tick",
            "metres_per_tick",
        )
    ):
        raise RuntimeError("Unknown encoder resolution must not be guessed")
    if (
        power.get("topology") != "4S4P"
        or power.get("total_cell_count") != 16
        or not math.isclose(as_float(power["pack"]["capacity_ah"]), 10.4)
        or not power["motor_rail"].get("regulator_required")
    ):
        raise RuntimeError("4S4P power contract is inconsistent")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Validate all source datasets without writing report artifacts.",
    )
    parser.add_argument(
        "--write-legacy-html",
        action="store_true",
        help="Also write the older compact HTML view (disabled by default).",
    )
    args = parser.parse_args()
    summary = load_geometry_summary()
    execution = load_execution_rows()
    validation = load_validation()
    current_audit = load_current_audit()
    hardware = load_hardware_profile()
    validate_inputs(summary, execution, validation, current_audit, hardware)
    if args.check_only:
        print("tutorial report inputs: OK")
        return

    ASSETS.mkdir(parents=True, exist_ok=True)
    set_plot_style()
    save_learning_roadmap()
    save_ros_nav2_flow()
    save_diff_drive_kinematics()
    save_costmap_footprint()
    save_pivot_g2_geometry()
    save_search_dp()
    copy_evidence_assets()
    for environment in ENVIRONMENTS:
        save_map_detail(environment, validation[environment])
    save_current_speed_trace(validation)
    save_current_all_map_error(validation)
    save_current_audit_comparison(current_audit)
    save_current_figure_manifest(validation, current_audit)
    if args.write_legacy_html:
        OUTPUT.write_text(
            tutorial_html(summary, execution, validation, current_audit, hardware),
            encoding="utf-8",
        )
    print(
        json.dumps(
            {
                "html": str(OUTPUT) if args.write_legacy_html else None,
                "assets": str(ASSETS),
                "asset_count": len(list(ASSETS.glob("*.png"))),
                "geometry_rows": summary["geometry_row_count"],
                "execution_rows": len(execution),
                "map_count": len(validation),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
