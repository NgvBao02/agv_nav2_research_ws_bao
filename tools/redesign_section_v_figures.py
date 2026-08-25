#!/usr/bin/env python3
"""Generate publication-ready and editable replacements for Figures 6–10."""

from __future__ import annotations

import base64
import copy
import csv
import json
import math
import mimetypes
import statistics
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
MAP_DIR = ROOT / "src/vacuum_robot_gazebo/maps"
RVIZ_DIR = ROOT / "docs/pstmo_bao_cao_toan_dien_assets/rviz_cases"
CSV_PATH = ROOT / "docs/pstmo_bao_cao_toan_dien_assets/benchmark_hinh_hoc_175_luot.csv"
OUT_DIR = ROOT / "final_bao_ICEEIS/section_v_redesigned_figures"
ASSET_DIR = OUT_DIR / "drawio_assets"
DRAWIO_PATH = OUT_DIR / "ICEEIS_2026_PSTMO_section_V_figures_6_10_editable.drawio"

FONT_REGULAR = "/usr/share/fonts/truetype/dejavu/DejaVuSerif.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSerif-Bold.ttf"
FONT_ITALIC = "/usr/share/fonts/truetype/dejavu/DejaVuSerif-Italic.ttf"

BLUE = "#0072B2"
ORANGE = "#D55E00"
GREEN = "#009E73"
PURPLE = "#7A3DF0"
PINK = "#CC79A7"
TEXT = "#202124"
GRAY = "#6B7280"
LIGHT_GRAY = "#D9DDE3"
VERY_LIGHT = "#F5F7FA"
WHITE = "#FFFFFF"

ENVIRONMENTS = [
    ("open_arena", "center_block_detour", "Không gian mở", 6, (950, 808), 1.8),
    ("narrow_aisles", "southwest_northeast_weave", "Lối đi hẹp", 7, (950, 825), 2.2),
    ("warehouse_cross_aisles", "cross_aisle_transfer", "Kho có lối giao cắt", 8, (950, 805), 2.0),
]
ENV_COLORS = {
    "open_arena": BLUE,
    "narrow_aisles": ORANGE,
    "warehouse_cross_aisles": GREEN,
}
ENV_SHORT = {
    "open_arena": "Không gian mở",
    "narrow_aisles": "Lối đi hẹp",
    "warehouse_cross_aisles": "Kho giao cắt",
}
PLANNERS = ["NavFnAStar", "NavFnDijkstra", "ThetaStar", "Smac2D", "SmacHybrid"]
PLANNER_LABELS = {
    "NavFnAStar": "NavFn A*",
    "NavFnDijkstra": "NavFn Dijkstra",
    "ThetaStar": "Theta*",
    "Smac2D": "Smac 2D",
    "SmacHybrid": "Smac Hybrid",
}


def font(size: int, *, bold: bool = False, italic: bool = False) -> ImageFont.FreeTypeFont:
    path = FONT_BOLD if bold else FONT_ITALIC if italic else FONT_REGULAR
    return ImageFont.truetype(path, size)


def hex_rgb(color: str) -> tuple[int, int, int]:
    color = color.lstrip("#")
    return tuple(int(color[index : index + 2], 16) for index in (0, 2, 4))


def data_uri(path: Path) -> str:
    mime = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
    return f"data:{mime},{base64.b64encode(path.read_bytes()).decode('ascii')}"


def draw_dashed(
    draw: ImageDraw.ImageDraw,
    points: list[tuple[float, float]],
    fill: str,
    width: int,
    dash: float = 12,
    gap: float = 8,
) -> None:
    for start, end in zip(points, points[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            continue
        ux, uy = dx / length, dy / length
        distance = 0.0
        while distance < length:
            stop = min(length, distance + dash)
            draw.line(
                (
                    start[0] + ux * distance,
                    start[1] + uy * distance,
                    start[0] + ux * stop,
                    start[1] + uy * stop,
                ),
                fill=fill,
                width=width,
            )
            distance += dash + gap


class MxPage:
    def __init__(self, mxfile: ET.Element, page_id: str, name: str, width: int, height: int) -> None:
        self.diagram = ET.SubElement(mxfile, "diagram", {"id": page_id, "name": name})
        self.model = ET.Element(
            "mxGraphModel",
            {
                "dx": "1422",
                "dy": "794",
                "grid": "1",
                "gridSize": "10",
                "guides": "1",
                "tooltips": "1",
                "connect": "1",
                "arrows": "1",
                "fold": "1",
                "page": "1",
                "pageScale": "1",
                "pageWidth": str(width),
                "pageHeight": str(height),
                "math": "1",
                "shadow": "0",
            },
        )
        self.root = ET.SubElement(self.model, "root")
        ET.SubElement(self.root, "mxCell", {"id": "0"})
        self.background_layer = f"{page_id}-background"
        self.content_layer = f"{page_id}-content"
        ET.SubElement(self.root, "mxCell", {"id": self.background_layer, "value": "Nền bản đồ", "parent": "0"})
        ET.SubElement(self.root, "mxCell", {"id": self.content_layer, "value": "Nội dung chỉnh sửa", "parent": "0"})
        self.counter = 0

    def finish(self) -> None:
        self.diagram.append(self.model)

    def next_id(self, label: str) -> str:
        self.counter += 1
        return f"{self.diagram.attrib['id']}-{self.counter}-{label}"

    def vertex(
        self,
        label: str,
        x: float,
        y: float,
        width: float,
        height: float,
        style: str,
        value: str = "",
        *,
        background: bool = False,
    ) -> None:
        cell = ET.SubElement(
            self.root,
            "mxCell",
            {
                "id": self.next_id(label),
                "value": value,
                "style": style,
                "parent": self.background_layer if background else self.content_layer,
                "vertex": "1",
            },
        )
        ET.SubElement(
            cell,
            "mxGeometry",
            {
                "x": f"{x:.2f}",
                "y": f"{y:.2f}",
                "width": f"{width:.2f}",
                "height": f"{height:.2f}",
                "as": "geometry",
            },
        )

    def edge(
        self,
        label: str,
        points: list[tuple[float, float]],
        *,
        stroke: str,
        width: float,
        dashed: bool = False,
        curved: bool = False,
        opacity: int = 100,
    ) -> None:
        if len(points) < 2:
            return
        style = (
            "edgeStyle=none;orthogonalLoop=0;jettySize=auto;html=1;rounded=0;startArrow=none;endArrow=none;"
            f"strokeColor={stroke};strokeWidth={width};dashed={1 if dashed else 0};"
            f"curved={1 if curved else 0};opacity={opacity};"
        )
        if dashed:
            style += "dashPattern=10 7;"
        cell = ET.SubElement(
            self.root,
            "mxCell",
            {"id": self.next_id(label), "value": "", "style": style, "parent": self.content_layer, "edge": "1"},
        )
        geometry = ET.SubElement(cell, "mxGeometry", {"relative": "1", "as": "geometry"})
        ET.SubElement(geometry, "mxPoint", {"x": f"{points[0][0]:.2f}", "y": f"{points[0][1]:.2f}", "as": "sourcePoint"})
        ET.SubElement(geometry, "mxPoint", {"x": f"{points[-1][0]:.2f}", "y": f"{points[-1][1]:.2f}", "as": "targetPoint"})
        if len(points) > 2:
            array = ET.SubElement(geometry, "Array", {"as": "points"})
            for x, y in points[1:-1]:
                ET.SubElement(array, "mxPoint", {"x": f"{x:.2f}", "y": f"{y:.2f}"})

    def text(
        self,
        label: str,
        value: str,
        x: float,
        y: float,
        width: float,
        height: float,
        *,
        size: int,
        color: str = TEXT,
        bold: bool = False,
        italic: bool = False,
        align: str = "center",
        valign: str = "middle",
        rotation: float = 0,
    ) -> None:
        font_style = (1 if bold else 0) + (2 if italic else 0)
        style = (
            "text;html=1;whiteSpace=wrap;strokeColor=none;fillColor=none;"
            f"align={align};verticalAlign={valign};fontFamily=Times New Roman;fontSize={size};"
            f"fontColor={color};fontStyle={font_style};rotation={rotation:.2f};"
        )
        self.vertex(label, x, y, width, height, style, value)

    def rect(
        self,
        label: str,
        x: float,
        y: float,
        width: float,
        height: float,
        *,
        fill: str,
        stroke: str,
        stroke_width: float = 2,
        rounded: bool = False,
        dashed: bool = False,
        opacity: int = 100,
        rotation: float = 0,
        background: bool = False,
    ) -> None:
        style = (
            f"rounded={1 if rounded else 0};whiteSpace=wrap;html=1;fillColor={fill};strokeColor={stroke};"
            f"strokeWidth={stroke_width};dashed={1 if dashed else 0};opacity={opacity};rotation={rotation:.2f};"
        )
        if dashed:
            style += "dashPattern=8 6;"
        self.vertex(label, x, y, width, height, style, background=background)

    def ellipse(
        self,
        label: str,
        cx: float,
        cy: float,
        rx: float,
        ry: float,
        *,
        fill: str,
        stroke: str,
        stroke_width: float = 2,
        opacity: int = 100,
    ) -> None:
        style = (
            f"ellipse;whiteSpace=wrap;html=1;fillColor={fill};strokeColor={stroke};"
            f"strokeWidth={stroke_width};opacity={opacity};"
        )
        self.vertex(label, cx - rx, cy - ry, 2 * rx, 2 * ry, style)

    def diamond(
        self,
        label: str,
        cx: float,
        cy: float,
        rx: float,
        ry: float,
        *,
        fill: str,
        stroke: str,
        stroke_width: float = 2,
    ) -> None:
        style = f"rhombus;whiteSpace=wrap;html=1;fillColor={fill};strokeColor={stroke};strokeWidth={stroke_width};"
        self.vertex(label, cx - rx, cy - ry, 2 * rx, 2 * ry, style)

    def image(self, label: str, path: Path, x: float, y: float, width: float, height: float) -> None:
        style = (
            "shape=image;html=1;aspect=fixed;locked=1;movable=0;resizable=0;"
            f"image={data_uri(path)};"
        )
        self.vertex(label, x, y, width, height, style, background=True)


class DualFigure:
    def __init__(self, mxfile: ET.Element, page_id: str, name: str, width: int, height: int) -> None:
        self.image = Image.new("RGB", (width, height), WHITE)
        self.draw = ImageDraw.Draw(self.image, "RGBA")
        self.mx = MxPage(mxfile, page_id, name, width, height)

    def text(
        self,
        label: str,
        value: str,
        x: float,
        y: float,
        width: float,
        height: float,
        *,
        size: int,
        color: str = TEXT,
        bold: bool = False,
        italic: bool = False,
        align: str = "center",
        mx_value: str | None = None,
        rotation: float = 0,
    ) -> None:
        typeface = font(size, bold=bold, italic=italic)
        if align == "left":
            anchor = "lm"
            position = (x + 2, y + height / 2)
        elif align == "right":
            anchor = "rm"
            position = (x + width - 2, y + height / 2)
        else:
            anchor = "mm"
            position = (x + width / 2, y + height / 2)
        if rotation:
            layer = Image.new("RGBA", (int(width), int(height)), (255, 255, 255, 0))
            layer_draw = ImageDraw.Draw(layer)
            layer_draw.multiline_text((width / 2, height / 2), value, font=typeface, fill=color, anchor="mm", align="center", spacing=3)
            layer = layer.rotate(-rotation, expand=True)
            self.image.paste(layer, (round(x + (width - layer.width) / 2), round(y + (height - layer.height) / 2)), layer)
        else:
            self.draw.multiline_text(position, value, font=typeface, fill=color, anchor=anchor, align=align, spacing=3)
        self.mx.text(label, mx_value or value, x, y, width, height, size=size, color=color, bold=bold, italic=italic, align=align, rotation=rotation)

    def line(
        self,
        label: str,
        points: list[tuple[float, float]],
        *,
        stroke: str,
        width: int,
        dashed: bool = False,
        curved: bool = False,
        opacity: int = 100,
    ) -> None:
        rgba = (*hex_rgb(stroke), round(255 * opacity / 100))
        if dashed:
            draw_dashed(self.draw, points, rgba, width)
        else:
            self.draw.line(points, fill=rgba, width=width, joint="curve")
        self.mx.edge(label, simplify_polyline(points, 0.75), stroke=stroke, width=width, dashed=dashed, curved=curved, opacity=opacity)

    def rect(
        self,
        label: str,
        x: float,
        y: float,
        width: float,
        height: float,
        *,
        fill: str,
        stroke: str,
        stroke_width: int = 2,
        rounded: bool = False,
        dashed: bool = False,
        opacity: int = 100,
        rotation: float = 0,
        background: bool = False,
    ) -> None:
        fill_rgba = (*hex_rgb(fill), round(255 * opacity / 100)) if fill != "none" else None
        stroke_rgba = (*hex_rgb(stroke), round(255 * opacity / 100)) if stroke != "none" else None
        if rotation:
            cx, cy = x + width / 2, y + height / 2
            angle = math.radians(rotation)
            corners = []
            for dx, dy in ((-width / 2, -height / 2), (width / 2, -height / 2), (width / 2, height / 2), (-width / 2, height / 2)):
                corners.append((cx + dx * math.cos(angle) - dy * math.sin(angle), cy + dx * math.sin(angle) + dy * math.cos(angle)))
            self.draw.polygon(corners, fill=fill_rgba)
            if stroke_rgba:
                self.draw.line(corners + [corners[0]], fill=stroke_rgba, width=stroke_width, joint="curve")
        elif rounded:
            self.draw.rounded_rectangle((x, y, x + width, y + height), radius=min(width, height) * 0.08, fill=fill_rgba, outline=stroke_rgba, width=stroke_width)
        elif dashed:
            draw_dashed(self.draw, [(x, y), (x + width, y), (x + width, y + height), (x, y + height), (x, y)], stroke_rgba, stroke_width, dash=9, gap=6)
        else:
            self.draw.rectangle((x, y, x + width, y + height), fill=fill_rgba, outline=stroke_rgba, width=stroke_width)
        self.mx.rect(label, x, y, width, height, fill=fill, stroke=stroke, stroke_width=stroke_width, rounded=rounded, dashed=dashed, opacity=opacity, rotation=rotation, background=background)

    def ellipse(
        self,
        label: str,
        cx: float,
        cy: float,
        rx: float,
        ry: float,
        *,
        fill: str,
        stroke: str,
        stroke_width: int = 2,
        opacity: int = 100,
    ) -> None:
        fill_rgba = (*hex_rgb(fill), round(255 * opacity / 100)) if fill != "none" else None
        stroke_rgba = (*hex_rgb(stroke), round(255 * opacity / 100))
        self.draw.ellipse((cx - rx, cy - ry, cx + rx, cy + ry), fill=fill_rgba, outline=stroke_rgba, width=stroke_width)
        self.mx.ellipse(label, cx, cy, rx, ry, fill=fill, stroke=stroke, stroke_width=stroke_width, opacity=opacity)

    def diamond(self, label: str, cx: float, cy: float, rx: float, ry: float, *, fill: str, stroke: str) -> None:
        self.draw.polygon(((cx, cy - ry), (cx + rx, cy), (cx, cy + ry), (cx - rx, cy)), fill=fill, outline=stroke)
        self.mx.diamond(label, cx, cy, rx, ry, fill=fill, stroke=stroke)

    def paste_image(self, label: str, path: Path, x: float, y: float, width: int, height: int) -> None:
        image = Image.open(path).convert("RGB").resize((width, height), Image.Resampling.NEAREST)
        self.image.paste(image, (round(x), round(y)))
        self.mx.image(label, path, x, y, width, height)

    def save(self, path: Path) -> None:
        self.mx.finish()
        self.image.save(path, dpi=(300, 300))


def point_segment_distance(point, start, end) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    denominator = dx * dx + dy * dy
    if denominator <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denominator))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def simplify_polyline(points: list[tuple[float, float]], epsilon: float) -> list[tuple[float, float]]:
    if len(points) <= 2:
        return points
    start, end = points[0], points[-1]
    max_distance = -1.0
    max_index = 0
    for index, point in enumerate(points[1:-1], start=1):
        distance = point_segment_distance(point, start, end)
        if distance > max_distance:
            max_distance = distance
            max_index = index
    if max_distance > epsilon:
        left = simplify_polyline(points[: max_index + 1], epsilon)
        right = simplify_polyline(points[max_index:], epsilon)
        return left[:-1] + right
    return [start, end]


def load_selected_rows() -> list[dict[str, str]]:
    selected = {item[0] for item in ENVIRONMENTS}
    with CSV_PATH.open(encoding="utf-8-sig", newline="") as handle:
        return [row for row in csv.DictReader(handle) if row["environment"] in selected and row["success"] == "True"]


def expand_bounds(bounds, aspect: float, limits=(-6.0, 6.0, -4.0, 4.0)):
    xmin, xmax, ymin, ymax = bounds
    width, height = xmax - xmin, ymax - ymin
    if width / height < aspect:
        target = height * aspect
        xmin -= (target - width) / 2
        xmax += (target - width) / 2
    else:
        target = width / aspect
        ymin -= (target - height) / 2
        ymax += (target - height) / 2
    lx0, lx1, ly0, ly1 = limits
    if xmin < lx0:
        xmax += lx0 - xmin
        xmin = lx0
    if xmax > lx1:
        xmin -= xmax - lx1
        xmax = lx1
    if ymin < ly0:
        ymax += ly0 - ymin
        ymin = ly0
    if ymax > ly1:
        ymin -= ymax - ly1
        ymax = ly1
    return max(lx0, xmin), min(lx1, xmax), max(ly0, ymin), min(ly1, ymax)


def crop_map(env: str, bounds, width: int, height: int, path: Path) -> None:
    pgm = Image.open(MAP_DIR / f"{env}.pgm").convert("L")
    meta = yaml.safe_load((MAP_DIR / f"{env}.yaml").read_text(encoding="utf-8"))
    resolution = float(meta["resolution"])
    ox, oy = map(float, meta["origin"][:2])
    xmin, xmax, ymin, ymax = bounds
    left = (xmin - ox) / resolution
    right = (xmax - ox) / resolution
    top = pgm.height - (ymax - oy) / resolution
    bottom = pgm.height - (ymin - oy) / resolution
    crop = pgm.crop((left, top, right, bottom)).resize((width, height), Image.Resampling.NEAREST).convert("RGB")
    crop.save(path)


def transform(point, bounds, panel):
    x, y = point
    xmin, xmax, ymin, ymax = bounds
    px, py, width, height = panel
    return px + (x - xmin) / (xmax - xmin) * width, py + height - (y - ymin) / (ymax - ymin) * height


def inside(point, bounds) -> bool:
    return bounds[0] <= point[0] <= bounds[1] and bounds[2] <= point[1] <= bounds[3]


def subset_path(points, bounds):
    indexes = [index for index, point in enumerate(points) if inside(point, bounds)]
    if not indexes:
        return []
    start = max(0, min(indexes) - 1)
    stop = min(len(points), max(indexes) + 2)
    return points[start:stop]


def format_vi(value: float, digits: int = 2) -> str:
    return f"{value:.{digits}f}".replace(".", ",")


def map_grid(figure: DualFigure, bounds, panel, prefix: str, *, labels: bool) -> None:
    xmin, xmax, ymin, ymax = bounds
    px, py, width, height = panel
    for value in range(math.ceil(xmin), math.floor(xmax) + 1):
        x, _ = transform((value, ymin), bounds, panel)
        figure.line(f"{prefix}-grid-x-{value}", [(x, py), (x, py + height)], stroke=LIGHT_GRAY, width=1, opacity=75)
        if labels and (value - math.ceil(xmin)) % max(1, math.ceil((xmax - xmin) / 4)) == 0:
            figure.text(f"{prefix}-tick-x-{value}", str(value), x - 22, py + height + 4, 44, 24, size=17)
    for value in range(math.ceil(ymin), math.floor(ymax) + 1):
        _, y = transform((xmin, value), bounds, panel)
        figure.line(f"{prefix}-grid-y-{value}", [(px, y), (px + width, y)], stroke=LIGHT_GRAY, width=1, opacity=75)
        if labels and (value - math.ceil(ymin)) % max(1, math.ceil((ymax - ymin) / 4)) == 0:
            figure.text(f"{prefix}-tick-y-{value}", str(value), px - 42, y - 12, 36, 24, size=17, align="right")
    figure.rect(f"{prefix}-border", px, py, width, height, fill="none", stroke=TEXT, stroke_width=2)
    if labels:
        figure.text(f"{prefix}-xlabel", "x (m)", px + width / 2 - 35, py + height + 26, 70, 25, size=18)
        figure.text(f"{prefix}-ylabel", "y (m)", px - 66, py + height / 2 - 30, 60, 60, size=18, rotation=90)


def path_xy(data, method: str):
    return [(float(pose["x"]), float(pose["y"])) for pose in data["paths"][method]["poses"]]


def draw_path(figure: DualFigure, label: str, points, bounds, panel, *, stroke: str, width: int, dashed: bool) -> None:
    selected = subset_path(points, bounds)
    if len(selected) < 2:
        return
    transformed = [transform(point, bounds, panel) for point in selected]
    figure.line(label, transformed, stroke=stroke, width=width, dashed=dashed, curved=not dashed)


def draw_start_goal(figure: DualFigure, data, bounds, panel, prefix: str) -> None:
    start = (float(data["start"][0]), float(data["start"][1]))
    goal = (float(data["goal"][0]), float(data["goal"][1]))
    if inside(start, bounds):
        sx, sy = transform(start, bounds, panel)
        figure.ellipse(f"{prefix}-start", sx, sy, 7, 7, fill=GREEN, stroke=TEXT, stroke_width=1)
    if inside(goal, bounds):
        gx, gy = transform(goal, bounds, panel)
        figure.diamond(f"{prefix}-goal", gx, gy, 8, 8, fill=PINK, stroke=TEXT)


def draw_footprints(figure: DualFigure, data, bounds, panel, prefix: str) -> None:
    poses = [pose for pose in data["paths"]["pstmo"]["poses"] if inside((pose["x"], pose["y"]), bounds)]
    if not poses:
        return
    indexes = sorted({round((0.15 + 0.70 * index / 3) * (len(poses) - 1)) for index in range(4)})
    scale = panel[2] / (bounds[1] - bounds[0])
    for number, index in enumerate(indexes, start=1):
        pose = poses[index]
        cx, cy = transform((pose["x"], pose["y"]), bounds, panel)
        yaw = 2.0 * math.atan2(float(pose.get("qz", 0.0)), float(pose.get("qw", 1.0)))
        figure.rect(
            f"{prefix}-footprint-{number}",
            cx - 0.22 * scale,
            cy - 0.17 * scale,
            0.44 * scale,
            0.34 * scale,
            fill="#B8DBEF",
            stroke="#005F91",
            stroke_width=2,
            opacity=55,
            rotation=-math.degrees(yaw),
        )


def representative_metrics(rows, env: str):
    by_method = {row["method"]: row for row in rows if row["environment"] == env and row["planner"] == "ThetaStar"}
    raw = by_method["raw"]
    pstmo = by_method["pstmo"]
    return {
        "raw_e": float(raw["curvature_energy_1pm"]),
        "pstmo_e": float(pstmo["curvature_energy_1pm"]),
        "raw_l": float(raw["path_length_m"]),
        "pstmo_l": float(pstmo["path_length_m"]),
        "time_ms": 1000 * float(pstmo["algorithm_time_s"]),
        "clearance": float(pstmo["footprint_clearance_min_m"]),
    }


def make_environment_figure(mxfile, rows, env, scenario, label, figure_number, size, inset_span):
    width, height = size
    output = OUT_DIR / f"figure_{figure_number:02d}_{env}.png"
    data = json.loads((RVIZ_DIR / f"{env}__{scenario}__ThetaStar.json").read_text(encoding="utf-8"))
    raw = path_xy(data, "raw")
    pstmo = path_xy(data, "pstmo")
    all_points = raw + pstmo
    margin = 0.75
    main_panel = (72, 72, 520, height - 220)
    main_bounds = expand_bounds(
        (
            min(point[0] for point in all_points) - margin,
            max(point[0] for point in all_points) + margin,
            min(point[1] for point in all_points) - margin,
            max(point[1] for point in all_points) + margin,
        ),
        main_panel[2] / main_panel[3],
    )
    deviation_point = max(
        pstmo,
        key=lambda point: min(point_segment_distance(point, raw[index], raw[index + 1]) for index in range(len(raw) - 1)),
    )
    half = inset_span / 2
    inset_panel = (636, 82, 278, 278)
    inset_bounds = expand_bounds(
        (deviation_point[0] - half, deviation_point[0] + half, deviation_point[1] - half, deviation_point[1] + half),
        1.0,
    )
    main_bg = ASSET_DIR / f"figure_{figure_number:02d}_{env}_main_map.png"
    inset_bg = ASSET_DIR / f"figure_{figure_number:02d}_{env}_inset_map.png"
    crop_map(env, main_bounds, main_panel[2], main_panel[3], main_bg)
    crop_map(env, inset_bounds, inset_panel[2], inset_panel[3], inset_bg)

    figure = DualFigure(mxfile, f"figure-{figure_number}", f"Hình {figure_number} – {label}", width, height)
    figure.text("panel-a", "(a) Bối cảnh bản đồ", main_panel[0], 22, main_panel[2], 32, size=22, bold=True)
    figure.text("panel-b", "(b) Phóng đại chuyển tiếp", inset_panel[0] - 10, 22, inset_panel[2] + 20, 32, size=20, bold=True)
    figure.paste_image("main-map", main_bg, *main_panel)
    map_grid(figure, main_bounds, main_panel, "main", labels=True)
    draw_path(figure, "main-raw", raw, main_bounds, main_panel, stroke=ORANGE, width=3, dashed=True)
    draw_path(figure, "main-pstmo", pstmo, main_bounds, main_panel, stroke=BLUE, width=4, dashed=False)
    draw_start_goal(figure, data, main_bounds, main_panel, "main")

    roi_top_left = transform((inset_bounds[0], inset_bounds[3]), main_bounds, main_panel)
    roi_bottom_right = transform((inset_bounds[1], inset_bounds[2]), main_bounds, main_panel)
    figure.rect(
        "roi",
        roi_top_left[0],
        roi_top_left[1],
        roi_bottom_right[0] - roi_top_left[0],
        roi_bottom_right[1] - roi_top_left[1],
        fill="none",
        stroke=PURPLE,
        stroke_width=2,
        dashed=True,
    )

    figure.paste_image("inset-map", inset_bg, *inset_panel)
    map_grid(figure, inset_bounds, inset_panel, "inset", labels=False)
    draw_path(figure, "inset-raw", raw, inset_bounds, inset_panel, stroke=ORANGE, width=3, dashed=True)
    draw_path(figure, "inset-pstmo", pstmo, inset_bounds, inset_panel, stroke=BLUE, width=5, dashed=False)
    draw_footprints(figure, data, inset_bounds, inset_panel, "inset")
    draw_start_goal(figure, data, inset_bounds, inset_panel, "inset")
    figure.rect("inset-highlight", inset_panel[0], inset_panel[1], inset_panel[2], inset_panel[3], fill="none", stroke=PURPLE, stroke_width=3)

    metrics = representative_metrics(rows, env)
    reduction = 100 * (1 - metrics["pstmo_e"] / metrics["raw_e"])
    length_change = 100 * (metrics["pstmo_l"] / metrics["raw_l"] - 1)
    box_y = 392
    box_h = height - box_y - 105
    figure.rect("metrics-box", 624, box_y, 302, box_h, fill=VERY_LIGHT, stroke="#AAB2BD", stroke_width=2, rounded=True)
    figure.text("metrics-title", "Ca đại diện Theta*", 638, box_y + 10, 274, 34, size=21, bold=True)
    figure.text(
        "metrics-e",
        f"Eκ: {format_vi(metrics['raw_e'])} → {format_vi(metrics['pstmo_e'])} m⁻¹\n(giảm {format_vi(reduction, 1)}%)",
        640,
        box_y + 50,
        270,
        60,
        size=19,
        mx_value=(
            f"E<sub>κ</sub>: {format_vi(metrics['raw_e'])} → {format_vi(metrics['pstmo_e'])} m<sup>−1</sup>"
            f"<br>(giảm {format_vi(reduction, 1)}%)"
        ),
    )
    figure.text(
        "metrics-l",
        f"L: {format_vi(metrics['raw_l'], 3)} → {format_vi(metrics['pstmo_l'], 3)} m\n(ΔL={format_vi(length_change, 1)}%)",
        640,
        box_y + 112,
        270,
        58,
        size=19,
    )
    figure.text(
        "metrics-extra",
        f"c_min={format_vi(metrics['clearance'], 3)} m    T={format_vi(metrics['time_ms'], 0)} ms",
        640,
        box_y + 174,
        270,
        38,
        size=18,
        mx_value=f"c<sub>min</sub>={format_vi(metrics['clearance'], 3)} m&nbsp;&nbsp;&nbsp;T={format_vi(metrics['time_ms'], 0)} ms",
    )

    legend_y = height - 62
    figure.line("legend-raw", [(80, legend_y), (130, legend_y)], stroke=ORANGE, width=3, dashed=True)
    figure.text("legend-raw-text", "Raw", 136, legend_y - 16, 58, 32, size=18, align="left")
    figure.line("legend-pstmo", [(210, legend_y), (260, legend_y)], stroke=BLUE, width=4)
    figure.text("legend-pstmo-text", "PSTMO", 266, legend_y - 16, 80, 32, size=18, align="left")
    figure.rect("legend-footprint", 365, legend_y - 10, 28, 20, fill="#B8DBEF", stroke="#005F91", stroke_width=1, opacity=65)
    figure.text("legend-footprint-text", "hình bao", 402, legend_y - 16, 95, 32, size=18, align="left")
    figure.ellipse("legend-start", 540, legend_y, 6, 6, fill=GREEN, stroke=TEXT, stroke_width=1)
    figure.text("legend-start-text", "đầu", 552, legend_y - 16, 52, 32, size=18, align="left")
    figure.diamond("legend-goal", 635, legend_y, 7, 7, fill=PINK, stroke=TEXT)
    figure.text("legend-goal-text", "đích", 648, legend_y - 16, 58, 32, size=18, align="left")
    figure.rect("legend-roi", 750, legend_y - 10, 28, 20, fill="none", stroke=PURPLE, stroke_width=2, dashed=True)
    figure.text("legend-roi-text", "vùng phóng đại", 786, legend_y - 16, 135, 32, size=18, align="left")
    figure.save(output)
    return output


def log_position(value: float, minimum: float, maximum: float, start: float, end: float) -> float:
    return start + (math.log10(value) - math.log10(minimum)) / (math.log10(maximum) - math.log10(minimum)) * (end - start)


def make_ratio_figure(mxfile, rows):
    width, height = 1060, 880
    output = OUT_DIR / "figure_09_paired_curvature_ratio.png"
    figure = DualFigure(mxfile, "figure-9", "Hình 9 – So sánh ghép cặp Eκ", width, height)
    figure.text(
        "title",
        "Tỷ số ghép cặp Eκ(PSTMO) / Eκ(Simple)",
        150,
        20,
        790,
        42,
        size=25,
        bold=True,
        mx_value="Tỷ số ghép cặp E<sub>κ</sub>(PSTMO) / E<sub>κ</sub>(Simple)",
    )
    grouped = defaultdict(dict)
    for row in rows:
        grouped[(row["environment"], row["planner"])][row["method"]] = row
    x_min, x_max = 0.04, 1.6
    plot_left, plot_right = 315, 930
    top = 120
    ticks = [0.05, 0.1, 0.2, 0.5, 1.0, 1.5]
    for tick in ticks:
        x = log_position(tick, x_min, x_max, plot_left, plot_right)
        figure.line(f"grid-{tick}", [(x, top), (x, 760)], stroke=LIGHT_GRAY, width=1)
        figure.text(f"tick-{tick}", str(tick).replace(".", ","), x - 30, 764, 60, 30, size=18)
    one_x = log_position(1.0, x_min, x_max, plot_left, plot_right)
    figure.line("reference-one", [(one_x, top), (one_x, 760)], stroke=TEXT, width=3, dashed=True)
    y = top + 22
    better_count = 0
    row_number = 0
    for env, _, env_label, _, _, _ in ENVIRONMENTS:
        color = ENV_COLORS[env]
        figure.rect(f"group-band-{env}", 30, y - 4, 960, 32, fill="#EEF2F7", stroke="none", stroke_width=0)
        figure.text(f"group-{env}", env_label, 40, y - 2, 250, 28, size=20, color=color, bold=True, align="left")
        y += 36
        for planner in PLANNERS:
            group = grouped[(env, planner)]
            ratio = float(group["pstmo"]["curvature_energy_1pm"]) / float(group["simple"]["curvature_energy_1pm"])
            if ratio < 1:
                better_count += 1
            if row_number % 2 == 0:
                figure.rect(f"row-bg-{env}-{planner}", 30, y - 13, 960, 31, fill="#FAFBFC", stroke="none", stroke_width=0)
            figure.text(f"planner-{env}-{planner}", PLANNER_LABELS[planner], 82, y - 14, 205, 30, size=18, align="left")
            x = log_position(ratio, x_min, x_max, plot_left, plot_right)
            figure.line(f"effect-{env}-{planner}", [(min(x, one_x), y), (max(x, one_x), y)], stroke=color, width=3, opacity=65)
            outline = "#B00020" if ratio > 1 else WHITE
            figure.ellipse(f"point-{env}-{planner}", x, y, 7, 7, fill=color, stroke=outline, stroke_width=2)
            figure.text(f"ratio-{env}-{planner}", format_vi(ratio, 2), 946, y - 14, 70, 28, size=17, align="right")
            y += 32
            row_number += 1
        y += 14
    figure.text("better-note", "← PSTMO có Eκ thấp hơn", plot_left, 806, one_x - plot_left - 8, 28, size=18, color=BLUE, align="left")
    figure.text("worse-note", "PSTMO có Eκ cao hơn →", one_x + 10, 806, plot_right - one_x - 10, 28, size=18, color="#B00020", align="right")
    figure.rect("count-box", 735, 70, 270, 38, fill=VERY_LIGHT, stroke="#AAB2BD", stroke_width=1, rounded=True)
    figure.text("count", f"{better_count}/15 nhóm nằm dưới ngưỡng 1", 743, 74, 254, 28, size=16, bold=True)
    figure.save(output)
    return output


def quartiles(values):
    q1, _, q3 = statistics.quantiles(values, n=4, method="inclusive")
    return q1, statistics.median(values), q3


def make_runtime_figure(mxfile, rows):
    width, height = 1110, 890
    output = OUT_DIR / "figure_10_runtime_distribution.png"
    figure = DualFigure(mxfile, "figure-10", "Hình 10 – Phân bố thời gian xử lý", width, height)
    figure.text("title", "Phân bố thời gian xử lý trên 15 nhóm ghép cặp", 165, 24, 800, 42, size=25, bold=True)
    plot_left, plot_right = 235, 1030
    plot_top, plot_bottom = 115, 690
    x_min, x_max = 3.0, 300.0
    for tick in (3, 10, 30, 100, 300):
        x = log_position(tick, x_min, x_max, plot_left, plot_right)
        figure.line(f"time-grid-{tick}", [(x, plot_top), (x, plot_bottom)], stroke=LIGHT_GRAY, width=1)
        figure.text(f"time-tick-{tick}", str(tick), x - 35, plot_bottom + 12, 70, 30, size=18)
    floor_x = log_position(3, x_min, x_max, plot_left, plot_right)
    figure.line("measurement-floor", [(floor_x, plot_top - 15), (floor_x, plot_bottom)], stroke=PURPLE, width=3, dashed=True)
    figure.text("floor-label", "ngưỡng phân giải 3 ms", floor_x + 8, 76, 215, 28, size=17, color=PURPLE, align="left")
    methods = ["raw", "simple", "savitzky_golay", "constrained", "pstmo"]
    labels = {
        "raw": "Raw",
        "simple": "Simple",
        "savitzky_golay": "Savitzky–Golay",
        "constrained": "Constrained",
        "pstmo": "PSTMO",
    }
    method_y = {method: 160 + index * 112 for index, method in enumerate(methods)}
    for index, method in enumerate(methods):
        y = method_y[method]
        if index % 2 == 0:
            figure.rect(f"runtime-row-{method}", 45, y - 45, 995, 90, fill="#FAFBFC", stroke="none", stroke_width=0)
        figure.text(f"method-{method}", labels[method], 48, y - 24, 165, 48, size=20, bold=method == "pstmo", align="right")
        method_rows = [row for row in rows if row["method"] == method]
        values = [max(3.0, 1000 * float(row["algorithm_time_s"])) for row in method_rows]
        q1, median, q3 = quartiles(values)
        minimum, maximum = min(values), max(values)
        figure.line(f"range-{method}", [(log_position(minimum, x_min, x_max, plot_left, plot_right), y), (log_position(maximum, x_min, x_max, plot_left, plot_right), y)], stroke="#7A818A", width=3)
        figure.line(f"iqr-{method}", [(log_position(q1, x_min, x_max, plot_left, plot_right), y), (log_position(q3, x_min, x_max, plot_left, plot_right), y)], stroke=TEXT, width=9)
        median_x = log_position(median, x_min, x_max, plot_left, plot_right)
        figure.diamond(f"median-{method}", median_x, y, 8, 8, fill=WHITE, stroke=TEXT)
        ordered = sorted(method_rows, key=lambda row: (row["environment"], PLANNERS.index(row["planner"])))
        for point_index, row in enumerate(ordered):
            actual = 1000 * float(row["algorithm_time_s"])
            plotted = max(3.0, actual)
            x = log_position(plotted, x_min, x_max, plot_left, plot_right)
            planner_offset = PLANNERS.index(row["planner"]) - 2
            env_offset = [item[0] for item in ENVIRONMENTS].index(row["environment"]) - 1
            point_y = y + planner_offset * 7 + env_offset * 2
            color = ENV_COLORS[row["environment"]]
            fill = WHITE if actual < 3.0 else color
            figure.ellipse(f"runtime-point-{method}-{point_index}", x, point_y, 6, 6, fill=fill, stroke=color, stroke_width=2)
    figure.text("axis-title", "Thời gian xử lý T (ms, thang logarit)", plot_left, 742, plot_right - plot_left, 34, size=20, bold=True)
    legend_x = 215
    for env, _, env_label, _, _, _ in ENVIRONMENTS:
        figure.ellipse(f"legend-{env}", legend_x, 825, 6, 6, fill=ENV_COLORS[env], stroke=ENV_COLORS[env], stroke_width=1)
        figure.text(f"legend-text-{env}", env_label, legend_x + 12, 808, 175, 34, size=17, align="left")
        legend_x += 250
    figure.ellipse("legend-censored", 905, 825, 6, 6, fill=WHITE, stroke=GRAY, stroke_width=2)
    figure.text("legend-censored-text", "<3 ms", 918, 808, 95, 34, size=17, align="left")
    figure.save(output)
    return output


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    rows = load_selected_rows()
    mxfile = ET.Element(
        "mxfile",
        {
            "host": "app.diagrams.net",
            "modified": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "agent": "Codex Section V scientific figure generator",
            "version": "26.0.16",
            "type": "device",
            "compressed": "false",
        },
    )
    outputs = []
    for env, scenario, label, number, size, inset_span in ENVIRONMENTS:
        outputs.append(make_environment_figure(mxfile, rows, env, scenario, label, number, size, inset_span))
    outputs.append(make_ratio_figure(mxfile, rows))
    outputs.append(make_runtime_figure(mxfile, rows))
    ET.indent(mxfile, space="  ")
    DRAWIO_PATH.write_bytes(ET.tostring(mxfile, encoding="utf-8", xml_declaration=True))
    separate_dir = OUT_DIR / "drawio_separate"
    separate_dir.mkdir(parents=True, exist_ok=True)
    for diagram in mxfile.findall("diagram"):
        single = ET.Element("mxfile", dict(mxfile.attrib))
        single.append(copy.deepcopy(diagram))
        ET.indent(single, space="  ")
        number = diagram.attrib["id"].split("-")[-1]
        path = separate_dir / f"Hinh_{number}_editable.drawio"
        path.write_bytes(ET.tostring(single, encoding="utf-8", xml_declaration=True))
    print(DRAWIO_PATH)
    for output in outputs:
        print(output)


if __name__ == "__main__":
    main()
