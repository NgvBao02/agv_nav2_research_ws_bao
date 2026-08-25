#!/usr/bin/env python3
"""Rebuild ICEEIS figures 1–5 as individually editable diagrams.net objects."""

from __future__ import annotations

import base64
import copy
import math
import mimetypes
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = ROOT / "final_bao_ICEEIS/ICEEIS_2026_PSTMO_drawio_assets"
OUTPUT = ROOT / "final_bao_ICEEIS/ICEEIS_2026_PSTMO_figures_1_5_editable.drawio"
SEPARATE_OUTPUT_DIR = ROOT / "final_bao_ICEEIS/ICEEIS_2026_PSTMO_figures_1_5_editable"

BLUE = "#0072B2"
ORANGE = "#D55E00"
GREEN = "#009E73"
PURPLE = "#7A3DF0"
PINK = "#CC79A7"
TEXT = "#111111"
GRAY = "#666666"
LIGHT_GRAY = "#8C8C8C"


def image_data_uri(path: Path) -> str:
    mime = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
    return f"data:{mime},{base64.b64encode(path.read_bytes()).decode('ascii')}"


def bezier5(points: list[tuple[float, float]], samples: int = 14) -> list[tuple[float, float]]:
    result = []
    for step in range(samples + 1):
        t = step / samples
        x = y = 0.0
        for index, point in enumerate(points):
            coefficient = math.comb(5, index) * (1 - t) ** (5 - index) * t**index
            x += coefficient * point[0]
            y += coefficient * point[1]
        result.append((x, y))
    return result


def arc_points(
    box: tuple[float, float, float, float], start_deg: float, end_deg: float, samples: int = 14
) -> list[tuple[float, float]]:
    left, top, right, bottom = box
    center_x = (left + right) / 2
    center_y = (top + bottom) / 2
    radius_x = (right - left) / 2
    radius_y = (bottom - top) / 2
    if end_deg < start_deg:
        end_deg += 360
    points = []
    for index in range(samples + 1):
        angle = math.radians(start_deg + (end_deg - start_deg) * index / samples)
        points.append((center_x + radius_x * math.cos(angle), center_y + radius_y * math.sin(angle)))
    return points


class DrawioPage:
    def __init__(
        self,
        mxfile: ET.Element,
        page_id: str,
        name: str,
        canvas_width: int,
        canvas_height: int,
        caption: str,
        reference_image: Path,
    ) -> None:
        self.margin_x = 40
        self.margin_y = 90
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        page_width = canvas_width + 80
        page_height = canvas_height + 220
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
                "pageWidth": str(page_width),
                "pageHeight": str(page_height),
                "math": "1",
                "shadow": "0",
            },
        )
        self.root = ET.SubElement(self.model, "root")
        ET.SubElement(self.root, "mxCell", {"id": "0"})
        ET.SubElement(
            self.root,
            "mxCell",
            {"id": f"{page_id}-reference-layer", "value": "Ảnh gốc tham chiếu (đang ẩn)", "parent": "0", "visible": "0"},
        )
        ET.SubElement(
            self.root,
            "mxCell",
            {"id": f"{page_id}-editable-layer", "value": "Các phần tử chỉnh sửa", "parent": "0"},
        )
        self.reference_layer = f"{page_id}-reference-layer"
        self.editable_layer = f"{page_id}-editable-layer"
        self.counter = 0
        self.text(
            name,
            0,
            -65,
            canvas_width,
            42,
            size=24,
            bold=True,
            align="center",
        )
        self._add_reference(reference_image)
        self.text(
            caption,
            25,
            canvas_height + 25,
            canvas_width - 50,
            70,
            size=17,
            italic=True,
            align="center",
            valign="top",
        )

    def finish(self) -> None:
        self.diagram.append(self.model)

    def _id(self, label: str) -> str:
        self.counter += 1
        return f"{self.diagram.attrib['id']}-{self.counter}-{label}"

    def _xy(self, x: float, y: float) -> tuple[float, float]:
        return x + self.margin_x, y + self.margin_y

    def _add_reference(self, image_path: Path) -> None:
        from PIL import Image

        with Image.open(image_path) as image:
            image_w, image_h = image.size
        scale = min(self.canvas_width / image_w, self.canvas_height / image_h)
        width = image_w * scale
        height = image_h * scale
        x = self.margin_x + (self.canvas_width - width) / 2
        y = self.margin_y + (self.canvas_height - height) / 2
        cell = ET.SubElement(
            self.root,
            "mxCell",
            {
                "id": self._id("reference"),
                "value": "",
                "style": (
                    "shape=image;html=1;aspect=fixed;opacity=30;locked=1;movable=0;resizable=0;"
                    f"image={image_data_uri(image_path)};"
                ),
                "parent": self.reference_layer,
                "vertex": "1",
            },
        )
        ET.SubElement(
            cell,
            "mxGeometry",
            {"x": f"{x:.2f}", "y": f"{y:.2f}", "width": f"{width:.2f}", "height": f"{height:.2f}", "as": "geometry"},
        )

    def vertex(
        self,
        label: str,
        x: float,
        y: float,
        width: float,
        height: float,
        style: str,
        value: str = "",
        parent: str | None = None,
    ) -> ET.Element:
        px, py = self._xy(x, y)
        cell = ET.SubElement(
            self.root,
            "mxCell",
            {
                "id": self._id(label),
                "value": value,
                "style": style,
                "parent": parent or self.editable_layer,
                "vertex": "1",
            },
        )
        ET.SubElement(
            cell,
            "mxGeometry",
            {"x": f"{px:.2f}", "y": f"{py:.2f}", "width": f"{width:.2f}", "height": f"{height:.2f}", "as": "geometry"},
        )
        return cell

    def text(
        self,
        value: str,
        x: float,
        y: float,
        width: float,
        height: float,
        *,
        size: int = 28,
        color: str = TEXT,
        bold: bool = False,
        italic: bool = False,
        align: str = "left",
        valign: str = "middle",
        rotation: float = 0,
        background: str = "none",
    ) -> ET.Element:
        font_style = (1 if bold else 0) + (2 if italic else 0)
        style = (
            "text;html=1;whiteSpace=wrap;strokeColor=none;"
            f"fillColor={background};align={align};verticalAlign={valign};fontFamily=Times New Roman;"
            f"fontSize={size};fontColor={color};fontStyle={font_style};rotation={rotation:.2f};"
        )
        return self.vertex("text", x, y, width, height, style, value)

    def rect(
        self,
        x: float,
        y: float,
        width: float,
        height: float,
        *,
        fill: str = "none",
        stroke: str = TEXT,
        stroke_width: float = 2,
        rounded: bool = False,
        rotation: float = 0,
        opacity: int = 100,
        label: str = "rectangle",
    ) -> ET.Element:
        style = (
            f"rounded={1 if rounded else 0};whiteSpace=wrap;html=1;fillColor={fill};strokeColor={stroke};"
            f"strokeWidth={stroke_width};rotation={rotation:.2f};opacity={opacity};"
        )
        return self.vertex(label, x, y, width, height, style)

    def ellipse(
        self,
        center_x: float,
        center_y: float,
        radius_x: float,
        radius_y: float | None = None,
        *,
        fill: str = TEXT,
        stroke: str = TEXT,
        stroke_width: float = 2,
        opacity: int = 100,
        label: str = "ellipse",
    ) -> ET.Element:
        radius_y = radius_x if radius_y is None else radius_y
        style = f"ellipse;whiteSpace=wrap;html=1;fillColor={fill};strokeColor={stroke};strokeWidth={stroke_width};opacity={opacity};"
        return self.vertex(label, center_x - radius_x, center_y - radius_y, 2 * radius_x, 2 * radius_y, style)

    def diamond(
        self,
        center_x: float,
        center_y: float,
        radius_x: float,
        radius_y: float,
        *,
        fill: str,
        stroke: str = TEXT,
        stroke_width: float = 2,
        label: str = "diamond",
    ) -> ET.Element:
        style = f"rhombus;whiteSpace=wrap;html=1;fillColor={fill};strokeColor={stroke};strokeWidth={stroke_width};"
        return self.vertex(label, center_x - radius_x, center_y - radius_y, 2 * radius_x, 2 * radius_y, style)

    def edge(
        self,
        points: list[tuple[float, float]],
        *,
        stroke: str = TEXT,
        width: float = 3,
        dashed: bool = False,
        curved: bool = False,
        end_arrow: str = "none",
        start_arrow: str = "none",
        label: str = "line",
        opacity: int = 100,
    ) -> ET.Element:
        if len(points) < 2:
            raise ValueError("An edge requires at least two points")
        style = (
            "edgeStyle=none;orthogonalLoop=0;jettySize=auto;html=1;rounded=0;"
            f"strokeColor={stroke};strokeWidth={width};dashed={1 if dashed else 0};"
            f"curved={1 if curved else 0};startArrow={start_arrow};endArrow={end_arrow};"
            f"endFill=1;startFill=1;opacity={opacity};"
        )
        if dashed:
            style += "dashPattern=12 8;"
        cell = ET.SubElement(
            self.root,
            "mxCell",
            {"id": self._id(label), "value": "", "style": style, "parent": self.editable_layer, "edge": "1"},
        )
        geometry = ET.SubElement(cell, "mxGeometry", {"relative": "1", "as": "geometry"})
        start_x, start_y = self._xy(*points[0])
        end_x, end_y = self._xy(*points[-1])
        ET.SubElement(geometry, "mxPoint", {"x": f"{start_x:.2f}", "y": f"{start_y:.2f}", "as": "sourcePoint"})
        ET.SubElement(geometry, "mxPoint", {"x": f"{end_x:.2f}", "y": f"{end_y:.2f}", "as": "targetPoint"})
        if len(points) > 2:
            array = ET.SubElement(geometry, "Array", {"as": "points"})
            for x, y in points[1:-1]:
                px, py = self._xy(x, y)
                ET.SubElement(array, "mxPoint", {"x": f"{px:.2f}", "y": f"{py:.2f}"})
        return cell


def build_figure_1(mxfile: ET.Element) -> None:
    page = DrawioPage(
        mxfile,
        "figure-1",
        "Hình 1 – Chuyển tiếp tại góc",
        1500,
        760,
        "Đỉnh gãy của đường gốc và đoạn chuyển tiếp cục bộ giữ nguyên tuyến nhưng phân bố biến thiên hướng trên một đoạn hữu hạn.",
        ASSET_DIR / "image1.png",
    )
    a0, vertex, b0 = (130, 610), (850, 610), (1290, 145)
    a, b = (520, 610), (1010, 440)
    controls = [a, (660, 610), (760, 610), (900, 555), (955, 495), b]
    page.edge([a0, vertex, b0], stroke=GRAY, width=7, dashed=True, label="đường-gốc")
    page.edge([a0, a], stroke=BLUE, width=10, label="đường-thẳng-trước")
    page.edge(bezier5(controls), stroke=BLUE, width=12, curved=True, label="đường-chuyển-tiếp")
    page.edge([b, b0], stroke=BLUE, width=10, label="đường-thẳng-sau")
    for point, name, dx, dy in ((a, "A", -48, 18), (vertex, "V", 8, 18), (b, "B", 15, -55)):
        page.ellipse(*point, 10, fill=ORANGE, stroke=TEXT, stroke_width=2, label=f"điểm-{name}")
        page.text(name, point[0] + dx, point[1] + dy, 55, 40, size=30, italic=True)
    # Show the signed heading change using co-located incoming/outgoing
    # direction vectors.  The incoming direction is extended through V so
    # that the angle is not confused with the interior angle of the polyline.
    outgoing_angle = math.atan2(b0[1] - vertex[1], b0[0] - vertex[0])
    direction_length = 150
    incoming_tip = (vertex[0] + direction_length, vertex[1])
    outgoing_tip = (
        vertex[0] + direction_length * math.cos(outgoing_angle),
        vertex[1] + direction_length * math.sin(outgoing_angle),
    )
    page.edge([vertex, incoming_tip], stroke="#444444", width=3, end_arrow="classic", label="vector-u")
    page.edge([vertex, outgoing_tip], stroke="#444444", width=3, end_arrow="classic", label="vector-v")
    page.text("u", 955, 602, 55, 42, size=29, italic=True, align="center")
    page.text("v", 940, 475, 55, 42, size=29, italic=True, align="center")

    angle_radius = 92
    angle_points = []
    for step in range(9):
        angle = outgoing_angle * step / 8
        angle_points.append(
            (vertex[0] + angle_radius * math.cos(angle), vertex[1] + angle_radius * math.sin(angle))
        )
    page.edge(
        angle_points,
        stroke=PURPLE,
        width=6,
        curved=True,
        end_arrow="classic",
        label="góc-delta-psi",
    )
    middle_angle = outgoing_angle / 2
    label_radius = 132
    label_center = (
        vertex[0] + label_radius * math.cos(middle_angle),
        vertex[1] + label_radius * math.sin(middle_angle),
    )
    page.text(
        "Δψ",
        label_center[0] - 48,
        label_center[1] - 25,
        96,
        50,
        size=30,
        color=PURPLE,
        italic=True,
        align="center",
    )
    page.edge([(330, 700), (450, 700)], stroke=GRAY, width=7, dashed=True, label="mẫu-đường-gốc")
    page.text("đường thô", 470, 676, 245, 50, size=29)
    page.edge([(770, 700), (890, 700)], stroke=BLUE, width=10, label="mẫu-đường-mượt")
    page.text("đường sau làm mượt", 910, 676, 430, 50, size=29)
    page.finish()


def build_figure_2(mxfile: ET.Element) -> None:
    page = DrawioPage(
        mxfile,
        "figure-2",
        "Hình 2 – Robot truyền động vi sai",
        1400,
        1010,
        "Mô hình động học robot hai bánh vi sai và quy ước vận tốc tuyến tính tại bánh.",
        ASSET_DIR / "image2.png",
    )
    page.rect(485, 260, 430, 390, fill="#D9EAF4", stroke="#222222", stroke_width=6, rounded=True, label="thân-robot")
    page.rect(435, 325, 55, 160, fill="#444444", stroke="#444444", label="bánh-trái")
    page.rect(910, 325, 55, 160, fill="#444444", stroke="#444444", label="bánh-phải")
    page.edge([(462, 215), (938, 215)], stroke="#333333", width=5, label="khoảng-cách-bánh")
    page.edge([(462, 195), (462, 235)], stroke="#333333", width=5, label="vạch-kích-thước-trái")
    page.edge([(938, 195), (938, 235)], stroke="#333333", width=5, label="vạch-kích-thước-phải")
    # O is the body reference point at the midpoint of the wheel axle.  Both
    # translational velocity v and yaw rate omega are defined at/about O.
    reference_point = (700, 405)
    page.edge([reference_point, (700, 78)], stroke=BLUE, width=9, end_arrow="classic", label="vector-v")
    page.text("v", 728, 68, 65, 55, size=38, color=BLUE, italic=True)
    page.edge([(462, 405), (462, 85)], stroke=ORANGE, width=8, end_arrow="classic", label="vector-vL")
    page.text("v<sub>L</sub>", 335, 68, 110, 55, size=38, color=ORANGE, italic=True, align="right")
    page.edge([(938, 405), (938, 85)], stroke=GREEN, width=8, end_arrow="classic", label="vector-vR")
    page.text("v<sub>R</sub>", 970, 68, 120, 55, size=38, color=GREEN, italic=True)
    page.edge(
        [(805, 505), (840, 460), (845, 405), (835, 350), (805, 305)],
        stroke=PURPLE,
        width=8,
        curved=True,
        end_arrow="classic",
        label="omega-quanh-O",
    )
    page.ellipse(*reference_point, 7, fill=PURPLE, stroke="#FFFFFF", stroke_width=2, label="tâm-quay-O")
    page.text("O", 660, 410, 42, 42, size=27, color=PURPLE, italic=True, align="center")
    page.text("ω", 842, 360, 70, 60, size=38, color=PURPLE, italic=True)
    page.edge([(120, 725), (200, 725)], stroke="#333333", width=5, label="mẫu-b")
    page.text("b: khoảng cách tâm hai bánh", 220, 695, 575, 60, size=32)
    page.edge([(120, 790), (200, 790)], stroke=ORANGE, width=9, label="mẫu-vL")
    page.text("v<sub>L</sub>: vận tốc bánh trái", 220, 760, 500, 60, size=32)
    page.edge([(755, 790), (835, 790)], stroke=GREEN, width=9, label="mẫu-vR")
    page.text("v<sub>R</sub>: vận tốc bánh phải", 855, 760, 500, 60, size=32)
    page.text("v<sub>L</sub> = v(1 − bκ/2)", 300, 875, 390, 60, size=32, align="center")
    page.text("v<sub>R</sub> = v(1 + bκ/2)", 710, 875, 430, 60, size=32, align="center")
    page.finish()


def build_figure_3(mxfile: ET.Element) -> None:
    page = DrawioPage(
        mxfile,
        "figure-3",
        "Hình 3 – Vùng quét hình bao robot",
        1400,
        1020,
        "Vùng quét được tạo từ hợp các hình bao robot lấy mẫu dọc đường tâm; va chạm được kiểm tra trên toàn vùng này.",
        ASSET_DIR / "image3.png",
    )
    curve = []
    for index in range(15):
        t = index / 14
        curve.append((150 + 1000 * t, 650 - 440 * (3 * t * t - 2 * t * t * t)))
    page.edge(curve, stroke=BLUE, width=10, curved=True, label="đường-tâm")
    for index, t in enumerate((0.05, 0.22, 0.39, 0.56, 0.73, 0.90), start=1):
        center_x = 150 + 1000 * t
        center_y = 650 - 440 * (3 * t * t - 2 * t * t * t)
        theta = math.degrees(math.atan2(-440 * (6 * t - 6 * t * t), 1000))
        page.rect(
            center_x - 95,
            center_y - 67.5,
            190,
            135,
            fill=BLUE,
            stroke="#004873",
            stroke_width=2,
            rotation=theta,
            opacity=30,
            label=f"hình-bao-{index}",
        )
    page.rect(1240, 80, 130, 250, fill="#4D004B", stroke=TEXT, stroke_width=5, label="ô-vật-cản")
    page.rect(115, 766, 80, 48, fill=BLUE, stroke="#004873", stroke_width=3, opacity=30, label="mẫu-hình-bao")
    page.text("Các tư thế hình bao robot được lấy mẫu", 220, 755, 750, 70, size=31)
    page.edge([(115, 865), (235, 865)], stroke=BLUE, width=10, label="mẫu-đường-tâm")
    page.text("Đường tâm robot r(s)", 255, 835, 480, 60, size=31)
    page.rect(115, 916, 80, 48, fill="#4D004B", stroke=TEXT, stroke_width=3, label="mẫu-vật-cản")
    page.text("Ô vật cản", 220, 905, 300, 70, size=31)
    page.finish()


def build_figure_4(mxfile: ET.Element) -> None:
    page = DrawioPage(
        mxfile,
        "figure-4",
        "Hình 4 – Bézier bậc năm",
        1600,
        900,
        "Cấu trúc sáu điểm điều khiển. Ba điểm đầu và ba điểm cuối thẳng hàng, cách đều nên độ cong tại hai đầu bằng không.",
        ASSET_DIR / "image4.png",
    )
    angle = math.radians(58)
    direction = (math.cos(angle), math.sin(angle))
    d = 1.0
    q = 0.34
    a = (-d, 0.0)
    b = (d * direction[0], d * direction[1])
    points = [
        a,
        (a[0] + q, a[1]),
        (a[0] + 2 * q, a[1]),
        (b[0] - 2 * q * direction[0], b[1] - 2 * q * direction[1]),
        (b[0] - q * direction[0], b[1] - q * direction[1]),
        b,
    ]

    def cv(point: tuple[float, float]) -> tuple[float, float]:
        return 790 + 580 * point[0], 520 - 500 * point[1]

    page.edge([cv((-1.25, 0)), cv((0.08, 0))], stroke="#555555", width=5, label="cạnh-vào")
    page.edge([cv((0, 0)), cv((0.72 * direction[0], 0.72 * direction[1]))], stroke="#555555", width=5, label="cạnh-ra")
    control_points = [cv(point) for point in points]
    page.edge(control_points, stroke=LIGHT_GRAY, width=4, label="đa-giác-điều-khiển")
    page.edge(bezier5(control_points, 16), stroke=BLUE, width=11, curved=True, label="đường-bezier")
    label_positions = [
        (control_points[0][0] - 55, control_points[0][1] + 70),
        (control_points[1][0], control_points[1][1] + 78),
        (control_points[2][0] + 12, control_points[2][1] + 92),
        (control_points[3][0] + 65, control_points[3][1] - 86),
        (control_points[4][0] + 78, control_points[4][1] - 78),
        (control_points[5][0] + 82, control_points[5][1] - 70),
    ]
    for index, (point, label_position) in enumerate(zip(control_points, label_positions)):
        page.ellipse(*point, 12, fill=ORANGE, stroke=TEXT, stroke_width=3, label=f"điểm-P{index}")
        page.edge([point, label_position], stroke="#777777", width=2, label=f"đường-dẫn-P{index}")
        page.text(
            f"P<sub>{index}</sub>",
            label_position[0] - 35,
            label_position[1] - 24,
            70,
            48,
            size=30,
            align="center",
            background="#FFFFFF",
        )
    vertex = cv((0, 0))
    vertex_label = (vertex[0] + 45, vertex[1] + 62)
    page.ellipse(*vertex, 9, fill="#222222", stroke="#222222", label="đỉnh-V")
    page.edge([vertex, vertex_label], stroke="#777777", width=2, label="đường-dẫn-V")
    page.text("V", vertex_label[0] - 30, vertex_label[1] - 24, 60, 48, size=30, align="center", background="#FFFFFF")
    legend_y = 820
    page.edge([(150, legend_y), (260, legend_y)], stroke=LIGHT_GRAY, width=4, label="mẫu-đa-giác")
    page.text("Đa giác điều khiển", 280, legend_y - 30, 330, 60, size=30)
    page.edge([(650, legend_y), (760, legend_y)], stroke=BLUE, width=11, label="mẫu-bezier")
    page.text("Đường Bézier bậc năm", 780, legend_y - 30, 420, 60, size=30)
    page.ellipse(1252, legend_y, 12, fill=ORANGE, stroke=TEXT, stroke_width=3, label="mẫu-điểm")
    page.text("Điểm điều khiển", 1280, legend_y - 30, 290, 60, size=30)
    page.finish()


def build_figure_5(mxfile: ET.Element) -> None:
    page = DrawioPage(
        mxfile,
        "figure-5",
        "Hình 5 – Ngân sách cạnh chung",
        1500,
        760,
        "Ngân sách chiều dài trên cạnh chung của hai góc kề nhau. Khoảng trống thực tế gᵢ phải thỏa gᵢ ≥ m, tương đương dᵢ + dᵢ₊₁ + m ≤ Lᵢ.",
        ASSET_DIR / "image5.png",
    )
    left_previous = (90, 640)
    left_vertex, right_vertex = (250, 430), (1250, 430)
    right_next = (1410, 170)
    left_cut, right_cut = (560, 430), (930, 430)

    page.text(
        "Điều kiện tương thích: d<sub>i</sub> + d<sub>i+1</sub> + m ≤ L<sub>i</sub>",
        350,
        25,
        800,
        55,
        size=30,
        bold=True,
        align="center",
    )

    # Original polyline is retained as a dashed reference.  The blue path
    # explicitly meets the shared edge at the two cut points.
    page.edge(
        [left_previous, left_vertex, right_vertex, right_next],
        stroke=GRAY,
        width=6,
        dashed=True,
        label="đường-gốc",
    )

    incoming_vector = (left_vertex[0] - left_previous[0], left_vertex[1] - left_previous[1])
    incoming_norm = math.hypot(*incoming_vector)
    incoming_unit = (incoming_vector[0] / incoming_norm, incoming_vector[1] / incoming_norm)
    outgoing_vector = (right_next[0] - right_vertex[0], right_next[1] - right_vertex[1])
    outgoing_norm = math.hypot(*outgoing_vector)
    outgoing_unit = (outgoing_vector[0] / outgoing_norm, outgoing_vector[1] / outgoing_norm)
    left_start = (
        left_vertex[0] - 145 * incoming_unit[0],
        left_vertex[1] - 145 * incoming_unit[1],
    )
    right_end = (
        right_vertex[0] + 155 * outgoing_unit[0],
        right_vertex[1] + 155 * outgoing_unit[1],
    )
    left_transition_controls = [
        left_start,
        (left_start[0] + 55 * incoming_unit[0], left_start[1] + 55 * incoming_unit[1]),
        (left_start[0] + 110 * incoming_unit[0], left_start[1] + 110 * incoming_unit[1]),
        (left_cut[0] - 110, left_cut[1]),
        (left_cut[0] - 55, left_cut[1]),
        left_cut,
    ]
    right_transition_controls = [
        right_cut,
        (right_cut[0] + 55, right_cut[1]),
        (right_cut[0] + 110, right_cut[1]),
        (right_end[0] - 110 * outgoing_unit[0], right_end[1] - 110 * outgoing_unit[1]),
        (right_end[0] - 55 * outgoing_unit[0], right_end[1] - 55 * outgoing_unit[1]),
        right_end,
    ]
    page.edge([left_previous, left_start], stroke=BLUE, width=10, label="đường-mượt-trước")
    page.edge(
        bezier5(left_transition_controls, 14),
        stroke=BLUE,
        width=12,
        curved=True,
        label="chuyển-tiếp-trái",
    )
    page.edge([left_cut, right_cut], stroke=BLUE, width=10, label="đoạn-thẳng-còn-lại")
    page.edge(
        bezier5(right_transition_controls, 14),
        stroke=BLUE,
        width=12,
        curved=True,
        label="chuyển-tiếp-phải",
    )
    page.edge([right_end, right_next], stroke=BLUE, width=10, label="đường-mượt-sau")

    for point, name in ((left_vertex, "V<sub>i</sub>"), (right_vertex, "V<sub>i+1</sub>")):
        page.ellipse(*point, 11, fill=TEXT, stroke=TEXT, label="đỉnh")
        page.text(name, point[0] - 70, point[1] - 85, 140, 55, size=31, italic=True, align="center")
    page.ellipse(*left_cut, 9, fill=ORANGE, stroke="#FFFFFF", stroke_width=2, label="điểm-cắt-trái")
    page.ellipse(*right_cut, 9, fill=GREEN, stroke="#FFFFFF", stroke_width=2, label="điểm-cắt-phải")

    y_segments, y_total = 570, 675
    for index, x in enumerate((left_vertex[0], left_cut[0], right_cut[0], right_vertex[0])):
        page.edge(
            [(x, 445), (x, y_segments - 15)],
            stroke="#AAAAAA",
            width=2,
            dashed=True,
            label=f"đường-chiếu-{index}",
        )
    page.edge([(left_vertex[0], y_total), (right_vertex[0], y_total)], stroke="#333333", width=4, label="chiều-dài-Li")
    for index, x in enumerate((left_vertex[0], right_vertex[0])):
        page.edge([(x, y_total - 14), (x, y_total + 14)], stroke="#333333", width=4, label=f"vạch-Li-{index}")
    page.text("L<sub>i</sub>", 700, y_total + 10, 100, 55, size=31, italic=True, align="center", valign="top")
    page.edge([(left_vertex[0], y_segments), (left_cut[0], y_segments)], stroke=ORANGE, width=9, label="đoạn-di")
    page.edge([(left_cut[0], y_segments), (right_cut[0], y_segments)], stroke=PURPLE, width=9, label="khoảng-trống-gi")
    page.edge([(right_cut[0], y_segments), (right_vertex[0], y_segments)], stroke=GREEN, width=9, label="đoạn-di1")
    for index, x in enumerate((left_vertex[0], left_cut[0], right_cut[0], right_vertex[0])):
        page.edge([(x, y_segments - 13), (x, y_segments + 13)], stroke="#333333", width=3, label=f"vạch-đoạn-{index}")
    page.text("d<sub>i</sub>", 350, y_segments - 72, 110, 55, size=31, color=ORANGE, italic=True, align="center")
    page.text("g<sub>i</sub>", 695, y_segments - 72, 100, 55, size=31, color=PURPLE, italic=True, align="center")
    page.text("d<sub>i+1</sub>", 1030, y_segments - 72, 140, 55, size=31, color=GREEN, italic=True, align="center")
    page.text("g<sub>i</sub> ≥ m", 660, y_segments + 8, 170, 48, size=26, color=PURPLE, italic=True, align="center", valign="top")
    page.finish()


def main() -> None:
    mxfile = ET.Element(
        "mxfile",
        {
            "host": "app.diagrams.net",
            "modified": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "agent": "Codex editable figure rebuilder",
            "version": "26.0.16",
            "type": "device",
            "compressed": "false",
        },
    )
    build_figure_1(mxfile)
    build_figure_2(mxfile)
    build_figure_3(mxfile)
    build_figure_4(mxfile)
    build_figure_5(mxfile)
    ET.indent(mxfile, space="  ")
    OUTPUT.write_bytes(ET.tostring(mxfile, encoding="utf-8", xml_declaration=True))
    print(OUTPUT)
    SEPARATE_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for index, diagram in enumerate(mxfile.findall("diagram"), start=1):
        single_file = ET.Element("mxfile", dict(mxfile.attrib))
        single_file.append(copy.deepcopy(diagram))
        ET.indent(single_file, space="  ")
        path = SEPARATE_OUTPUT_DIR / f"Hinh_{index}_editable.drawio"
        path.write_bytes(ET.tostring(single_file, encoding="utf-8", xml_declaration=True))
        print(path)


if __name__ == "__main__":
    main()
