#!/usr/bin/env python3
"""Extract DOCX images and place them on editable diagrams.net pages."""

from __future__ import annotations

import argparse
import base64
import mimetypes
import struct
import zipfile
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from xml.etree import ElementTree as ET


NS = {
    "a": "http://schemas.openxmlformats.org/drawingml/2006/main",
    "pr": "http://schemas.openxmlformats.org/package/2006/relationships",
    "r": "http://schemas.openxmlformats.org/officeDocument/2006/relationships",
    "w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main",
}


def image_size(blob: bytes) -> tuple[int, int]:
    if blob.startswith(b"\x89PNG\r\n\x1a\n"):
        return struct.unpack(">II", blob[16:24])
    if blob[:2] == b"\xff\xd8":
        offset = 2
        while offset + 9 < len(blob):
            if blob[offset] != 0xFF:
                offset += 1
                continue
            marker = blob[offset + 1]
            offset += 2
            if marker in (0xD8, 0xD9):
                continue
            length = int.from_bytes(blob[offset : offset + 2], "big")
            if 0xC0 <= marker <= 0xC3:
                height = int.from_bytes(blob[offset + 3 : offset + 5], "big")
                width = int.from_bytes(blob[offset + 5 : offset + 7], "big")
                return width, height
            offset += length
    raise ValueError("Unsupported image format for dimension detection")


def paragraph_text(paragraph: ET.Element) -> str:
    return "".join(node.text or "" for node in paragraph.findall(".//w:t", NS)).strip()


def extract_figures(docx_path: Path) -> list[dict[str, object]]:
    with zipfile.ZipFile(docx_path) as archive:
        rels_root = ET.fromstring(archive.read("word/_rels/document.xml.rels"))
        relationships = {
            rel.attrib["Id"]: rel.attrib["Target"]
            for rel in rels_root.findall("pr:Relationship", NS)
            if rel.attrib.get("Type", "").endswith("/image")
        }
        document_root = ET.fromstring(archive.read("word/document.xml"))
        paragraphs = document_root.findall(".//w:p", NS)

        ordered: list[tuple[str, str]] = []
        for index, paragraph in enumerate(paragraphs):
            blips = paragraph.findall(".//a:blip", NS)
            if not blips:
                continue
            caption = ""
            for following in paragraphs[index + 1 : index + 6]:
                text = paragraph_text(following)
                if text:
                    if text.startswith(("Hình ", "Figure ")):
                        caption = text
                    break
            for blip in blips:
                relation_id = blip.attrib.get(f"{{{NS['r']}}}embed")
                if relation_id in relationships:
                    ordered.append((relationships[relation_id], caption))

        seen: set[str] = set()
        figures: list[dict[str, object]] = []
        for target, caption in ordered:
            normalized = str(PurePosixPath("word") / PurePosixPath(target))
            if normalized in seen:
                continue
            seen.add(normalized)
            blob = archive.read(normalized)
            width, height = image_size(blob)
            figures.append(
                {
                    "filename": PurePosixPath(target).name,
                    "blob": blob,
                    "width": width,
                    "height": height,
                    "caption": caption,
                }
            )

        media_names = sorted(
            (name for name in archive.namelist() if name.startswith("word/media/") and not name.endswith("/")),
            key=lambda name: name.lower(),
        )
        for name in media_names:
            if name in seen:
                continue
            blob = archive.read(name)
            width, height = image_size(blob)
            figures.append(
                {
                    "filename": PurePosixPath(name).name,
                    "blob": blob,
                    "width": width,
                    "height": height,
                    "caption": "",
                }
            )
        return figures


def add_cell(
    root: ET.Element,
    cell_id: str,
    *,
    value: str = "",
    style: str = "",
    x: float = 0,
    y: float = 0,
    width: float = 0,
    height: float = 0,
    vertex: bool = True,
) -> ET.Element:
    attrs = {"id": cell_id, "value": value, "style": style, "parent": "1"}
    if vertex:
        attrs["vertex"] = "1"
    cell = ET.SubElement(root, "mxCell", attrs)
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
    return cell


def graph_model(page_width: int, page_height: int) -> tuple[ET.Element, ET.Element]:
    model = ET.Element(
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
    root = ET.SubElement(model, "root")
    ET.SubElement(root, "mxCell", {"id": "0"})
    ET.SubElement(root, "mxCell", {"id": "1", "parent": "0"})
    return model, root


def data_uri(filename: str, blob: bytes) -> str:
    mime = mimetypes.guess_type(filename)[0] or "application/octet-stream"
    encoded = base64.b64encode(blob).decode("ascii")
    # diagrams.net's embedded-image form intentionally omits the normal
    # ';base64' token because semicolons delimit mxGraph style properties.
    return f"data:{mime},{encoded}"


def figure_title(index: int, caption: str) -> str:
    if caption:
        head = caption.split(".", 1)[0].strip()
        if head:
            return head
    return f"Hình {index}"


def add_summary_page(mxfile: ET.Element, figures: list[dict[str, object]]) -> None:
    cols = 2
    page_width = 1300
    cell_width = 560
    cell_height = 500
    rows = (len(figures) + cols - 1) // cols
    page_height = 110 + rows * cell_height + 70
    diagram = ET.SubElement(mxfile, "diagram", {"id": "summary", "name": "Tất cả hình"})
    model, root = graph_model(page_width, page_height)
    add_cell(
        root,
        "summary-title",
        value="Tất cả hình trích từ ICEEIS_2026_PSTMO_final.docx",
        style="text;html=1;strokeColor=none;fillColor=none;align=center;verticalAlign=middle;fontSize=24;fontStyle=1;",
        x=100,
        y=25,
        width=1100,
        height=45,
    )
    for index, figure in enumerate(figures, start=1):
        col = (index - 1) % cols
        row = (index - 1) // cols
        base_x = 70 + col * 610
        base_y = 100 + row * cell_height
        original_w = int(figure["width"])
        original_h = int(figure["height"])
        scale = min(500 / original_w, 385 / original_h)
        display_w = original_w * scale
        display_h = original_h * scale
        x = base_x + (cell_width - display_w) / 2
        add_cell(
            root,
            f"summary-image-{index}",
            value="",
            style=(
                "shape=image;html=1;verticalLabelPosition=bottom;verticalAlign=top;"
                f"aspect=fixed;image={data_uri(str(figure['filename']), bytes(figure['blob']))};"
            ),
            x=x,
            y=base_y + 45,
            width=display_w,
            height=display_h,
        )
        add_cell(
            root,
            f"summary-label-{index}",
            value=figure_title(index, str(figure["caption"])),
            style="text;html=1;strokeColor=none;fillColor=none;align=center;verticalAlign=middle;fontSize=18;fontStyle=1;",
            x=base_x,
            y=base_y,
            width=cell_width,
            height=35,
        )
    diagram.append(model)


def add_individual_page(mxfile: ET.Element, index: int, figure: dict[str, object]) -> None:
    original_w = int(figure["width"])
    original_h = int(figure["height"])
    scale = min(1.0, 1200 / original_w, 800 / original_h)
    display_w = original_w * scale
    display_h = original_h * scale
    page_width = max(1100, round(display_w + 160))
    page_height = round(display_h + 270)
    title = figure_title(index, str(figure["caption"]))
    diagram = ET.SubElement(mxfile, "diagram", {"id": f"figure-{index}", "name": title})
    model, root = graph_model(page_width, page_height)
    image_x = (page_width - display_w) / 2
    add_cell(
        root,
        f"title-{index}",
        value=title,
        style="text;html=1;strokeColor=none;fillColor=none;align=center;verticalAlign=middle;fontSize=24;fontStyle=1;",
        x=80,
        y=25,
        width=page_width - 160,
        height=45,
    )
    add_cell(
        root,
        f"image-{index}",
        value="",
        style=(
            "shape=image;html=1;verticalLabelPosition=bottom;verticalAlign=top;"
            f"aspect=fixed;image={data_uri(str(figure['filename']), bytes(figure['blob']))};"
        ),
        x=image_x,
        y=85,
        width=display_w,
        height=display_h,
    )
    caption = str(figure["caption"])
    if caption:
        add_cell(
            root,
            f"caption-{index}",
            value=caption,
            style="text;html=1;whiteSpace=wrap;strokeColor=none;fillColor=none;align=center;verticalAlign=top;fontSize=17;fontStyle=2;",
            x=80,
            y=display_h + 105,
            width=page_width - 160,
            height=85,
        )
    diagram.append(model)


def create_drawio(docx_path: Path, output_path: Path) -> int:
    figures = extract_figures(docx_path)
    mxfile = ET.Element(
        "mxfile",
        {
            "host": "app.diagrams.net",
            "modified": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "agent": "Codex DOCX image importer",
            "version": "26.0.16",
            "type": "device",
            "compressed": "false",
        },
    )
    add_summary_page(mxfile, figures)
    for index, figure in enumerate(figures, start=1):
        add_individual_page(mxfile, index, figure)
    ET.indent(mxfile, space="  ")
    output_path.write_bytes(ET.tostring(mxfile, encoding="utf-8", xml_declaration=True))
    return len(figures)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("docx", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    count = create_drawio(args.docx.resolve(), args.output.resolve())
    print(f"Created {args.output} with {count} images and {count + 1} pages")


if __name__ == "__main__":
    main()
