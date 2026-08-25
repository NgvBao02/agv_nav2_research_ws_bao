#!/usr/bin/env python3
"""Create the concise Section V version without redundant Figures 9 and 10."""

from __future__ import annotations

import copy
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from docx import Document
from docx.oxml.ns import qn


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "final_bao_ICEEIS" / "ICEEIS_2026_PSTMO_final_section_V_redesigned.docx"
OUTPUT = ROOT / "final_bao_ICEEIS" / "ICEEIS_2026_PSTMO_final_section_V_3_figures.docx"
FIGURE_DIR = ROOT / "final_bao_ICEEIS" / "section_v_redesigned_figures"
SOURCE_DRAWIO = FIGURE_DIR / "ICEEIS_2026_PSTMO_section_V_figures_6_10_editable.drawio"
OUTPUT_DRAWIO = FIGURE_DIR / "ICEEIS_2026_PSTMO_section_V_figures_6_8_editable.drawio"
SOURCE_SEPARATE = FIGURE_DIR / "drawio_separate"
OUTPUT_SEPARATE = FIGURE_DIR / "drawio_final_3_figures"


def remove_paragraph(paragraph) -> None:
    element = paragraph._element
    element.getparent().remove(element)
    paragraph._p = paragraph._element = None


def embedded_relationship_ids(paragraph) -> set[str]:
    return {
        blip.get(qn("r:embed"))
        for blip in paragraph._element.xpath(".//a:blip")
        if blip.get(qn("r:embed"))
    }


def remove_figures_from_document() -> None:
    document = Document(SOURCE)
    paragraphs = document.paragraphs
    targets = []
    relationship_ids: set[str] = set()

    for caption_prefix in ("Hình 9.", "Hình 10."):
        caption_index = next(i for i, p in enumerate(paragraphs) if p.text.startswith(caption_prefix))
        image_paragraph = paragraphs[caption_index - 1]
        relationship_ids.update(embedded_relationship_ids(image_paragraph))
        targets.extend((image_paragraph, paragraphs[caption_index]))

    discussion = next(
        p for p in paragraphs if p.text.startswith("Hình 9 cho thấy lợi thế trung bình của PSTMO")
    )
    targets.append(discussion)

    for paragraph in targets:
        remove_paragraph(paragraph)
    for relationship_id in relationship_ids:
        document.part.drop_rel(relationship_id)

    document.core_properties.comments = (
        "Phần V dùng Hình 6–8 cho minh họa trực quan và Bảng IV cho so sánh định lượng; "
        "Hình 9–10 được loại bỏ vì trùng thông tin với bảng."
    )
    document.save(OUTPUT)


def create_three_page_drawio() -> None:
    tree = ET.parse(SOURCE_DRAWIO)
    root = tree.getroot()
    for diagram in list(root.findall("diagram")):
        if diagram.get("name", "").startswith(("Hình 9", "Hình 10")):
            root.remove(diagram)
    if len(root.findall("diagram")) != 3:
        raise RuntimeError("The concise Draw.io file must contain exactly three diagrams")
    tree.write(OUTPUT_DRAWIO, encoding="UTF-8", xml_declaration=True)

    OUTPUT_SEPARATE.mkdir(parents=True, exist_ok=True)
    for number in (6, 7, 8):
        source = SOURCE_SEPARATE / f"Hinh_{number}_editable.drawio"
        shutil.copy2(source, OUTPUT_SEPARATE / source.name)


def main() -> None:
    remove_figures_from_document()
    create_three_page_drawio()
    print(OUTPUT)
    print(OUTPUT_DRAWIO)


if __name__ == "__main__":
    main()
