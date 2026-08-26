#!/usr/bin/env python3
"""Create diagrams.net sources for the Vietnamese ICEEIS ver2 figures."""

from __future__ import annotations

import shutil
from pathlib import Path
from xml.etree import ElementTree as ET

import create_drawio_from_docx_images as raster_drawio
import create_editable_drawio_figures_1_5 as editable_drawio


ROOT = Path(__file__).resolve().parents[1]
VER2 = ROOT / "final_bao_ICEEIS/ver2"
ASSETS = VER2 / "assets"
REFERENCE_ASSETS = ASSETS / "drawio_reference"
DOCX = VER2 / "ICEEIS_2026_PSTMO_ver2_tieng_Viet.docx"
ALL_FIGURES = VER2 / "PSTMO_ver2_tat_ca_hinh.drawio"
EDITABLE_FIGURES = VER2 / "PSTMO_ver2_hinh_ky_thuat_editable.drawio"
EDITABLE_SEPARATE = VER2 / "PSTMO_ver2_hinh_ky_thuat_editable"
VER1 = ROOT / "final_bao_ICEEIS/ver1"
VER1_TECHNICAL = VER1 / "ICEEIS_2026_PSTMO_figures_1_5_editable"
PRESERVED_TECHNICAL = VER2 / "PSTMO_ver2_hinh_1_5_goc_tu_ver1"


def prepare_reference_images() -> None:
    REFERENCE_ASSETS.mkdir(parents=True, exist_ok=True)
    sources = {
        1: ASSETS / "ver1_hinh_1.png",
        2: ASSETS / "ver1_hinh_2.png",
        3: ASSETS / "ver1_hinh_3.png",
        4: ASSETS / "ver1_hinh_4.png",
        5: ASSETS / "ver1_hinh_5.png",
    }
    for index, source in sources.items():
        shutil.copyfile(source, REFERENCE_ASSETS / f"image{index}.png")


def preserve_ver1_drawio_sources() -> None:
    """Carry the user's edited ver1 Draw.io sources into the ver2 handoff."""
    shutil.copytree(VER1_TECHNICAL, PRESERVED_TECHNICAL, dirs_exist_ok=True)


def keep_exactly_eight_figure_pages() -> None:
    """Remove the contact-sheet page so the file matches Hình 1–8 exactly."""
    tree = ET.parse(ALL_FIGURES)
    root = tree.getroot()
    for diagram in list(root.findall("diagram")):
        if diagram.get("id") == "summary":
            root.remove(diagram)
    pages = root.findall("diagram")
    if len(pages) != 8:
        raise RuntimeError(f"Expected exactly 8 figure pages, found {len(pages)}")
    ET.indent(tree, space="  ")
    tree.write(ALL_FIGURES, encoding="utf-8", xml_declaration=True)


def main() -> None:
    if not DOCX.exists():
        raise FileNotFoundError(DOCX)
    prepare_reference_images()
    count = raster_drawio.create_drawio(DOCX, ALL_FIGURES)
    keep_exactly_eight_figure_pages()
    print(f"{ALL_FIGURES} ({count} hình từ bản thảo)")

    editable_drawio.ASSET_DIR = REFERENCE_ASSETS
    editable_drawio.OUTPUT = EDITABLE_FIGURES
    editable_drawio.SEPARATE_OUTPUT_DIR = EDITABLE_SEPARATE
    editable_drawio.main()
    preserve_ver1_drawio_sources()


if __name__ == "__main__":
    main()
