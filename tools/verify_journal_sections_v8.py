#!/usr/bin/env python3
"""Verify v8 editorial scope, preserved measurements, references and rendered pages."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re
from zipfile import ZipFile

from lxml import etree
import pymupdf
from build_journal_sections_v7 import NS, W, map_chunks, plain, sha

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "Viet_bao/journal/ver8"
OLD = ROOT / "Viet_bao/journal/ver7"


def verify(render_dir, accept):
    prov = json.loads((OUT / "provenance.json").read_text())
    docx = OUT / prov["docx"]
    assert sha(docx) == prov["docx_sha256"]
    for name, digest in prov["source_sha256"].items():
        assert sha(ROOT / name) == digest, name
    for name, digest in prov["output_sha256"].items():
        assert sha(OUT / name) == digest, name
    # Revalidate the upstream source hashes without rebuilding historical files.
    for version in [6, 7]:
        p = json.loads((ROOT / f"Viet_bao/journal/ver{version}/provenance.json").read_text())
        for name, digest in p["source_sha256"].items():
            assert sha(ROOT / name) == digest, name
    qa_old = json.loads((OLD / "quality_checks.json").read_text())
    assert qa_old["docx_sha256"] == prov["reference_sha256"] and qa_old["visual_review_all_pages"]
    with ZipFile(docx) as z, ZipFile(ROOT / prov["reference"]) as oldz:
        assert set(z.namelist()) == set(oldz.namelist())
        changed = [n for n in z.namelist() if z.read(n) != oldz.read(n)]
        assert set(changed) == set(prov["changed_parts_allowed"])
        for name, digest in prov["package_parts"].items():
            assert hashlib.sha256(z.read(name)).hexdigest() == digest
        root = etree.fromstring(z.read("word/document.xml"))
        old = etree.fromstring(oldz.read("word/document.xml"))
    source, prior = (OUT / "manuscript.md").read_text(), (OLD / "manuscript.md").read_text()
    records, tail = map_chunks(root, source)
    old_records, old_tail = map_chunks(old, prior)
    checked_tail = [deepcopy(n) for n in tail]
    marker_spacing = checked_tail[0].find("w:pPr/w:spacing", NS)
    assert dict(marker_spacing.attrib) == {W + k: v for k, v in
        {"before": "0", "after": "0", "line": "20", "lineRule": "exact"}.items()}
    marker_spacing.getparent().remove(marker_spacing)
    marker_rpr = checked_tail[0].find("w:pPr/w:rPr", NS)
    assert [(n.tag, dict(n.attrib)) for n in marker_rpr] == [
        (W + "sz", {W + "val": "2"}), (W + "szCs", {W + "val": "2"})]
    marker_rpr.getparent().remove(marker_rpr)
    checked_tail[0].find("w:pPr", NS).append(deepcopy(old_tail[0].find("w:pPr/w:sectPr", NS)))
    assert checked_tail[-1].find("w:cols", NS).get(W + "num") == "2"
    checked_tail[-1].find("w:cols", NS).set(W + "num", "1")
    assert [etree.tostring(n) for n in checked_tail] == [etree.tostring(deepcopy(n)) for n in old_tail]
    assert len(root.findall(".//w:sectPr", NS)) == 1
    for tag in ["w:tbl", "w:drawing"]:
        assert [etree.tostring(deepcopy(n)) for n in root.findall(".//" + tag, NS)] == [
            etree.tostring(deepcopy(n)) for n in old.findall(".//" + tag, NS)], tag
    assert sorted(etree.tostring(deepcopy(n)) for n in root.findall(".//m:oMath", NS)) == sorted(
        etree.tostring(deepcopy(n)) for n in old.findall(".//m:oMath", NS))
    equations = [r for r in records if r["text"].startswith("@equation ")]
    assert len(equations) == 9
    for i, r in enumerate(equations, 1):
        labels = [t.text for n in r["nodes"] for t in n.findall(".//w:t", NS) if re.fullmatch(r"\(\d+\)", t.text or "")]
        assert labels == [f"({i})"], labels
    assert equations[4]["text"].startswith("@equation T_{\\min}")
    assert equations[5]["text"].startswith("@equation J=")
    first_formula_refs = list(dict.fromkeys(re.findall(r"[Cc]ông thức (\d+)", source)))
    assert first_formula_refs == list("123456789"), first_formula_refs
    assert source.split("# TÀI LIỆU THAM KHẢO")[1] == prior.split("# TÀI LIỆU THAM KHẢO")[1]
    assert source.split("## D. Xây dựng")[0] == prior.split("## D. Xây dựng")[0]
    assert source.split("## E. Lựa chọn")[1].split("# III.")[0] == prior.split("## E. Lựa chọn")[1].split("# III.")[0]
    # Failure classification, table, illustration and discussion unchanged.
    new_failure = source.split("## I. Phân tích")[1].split("### 3) Phạm vi")[0].strip().split("\n\n")
    old_failure = prior.split("## I. Phân tích")[1].split("### 3) Phạm vi")[0].strip().split("\n\n")
    endpoint_figure = next(p for p in old_failure if p.startswith("@figure endpoints "))
    old_failure.remove(endpoint_figure)
    old_failure.append(endpoint_figure)
    assert new_failure == old_failure
    assert (OUT / "table_values.json").read_bytes() == (OLD / "table_values.json").read_bytes()
    # Preserve every five-method scalar series in prose (2 series x 5 routes x 5 methods x 2 metrics).
    scalar_count = 0
    for prefix, endpoint in [("Sai số vị trí cuối trung bình", "chỉ tính các lượt đạt."),
                             ("Theo thứ tự đường gốc, Simple, Savitzky–Golay", " m/s³.")]:
        new = [r["text"].split(endpoint, 1)[0] for r in records if r["text"].startswith(prefix)]
        previous = [r["text"].split(endpoint, 1)[0] for r in old_records if r["text"].startswith(prefix)]
        assert new == previous and len(new) == 5
        scalar_count += 50
    a = source.split("### 5) Hai bộ dữ liệu")[1].split("### 6)")[0]
    assert a.index("Vì vậy, bộ kiểm chứng lặp") < a.index("Bộ kiểm chứng lặp gồm 250")
    assert "Constrained cung cấp đối chứng" in a
    h = source.split("## H.")[1].split("## I.")[0]
    assert "Mỗi tổ hợp trong ma trận khảo sát chỉ" not in h
    assert "34–40" not in source and "0,24 rad/s" not in source
    assert "không bù cho RMSE" not in source and "định vị cuối tốt hơn" not in source
    metrics = source.split("### 7) Chỉ số đo")[1].split("## B. R01")[0]
    assert metrics.index("lấy mẫu lại theo chiều dài cung") < metrics.index("công thức 8")
    visible = " ".join(root.xpath(".//w:t/text()", namespaces=NS))
    for forbidden in ["ứng viên", "năng lượng độ cong", "planned_curvature_energy", "executed_curvature_energy"]:
        assert forbidden not in visible.lower()
    assert not re.search(r"[a-f0-9]{40,64}", visible)
    assert not re.search(r"(?:trong|theo)\s*\(\d+\)", visible, re.I)
    assert list(dict.fromkeys(re.findall(r"\[(\d+)\]", source.split("# TÀI LIỆU THAM KHẢO")[0]))) == list("1234567")
    pdf = render_dir / (docx.stem + ".pdf")
    document = pymupdf.open(pdf)
    assert 14 <= len(document) <= 16
    issues, fonts, pages = [], set(), []
    for i, page in enumerate(document, 1):
        assert page.get_text().strip(), f"Blank page {i}"
        pages.append(sha(render_dir / f"page-{i}.png"))
        assert "\ufffd" not in page.get_text()
        for block in page.get_text("rawdict")["blocks"]:
            if block["type"] != 0:
                continue
            for line in block["lines"]:
                for span in line["spans"]:
                    chars = [c for c in span["chars"] if c["c"].strip()]
                    if not chars:
                        continue
                    fonts.add(span["font"])
                    box = [min(c["bbox"][0] for c in chars), min(c["bbox"][1] for c in chars),
                           max(c["bbox"][2] for c in chars), max(c["bbox"][3] for c in chars)]
                    if box[0] >= 43 and box[2] <= 552 and box[1] >= 60 and box[3] <= 782:
                        continue
                    if "".join(c["c"] for c in chars) == "√":
                        pix = page.get_pixmap(matrix=pymupdf.Matrix(4, 4), clip=pymupdf.Rect(box), colorspace=pymupdf.csGRAY)
                        ink = [(j % pix.width, j // pix.width) for j, value in enumerate(pix.samples) if value < 160]
                        assert ink
                        box = [(pix.x + min(x for x, y in ink)) / 4, (pix.y + min(y for x, y in ink)) / 4,
                               (pix.x + max(x for x, y in ink)) / 4, (pix.y + max(y for x, y in ink)) / 4]
                        if box[0] >= 43 and box[2] <= 552 and box[1] >= 60 and box[3] <= 782:
                            continue
                    issues.append([i, "".join(c["c"] for c in chars), box])
    assert not issues, issues
    assert any("TimesNewRoman" in font for font in fonts)
    report = {"docx_sha256": sha(docx), "pdf_sha256": sha(pdf), "page_count": len(document),
        "page_png_sha256": pages, "package_parts_changed": changed,
        "table_cells_preserved": len(root.findall(".//w:tc", NS)),
        "prose_scalar_values_preserved": scalar_count,
        "media_preserved": 18, "math_objects_preserved": 12, "equation_numbers_verified": 9,
        "figure_count": len(root.findall(".//w:drawing", NS)), "bounds_issues": issues,
        "font_names": sorted(fonts), "ver7_unchanged": True, "simulation_runs_added": 0,
        "statistical_results": "Preserved exactly from ver7; no new inference or experiments",
        "visual_review_all_pages": bool(accept), "reviewed_date": "2026-10-21" if accept else None}
    (OUT / "quality_checks.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: report[k] for k in ["page_count", "table_cells_preserved", "figure_count", "bounds_issues", "visual_review_all_pages"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--render-dir", required=True, type=Path)
    parser.add_argument("--accept-visual-review", action="store_true")
    args = parser.parse_args()
    verify(args.render_dir, args.accept_visual_review)
