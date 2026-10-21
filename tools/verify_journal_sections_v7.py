#!/usr/bin/env python3
"""Verify immutable sources, package fidelity, editorial claims and render."""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
from statistics import mean
from zipfile import ZipFile

from lxml import etree
import pymupdf

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "Viet_bao/journal/ver7"
OLD = ROOT / "Viet_bao/journal/ver6"
NS = {"w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main",
      "m": "http://schemas.openxmlformats.org/officeDocument/2006/math",
      "wp": "http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing"}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(render_dir, accept):
    p = json.loads((OUT / "provenance.json").read_text())
    oldp = json.loads((OLD / "provenance.json").read_text())
    docx = OUT / p["docx"]
    assert digest(docx) == p["docx_sha256"]
    assert digest(ROOT / p["reference"]) == p["reference_sha256"] == oldp["docx_sha256"]
    assert digest(OLD / "provenance.json") == p["reference_provenance_sha256"]
    for name, sha in {**oldp["source_sha256"], **p["source_sha256"]}.items():
        assert digest(ROOT / name) == sha, name
    with ZipFile(docx) as z, ZipFile(ROOT / p["reference"]) as oldz:
        assert set(z.namelist()) == set(oldz.namelist())
        changed = [n for n in z.namelist() if z.read(n) != oldz.read(n)]
        assert set(changed) == set(p["changed_parts_allowed"]), changed
        for n, sha in p["package_parts"].items():
            assert hashlib.sha256(z.read(n)).hexdigest() == sha
        root = etree.fromstring(z.read("word/document.xml"))
        old = etree.fromstring(oldz.read("word/document.xml"))
    for tag in ["m:oMath", "w:sectPr"]:
        assert [etree.tostring(n) for n in root.findall(".//" + tag, NS)] == [
            etree.tostring(n) for n in old.findall(".//" + tag, NS)], tag
    assert len(root.findall(".//m:oMath", NS)) == 12
    assert len(root.findall(".//wp:inline", NS)) == 18
    source = (OUT / "manuscript.md").read_text()
    prior = (OLD / "manuscript.md").read_text()
    assert source.split("# TÀI LIỆU THAM KHẢO")[1] == prior.split("# TÀI LIỆU THAM KHẢO")[1]
    assert source.split("## E. Lựa chọn")[0] == prior.split("## E. Lựa chọn")[0]
    assert source.split("## I. Phân tích")[1] == prior.split("## I. Phân tích")[1]
    assert "## F. Trình tự thực hiện" not in source
    section_a = source.split("## A. Môi trường")[1].split("## B. R01")[0]
    headings = re.findall(r"^### (\d+)\)", section_a, re.M)
    assert headings == list("1234567")
    assert section_a.index("### 1) Robot") < section_a.index("### 2) Cấu hình") < section_a.index("### 3) Bản đồ")
    assert "@table params" not in source.split("# III.")[0]
    assert "trọng số độ cong" in source and "không xác lập ưu thế" in source
    assert list(dict.fromkeys(re.findall(r"\[(\d+)\]", source.split("# TÀI LIỆU THAM KHẢO")[0]))) == list("1234567")
    visible = " ".join(root.xpath(".//w:t/text()", namespaces=NS))
    for forbidden in ["ứng viên", "năng lượng độ cong", "planned_curvature_energy",
                      "executed_curvature_energy", "ground_truth_state_trace"]:
        assert forbidden not in visible.lower(), forbidden
    assert not re.search(r"[a-f0-9]{40,64}", visible)
    assert not re.search(r"(?:trong|theo)\s*\(\d+\)", visible, re.I)

    # Every table unchanged at XML level except one intentionally renumbered
    # cross-reference. This carries forward v6's independently audited numbers.
    old_keys = re.findall(r"^@table (\S+)", prior, re.M)
    new_keys = re.findall(r"^@table (\S+)", source, re.M)
    tables = root.findall(".//w:tbl", NS)
    assert new_keys == p["table_order"] and len(tables) == 23
    old_tables = dict(zip(old_keys, old.findall(".//w:tbl", NS), strict=True))
    cells = 0
    for key, table in zip(new_keys, tables, strict=True):
        expected = deepcopy(old_tables[key])
        if key == "baselines":
            for t in expected.findall(".//w:t", NS):
                t.text = (t.text or "").replace("Bảng I;", "Bảng II;")
        assert etree.tostring(expected) == etree.tostring(deepcopy(table)), key
        cells += len(table.findall(".//w:tc", NS))
    # Verify the exported table values likewise preserve all numerical cells.
    a = json.loads((OLD / "table_values.json").read_text())
    b = json.loads((OUT / "table_values.json").read_text())
    a["baselines"][1][-1][1] = a["baselines"][1][-1][1].replace("Bảng I;", "Bảng II;")
    assert a == b
    captions = [s for s in root.xpath("./w:body/w:p/w:r/w:t/text()", namespaces=NS) if s.startswith("BẢNG ")]
    assert len(captions) == 23 and captions[0].startswith("BẢNG I.") and captions[1].startswith("BẢNG II.")
    assert "BẢNG I. THAM SỐ ROBOT" in visible
    assert "BẢNG II. CẤU HÌNH PSTMO" in visible
    old_qa = json.loads((OLD / "quality_checks.json").read_text())
    assert old_qa["docx_sha256"] == p["reference_sha256"] and old_qa["visual_review_all_pages"]

    # Independently recompute new prose evidence from immutable trial JSON.
    ev = json.loads((OUT / "editorial_evidence.json").read_text())
    methods = ["raw", "simple", "savitzky_golay", "constrained", "pstmo"]
    routes = ["cross_aisle_transfer", "wide_diagonal_transfer", "south_u_transfer",
              "reverse_diagonal_transfer", "cross_to_picking_turn"]
    records = []
    for name in oldp["source_sha256"]:
        if name.endswith(".json") and "/analysis/aggregate.json" not in name:
            method = next(m for m in sorted(methods, key=len, reverse=True) if name.endswith("_" + m + ".json"))
            rt = next(r for r in routes if r in name)
            records.append((rt, method, json.loads((ROOT / name).read_text())))
    assert len(records) == 125
    claims = 0
    for i, rt in enumerate(routes, 1):
        for m in methods:
            ds = [d for r, method, d in records if r == rt and m == method]
            for k, value in ev["route_means"][f"R{i:02}"][m].items():
                vals = [d[k] for d in ds if k.startswith("planned_") or k == "smoothing_time_s" or d["success"]]
                assert math.isclose(sum(vals) / len(vals), value, rel_tol=1e-12, abs_tol=1e-12)
                claims += 1
    for method, c in ev["trace"].items():
        d = json.loads((ROOT / c["source"]).read_text())
        hi = [s for s in d["ground_truth_state_trace"] if 34 <= s[0] <= 40 and s[5] > .24]
        assert c["sample_span_s"] == hi[-1][0] - hi[0][0]
        assert c["execution_time_s"] == d["execution_time_s"]
    # Key comparative claims checked at full precision, not rounded display.
    for rt, bymethod in ev["route_means"].items():
        keys = ["planned_path_length_m"]
        if rt != "R02":
            keys.append("tracking_max_error_m")
        for k in keys:
            assert bymethod["pstmo"][k] < min(bymethod[m][k] for m in methods[:-1])
    assert ev["trace"]["simple"]["execution_time_s"] < ev["trace"]["pstmo"]["execution_time_s"] < ev["trace"]["constrained"]["execution_time_s"]
    for i in [1, 2, 4, 5]:
        row = ev["route_means"][f"R{i:02}"]
        assert row["pstmo"]["planned_max_abs_curvature_1pm"] < min(row[m]["planned_max_abs_curvature_1pm"] for m in methods[:-1])
    r3 = ev["route_means"]["R03"]
    assert r3["pstmo"]["planned_max_abs_curvature_1pm"] > r3["simple"]["planned_max_abs_curvature_1pm"]

    pdf = render_dir / (docx.stem + ".pdf")
    rendered = pymupdf.open(pdf)
    assert 14 <= len(rendered) <= 16
    bounds, fonts, headings, page_hashes = [], set(), {}, []
    for i, pg in enumerate(rendered, 1):
        page_hashes.append(digest(render_dir / f"page-{i}.png"))
        assert "\ufffd" not in pg.get_text()
        for label in ["III. XÂY DỰNG", "TÀI LIỆU THAM KHẢO CHO PHẦN II VÀ III"]:
            hits = pg.search_for(label)
            if hits:
                headings[label] = {"page": i, "rect": list(hits[0])}
        for block in pg.get_text("rawdict")["blocks"]:
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
                    if box[0] < 43 or box[2] > 552 or box[1] < 60 or box[3] > 782:
                        if "".join(c["c"] for c in chars) == "√":
                            pix = pg.get_pixmap(matrix=pymupdf.Matrix(4, 4), clip=pymupdf.Rect(box), colorspace=pymupdf.csGRAY)
                            ink = [(j % pix.width, j // pix.width) for j, v in enumerate(pix.samples) if v < 160]
                            assert ink
                            box = [(pix.x + min(x for x, y in ink)) / 4, (pix.y + min(y for x, y in ink)) / 4,
                                   (pix.x + max(x for x, y in ink)) / 4, (pix.y + max(y for x, y in ink)) / 4]
                            if box[0] >= 43 and box[2] <= 552 and box[1] >= 60 and box[3] <= 782:
                                continue
                        bounds.append([i, "".join(c["c"] for c in chars), box])
    assert not bounds, bounds
    assert any("TimesNewRoman" in f for f in fonts)
    report = {"docx_sha256": digest(docx), "pdf_sha256": digest(pdf), "page_count": len(rendered),
        "page_png_sha256": page_hashes, "changed_package_parts": changed,
        "table_cells_preserved": cells, "route_mean_values_recomputed": claims,
        "trace_records_recomputed": 5, "unchanged_equations": 12, "unchanged_media": 18,
        "headings": headings, "font_names": sorted(fonts), "bounds_issues": bounds,
        "ver6_unchanged": True, "simulation_runs_added": 0,
        "visual_review_all_pages": bool(accept), "reviewed_date": "2026-10-21" if accept else None}
    (OUT / "quality_checks.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: report[k] for k in ["page_count", "table_cells_preserved",
        "route_mean_values_recomputed", "bounds_issues", "visual_review_all_pages"]}, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--render-dir", required=True, type=Path)
    parser.add_argument("--accept-visual-review", action="store_true")
    args = parser.parse_args()
    main(args.render_dir, args.accept_visual_review)
