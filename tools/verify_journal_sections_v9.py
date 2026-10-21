#!/usr/bin/env python3
"""Verify v9 edits, immutable evidence and the exact visually reviewed render."""
import argparse
from copy import deepcopy
import csv
import hashlib
import json
import math
from pathlib import Path
import re
from statistics import mean
from zipfile import ZipFile

from lxml import etree
import pymupdf
from build_journal_sections_v7 import NS, W, map_chunks, sha
from build_journal_sections_v9 import ROOT, OLD, OUT, COHORT, validate_sources, dump


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def display_cell(value):
    text = str(value).replace("Smac2D", "Smac 2D")
    if re.fullmatch(r"[0-9]+\.[0-9]+e-[0-9]+", text):
        mantissa, exponent = text.split("e-")
        superscript = str(int(exponent)).translate(str.maketrans("0123456789", "⁰¹²³⁴⁵⁶⁷⁸⁹"))
        return mantissa.replace(".", ",") + " × 10⁻" + superscript
    if re.fullmatch(r"[+−-]?\d+\.\d+", text) or re.fullmatch(r"\[[+−-]?\d+\.\d+;\s*[+−-]?\d+\.\d+\]", text):
        return text.replace(".", ",")
    return text


def check_source_scalars(expected):
    """Recheck descriptive cells only; never rerun bootstrap or hypothesis tests."""
    records = read_csv(ROOT / "docs/warehouse_cross_aisles_5_routes/all_125_trials.csv")
    assert len(records) == 125 and sum(r["success"] == "True" for r in records) == 123
    for record in records:
        record["data"] = json.loads((ROOT / record["source"]).read_text())
    methods = ["raw", "simple", "savitzky_golay", "constrained", "pstmo"]

    def values(route, method, key, success=False):
        return [float(r["data"][key]) for r in records if r["method"] == method
                and (route is None or r["route_id"] == route)
                and (not success or r["success"] == "True")]

    def fmt(value, digits):
        return f"{value:.{digits}f}".replace(".", ",")

    checked = 0
    for i in range(1, 6):
        for method, row in zip(methods, expected[f"r{i:02}_extra"][1], strict=True):
            for cell, (key, scale, digits, success) in zip(row[1:], [
                ("planned_path_length_m", 1, 3, False), ("planned_max_abs_curvature_1pm", 1, 2, False),
                ("traveled_distance_m", 1, 3, True), ("tracking_max_error_m", 1000, 1, True),
                ("smoothing_time_s", 1000, 1, False)], strict=True):
                assert cell == fmt(mean(values(f"R{i:02}", method, key, success)) * scale, digits)
                checked += 1
    for method, row in zip(methods, expected["execution_extra"][1], strict=True):
        for cell, (key, scale, digits) in zip(row[1:], [
            ("executed_curvature_energy_1pm", 1, 3), ("actual_p95_abs_jerk_mps3", 1, 3),
            ("final_position_error_m", 1000, 1), ("final_yaw_error_rad", 1, 3)], strict=True):
            assert cell == fmt(mean(values(None, method, key, True)) * scale, digits)
            checked += 1
    for method, row in zip(methods, expected["dynamics"][1], strict=True):
        for cell, key in zip(row[1:], ["actual_max_angular_radps", "actual_max_wheel_linear_mps",
            "actual_max_abs_lateral_acceleration_mps2", "actual_max_abs_acceleration_mps2",
            "actual_max_abs_angular_acceleration_radps2"], strict=True):
            assert cell == fmt(max(values(None, method, key)), 3)
            checked += 1
    for method, row in zip(methods, expected["translation"][1], strict=True):
        assert row[1:] == [fmt(mean(values(None, method, "planned_translation_max_abs_curvature_1pm")), 2),
            fmt(mean(values(None, method, "planned_translation_curvature_energy_1pm")), 2),
            fmt(mean(values(None, method, "planned_point_count")), 1),
            str(int(sum(values(None, method, "planned_pivot_marker_count"))))]
        checked += 4
    for method, row in zip(methods, expected["safety"][1], strict=True):
        assert row[1:] == ["—" if method == "raw" else fmt(mean(values(None, method, "smoothing_time_s")) * 1000, 2),
            f"{int(sum(values(None, method, 'physically_settled')))}/25",
            str(int(sum(values(None, method, "collision_monitor_interventions")))),
            str(int(sum(values(None, method, "planned_footprint_collision_sample_count"))))]
        checked += 4
    blocks = read_csv(ROOT / COHORT / "analysis/block_differences.csv")
    aggregate = json.loads((ROOT / COHORT / "analysis/aggregate.json").read_text())
    for metric, data in aggregate["metrics"].items():
        differences = [float(r["pstmo_minus_constrained"]) for r in blocks if r["metric"] == metric]
        assert len(differences) == 25
        assert math.isclose(mean(differences), data["mean_block_difference_pstmo_minus_constrained"], abs_tol=1e-10)
    return checked


def verify(render_dir, visual_review):
    prov = json.loads((OUT / "provenance.json").read_text())
    docx = OUT / prov["docx"]
    assert sha(docx) == prov["docx_sha256"]
    for name, digest in prov["source_sha256"].items():
        assert sha(ROOT / name) == digest, name
    for name, digest in prov["output_sha256"].items():
        assert sha(OUT / name) == digest, name
    validation = validate_sources()
    assert validation == json.loads((OUT / "source_validation.json").read_text())
    with ZipFile(docx) as current, ZipFile(ROOT / prov["reference"]) as previous:
        assert current.namelist() == previous.namelist()
        changed = [n for n in current.namelist() if current.read(n) != previous.read(n)]
        assert set(changed) == set(prov["changed_parts_allowed"])
        for name, digest in prov["package_parts"].items():
            assert hashlib.sha256(current.read(name)).hexdigest() == digest
        root = etree.fromstring(current.read("word/document.xml"))
        old_root = etree.fromstring(previous.read("word/document.xml"))
    source, old_source = (OUT / "manuscript.md").read_text(), (OLD / "manuscript.md").read_text()
    records, tail = map_chunks(root, source)
    old_records, old_tail = map_chunks(old_root, old_source)
    serialize = lambda nodes: [etree.tostring(deepcopy(n)) for n in nodes]
    assert serialize(tail) == serialize(old_tail)
    edits = json.loads((OUT / "revision_log.json").read_text())
    by_block = {e["source_block"]: e for e in edits}
    assert len(edits) == len(by_block) == 21
    for i, (record, prior) in enumerate(zip(records, old_records, strict=True)):
        if i in by_block:
            assert record["text"] == by_block[i]["after"] and prior["text"] == by_block[i]["before"]
            assert serialize(record["nodes"][0].findall("w:pPr", NS)) == serialize(prior["nodes"][0].findall("w:pPr", NS))
        else:
            assert record["text"] == prior["text"]
            assert serialize(record["nodes"]) == serialize(prior["nodes"]), i
    for tag, count in [("w:tbl", 23), ("w:drawing", 18), ("m:oMath", 12), ("w:sectPr", 1)]:
        current, prior = root.findall(".//" + tag, NS), old_root.findall(".//" + tag, NS)
        assert len(current) == count and serialize(current) == serialize(prior), tag
    equations = [r for r in records if r["text"].startswith("@equation ")]
    assert len(equations) == 9
    for i, record in enumerate(equations, 1):
        assert [t.text for n in record["nodes"] for t in n.findall(".//w:t", NS)
                if re.fullmatch(r"\(\d+\)", t.text or "")] == [f"({i})"]
    assert list(dict.fromkeys(re.findall(r"[Cc]ông thức (\d+)", source))) == list("123456789")
    assert source.split("# TÀI LIỆU THAM KHẢO")[1] == old_source.split("# TÀI LIỆU THAM KHẢO")[1]
    assert (OUT / "table_values.json").read_bytes() == (OLD / "table_values.json").read_bytes()
    expected = json.loads((OUT / "table_values.json").read_text())
    table_cells = 0
    for key, table in zip(re.findall(r"^@table (\S+)", source, re.M), root.findall(".//w:tbl", NS), strict=True):
        actual = [["".join(cell.xpath(".//w:t/text()", namespaces=NS)) for cell in row.findall("w:tc", NS)]
                  for row in table.findall("w:tr", NS)]
        head, rows, widths = expected[key]
        assert actual == [[display_cell(c) for c in row] for row in [head] + rows], key
        table_cells += sum(map(len, actual))
    assert table_cells == 696
    scalars = check_source_scalars(expected)
    pdf = render_dir / (docx.stem + ".pdf")
    document = pymupdf.open(pdf)
    assert 14 <= len(document) <= 17, len(document)
    pages, fonts, issues = [], set(), []
    for i, page in enumerate(document, 1):
        pages.append(sha(render_dir / f"page-{i}.png"))
        assert page.get_text().strip() and "\ufffd" not in page.get_text()
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
    reviewed = False
    if visual_review:
        review = json.loads(visual_review.read_text())
        assert review["docx_sha256"] == sha(docx) and review["pdf_sha256"] == sha(pdf)
        assert review["page_png_sha256"] == pages
        assert len(review["pages"]) == len(document)
        assert [p["page"] for p in review["pages"]] == list(range(1, len(document) + 1))
        assert all(p["accepted"] is True and p["notes"].strip() for p in review["pages"])
        reviewed = True
    report = {"status": "passed" if reviewed else "awaiting_visual_review", "docx_sha256": sha(docx),
        "pdf_sha256": sha(pdf), "page_count": len(document), "page_png_sha256": pages,
        "package_parts_changed": changed, "edited_paragraphs": len(edits), "table_cells_verified": table_cells,
        "descriptive_numeric_cells_rechecked": scalars, "figures_preserved": 18, "math_objects_preserved": 12,
        "equation_numbers_verified": 9, "historical_source_hashes_checked": len(validation["historical_hashes_checked"]),
        "cohort_source_files_matched": len(validation["cohort_source_matches"]), "bounds_issues": issues,
        "font_names": sorted(fonts), "visual_review_all_pages": reviewed,
        "visual_review_sha256": sha(visual_review) if reviewed else None, "verifier_sha256": sha(Path(__file__)),
        "simulation_runs_added": 0, "inferential_statistics_recomputed": False,
        "notes": ["Unedited prose and all tables, figures, equations and references are XML-identical to ver8.",
                  "Descriptive scalar checks do not rerun bootstrap, Wilcoxon or Holm.",
                  "Journal template and whole-article reference numbering still require confirmation."]}
    dump(OUT / "quality_checks.json", report)
    print(json.dumps({k: report[k] for k in ["status", "page_count", "edited_paragraphs", "table_cells_verified",
        "descriptive_numeric_cells_rechecked", "bounds_issues"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--render-dir", required=True, type=Path)
    parser.add_argument("--visual-review", type=Path)
    args = parser.parse_args()
    verify(args.render_dir, args.visual_review)
