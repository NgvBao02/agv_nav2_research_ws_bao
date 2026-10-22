#!/usr/bin/env python3
"""Build the separate ver10 sections; no simulator or historical-file writes.

Run with the Documents workspace Python (lxml and python-docx available).
The retained ver9 ZIP controls typography and geometry. Only three ZIP parts
are edited. Markdown and references.json are the editable editorial sources.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re
from zipfile import ZipFile

from lxml import etree as ET

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "Viet_bao/journal/ver10"
BASE = ROOT / "Viet_bao/journal/ver9/PSTMO_JOURNAL_VER9_PHAN_II_III.docx"
EXPECTED = "4f5743cc02139a23da557da46ee84704f7d5228bb371b55e7b0a19b0b78a53dc"
FINAL = OUT / "PSTMO_JOURNAL_VER10_ABSTRACT_INTRO_CONCLUSION.docx"
W = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
NS = {"w": W}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, obj):
    Path(path).write_text(json.dumps(obj, ensure_ascii=False, indent=2) + "\n")


def node(name, **attrs):
    e = ET.Element(f"{{{W}}}{name}")
    for key, value in attrs.items():
        e.set(f"{{{W}}}{key}", str(value))
    return e


def paragraph(text, style="BodyText", center=False, size=None, bold=False):
    p = node("p")
    pr = node("pPr")
    pr.append(node("pStyle", val=style))
    if style.startswith("Heading"):
        num = node("numPr")
        num.append(node("numId", val=0))
        pr.append(num)
    if center:
        pr.append(node("jc", val="center"))
        pr.append(node("ind", firstLine=0))
    p.append(pr)
    for token in re.split(r"(⁻¹)", text):
        if not token:
            continue
        r = node("r")
        if size or bold or token == "⁻¹":
            rp = node("rPr")
            if size:
                rp.append(node("sz", val=size))
            if bold:
                rp.append(node("b"))
            if token == "⁻¹":
                rp.append(node("vertAlign", val="superscript"))
            r.append(rp)
        t = node("t")
        t.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
        t.text = "−1" if token == "⁻¹" else token
        r.append(t)
        p.append(r)
    return p


def sources():
    md = (OUT / "manuscript.md").read_text()
    refs = json.loads((OUT / "references.json").read_text())
    by_id = {r["id"]: r for r in refs}
    used = list(dict.fromkeys(re.findall(r"\[(R[A-Z0-9]+)\]", md)))
    assert len(by_id) == len(refs) and set(used) == set(by_id), "Missing/unused/duplicate references"
    numbers = {key: i + 1 for i, key in enumerate(used)}
    numbered = re.sub(r"\[(R[A-Z0-9]+)\]", lambda m: f"[{numbers[m[1]]}]", md)
    return md, numbered, [by_id[k] for k in used], numbers


def build(replace_generated=False):
    assert sha(BASE) == EXPECTED, "ver9 changed: inspect manually before rebuilding"
    md, numbered, refs, numbers = sources()
    if FINAL.exists():
        assert replace_generated, "Output exists: use --check or --replace-generated"
        previous = json.loads((OUT / "provenance.json").read_text())
        assert sha(FINAL) == previous["output_sha256"], "Manual Word edits detected: archive them first"
    with ZipFile(BASE) as z:
        parts = {i.filename: z.read(i.filename) for i in z.infolist()}
        infos = z.infolist()
    document = ET.fromstring(parts["word/document.xml"])
    body = document.find("w:body", NS)
    section = deepcopy(body.find("w:sectPr", NS))
    for child in list(body):
        body.remove(child)

    title = md.splitlines()[0][2:]
    subtitle = next(l[10:] for l in md.splitlines() if l.startswith("@subtitle "))
    title_p = paragraph(title, "Title", center=True)
    title_run = title_p.find("w:r", NS)
    title_run.find("w:t", NS).text = title.split(" cho robot ")[0]
    title_run.append(node("br"))
    continuation = node("t")
    continuation.text = "cho robot " + title.split(" cho robot ")[1]
    title_run.append(continuation)
    body.append(title_p)
    sub = paragraph(subtitle, center=True, size=22)
    sub.find("w:pPr", NS).append(node("keepNext"))
    body.append(sub)
    # One full-width title block followed by the inherited two-column body.
    sep = paragraph("")
    pr = sep.find("w:pPr", NS)
    pr.append(node("spacing", before=0, after=100, line=20, lineRule="exact"))
    wide = deepcopy(section)
    wide.find("w:cols", NS).set(f"{{{W}}}num", "1")
    pr.append(wide)
    body.append(sep)

    for block in numbered.split("\n\n"):
        block = block.strip()
        if not block or block.startswith("# ") or block.startswith("@subtitle "):
            continue
        if block.startswith("### "):
            p = paragraph(block[4:], "Heading2")
        elif block.startswith("## "):
            p = paragraph(block[3:], "Heading1", center=True)
        elif block.startswith("@keywords "):
            p = paragraph("Từ khóa — " + block[10:])
            p.find("w:pPr", NS).append(node("ind", firstLine=0))
        else:
            p = paragraph(block)
        body.append(p)
    body.append(paragraph("TÀI LIỆU THAM KHẢO", "Heading1", center=True))
    for ref in refs:
        p = paragraph(f'[{numbers[ref["id"]]}] {ref["citation"]}', "ScientificReference")
        p.find("w:pPr", NS).append(node("keepLines"))
        body.append(p)
    body.append(section)

    styles = ET.fromstring(parts["word/styles.xml"])
    assert styles.find("w:style[@w:styleId='Title']", NS) is None
    st = node("style", type="paragraph", styleId="Title")
    st.extend([node("name", val="Title"), node("basedOn", val="BodyText"), node("qFormat")])
    pp = node("pPr")
    pp.extend([node("keepNext"), node("keepLines"), node("spacing", before=0, after=120),
               node("ind", firstLine=0), node("jc", val="center")])
    rp = node("rPr")
    rp.extend([node("rFonts", ascii="Times New Roman", hAnsi="Times New Roman", cs="Times New Roman"),
               node("b"), node("color", val="000000"), node("sz", val=28)])
    st.extend([pp, rp])
    styles.append(st)
    core = ET.fromstring(parts["docProps/core.xml"])
    dc = "http://purl.org/dc/elements/1.1/"
    ct = core.find(f"{{{dc}}}title")
    if ct is None:
        ct = ET.SubElement(core, f"{{{dc}}}title")
    ct.text = title
    for k, root in [("word/document.xml", document), ("word/styles.xml", styles), ("docProps/core.xml", core)]:
        parts[k] = ET.tostring(root, xml_declaration=True, encoding="UTF-8", standalone=True)
    with ZipFile(FINAL, "w") as z:
        for info in infos:
            z.writestr(info, parts[info.filename])
    source_records = []
    for ref in refs:
        entry = {"id": ref["id"], "number": numbers[ref["id"]], "url": ref["url"], "pdf_pages_reviewed": ref["pages"]}
        if ref.get("local_id"):
            matches = list((ROOT / "REFERENCES").glob(ref["local_id"] + "_*.pdf"))
            assert len(matches) == 1
            entry.update(path=str(matches[0].relative_to(ROOT)), sha256=sha(matches[0]))
        else:
            entry.update(accessed="2026-10-22", source_kind="primary_web_page")
        source_records.append(entry)
    inventory = []
    with ZipFile(BASE) as z:
        for info in infos:
            name = info.filename
            inventory.append({"part": name, "source_size": info.file_size,
                              "source_sha256": hashlib.sha256(z.read(name)).hexdigest(),
                              "output_sha256": hashlib.sha256(parts[name]).hexdigest(),
                              "editable": name in {"word/document.xml", "word/styles.xml", "docProps/core.xml"}})
    provenance = {"template": str(BASE.relative_to(ROOT)), "template_sha256": EXPECTED,
                  "output_sha256": sha(FINAL), "source_files": [], "references": source_records,
                  "package_inventory": inventory, "simulations_run": False,
                  "statistical_reanalysis": False, "visual_review": "pending"}
    for p in [OUT / "manuscript.md", OUT / "references.json", Path(__file__),
              BASE.parent / "manuscript.md", BASE.parent / "table_values.json",
              ROOT / "results/pstmo_journal_repeatability_20261011/analysis/aggregate.json"]:
        provenance["source_files"].append({"path": str(p.relative_to(ROOT)), "sha256": sha(p)})
    dump(OUT / "provenance.json", provenance)
    (OUT / "manuscript_numbered.md").write_text(numbered + "\n## TÀI LIỆU THAM KHẢO\n\n" +
        "\n\n".join(f'[{numbers[r["id"]]}] {r["citation"]}' for r in refs) + "\n")
    dump(OUT / "quality_checks.json", {"status": "pending_render_and_visual_review", "docx_sha256": sha(FINAL)})
    print(FINAL)


def check(render_dir=None):
    assert sha(BASE) == EXPECTED
    md, numbered, refs, numbers = sources()
    prov = json.loads((OUT / "provenance.json").read_text())
    assert sha(FINAL) == prov["output_sha256"]
    for entry in prov["source_files"]:
        assert sha(ROOT / entry["path"]) == entry["sha256"], entry["path"]
    for entry in prov["references"]:
        if "path" in entry:
            assert sha(ROOT / entry["path"]) == entry["sha256"]
    with ZipFile(BASE) as old, ZipFile(FINAL) as new:
        assert set(old.namelist()) == set(new.namelist())
        for entry in prov["package_inventory"]:
            if not entry["editable"]:
                assert old.read(entry["part"]) == new.read(entry["part"])
        d = ET.fromstring(new.read("word/document.xml"))
    text = "\n".join("".join((" " if e.tag == f"{{{W}}}br" else (e.text or ""))
                              for e in p.iter() if e.tag in {f"{{{W}}}t", f"{{{W}}}br"})
                     for p in d.findall("w:body/w:p", NS))
    assert not re.search(r"\[R[A-Z0-9]+\]|TODO|TBD|FIXME", text)
    assert not d.findall(".//w:tbl", NS) and not d.findall(".//w:drawing", NS)
    assert len(d.findall(".//w:sectPr", NS)) == 2
    assert "II. XÂY DỰNG" not in text and "III. XÂY DỰNG" not in text
    for block in numbered.split("\n\n"):
        expected = re.sub(r"^(#{1,3} |@subtitle |@keywords )", "", block.strip())
        assert expected.replace("⁻¹", "−1") in text, expected[:60]
    # Check every reported numerical effect against the fixed rounded ver9 tables.
    tables = json.loads((BASE.parent / "table_values.json").read_text())
    checks = [("primary", 0, 1, "-2.857"), ("primary", 1, 1, "-2.985"),
              ("primary", 2, 1, "-1.038"), ("primary", 0, 2, "[-3.716; -2.039]"),
              ("primary", 1, 2, "[-4.487; -1.506]"), ("secondary", 3, 1, "+3.380"),
              ("secondary", 4, 1, "-35.161"), ("secondary", 5, 1, "+71.220")]
    for key, row, col, expected in checks:
        assert tables[key][1][row][col] == expected
        for n in re.findall(r"\d+\.\d+", expected):
            assert n.replace(".", ",") in text
    result = {"docx_sha256": sha(FINAL), "reference_count": len(refs), "body_only_requested_sections": True,
              "template_unchanged": True, "preserved_package_parts": True, "source_hashes_match": True,
              "numerical_effects_match_ver9": True, "missing_or_uncited_references": 0,
              "full_paper_50_reference_target_met": False, "visual_review": "pending"}
    if render_dir:
        import fitz
        rd = Path(render_dir)
        pdfs = list(rd.glob("*.pdf"))
        assert len(pdfs) == 1
        pdf = fitz.open(pdfs[0])
        fonts = sorted({f[3] for p in pdf for f in p.get_fonts()})
        assert all("TimesNewRoman" in f for f in fonts), fonts
        pages = sorted(rd.glob("page-*.png"), key=lambda p: int(p.stem.split("-")[1]))
        assert len(pages) == len(pdf)
        result.update(page_count=len(pdf), pdf_sha256=sha(pdfs[0]), fonts=fonts)
        reviewed = OUT / "visual_review.json"
        if reviewed.exists():
            v = json.loads(reviewed.read_text())
            assert v["docx_sha256"] == sha(FINAL) and v["pdf_sha256"] == sha(pdfs[0])
            assert v["page_png_sha256"] == [sha(p) for p in pages]
            assert [p["page"] for p in v["pages"]] == list(range(1, len(pdf) + 1))
            assert all(p["accepted"] and p["notes"] for p in v["pages"])
            result["visual_review"] = "passed_all_pages"
    result["status"] = "passed" if result["visual_review"] == "passed_all_pages" else "pending_visual_review"
    dump(OUT / "quality_checks.json", result)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--replace-generated", action="store_true", help="Rebuild only if DOCX matches the previous generated hash")
    ap.add_argument("--render-dir")
    args = ap.parse_args()
    if args.check:
        check(args.render_dir)
    else:
        build(args.replace_generated)
