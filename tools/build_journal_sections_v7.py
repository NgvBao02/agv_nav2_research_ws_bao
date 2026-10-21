#!/usr/bin/env python3
"""Surgical, package-preserving editorial revision of journal v6.

No ROS invocation, data mutation, figure regeneration or old-version writes.
Only document.xml and core.xml may change in the retained Word package.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re
from statistics import mean
from zipfile import ZipFile

from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / "Viet_bao/journal/ver6"
OUT = ROOT / "Viet_bao/journal/ver7"
NS = {"w": "http://schemas.openxmlformats.org/wordprocessingml/2006/main",
      "m": "http://schemas.openxmlformats.org/officeDocument/2006/math"}
W = "{" + NS["w"] + "}"
METHODS = ["raw", "simple", "savitzky_golay", "constrained", "pstmo"]
ROUTES = ["cross_aisle_transfer", "wide_diagonal_transfer", "south_u_transfer",
          "reverse_diagonal_transfer", "cross_to_picking_turn"]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fmt(value, n=3):
    return f"{value:.{n}f}".replace(".", ",")


def plain(element):
    return "".join(element.xpath(".//w:t/text()", namespaces=NS))


def scientific_runs(p, text):
    """Use editable subscripts in rewritten prose; inherit paragraph typography."""
    forms = {"Eκ,ref": ("E", "κ,ref"), "Eκ,exec": ("E", "κ,exec"),
             "Eκ,t": ("E", "κ,t"), "Eκ": ("E", "κ"), "κmax": ("κ", "max"),
             "κt,max": ("κ", "t,max"), "emax": ("e", "max"), "j95": ("j", "95"),
             "ep": ("e", "p"), "eψ": ("e", "ψ"), "δT": ("δ", "T"),
             "Tmin": ("T", "min"), "Tp": ("T", "p"), "Rb": ("R", "b"),
             "db": ("d", "b")}
    for child in list(p):
        if child.tag != W + "pPr":
            p.remove(child)
    pattern = "(" + "|".join(re.escape(k) for k in sorted(forms, key=len, reverse=True)) + ")"
    for part in re.split(pattern, text):
        if not part:
            continue
        runs = [(part, None)] if part not in forms else [
            (forms[part][0], "i"), (forms[part][1], "subscript")]
        for value, kind in runs:
            r = etree.SubElement(p, W + "r")
            if kind:
                pr = etree.SubElement(r, W + "rPr")
                if kind == "i":
                    etree.SubElement(pr, W + "i")
                else:
                    etree.SubElement(pr, W + "vertAlign", {W + "val": kind})
            t = etree.SubElement(r, W + "t")
            t.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
            t.text = value


def map_chunks(root, source):
    """Map all source chunks to existing elements; fail on hand-edited mismatch."""
    body = root.find("w:body", NS)
    children = list(body)
    chunks = re.split(r"\n\s*\n", source.strip())
    result, pos = [], 0
    for c in chunks:
        if c.startswith("<!--"):
            continue
        count = 3 if c.startswith("@table ") else 2 if c.startswith("@figure ") else (
            len(c.split(" || ")) if c.startswith("@equation ") else 1)
        nodes = children[pos:pos + count]
        assert len(nodes) == count
        if c.startswith("@table "):
            assert nodes[1].tag == W + "tbl"
        elif c.startswith("@figure "):
            assert nodes[0].find(".//w:drawing", NS) is not None
            assert plain(nodes[1]).split(". ", 1)[1] == c.split(" ", 2)[2]
        elif c.startswith("@equation "):
            assert all(e.find(".//m:oMath", NS) is not None for e in nodes)
        else:
            visible = re.sub(r"^(?:#{1,3} |@note )", "", c).replace("\n", " ")
            assert plain(nodes[0]) == visible, (c[:90], plain(nodes[0])[:90])
        result.append({"text": c, "nodes": nodes})
        pos += count
    tail = children[pos:]
    assert len(tail) == 2 and tail[-1].tag == W + "sectPr"
    return result, tail


def data_evidence(prov):
    records = []
    for path, digest in prov["source_sha256"].items():
        assert sha(ROOT / path) == digest, f"Changed source: {path}"
        if path.endswith(".json") and path != "results/pstmo_journal_repeatability_20261011/analysis/aggregate.json":
            d = json.loads((ROOT / path).read_text())
            method = next(m for m in sorted(METHODS, key=len, reverse=True)
                          if path.endswith("_" + m + ".json"))
            route = next(r for r in ROUTES if r in path)
            records.append((route, method, d, path))
    assert len(records) == 125
    claims = {"route_means": {}, "trace": {}}
    keys = ["planned_path_length_m", "planned_max_abs_curvature_1pm",
            "traveled_distance_m", "tracking_max_error_m", "smoothing_time_s",
            "final_position_error_m", "final_yaw_error_rad",
            "executed_curvature_energy_1pm", "actual_p95_abs_jerk_mps3"]
    for i, route in enumerate(ROUTES, 1):
        claims["route_means"][f"R{i:02}"] = {}
        for method in METHODS:
            ds = [d for r, m, d, _ in records if (r, m) == (route, method)]
            assert len(ds) == 5
            claims["route_means"][f"R{i:02}"][method] = {
                k: mean(d[k] for d in ds if
                        k.startswith("planned_") or k == "smoothing_time_s" or d["success"])
                for k in keys}
    for path in prov["figures"]["execution_trace"]["sources"]:
        d = json.loads((ROOT / path).read_text())
        method = next(m for m in sorted(METHODS, key=len, reverse=True)
                      if path.endswith("_" + m + ".json"))
        gt = d["ground_truth_state_trace"]
        # Fixed descriptive window and threshold, not a new performance metric.
        high = [s for s in gt if 34 <= s[0] <= 40 and s[5] > .24]
        assert high and all(b[0] - a[0] < .25 for a, b in zip(high, high[1:]))
        claims["trace"][method] = {"source": path, "execution_time_s": d["execution_time_s"],
            "positive_plateau_start_s": high[0][0], "positive_plateau_end_s": high[-1][0],
            "sample_span_s": high[-1][0] - high[0][0],
            "max_angular_radps": max(s[5] for s in gt)}
    return claims


def main():
    OUT.mkdir(exist_ok=True)
    prov = json.loads((OLD / "provenance.json").read_text())
    original = OLD / prov["docx"]
    assert sha(original) == prov["docx_sha256"]
    with ZipFile(original) as z:
        parts = {n: z.read(n) for n in z.namelist()}
        infos = z.infolist()
    root = etree.fromstring(parts["word/document.xml"])
    original_source = (OLD / "manuscript.md").read_text()
    records, tail = map_chunks(root, original_source)
    changes = []

    def index(prefix):
        found = [i for i, r in enumerate(records) if r["text"].startswith(prefix)]
        assert len(found) == 1, (prefix, found)
        return found[0]

    def replace(prefix, text):
        i = index(prefix)
        record = records[i]
        assert len(record["nodes"]) == 1
        changes.append({"before": record["text"], "after": text})
        record["text"] = text
        scientific_runs(record["nodes"][0], re.sub(r"^#{1,3} ", "", text))

    def insert_before(prefix, text, heading=False):
        template = records[index("### 1) Robot" if heading else "PSTMO nhận đường")]
        p = deepcopy(template["nodes"][0])
        scientific_runs(p, re.sub(r"^#{1,3} ", "", text))
        records.insert(index(prefix), {"text": text, "nodes": [p]})
        changes.append({"before": "", "after": text})

    # Section II retains algorithm acceptance; deployment moves to III-A.
    replace("Sau khi ghép, PSTMO kiểm tra",
        "Sau khi truy vết, các đoạn thẳng, chuyển tiếp và tư thế quay tại chỗ được ghép theo thứ tự đường đi. "
        "PSTMO kiểm tra vị trí đầu–cuối, giới hạn chuyển động, đường bao tại các mối nối và hướng cuối. "
        "Thuật toán từ chối đường nếu thiếu phương án khả thi, không có chuỗi tương thích hoặc đầu ra không vượt qua toàn bộ kiểm tra.")
    params = records.pop(index("@table params "))
    params_note = records.pop(index("@note Ký hiệu δT"))
    begin, end = index("## F. Trình tự"), index("# III.")
    for r in records[begin:end]:
        changes.append({"before": r["text"], "after": ""})
    del records[begin:end]
    insert_before("# III.",
        "Hai chiều dài cắt và phép tìm kiếm phân cấp giới hạn số đường cong cần đánh giá nhưng có thể bỏ sót hình dạng khả thi khác. "
        "Tính tối ưu chỉ áp dụng cho tập phương án đã xây dựng; đường được chấp nhận chưa bảo đảm robot hoàn thành nhiệm vụ khi thực thi. "
        "Phần III đánh giá riêng kết quả chuyển động dưới bộ điều khiển.")
    replace("Thử nghiệm sử dụng robot vi sai",
        "Thử nghiệm sử dụng robot vi sai trong ROS 2 Jazzy, Nav2 và Gazebo. Đường sau hậu xử lý làm tham chiếu cho bộ điều khiển "
        "Regulated Pure Pursuit, viết tắt RPP [1]. Lệnh vận tốc đi qua bộ làm mượt vận tốc trước khi robot thực thi; bộ giám sát "
        "va chạm hoạt động trong cùng hệ điều hướng. Trong mỗi nhóm tuyến–bộ lập kế hoạch, các phương án dùng chung bản đồ, robot, bộ điều khiển và tiêu chí đích.")
    rpp = records.pop(index("RPP sử dụng vận tốc đặt"))
    insert_before("### 2) Bản đồ", "### 2) Cấu hình PSTMO và tích hợp trong hệ điều hướng", True)
    insert_before("### 2) Bản đồ",
        "Bảng I trình bày các giá trị cấu hình PSTMO dùng trong khảo sát. Sau các phép kiểm tra đầu ra ở Mục II-E, "
        "đường được chuyển tới RPP để tạo lệnh vận tốc bám đường. Thời gian dự đoán tại góc dùng để lựa chọn chuyển tiếp; "
        "thời gian thực thi đo toàn nhiệm vụ có phản hồi điều khiển. Hai đại lượng không được dùng thay thế cho nhau.")
    at = index("### 2) Bản đồ")
    records[at:at] = [params, params_note, rpp]
    for prefix in ["### 6) Chỉ số", "### 5) Tiêu chí", "### 4) Hai bộ", "### 3) Bộ lập", "### 2) Bản đồ"]:
        rec = records[index(prefix)]
        n = int(rec["text"][4])
        replace(prefix, rec["text"].replace(f"### {n})", f"### {n + 1})", 1))

    edits = json.loads((OUT / "editorial_revisions.json").read_text())
    for change in edits["replacements"]:
        replace(change["prefix"], change["text"])

    evidence = data_evidence(prov)
    # Retain every five-method scalar series; add interpretation at its location.
    endpoints = [r for r in records if r["text"].startswith("Sai số vị trí cuối trung bình của năm phương án")]
    assert len(endpoints) == 5
    for i, r in enumerate(endpoints, 1):
        vals = evidence["route_means"][f"R{i:02}"]
        expected_p = "; ".join(fmt(vals[m]["final_position_error_m"] * 1000, 1) for m in METHODS)
        expected_y = "; ".join(fmt(vals[m]["final_yaw_error_rad"], 3) for m in METHODS)
        assert expected_p in r["text"] and expected_y in r["text"]
        replace(r["text"], r["text"] + " " + edits["endpoint_analysis"][i - 1])
    curvature = [r for r in records if r["text"].startswith("Theo thứ tự đường gốc, Simple, Savitzky–Golay")]
    assert len(curvature) == 5
    for i, r in enumerate(curvature, 1):
        vals = evidence["route_means"][f"R{i:02}"]
        for key in ["executed_curvature_energy_1pm", "actual_p95_abs_jerk_mps3"]:
            assert "; ".join(fmt(vals[m][key]) for m in METHODS) in r["text"]
        # Replace the old interpretation rather than repeat it.
        kept = r["text"].split(" m/s³.", 1)[0] + " m/s³. "
        replace(r["text"], kept + edits["extended_analysis"][i - 1])

    for i in range(1, 6):
        pref = f"@table r{i:02}_extra "
        at = index(pref)
        assert records[at + 1]["text"].startswith("@note ")
        text = edits["geometry_analysis"][i - 1]
        template = records[index("PSTMO nhận đường")]["nodes"][0]
        p = deepcopy(template)
        scientific_runs(p, text)
        records.insert(at + 2, {"text": text, "nodes": [p]})
        changes.append({"before": "", "after": text})

    tr = evidence["trace"]
    trace_text = (
        "Hình 16 cho thấy các phương án đều xuất hiện đoạn vận tốc góc dương gần 0,25 rad/s, nhưng thời điểm và khoảng duy trì khác nhau. "
        "Trong khoảng 34–40 s, khoảng từ mẫu đầu đến mẫu cuối có ω > 0,24 rad/s là "
        f"{fmt(tr['pstmo']['sample_span_s'], 2)} s với PSTMO, {fmt(tr['simple']['sample_span_s'], 2)} s với Simple và "
        f"{fmt(tr['constrained']['sample_span_s'], 2)} s với Constrained. "
        "Biên độ gần nhau không có nghĩa thời gian toàn nhiệm vụ bằng nhau. Ở lượt R02/Theta* này, PSTMO hoàn thành trong "
        f"{fmt(tr['pstmo']['execution_time_s'], 3)} s, nhanh hơn Constrained "
        f"{fmt(tr['constrained']['execution_time_s'] - tr['pstmo']['execution_time_s'], 3)} s nhưng chậm hơn Simple "
        f"{fmt(tr['pstmo']['execution_time_s'] - tr['simple']['execution_time_s'], 3)} s. "
        "Vùng quỹ đạo phóng to chỉ biểu diễn một phần hành trình, không ứng với toàn bộ đồ thị thời gian. "
        "RPP tắt điều tiết vận tốc theo độ cong và chi phí; các tín hiệu này chưa tách được nguyên nhân của chênh lệch thời gian.")
    replace("Hình 16 đối chiếu hình dạng đường", trace_text)

    # Separate large adjacent figures with the analysis they support. Preserve
    # figure/table numbering, image dimensions and all measurements.
    r05_pair = "So sánh theo bộ lập kế hoạch cho thấy PSTMO nhanh hơn Constrained ở bốn cặp, nhưng"
    figure = records.pop(index("@figure r05_metrics "))
    records.insert(index(r05_pair) + 1, figure)
    figure = records.pop(index("@figure matrix_delta "))
    records.insert(index("Bảng XVI ghép PSTMO"), figure)

    # Robot table now precedes PSTMO table: update all references, including cells.
    def renumber(text):
        return re.sub(r"(?i)(bảng )([IVX]+)\b",
                      lambda m: m[1] + {"I": "II", "II": "I"}.get(m[2], m[2]), text)
    for r in records:
        r["text"] = renumber(r["text"])
        if r["text"].startswith("@note "):
            pr = r["nodes"][0].find("w:pPr", NS)
            assert pr is not None
            keep = pr.find("w:keepLines", NS)
            if keep is None:
                keep = etree.SubElement(pr, W + "keepLines")
            keep.set(W + "val", "1")
        for node in r["nodes"]:
            for t in node.findall(".//w:t", NS):
                t.text = renumber(t.text or "")
            for dp in node.findall(".//{http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing}docPr"):
                if dp.get("descr"):
                    dp.set("descr", renumber(dp.get("descr")))
    body = root.find("w:body", NS)
    for node in list(body):
        body.remove(node)
    for r in records:
        for node in r["nodes"]:
            body.append(node)
    for node in tail:
        body.append(node)
    parts["word/document.xml"] = etree.tostring(root, xml_declaration=True, encoding="UTF-8", standalone=True)
    core = etree.fromstring(parts["docProps/core.xml"])
    subject = core.find("{http://purl.org/dc/elements/1.1/}subject")
    subject.text = "Phần II và III của bản thảo journal phiên bản 7"
    parts["docProps/core.xml"] = etree.tostring(core, xml_declaration=True, encoding="UTF-8", standalone=True)
    output = OUT / "PSTMO_JOURNAL_VER7_PHAN_II_III.docx"
    with ZipFile(output, "w") as z:
        for info in infos:
            z.writestr(info, parts[info.filename])
    source = "\n\n".join(r["text"] for r in records) + "\n"
    (OUT / "manuscript.md").write_text(source)
    (OUT / "editorial_evidence.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n")
    (OUT / "revision_log.json").write_text(json.dumps(changes, ensure_ascii=False, indent=2) + "\n")
    table_order = re.findall(r"^@table (\S+)", source, re.M)
    values = json.loads((OLD / "table_values.json").read_text())
    values = {k: values[k] for k in table_order}
    values["baselines"][1][-1][1] = renumber(values["baselines"][1][-1][1])
    (OUT / "table_values.json").write_text(json.dumps(values, ensure_ascii=False, indent=2) + "\n")
    provenance = {"docx": output.name, "docx_sha256": sha(output),
        "reference": str(original.relative_to(ROOT)), "reference_sha256": sha(original),
        "reference_provenance_sha256": sha(OLD / "provenance.json"),
        "table_order": table_order, "counts": prov["counts"],
        "source_sha256": {str(p.relative_to(ROOT)): sha(p) for p in [
            OLD / "manuscript.md", OLD / "table_values.json", OLD / "quality_checks.json",
            Path(__file__), OUT / "editorial_revisions.json", OUT / "manuscript.md",
            OUT / "editorial_evidence.json", OUT / "table_values.json"]},
        "package_parts": {name: hashlib.sha256(data).hexdigest() for name, data in parts.items()},
        "changed_parts_allowed": ["word/document.xml", "docProps/core.xml"],
        "simulation_runs_added": 0, "date": "2026-10-21"}
    (OUT / "provenance.json").write_text(json.dumps(provenance, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(output), "counts": prov["counts"],
                      "word_delta": len(source.split()) - len(original_source.split())}, ensure_ascii=False))


if __name__ == "__main__":
    main()
