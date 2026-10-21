#!/usr/bin/env python3
"""Revise the argument flow of journal v7 without altering measurements."""
from copy import deepcopy
import json
from pathlib import Path
import re
from zipfile import ZipFile
from lxml import etree

from build_journal_sections_v7 import NS, W, map_chunks, plain, scientific_runs, sha

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / "Viet_bao/journal/ver7"
OUT = ROOT / "Viet_bao/journal/ver8"
REFERENCE = OUT / "reference_ver7_original.docx"
REFERENCE_SHA = "bef87e2d34ea41b54e1647f8954fa7a4cce00405e73e6a7189c8707c62697aa9"


def main():
    original = REFERENCE
    assert sha(original) == REFERENCE_SHA, "Reference changed; inspect manual edits first"
    prior_prov = json.loads((OLD / "provenance.json").read_text())
    v6_prov = json.loads((ROOT / "Viet_bao/journal/ver6/provenance.json").read_text())
    for name, digest in {**v6_prov["source_sha256"], **prior_prov["source_sha256"]}.items():
        assert sha(ROOT / name) == digest, name
    with ZipFile(original) as z:
        parts = {n: z.read(n) for n in z.namelist()}
        infos = z.infolist()
    root = etree.fromstring(parts["word/document.xml"])
    source = (OLD / "manuscript.md").read_text()
    records, tail = map_chunks(root, source)
    for i, r in enumerate(records):
        r["source_block"] = i
    changes = []

    def index(prefix):
        hits = [i for i, r in enumerate(records) if r["text"].startswith(prefix)]
        assert len(hits) == 1, (prefix, hits)
        return hits[0]

    def replace(prefix, text):
        r = records[index(prefix)]
        assert len(r["nodes"]) == 1
        changes.append({"operation": "replace", "source_block": r["source_block"],
                        "before": r["text"], "after": text})
        r["text"] = text
        scientific_runs(r["nodes"][0], re.sub(r"^(?:#{1,3} |@note )", "", text))

    def inline(prefix, old, new):
        r = records[index(prefix)]
        before = r["text"]
        assert old in before
        changed = 0
        for node in r["nodes"]:
            for t in node.findall(".//w:t", NS):
                if old in (t.text or ""):
                    t.text = t.text.replace(old, new)
                    changed += 1
        assert changed == 1, (prefix, old, changed)
        r["text"] = before.replace(old, new)
        changes.append({"operation": "inline", "source_block": r["source_block"],
                        "before": before, "after": r["text"]})

    def insert(prefix, text):
        p = deepcopy(records[index("PSTMO nhận đường")]["nodes"][0])
        scientific_runs(p, text)
        records.insert(index(prefix), {"text": text, "nodes": [p], "source_block": None})
        changes.append({"operation": "insert", "before": "", "after": text})

    def move(prefix, before):
        r = records.pop(index(prefix))
        records.insert(index(before), r)
        changes.append({"operation": "move", "source_block": r["source_block"],
                        "text": r["text"], "before_prefix": before})

    def remove(prefix):
        r = records.pop(index(prefix))
        changes.append({"operation": "remove", "source_block": r["source_block"],
                        "before": r["text"], "after": ""})

    # Algorithm order follows the implementation at the common-window gate.
    replace("Thời gian của các phương án được tính",
        "Thời gian của các phương án được tính trên cùng đoạn xử lý góc, với cùng giới hạn vận tốc đầu và cuối có thể đạt được. "
        "Gọi Tmin là thời gian ngắn nhất của các chuyển tiếp khả thi, Tp là thời gian quay tại chỗ trên đoạn này và δT là mức giảm thời gian tối thiểu yêu cầu. "
        "Khi quay tại chỗ khả thi, nhánh chuyển tiếp chỉ được mở nếu thỏa công thức 5.")
    move("@equation T_{\\min}", "Hàm chi phí trong công thức")
    # Number labels live outside OMML; mathematical objects stay unchanged.
    eq_time = records[index("@equation T_{\\min}")]
    eq_cost = records[index("@equation J=")]
    for record, old_label, new_label in [(eq_time, "(6)", "(5)"), (eq_cost, "(5)", "(6)")]:
        labels = [t for n in record["nodes"] for t in n.findall(".//w:t", NS) if t.text == old_label]
        assert len(labels) == 1
        labels[0].text = new_label
    insert("Hàm chi phí trong công thức",
        "Trong nhánh này, thời gian của chuyển tiếp được giữ không vượt Tmin cộng khoảng thời gian cho phép đã cấu hình và không lớn hơn Tp − δT, bỏ qua dung sai số học. "
        "Nếu quay tại chỗ không khả thi, chỉ áp dụng giới hạn so với Tmin.")
    inline("Hàm chi phí trong công thức", "công thức 5", "công thức 6")
    replace("Nếu quay tại chỗ không khả thi, thuật toán vẫn xét",
        "Nếu không có chuyển tiếp đáp ứng yêu cầu, thuật toán sử dụng phương án quay tại chỗ khả thi. "
        "Chuỗi được chọn có thể kết hợp hai dạng chuyển động và được kiểm tra lại sau khi ghép.")
    replace("@note Ký hiệu δT",
        "@note Ký hiệu δT là mức giảm thời gian tối thiểu so với quay tại chỗ. Khoảng thời gian cho phép so với Tmin được chặn thêm bởi điều kiện tại Mục II-D. "
        "Khoảng cách lấy mẫu đường đầu ra khác khoảng cách kiểm tra đường cong; khoảng dự phòng 0,05 m được suy ra từ cấu hình và độ phân giải bản đồ.")

    # State experimental purposes before outcomes, without invented preregistration.
    inline("Ma trận khảo sát gồm 125 lượt",
           "R01 sử dụng dữ liệu ngày 02–03/08/2026; R02–R05 thuộc đợt bổ sung sau đó. ", "")
    replace("Năm tuyến khảo sát các dạng chuyển hướng",
        "Năm tuyến khảo sát các dạng chuyển hướng trên cùng một bố trí vật cản: R01 xét chuyển lối tại giao cắt; R02 xét hành trình dài với các đoạn gần thẳng; "
        "R03 xét chuỗi góc liên tiếp; R04 bổ sung chiều tiếp cận và chuỗi chuyển hướng khác; R05 tập trung chuyển hướng và tiếp cận đích trên tuyến ngắn. "
        "Các tuyến dùng chung một số hành lang. R04 khác R02 cả điểm đầu, điểm đích và hình học đường, nên không tạo thành thí nghiệm tách riêng ảnh hưởng của chiều tiếp cận.")
    replace("Năm phương án xử lý gồm",
        "Năm phương án gồm đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO. Đường gốc giữ nguyên đầu ra của bộ lập kế hoạch để đối chiếu thay đổi do hậu xử lý. "
        "Simple làm mượt lặp; Savitzky–Golay, viết tắt SG trong bảng và hình, dùng bộ lọc bình phương tối thiểu cục bộ [2]; Constrained tối ưu số các thành phần chất lượng đường [3]. "
        "Ba đối chứng làm mượt cho phép so sánh các cách xử lý khác nhau dưới cùng bộ điều khiển. Bảng IV trình bày cấu hình từng phương án.")
    insert("Bộ kiểm chứng lặp gồm 250 lượt",
        "Một lượt cho mỗi tổ hợp chưa xác định được biến thiên khi thực thi lại cùng nhiệm vụ. Vì vậy, bộ kiểm chứng lặp đánh giá mức độ duy trì của chênh lệch hiệu năng khi giữ cố định đầu vào và cấu hình. "
        "Constrained cung cấp đối chứng tối ưu số trong cùng Nav2 để đối chiếu với cách xây dựng và lựa chọn chuỗi chuyển tiếp của PSTMO. "
        "Phạm vi lặp chỉ gồm hai phương án này, không thay thế so sánh đủ năm phương án hoặc xác lập ưu thế so với các đối chứng còn lại.")
    remove("Mỗi tổ hợp trong ma trận khảo sát chỉ được thực thi một lần,")

    # Replace and reorder the measurement section as a single auditable edit.
    a, b = index("### 7) Chỉ số đo"), index("## B. R01")
    old_metrics = records[a + 1:b]
    rmse_eq = next(r for r in old_metrics if r["text"].startswith("@equation "))
    metric_texts = [
        "Toàn bộ 125 bản ghi có đủ đường tham chiếu, quỹ đạo thực thi, độ giật và chuỗi trạng thái. Các chỉ số của cùng lượt được trích từ cùng bản ghi. "
        "Quỹ đạo thực thi được lấy mẫu lại theo chiều dài cung với bước 0,05 m, rồi làm mượt bằng trung bình trượt đối xứng với hai mẫu mỗi phía, giữ hai đầu đường. "
        "Quy trình này được áp dụng thống nhất trước khi tính RMSE và tích phân bình phương độ cong của quỹ đạo thực thi.",
        "Chiều dài đường tham chiếu L là tổng khoảng cách giữa các điểm đầu ra. Quãng đường thực thi D được tính từ các vị trí robot trước bước xử lý quỹ đạo. "
        "Ký hiệu Eκ,ref và Eκ,exec lần lượt chỉ tích phân bình phương độ cong của đường tham chiếu và quỹ đạo thực thi đã xử lý. Mức giảm Eκ,ref không mặc nhiên dẫn đến mức giảm tương ứng của Eκ,exec.",
        "Giá trị xấp xỉ của tích phân bình phương độ cong được tính từ các bộ ba điểm liên tiếp. Với độ dài ba cạnh a, b, c và diện tích tam giác có dấu A, độ cong mẫu bằng 4A/(abc). "
        "Mỗi giá trị κ² được nhân với nửa tổng chiều dài hai cạnh kề rồi cộng trên toàn đường; các bộ ba suy biến được bỏ qua theo ngưỡng số học. "
        "Độ cong cực đại κmax là cực đại trị tuyệt đối độ cong ước lượng trên các điểm đầu ra chưa lấy mẫu lại. Các chỉ số này phụ thuộc mật độ điểm.",
        "Để kiểm tra ảnh hưởng lấy mẫu, phần tịnh tiến của đường tham chiếu được tách tại các tư thế quay tại chỗ và lấy mẫu lại với bước 0,05 m, cho các chỉ số κt,max và Eκ,t. "
        "Dấu quay tại chỗ được nhận diện khi dịch chuyển không quá 0,0001 m và thay đổi hướng ít nhất 5°. Đây là dấu trong đường tham chiếu, không phải số lần robot thực sự xoay khi điều khiển.",
        "Sai số bám đường là khoảng cách nhỏ nhất từ vị trí xₖ trên quỹ đạo thực thi đã xử lý đến các đoạn của đường tham chiếu P. "
        "Với N vị trí, căn trung bình bình phương sai số, viết tắt RMSE, được xác định trong công thức 8. Mỗi phương án dùng đường tham chiếu riêng; RMSE phản ánh khả năng bám đường, không phải sai số định vị.",
        "@KEEP_RMSE_EQUATION",
        "Sai số bám cực đại emax dùng cùng phép đo khoảng cách và quỹ đạo đã xử lý như RMSE. Sai số cuối ep và eψ lần lượt là khoảng cách tới vị trí đích và trị tuyệt đối sai lệch hướng đã quy về [0; π], "
        "tính từ trạng thái cuối của mô phỏng. Sai số cuối phản ánh kết quả dừng tại đích, khác với sai số bám trên hành trình.",
        "Thời gian thực thi T được đo bằng đồng hồ mô phỏng, gồm thời gian điều hướng và thời gian chờ robot dừng. Thời lượng hậu xử lý do tác vụ làm mượt Nav2 trả về, được tính bằng đồng hồ của nút phần mềm [4]. "
        "Khi dùng thời gian mô phỏng, đại lượng này không phải thời gian CPU hoặc thời gian thực độc lập của máy tính.",
        "Khoảng hở c là khoảng cách nhỏ nhất giữa đường bao đã định hướng và vật cản trên bản đồ tĩnh. Phép đo trên đường tham chiếu nội suy tư thế với bước tịnh tiến 0,05 m và bước hướng 5°. "
        "Đường bao được lấy mẫu với bước không quá 0,025 m; trường khoảng cách được hiệu chỉnh nửa đường chéo ô. Ô chưa biết và ô có mức chiếm chỗ từ 65 trở lên được xem là vật cản. "
        "Độ phân giải bản đồ giới hạn cách diễn giải những chênh lệch khoảng hở rất nhỏ.",
        "Phân vị 95% của trị tuyệt đối độ giật tịnh tiến mô tả biến thiên gia tốc. Độ giật được ước lượng từ chuỗi vận tốc mô phỏng bằng sai phân và phụ thuộc nhịp lấy mẫu. "
        "Vận tốc bánh tương đương được suy từ v ± bω/2 với b = 0,2548 m; gia tốc ngang là |vω|. Gia tốc và độ giật dùng bước thời gian hợp lệ từ 0,0001 đến 0,5 s. "
        "Cực đại chỉ mô tả các mẫu đã ghi, chưa chứng minh ràng buộc liên tục giữa các mẫu. Hệ thống còn ghi số lần giám sát va chạm can thiệp và số mẫu đường bao va chạm; chưa đo điện năng, rung tải hàng hoặc lực tiếp xúc.",
        "Các chỉ số hình học dùng toàn bộ 25 đường của mỗi phương án; trung bình thực thi dùng tập lượt đạt và ghi rõ mẫu số. So sánh ghép cặp chỉ dùng các lượt cùng đầu vào mà cả hai phương án đạt. "
        "Trong phân tích bộ lặp, thời gian, RMSE và Eκ,exec là ba chỉ số chính được hiệu chỉnh đa kiểm định; các chỉ số còn lại bổ sung thông tin về đánh đổi. Phương pháp thống kê được trình bày tại Mục III-H."
    ]
    template = records[index("PSTMO nhận đường")]["nodes"][0]
    new_metrics = []
    for text in metric_texts:
        if text == "@KEEP_RMSE_EQUATION":
            new_metrics.append(rmse_eq)
        else:
            p = deepcopy(template)
            scientific_runs(p, text)
            new_metrics.append({"text": text, "nodes": [p], "source_block": None})
    changes.append({"operation": "rewrite_measurement_section", "source_blocks": [r["source_block"] for r in old_metrics],
                    "before": [r["text"] for r in old_metrics], "after": [r["text"] for r in new_metrics]})
    records[a + 1:b] = new_metrics

    replace("### 2) Thời gian thực thi và tích phân", "### 2) Hình học đường và kết quả thực thi")
    replace("### 3) So sánh với Simple", "### 3) So sánh các phương án và đánh đổi")
    # Keep the planner-wise figure beside the corresponding interpretation;
    # let the compact aggregate table fill the preceding column naturally.
    move("@figure r02_metrics ",
         "So sánh theo bộ lập kế hoạch cho thấy PSTMO nhanh hơn Constrained ở bốn trong năm cặp.")
    replace("R04 nối (−4,6; 2,8)",
        "R04 nối (−4,6; 2,8) m với (2,0; −2,8) m và kết thúc ở hướng −π/2. Tuyến này bổ sung trường hợp tiếp cận hành lang theo chiều khác, "
        "với chuỗi chuyển hướng và tư thế đích khác R02. Kết quả mô tả toàn bộ nhiệm vụ, không tách riêng ảnh hưởng của thứ tự các góc rẽ.")
    move("So sánh theo bộ lập kế hoạch cho thấy PSTMO nhanh hơn Constrained ở bốn cặp;",
         "### 3) Trường hợp sát ngưỡng")
    remove("Bộ kiểm chứng lặp ghi nhận một số lượt kết thúc trước đích ngay trên R05.")
    replace("### 3) Khoảng hở và chuyển tiếp", "### 3) Khoảng hở và độ chính xác cuối")
    inline("Sai số vị trí cuối trung bình của năm phương án theo thứ tự trên là 37,1;",
        "Kết quả dừng không bù cho RMSE cao hơn: lợi ích chính trên R02 là thời gian trung bình, không phải độ chính xác bám đường.",
        "Sai số cuối và RMSE phản ánh hai khía cạnh khác nhau. Trên R02, thời gian trung bình giảm nhưng sai số bám đường trung bình tăng.")
    inline("Sai số vị trí cuối trung bình của năm phương án theo thứ tự trên là 40,3;",
        "định vị cuối tốt hơn", "sai số vị trí cuối nhỏ hơn")

    # Clarify different curvature measures across the aggregate table and figure.
    replace("Bảng XVI ghép PSTMO",
        "Bảng XVI ghép PSTMO với từng đối chứng cùng tuyến và bộ lập kế hoạch, chỉ giữ cặp cùng hoàn thành. Có 23 cặp với đường gốc và 24 cặp với mỗi phương án làm mượt. "
        "PSTMO giảm thời gian, RMSE và Eκ,exec trung bình trên các cặp này; mức giảm thời gian nhỏ nhất khi so với Simple.")
    replace("Hình 15 cho thấy chênh lệch",
        "Hình 15 cho thấy chênh lệch thời gian và RMSE có thể đổi dấu giữa các đầu vào dù trung bình âm. Hàng cuối của hình dùng Eκ,ref trên đủ 25 cặp đường, "
        "khác với Eκ,exec trên các cặp cùng đạt trong Bảng XVI. Mục III-B–F phân tích theo tuyến; phần tiếp theo kiểm tra ảnh hưởng mật độ điểm tới chỉ số độ cong tham chiếu.")
    insert("@figure execution_trace ",
        "Hình 16 minh họa tín hiệu của R02 với Theta*, bộ lập kế hoạch cũng được dùng trong các hình đường tham chiếu theo tuyến. "
        "Trường hợp này giúp đối chiếu đường tham chiếu, chuyển động thực thi và vận tốc góc trong cùng lượt; không được dùng để ước lượng hiệu năng chung của năm tuyến.")
    replace("Hình 16 cho thấy các phương án đều xuất hiện",
        "Các tín hiệu trong Hình 16 đều có đoạn vận tốc góc dương gần 0,25 rad/s, nhưng diễn biến theo thời gian khác nhau. "
        "Ở lượt R02/Theta* này, PSTMO hoàn thành trong 55,764 s, nhanh hơn Constrained 0,939 s nhưng chậm hơn Simple 0,636 s. "
        "Vùng quỹ đạo phóng to chỉ biểu diễn một phần hành trình, còn đồ thị vận tốc bao quát cả lượt. Biên độ vận tốc góc gần nhau chưa giải thích chênh lệch thời gian toàn nhiệm vụ; "
        "các tín hiệu không tách được tác động riêng của hình học đường và bộ điều khiển.")
    replace("### 1) Thiết kế và phương pháp phân tích", "### 1) Phương pháp phân tích thống kê")
    move("@figure endpoints ", "### 3) Phạm vi kết luận")
    # Failure descriptions and all numerical results stay intact. Condense only
    # repeated design limitations, retaining their consequences for inference.
    replace("Nghiên cứu đánh giá năm dạng tuyến trên một bản đồ tĩnh,",
        "Khảo sát dùng một bản đồ tĩnh, một robot mô phỏng và một cấu hình RPP. R01 được thu ở đợt khác R02–R05; bộ lặp chỉ gồm PSTMO và Constrained. "
        "Chưa xác nhận thứ tự chạy được ngẫu nhiên hóa; bốn tiến trình dùng chung tài nguyên máy tính dù có miền truyền thông riêng. "
        "Các điều kiện này hạn chế khả năng suy rộng và cách diễn giải thời lượng xử lý.")
    replace("Constrained được đánh giá với trọng số độ cong bằng không.",
        "Kết quả chỉ đại diện cho các cấu hình ở Mục III-A, chưa có tinh chỉnh với cùng ngân sách đánh giá tham số trên tập hiệu chỉnh tách biệt. "
        "Phân tích cặp cùng đạt có thể thiên lệch về các nhiệm vụ dễ hoàn thành; vì vậy phải đọc cùng tỷ lệ thành công và các trường hợp không đạt.")

    # The empty section-break marker has no manuscript content. Prevent its
    # inherited body line height from creating a blank trailing page.
    marker_properties = tail[0].find("w:pPr", NS)
    assert plain(tail[0]) == "" and marker_properties.find("w:spacing", NS) is None
    marker_spacing = etree.Element(W + "spacing")
    for key, value in {"before": "0", "after": "0", "line": "20", "lineRule": "exact"}.items():
        marker_spacing.set(W + key, value)
    marker_properties.insert(0, marker_spacing)
    marker_run_properties = etree.Element(W + "rPr")
    etree.SubElement(marker_run_properties, W + "sz").set(W + "val", "2")
    etree.SubElement(marker_run_properties, W + "szCs").set(W + "val", "2")
    marker_properties.insert(1, marker_run_properties)
    marker_properties.remove(marker_properties.find("w:sectPr", NS))
    tail[-1].find("w:cols", NS).set(W + "num", "2")
    changes.append({"operation": "empty_section_marker_spacing", "line_pt": 1,
                    "body_font_or_spacing_changed": False, "empty_terminal_section_removed": True})
    body = root.find("w:body", NS)
    for node in list(body):
        body.remove(node)
    for record in records:
        for node in record["nodes"]:
            body.append(node)
    for node in tail:
        body.append(node)
    parts["word/document.xml"] = etree.tostring(root, xml_declaration=True, encoding="UTF-8", standalone=True)
    core = etree.fromstring(parts["docProps/core.xml"])
    core.find("{http://purl.org/dc/elements/1.1/}subject").text = "Phần II và III của bản thảo journal phiên bản 8"
    parts["docProps/core.xml"] = etree.tostring(core, xml_declaration=True, encoding="UTF-8", standalone=True)
    OUT.mkdir(exist_ok=True)
    output = OUT / "PSTMO_JOURNAL_VER8_PHAN_II_III.docx"
    with ZipFile(output, "w") as z:
        for info in infos:
            z.writestr(info, parts[info.filename])
    new_source = "\n\n".join(r["text"] for r in records) + "\n"
    (OUT / "manuscript.md").write_text(new_source)
    (OUT / "table_values.json").write_bytes((OLD / "table_values.json").read_bytes())
    (OUT / "revision_log.json").write_text(json.dumps(changes, ensure_ascii=False, indent=2) + "\n")
    inputs = [original, OLD / "PSTMO_JOURNAL_VER7_PHAN_II_III.docx", OLD / "manuscript.md", OLD / "provenance.json", OLD / "quality_checks.json",
              OLD / "table_values.json", OLD / "editorial_evidence.json", Path(__file__),
              ROOT / "tools/build_journal_sections_v7.py", ROOT / "tools/build_journal_sections_v6.py",
              ROOT / "src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp",
              ROOT / "src/adaptive_pivot_g2/src/candidate_selection.cpp",
              ROOT / "results/pstmo_journal_repeatability_20261011/study_manifest.json"]
    import hashlib
    provenance = {"docx": output.name, "docx_sha256": sha(output),
        "reference": str(original.relative_to(ROOT)), "reference_sha256": sha(original),
        "source_sha256": {str(p.relative_to(ROOT)): sha(p) for p in inputs},
        "output_sha256": {p.name: sha(p) for p in [OUT / "manuscript.md", OUT / "table_values.json", OUT / "revision_log.json"]},
        "package_parts": {n: hashlib.sha256(data).hexdigest() for n, data in parts.items()},
        "changed_parts_allowed": ["word/document.xml", "docProps/core.xml"],
        "equation_label_permutation": {"5": 6, "6": 5},
        "counts": prior_prov["counts"], "simulation_runs_added": 0,
        "changes_count": len(changes), "date": "2026-10-21"}
    (OUT / "provenance.json").write_text(json.dumps(provenance, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(output), "changes": len(changes),
                      "word_delta": len(new_source.split()) - len(source.split())}, ensure_ascii=False))


if __name__ == "__main__":
    main()
