#!/usr/bin/env python3
"""Evidence-led, package-preserving refinement of PSTMO journal sections II–III.

Only ver9 outputs are written. Historical builders are never executed.
The v7 module supplies read-only mapping/hash helpers, not an authoring pipeline.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re
import subprocess
from zipfile import ZipFile

from lxml import etree

from build_journal_sections_v7 import NS, W, map_chunks, plain, sha

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / "Viet_bao/journal/ver8"
OUT = ROOT / "Viet_bao/journal/ver9"
REFERENCE = OLD / "PSTMO_JOURNAL_VER8_PHAN_II_III.docx"
REFERENCE_SHA = "bb3e225b95938fa54138ef6474ff3fddab675a757656309ef0ee17a2249ccc8e"
DATE = "2026-10-22"
COHORT = "results/pstmo_journal_repeatability_20261011"
CORE = "src/adaptive_pivot_g2/src/"
PLUGIN = "src/adaptive_pivot_g2_nav2/src/"
CONFIG = "src/vacuum_robot_gazebo/config/nav2_params.yaml"
SOURCES = [CORE + name + ".cpp" for name in [
    "quintic_transition", "hierarchical_shape_search", "candidate_selection",
    "path_optimization", "time_parameterization", "path_conditioning"]] + [
    PLUGIN + "adaptive_pivot_g2_smoother.cpp", PLUGIN + "footprint_safety.cpp",
    CONFIG, "src/vacuum_robot_gazebo/models/vacuum_robot/model.sdf"]


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def scientific_runs(p, text):
    """Keep newly clarified scientific notation editable and subscripted."""
    forms = {"Eκ,ref": ("E", "κ,ref"), "Eκ,exec": ("E", "κ,exec"),
             "Eκ,t": ("E", "κ,t"), "Eκ": ("E", "κ"), "κmax": ("κ", "max"),
             "κt,max": ("κ", "t,max"), "emax": ("e", "max"), "j95": ("j", "95"),
             "ep": ("e", "p"), "eψ": ("e", "ψ"), "δT": ("δ", "T"),
             "Tmin": ("T", "min"), "Tp": ("T", "p"), "Rb": ("R", "b"),
             "db": ("d", "b"), "dmax": ("d", "max"), "dmin": ("d", "min"),
             "vL": ("v", "L"), "vR": ("v", "R"), "cmax": ("c", "max"),
             "ωpeak": ("ω", "peak"), "ωmax": ("ω", "max"), "Escale": ("E", "scale"),
             "ρc": ("ρ", "c"), "ρω": ("ρ", "ω"), "ρE": ("ρ", "E"),
             "yP,br": ("y", "P,br"), "yC,br": ("y", "C,br")}
    for child in list(p):
        if child.tag != W + "pPr":
            p.remove(child)
    pattern = "(" + "|".join(re.escape(k) for k in sorted(forms, key=len, reverse=True)) + ")"
    for part in re.split(pattern, text):
        if not part:
            continue
        runs = [(part, None)] if part not in forms else [(forms[part][0], "i"), (forms[part][1], "subscript")]
        for value, kind in runs:
            run = etree.SubElement(p, W + "r")
            if kind:
                pr = etree.SubElement(run, W + "rPr")
                etree.SubElement(pr, W + "i" if kind == "i" else W + "vertAlign",
                                 {} if kind == "i" else {W + "val": kind})
            node = etree.SubElement(run, W + "t")
            node.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
            node.text = value


def validate_sources():
    """Check the recorded lineage; do not infer a historical build from HEAD."""
    assert sha(REFERENCE) == REFERENCE_SHA, "ver8 changed; inspect before editing"
    checked = {}
    for version in [6, 7, 8]:
        provenance = json.loads((ROOT / f"Viet_bao/journal/ver{version}/provenance.json").read_text())
        for name, expected in provenance["source_sha256"].items():
            actual = sha(ROOT / name)
            assert actual == expected, f"Historical source mismatch: {name}"
            checked[name] = actual
    manifest = json.loads((ROOT / COHORT / "study_manifest.json").read_text())
    assert manifest["status"] == "complete"
    assert sha(ROOT / CONFIG) == manifest["nav2_params_sha256"]
    comparisons = {}
    for name in SOURCES:
        historical = subprocess.run(
            ["git", "show", f"{manifest['git_commit']}:{name}"], cwd=ROOT,
            check=True, capture_output=True).stdout
        historical_hash = hashlib.sha256(historical).hexdigest()
        assert historical_hash == sha(ROOT / name), f"Cohort code/config drift: {name}"
        comparisons[name] = historical_hash
    aggregate = json.loads((ROOT / COHORT / "analysis/aggregate.json").read_text())
    assert aggregate["trial_count"] == 250 and aggregate["paired_trial_count"] == 125
    assert aggregate["block_count"] == 25 and not aggregate["audit_issues"]
    assert aggregate["success"]["constrained"] == 122
    assert aggregate["success"]["pstmo"] == 121 and len(aggregate["failures"]) == 7
    assert all(m["paired_successful_trial_count"] == 118 for m in aggregate["metrics"].values())
    return {"historical_hashes_checked": checked,
            "cohort_commit": manifest["git_commit"],
            "cohort_source_matches": comparisons,
            "scope": "Matches the recorded repeatability commit and ver6–8 provenance; does not claim a fully pinned third-party binary environment for every historical trial.",
            "aggregate_invariants_verified": True,
            "statistics_recomputed": False}


def edit_plan(records):
    edits = []

    def add(id_, section, prefix, text, reason, sources, kind="clarification"):
        hits = [(i, r) for i, r in enumerate(records) if r["text"].startswith(prefix)]
        assert len(hits) == 1, (id_, prefix)
        i, r = hits[0]
        assert len(r["nodes"]) == 1 and not r["text"].startswith(("#", "@equation", "@table", "@figure"))
        edits.append({"id": id_, "section": section, "kind": kind,
                      "source_block": i, "body_xpath": r["nodes"][0].getroottree().getpath(r["nodes"][0]),
                      "before": r["text"], "after": text, "reason": reason, "sources": sources})

    add("A01", "II-A", "Bước tiền xử lý loại",
        "Bước tiền xử lý loại các điểm liên tiếp trùng nhau, rút gọn đường theo giới hạn độ lệch và xử lý dao động hình học cục bộ; các đoạn thay thế đều được kiểm tra va chạm. Cấu hình khảo sát không bật bước rút gọn riêng theo đường nhìn giữa các điểm. Tiền xử lý vẫn có thể thay đổi hình học đường đầu vào. Kết quả thực nghiệm vì thế đánh giá toàn bộ PSTMO, bao gồm tiền xử lý và lựa chọn chuyển tiếp.",
        "Ver8 chưa nêu bước rút gọn theo giới hạn độ lệch trong condition_polyline, dễ khiến condition_only bị hiểu là chỉ loại điểm/dao động.",
        [CORE + "path_conditioning.cpp:condition_polyline", PLUGIN + "adaptive_pivot_g2_smoother.cpp:prepare_path"], "implementation_correction")
    add("B01", "II-B", "Chiều dài cắt d là",
        "Chiều dài cắt d là khoảng cách từ đỉnh C đến điểm nối trên mỗi cạnh kề. Với d > 0 và 0 < α ≤ 0,5, đặt A = C − du, B = C + dw và q = αd. Sáu điểm điều khiển được xác định trong công thức 1; công thức 2 biểu diễn đường Bézier bậc năm theo tham số hình học t, không phải thời gian. Hình 2 thể hiện các điểm nối và hướng tiếp tuyến tại hai đầu.",
        "Bổ sung miền α của triển khai và phân biệt tham số đường cong với thời gian.",
        [CORE + "quintic_transition.cpp:generate_quintic_transition_impl"])
    add("B02", "II-B", "Ba điểm điều khiển ở mỗi đầu",
        "Ba điểm điều khiển ở mỗi đầu thẳng hàng và cách đều. Do đó, r′(0) = 5qu, r′(1) = 5qw và r″(0) = r″(1) = 0. Vì q > 0 và u, w là vectơ đơn vị, đạo hàm bậc nhất tại hai đầu khác không. Độ cong hai đầu bằng không, trùng với cạnh thẳng. Các mối nối này đạt G² về vị trí, hướng tiếp tuyến và độ cong; tính chất đó không áp dụng cho quay tại chỗ và không bảo đảm liên tục theo thời gian của vận tốc, gia tốc hoặc độ giật khi thực thi.",
        "Nêu trực tiếp điều kiện chính quy tại điểm nối, giới hạn G² tại các mối nối được xây dựng; không gán bảo đảm động học theo thời gian.",
        [CORE + "quintic_transition.cpp:evaluate", "Suy ra trực tiếp từ sáu điểm điều khiển trong công thức 1"])
    add("B03", "II-B", "Công thức 3 xác định độ cong",
        "Công thức 3 xác định độ cong κ và tích phân bình phương độ cong Eκ theo chiều dài cung. Eκ có đơn vị m⁻¹, không biểu thị năng lượng tiêu thụ; giá trị nhỏ hơn không có nghĩa độ cong nhỏ hơn tại mọi điểm. Trong tìm hình dạng, độ cong được tính từ đạo hàm Bézier tại các mẫu; Eκ được xấp xỉ bằng quy tắc hình thang với chiều dài dây cung. Thuật toán loại đường khi phát hiện đạo hàm suy biến, giá trị không hữu hạn hoặc đổi dấu độ cong ngoài dung sai tại các mẫu kiểm tra.",
        "Phân biệt tích phân lý thuyết, xấp xỉ nội bộ và phép đo ba điểm ở III-A; giới hạn phát hiện suy biến/đổi dấu theo mẫu.",
        [CORE + "quintic_transition.cpp:sample_curve", CORE + "quintic_transition.cpp:generate_quintic_transition_impl"])
    add("C01", "II-C", "Với khoảng cách hai bánh b,",
        "Với khoảng cách hai bánh b, vận tốc tịnh tiến v và độ cong có dấu κ, vận tốc dài tại bánh trái và phải là vL = v(1 − bκ/2) và vR = v(1 + bκ/2). Chuyển tiếp tiến yêu cầu hai hệ số 1 ± bκ/2 không âm; quay tại chỗ được xét riêng. Công thức 4 kết hợp giới hạn vận tốc thân xe, vận tốc góc, gia tốc ngang và vận tốc bánh. Các lượt quét xuôi–ngược áp đặt tăng tốc, giảm tốc; vòng lặp tiếp tục hạ giới hạn vận tốc để thỏa gia tốc góc ước lượng giữa các mẫu. Hồ sơ không hội tụ bị loại. Tính khả thi theo mô hình này chưa xác định khả năng bám đường của bộ điều khiển.",
        "Ver8 gộp áp đặt gia tốc góc vào hai lượt quét; mã dùng vòng lặp hạ caps và kiểm tra hội tụ. Làm rõ vL/vR là vận tốc dài, không phải vận tốc góc bánh.",
        [CORE + "time_parameterization.cpp:parameterize_time", CORE + "quintic_transition.cpp:generate_quintic_transition_impl"], "implementation_correction")
    add("C02", "II-C", "Tại đoạn thẳng, các hạng",
        "Tại đoạn thẳng, các hạng chia cho |κ| trong công thức 4 không tạo thêm giới hạn hữu hạn. Chi phí tại tâm phải qua ngưỡng cấu hình; vùng hình bao đã định hướng phải không chứa ô vật cản gây va chạm hoặc ô chưa biết. Ngưỡng chi phí mở rộng quanh vật cản được áp tại tâm, không áp thêm cùng ngưỡng lên toàn đường bao. Kiểm tra chuyển tiếp nội suy cả vị trí và hướng giữa các mẫu; quay tại chỗ được lấy mẫu theo góc. Đây vẫn là kiểm tra hình bao tại các tư thế rời rạc, phụ thuộc độ phân giải và khoảng lấy mẫu, chưa bảo đảm an toàn liên tục.",
        "Làm rõ phân biệt ngưỡng inflation tại tâm với va chạm footprint, và nội suy tư thế không phải chứng minh swept volume liên tục.",
        [PLUGIN + "footprint_safety.cpp:pose_is_safe", PLUGIN + "adaptive_pivot_g2_smoother.cpp:transition_is_safe"])
    add("D01", "II-D", "Hai phương án được so sánh",
        "Sau tiền xử lý, độ đổi hướng được xác định từ hai vectơ cạnh kề. Đỉnh có độ đổi hướng nhỏ hơn 5° được giữ làm điểm đi qua, không tạo dấu quay tại chỗ. Với các góc còn lại, hai dạng chuyển động được xét là chuyển tiếp tiến và dừng để quay tại chỗ. Bộ sinh Bézier chỉ nhận góc nhỏ hơn 170°; ngoài miền này, thuật toán chỉ còn phương án quay tại chỗ nếu khả thi.",
        "Thiếu ngưỡng phát hiện góc, pass-through và miền góc của bộ sinh; tránh diễn giải toàn bộ đường đầu ra đều G².",
        [PLUGIN + "adaptive_pivot_g2_smoother.cpp:smooth_pipeline", "src/adaptive_pivot_g2/include/adaptive_pivot_g2/types.hpp:TransitionOptions", CONFIG])
    add("D02", "II-D", "PSTMO xét hai chiều dài cắt",
        "PSTMO xét tối đa hai chiều dài cắt tại mỗi góc. Chiều dài ưu tiên là giá trị nhỏ nhất của hai cạnh kề và dmax. Chiều dài tương thích được chặn thêm bởi phần dành cho mỗi đầu của từng cạnh chung, bằng max(0, L − m)/2; cạnh nối trực tiếp điểm đầu hoặc đích được dùng toàn chiều dài. Thuật toán bỏ giá trị dưới dmin hoặc trùng chiều dài ưu tiên trong độ phân giải tìm kiếm. Cách tạo này dành chiều dài cho cả góc trước và góc sau.",
        "Ver8 chỉ nói dành phần cạnh cho góc tiếp theo; công thức triển khai xét cả hai cạnh, trường hợp biên và tối đa hai giá trị.",
        [CORE + "hierarchical_shape_search.cpp:derive_two_trim_candidates", "src/adaptive_pivot_g2/test/test_hierarchical_shape_search.cpp:DerivedTrimCandidates"], "implementation_correction")
    add("D03", "II-D", "Với mỗi chiều dài cắt, thuật toán",
        "Với mỗi chiều dài cắt, thuật toán xét α = {0,1; 0,2; 0,3; 0,4; 0,5}; nếu chưa có hình dạng khả thi thì xét thêm {0,15; 0,25; 0,35; 0,45}. Quanh giá trị có Eκ nhỏ nhất, khoảng tinh chỉnh nối hai mốc thô kề hoặc là ô thô chứa giá trị bổ sung; tại biên miền chỉ dùng khoảng một phía. Khoảng này được chia thành mười phần bằng nhau và bỏ mẫu trùng. Mỗi giá trị phải qua kiểm tra hình học, hình bao và tham số hóa thời gian. Sau tinh chỉnh, chỉ giữ một hình dạng có Eκ nhỏ nhất cho mỗi chiều dài cắt; khi bằng nhau trong dung sai, ưu tiên α nhỏ hơn. Hàm chi phí tổng hợp ở bước sau không được tối ưu trực tiếp trên toàn miền α.",
        "Bổ sung quy tắc tái lập refinement, khả thi và mỗi trim chỉ giữ một shape; không nhầm mục tiêu Eκ với J.",
        [CORE + "hierarchical_shape_search.cpp:search_control_fraction_coarse_to_fine", PLUGIN + "adaptive_pivot_g2_smoother.cpp:evaluate_shape"])
    add("D04", "II-D", "Thời gian của các phương án được tính",
        "Các chuyển tiếp sau tìm hình dạng được bổ sung đoạn thẳng để cùng bắt đầu và kết thúc trên cửa sổ có chiều dài cắt lớn nhất. Vận tốc biên chung được hạ lặp đến trạng thái có thể đạt, rồi tính lại hồ sơ; phương án không đáp ứng trạng thái biên bị loại. Tmin là thời gian dự đoán nhỏ nhất trong tập còn lại. Tp gồm thời gian tiến đến đỉnh, dừng–quay và rời đỉnh trên cùng cửa sổ với cùng vận tốc biên. Khi quay tại chỗ khả thi, nhánh chuyển tiếp chỉ mở nếu đạt mức giảm tối thiểu δT trong công thức 5.",
        "Định nghĩa Tmin trên tập hữu hạn sau tìm hình dạng, Tp gồm tịnh tiến và quay; giải thích cửa sổ và biên chung.",
        [PLUGIN + "adaptive_pivot_g2_smoother.cpp:parameterize_common_window", CORE + "time_parameterization.cpp:estimate_pivot_window_time"])
    add("D05", "II-D", "Hàm chi phí trong công thức",
        "Hàm chi phí trong công thức 6 cung cấp chi phí cục bộ cho bước chọn chuỗi. Gọi cmax là cực đại, trên các tư thế kiểm tra chuyển tiếp, của giá trị lớn hơn giữa chi phí tại tâm và chi phí trên đường bao; đây là chỉ số chi phí bản đồ, không phải khoảng hở đo bằng mét. ωpeak là trị tuyệt đối vận tốc góc lớn nhất của hồ sơ dự đoán trên cửa sổ chung. Các thành phần không thứ nguyên là ρc = min(1, cmax/252), ρω = min(1, ωpeak/ωmax) và ρE = Eκ/(Eκ + Escale), với Escale = 1 m⁻¹. Ba trọng số có tổng bằng một và là cấu hình dùng trong nghiên cứu; chưa có bằng chứng xác lập bộ trọng số tối ưu.",
        "Sửa định nghĩa cmax: pose_is_safe trả max(center_cost, footprint_cost), transition_is_safe lấy cực đại qua tư thế. Bổ sung đơn vị Escale và giới hạn diễn giải trọng số.",
        [PLUGIN + "footprint_safety.cpp:pose_is_safe", PLUGIN + "adaptive_pivot_g2_smoother.cpp:transition_is_safe", CORE + "candidate_selection.cpp:stable_candidate_cost", CONFIG], "implementation_correction")
    add("D06", "II-D", "Nếu không có chuyển tiếp đáp ứng yêu cầu,",
        "Điều kiện thời gian và khoảng cho phép lọc tập phương án trước khi so sánh chi phí, nên thời gian không phải một hạng bù trừ trong J. Phương án quay tại chỗ khả thi nhận chi phí quy ước 1 khi nhánh chuyển tiếp mở, và 0 khi nhánh đóng; không áp Eκ cho thao tác quay. Các chuyển tiếp qua bộ lọc và phương án quay khả thi được đưa vào quy hoạch động. Vì phải bảo đảm tương thích giữa các góc, một góc vẫn có thể được chọn quay tại chỗ dù có chuyển tiếp khả thi cục bộ.",
        "Nêu logic của hai tầng dưới dạng quy tắc triển khai; pivot là chi phí quy ước và vẫn có thể được DP chọn do xung đột cạnh chung.",
        [PLUGIN + "adaptive_pivot_g2_smoother.cpp:transition_branch_open", CORE + "path_optimization.cpp:optimize_corner_states"])
    add("E01", "II-E", "Gọi Sᵢ là tập phương án",
        "Gọi Sᵢ là tập phương án khả thi được giữ tại góc i; mỗi phương án là một trạng thái của quy hoạch động. Chuyển tiếp dùng chiều dài cắt dᵢ, còn quay tại chỗ dùng chiều dài cắt bằng không. Chọn riêng phương án có J nhỏ nhất ở từng góc có thể làm các chuyển tiếp chồng lấn trên cạnh chung. Quy hoạch động chỉ nối các trạng thái tương thích và tối thiểu hóa tổng chi phí cục bộ. Công thức 7 xác định Fᵢ(k), tổng chi phí nhỏ nhất đến phương án k tại góc i; khởi tạo F₁(k) = J₁(k). Trạng thái không có tiền nhiệm đạt tới được nhận chi phí vô cùng. Truy vết từ trạng thái có chi phí nhỏ nhất ở góc cuối xác định chuỗi được chọn.",
        "Bổ sung lý do cần DP, khởi tạo, trạng thái không đạt tới và điều kiện chọn trạng thái cuối.",
        [CORE + "path_optimization.cpp:optimize_corner_states"])
    add("E02", "II-E", "Hai chiều dài cắt và phép tìm kiếm",
        "Hai chiều dài cắt và phép tìm kiếm phân cấp giới hạn số đường cong được đánh giá nhưng có thể bỏ sót hình dạng khả thi hoặc có chi phí thấp hơn. Tối ưu trên đồ thị không phải tối ưu toàn cục trên mọi đường liên tục hay thời gian thực thi. Tính G² thuộc các mối nối Bézier–đoạn thẳng đã xây dựng; chuỗi tư thế rời rạc, các điểm đi qua dưới ngưỡng và thao tác quay không tạo thành bảo đảm G² cho toàn bộ đầu ra. Phần III đánh giá riêng chuyển động dưới bộ điều khiển.",
        "Giới hạn phát biểu G²/tối ưu theo đúng đối tượng; đầu ra không phải quỹ đạo thời gian tối ưu.",
        [CORE + "hierarchical_shape_search.cpp", PLUGIN + "adaptive_pivot_g2_smoother.cpp:build_output_path"])
    add("R01", "III-A.2", "RPP sử dụng vận tốc đặt",
        "RPP sử dụng vận tốc đặt 0,30 m/s và khoảng nhìn trước cố định 0,50 m [1]. Điều chỉnh vận tốc theo độ cong và chi phí được tắt; kiểm tra va chạm, xoay hướng và giảm tốc gần đích vẫn hoạt động. Giá trị 0,25 rad/s trong Bảng I là vận tốc đặt cho xoay hướng, không phải giới hạn tuyệt đối của mọi lệnh vận tốc góc khi bám đường. Mô hình PSTMO và bộ làm mượt vận tốc dùng giới hạn 0,80 rad/s. Bộ làm mượt còn áp đặt tăng tốc, giảm tốc; giám sát va chạm có thể điều chỉnh lệnh trước khi tới robot. Do đó, thời gian dự đoán trong PSTMO không phải cam kết thời gian thực thi của hệ điều khiển.",
        "Phân biệt vận tốc xoay hướng RPP với giới hạn góc PSTMO/downstream và dự đoán thời gian với thực thi.",
        [CONFIG, "https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp:computeVelocityCommands,rotateToHeading"])
    add("R02", "III-A.7", "Sai số bám đường là khoảng cách nhỏ nhất",
        "Sai số bám đường là khoảng cách nhỏ nhất từ vị trí xₖ trên quỹ đạo thực thi đã xử lý đến các đoạn của đường tham chiếu P. Với N vị trí, căn trung bình bình phương sai số, viết tắt RMSE, được xác định trong công thức 8. Mỗi phương án dùng đường tham chiếu riêng; RMSE phản ánh khả năng bám đường, không phải sai số định vị. Do lấy mẫu theo chiều dài cung, đây là chỉ số trên không gian đường đi, không phải trung bình theo thời gian; nó không mô tả riêng sai số hướng khi quay tại chỗ.",
        "Làm rõ đối tượng đo RMSE và giới hạn với quay tại chỗ, dựa trên phép lấy mẫu lại theo cung đã công bố.",
        ["src/adaptive_pivot_g2_benchmark/adaptive_pivot_g2_benchmark/execution_trial.py", "src/adaptive_pivot_g2_benchmark/adaptive_pivot_g2_benchmark/compare_paths.py"])
    add("H01", "III-H.1", "Toàn bộ 125 cặp có cùng đường gốc.",
        "Trong toàn bộ 125 cặp, hai phương án nhận cùng đường gốc trong từng cặp. Constrained đạt 122/125 và PSTMO đạt 121/125; có 118 cặp cùng đạt để phân tích chỉ số liên tục. Mỗi tổ hợp tuyến–bộ lập kế hoạch tạo một nhóm b. Gọi Rb là tập lần lặp cùng đạt trong nhóm; db là trung bình hiệu PSTMO trừ Constrained. Công thức 9 tính chênh lệch chung Δ bằng trung bình đều của 25 nhóm, tránh coi các lần lặp cùng đầu vào là các đầu vào độc lập. Các đại lượng yP,br và yC,br là giá trị chỉ số của hai phương án tại lần lặp r trong nhóm b.",
        "Sửa câu dễ bị hiểu 125 cặp dùng duy nhất một Raw path; bổ sung biến y trong công thức 9.",
        [COHORT + "/analysis/aggregate.json", "tools/analyze_journal_repeatability.py"], "ambiguity_correction")
    add("H02", "III-H.1", "Khoảng tin cậy 95% dùng bootstrap",
        "Khoảng tin cậy 95% của Δ dùng bootstrap phân vị trên 25 nhóm, lấy mẫu lại 20.000 lần [5]. Kiểm định Wilcoxon hạng có dấu hai phía dùng db, loại các hiệu có trị tuyệt đối không quá 10⁻¹² [6]; xét phân bố chênh lệch đối xứng quanh không, không trực tiếp kiểm định trung bình Δ. Giá trị p của thời gian, RMSE và Eκ,exec được hiệu chỉnh Holm [7], nhưng khoảng tin cậy chỉ ở mức từng chỉ số. Các nhóm chia sẻ tuyến, bộ lập kế hoạch và bản đồ; gom nhóm chưa loại hết phụ thuộc. Suy luận giới hạn trong thiết kế đầu vào cố định.",
        "Phân biệt ước lượng mean/CI với Wilcoxon, xử lý zero và hiệu chỉnh p với CI. Giữ toàn bộ số thống kê.",
        ["tools/analyze_journal_repeatability.py:bootstrap_mean_ci,wilcoxon_pvalue,holm_adjust", "https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.wilcoxon.html"])
    add("H03", "III-H.2", "Hình 17 cho thấy RMSE tăng",
        "Hình 17 cho thấy RMSE tăng ở một số đầu vào dù chênh lệch chung âm. Trung bình đều theo công thức 9 tránh để nhóm nhiều cặp cùng đạt hơn chi phối kết quả, nhưng không hiệu chỉnh thiên lệch do loại cặp không cùng đạt. Ý nghĩa vận hành còn phụ thuộc yêu cầu thời gian, sai số và khoảng hở; p nhỏ không thay thế các tiêu chí đó. Bảng XXII trình bày các đánh đổi.",
        "Tách equal block weighting khỏi xử lý selection bias và phân biệt ý nghĩa thống kê với ý nghĩa thực tiễn.",
        [COHORT + "/analysis/aggregate.json", "tools/analyze_journal_repeatability.py"])
    add("I01", "III-I.3", "Khảo sát dùng một bản đồ tĩnh,",
        "Khảo sát dùng một bản đồ tĩnh, một robot mô phỏng và một cấu hình RPP; chưa xét vật cản động, nhiều mức khởi tạo ngẫu nhiên hay nhiễu. R01 khác đợt thu R02–R05; bộ lặp chỉ gồm PSTMO và Constrained. Chưa xác nhận ngẫu nhiên hóa thứ tự chạy; bốn tiến trình dùng chung tài nguyên máy dù tách miền truyền thông. Điều này hạn chế suy rộng và diễn giải thời lượng xử lý.",
        "Gom rõ giới hạn vật cản động/multi-seed vào phần giới hạn, giữ khác đợt thu và shared resources.",
        [COHORT + "/README.md", "docs/warehouse_cross_aisles_5_routes/README.md"])
    add("I02", "III-I.3", "PSTMO cho thấy lợi ích về hình học",
        "Lợi ích hình học và thực thi của PSTMO đi kèm đánh đổi về khoảng hở, sai số cuối và thời lượng xử lý. Chưa kiểm chứng trên robot thực hoặc loại bỏ từng thành phần trên cùng phiên bản để tách đóng góp của tiền xử lý, tìm hình dạng và quy hoạch động. Kiểm tra hình bao phụ thuộc bản đồ và lấy mẫu. Dữ liệu chưa hỗ trợ kết luận về điện năng, rung tải hàng hay ưu thế tỷ lệ thành công.",
        "Nêu rõ thiếu ablation cùng phiên bản và robot thực, giữ đánh đổi và phạm vi kết luận về reliability/energy.",
        [COHORT + "/README.md", "docs/agent/DATA_POLICY.md"])
    return edits


def report(edits, validation):
    lines = ["# Hồ sơ hiệu đính khoa học journal ver9", "",
        "Ngày 22/10/2026. Bản gốc: ver8; phạm vi phần II–III. Không chạy ROS/Gazebo/benchmark hoặc tính lại kiểm định.", "",
        "## Kết quả audit", "",
        "Lỗi triển khai rõ nhất là định nghĩa cmax chỉ lấy chi phí tâm. Các sửa khác làm rõ conditioning, gia tốc góc, hai trim, chính quy/G², phạm vi tối ưu và suy luận thống kê. Không phát hiện lý do để đổi chín công thức, bảng số liệu hoặc hình.", "",
        "Phân loại: implementation_correction = sửa mô tả sai/thiếu bước triển khai; ambiguity_correction = sửa câu dễ hiểu sai; clarification = bổ sung điều kiện/giới hạn. Không có đợt viết lại văn phong thuần túy.", "",
        "Đã đọc toàn bộ manuscript và xem đủ 15 trang render ver8. Giữ nguyên R01–R05: đã có quan sát, so sánh theo planner và đánh đổi; giữ cả RMSE bất lợi ở R02, thất bại R03/R04, độ giật bất lợi R01/R03 và sai số cuối. Giữ III-G, mô tả thất bại III-I.1–2, tiêu chí thành công, các bảng/hình và hệ trích dẫn.", "",
        "## Đối chiếu nguồn", "",
        f"Đã kiểm tra {len(validation['historical_hashes_checked'])} tệp nguồn duy nhất trong provenance ver6–8 và {len(validation['cohort_source_matches'])} tệp C++/config/mô hình đối với commit cohort `{validation['cohort_commit']}`. Kết quả chi tiết: [source_validation.json](source_validation.json).", "",
        "Hash khớp xác nhận nguồn của cohort lặp; không suy rộng thành hồ sơ đầy đủ của mọi binary phụ thuộc hay mọi lượt lịch sử R01. Các phiên bản và dữ liệu cũ không được dựng lại.", "",
        "## Chỉnh sửa có bằng chứng", ""]
    for e in edits:
        lines += [f"### {e['id']} — {e['section']} — {e['kind']}", "",
                  f"Vị trí ver8: khối {e['source_block']} (đánh số từ 0); `{e['body_xpath']}` trong word/document.xml.", "",
                  "Trước: " + e["before"], "", "Sau: " + e["after"], "",
                  "Lý do: " + e["reason"], "", "Nguồn:", ""]
        lines += ["- `" + s + "`" for s in e["sources"]] + [""]
    lines += ["## Giới hạn chưa giải quyết", "",
        "- Chưa có hồ sơ tối ưu trọng số hoặc ablation cùng phiên bản; ver9 chỉ báo đúng cấu hình, không đặt lý do thiết kế giả định.",
        "- Không chứng minh an toàn liên tục, tối ưu quỹ đạo liên tục hay bảo đảm thực thi từ kiểm tra rời rạc.",
        "- Bộ lặp có phụ thuộc giữa các nhóm dùng chung tuyến/planner/bản đồ; chưa có phân tích mô hình phụ thuộc khác, kiểm tra tương đương hoặc chính sách phục hồi được đo. Giữ nguyên CI/p lịch sử và giảm phạm vi diễn giải.",
        "- Mã phân tích dùng scipy.stats.wilcoxon(method='auto'); manifest không khóa đầy đủ phiên bản thư viện thống kê. Đợt này đối chiếu và bảo toàn kết quả đã lưu, không tuyên bố tái tính đúng bit.",
        "- Tên Gazebo Harmonic được yêu cầu trong brief, nhưng manifest cohort không ghi bản phát hành Gazebo. Giữ cách gọi Gazebo của ver8; không suy ra phiên bản thực nghiệm chỉ từ máy hiện tại.",
        "- Nguồn RPP Jazzy và tài liệu SciPy trực tuyến được đọc ngày 22/10/2026 để kiểm tra ý nghĩa tham số/kiểm định; đây là nguồn bổ trợ, không xác nhận hash binary lịch sử. Không thêm trích dẫn mới vào bài.",
        "- Hai phần vẫn cần ghép toàn bài và kiểm tra mẫu tạp chí đích trước khi nộp.", "",
        "## Kiểm chứng đầu ra", "",
        "Các giá trị thực nghiệm, 18 hình, 23 bảng, 12 đối tượng Office Math và chín số công thức được giữ nguyên. Script kiểm tra đối chiếu cấu trúc gói, nội dung, lineage và số liệu nguồn; trạng thái thực tế nằm trong [quality_checks.json](quality_checks.json). Xác nhận trực quan phải gắn với hash DOCX/PDF/PNG của bản cuối qua [visual_review.json](visual_review.json).", ""]
    return "\n".join(lines)


def main():
    validation = validate_sources()
    with ZipFile(REFERENCE) as z:
        infos = z.infolist()
        parts = {n: z.read(n) for n in z.namelist()}
    root = etree.fromstring(parts["word/document.xml"])
    source = (OLD / "manuscript.md").read_text()
    records, tail = map_chunks(root, source)
    edits = edit_plan(records)
    for e in edits:
        record = records[e["source_block"]]
        record["text"] = e["after"]
        scientific_runs(record["nodes"][0], e["after"])
    # No order or section changes; all edits stay in their original body slots.
    new_source = "\n\n".join(r["text"] for r in records) + "\n"
    map_chunks(root, new_source)
    parts["word/document.xml"] = etree.tostring(root, xml_declaration=True, encoding="UTF-8", standalone=True)
    core = etree.fromstring(parts["docProps/core.xml"])
    core.find("{http://purl.org/dc/elements/1.1/}subject").text = "Phần II và III của bản thảo journal phiên bản 9"
    parts["docProps/core.xml"] = etree.tostring(core, xml_declaration=True, encoding="UTF-8", standalone=True)
    OUT.mkdir(exist_ok=True)
    docx = OUT / "PSTMO_JOURNAL_VER9_PHAN_II_III.docx"
    with ZipFile(docx, "w") as z:
        for info in infos:
            z.writestr(info, parts[info.filename])
    (OUT / "manuscript.md").write_text(new_source, encoding="utf-8")
    (OUT / "table_values.json").write_bytes((OLD / "table_values.json").read_bytes())
    dump(OUT / "revision_log.json", edits)
    dump(OUT / "source_validation.json", validation)
    (OUT / "REVISION_REPORT.md").write_text(report(edits, validation), encoding="utf-8")
    inputs = list(OLD.iterdir()) + [Path(__file__), ROOT / "tools/build_journal_sections_v7.py"]
    inputs += [ROOT / p for p in SOURCES] + [
        ROOT / COHORT / "study_manifest.json", ROOT / COHORT / "analysis/aggregate.json",
        ROOT / COHORT / "analysis/block_differences.csv", ROOT / COHORT / "analysis/trial_metrics.csv",
        ROOT / "docs/warehouse_cross_aisles_5_routes/all_125_trials.csv",
        ROOT / "tools/analyze_journal_repeatability.py"]
    outputs = ["manuscript.md", "table_values.json", "revision_log.json", "source_validation.json", "REVISION_REPORT.md"]
    dump(OUT / "provenance.json", {
        "docx": docx.name, "docx_sha256": sha(docx), "reference": str(REFERENCE.relative_to(ROOT)),
        "reference_sha256": REFERENCE_SHA, "date": DATE,
        "source_sha256": {str(p.relative_to(ROOT)): sha(p) for p in inputs if p.is_file()},
        "output_sha256": {p: sha(OUT / p) for p in outputs},
        "package_parts": {n: hashlib.sha256(data).hexdigest() for n, data in parts.items()},
        "changed_parts_allowed": ["word/document.xml", "docProps/core.xml"],
        "counts": {"figures": 18, "tables": 23, "table_cells": 696, "math_objects": 12,
                   "equations": 9, "main_trials": 125, "repeat_trials": 250},
        "changes_count": len(edits), "simulation_runs_added": 0, "statistics_recomputed": False,
        "word_count_before": len(source.split()), "word_count_after": len(new_source.split())})
    # A rebuild always invalidates the prior review until the verifier checks exact hashes.
    dump(OUT / "quality_checks.json", {"status": "awaiting_verification", "docx_sha256": sha(docx),
                                     "visual_review_all_pages": False})
    print(json.dumps({"output": str(docx), "changes": len(edits),
                      "word_delta": len(new_source.split()) - len(source.split())}, ensure_ascii=False))


if __name__ == "__main__":
    main()
