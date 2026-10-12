#!/usr/bin/env python3
"""Build the publication-oriented PSTMO journal outline from audited evidence."""

from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Pt


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'Viet_bao/journal/dan_y/ver1/DAN_Y_JOURNAL_PSTMO_Q2_CHUYEN_SAU.docx'
OUTPUT_DIR = ROOT / 'Viet_bao/journal/dan_y/ver2'
OUTPUT = OUTPUT_DIR / 'DAN_Y_JOURNAL_PSTMO_Q2_VER2.docx'
STATS = ROOT / 'results/pstmo_journal_repeatability_20261011/analysis/aggregate.json'


def clear_body(document):
    body = document._element.body
    for child in list(body):
        if child.tag != qn('w:sectPr'):
            body.remove(child)


def update_header_date(document, old='09.10.2026', new='12.10.2026'):
    """Refresh the inherited template date without discarding header styling."""
    for section in document.sections:
        for paragraph in section.header.paragraphs:
            for run in paragraph.runs:
                if old in run.text:
                    run.text = run.text.replace(old, new)


def set_repeat_table_header(row):
    properties = row._tr.get_or_add_trPr()
    repeat = OxmlElement('w:tblHeader')
    repeat.set(qn('w:val'), 'true')
    properties.append(repeat)


def keep_row(row):
    properties = row._tr.get_or_add_trPr()
    properties.append(OxmlElement('w:cantSplit'))


def add_paragraph(document, text='', style=None, bold_prefix=None):
    paragraph = document.add_paragraph(style=style)
    if bold_prefix and text.startswith(bold_prefix):
        paragraph.add_run(bold_prefix).bold = True
        paragraph.add_run(text[len(bold_prefix):])
    else:
        paragraph.add_run(text)
    return paragraph


def add_bullets(document, items):
    for item in items:
        add_paragraph(document, '• ' + item, 'Compact')


def add_table(document, headers, rows, widths=None):
    table = document.add_table(rows=1, cols=len(headers))
    table.style = 'Table Grid'
    set_repeat_table_header(table.rows[0])
    for cell, value in zip(table.rows[0].cells, headers):
        cell.text = str(value)
        for run in cell.paragraphs[0].runs:
            run.bold = True
    for values in rows:
        row = table.add_row()
        keep_row(row)
        for cell, value in zip(row.cells, values):
            cell.text = str(value)
    if widths:
        for row in table.rows:
            for cell, width in zip(row.cells, widths):
                cell.width = width
    return table


def page_break(document):
    document.add_page_break()


def metric_row(statistics_document, key):
    result = statistics_document['metrics'][key]
    lower, upper = result['mean_block_difference_bootstrap_95_ci']
    adjusted = result.get('holm_adjusted_p_primary_family')
    return [
        result['label'],
        result['unit'],
        f"{result['constrained_mean']:.5g}",
        f"{result['pstmo_mean']:.5g}",
        f"{result['mean_block_difference_pstmo_minus_constrained']:.5g}",
        f"[{lower:.5g}; {upper:.5g}]",
        f"{adjusted:.3g}" if adjusted is not None else 'exploratory',
    ]


def add_subsection(document, title, purpose, items, output=None, evidence=None):
    document.add_heading(title, level=2)
    add_paragraph(document, 'Mục đích: ' + purpose, 'Compact', 'Mục đích: ')
    add_bullets(document, items)
    if evidence:
        add_paragraph(document, 'Bằng chứng: ' + evidence, 'Note', 'Bằng chứng: ')
    if output:
        add_paragraph(document, 'Đầu ra khi viết: ' + output, 'Note', 'Đầu ra khi viết: ')


def build():
    if not SOURCE.exists():
        raise FileNotFoundError(SOURCE)
    if not STATS.exists():
        raise RuntimeError(
            'Repeatability analysis is missing; run '
            'tools/analyze_journal_repeatability.py first'
        )
    stats = json.loads(STATS.read_text(encoding='utf-8'))
    if stats.get('audit_issues'):
        raise RuntimeError('Cannot build outline from an unaudited cohort')
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    shutil.copy2(SOURCE, OUTPUT)
    document = Document(OUTPUT)
    clear_body(document)
    update_header_date(document)
    document.core_properties.title = 'Dàn ý journal PSTMO Q2 — phiên bản 2'
    document.core_properties.subject = (
        'Publication-oriented outline with repeated validation and statistical design'
    )
    document.core_properties.author = 'PSTMO research workspace'
    document.core_properties.comments = (
        'Generated from repository evidence; not a guarantee of journal acceptance.'
    )

    add_paragraph(document, 'BẢN THIẾT KẾ BÀI BÁO JOURNAL', 'Note')
    title = add_paragraph(document, 'DÀN Ý JOURNAL PSTMO — PHIÊN BẢN 2', 'Title')
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    lead = add_paragraph(
        document,
        'Pivot–G² Path Smoothing with Swept-Footprint Feasibility and '
        'Path-Level Dynamic Programming for Differential-Drive Warehouse Robots',
        'Lead')
    lead.alignment = WD_ALIGN_PARAGRAPH.CENTER
    add_paragraph(
        document,
        'Dàn ý để viết manuscript 18–22 trang nội dung chính. Cấu trúc ưu tiên '
        'câu hỏi nghiên cứu, phương pháp tái lập, kiểm định theo khối và giới hạn '
        'kết luận; không dùng số lượng tài liệu tham khảo để thay thế lập luận.',
        'Lead')
    add_paragraph(
        document,
        'Cập nhật dữ liệu: 125 lượt ma trận chính + 250 lượt lặp '
        'PSTMO–Constrained. Ngày dựng: '
        f"{datetime.now(timezone.utc).date().isoformat()} UTC.",
        'Note')

    document.add_heading('A. QUYẾT ĐỊNH BIÊN TẬP TRƯỚC KHI VIẾT', level=1)
    add_subsection(
        document,
        'A.1. Luận điểm trung tâm có thể bảo vệ',
        'Chốt một đóng góp hẹp, kiểm chứng được và không trùng với novelty đã có của Bézier/G².',
        [
            'PSTMO là post-processing plugin cho Nav2 Smoother Server, không phải global planner hoặc controller mới.',
            'Novelty nên đặt ở tổ hợp cơ chế: conditioning an toàn → Pivot/G² candidates → hierarchical two-trim α-search → common-window feasibility → path-level DP → invariant cuối.',
            'Câu hỏi thực nghiệm chính là liệu tổ hợp đó có giảm mức uốn và cải thiện thực thi so với các smoother công khai khi Raw path, controller, robot và world được giữ cố định.',
            'Câu hỏi thống kê trọng tâm là PSTMO so với Constrained trên 25 khối route–planner, năm lần lặp mỗi ô; không gọi 125 lượt là 125 đơn vị độc lập.',
            'Khoảng hở, tỷ lệ hoàn tất và RMSE là các ràng buộc/đánh đổi song song, không được che bởi một chỉ số Eκ tốt.',
        ],
        evidence='adaptive_pivot_g2_smoother.cpp; hierarchical_shape_search.cpp; path_optimization.cpp; footprint_safety.cpp; aggregate.json.',
        output='Một câu thesis statement 35–50 từ, dùng nhất quán ở Abstract, cuối Introduction và đầu Discussion.')
    add_subsection(
        document,
        'A.2. Mức sẵn sàng và giới hạn không được che giấu',
        'Phân biệt bằng chứng đã có với work package còn thiếu trước khi nộp.',
        [
            'Đã có: ma trận năm smoother; 5 route × 5 planner; trace vòng kín; hash Raw; diagnostics; cohort lặp PSTMO–Constrained; CI bootstrap theo khối; Wilcoxon + Holm.',
            'Đã có ở mức mô tả: snapshot đa môi trường của cấu hình hiện hành và các snapshot ablation trong quá trình phát triển.',
            'Chưa có ablation nhân quả trên cùng commit cho DP-on/off, swept-footprint-on/off và two-trim/one-trim. Không dùng snapshot khác cấu hình để tuyên bố tác động riêng của từng module.',
            'Chưa có random-seed/noise injection, dynamic obstacles, payload sweep, energy/Wh hoặc real robot. Các khoảng trống này phải xuất hiện trong Threats to Validity và Future Work.',
            'Không thể bảo đảm bài được nhận. Mục tiêu thực tế là loại các lý do desk-reject/reviewer-reject có thể phòng tránh: novelty mơ hồ, baseline yếu, thống kê sai, thiếu provenance và overclaim.',
        ],
        output='Một bảng “claim–evidence–scope” dùng nội bộ; chỉ đưa phiên bản rút gọn vào Discussion nếu tạp chí cho phép.')

    success = stats['success']
    page_break(document)
    add_table(document,
        ['Hạng mục', 'Trạng thái dùng trong manuscript'],
        [
            ['Ma trận chính', '125 lượt; 5 route × 5 planner × 5 phương án; 123 lượt đạt.'],
            ['Cohort lặp', f"250 lượt; Constrained {success['constrained']}/125 đạt; PSTMO {success['pstmo']}/125 đạt; {len(stats['failures'])} lượt không đạt được giữ."],
            ['Ghép cặp', f"{stats['valid_raw_pair_count']}/{stats['paired_trial_count']} cặp có cùng Raw hash."],
            ['Đơn vị thống kê', '25 khối route–planner; lấy trung bình 5 repetitions trong khối.'],
            ['Chưa được gọi là', 'multi-seed, robustness test, real-world validation, energy optimization.'],
        ])

    document.add_heading('B. CẤU TRÚC MANUSCRIPT ĐỀ XUẤT', level=1)
    add_paragraph(
        document,
        'Cấu trúc mới tách Related Work, Experimental Methodology, Results và '
        'Discussion. Cách này phù hợp journal hơn cấu trúc bốn phần kiểu báo cáo '
        'khóa luận, đồng thời cho reviewer thấy rõ đâu là method, đâu là evidence.',
        'Compact')
    add_table(document,
        ['Phần', 'Trang mục tiêu', 'Vai trò'],
        [
            ['Title / Abstract / Keywords', '0.6–0.8', 'Nêu problem–method–design–effect–scope bằng số đã audit.'],
            ['1. Introduction', '2.0–2.5', 'Bối cảnh, gap tổ hợp, RQ/H, contribution; không chứa survey dài.'],
            ['2. Related Work', '2.0–2.5', 'So sánh theo trục kỹ thuật và định vị novelty.'],
            ['3. PSTMO Method', '4.5–5.5', 'Formulation, hình học, feasibility, DP, complexity, Nav2 integration.'],
            ['4. Experimental Methodology', '2.5–3.0', 'Robot/world, baselines, protocol, metrics, statistics, provenance.'],
            ['5. Results', '4.0–5.0', 'RQ-oriented; primary, safety/trade-off, repeatability, diagnostics.'],
            ['6. Discussion', '1.2–1.8', 'Mechanism, applicability, threats, external validity, missing ablation.'],
            ['7. Conclusion', '0.4–0.6', 'Trả lời RQ, không thêm claim mới.'],
            ['References', '3–5', 'Chỉ nguồn được dẫn; kiểm venue/DOI/version final.'],
        ])
    add_paragraph(
        document,
        'Nếu journal tính References vào giới hạn 20 trang, giảm số route figure '
        'từ năm xuống hai case đại diện + một aggregate figure; chuyển bảng per-route '
        'và trace chi tiết sang supplementary material.',
        'Note')

    document.add_heading('TITLE, ABSTRACT VÀ CONTRIBUTIONS', level=1)
    add_subsection(
        document,
        'Title',
        'Nêu đúng tầng đóng góp và đối tượng robot; tránh “optimal” hoặc “safe” không điều kiện.',
        [
            'Ưu tiên: “Pivot–G² Path Smoothing with Swept-Footprint Feasibility and Path-Level Dynamic Programming for Differential-Drive Warehouse Robots”.',
            'Phương án nhấn mạnh Nav2: “A Nav2-Integrated Pivot–G² Path Smoother for Differential-Drive Robots in Cross-Aisle Warehouses”.',
            'Không dùng “globally optimal”, “collision-free” hoặc “real-time optimal” trong title.',
        ])
    add_subsection(
        document,
        'Abstract (200–250 từ, 6 câu chức năng)',
        'Đưa đủ problem, gap, method, design, quantitative result và scope.',
        [
            'Câu 1 — problem: global paths trong kho có thể chứa góc gãy/dao động không phù hợp cho robot vi sai.',
            'Câu 2 — gap: cần phối hợp lựa chọn Pivot/G², swept footprint và tương thích nhiều góc trong Smoother Server.',
            'Câu 3 — method: quintic G², hierarchical two-trim α-search, common-window time feasibility và DP.',
            'Câu 4 — design: ma trận 125 lượt cho năm smoother và cohort 250 lượt PSTMO–Constrained; cùng RPP/controller và Raw hash.',
            'Câu 5 — result: dùng đúng effect size, CI và p Holm lấy từ Bảng 5.2; thêm tỷ lệ hoàn tất.',
            'Câu 6 — scope: kết luận cho mô phỏng kho tĩnh; fixed-condition repeatability, chưa phải multi-seed/robot thật.',
        ],
        output='Một abstract không citation, không acronym thừa, không liệt kê tất cả metric.')
    add_subsection(
        document,
        'Contributions (đánh số C1–C4)',
        'Mỗi contribution phải nối được tới một phương trình/algorithm và một bằng chứng.',
        [
            'C1 — A corner-aware Pivot–G² representation preserving endpoints and testing the complete swept footprint.',
            'C2 — A bounded hierarchical search over at most two geometry-derived trim distances and coarse/recovery/refinement α grids.',
            'C3 — A path-level DP selecting compatible corner states under shared-segment budgets, followed by stitched-path invariants.',
            'C4 — A reproducible Nav2/Gazebo evaluation: five planners, four public baselines plus Raw, traceable failures, and focused repeated comparison against Constrained.',
        ],
        evidence='Mỗi contribution trỏ tới source file, figure method, test và result tương ứng.')

    page_break(document)
    document.add_heading('1. INTRODUCTION (2.0–2.5 TRANG)', level=1)
    add_subsection(document, '1.1. Operational context and problem',
        'Đi từ logistics xuống đúng vấn đề path post-processing của một robot.', [
            'Một đoạn AGV/AMR trong kho; tránh số liệu thị trường không có nguồn gốc.',
            'Một đoạn pipeline planner–smoother–controller; giải thích vì sao collision-free polyline chưa đủ cho diff-drive.',
            'Một đoạn failure mode tại cross-aisles: góc gấp, pivot, shared short segments, footprint sweep.',
        ], output='3 đoạn, 8–12 citations; không giảng Dijkstra/A* chi tiết.')
    add_subsection(document, '1.2. Gap and research questions',
        'Đặt gap tổ hợp thay vì tuyên bố G²/Bézier mới.', [
            'RQ1: PSTMO thay đổi hình học đường thế nào so với Raw và các smoother Nav2?',
            'RQ2: PSTMO ảnh hưởng tỷ lệ hoàn tất, thời gian, tracking và dynamics thế nào trong vòng kín?',
            'RQ3: Hiệu ứng PSTMO–Constrained có ổn định qua route/planner blocks và repetitions cố định không?',
            'RQ4: Những trade-off nào xuất hiện ở clearance, computation và planner dependency?',
        ], output='Một đoạn gap 8–10 câu + danh sách RQ ngắn.')
    add_subsection(document, '1.3. Hypotheses and contributions',
        'Chuyển RQ thành kỳ vọng có hướng nhưng không cherry-pick.', [
            'H1: PSTMO giảm planned/executed curvature energy so với Constrained.',
            'H2 (outcome nhị phân): so sánh discordance thành công theo cặp bằng McNemar; không diễn giải p không có ý nghĩa thành tương đương hay non-inferiority nếu chưa định trước margin/power.',
            'H3: hiệu ứng time/RMSE thay đổi theo route/planner; báo cáo distribution và block-level result.',
            'Kết thúc bằng C1–C4 và sơ đồ paper organization.',
        ])

    document.add_heading('2. RELATED WORK (2.0–2.5 TRANG)', level=1)
    add_subsection(document, '2.1. Global planning and Nav2 smoothing',
        'Định vị đầu vào và baseline mà không biến phần này thành textbook.', [
            'Nhóm NavFn/A*, Theta*, Smac2D/Hybrid-A*: nhiệm vụ tạo Raw path.',
            'Simple, Savitzky–Golay, Constrained Smoother: đúng vai trò và tham số công khai.',
            'RPP là controller cố định; trích bản final Autonomous Robots, không giữ arXiv như venue chính.',
        ])
    add_subsection(document, '2.2. Curvature-continuous and Bézier smoothing',
        'Thừa nhận prior art gần nhất rồi chỉ ra khác biệt representation/selection.', [
            'Piazzi et al.; Pan et al.; Brezak & Petrović; quintic Bézier warehouse work; recent continuous-curvature methods.',
            'So sánh continuity order, primitive, collision model, time feasibility, multi-corner coupling và evaluation platform.',
            'Không nói “first quintic/G²”; novelty là cơ chế tích hợp và decision process.',
        ])
    add_subsection(document, '2.3. Trajectory optimization and local methods',
        'Phân biệt path smoother với timed local trajectory optimizer.', [
            'TEB, CHOMP, STOMP, GRIPS, universal differential-drive trajectory optimization.',
            'Nêu khác nhau về search space, timing, obstacle dynamics và vị trí trong navigation stack.',
        ])
    add_subsection(document, '2.4. Synthesis table and defensible gap',
        'Kết thúc Related Work bằng bảng định vị trực tiếp.', [
            'Cột bắt buộc: continuity; pivot support; swept footprint; time feasibility; multi-corner coupling; Nav2 integration; closed-loop repeats.',
            'Andrei 2025 và Serdel 2024 phải ghi đúng là conference, không gắn nhãn journal.',
            'Kiểm lại version final/DOI của FDSPC RA-L 2025, T-ASE 2025 và các bài 2025–2026 trước khi trích claim.',
        ], output='Bảng 8–12 công trình thật sự gần; phần còn lại dẫn trong prose.')

    page_break(document)
    document.add_heading('3. PSTMO METHOD (4.5–5.5 TRANG)', level=1)
    add_subsection(document, '3.1. Problem formulation and notation',
        'Định nghĩa input/output, robot, costmap, footprint và constraints.', [
            'P_raw={p0,…,pn}; map M; footprint F; limits ξ; output path giữ start/goal.',
            'ẋ=v cosθ, ẏ=v sinθ, θ̇=ω; vL=v−bω/2, vR=v+bω/2; ω=vκ trên translational segments.',
            'Nêu v, ω, wheel, lateral/linear/angular acceleration limits; tách path geometry khỏi controller command.',
        ], output='Bảng ký hiệu 10–14 dòng + 4 phương trình.')
    add_subsection(document, '3.2. Path conditioning and corner extraction',
        'Mô tả đúng condition_only của cấu hình hiện hành.', [
            'Remove duplicates; deviation-bounded shortcuts with footprint-safe predicate; A–B–A oscillation handling.',
            'Δθ bằng atan2(cross,dot); threshold và pass-through semantics.',
            'Greedy LOS không thuộc pipeline mặc định; chỉ được nói trong historical ablation.',
        ])
    add_subsection(document, '3.3. Quintic G² transition geometry',
        'Cho phép reviewer tái lập curve construction.', [
            'A=C−du, B=C+dw, q=αd; liệt kê P0…P5 đúng source.',
            'B(t), first/second derivatives, κ(t), endpoint tangent and zero-curvature argument.',
            'Eκ=∫κ²ds; đơn vị 1/m; không gọi là battery energy.',
            'Rejection: degenerate derivative, unintended curvature sign, wheel reversal, invalid speed cap.',
        ], output='Hình control points duy nhất + derivation ngắn trong appendix nếu dài.')
    add_subsection(document, '3.4. Hierarchical two-trim α-search',
        'Làm rõ bounded deterministic search và hai tầng objective.', [
            'd_preferred và d_compatible từ shared-segment budgets/margin.',
            'Coarse α grid, recovery midpoints, refinement near winner; complexity bounded by number of corners and samples.',
            'Shape search minimizes curvature energy only after feasibility; stable_candidate_cost/common-window time dùng ở bước retention/selection.',
        ], output='Algorithm 1 pseudocode + complexity O(NKshape).')
    add_subsection(document, '3.5. Swept-footprint and time feasibility',
        'Phân biệt hard collision, inflation cost và motion feasibility.', [
            'Filled footprint at pose, sampled translation sweep, in-place rotation sweep, terminal rotation.',
            'Common-window parameterization; linear, angular, wheel, lateral and acceleration constraints.',
            'Tuyên bố an toàn phải gắn với sampled costmap checks và configured footprint; không mở rộng thành formal safety guarantee.',
        ])
    add_subsection(document, '3.6. Path-level dynamic programming',
        'Chứng minh lý do local best không đủ khi corner windows chia sẻ segment.', [
            'State s_i=(pivot/transition, d_i, J_i, payload); compatibility d_i+d_(i+1)+m_i≤L_i.',
            'Recurrence, predecessor reconstruction, tie-break: cost → fewer pivots → stable index.',
            'Complexity O(NK²), memory and deterministic behavior.',
        ], output='Algorithm 2 + state-lattice figure; không gọi global optimum ngoài finite candidate set.')
    add_subsection(document, '3.7. Stitching, invariants and Nav2 integration',
        'Khóa contract giữa method và implementation.', [
            'Build line/transition/pivot sequence; preserve endpoints/goal orientation.',
            'Final checks: endpoint, stitched timing, swept path and terminal rotation.',
            'Plugin lifecycle, diagnostics topic, failure semantics; sơ đồ planner → smoother → RPP.',
        ])

    page_break(document)
    document.add_heading('4. EXPERIMENTAL METHODOLOGY (2.5–3.0 TRANG)', level=1)
    add_subsection(document, '4.1. Platform, robot and environment',
        'Cho phép tái lập và nhận diện simulation–hardware gap.', [
            'ROS 2 Jazzy, Nav2, Gazebo; OS/CPU; commit and hashes in supplementary manifest.',
            'World cross-aisles, map resolution, footprint 0.44×0.34 m, wheel separation và giới hạn PSTMO.',
            'Nêu mismatch CAD/Gazebo nếu dùng kích thước robot; không gọi Gazebo model là physical robot.',
        ])
    add_subsection(document, '4.2. Factors, baselines and controlled variables',
        'Giải thích đúng factorial structure.', [
            'Routes R01–R05; planners NavFnAStar, NavFnDijkstra, ThetaStar, Smac2D, SmacHybrid.',
            'Methods Raw, Simple, Savitzky–Golay, Constrained, PSTMO; RPP/controller và world cố định.',
            'Main matrix 125 trials; repeated cohort 250 trials only for Constrained/PSTMO.',
            'Mỗi trial isolated ROS_DOMAIN_ID and GZ_PARTITION; infrastructure retry separated from algorithm failure.',
        ])
    add_subsection(document, '4.3. Outcomes and success criteria',
        'Đăng ký primary/secondary outcomes trước khi kể result.', [
            'Primary: execution time, tracking RMSE, executed curvature energy; success analyzed separately.',
            'Secondary: planned Eκ, jerk p95, final error, distance, T smooth, minimum clearance.',
            'Success requires action success, ground-truth goal tolerance and physical settling; retain failures in denominator.',
            'Collision Monitor interventions and planned footprint collision samples reported as audit outcomes.',
        ])
    add_subsection(document, '4.4. Pairing, statistics and multiplicity',
        'Tránh pseudoreplication và significance fishing.', [
            'Pair methods by scenario, planner, repetition and Raw path SHA-256.',
            'Average five repetitions within each route–planner block; n=25 blocks for inference.',
            'Report block mean difference PSTMO−Constrained, 95% cluster bootstrap CI (20,000 resamples), two-sided Wilcoxon.',
            'Holm correction for three primary continuous outcomes; McNemar exact for paired success discordance.',
            'Secondary outcomes exploratory; report effect/CI, do not convert every p-value into a claim.',
            'Repetitions have no exposed random seed: describe as fixed-input repeatability, not stochastic robustness.',
        ])
    add_subsection(document, '4.5. Reproducibility and data policy',
        'Nối manuscript với artifact có thể audit.', [
            'Publish scenario YAML, params hash, commit, per-trial JSON/log, aggregate CSV/JSON and analysis script.',
            'Preserve failed trials and infrastructure attempts; never overwrite historical cohort.',
            'Supplementary README documents exact command and interpretation limits.',
        ])

    page_break(document)
    document.add_heading('Bảng 4.1. Thống kê cohort lặp phải đưa vào manuscript', level=2)
    add_table(document,
        ['Metric', 'Unit', 'Constrained', 'PSTMO', 'Δ block mean', '95% CI', 'Holm p'],
        [metric_row(stats, key) for key in (
            'execution_time_s', 'tracking_rmse_m',
            'executed_curvature_energy_1pm')])
    add_paragraph(
        document,
        'Δ = PSTMO − Constrained. Khi viết, không chọn số chữ số vượt quá độ '
        'chính xác đo; giải thích dấu của Δ trong caption.',
        'Note')

    document.add_heading('5. RESULTS (4.0–5.0 TRANG)', level=1)
    add_subsection(document, '5.1. Data integrity and completion',
        'Mở Results bằng audit, không mở bằng case đẹp nhất.', [
            'Sơ đồ dataset: 125 main + 250 repeated; số success/failure theo method.',
            'Raw hash pairing, diagnostics coverage, invariant count, retries and retained failures.',
            'McNemar success result; nếu không significant, viết đúng là chưa có bằng chứng khác biệt—not equivalence.',
        ], output='Table 1 protocol/audit; không dùng figure nếu bảng đủ rõ.')
    add_subsection(document, '5.2. RQ1 — Geometric path quality',
        'So sánh đường trước khi trộn với controller behavior.', [
            'Main matrix: path length, κmax, Eκ, clearance, collision samples across five methods.',
            'Paired comparison by route/planner; distribution, not only grand mean.',
            'Nêu trade-off: Eκ thấp có thể đi kèm clearance thấp hoặc local κmax cao.',
        ], output='Một aggregate multi-panel + một zoom case đại diện; per-route tables sang supplement.')
    add_subsection(document, '5.3. RQ2 — Closed-loop execution',
        'Trình bày success, time, tracking, final error and dynamics.', [
            'Main matrix gives breadth across five methods; repeated cohort gives strength for PSTMO–Constrained.',
            'Tách conditional metrics on successful trials khỏi success analysis.',
            'Giữ hai lượt main-matrix không đạt; không biến thời gian của failure thành win/loss.',
        ])
    add_subsection(document, '5.4. RQ3 — Repeatability and statistical evidence',
        'Dùng 25 block effects làm hình chính.', [
            'Hình box/points PSTMO−Constrained cho time, RMSE, executed Eκ.',
            'Trong text: effect size, CI, adjusted p; không dùng chỉ p-value.',
            'Phân tích heterogeneity theo route/planner mang tính mô tả; không chạy hàng chục subgroup tests.',
        ])
    add_subsection(document, '5.5. RQ4 — Safety/computation trade-offs and diagnostics',
        'Đối chiếu các lợi ích với cost và giới hạn.', [
            'Minimum clearance, collision audit, jerk, T smooth, DP states/compatible edges.',
            'T smooth từ four-worker run chịu load; không dùng nó như hard real-time certificate.',
            'Diagnostics xác nhận pipeline mode, không chứng minh hiệu quả nhân quả của từng component.',
        ])

    document.add_heading('6. DISCUSSION (1.2–1.8 TRANG)', level=1)
    add_subsection(document, '6.1. Mechanistic interpretation',
        'Giải thích tại sao hiệu ứng xuất hiện, gắn với source và figures.', [
            'Two-trim/α-search giảm candidate burden nhưng vẫn có compatible alternative.',
            'DP hữu ích khi nhiều góc cạnh nhau; pivot remains fallback under feasibility/safety constraints.',
            'Controller có thể làm time/RMSE không đi cùng planned Eκ; tránh causal claim từ correlation.',
        ])
    add_subsection(document, '6.2. Comparison with prior work',
        'Trả lời novelty sau khi đã có result.', [
            'So continuity, collision representation, timing and multi-corner coupling với closest methods.',
            'Nhấn integration/evaluation contribution; không hạ thấp TEB/trajectory optimization vì khác tầng.',
        ])
    add_subsection(document, '6.3. Threats to validity',
        'Nêu limitation theo bốn nhóm chuẩn.', [
            'Internal: concurrency load, measurement discretization, sequential method order, simulator determinism.',
            'Construct: Eκ không phải energy; PGM clearance không phải physical contact; success threshold sensitivity.',
            'External: one robot model/controller/repeated map; historical multi-map cohort only descriptive.',
            'Conclusion: 25 blocks and fixed repetitions; no stochastic seed; secondary outcomes exploratory.',
        ])
    add_subsection(document, '6.4. Required ablation and future validation',
        'Biến các thiếu sót còn lại thành kế hoạch rõ ràng.', [
            'Ablation A: DP-on vs local-greedy on identical candidates; outcomes compatibility failures, pivots, Eκ, runtime.',
            'Ablation B: swept-footprint vs center/pose check on targeted near-rack cases; report unsafe candidates rejected, not just final success.',
            'Ablation C: two-trim vs preferred-only; report feasibility, candidate evaluations and selected transitions.',
            'Robustness: controlled pose noise, friction/payload, dynamic obstacle episodes with preregistered seeds.',
            'Hardware: calibration, wheel separation/footprint reconciliation, onboard CPU latency, current/Wh and safety supervision.',
        ], output='Nếu chưa chạy trước submission, ghi thẳng là future work; không trình bày snapshot development như definitive ablation.')

    document.add_heading('7. CONCLUSION (0.4–0.6 TRANG)', level=1)
    add_bullets(document, [
        '1–2 câu nhắc bài toán và pipeline PSTMO.',
        '2–3 câu trả lời RQ bằng effect size/CI và success, không lặp toàn bộ bảng.',
        '1 câu về trade-off clearance/computation và planner dependence.',
        '1 câu scope: static Gazebo warehouse, fixed-condition repetitions.',
        '1 câu next step: same-commit component ablation + controlled robustness + hardware validation.',
    ])

    page_break(document)
    document.add_heading('C. HÌNH, BẢNG VÀ SUPPLEMENTARY MATERIAL', level=1)
    add_table(document,
        ['ID', 'Nội dung', 'Vai trò / nguồn'],
        [
            ['Fig. 1', 'Nav2 architecture and PSTMO position', 'Tự dựng từ integration source.'],
            ['Fig. 2', 'Quintic G² geometry P0…P5', 'Tự dựng từ quintic_transition.cpp.'],
            ['Fig. 3', 'Two-trim search + DP state lattice', 'Tự dựng; hình method quan trọng nhất.'],
            ['Fig. 4', 'Map, footprint and five routes', 'overview_5_routes + dimensions.'],
            ['Fig. 5', 'Main-matrix geometry trade-offs', 'Eκ / κmax / clearance; paired groups.'],
            ['Fig. 6', 'Closed-loop success/time/tracking', 'Aggregate; failures retained.'],
            ['Fig. 7', '25 block effects PSTMO−Constrained', 'repeatability_block_differences.png.'],
            ['Fig. 8', 'One representative path/trajectory case', 'Choose median-effect block, not best case.'],
            ['Optional', 'Failure or near-rack case', 'Chỉ giữ nếu làm rõ mechanism/scope.'],
        ])
    add_table(document,
        ['Table', 'Nội dung bắt buộc'],
        [
            ['T1', 'Related-work capability matrix.'],
            ['T2', 'Robot, map, controller and PSTMO parameters with units.'],
            ['T3', 'Experimental factors, sample counts, success criteria and metrics.'],
            ['T4', 'Main-matrix descriptive comparison for five methods.'],
            ['T5', 'Repeated primary outcomes: effect, CI, adjusted p.'],
            ['T6', 'Safety/computation audit and limitations.'],
        ])
    add_paragraph(
        document,
        'Supplement: per-route/per-planner tables, traces, all secondary outcomes, '
        'scenario YAML, config/source hashes, raw JSON/log, analysis scripts and '
        'the 106+ page technical report. Main paper must remain readable without supplement.',
        'Note')

    document.add_heading('D. CLAIM–EVIDENCE MATRIX', level=1)
    add_table(document,
        ['Claim', 'Evidence allowed', 'Required wording'],
        [
            ['PSTMO reduces curvature energy vs Constrained', 'Repeated block effect + CI + Holm p.', 'Quantify; distinguish planned/executed.'],
            ['PSTMO changes execution time/tracking', 'Repeated primary outcomes.', 'Use effect/CI; mention heterogeneity.'],
            ['No observed collision signal', '0 interventions / 0 planned samples if audit remains zero.', '“Observed in this cohort”, not “guaranteed safe”.'],
            ['DP enforces compatibility', 'Algorithm, unit tests, diagnostics.', 'Design property; no causal performance claim without ablation.'],
            ['Works across planners/routes', '25 blocks in one map.', 'Within tested routes/planners; not universal.'],
            ['Generalizes across maps', 'Historical seven-environment descriptive snapshot.', 'Supporting evidence only unless rerun current repeated protocol.'],
            ['Real-world capable', 'No current evidence.', 'Do not claim; list hardware validation plan.'],
        ])

    document.add_heading('E. TRÌNH TỰ VIẾT VÀ GATE TRƯỚC KHI NỘP', level=1)
    add_bullets(document, [
        'Gate 1 — chọn journal và tải author guidelines/template; xác nhận page limit, article type, data/code policy và quartile theo năm nộp.',
        'Gate 2 — khóa dataset/analysis; lưu commit, config hash, aggregate SHA; không retune trên test cases sau khi xem kết quả.',
        'Gate 3 — hoàn thành Fig. 1–7 và Table T1–T6 trước khi viết prose; mọi con số trong Abstract phải truy về aggregate.',
        'Gate 4 — viết Sections 3–5 trước, rồi Related Work/Introduction, cuối cùng Abstract/Conclusion.',
        'Gate 5 — kiểm định terminology: planner ≠ smoother ≠ controller; path ≠ trajectory; Eκ ≠ energy; repeatability ≠ robustness.',
        'Gate 6 — reference audit: venue, year, DOI, final version; loại citation không được dùng; ưu tiên primary sources.',
        'Gate 7 — reproducibility audit: clean-clone command, checksums, README, failed trials, software/license metadata.',
        'Gate 8 — internal review: novelty reviewer, methods/statistics reviewer, English/figure reviewer; chuẩn bị response-to-reviewers matrix.',
    ])
    add_paragraph(
        document,
        'Điều kiện “ready to submit”: không còn placeholder; tất cả figures có '
        'caption tự đủ nghĩa; primary outcomes khớp aggregate; limitation xuất hiện '
        'ở Abstract/Discussion/Conclusion; source và data package tái lập được.',
        'Note')

    document.add_heading('F. NGUỒN DỮ LIỆU VÀ TỆP LÀM VIỆC', level=1)
    add_bullets(document, [
        'docs/PSTMO_KHO_GIAO_CAT_5_QUY_DAO.pdf — báo cáo kỹ thuật đã bổ sung cohort lặp.',
        'docs/warehouse_cross_aisles_5_routes/all_125_trials.csv — ma trận chính.',
        'results/pstmo_journal_repeatability_20261011/ — 250 trial, logs, diagnostics, manifest.',
        'results/pstmo_journal_repeatability_20261011/analysis/aggregate.json — kết quả thống kê dùng cho Table T5.',
        'tools/run_journal_repeatability_study.py — orchestration.',
        'tools/analyze_journal_repeatability.py — audit/statistics.',
        'src/adaptive_pivot_g2_nav2/src/adaptive_pivot_g2_smoother.cpp — pipeline integration.',
        'src/adaptive_pivot_g2/src/{quintic_transition,hierarchical_shape_search,path_optimization}.cpp — method core.',
        'src/adaptive_pivot_g2_nav2/src/footprint_safety.cpp — swept-footprint checks.',
    ])
    commit = subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True
    ).strip()
    add_paragraph(document, f'Git commit của cohort: {commit}', 'Note')
    add_paragraph(
        document,
        'Dàn ý này là bản thiết kế để bắt đầu viết. Nó tăng xác suất một bài '
        'được đánh giá nghiêm túc nhưng không thể bảo đảm quyết định nhận đăng, '
        'vốn còn phụ thuộc novelty, fit với journal, chất lượng tiếng Anh và phản biện.',
        'Note')

    # Normalize body typography without changing the inherited cover hierarchy.
    for paragraph in document.paragraphs:
        if paragraph.style.name in ('Normal', 'Compact', 'Lead', 'Note'):
            paragraph.paragraph_format.space_after = Pt(3)
    document.save(OUTPUT)
    print(OUTPUT)


if __name__ == '__main__':
    build()
