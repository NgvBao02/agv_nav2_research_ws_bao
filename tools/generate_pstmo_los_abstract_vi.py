#!/usr/bin/env python3

"""Audit single-pipeline greedy LOS results and rebuild the PSTMO VI abstract."""

from __future__ import annotations

import ast
import collections
import csv
import hashlib
import html
import io
import json
import math
import shutil
import statistics
import subprocess
import tempfile
import zipfile
from pathlib import Path

from docx import Document
from docx.oxml.ns import qn
from docx.shared import Pt


ROOT = Path(__file__).resolve().parents[1]
GEOMETRY_DIR = ROOT / "results" / "pstmo_greedy_los_single_pipeline_full_20260802"
ABLATION_DIR = ROOT / "results" / "pstmo_joint_dq_condition_only_ablation_20260802"
REQUESTED_DOCX = (
    ROOT / "abstract" / "ICEEIS_2026_ADAPTIVE_PIVOT_G2_ABSTRACT_VI.docx"
)
REQUESTED_HTML = (
    ROOT / "abstract" / "ICEEIS_2026_ADAPTIVE_PIVOT_G2_ABSTRACT_VI.html"
)
REQUESTED_PDF = (
    ROOT / "abstract" / "ICEEIS_2026_ADAPTIVE_PIVOT_G2_ABSTRACT_VI.pdf"
)
OUTPUT_DOCX = (
    ROOT / "abstract" / "ICEEIS_2026_PSTMO_GREEDY_LOS_ABSTRACT_VI.docx"
)
OUTPUT_HTML = (
    ROOT / "abstract" / "ICEEIS_2026_PSTMO_GREEDY_LOS_ABSTRACT_VI.html"
)
OUTPUT_PDF = (
    ROOT / "abstract" / "ICEEIS_2026_PSTMO_GREEDY_LOS_ABSTRACT_VI.pdf"
)
COMPAT_DOCX = (
    ROOT / "abstract" / "ICEEIS_2026_PSTMO_FOOTPRINT_LOS_ABSTRACT_VI.docx"
)
COMPAT_HTML = (
    ROOT / "abstract" / "ICEEIS_2026_PSTMO_FOOTPRINT_LOS_ABSTRACT_VI.html"
)
NAV2_PARAMS = (
    ROOT / "src" / "vacuum_robot_gazebo" / "config" / "nav2_params.yaml"
)

METHODS = ("raw", "simple", "savitzky_golay", "constrained", "pstmo")
METHOD_LABELS = {
    "raw": "Raw",
    "simple": "Simple",
    "savitzky_golay": "Savitzky–Golay",
    "constrained": "Constrained",
    "pstmo": "PSTMO",
}
EXPECTED_ENVIRONMENTS = {
    "research_warehouse": "lower_left_diagonal",
    "narrow_aisles": "southwest_northeast_weave",
    "office_maze": "office_long_diagonal",
    "open_arena": "center_block_detour",
    "warehouse_cross_aisles": "cross_aisle_transfer",
    "warehouse_dispatch": "full_replenishment",
    "warehouse_long_aisles": "diagonal_replenishment",
}
GROUP_FIELDS = ("environment", "scenario", "planner", "repetition")
COMPARISON_METRICS = (
    "translation_path_length_m",
    "translation_max_abs_curvature_1pm",
    "translation_curvature_energy_1pm",
    "pivot_total_angle_rad",
    "footprint_clearance_min_m",
    "algorithm_time_s",
    "wall_time_s",
)


def as_float(value: object) -> float:
    """Convert a benchmark field to a finite float."""
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"Non-finite numeric value: {value!r}")
    return result


def percent_change(value: float, reference: float) -> float:
    """Return a signed percentage change."""
    if abs(reference) <= 1.0e-15:
        return 0.0 if abs(value) <= 1.0e-15 else math.inf
    return 100.0 * (value - reference) / reference


def sha256(path: Path) -> str:
    """Hash a configuration artifact used by the benchmark."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_csv_directory(directory: Path) -> list[dict[str, str]]:
    """Load exactly one benchmark CSV for every representative environment."""
    rows: list[dict[str, str]] = []
    for environment, scenario in EXPECTED_ENVIRONMENTS.items():
        path = directory / f"{environment}.csv"
        with path.open(newline="", encoding="utf-8") as stream:
            selected = list(csv.DictReader(stream))
        if {row["scenario"] for row in selected} != {scenario}:
            raise RuntimeError(f"Unexpected scenario set in {path}")
        for row in selected:
            row["environment"] = environment
        rows.extend(selected)
    return rows


def group_rows(rows: list[dict[str, str]]) -> dict[tuple[str, ...], list[dict[str, str]]]:
    """Group paired records by environment, scenario, planner and repetition."""
    groups: dict[tuple[str, ...], list[dict[str, str]]] = collections.defaultdict(list)
    for row in rows:
        groups[tuple(row[field] for field in GROUP_FIELDS)].append(row)
    return groups


def validate_single_pipeline_contract(rows: list[dict[str, str]], mode: str) -> None:
    """Validate diagnostics emitted by one PSTMO pipeline execution."""
    if len(rows) != 35 or any(row["success"] != "True" for row in rows):
        raise RuntimeError(f"PSTMO {mode} must succeed in all 35 cases")
    required = {
        "pstmo_preprocessing_mode": mode,
        "pstmo_pipeline_execution_count": "1",
        "pstmo_final_invariants_verified": "True",
        "pstmo_search_mode": "joint_d_q",
        "pstmo_trim_domain": "direct_metric",
    }
    for field, expected in required.items():
        if any(row.get(field) != expected for row in rows):
            raise RuntimeError(f"PSTMO contract mismatch: {field} != {expected}")
    removed = {
        "pstmo_los_selection_enabled",
        "pstmo_los_selected",
        "pstmo_los_fallback_to_input",
        "pstmo_los_fallback_reason",
        "pstmo_los_footprint_padding_m",
    }
    if any(field in row for row in rows for field in removed):
        raise RuntimeError("A removed selector, fallback or padding field remains")
    if sum(
        int(as_float(row["footprint_collision_sample_count"])) for row in rows
    ):
        raise RuntimeError(f"A {mode} output contains footprint collision samples")


def mean(rows: list[dict[str, str]], field: str) -> float:
    """Calculate a finite arithmetic mean."""
    return statistics.fmean(as_float(row[field]) for row in rows)


def paired_comparison(
    baseline: dict[tuple[str, ...], dict[str, str]],
    proposed: dict[tuple[str, ...], dict[str, str]],
) -> dict:
    """Compare condition-only and condition-then-LOS paths on identical inputs."""
    if baseline.keys() != proposed.keys() or len(proposed) != 35:
        raise RuntimeError("LOS and condition-only paired keys do not match")
    hash_matches = sum(
        baseline[key]["raw_path_sha256"] == proposed[key]["raw_path_sha256"]
        for key in proposed
    )
    if hash_matches != 35:
        raise RuntimeError("LOS ablation raw-path hash mismatch")

    metrics = {}
    for field in COMPARISON_METRICS:
        before = [as_float(baseline[key][field]) for key in proposed]
        after = [as_float(proposed[key][field]) for key in proposed]
        before_mean = statistics.fmean(before)
        after_mean = statistics.fmean(after)
        metrics[field] = {
            "condition_only_mean": before_mean,
            "condition_then_los_mean": after_mean,
            "relative_change_percent": percent_change(after_mean, before_mean),
            "lower_count": sum(
                right < left - 1.0e-12 for left, right in zip(before, after)
            ),
            "equal_count": sum(
                abs(right - left) <= 1.0e-12
                for left, right in zip(before, after)
            ),
            "higher_count": sum(
                right > left + 1.0e-12 for left, right in zip(before, after)
            ),
        }
    return {
        "paired_group_count": 35,
        "raw_path_hash_match_count": hash_matches,
        "condition_only_success_count": 35,
        "condition_then_los_success_count": 35,
        "metrics": metrics,
    }


def load_geometry() -> tuple[list[dict[str, str]], dict]:
    """Audit final and ablation datasets and calculate report statistics."""
    rows = load_csv_directory(GEOMETRY_DIR)
    if len(rows) != 175 or {row["method"] for row in rows} != set(METHODS):
        raise RuntimeError("Final comparison must contain 175 rows and five methods")
    if any(row["method"] == "adaptive_hybrid" for row in rows):
        raise RuntimeError("Adaptive Hybrid appeared in the PSTMO-only report")

    groups = group_rows(rows)
    if len(groups) != 35:
        raise RuntimeError(f"Expected 35 paired groups, found {len(groups)}")
    for key, group in groups.items():
        if len(group) != 5 or {row["method"] for row in group} != set(METHODS):
            raise RuntimeError(f"Incomplete method set in {key}")
        if len({row["raw_path_sha256"] for row in group}) != 1:
            raise RuntimeError(f"Raw-path hash mismatch in {key}")

    success_counts = {
        method: sum(
            row["method"] == method and row["success"] == "True"
            for row in rows
        )
        for method in METHODS
    }
    expected_success = {
        "raw": 35,
        "simple": 34,
        "savitzky_golay": 35,
        "constrained": 35,
        "pstmo": 35,
    }
    if success_counts != expected_success:
        raise RuntimeError(f"Unexpected success counts: {success_counts}")

    pstmo_rows = [row for row in rows if row["method"] == "pstmo"]
    validate_single_pipeline_contract(pstmo_rows, "condition_then_los")
    if any(as_float(row["pstmo_los_accepted_shortcuts"]) < 0.0 for row in pstmo_rows):
        raise RuntimeError("Invalid accepted shortcut count")

    ablation_rows = load_csv_directory(ABLATION_DIR)
    if len(ablation_rows) != 70 or {
        row["method"] for row in ablation_rows
    } != {"raw", "pstmo"}:
        raise RuntimeError("Condition-only ablation must contain 70 Raw/PSTMO rows")
    ablation_pstmo = [
        row for row in ablation_rows if row["method"] == "pstmo"
    ]
    validate_single_pipeline_contract(ablation_pstmo, "condition_only")
    final_by_key = {
        tuple(row[field] for field in GROUP_FIELDS): row for row in pstmo_rows
    }
    ablation_by_key = {
        tuple(row[field] for field in GROUP_FIELDS): row
        for row in ablation_pstmo
    }
    comparison = paired_comparison(ablation_by_key, final_by_key)

    common_groups = [
        group for group in groups.values()
        if all(row["success"] == "True" for row in group)
    ]
    if len(common_groups) != 34:
        raise RuntimeError("Expected 34 all-method common-success groups")
    common_rows = [row for group in common_groups for row in group]
    report_metrics = (
        "translation_path_length_m",
        "translation_max_abs_curvature_1pm",
        "translation_curvature_energy_1pm",
        "footprint_clearance_min_m",
        "algorithm_time_s",
        "wall_time_s",
    )
    common_means = {
        method: {
            field: mean(
                [row for row in common_rows if row["method"] == method],
                field,
            )
            for field in report_metrics
        }
        for method in METHODS
    }

    corner_records = []
    for row in pstmo_rows:
        parsed = ast.literal_eval(row["pstmo_corner_search"])
        if not isinstance(parsed, list):
            raise RuntimeError("Expected a list in pstmo_corner_search")
        corner_records.extend(parsed)
    transitions = [
        corner for corner in corner_records
        if as_float(corner.get("selected_trim", 0.0)) > 0.0
    ]
    q_values = [as_float(corner["selected_control_fraction"]) for corner in transitions]
    d_values = [as_float(corner["selected_trim"]) for corner in transitions]

    los_stat_fields = (
        "pstmo_los_input_points",
        "pstmo_los_output_points",
        "pstmo_los_attempted_shortcuts",
        "pstmo_los_accepted_shortcuts",
        "pstmo_los_safety_rejections",
        "pstmo_los_runtime_s",
        "pstmo_runtime_s",
    )
    los_statistics = {}
    for field in los_stat_fields:
        values = [as_float(row[field]) for row in pstmo_rows]
        name = field.removeprefix("pstmo_")
        los_statistics[name] = {
            "mean": statistics.fmean(values),
            "minimum": min(values),
            "maximum": max(values),
            "sum": sum(values),
        }

    successful = [row for row in rows if row["success"] == "True"]
    collision_record_count = sum(
        as_float(row.get("footprint_collision_sample_count", "0")) > 0.0
        for row in successful
    )
    summary = {
        "design": {
            "environment_count": 7,
            "scenario_count": 7,
            "planner_count": 5,
            "methods": list(METHODS),
            "repetitions": 1,
            "paired_group_count": 35,
            "record_count": 175,
            "common_success_group_count": 34,
        },
        "configuration": {
            "preprocessing": "condition_then_greedy_los",
            "line_of_sight_is_mandatory": True,
            "line_of_sight_footprint_padding_m": 0.0,
            "line_of_sight_clearance_objective": False,
            "swept_translation_checked": True,
            "swept_rotation_checked_at_start_junctions_and_goal": True,
            "adjacent_conditioned_edge_is_normal_candidate": True,
            "alternative_pipeline_fallback": False,
            "minimum_trim_distance_m": 0.02,
            "maximum_trim_distance_m": 0.8,
            "nav2_params_sha256": sha256(NAV2_PARAMS),
        },
        "validation": {
            "raw_path_hash_consistent_group_count": 35,
            "single_pipeline_diagnostic_count": 35,
            "final_invariants_verified_count": 35,
            "successful_record_count": len(successful),
            "failed_record_count": len(rows) - len(successful),
            "successful_footprint_collision_record_count": collision_record_count,
            "pstmo_footprint_collision_sample_count": sum(
                int(as_float(row["footprint_collision_sample_count"]))
                for row in pstmo_rows
            ),
            "adaptive_hybrid_record_count": 0,
        },
        "success_count": success_counts,
        "greedy_los": los_statistics,
        "condition_only_ablation": comparison,
        "joint_d_q": {
            "selected_transition_count": len(transitions),
            "angle_aware_q_count": sum(
                abs(value - 0.35) > 1.0e-8 for value in q_values
            ),
            "control_fraction_mean": statistics.fmean(q_values),
            "control_fraction_min": min(q_values),
            "control_fraction_max": max(q_values),
            "trim_distance_mean_m": statistics.fmean(d_values),
            "trim_distance_min_m": min(d_values),
            "trim_distance_max_m": max(d_values),
        },
        "common_success_means": common_means,
    }
    return rows, summary


def vi_number(value: float, digits: int = 2) -> str:
    """Format a number with a Vietnamese decimal comma."""
    return f"{value:.{digits}f}".replace(".", ",")


def abstract_text(geometry: dict) -> str:
    """Build the Vietnamese abstract from audited values only."""
    comparison = geometry["condition_only_ablation"]["metrics"]
    means = geometry["common_success_means"]
    los = geometry["greedy_los"]
    dq = geometry["joint_d_q"]
    pstmo = means["pstmo"]
    best_stock_energy = min(
        means[method]["translation_curvature_energy_1pm"]
        for method in ("simple", "savitzky_golay", "constrained")
    )
    best_stock_curvature = min(
        means[method]["translation_max_abs_curvature_1pm"]
        for method in ("simple", "savitzky_golay", "constrained")
    )
    best_stock_length = min(
        means[method]["translation_path_length_m"]
        for method in ("simple", "savitzky_golay", "constrained")
    )
    length = comparison["translation_path_length_m"]
    curvature = comparison["translation_max_abs_curvature_1pm"]
    energy = comparison["translation_curvature_energy_1pm"]
    clearance = comparison["footprint_clearance_min_m"]
    runtime = comparison["algorithm_time_s"]
    return (
        "Tóm tắt—Các bộ lập kế hoạch toàn cục trong ROS 2/Nav2 thường tạo "
        "đường gấp khúc có đổi hướng đột ngột. Nghiên cứu này đề xuất phương "
        "pháp làm mượt đường đi và tối ưu hóa thao tác chuyển hướng PSTMO cho "
        "robot di động vi sai hai bánh. Sau khi điều kiện hóa đường planner, "
        "PSTMO luôn chạy line-of-sight (LOS) tham lam: từ mỗi điểm neo, thuật "
        "toán chọn điểm xa nhất mà dịch chuyển theo dây cung và các phép xoay "
        "tại start, đỉnh giữ lại và goal đều không va chạm khi quét bằng "
        "footprint thật. LOS không phóng to footprint, không dùng clearance "
        "riêng, không so sánh hai pipeline và không fallback; cạnh liên tiếp "
        "của polyline đã điều kiện hóa là ứng viên tự nhiên cuối cùng. Mỗi góc "
        "sau LOS được biểu diễn bằng quay tại chỗ hoặc chuyển tiếp Bézier bậc "
        "năm liên tục hình học G². Khoảng cắt d được tìm trực tiếp theo mét; "
        "với từng d, nhiều tỷ lệ q/d theo góc được kiểm tra giới hạn bánh xe, "
        "tham số hóa thời gian và swept-footprint trước khi tối ưu hóa toàn "
        "cục giữa các góc. "
        f"Trong {dq['selected_transition_count']} chuyển tiếp được chọn, "
        f"{dq['angle_aware_q_count']} chuyển tiếp dùng q/d khác 0,35. Đánh giá "
        "gồm 175 bản ghi của 7 tình huống, 5 global planner và 5 phương án "
        "Raw, Simple, Savitzky–Golay, Constrained và PSTMO; 35/35 nhóm có cùng "
        "hash đường Raw. Một ablation độc lập condition-only dùng cùng source "
        "và cùng 35 hash, trong đó mỗi cấu hình chỉ chạy một pipeline. So với "
        "condition-only, LOS giảm chiều dài trung bình từ "
        f"{vi_number(length['condition_only_mean'], 3)} xuống "
        f"{vi_number(length['condition_then_los_mean'], 3)} m "
        f"({vi_number(abs(length['relative_change_percent']))}%), giảm độ cong "
        "cực đại từ "
        f"{vi_number(curvature['condition_only_mean'], 3)} xuống "
        f"{vi_number(curvature['condition_then_los_mean'], 3)} m⁻¹ "
        f"({vi_number(abs(curvature['relative_change_percent']))}%) và giảm "
        "năng lượng độ cong từ "
        f"{vi_number(energy['condition_only_mean'], 3)} xuống "
        f"{vi_number(energy['condition_then_los_mean'], 3)} m⁻¹ "
        f"({vi_number(abs(energy['relative_change_percent']))}%). Do LOS giảm "
        "số điểm neo, thời gian thuật toán giảm từ "
        f"{vi_number(1000.0 * runtime['condition_only_mean'], 1)} xuống "
        f"{vi_number(1000.0 * runtime['condition_then_los_mean'], 1)} ms dù "
        f"bản thân LOS mất trung bình {vi_number(1000.0 * los['los_runtime_s']['mean'], 2)} ms. "
        "Đánh đổi là khoảng hở footprint nhỏ nhất trung bình giảm từ "
        f"{vi_number(clearance['condition_only_mean'], 3)} xuống "
        f"{vi_number(clearance['condition_then_los_mean'], 3)} m. PSTMO và ba "
        "đối chứng còn lại thành công 35/35, Simple thành công 34/35; không đầu "
        "ra PSTMO nào có mẫu va chạm footprint. Trên 34 nhóm mọi phương án đều "
        "thành công, PSTMO đạt năng lượng độ cong "
        f"{vi_number(pstmo['translation_curvature_energy_1pm'], 3)} m⁻¹, độ "
        f"cong cực đại {vi_number(pstmo['translation_max_abs_curvature_1pm'], 3)} m⁻¹ "
        f"và chiều dài {vi_number(pstmo['translation_path_length_m'], 3)} m, "
        "so với đối chứng Nav2 tốt nhất tương ứng "
        f"{vi_number(best_stock_energy, 3)} m⁻¹, "
        f"{vi_number(best_stock_curvature, 3)} m⁻¹ và "
        f"{vi_number(best_stock_length, 3)} m. Kết quả cho thấy LOS tham lam "
        "cải thiện hiệu quả hình học và thời gian xử lý, nhưng không chi phối "
        "condition-only về clearance; inflation layer, thử nghiệm lặp, vòng "
        "kín và phần cứng vẫn cần thiết để xác nhận dự phòng vận hành."
    )


def replace_paragraph(paragraph, label: str, body: str) -> None:
    """Replace a labeled DOCX paragraph while retaining paper typography."""
    paragraph.clear()
    first = paragraph.add_run(label)
    first.bold = True
    second = paragraph.add_run(body)
    for run in (first, second):
        run.font.name = "Times New Roman"
        run._element.get_or_add_rPr().rFonts.set(
            qn("w:eastAsia"), "Times New Roman"
        )
        run.font.size = Pt(11)


def deduplicate_docx_archive(path: Path) -> None:
    """Remove duplicate ZIP entries sometimes introduced by office tooling."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(path, "r") as source:
        latest = {info.filename: info for info in source.infolist()}
        with zipfile.ZipFile(buffer, "w") as destination:
            for name, info in latest.items():
                destination.writestr(info, source.read(info))
    path.write_bytes(buffer.getvalue())


def write_docx(text: str) -> None:
    """Update the requested DOCX and synchronized PSTMO copies."""
    document = Document(REQUESTED_DOCX)
    abstracts = [
        paragraph for paragraph in document.paragraphs
        if paragraph.text.strip().startswith("Tóm tắt—")
    ]
    if len(abstracts) != 1:
        raise RuntimeError("Could not uniquely locate the Vietnamese abstract")
    replace_paragraph(abstracts[0], "Tóm tắt—", text.removeprefix("Tóm tắt—"))
    keywords = [
        paragraph for paragraph in document.paragraphs
        if paragraph.text.strip().startswith("Từ khóa—")
    ]
    if len(keywords) != 1:
        raise RuntimeError("Could not uniquely locate the keyword paragraph")
    replace_paragraph(
        keywords[0],
        "Từ khóa—",
        "PSTMO; robot di động vi sai; làm mượt đường đi; line-of-sight tham lam; "
        "swept-footprint; ROS 2/Nav2.",
    )
    document.core_properties.title = "ICEEIS 2026 — PSTMO với LOS tham lam"
    document.core_properties.subject = "Bản tiếng Việt dùng để đối chiếu"
    document.core_properties.keywords = (
        "PSTMO, greedy line-of-sight, swept footprint, path smoothing, "
        "differential drive, ROS 2, Nav2"
    )
    document.save(OUTPUT_DOCX)
    deduplicate_docx_archive(OUTPUT_DOCX)
    for target in (REQUESTED_DOCX, COMPAT_DOCX):
        target.write_bytes(OUTPUT_DOCX.read_bytes())


def write_html(text: str) -> None:
    """Write browser-readable mirrors of the Vietnamese abstract."""
    title = (
        "Phương pháp làm mượt đường đi và tối ưu hóa thao tác chuyển hướng "
        "có xét an toàn cho robot di động vi sai hai bánh"
    )
    body = html.escape(text.removeprefix("Tóm tắt—"))
    output = f"""<!doctype html>
<html lang="vi"><head><meta charset="utf-8"><title>ICEEIS 2026 — PSTMO với LOS tham lam</title>
<style>
body{{font-family:'Times New Roman',serif;max-width:850px;margin:40px auto;
line-height:1.45;font-size:12pt}}
h1{{text-align:center;font-size:18pt}}.notice{{text-align:center;font-weight:bold}}
p{{text-align:justify}}.label{{font-weight:bold}}
</style></head>
<body><p class="notice">BẢN TIẾNG VIỆT CHỈ DÙNG ĐỂ ĐỐI CHIẾU — KHÔNG NỘP LÊN CMT</p>
<h1>{html.escape(title)}</h1>
<p><span class="label">Tóm tắt—</span>{body}</p>
<p><span class="label">Từ khóa—</span>PSTMO; robot di động vi sai; làm mượt
đường đi; line-of-sight tham lam; swept-footprint; ROS 2/Nav2.</p>
</body></html>\n"""
    for target in (OUTPUT_HTML, REQUESTED_HTML, COMPAT_HTML):
        target.write_text(output, encoding="utf-8")


def write_pdf() -> None:
    """Render the synchronized DOCX to PDF with LibreOffice."""
    with tempfile.TemporaryDirectory(prefix="pstmo_abstract_") as directory:
        subprocess.run(
            [
                "libreoffice",
                "--headless",
                "--convert-to",
                "pdf",
                "--outdir",
                directory,
                str(OUTPUT_DOCX),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        rendered = Path(directory) / f"{OUTPUT_DOCX.stem}.pdf"
        if not rendered.exists():
            raise RuntimeError("LibreOffice did not produce the abstract PDF")
        shutil.copy2(rendered, OUTPUT_PDF)
        shutil.copy2(rendered, REQUESTED_PDF)


def write_results_readme(geometry: dict) -> None:
    """Write a concise, reproducible Vietnamese result report."""
    means = geometry["common_success_means"]
    comparison = geometry["condition_only_ablation"]["metrics"]
    los = geometry["greedy_los"]
    lines = [
        "# PSTMO với LOS tham lam swept-footprint — benchmark cuối",
        "",
        "PSTMO độc lập chạy đúng một pipeline `condition_polyline → greedy LOS → "
        "joint (d,q) → stitch/final invariant`. Adaptive Hybrid không nằm trong "
        "benchmark hoặc báo cáo này.",
        "",
        "## Thiết kế và kiểm định",
        "",
        "- 7 môi trường, 7 tình huống đại diện, 5 global planner;",
        "- 5 phương án Raw, Simple, Savitzky–Golay, Constrained và PSTMO;",
        "- 35 nhóm ghép cặp, 175 bản ghi; mọi phương án trong từng nhóm dùng "
        "cùng `raw_path_sha256`;",
        "- PSTMO, Raw, Savitzky–Golay và Constrained thành công 35/35; Simple "
        "thành công 34/35;",
        "- 35/35 bản ghi PSTMO có `condition_then_los`, "
        "`pipeline_execution_count=1` và `final_invariants_verified=true`;",
        "- PSTMO không có mẫu va chạm footprint; không có selector hai nhánh, "
        "padding hay fallback;",
        "- ablation condition-only chạy độc lập, cũng một pipeline, và khớp "
        "35/35 Raw hash với cấu hình LOS.",
        "",
        "## LOS so với condition-only trên đúng 35 đường Raw",
        "",
        "| Chỉ số | Condition-only | Condition + LOS | Thay đổi | Thấp/Bằng/Cao |",
        "|---|---:|---:|---:|---:|",
    ]
    labels = (
        ("translation_path_length_m", "Chiều dài", "m", 3),
        ("translation_max_abs_curvature_1pm", "Kmax", "1/m", 3),
        ("translation_curvature_energy_1pm", "Eκ", "1/m", 3),
        ("pivot_total_angle_rad", "Tổng quay tại chỗ", "rad", 4),
        ("footprint_clearance_min_m", "Clearance nhỏ nhất", "m", 3),
        ("algorithm_time_s", "Thời gian thuật toán", "ms", 1),
        ("wall_time_s", "Wall time", "ms", 1),
    )
    for field, label, unit, digits in labels:
        row = comparison[field]
        scale = 1000.0 if unit == "ms" else 1.0
        lines.append(
            f"| {label} | {scale * row['condition_only_mean']:.{digits}f} {unit} | "
            f"{scale * row['condition_then_los_mean']:.{digits}f} {unit} | "
            f"{row['relative_change_percent']:+.2f}% | "
            f"{row['lower_count']}/{row['equal_count']}/{row['higher_count']} |"
        )
    lines.extend(
        [
            "",
            f"LOS thử {int(los['los_attempted_shortcuts']['sum'])} shortcut, "
            f"chấp nhận {int(los['los_accepted_shortcuts']['sum'])}, loại "
            f"{int(los['los_safety_rejections']['sum'])}; thời gian LOS trung "
            f"bình {1000.0 * los['los_runtime_s']['mean']:.2f} ms. Số điểm neo "
            f"trung bình giảm từ {los['los_input_points']['mean']:.2f} xuống "
            f"{los['los_output_points']['mean']:.2f}.",
            "",
            "LOS giảm chiều dài, Kmax, Eκ và thời gian tổng; đổi lại clearance "
            "giảm mạnh và tổng góc quay tại chỗ tăng nhẹ. Vì vậy LOS không "
            "Pareto-trội condition-only trên mọi chỉ số, dù cả hai cấu hình đều "
            "không có mẫu va chạm trong benchmark hình học.",
            "",
            "## So sánh trên 34 nhóm mọi phương án đều thành công",
            "",
            "| Phương án | L (m) | Kmax (1/m) | Eκ (1/m) | Clearance (m) | "
            "Thuật toán (ms) | Wall (ms) |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for method in METHODS:
        row = means[method]
        lines.append(
            f"| {METHOD_LABELS[method]} | "
            f"{row['translation_path_length_m']:.3f} | "
            f"{row['translation_max_abs_curvature_1pm']:.3f} | "
            f"{row['translation_curvature_energy_1pm']:.3f} | "
            f"{row['footprint_clearance_min_m']:.3f} | "
            f"{1000.0 * row['algorithm_time_s']:.1f} | "
            f"{1000.0 * row['wall_time_s']:.1f} |"
        )
    lines.extend(
        [
            "",
            "Kết luận: cấu hình LOS bắt buộc là lựa chọn tốt hơn nếu ưu tiên "
            "đường ngắn, độ cong thấp và thời gian xử lý; nó không tốt hơn nếu "
            "ưu tiên clearance. Inflation layer chịu trách nhiệm cho dự phòng "
            "vận hành như thiết kế đã chốt, nhưng cần benchmark lặp, vòng kín "
            "và phần cứng trước khi khẳng định an toàn vận hành.",
            "",
        ]
    )
    (GEOMETRY_DIR / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    """Audit data and regenerate all synchronized report artifacts."""
    _, geometry = load_geometry()
    (GEOMETRY_DIR / "aggregate_summary.json").write_text(
        json.dumps(geometry, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    text = abstract_text(geometry)
    write_docx(text)
    write_html(text)
    write_pdf()
    write_results_readme(geometry)
    print(f"Wrote {GEOMETRY_DIR / 'aggregate_summary.json'}")
    print(f"Wrote {GEOMETRY_DIR / 'README.md'}")
    print(f"Updated {REQUESTED_DOCX}")
    print(f"Updated {REQUESTED_HTML}")
    print(f"Updated {REQUESTED_PDF}")


if __name__ == "__main__":
    main()
