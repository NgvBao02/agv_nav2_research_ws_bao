#!/usr/bin/env python3

"""Audit and summarize the full PSTMO travel-time execution matrix.

The benchmark combines the four planners recorded on 2026-08-03 with the
Theta* trials recorded on 2026-08-02.  Every comparison is paired by the raw
planner-path SHA-256 inside one environment and planner.
"""

from __future__ import annotations

import csv
import json
from collections import defaultdict
from pathlib import Path
import statistics


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results" / "pstmo_execution_full_20260803"
THETA_RESULTS = ROOT / "results" / "pstmo_execution_theta_star_20260802"
METHODS = ("raw", "simple", "savitzky_golay", "constrained", "pstmo")
PLANNERS = (
    "NavFnAStar",
    "NavFnDijkstra",
    "ThetaStar",
    "Smac2D",
    "SmacHybrid",
)
NEW_PLANNERS = tuple(planner for planner in PLANNERS if planner != "ThetaStar")
EXPECTED = {
    "open_arena": ("center_block_detour", "pilot_open_arena"),
    "research_warehouse": ("lower_left_diagonal", "research_warehouse"),
    "narrow_aisles": ("southwest_northeast_weave", "narrow_aisles"),
    "office_maze": ("office_long_diagonal", "office_maze"),
    "warehouse_cross_aisles": (
        "cross_aisle_transfer",
        "warehouse_cross_aisles",
    ),
    "warehouse_dispatch": ("full_replenishment", "warehouse_dispatch"),
    "warehouse_long_aisles": (
        "diagonal_replenishment",
        "warehouse_long_aisles",
    ),
}


def _read(path: Path) -> dict:
    if not path.is_file():
        raise RuntimeError(f"missing benchmark summary: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _audit_record(record: dict, environment: str) -> None:
    label = f"{environment}/{record.get('planner')}/{record.get('method')}"
    if record.get("success"):
        for field in (
            "controller_action_succeeded",
            "physically_settled",
            "ground_truth_goal_reached",
        ):
            if not record.get(field):
                raise RuntimeError(f"{label}: successful trial has {field}=false")
    for field in (
        "collision_monitor_interventions",
        "planned_footprint_collision_sample_count",
    ):
        value = record.get(field)
        if value is not None and int(value) != 0:
            raise RuntimeError(f"{label}: {field} is not zero")


def load_records() -> list[dict]:
    """Load, validate, and return the complete 175-trial matrix."""
    records: list[dict] = []
    for environment, (scenario, theta_directory) in EXPECTED.items():
        new_summary = _read(RESULTS / environment / f"{scenario}_summary.json")
        if new_summary.get("planners") != list(NEW_PLANNERS):
            raise RuntimeError(f"unexpected planners for {environment}")
        if new_summary.get("methods") != list(METHODS):
            raise RuntimeError(f"unexpected methods for {environment}")
        new_records = new_summary.get("records", [])
        if len(new_records) != len(NEW_PLANNERS) * len(METHODS):
            raise RuntimeError(f"incomplete new records for {environment}")

        theta_summary = _read(
            THETA_RESULTS / theta_directory / f"{scenario}_summary.json"
        )
        if theta_summary.get("planner") != "ThetaStar":
            raise RuntimeError(f"unexpected Theta* planner for {environment}")
        if theta_summary.get("methods") != list(METHODS):
            raise RuntimeError(f"unexpected Theta* methods for {environment}")
        if not theta_summary.get("paired_comparison_valid"):
            raise RuntimeError(f"unpaired Theta* records for {environment}")
        theta_records = theta_summary.get("records", [])
        if len(theta_records) != len(METHODS):
            raise RuntimeError(f"incomplete Theta* records for {environment}")

        for source, source_directory, source_records in (
            (
                "2026-08-03 four-planner matrix",
                RESULTS / environment,
                new_records,
            ),
            (
                "2026-08-02 ThetaStar matrix",
                THETA_RESULTS / theta_directory,
                theta_records,
            ),
        ):
            for source_record in source_records:
                _audit_record(source_record, environment)
                record = dict(source_record)
                if record["planner"] == "ThetaStar":
                    trial_name = f"{scenario}_{record['method']}.json"
                else:
                    trial_name = (
                        f"{scenario}_{record['planner'].lower()}_"
                        f"{record['method']}.json"
                    )
                trial_path = source_directory / trial_name
                if not trial_path.is_file():
                    raise RuntimeError(f"missing per-trial evidence: {trial_path}")
                record["benchmark_environment"] = environment
                record["benchmark_source"] = source
                record["trial_json"] = str(trial_path.relative_to(ROOT))
                records.append(record)

    expected_count = len(EXPECTED) * len(PLANNERS) * len(METHODS)
    if len(records) != expected_count:
        raise RuntimeError(f"expected {expected_count} records, found {len(records)}")

    grouped: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for record in records:
        grouped[(record["benchmark_environment"], record["planner"])].append(record)
    for (environment, planner), group in grouped.items():
        found_methods = {record["method"] for record in group}
        if found_methods != set(METHODS) or len(group) != len(METHODS):
            raise RuntimeError(f"incomplete group: {environment}/{planner}")
        hashes = {
            record["raw_path_sha256"]
            for record in group
            if record.get("raw_path_sha256")
        }
        if len(hashes) != 1:
            raise RuntimeError(f"raw-path SHA mismatch: {environment}/{planner}")
    return records


def _stats(records: list[dict]) -> dict:
    successful = [
        record for record in records
        if record.get("success") and record.get("execution_time_s") is not None
    ]
    values = [float(record["execution_time_s"]) for record in successful]
    return {
        "trial_count": len(records),
        "successful_time_sample_count": len(values),
        "success_count": sum(bool(record["success"]) for record in records),
        "failure_count": sum(not bool(record["success"]) for record in records),
        "execution_time_s_mean": statistics.fmean(values),
        "execution_time_s_stdev": statistics.stdev(values) if len(values) > 1 else 0.0,
        "execution_time_s_min": min(values),
        "execution_time_s_max": max(values),
    }


def _paired_comparison(records: list[dict], baseline_method: str) -> dict:
    differences: list[float] = []
    relative_changes: list[float] = []
    wins = ties = losses = 0
    by_planner: dict[str, dict] = {}
    for planner in PLANNERS:
        planner_differences: list[float] = []
        planner_wins = 0
        for environment in EXPECTED:
            group = [
                record
                for record in records
                if record["benchmark_environment"] == environment
                and record["planner"] == planner
            ]
            baseline = next(
                record for record in group if record["method"] == baseline_method
            )
            pstmo = next(record for record in group if record["method"] == "pstmo")
            if not baseline.get("success") or not pstmo.get("success"):
                continue
            baseline_time = float(baseline["execution_time_s"])
            difference = float(pstmo["execution_time_s"]) - baseline_time
            differences.append(difference)
            planner_differences.append(difference)
            relative_changes.append(100.0 * difference / baseline_time)
            if difference < -1.0e-9:
                wins += 1
                planner_wins += 1
            elif difference > 1.0e-9:
                losses += 1
            else:
                ties += 1
        by_planner[planner] = {
            "paired_difference_s_mean": (
                statistics.fmean(planner_differences)
                if planner_differences else None
            ),
            "pstmo_faster_environment_count": planner_wins,
            "paired_environment_count": len(planner_differences),
        }
    return {
        "pair_count": len(differences),
        "paired_difference_s_mean": statistics.fmean(differences),
        "paired_relative_change_percent_mean": statistics.fmean(relative_changes),
        "pstmo_faster_pair_count": wins,
        "tie_pair_count": ties,
        "pstmo_slower_pair_count": losses,
        "by_planner": by_planner,
    }


def main() -> None:
    records = load_records()
    grouped: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for record in records:
        grouped[(record["benchmark_environment"], record["planner"])].append(
            record
        )
    aggregate: dict = {
        "protocol": {
            "environment_count": len(EXPECTED),
            "planners": list(PLANNERS),
            "methods": list(METHODS),
            "repetitions_per_environment_planner_method": 1,
            "isolated_fresh_simulation_per_trial": True,
            "pairing_key": "environment + planner + raw_path_sha256",
            "execution_time_definition": (
                "FollowPath acceptance to physical ground-truth settling"
            ),
        },
        "audit": {
            "trial_count": len(records),
            "success_count": sum(bool(record["success"]) for record in records),
            "failure_count": sum(not bool(record["success"]) for record in records),
            "controller_success_count": sum(
                bool(record.get("controller_action_succeeded"))
                for record in records
            ),
            "physically_settled_count": sum(
                bool(record.get("physically_settled")) for record in records
            ),
            "ground_truth_goal_reached_count": sum(
                bool(record.get("ground_truth_goal_reached"))
                for record in records
            ),
            "collision_monitor_intervention_count": sum(
                int(record.get("collision_monitor_interventions") or 0)
                for record in records
            ),
            "planned_footprint_collision_sample_count": sum(
                int(record.get("planned_footprint_collision_sample_count") or 0)
                for record in records
            ),
            "complete_paired_group_count": len(EXPECTED) * len(PLANNERS),
            "all_methods_successful_group_count": sum(
                all(record.get("success") for record in group)
                for group in grouped.values()
            ),
            "exact_raw_hash_complete_group_count": sum(
                all(record.get("raw_path_sha256") for record in group)
                for group in grouped.values()
            ),
        },
        "overall_by_method": {},
        "by_planner_and_method": {},
        "by_environment_and_method": {},
        "pstmo_paired_comparison": {},
        "failures": [
            {
                "benchmark_environment": record["benchmark_environment"],
                "scenario": record.get("scenario"),
                "planner": record.get("planner"),
                "method": record.get("method"),
                "controller_error_code": record.get("controller_error_code"),
                "controller_error_msg": record.get("controller_error_msg"),
                "planned_footprint_clearance_min_m": record.get(
                    "planned_footprint_clearance_min_m"
                ),
                "planned_footprint_collision_sample_count": record.get(
                    "planned_footprint_collision_sample_count"
                ),
                "trial_json": record["trial_json"],
                "trial_log": record.get("trial_log"),
            }
            for record in records
            if not record.get("success")
        ],
    }
    for method in METHODS:
        aggregate["overall_by_method"][method] = _stats(
            [record for record in records if record["method"] == method]
        )
    for planner in PLANNERS:
        aggregate["by_planner_and_method"][planner] = {}
        for method in METHODS:
            aggregate["by_planner_and_method"][planner][method] = _stats(
                [
                    record
                    for record in records
                    if record["planner"] == planner and record["method"] == method
                ]
            )
    for environment in EXPECTED:
        aggregate["by_environment_and_method"][environment] = {}
        for method in METHODS:
            aggregate["by_environment_and_method"][environment][method] = _stats(
                [
                    record
                    for record in records
                    if record["benchmark_environment"] == environment
                    and record["method"] == method
                ]
            )
    for baseline in METHODS[:-1]:
        aggregate["pstmo_paired_comparison"][baseline] = _paired_comparison(
            records, baseline
        )

    fields = [
        "benchmark_environment",
        "benchmark_source",
        "trial_json",
        "scenario",
        "planner",
        "method",
        "success",
        "execution_time_s",
        "controller_action_time_s",
        "physical_settle_time_s",
        "traveled_distance_m",
        "final_position_error_m",
        "final_yaw_error_rad",
        "mean_cross_track_error_m",
        "max_cross_track_error_m",
        "tracking_rmse_m",
        "tracking_max_error_m",
        "collision_monitor_interventions",
        "planned_footprint_collision_sample_count",
        "raw_path_sha256",
        "selected_path_sha256",
    ]
    csv_path = RESULTS / "execution_175_cases.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in sorted(
            records,
            key=lambda item: (
                list(EXPECTED).index(item["benchmark_environment"]),
                PLANNERS.index(item["planner"]),
                METHODS.index(item["method"]),
            ),
        ):
            writer.writerow({field: record.get(field) for field in fields})

    json_path = RESULTS / "execution_aggregate_5planners_7env.json"
    json_path.write_text(
        json.dumps(aggregate, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(aggregate, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
