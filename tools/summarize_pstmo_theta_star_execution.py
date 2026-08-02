#!/usr/bin/env python3

"""Audit and summarize the seven-environment Theta* execution benchmark."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import statistics


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results" / "pstmo_execution_theta_star_20260802"
METHODS = ("raw", "simple", "savitzky_golay", "constrained", "pstmo")
EXPECTED = {
    "pilot_open_arena": "center_block_detour",
    "research_warehouse": "lower_left_diagonal",
    "narrow_aisles": "southwest_northeast_weave",
    "office_maze": "office_long_diagonal",
    "warehouse_cross_aisles": "cross_aisle_transfer",
    "warehouse_dispatch": "full_replenishment",
    "warehouse_long_aisles": "diagonal_replenishment",
}


def load_records() -> list[dict]:
    """Load only complete, paired, ground-truth-valid execution records."""
    records: list[dict] = []
    for directory, scenario in EXPECTED.items():
        summary_path = RESULTS / directory / f"{scenario}_summary.json"
        document = json.loads(summary_path.read_text(encoding="utf-8"))
        if document.get("planner") != "ThetaStar":
            raise RuntimeError(f"unexpected planner in {summary_path}")
        if document.get("methods") != list(METHODS):
            raise RuntimeError(f"unexpected methods in {summary_path}")
        if not document.get("paired_comparison_valid"):
            raise RuntimeError(f"unpaired Raw paths in {summary_path}")
        case_records = document.get("records", [])
        if len(case_records) != len(METHODS):
            raise RuntimeError(f"incomplete method set in {summary_path}")
        for record in case_records:
            if not (
                record.get("success")
                and record.get("controller_action_succeeded")
                and record.get("physically_settled")
                and record.get("ground_truth_goal_reached")
            ):
                raise RuntimeError(
                    f"execution did not reach the ground-truth goal: "
                    f"{directory}/{record.get('method')}"
                )
            if int(record.get("collision_monitor_interventions", 0)) != 0:
                raise RuntimeError(
                    f"collision intervention: {directory}/{record.get('method')}"
                )
            record = dict(record)
            record["benchmark_environment"] = directory
            records.append(record)
    return records


def main() -> None:
    """Write auditable row-level CSV and aggregate JSON files."""
    records = load_records()
    aggregate = {
        "protocol": {
            "environment_count": len(EXPECTED),
            "planner": "ThetaStar",
            "methods": list(METHODS),
            "repetitions_per_environment_method": 1,
            "isolated_fresh_simulation_per_trial": True,
            "execution_time_definition": (
                "FollowPath acceptance to physical ground-truth settling"
            ),
        },
        "trial_count": len(records),
        "success_count": sum(bool(record["success"]) for record in records),
        "collision_monitor_intervention_count": sum(
            int(record["collision_monitor_interventions"])
            for record in records
        ),
        "methods": {},
    }
    for method in METHODS:
        selected = [record for record in records if record["method"] == method]
        values = [float(record["execution_time_s"]) for record in selected]
        aggregate["methods"][method] = {
            "sample_count": len(values),
            "success_count": sum(bool(record["success"]) for record in selected),
            "execution_time_s_mean": statistics.fmean(values),
            "execution_time_s_stdev": statistics.stdev(values),
            "execution_time_s_min": min(values),
            "execution_time_s_max": max(values),
        }
    pstmo_mean = aggregate["methods"]["pstmo"]["execution_time_s_mean"]
    aggregate["pstmo_comparison"] = {}
    for method in METHODS[:-1]:
        baseline_mean = aggregate["methods"][method]["execution_time_s_mean"]
        paired_differences = []
        pstmo_wins = 0
        for environment in EXPECTED:
            baseline = next(
                record for record in records
                if record["benchmark_environment"] == environment
                and record["method"] == method
            )
            pstmo = next(
                record for record in records
                if record["benchmark_environment"] == environment
                and record["method"] == "pstmo"
            )
            difference = (
                float(pstmo["execution_time_s"])
                - float(baseline["execution_time_s"])
            )
            paired_differences.append(difference)
            if difference < 0.0:
                pstmo_wins += 1
        aggregate["pstmo_comparison"][method] = {
            "mean_difference_s": pstmo_mean - baseline_mean,
            "mean_relative_change_percent": (
                (pstmo_mean / baseline_mean - 1.0) * 100.0
            ),
            "paired_difference_s_mean": statistics.fmean(paired_differences),
            "pstmo_faster_environment_count": pstmo_wins,
            "environment_count": len(EXPECTED),
        }

    csv_path = RESULTS / "execution_7env_theta_star.csv"
    fields = [
        "benchmark_environment",
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
        "collision_monitor_interventions",
        "raw_path_sha256",
        "selected_path_sha256",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            writer.writerow({field: record.get(field) for field in fields})

    json_path = RESULTS / "execution_aggregate_7env_theta_star.json"
    json_path.write_text(
        json.dumps(aggregate, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(aggregate, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
