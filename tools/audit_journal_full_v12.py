#!/usr/bin/env python3
"""Read-only scientific inputs; writes only ver12/evidence_audit.json.

Does not execute experiments or statistical tests. Numeric checks merely format
already stored aggregate fields in the units of the preserved Word tables.
"""
import hashlib
import json
from pathlib import Path
import subprocess
from zipfile import ZipFile

from lxml import etree as ET

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'Viet_bao/journal/ver12'
COHORT = ROOT/'results/pstmo_journal_repeatability_20261011'
NS = {'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main'}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def main():
    aggregate = read(COHORT/'analysis/aggregate.json')
    manifest = read(COHORT/'study_manifest.json')
    assert sha(COHORT/'study_manifest.json') == aggregate['manifest_sha256']
    fingerprints = []
    for path, expected in aggregate['source_fingerprints'].items():
        actual = sha(ROOT/path)
        assert actual == expected, path
        fingerprints.append({'path':path,'sha256':actual,'matches_aggregate':True})
    matched_code = []
    paths = list((ROOT/'src/adaptive_pivot_g2/src').glob('*.cpp'))
    paths += list((ROOT/'src/adaptive_pivot_g2_nav2/src').glob('*.cpp'))
    paths += [ROOT/'src/vacuum_robot_gazebo/config/nav2_params.yaml']
    for path in sorted(paths):
        rel = str(path.relative_to(ROOT))
        blob = subprocess.check_output(['git','show',manifest['git_commit']+':'+rel],cwd=ROOT)
        assert hashlib.sha256(blob).hexdigest() == sha(path), rel
        matched_code.append({'path':rel,'sha256':sha(path),'matches_cohort_commit':True})
    historical = read(ROOT/'docs/warehouse_cross_aisles_5_routes/audit_manifest.json')
    historical_sources = []
    for item in historical['files']:
        if item['cohort'] == 'source':
            assert sha(ROOT/item['path']) == item['sha256'],item['path']
            historical_sources.append(item)
    with ZipFile(OUT/'PSTMO_JOURNAL_VER12_FULL_MANUSCRIPT.docx') as z:
        doc = ET.fromstring(z.read('word/document.xml'))
    tables = [[[ ''.join(c.xpath('.//w:t/text()',namespaces=NS)) for c in row.findall('w:tc',NS)] for row in t.findall('w:tr',NS)] for t in doc.findall('.//w:tbl',NS)]
    mapping = [(20,1,'execution_time_s',1),(20,2,'tracking_rmse_m',1000),(20,3,'executed_curvature_energy_1pm',1),(21,1,'planned_curvature_energy_1pm',1),(21,2,'actual_p95_abs_jerk_mps3',1),(21,3,'traveled_distance_m',1),(21,4,'final_position_error_m',1000),(21,5,'planned_footprint_clearance_min_m',1000),(21,6,'smoothing_time_s',1000)]
    checks = []
    for table_idx,row_idx,key,factor in mapping:
        metric = aggregate['metrics'][key]
        row = tables[table_idx][row_idx]
        signed = metric['mean_block_difference_pstmo_minus_constrained'] >= 0
        fmt = lambda value: format(value*factor,'+.3f' if signed else '.3f').replace('.',',')
        mean = fmt(metric['mean_block_difference_pstmo_minus_constrained'])
        ci = '['+'; '.join(fmt(v) for v in metric['mean_block_difference_bootstrap_95_ci'])+']'
        assert row[1] == mean and row[2] == ci, (key,row,mean,ci)
        assert metric['paired_successful_trial_count'] == 118
        assert metric['independent_block_count'] == 25
        item = {'metric':key,'table':table_idx+1,'row':row_idx,'unit_multiplier':factor,'delta_as_printed':mean,'ci_as_printed':ci,'matches':True}
        if table_idx == 20:
            value = metric['holm_adjusted_p_primary_family']
            mantissa, exponent = format(value,'.2e').split('e')
            supers = str(int(exponent)).translate(str.maketrans('-0123456789','⁻⁰¹²³⁴⁵⁶⁷⁸⁹'))
            expected_p = mantissa.replace('.',',')+' × 10'+supers
            assert row[3] == expected_p,(key,row[3],expected_p)
            item['p_holm_as_printed'] = expected_p
        checks.append(item)
    assert aggregate['trial_count'] == 250 and aggregate['paired_trial_count'] == 125
    assert aggregate['valid_raw_pair_count'] == 125 and aggregate['block_count'] == 25
    assert aggregate['success']['pstmo'] == 121 and aggregate['success']['constrained'] == 122
    assert len(aggregate['failures']) == 7 and len(tables[22])-1 == 9
    assert historical['record_count'] == 125 and historical['success_count'] == 123
    assert not aggregate['audit_issues'] and not historical['issues']
    report = {'status':'passed','operation':'stored-evidence comparison and arithmetic/unit formatting only','experimental_runs':0,'statistical_tests_executed':0,'docx_sha256':sha(OUT/'PSTMO_JOURNAL_VER12_FULL_MANUSCRIPT.docx'),'aggregate_sha256':sha(COHORT/'analysis/aggregate.json'),'manifest_sha256':sha(COHORT/'study_manifest.json'),'cohort_commit':manifest['git_commit'],'source_fingerprints':fingerprints,'code_and_config_commit_comparisons':matched_code,'historical_manifest_sha256':sha(ROOT/'docs/warehouse_cross_aisles_5_routes/audit_manifest.json'),'historical_source_hashes_checked':historical_sources,'historical_interpretation':'125-trial report mixes historical R01 with later R02–R05. Source hashes in its audit manifest describe provenance, not proof that the current configuration governed all historical trials. No raw-trial reanalysis was performed.','numeric_table_checks':checks,'success':aggregate['success'],'main_matrix':{'trials':125,'success':123},'repeatability':{'trials':250,'pairs':125,'same_raw_pairs':125,'joint_success_pairs':118,'blocks':25},'failures_preserved':{'main_matrix':2,'repeatability':7},'statistical_interpretation':'The aggregate key independent_block_count is an implementation label. Blocks share map, route and planner; manuscript does not assert full statistical independence. Holm applies to three primary p-values, not all reported confidence intervals.'}
    (OUT/'evidence_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print('PASS: 9 stored mean/CI rows, 3 Holm p-values, cohort counts, 7 source fingerprints, 13 commit comparisons; no statistical reanalysis.')


if __name__ == '__main__':
    main()
