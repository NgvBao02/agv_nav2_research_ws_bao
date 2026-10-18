#!/usr/bin/env python3
"""Verify journal v6 against source records and the exact rendered artifact."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
from statistics import mean
from zipfile import ZipFile

from lxml import etree
from PIL import Image
import pymupdf

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'Viet_bao/journal/ver6'
NS = {'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main',
      'm':'http://schemas.openxmlformats.org/officeDocument/2006/math',
      'wp':'http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing'}
METHODS = ['raw','simple','savitzky_golay','constrained','pstmo']


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_csv(path):
    with path.open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def xml(path):
    with ZipFile(path) as z:
        return etree.fromstring(z.read('word/document.xml'))


def check(render_dir, accept):
    p = json.loads((OUT/'provenance.json').read_text())
    docx = OUT/p['docx']
    assert digest(docx) == p['docx_sha256']
    for name, sha in p['source_sha256'].items():
        assert digest(ROOT/name) == sha, name
    assert digest(ROOT/'Viet_bao/journal/ver5/PSTMO_JOURNAL_VER5_PHAN_II_III.docx') == 'b6d36586fe58faa27c3a34ec64aabcdf768305b5147aeef9ef48514a4532b562'
    assert digest(ROOT/'Viet_bao/journal/ver5/manuscript.md') == '97e9bed179b39a8887c0c7e60a633ba0c4f95a9aba21033cff78f8c7eabefcc6'
    root = xml(docx)
    old = xml(ROOT/'Viet_bao/journal/ver5/PSTMO_JOURNAL_VER5_PHAN_II_III.docx')
    assert [etree.tostring(e) for e in root.findall('.//m:oMath',NS)] == [etree.tostring(e) for e in old.findall('.//m:oMath',NS)]
    assert len(root.findall('.//m:oMath',NS)) == 12
    assert len(root.findall('.//wp:inline',NS)) == 18
    tables = root.findall('.//w:tbl',NS)
    assert len(tables) == 23
    for a,b in zip(root.findall('.//w:sectPr',NS),old.findall('.//w:sectPr',NS),strict=True):
        for tag in ['pgSz','pgMar','cols']:
            assert etree.tostring(a.find('w:'+tag,NS)) == etree.tostring(b.find('w:'+tag,NS))
    source = (OUT/'manuscript.md').read_text()
    previous = (ROOT/'Viet_bao/journal/ver5/manuscript.md').read_text()
    assert source.split('# III.')[0] == previous.split('# III.')[0], 'Section II changed'
    assert source.split('# TÀI LIỆU THAM KHẢO')[1] == previous.split('# TÀI LIỆU THAM KHẢO')[1]
    cite_order = list(dict.fromkeys(int(x) for x in re.findall(r'\[(\d+)\]', source.split('# TÀI LIỆU THAM KHẢO')[0])))
    assert cite_order == list(range(1,8))
    visible = ' '.join(root.xpath('.//w:t/text()',namespaces=NS))
    for term in ['ứng viên','năng lượng độ cong','planned_curvature_energy','executed_curvature_energy','ground_truth_state_trace']:
        assert term not in visible.lower(), term
    assert not re.search(r'(?:trong|theo)\s*\(\d+\)',visible,re.I)
    assert not re.search(r'[a-f0-9]{40,64}',visible)
    assert not re.search(r'\{(?:table|figure):',visible)
    # Each formatted Word table must agree with its machine-readable values.
    import build_journal_sections_v5 as renderer
    expected = json.loads((OUT/'table_values.json').read_text())
    checked_cells = 0
    for key, table in zip(p['table_order'],tables,strict=True):
        actual = [[''.join(c.xpath('.//w:t/text()',namespaces=NS)) for c in row.findall('w:tc',NS)] for row in table.findall('w:tr',NS)]
        head, rows, widths = expected[key]
        wanted = [[renderer.display_cell(c) for c in row] for row in [head]+rows]
        assert actual == wanted, key
        checked_cells += sum(len(row) for row in actual)
    old_keys=re.findall(r'^@table (\S+)',previous,re.M)
    old_tables=dict(zip(old_keys,old.findall('.//w:tbl',NS),strict=True))
    unchanged_tables=[]
    def table_text(t):
        return [[''.join(c.xpath('.//w:t/text()',namespaces=NS)) for c in r.findall('w:tc',NS)]
                for r in t.findall('w:tr',NS)]
    for key,table in zip(p['table_order'],tables,strict=True):
        if key in old_tables and key not in ['baselines','failures']:
            assert table_text(table)==table_text(old_tables[key]),key
            unchanged_tables.append(key)
    # Independently reconstruct all newly added numerical tables from the
    # original CSV and original JSON, not from the exported v6 analysis.
    records = read_csv(ROOT/'docs/warehouse_cross_aisles_5_routes/all_125_trials.csv')
    for r in records:
        r['data'] = json.loads((ROOT/r['source']).read_text())
    def vals(route, method, key, success=False):
        return [float(r['data'][key]) for r in records if r['method']==method and
                (route is None or r['route_id']==route) and (not success or r['success']=='True')]
    def fmt(value, digits):
        return f'{value:.{digits}f}'.replace('.',',')
    tests = 0
    for i in range(1,6):
        for method,row in zip(METHODS,expected[f'r{i:02}_extra'][1],strict=True):
            for cell,(key,scale,nd,succ) in zip(row[1:],[
                ('planned_path_length_m',1,3,False),('planned_max_abs_curvature_1pm',1,2,False),
                ('traveled_distance_m',1,3,True),('tracking_max_error_m',1000,1,True),
                ('smoothing_time_s',1000,1,False)],strict=True):
                assert cell==fmt(mean(vals(f'R{i:02}',method,key,succ))*scale,nd),(i,method,key)
                tests += 1
    for method,row in zip(METHODS,expected['execution_extra'][1],strict=True):
        for cell,(key,scale,nd) in zip(row[1:],[('executed_curvature_energy_1pm',1,3),
            ('actual_p95_abs_jerk_mps3',1,3),('final_position_error_m',1000,1),('final_yaw_error_rad',1,3)],strict=True):
            assert cell==fmt(mean(vals(None,method,key,True))*scale,nd)
            tests += 1
    for method,row in zip(METHODS,expected['dynamics'][1],strict=True):
        for cell,key in zip(row[1:],['actual_max_angular_radps','actual_max_wheel_linear_mps',
            'actual_max_abs_lateral_acceleration_mps2','actual_max_abs_acceleration_mps2',
            'actual_max_abs_angular_acceleration_radps2'],strict=True):
            assert cell == fmt(max(vals(None,method,key)),3)
            tests += 1
    for method,row in zip(METHODS,expected['translation'][1],strict=True):
        assert row[1:]==[fmt(mean(vals(None,method,'planned_translation_max_abs_curvature_1pm')),2),
            fmt(mean(vals(None,method,'planned_translation_curvature_energy_1pm')),2),
            fmt(mean(vals(None,method,'planned_point_count')),1),
            str(int(sum(vals(None,method,'planned_pivot_marker_count'))))]
        tests += 4
    for method,row in zip(METHODS,expected['safety'][1],strict=True):
        assert row[1:]==['—' if method=='raw' else fmt(mean(vals(None,method,'smoothing_time_s'))*1000,2),
            f"{int(sum(vals(None,method,'physically_settled')))}/25",
            str(int(sum(vals(None,method,'collision_monitor_interventions')))),
            str(int(sum(vals(None,method,'planned_footprint_collision_sample_count'))))]
        tests += 4
    keyed = {(r['route'],r['planner'],r['method']):r for r in records}
    comparisons = read_csv(OUT/'analysis/pairwise_comparisons.csv')
    for c in comparisons:
        differences=[]
        for r in records:
            if r['method']!='pstmo' or c['route'] not in ('ALL',r['route']):
                continue
            base=keyed[r['route'],r['planner'],c['baseline']]
            if c['success_only']=='True' and not (r['success']==base['success']=='True'):
                continue
            assert r['raw_path_sha256']==base['raw_path_sha256']
            def value(rec):
                key=c['metric']
                return float(rec[key] if key in rec else rec['data'][key])
            differences.append(value(r)-value(base))
        assert len(differences)==int(c['n'])
        assert math.isclose(mean(differences),float(c['mean_delta']),rel_tol=1e-12,abs_tol=1e-12)
        assert sum(v < -1e-12 for v in differences)==int(c['pstmo_lower'])
    for method,row in zip(METHODS[:-1],expected['all_pairs'][1],strict=True):
        cs={c['metric']:c for c in comparisons if c['route']=='ALL' and c['baseline']==method}
        assert row[1]==cs['execution_time_s']['n']
        for cell,key,scale in zip(row[2:],['execution_time_s','tracking_rmse_m','executed_curvature_energy_1pm'],[1,1000,1],strict=True):
            assert cell==fmt(float(cs[key]['mean_delta'])*scale,3)
            tests += 1
    blocks=read_csv(ROOT/'results/pstmo_journal_repeatability_20261011/analysis/block_differences.csv')
    aggregate=json.loads((ROOT/'results/pstmo_journal_repeatability_20261011/analysis/aggregate.json').read_text())
    for metric,a in aggregate['metrics'].items():
        values=[float(r['pstmo_minus_constrained']) for r in blocks if r['metric']==metric]
        assert len(values)==25 and math.isclose(mean(values),a['mean_block_difference_pstmo_minus_constrained'],abs_tol=1e-10)
    for name, figure in p['figures'].items():
        im=Image.open(OUT/'figures'/f'{name}.png')
        assert im.width/figure['width_in']>=449
    pdf=render_dir/(docx.stem+'.pdf')
    d=pymupdf.open(pdf)
    assert 14 <= len(d) <= 16, len(d)
    fonts=set();bounds=[];headings={};page_hashes=[];radicals=[]
    for i,page in enumerate(d,1):
        page_hashes.append(digest(render_dir/f'page-{i}.png'))
        assert '\ufffd' not in page.get_text() and '¿' not in page.get_text()
        for word in ['III. XÂY DỰNG','TÀI LIỆU THAM KHẢO CHO PHẦN II VÀ III']:
            hits=page.search_for(word)
            if hits:headings[word]={'page':i,'rect':list(hits[0])}
        for block in page.get_text('rawdict')['blocks']:
            if block['type']!=0:continue
            for line in block['lines']:
                for span in line['spans']:
                    chars=[c for c in span['chars'] if c['c'].strip()]
                    if not chars:continue
                    fonts.add(span['font'])
                    box=[min(c['bbox'][0] for c in chars),min(c['bbox'][1] for c in chars),
                         max(c['bbox'][2] for c in chars),max(c['bbox'][3] for c in chars)]
                    if box[0]<43 or box[2]>552 or box[1]<60 or box[3]>782:
                        if ''.join(c['c'] for c in chars)=='√':
                            pix=page.get_pixmap(matrix=pymupdf.Matrix(4,4),clip=pymupdf.Rect(box),colorspace=pymupdf.csGRAY)
                            ink=[(j%pix.width,j//pix.width) for j,v in enumerate(pix.samples) if v<160]
                            assert ink
                            box=[(pix.x+min(x for x,y in ink))/4,(pix.y+min(y for x,y in ink))/4,
                                 (pix.x+max(x for x,y in ink))/4,(pix.y+max(y for x,y in ink))/4]
                            radicals.append({'page':i,'ink_bbox':box})
                            if box[0]>=43 and box[2]<=552 and box[1]>=60 and box[3]<=782:continue
                        bounds.append([i,''.join(c['c'] for c in chars),box])
    assert not bounds, bounds[:10]
    assert any('TimesNewRoman' in f for f in fonts),fonts
    report={'docx_sha256':digest(docx),'pdf_sha256':digest(pdf),'page_count':len(d),
        'page_png_sha256':page_hashes,'source_hash_count':len(p['source_sha256']),
        'counts':p['counts'],'table_cells_checked':checked_cells,'new_numeric_cells_recomputed':tests,
        'pairwise_statistics_recomputed':len(comparisons),'section_II_unchanged':True,
        'preserved_tables_unchanged':unchanged_tables,
        'equation_XML_unchanged':True,'ver5_unchanged':True,'references_unchanged':True,
        'font_names':sorted(fonts),'bounds_issues':bounds,'headings':headings,
        'math_radical_ink_checks':radicals,'data_audit':p['data_audit'],
        'visual_review_all_pages':bool(accept),'reviewed_date':'2026-10-18' if accept else None,
        'notes':['No simulation launched; no source dataset overwritten.',
                 'Local section reference numbering must be merged into the full article.',
                 'Inherited IEEE conference A4 layout; journal-specific template not confirmed.']}
    (OUT/'quality_checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ['page_count','table_cells_checked','new_numeric_cells_recomputed',
        'pairwise_statistics_recomputed','bounds_issues','visual_review_all_pages']},ensure_ascii=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--render-dir',type=Path,required=True)
    parser.add_argument('--accept-visual-review',action='store_true')
    args=parser.parse_args();check(args.render_dir,args.accept_visual_review)
