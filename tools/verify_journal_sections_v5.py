#!/usr/bin/env python3
"""Read-only artifact checks, with an explicit human visual-review record."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import re
import statistics
from zipfile import ZipFile

from lxml import etree
from PIL import Image
import pymupdf

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'Viet_bao/journal/ver5'
NS={'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main',
    'm':'http://schemas.openxmlformats.org/officeDocument/2006/math',
    'wp':'http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing'}


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def check(render_dir, accept):
    manifest=json.loads((OUT/'provenance.json').read_text())
    docx=OUT/manifest['docx']
    assert digest(docx)==manifest['docx_sha256'], 'DOCX changed after build'
    for name,sha in manifest['source_sha256'].items():
        assert digest(ROOT/name)==sha, 'Source changed: '+name
    with ZipFile(docx) as z:
        root=etree.fromstring(z.read('word/document.xml'))
        styles=etree.fromstring(z.read('word/styles.xml'))
    tables=root.findall('.//w:tbl',NS)
    assert len(tables)==20
    assert len(root.findall('.//wp:inline',NS))==19
    assert len(root.findall('.//m:oMath',NS))==12
    for s in root.findall('.//w:sectPr',NS):
        p=s.find('w:pgSz',NS);m=s.find('w:pgMar',NS)
        assert abs(int(p.get('{'+NS['w']+'}w'))/20-595.3)<.1
        assert abs(int(p.get('{'+NS['w']+'}h'))/20-841.9)<.1
        for k,v in [('top',62.35),('bottom',62.35),('left',45.35),('right',45.35)]:
            assert abs(int(m.get('{'+NS['w']+'}'+k))/20-v)<.1
    body=styles.find('w:style[@w:styleId="BodyText"]',NS)
    assert body.find('w:rPr/w:rFonts',NS).get('{'+NS['w']+'}ascii')=='Times New Roman'
    assert body.find('w:rPr/w:sz',NS).get('{'+NS['w']+'}val')=='20'
    source=(OUT/'manuscript.md').read_text()
    before_refs=source.split('# TÀI LIỆU THAM KHẢO')[0]
    cite_order=list(dict.fromkeys(int(x) for x in re.findall(r'\[(\d+)\]',before_refs)))
    assert cite_order==list(range(1,8)),cite_order
    assert source.count('@figure ')==19 and source.count('@table ')==20
    for name,fig in manifest['figures'].items():
        im=Image.open(OUT/'figures'/f'{name}.png')
        assert im.width/fig['width_in']>=449
    with (ROOT/'results/pstmo_journal_repeatability_20261011/analysis/block_differences.csv').open() as f:
        blocks=list(csv.DictReader(f))
    aggregate=json.loads((ROOT/'results/pstmo_journal_repeatability_20261011/analysis/aggregate.json').read_text())
    for key,a in aggregate['metrics'].items():
        vals=[float(r['pstmo_minus_constrained']) for r in blocks if r['metric']==key]
        assert len(vals)==25
        assert abs(statistics.mean(vals)-a['mean_block_difference_pstmo_minus_constrained'])<1e-10
    # Regression against the untouched previous edition: equations and every
    # numerical table cell must remain identical after display normalization.
    original=ROOT/'Viet_bao/journal/ver3'
    old_source=(original/'manuscript.md').read_text()
    assert re.findall(r'^@equation (.+)$',source,re.M)==re.findall(r'^@equation (.+)$',old_source,re.M)
    with ZipFile(original/'PSTMO_JOURNAL_VER3_PHAN_II_III.docx') as z:
        old_root=etree.fromstring(z.read('word/document.xml'))
    def table_rows(table):
        return [[ ''.join(c.xpath('.//w:t/text()',namespaces=NS))
                  for c in row.findall('w:tc',NS)] for row in table.findall('w:tr',NS)]
    def canonical_number(s):
        s=s.strip().replace('−','-')
        if re.fullmatch(r'[0-9]+,[0-9]+ × 10⁻[⁰¹²³⁴⁵⁶⁷⁸⁹]+',s):
            mantissa, exponent=s.split(' × 10⁻')
            exponent=exponent.translate(str.maketrans('⁰¹²³⁴⁵⁶⁷⁸⁹','0123456789'))
            return float(mantissa.replace(',','.'))*10**(-int(exponent))
        if re.fullmatch(r'[+-]?[0-9]+[.,]?[0-9]*(e-[0-9]+)?',s):
            return float(s.replace(',','.'))
        return None
    checked_cells=0
    for old,new in zip(old_root.findall('.//w:tbl',NS),tables,strict=True):
        old_rows,new_rows=table_rows(old),table_rows(new)
        assert len(old_rows)==len(new_rows)
        for a,b in zip(old_rows,new_rows,strict=True):
            for x,y in zip(a,b,strict=True):
                nx,ny=canonical_number(x),canonical_number(y)
                if nx is not None:
                    assert ny is not None and abs(nx-ny)<=max(abs(nx),1)*1e-12,(x,y)
                    checked_cells+=1
                elif '[' in x and ';' in x:
                    assert x.replace('.',',')==y.replace('.',','),(x,y)
                    checked_cells+=1
    # Thousands separators in textual parameter descriptions must not change.
    assert '200.000' in ''.join(root.xpath('.//w:t/text()',namespaces=NS))
    assert '1.000' in ''.join(root.xpath('.//w:t/text()',namespaces=NS))
    preserved={
        'Viet_bao/journal/ver4/PSTMO_JOURNAL_VER4_PHAN_II_III.docx':'ab04f962109b12457151338de8e606e1fd9d8799590c9d9bb5d4dbb83f3538ef',
        'Viet_bao/journal/ver4/manuscript.md':'9b5e6f756431b90e355511ca64a1f75aa12c0875e775808a5a99320b08f58435',
        'Viet_bao/journal/ver3/PSTMO_JOURNAL_VER3_PHAN_II_III.docx':'1339162f089a8353c8c4f05154fcb23d757b408f992aa0b2cc2dbec1959f6691',
        'Viet_bao/journal/ver3/manuscript.md':'a8aeebe838e37812f47d54e3b448393b4fa8730641496386ba120d4cf8079ea8',
        'Viet_bao/bao_Rev_ecit_2026/ver5/REV_ECIT2026_PSTMO_VER5.docx':'789216c88d198a15198e56d45cb08e93b646d931ce6600928cf8b6c2dfcf3739'}
    for name,sha in preserved.items():
        assert digest(ROOT/name)==sha, 'Reference edition was altered: '+name
    v4_source=(ROOT/'Viet_bao/journal/ver4/manuscript.md').read_text()
    number_pattern=r'(?<![A-Za-z])[-−]?\d+[,.]\d+(?:\s*[×/]\s*\d+)?'
    assert Counter(re.findall(number_pattern,source)) == Counter(
        re.findall(number_pattern,v4_source)), 'Decimal values in prose changed'
    assert source.split('# TÀI LIỆU THAM KHẢO')[1] == v4_source.split(
        '# TÀI LIỆU THAM KHẢO')[1], 'Section references changed'
    visible=' '.join(root.xpath('.//w:t/text()',namespaces=NS))
    forbidden=['ứng viên','năng lượng độ cong','vùng lạm phát','hạt khởi tạo',
               'biên lợi thế','khối giảm','hiệu ứng phụ','đi qua vật cản',
               'planned_curvature_energy','executed_curvature_energy']
    for term in forbidden:
        assert term not in visible.lower(), 'Unrevised term: '+term
    assert not re.search(r'quỹ đạo thực(?! thi)',visible,re.I)
    assert not re.search(r'(?:trong|theo)\s*\(\d+\)',visible,re.I)
    assert 'trọng số khoảng cách và độ cong đều bằng 0' in visible
    assert 'tích phân bình phương độ cong' in visible.lower()
    with ZipFile(ROOT/'Viet_bao/journal/ver4/PSTMO_JOURNAL_VER4_PHAN_II_III.docx') as z:
        retained_root=etree.fromstring(z.read('word/document.xml'))
    old_coordinates=table_rows(retained_root.findall('.//w:tbl',NS)[2])[1:]
    new_coordinates=table_rows(tables[2])[1:]
    for old_row,new_row in zip(old_coordinates,new_coordinates,strict=True):
        for old_cell,new_cell in zip(old_row[1:],new_row[1:],strict=True):
            assert [float(v) for v in old_cell.split(', ')] == [
                float(v.replace(',','.')) for v in new_cell.split('; ')]
    assert [etree.tostring(e) for e in root.findall('.//m:oMath',NS)] == [
        etree.tostring(e) for e in retained_root.findall('.//m:oMath',NS)], 'Equation XML changed'
    with (ROOT/'docs/warehouse_cross_aisles_5_routes/all_125_trials.csv').open(encoding='utf-8-sig') as f:
        trials=list(csv.DictReader(f))
    expected_time=[[5,4,5,5],[4,2,4,4],[4,3,4,4],[4,2,5,4],[4,3,5,4]]
    expected_rmse=[[3,4,4,3],[2,0,1,2],[2,1,2,3],[3,4,5,5],[2,5,3,4]]
    comparison_audit=[]
    for i,route in enumerate(['R01','R02','R03','R04','R05']):
        pstmo={r['planner']:r for r in trials if r['route_id']==route and r['method']=='pstmo'}
        for j,method in enumerate(['raw','simple','savitzky_golay','constrained']):
            base={r['planner']:r for r in trials if r['route_id']==route and r['method']==method}
            pairs=[(r,base[k]) for k,r in pstmo.items()
                   if r['success']=='True' and base[k]['success']=='True']
            assert all(a['raw_path_sha256']==b['raw_path_sha256'] for a,b in pairs)
            t=sum(float(a['execution_time_s'])<float(b['execution_time_s']) for a,b in pairs)
            e=sum(float(a['tracking_rmse_m'])<float(b['tracking_rmse_m']) for a,b in pairs)
            assert t==expected_time[i][j] and e==expected_rmse[i][j],(route,method,t,e)
            comparison_audit.append({'route':route,'baseline':method,'both_success':len(pairs),
                                     'pstmo_lower_time':t,'pstmo_lower_rmse':e})
    pdf=render_dir/(docx.stem+'.pdf');d=pymupdf.open(pdf)
    assert 14 <= len(d) <= 15, f'Unexpected page count: {len(d)}'
    page_hashes=[];bounds_issues=[];font_names=set();headings={};math_bbox_checks=[]
    for i,page in enumerate(d,1):
        png=render_dir/f'page-{i}.png';assert png.exists()
        page_hashes.append(digest(png))
        text=page.get_text()
        assert '¿' not in text and '\ufffd' not in text,f'Math/glyph error on page {i}'
        for word in ['III. XÂY DỰNG','TÀI LIỆU THAM KHẢO CHO PHẦN II VÀ III']:
            hit=page.search_for(word)
            if hit:headings[word]={'page':i,'rect':list(hit[0])}
        for block in page.get_text('rawdict')['blocks']:
            if block['type']!=0:continue
            for line in block['lines']:
                for span in line['spans']:
                    chars=[c for c in span['chars'] if c['c'].strip()]
                    if not chars:continue
                    font_names.add(span['font'])
                    x0=min(c['bbox'][0] for c in chars);x1=max(c['bbox'][2] for c in chars)
                    y0=min(c['bbox'][1] for c in chars);y1=max(c['bbox'][3] for c in chars)
                    if x0<43 or x1>552 or y0<60 or y1>782:
                        # Stretchy radicals have oversized font-metric boxes.
                        # Test rendered ink at 4x instead of exempting the glyph.
                        if ''.join(c['c'] for c in chars)=='√':
                            pix=page.get_pixmap(matrix=pymupdf.Matrix(4,4),
                                clip=pymupdf.Rect(x0,y0,x1,y1),colorspace=pymupdf.csGRAY)
                            ink=[(j%pix.width,j//pix.width)
                                 for j,v in enumerate(pix.samples) if v<160]
                            assert ink, 'Empty radical rendering'
                            box=[(pix.x+min(x for x,y in ink))/4,
                                 (pix.y+min(y for x,y in ink))/4,
                                 (pix.x+max(x for x,y in ink))/4,
                                 (pix.y+max(y for x,y in ink))/4]
                            math_bbox_checks.append({'page':i,'glyph':'√',
                                'font_metric_bbox':[x0,y0,x1,y1],'rendered_ink_bbox':box})
                            if box[0]>=43 and box[2]<=552 and box[1]>=60 and box[3]<=782:
                                continue
                        bounds_issues.append([i,''.join(c['c'] for c in chars),[x0,y0,x1,y1]])
    assert not bounds_issues, f'{len(bounds_issues)} out-of-bounds spans: {bounds_issues[:5]}'
    assert any('TimesNewRoman' in f for f in font_names),font_names
    report={'docx_sha256':digest(docx),'pdf_sha256':digest(pdf),'page_count':len(d),
        'page_png_sha256':page_hashes,'source_hash_count':len(manifest['source_sha256']),
        'counts':manifest['counts'],'numeric_table_cells_checked':checked_cells,
        'equations_unchanged':True,'reference_editions_unchanged':True,
        'terminology_and_equation_reference_checks':True,
        'prose_decimal_values_and_references_unchanged':True,
        'coordinate_values_unchanged':True,
        'main_pairwise_comparisons':comparison_audit,
        'font_names':sorted(font_names),'bounds_issues':bounds_issues,
        'headings':headings,'citation_first_appearance_order':cite_order,
        'math_glyph_rendered_ink_checks':math_bbox_checks,
        'visual_review_all_pages':bool(accept),'reviewed_date':'2026-10-17' if accept else None,
        'notes':['QA PDF and PNG are temporary, not deliverables.',
                 'Seven local section references; renumber when merging the complete manuscript.',
                 'IEEE conference A4 retained; target journal template still requires confirmation.']}
    # A generated verification artifact, not an input dataset.
    (OUT/'quality_checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ['page_count','counts','source_hash_count','bounds_issues','visual_review_all_pages']},ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--render-dir',type=Path,required=True)
    p.add_argument('--accept-visual-review',action='store_true',help='Only after viewing every page PNG of this exact PDF.')
    a=p.parse_args();check(a.render_dir,a.accept_visual_review)
