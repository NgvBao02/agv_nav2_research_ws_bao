#!/usr/bin/env python3
"""Read-only artifact checks, with an explicit human visual-review record."""
import argparse
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
OUT=ROOT/'Viet_bao/journal/ver3'
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
    pdf=render_dir/(docx.stem+'.pdf');d=pymupdf.open(pdf)
    assert len(d)==15, f'Unexpected page count: {len(d)}'
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
        'counts':manifest['counts'],'font_names':sorted(font_names),'bounds_issues':bounds_issues,
        'headings':headings,'citation_first_appearance_order':cite_order,
        'math_glyph_rendered_ink_checks':math_bbox_checks,
        'visual_review_all_pages':bool(accept),'reviewed_date':'2026-10-13' if accept else None,
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
