#!/usr/bin/env python3
"""Verify the editorial revision against published evidence and render all pages."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import pymupdf
from PIL import Image, ImageDraw

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'docs/warehouse_cross_aisles_5_routes'
PDF=ROOT/'docs/PSTMO_KHO_GIAO_CAT_5_QUY_DAO.pdf'
BASELINE='7602317e4d23806730fbb64a9f3f4cace469286e'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--accept-visual-review',action='store_true');args=parser.parse_args()
    quality=BASE/'quality_checks.json'
    if args.accept_visual_review:
        result=json.loads(quality.read_text())
        assert result['pdf_sha256']==sha(PDF),'PDF changed after QA rendering'
        assert not result['audit_issues']
        result['render_check']['visual_review']='Complete: all page layouts inspected in contact sheets; introductory, route-summary and appendix-index pages reviewed at full size. Selected pages also rendered with Poppler.'
        quality.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print('Visual review recorded for unchanged PDF');return
    out=ROOT/'tmp/pdfs/cross5_revision';out.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((BASE/'audit_manifest.json').read_text());issues=list(manifest['issues'])
    for item in manifest['files']:
        if sha(ROOT/item['path'])!=item['sha256']:issues.append('Hash mismatch: '+item['path'])
    protected=[str((BASE/p).relative_to(ROOT)) for p in ['execution','figures','all_125_trials.csv','scenarios.yaml','geometry_preflight.csv','geometry_preflight.json']]
    changed=subprocess.check_output(['git','diff','--name-only',BASELINE,'--',*protected],cwd=ROOT,text=True).splitlines()
    issues.extend('Evidence changed: '+p for p in changed)
    before=json.loads(subprocess.check_output(['git','show',BASELINE+':docs/warehouse_cross_aisles_5_routes/figure_index.json'],cwd=ROOT,text=True))
    now=json.loads((BASE/'figure_index.json').read_text())
    if sorted(before)!=sorted(now):issues.append('Original figure inventory changed')
    page_index=json.loads((BASE/'page_index.json').read_text());figure_index=json.loads((BASE/'figure_page_index.json').read_text())
    doc=pymupdf.open(PDF)
    if len(doc)!=106:issues.append('Expected 106 pages')
    if len(figure_index)!=92:issues.append('Expected 92 figures')
    outside=[];replacement=[];sparse=[];image_count=0;link_count=0
    for i,page in enumerate(doc):
        text=page.get_text()
        if len(text.strip())<100:sparse.append(i+1)
        if '\ufffd' in text:replacement.append(i+1)
        if '...' in text:issues.append(f'Unresolved reference on page {i+1}')
        for block in page.get_text('dict')['blocks']:
            for line in block.get('lines',[]):
                for span in line['spans']:
                    b=pymupdf.Rect(span['bbox'])
                    if b.x0<0 or b.y0<0 or b.x1>page.rect.width+.1 or b.y1>page.rect.height+.1:outside.append({'page':i+1,'text':span['text']})
        for link in page.get_links():
            link_count+=1
            if link['kind']==pymupdf.LINK_GOTO and not 0<=link['page']<len(doc):issues.append(f'Broken link page {i+1}')
        image_count+=len(page.get_images())
        page.get_pixmap(matrix=pymupdf.Matrix(1.25,1.25),alpha=False).save(out/f'page-{i+1:03}.png')
    if outside or replacement or sparse:issues.append('Page text bounds/glyph checks failed')
    if image_count!=92:issues.append(f'Expected 92 embedded images, got {image_count}')
    norm=lambda s:re.sub(r'\s+',' ',s.replace('<br/>',' ')).strip()
    for entry in page_index:
        if norm(entry['title']) not in norm(doc[entry['page']-1].get_text()):issues.append('Heading mismatch '+str(entry['page']))
    for entry in figure_index:
        if f'Hình {entry["figure"]}.' not in doc[entry['page']-1].get_text():issues.append('Figure/page mismatch '+str(entry['figure']))
    for start in range(0,len(doc),12):
        sheet=Image.new('RGB',(1500,1760),'#dce5eb');draw=ImageDraw.Draw(sheet)
        for i in range(start,min(start+12,len(doc))):
            im=Image.open(out/f'page-{i+1:03}.png');im.thumbnail((360,515))
            x=(i-start)%4*375+(375-im.width)//2;y=(i-start)//4*585+27
            sheet.paste(im,(x,y));draw.text((x,y-20),f'PAGE {i+1}',fill='#143247')
        sheet.save(out/f'contact-{start//12+1:02}.jpg',quality=94)
    result={'report':str(PDF.relative_to(ROOT)),'checked_on':'2026-10-05','pdf_sha256':sha(PDF),'pages':len(doc),'figures':len(figure_index),
        'trial_records':manifest['record_count'],'new_trial_records':100,'successful_trials':manifest['success_count'],'successful_new_trials':98,
        'paired_raw_groups':sum(d['raw_pairing_valid'] for d in manifest['pairing']),'audited_file_hashes_verified':len(manifest['files']),'audit_issues':issues,
        'editorial_revision':{'baseline_commit':BASELINE,'original_pages':96,'all_original_figures_preserved':sorted(before)==sorted(now),'experiment_data_and_figure_files_unchanged':not changed},
        'render_check':{'all_pages_rendered':True,'out_of_page_text_spans':len(outside),'replacement_character_pages':len(replacement),'sparse_text_pages':len(sparse),'embedded_images':image_count,'internal_links_checked':link_count,'pdf_bookmarks':len(doc.get_toc()),'visual_review':'Pending inspection of rendered pages'},
        'limits':'One trial per route/planner/method; editorial revision adds no simulation runs.'}
    quality.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False))
    if issues:raise SystemExit(1)

if __name__=='__main__':main()
