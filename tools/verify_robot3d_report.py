#!/usr/bin/env python3
"""Render all PDF pages with Poppler, check text bounds and evidence completeness."""
import json,subprocess,hashlib,math
from pathlib import Path
import pymupdf
from PIL import Image,ImageOps,ImageDraw
ROOT=Path(__file__).resolve().parents[1];A=ROOT/'docs/robot_3d_report';QA=A/'qa';PAGES=QA/'pages'
PAGES.mkdir(parents=True,exist_ok=True)
pdf=ROOT/'docs/BAO_CAO_MO_HINH_3D_STEP_URDF_GAZEBO_RVIZ2.pdf'
doc=pymupdf.open(pdf);idx=json.loads((A/'figure_page_index.json').read_text());issues=[]
assert len(doc)==len(idx['pages'])
for i,p in enumerate(doc):
    text=p.get_text()
    if '\ufffd' in text:issues.append(dict(page=i+1,issue='replacement character'))
    if len(text)<80:issues.append(dict(page=i+1,issue='unusually little text'))
    for b in p.get_text('dict')['blocks']:
        if 'lines' not in b:continue
        for line in b['lines']:
            for s in line['spans']:
                x0,y0,x1,y1=s['bbox']
                if x0<30 or x1>p.rect.width-30 or y0<12 or y1>p.rect.height-12:
                    issues.append(dict(page=i+1,issue='text bounds',text=s['text'],bbox=s['bbox']))
for f in idx['figures']:assert (ROOT/f['path']).exists(),f
metrics=json.loads((A/'motion/summary.json').read_text());assert len(metrics)==7 and all(x['n']==3 for x in metrics)
assert json.loads((A/'warehouse_demo.json').read_text())['status']==4
assert json.loads((A/'provenance.json').read_text())['test_exit_code']==0
subprocess.run(['pdftoppm','-r','80','-png',str(pdf),str(PAGES/'page')],check=True)
files=sorted(PAGES.glob('page-*.png'));assert len(files)==len(doc)
for offset in range(0,len(files),20):
    sheet=Image.new('RGB',(5*230,4*340),'#d4dce1');draw=ImageDraw.Draw(sheet)
    for k,file in enumerate(files[offset:offset+20]):
        im=Image.open(file).convert('RGB');im.thumbnail((216,309));x=(k%5)*230+7;y=(k//5)*340+22
        sheet.paste(im,(x,y));draw.text((x,y-17),f'Page {offset+k+1}',fill='black')
    sheet.save(QA/f'contact_{offset//20+1}.png')
for page in [1,2,3,13,14,16,20,21,24,25,37,40,41,42,54,58,60,61,65,69]:
    if page<=len(doc):doc[page-1].get_pixmap(matrix=pymupdf.Matrix(1.65,1.65)).save(QA/f'detail_{page:02}.png')
result=dict(pages=len(doc),figures=len(idx['figures']),pdf_bytes=pdf.stat().st_size,pdf_sha256=hashlib.sha256(pdf.read_bytes()).hexdigest(),issues=issues,rendered_pages=len(files),trials=21,warehouse_status=4)
(QA/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,ensure_ascii=False))
assert not issues,issues
