#!/usr/bin/env python3
"""Merge preserved ver9 scientific objects and ver10 sections with audited references.

Editorial inputs: introduction.md, insertions.md, front_conclusion.md, references.json.
Run with Documents runtime Python. No historical artifact or simulator writes.
"""
from __future__ import annotations
import argparse
from copy import deepcopy
import csv
import hashlib
import json
from pathlib import Path
import re
from zipfile import ZipFile
from lxml import etree as ET
from build_journal_sections_v10 import paragraph, node, sha, dump, W, NS

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'Viet_bao/journal/ver11'
V9 = ROOT / 'Viet_bao/journal/ver9/PSTMO_JOURNAL_VER9_PHAN_II_III.docx'
V10 = ROOT / 'Viet_bao/journal/ver10/PSTMO_JOURNAL_VER10_ABSTRACT_INTRO_CONCLUSION.docx'
FINAL = OUT / 'PSTMO_JOURNAL_VER11_FULL_MANUSCRIPT.docx'
HASH9 = '4f5743cc02139a23da557da46ee84704f7d5228bb371b55e7b0a19b0b78a53dc'
# The retained source hash is deliberately explicit; do not silently accept new teacher edits.
HASH10 = '6c61300fa01e012b326a945bac4f51fd7dda5ab2f7af5900e79c2823750395b4'
OLD_REFS = {1:'R061', 2:'R060', 3:'RNAV', 4:'RJAZZY', 5:'REFRON', 6:'RWILCOXON', 7:'RHOLM'}
TOKEN = re.compile(r'\[(R[A-Z0-9]+)\]')
M = 'http://schemas.openxmlformats.org/officeDocument/2006/math'

def txt(e):
    return ''.join(e.itertext()) if e.tag.endswith('}t') else ''.join(e.xpath('.//w:t/text()', namespaces=NS))

def load_doc(path):
    with ZipFile(path) as z:
        return ET.fromstring(z.read('word/document.xml'))

def init_sources():
    """Mechanical import, once only. Subsequent editorial corrections live in JSON."""
    assert not (OUT/'references.json').exists()
    old = json.loads((V10.parent/'references.json').read_text())
    by_id = {r['id']: r for r in old}
    manifest = list(csv.DictReader((ROOT/'REFERENCES/REFERENCE_MANIFEST.csv').open()))
    rows = {}
    for line in (ROOT/'REFERENCES/TAI_LIEU_THAM_KHAO_VER2.md').read_text().splitlines():
        if re.match(r'\| \[\d+\]', line):
            cols = [c.strip() for c in line.split('|')[1:-1]]
            rows[int(cols[0][1:-1])] = cols[1]
    refs = []
    for row in manifest:
        i = int(row['number']); key=f'R{i:03}'
        p=ROOT/'REFERENCES'/row['filename']
        assert sha(p)==row['sha256']
        raw=rows[i]
        raw=re.sub(r'; Ravankar ref\. \d+\.', '.', raw)
        raw=re.sub(r'; (đường dẫn gốc|nguồn đã liệt kê).*', '.', raw)
        r=deepcopy(by_id.get(key, {'id':key,'citation':raw,'url':row['source_url'],'pages':[1]}))
        r.update(local_id=f'{i:03}',path=str(p.relative_to(ROOT)),sha256=sha(p),
                 evidence_type='local_pdf' if row['format']=='pdf' else 'local_html_snapshot')
        if row['format']=='html':r.update(pages=[],html_section='Article body, description and parameters')
        refs.append(r)
    for key in ['RCHOI','RNAV']:refs.append(by_id[key])
    md=(V9.parent/'manuscript.md').read_text().split('# TÀI LIỆU THAM KHẢO CHO PHẦN II VÀ III')[1]
    for line in md.splitlines():
        m=re.match(r'\[(\d+)\] (.*)',line)
        if m and int(m[1])>=4:
            refs.append({'id':OLD_REFS[int(m[1])],'citation':m[2], 'local_id':None,
                         'evidence_type':'retained_verified_ver9_reference','pages':[],
                         'url':'','metadata_note':'Kế thừa nguồn phương pháp/triển khai của ver9; không chạy lại thống kê.'})
    dump(OUT/'references.json',refs)
    md=(V10.parent/'manuscript.md').read_text()
    front=md.split('## I GIỚI THIỆU')[0]
    conclusion='## IV KẾT LUẬN'+md.split('## IV KẾT LUẬN')[1]
    (OUT/'front_conclusion.md').write_text(front+'@BODY\n\n'+conclusion)

def blocks(md):
    for b in md.split('\n\n'):
        b=b.strip()
        if not b or b.startswith(('# ','@subtitle','@BODY')):continue
        if b.startswith('### '):p=paragraph(b[4:],'Heading2')
        elif b.startswith('## '):p=paragraph(b[3:],'Heading1',center=True)
        elif b.startswith('@keywords '):p=paragraph('Từ khóa — '+b[10:])
        elif b.startswith('@enkeywords '):p=paragraph('Keywords — '+b[12:])
        else:p=paragraph(b)
        yield p

def build(replace=False):
    assert sha(V9)==HASH9 and sha(V10)==HASH10
    if FINAL.exists():
        assert replace, 'Use --replace-generated only after preserving manual edits'
        assert sha(FINAL)==json.loads((OUT/'provenance.json').read_text())['output_sha256'], 'Manual Word edits detected'
    refs=json.loads((OUT/'references.json').read_text());by_id={r['id']:r for r in refs}
    with ZipFile(V9) as z:
        infos=z.infolist();parts={i.filename:z.read(i.filename) for i in infos}
    d=ET.fromstring(parts['word/document.xml']);body=d.find('w:body',NS)
    original=list(body);section=deepcopy(body.find('w:sectPr',NS))
    stop=next(i for i,e in enumerate(original) if txt(e)=='TÀI LIỆU THAM KHẢO CHO PHẦN II VÀ III')
    retained=deepcopy(original[:stop])
    for e in retained:
        for t in e.findall('.//w:t',NS):
            if t.text:t.text=re.sub(r'\[([1-7])\]',lambda m:'['+OLD_REFS[int(m[1])]+']',t.text)
    changes=[]
    ims=(OUT/'insertions.md').read_text()
    for match in re.finditer(r'@before ([^\n]+)\n\n(.*?)(?=\n\n@before |\Z)',ims,re.S):
        anchor,md=match.groups();hits=[e for e in retained if txt(e).startswith(anchor)]
        assert len(hits)==1,(anchor,len(hits))
        index=retained.index(hits[0]);new=list(blocks(md));retained[index:index]=new
        changes.append({'before':anchor,'added_paragraphs':len(new),'text':md})
    for e in list(body):body.remove(e)
    # Reuse the exact three-element title block, including the continuous section break.
    front=(OUT/'front_conclusion.md').read_text();before,after=front.split('@BODY')
    source_front=(V10.parent/'manuscript.md').read_text()
    for prefix in ['# ','@subtitle ']:
        assert next(l for l in front.splitlines() if l.startswith(prefix))==next(l for l in source_front.splitlines() if l.startswith(prefix)), 'Title is locked to ver10; update title-block generation explicitly if changing it'
    title_nodes=list(load_doc(V10).find('w:body',NS))[:3]
    for e in title_nodes:body.append(deepcopy(e))
    for e in blocks(before):body.append(e)
    for e in blocks((OUT/'introduction.md').read_text()):body.append(e)
    for e in retained:body.append(e)
    for e in blocks(after):body.append(e)
    used=[];locations={};heading='Front matter'
    for idx,e in enumerate(body):
        st=e.find('w:pPr/w:pStyle',NS)
        if st is not None and st.get(f'{{{W}}}val','').startswith('Heading'):heading=txt(e)
        for key in TOKEN.findall(txt(e)):
            if key not in used:used.append(key)
            locations.setdefault(key,[]).append({'heading':heading,'body_index':idx,'paragraph':txt(e)})
    assert set(used)==set(by_id),(set(by_id)-set(used),set(used)-set(by_id))
    numbers={k:i+1 for i,k in enumerate(used)}
    for e in body:
        for t in e.findall('.//w:t',NS):
            if t.text:t.text=TOKEN.sub(lambda m:f'[{numbers[m[1]]}]',t.text)
    body.append(paragraph('TÀI LIỆU THAM KHẢO','Heading1',center=True))
    for key in used:
        p=paragraph(f'[{numbers[key]}] '+by_id[key]['citation'],'ScientificReference')
        p.find('w:pPr',NS).extend([node('keepLines'),node('jc',val='left')]);body.append(p)
    body.append(section)
    with ZipFile(V10) as z:
        parts['word/styles.xml']=z.read('word/styles.xml')
        parts['docProps/core.xml']=z.read('docProps/core.xml')
    parts['word/document.xml']=ET.tostring(d,xml_declaration=True,encoding='UTF-8',standalone=True)
    with ZipFile(FINAL,'w') as z:
        for info in infos:z.writestr(info,parts[info.filename])
    sources=[V9,V10,V10.parent/'manuscript.md',V9.parent/'table_values.json',OUT/'front_conclusion.md',OUT/'introduction.md',OUT/'insertions.md',OUT/'references.json',Path(__file__),ROOT/'tools/build_journal_sections_v10.py',ROOT/'REFERENCES/REFERENCE_MANIFEST.csv',ROOT/'REFERENCES/TAI_LIEU_THAM_KHAO_VER2.md']
    dump(OUT/'provenance.json',{'output_sha256':sha(FINAL),'source_files':[{'path':str(p.relative_to(ROOT)),'sha256':sha(p)} for p in sources], 'simulations_run':False,'statistical_reanalysis':False,'edited_zip_parts':['word/document.xml','word/styles.xml','docProps/core.xml']})
    dump(OUT/'citation_map.json', [{'id':k,'number':numbers[k],**{a:b for a,b in by_id[k].items() if a!='id'},'locations':locations[k]} for k in used])
    dump(OUT/'revision_log.json',changes)
    # Readable manuscript extraction; math/media stay authoritative in DOCX, not reconstructed.
    lines=['# Văn bản trích xuất từ Word ver11','', 'Bảng, hình và công thức gốc nằm trong Word; đây không phải nguồn dựng.','']
    for e in body:
        if e.tag==f'{{{W}}}tbl':
            for row in e.findall('w:tr',NS):lines.append('| '+' | '.join(txt(c) for c in row.findall('w:tc',NS))+' |')
        elif e.findall('.//w:drawing',NS):lines.append('[Hình được bảo toàn trong Word]')
        elif e.findall(f'.//{{{M}}}oMath'):lines.append('[Office Math được bảo toàn trong Word] '+txt(e))
        elif txt(e):lines.append(txt(e))
        lines.append('')
    (OUT/'manuscript_numbered.md').write_text('\n'.join(lines)+'\n')
    rows=['# Đối chiếu 74 mục corpus và nguồn bổ sung','','Các vị trí dưới đây được sinh từ chính Word trước khi thay khóa bằng số; xem citation_map.json để đọc nguyên đoạn dẫn. Phạm vi đọc là các trang/đoạn có ghi, không phải đọc toàn bộ mọi tài liệu.','','| Mục corpus | Số trích dẫn | Nguồn | Vị trí | Ghi chú |','|---|---|---|---|---|']
    for r in sorted(refs,key=lambda r:r.get('local_id') or '999'+r['id']):
        loc='; '.join(dict.fromkeys(x['heading'] for x in locations[r['id']]))
        rows.append('| '+ ' | '.join([r.get('local_id') or 'Bổ sung',str(numbers[r['id']]),r['citation'].replace('|','/'),loc,r.get('metadata_note','Đối chiếu nội dung có mục tiêu.')])+' |')
    (OUT/'REFERENCE_COVERAGE.md').write_text('\n'.join(rows)+'\n')
    dump(OUT/'quality_checks.json',{'status':'pending_render_and_visual_review','docx_sha256':sha(FINAL)})
    check()
    print(FINAL)

def check(render_dir=None):
    prov=json.loads((OUT/'provenance.json').read_text());assert sha(FINAL)==prov['output_sha256']
    for p in prov['source_files']:assert sha(ROOT/p['path'])==p['sha256'],p['path']
    assert sha(V9)==HASH9 and sha(V10)==HASH10
    a,b=load_doc(V9),load_doc(FINAL)
    counts={}
    for label,xpath,expected in [('tables','.//w:tbl',23),('cells','.//w:tc',696),('figures','.//w:drawing',18),('math',f'.//{{{M}}}oMath',12)]:
        aa=a.findall(xpath,NS);bb=b.findall(xpath,NS);assert len(aa)==len(bb)==expected
        assert [ET.tostring(e) for e in aa]==[ET.tostring(e) for e in bb],label
        counts[label]=len(bb)
    with ZipFile(V9) as old,ZipFile(FINAL) as new:
        assert old.namelist()==new.namelist()
        for name in old.namelist():
            if name not in prov['edited_zip_parts']:assert old.read(name)==new.read(name),name
    refs=json.loads((OUT/'citation_map.json').read_text())
    assert len(refs)==len(set(r['id'] for r in refs))
    assert {r['local_id'] for r in refs if r.get('local_id')}=={f'{i:03}' for i in range(1,75)}
    for r in refs:
        if r.get('path'):assert sha(ROOT/r['path'])==r['sha256']
    body=b.find('w:body',NS);stop=next(i for i,e in enumerate(body) if txt(e)=='TÀI LIỆU THAM KHẢO')
    text='\n'.join(txt(e) for e in list(body)[:stop]);assert not TOKEN.search(text)
    first=list(dict.fromkeys(int(n) for n in re.findall(r'\[(\d+)\]',text)))
    assert first==list(range(1,len(refs)+1)),first
    assert len(b.findall('.//w:sectPr',NS))==2
    labels=[txt(p) for p in body if p.findall(f'.//{{{M}}}oMath') and txt(p)]
    assert labels==[f'({i})' for i in range(1,10)]
    tables=json.loads((V9.parent/'table_values.json').read_text())
    for key,row,col,value in [('primary',0,1,'-2.857'),('primary',1,1,'-2.985'),('primary',2,1,'-1.038'),('primary',0,2,'[-3.716; -2.039]'),('primary',1,2,'[-4.487; -1.506]'),('secondary',3,1,'+3.380'),('secondary',4,1,'-35.161'),('secondary',5,1,'+71.220')]:
        assert tables[key][1][row][col]==value
        for n in re.findall(r'\d+\.\d+',value):assert n.replace('.',',') in text
    for n in ['2.857','2.985','3.716','2.039','4.487','1.506']:assert n in text
    # Recover all retained scientific prose exactly, ignoring only bibliography renumbering.
    num_to_old={r['number']:next((n for n,k in OLD_REFS.items() if k==r['id']),None) for r in refs}
    normalized=[]
    for e in list(body)[:stop]:
        normalized.append(re.sub(r'\[(\d+)\]',lambda m:f'[{num_to_old[int(m[1])]}]' if num_to_old.get(int(m[1])) else m[0],txt(e)))
    oldbody=a.find('w:body',NS);oldstop=next(i for i,e in enumerate(oldbody) if txt(e)=='TÀI LIỆU THAM KHẢO CHO PHẦN II VÀ III')
    cursor=0
    for e in list(oldbody)[:oldstop]:
        t=txt(e)
        if not t:continue
        while cursor<len(normalized) and normalized[cursor]!=t:cursor+=1
        assert cursor<len(normalized),'Original prose modified: '+t[:80]
        cursor+=1
    result={'status':'structural_checks_passed_visual_pending','docx_sha256':sha(FINAL),**counts,'numbered_equations':9,'corpus_items_cited':74,'reference_count':len(refs),'all_original_scientific_prose_preserved':True,'all_table_math_drawing_xml_preserved':True,'all_other_zip_parts_preserved':True,'sequential_first_occurrence_citations':True}
    if render_dir:
        import fitz
        rd=Path(render_dir);pdf=rd/(FINAL.stem+'.pdf');doc=fitz.open(pdf)
        pngs=sorted(rd.glob('page-*.png'),key=lambda p:int(p.stem.split('-')[-1]));assert len(pngs)==len(doc)
        review=json.loads((OUT/'visual_review.json').read_text())
        assert review['docx_sha256']==sha(FINAL) and review['pdf_sha256']==sha(pdf)
        assert review['page_png_sha256']==[sha(p) for p in pngs]
        assert [p['page'] for p in review['pages']]==list(range(1,len(doc)+1))
        assert all(p['accepted'] and p['notes'] for p in review['pages'])
        fonts=sorted({f[3] for p in doc for f in p.get_fonts()})
        # Native OMML and Unicode indices in ver9 use renderer math/glyph fallback.
        # Do not alter preserved scientific objects just to force a font inventory.
        permitted=('TimesNewRoman','LiberationSerif','NotoSerif','OpenSymbol','NimbusRoman','DejaVuSerif')
        assert all(any(name in f for name in permitted) for f in fonts),fonts
        result.update(status='passed',rendered_pages=len(doc),fonts=fonts,visual_review_sha256=sha(OUT/'visual_review.json'))
    dump(OUT/'quality_checks.json',result);print(json.dumps(result,ensure_ascii=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--init',action='store_true');p.add_argument('--check',action='store_true');p.add_argument('--replace-generated',action='store_true');p.add_argument('--render-dir')
    args=p.parse_args()
    if args.init:init_sources()
    elif args.check:check(args.render_dir)
    else:build(args.replace_generated)
