#!/usr/bin/env python3
"""Read-only structural/render checks for the theory report; writes QA metadata."""
from pathlib import Path
from zipfile import ZipFile
import hashlib
import json
import re
import sys
from lxml import etree as E
import pdfplumber

ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT / 'docs/theory_report'
OUT = ROOT / 'docs/CO_SO_LY_THUYET_TOAN_DIEN_DU_AN_AGV_PSTMO.docx'
REF = ROOT / 'docs/PSTMO.docx'
NS = {'w': 'http://schemas.openxmlformats.org/wordprocessingml/2006/main',
      'm': 'http://schemas.openxmlformats.org/officeDocument/2006/math',
      'a': 'http://schemas.openxmlformats.org/drawingml/2006/main'}
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def verify(render_dir):
    allowed = {'word/document.xml', 'word/_rels/document.xml.rels',
               'word/settings.xml', 'docProps/core.xml', 'docProps/app.xml', 'word/styles.xml'}
    with ZipFile(REF) as a, ZipFile(OUT) as b:
        modified = [n for n in a.namelist() if n not in allowed and a.read(n) != b.read(n)]
        assert not modified, modified
        old = E.fromstring(a.read('word/document.xml'))
        doc = E.fromstring(b.read('word/document.xml'))
        assert [E.tostring(x, method='c14n') for x in old.findall('.//w:sectPr', NS)] == [E.tostring(x, method='c14n') for x in doc.findall('.//w:sectPr', NS)]
        for name in ['word/styles.xml']:
            sa, sb = E.fromstring(a.read(name)), E.fromstring(b.read(name))
            attr = '{'+NS['w']+'}styleId'
            for s in sa.findall('w:style', NS):
                sid = s.get(attr)
                if sid == 'Title': continue
                target = sb.find(f"w:style[@w:styleId='{sid}']", NS)
                assert E.tostring(s, method='c14n') == E.tostring(target, method='c14n'), sid
        oldrels = E.fromstring(a.read('word/_rels/document.xml.rels'))
        newrels = E.fromstring(b.read('word/_rels/document.xml.rels'))
        nr = {r.get('Id'): dict(r.attrib) for r in newrels}
        assert all(nr[r.get('Id')] == dict(r.attrib) for r in oldrels)
        for r in newrels:
            if r.get('Type', '').endswith('/image') and r.get('TargetMode') != 'External':
                assert 'word/'+r.get('Target') in b.namelist(), r.get('Target')
        equations = len(doc.findall('.//m:oMath', NS))
        for tag, required in [('sSub', ['e','sub']), ('sSup', ['e','sup']),
                              ('sSubSup', ['e','sub','sup']), ('f', ['num','den'])]:
            for node in doc.findall('.//m:'+tag, NS):
                for child in required:
                    part=node.find('m:'+child, NS)
                    assert part is not None and len(part), (tag,child)
        images = len(doc.findall('.//a:blip', NS))
        assert (equations, images) == (60,45), (equations,images)
        bookmarks = {x.get('{'+NS['w']+'}name') for x in doc.findall('.//w:bookmarkStart', NS)}
        links = doc.findall('.//w:hyperlink', NS)
        assert len(links) == 19
        assert all(x.get('{'+NS['w']+'}anchor') in bookmarks for x in links)
        text = ''.join(doc.itertext())
        assert '[frac:' not in text and ':codex' not in text
    manifest = json.loads((ASSET/'source_manifest.json').read_text())
    source_mismatch = [x['path'] for x in manifest['files'] if digest(ROOT/x['path']) != x['sha256']]
    assert not source_mismatch, source_mismatch
    with pdfplumber.open(render_dir/(OUT.stem+'.pdf')) as pdf:
        violations=[]; missing=[]; captions=[]; words=0; footers=[]; blank=[]
        for num,page in enumerate(pdf.pages,1):
            chars=page.chars
            violations.extend((num,c['text']) for c in chars if c['x0'] < 45 or c['x1'] > 552)
            missing.extend((num,c['text']) for c in chars if '\ufffd' in c['text'])
            words+=len(page.extract_words())
            fulltext=page.extract_text() or ''
            captions.extend((int(x),num) for x in re.findall(r'Hình\s+(\d+)\.',fulltext))
            footers.append(''.join(c['text'] for c in chars if c['top'] > 800).strip())
            if not any(c['top'] < 780 for c in chars) and not page.images: blank.append(num)
        assert not violations, violations
        assert not missing, missing
        assert not blank, blank
        assert sorted(x[0] for x in captions) == list(range(1,46)), captions
        assert footers == [str(n) for n in range(1,len(pdf.pages)+1)], footers
        pages=len(pdf.pages)
    assert len(list(render_dir.glob('page-*.png'))) == pages
    summary = {'date': '2026-10-06', 'docx_sha256': digest(OUT),
               'template_sha256': digest(REF), 'pages': pages, 'render_word_count': words,
               'figures': images, 'native_equations': equations, 'linked_toc_entries': len(links),
               'preserve_only_parts_changed': modified, 'preserved_relationships': True,
               'preserved_styles_except_title': True, 'section_geometry_preserved': True,
               'source_snapshot_recheck': 'pass', 'horizontal_overflow': violations,
               'replacement_glyphs': missing, 'empty_pages': blank,
               'consecutive_page_numbers': True, 'figure_caption_pages': captions,
               'render_engine': 'Bundled LibreOffice export, 110 dpi PNG',
               'desktop_word_render_tested': False,
               'visual_review': 'Pending separate inspection of every final page'}
    (ASSET/'qa_summary.json').write_text(json.dumps(summary, ensure_ascii=False,indent=2))
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__ == '__main__':
    verify(Path(sys.argv[1]).resolve())
