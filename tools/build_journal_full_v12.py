#!/usr/bin/env python3
"""Scientific revision from the author's CURRENT ver11 package, not old builders.

Only word/document.xml is changed. No simulator, benchmark or statistical calls.
Use --init once, then build; --check optionally takes --render-dir after visual QA.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re
import subprocess
from zipfile import ZipFile

from lxml import etree as ET
from build_journal_sections_v10 import paragraph, sha, dump, W, NS

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'Viet_bao/journal/ver12'
OLD = ROOT / 'Viet_bao/journal/ver11'
SOURCE = OLD / 'PSTMO_JOURNAL_VER11_FULL_MANUSCRIPT.docx'
FINAL = OUT / 'PSTMO_JOURNAL_VER12_FULL_MANUSCRIPT.docx'
EXPECTED = 'debafc4f83a9b7a4dfd52230908cb37fac42ccfb5c23bebea45be996ce556fbc'
M = 'http://schemas.openxmlformats.org/officeDocument/2006/math'
TOKEN = re.compile(r'\[(R[A-Z0-9]+)\]')
NUM = re.compile(r'\[(\d+)\]')
COHORT = ROOT / 'results/pstmo_journal_repeatability_20261011'


def txt(e):
    return ''.join(e.xpath('.//w:t/text()', namespaces=NS))


def document(path):
    with ZipFile(path) as z:
        return ET.fromstring(z.read('word/document.xml'))


def replace_text(e, pattern, callback):
    """Replace text spans even across Word runs; preserve unrelated run formatting."""
    nodes = e.findall('.//w:t', NS)
    spans = []
    pos = 0
    for t in nodes:
        length = len(t.text or '')
        spans.append((pos, pos + length, t))
        pos += length
    for match in reversed(list(pattern.finditer(''.join(t.text or '' for t in nodes)))):
        hits = [(a, b, t) for a, b, t in spans if a < match.end() and b > match.start()]
        assert hits
        for i, (a, b, t) in enumerate(hits):
            left, right = max(0, match.start()-a), min(b-a, match.end()-a)
            t.text = (t.text or '')[:left] + (callback(match) if i == 0 else '') + (t.text or '')[right:]


def old_maps():
    refs = json.loads((OLD/'citation_map.json').read_text())
    return {r['number']: r['id'] for r in refs}


def key_text(e, mapping):
    return NUM.sub(lambda m: '['+mapping[int(m[1])]+']', txt(e))


def init():
    assert sha(SOURCE) == EXPECTED
    assert not (OUT/'baseline.json').exists(), 'Initialization is once only'
    removed = json.loads((OUT/'reference_decisions.json').read_text())
    refs = json.loads((OLD/'references.json').read_text())
    kept = [deepcopy(r) for r in refs if r['id'] not in removed]
    for r in kept:
        if r['id'] in ('R010', 'R031', 'R043'):
            r['ver12_read_scope'] = {'R010':'Toàn văn 6 trang; xem ảnh trang PDF 2–5, đặc biệt Mục 2.2, 3, 4.1 và 5.', 'R031':'Toàn văn 10 trang; xem ảnh trang PDF 2–4 để kiểm tra định nghĩa và nội suy G².', 'R043':'Toàn văn 6 trang, gồm mô hình, giới hạn không gian, khả thi và thực nghiệm robot.'}[r['id']]
        elif r['id'] == 'RCHOI':
            r['ver12_read_scope'] = 'Trang nhà xuất bản: Abstract, Introduction, Mục 2/2.1/2.2 (đoạn định nghĩa trạng thái và giai đoạn), Mục 3 và Summary. Không tuyên bố đã tải toàn bộ PDF.'
        elif r['id'] == 'RNAV':
            r['citation'] = r['citation'].replace('Oct. 22, 2026', 'Oct. 23, 2026')
            r['ver12_read_scope'] = 'Jazzy README và toàn bộ smoother_cost_function.hpp, truy cập 2026-10-23; không suy cấu hình cohort từ tài liệu Rolling.'
            r['ver12_implementation_url'] = 'https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_constrained_smoother/include/nav2_constrained_smoother/smoother_cost_function.hpp'
        else:
            r['ver12_read_scope'] = 'Kế thừa hồ sơ nguồn và phạm vi trang/đoạn đã xác minh trong ver11; không tuyên bố đọc lại toàn văn ở ver12.'
    dump(OUT/'references.json', kept)
    paths = list(OLD.iterdir()) + [ROOT/'REFERENCES/REFERENCE_MANIFEST.csv', ROOT/'REFERENCES/TAI_LIEU_THAM_KHAO_VER2.md']
    paths += [ROOT/r['path'] for r in refs if r.get('path')]
    paths += list((ROOT/'src/adaptive_pivot_g2/src').glob('*.cpp'))
    paths += list((ROOT/'src/adaptive_pivot_g2_nav2/src').glob('*.cpp'))
    paths += [ROOT/'src/vacuum_robot_gazebo/config/nav2_params.yaml', COHORT/'study_manifest.json', COHORT/'analysis/aggregate.json', COHORT/'README.md', ROOT/'docs/warehouse_cross_aisles_5_routes/audit_manifest.json', ROOT/'docs/warehouse_cross_aisles_5_routes/scenarios.yaml']
    dump(OUT/'baseline.json', {'author_edited_ver11_sha256': EXPECTED, 'legacy_generated_sha256':json.loads((OLD/'provenance.json').read_text())['output_sha256'], 'git_status_at_start':'?? .vscode/', 'git_head':subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip(), 'protected_files':{str(p.relative_to(ROOT)):sha(p) for p in sorted(set(paths)) if p.is_file()}})
    print('Initialized', len(kept), 'references')


def build(replace=False):
    assert sha(SOURCE) == EXPECTED
    if FINAL.exists():
        assert replace, 'Output exists; use --check or --replace-generated'
        assert sha(FINAL) == json.loads((OUT/'provenance.json').read_text())['output_sha256'], 'Manual ver12 edits detected'
    with ZipFile(SOURCE) as z:
        infos = z.infolist()
        parts = {i.filename:z.read(i.filename) for i in infos}
    doc = ET.fromstring(parts['word/document.xml'])
    body = doc.find('w:body', NS)
    original = list(body)
    assert txt(original[31]).startswith('II.') and txt(original[73]).startswith('III.')
    assert txt(original[348]) == 'IV KẾT LUẬN' and txt(original[353]) == 'TÀI LIỆU THAM KHẢO'
    templates = {'BodyText':original[4], 'Heading1':original[6], 'Heading2':original[9], 'ScientificReference':original[354]}

    def p(text, style='BodyText'):
        el = paragraph(text, style)
        el.replace(el.find('w:pPr', NS), deepcopy(templates[style].find('w:pPr', NS)))
        # New explanatory prose uses the same native subscript as inherited E_kappa.
        for run in list(el.findall('w:r', NS)):
            t = run.find('w:t', NS)
            if t is None or 'Eκ' not in (t.text or ''):
                continue
            index = el.index(run)
            replacement = []
            for piece in re.split('(Eκ)', t.text):
                if not piece:
                    continue
                for value, subscript in ([('E',False),('κ',True)] if piece == 'Eκ' else [(piece,None)]):
                    new_run = deepcopy(run)
                    new_run.find('w:t', NS).text = value
                    if subscript is not None:
                        rp = new_run.find('w:rPr', NS)
                        if rp is None:
                            rp = ET.Element(f'{{{W}}}rPr')
                            new_run.insert(0,rp)
                        ET.SubElement(rp,f'{{{W}}}i')
                        if subscript:
                            ET.SubElement(rp,f'{{{W}}}vertAlign').set(f'{{{W}}}val','subscript')
                    replacement.append(new_run)
            el.remove(run)
            for offset,new_run in enumerate(replacement):
                el.insert(index+offset,new_run)
        return el

    def blocks(md):
        for block in md.split('\n\n'):
            b = block.strip()
            if not b or b.startswith(('# ', '@subtitle', '@BODY')):
                continue
            if b.startswith('### '):
                yield p(b[4:], 'Heading2')
            elif b.startswith('## '):
                yield p(b[3:], 'Heading1')
            elif b.startswith('@keywords '):
                yield p('Từ khóa — '+b[10:])
            else:
                yield p(b)

    mapping = old_maps()
    retained = deepcopy(original[31:348])
    for e in retained:
        replace_text(e, NUM, lambda m:'['+mapping[int(m[1])]+']')
    additions = []
    for match in re.finditer(r'@before ([^\n]+)\n\n(.*?)(?=\n\n@before |\Z)', (OUT/'insertions.md').read_text(), re.S):
        anchor, md = match.groups()
        hits = [e for e in retained if txt(e).startswith(anchor)]
        assert len(hits) == 1, (anchor, len(hits))
        idx = retained.index(hits[0])
        added = list(blocks(md))
        retained[idx:idx] = added
        additions.append({'anchor':anchor, 'added':[txt(e) for e in added]})
    front, conclusion = (OUT/'front_conclusion.md').read_text().split('@BODY')
    for e in list(body):
        body.remove(e)
    for e in deepcopy(original[:3]) + list(blocks(front)) + list(blocks((OUT/'introduction.md').read_text())) + retained + list(blocks(conclusion)):
        body.append(e)
    refs = json.loads((OUT/'references.json').read_text())
    by_id = {r['id']:r for r in refs}
    used, locations = [], {}
    heading = 'Tóm tắt'
    for idx, e in enumerate(body):
        style = e.find('w:pPr/w:pStyle', NS)
        if style is not None and style.get(f'{{{W}}}val','').startswith('Heading'):
            heading = txt(e)
        for key in TOKEN.findall(txt(e)):
            if key not in used:
                used.append(key)
            locations.setdefault(key, []).append({'heading':heading, 'body_index':idx, 'paragraph':txt(e)})
    assert set(used) == set(by_id), (set(by_id)-set(used), set(used)-set(by_id))
    numbers = {key:i+1 for i,key in enumerate(used)}
    for e in body:
        replace_text(e, TOKEN, lambda m:f'[{numbers[m[1]]}]')
    body.append(deepcopy(original[353]))
    for key in used:
        body.append(p(f'[{numbers[key]}] '+by_id[key]['citation'], 'ScientificReference'))
    body.append(deepcopy(original[-1]))
    parts['word/document.xml'] = ET.tostring(doc, xml_declaration=True, encoding='UTF-8', standalone=True)
    with ZipFile(FINAL, 'w') as z:
        for info in infos:
            z.writestr(info, parts[info.filename])
    dump(OUT/'citation_map.json', [{'id':key,'number':numbers[key], **{k:v for k,v in by_id[key].items() if k!='id'}, 'locations':locations[key]} for key in used])
    dump(OUT/'revision_log.json', additions)
    source_paths = [SOURCE, OLD/'citation_map.json', OUT/'front_conclusion.md', OUT/'introduction.md', OUT/'insertions.md', OUT/'references.json', OUT/'reference_decisions.json', OUT/'baseline.json', Path(__file__), ROOT/'tools/build_journal_sections_v10.py']
    dump(OUT/'provenance.json', {'source_docx_sha256':EXPECTED, 'output_sha256':sha(FINAL), 'source_files':[{'path':str(path.relative_to(ROOT)), 'sha256':sha(path)} for path in source_paths], 'edited_zip_parts':['word/document.xml'], 'language_decision':'Vietnamese summary and keywords only; English title retained by author decision', 'simulations_run':False, 'statistical_reanalysis':False, 'new_experimental_results':False})
    lines = ['# Văn bản trích xuất từ Word ver12', '', 'Đây là bản đọc, không phải nguồn tái tạo công thức/hình. Dựng từ Word ver11 đã khóa hash cùng Markdown/JSON ver12.', '']
    for e in body:
        if e.tag == f'{{{W}}}tbl':
            for row in e.findall('w:tr', NS):
                lines.append('| '+' | '.join(txt(c) for c in row.findall('w:tc', NS))+' |')
        elif e.findall('.//w:drawing', NS):
            lines.append('[Hình giữ nguyên trong Word]')
        elif e.findall(f'.//{{{M}}}oMath'):
            lines.append(('[Office Math giữ nguyên trong Word] '+txt(e)).rstrip())
        elif txt(e):
            lines.append(txt(e))
        lines.append('')
    (OUT/'manuscript_numbered.md').write_text('\n'.join(lines)+'\n')
    rows = ['# Quyết định giữ và bỏ tài liệu tham khảo', '', '54 nguồn có vị trí dẫn thực tế; 26 nguồn chỉ ở Introduction cũ được bỏ theo lý do riêng. Không xóa tệp nào trong REFERENCES. Phạm vi đọc được ghi riêng trong references.json, không coi kiểm tra hash là đọc toàn văn.', '', '| Khóa | Số mới | Vị trí hỗ trợ | Nguồn |', '|---|---:|---|---|']
    for key in used:
        rows.append('| '+' | '.join([key,str(numbers[key]),'; '.join(dict.fromkeys(x['heading'] for x in locations[key])),by_id[key]['citation'].replace('|','/')])+' |')
    rows += ['', '## Các nguồn bỏ khỏi danh mục bài báo', '', '| Khóa | Lý do |', '|---|---|']
    for key, reason in json.loads((OUT/'reference_decisions.json').read_text()).items():
        rows.append(f'| {key} | {reason} |')
    (OUT/'REFERENCE_COVERAGE.md').write_text('\n'.join(rows)+'\n')
    check()
    print(FINAL)


def check(render_dir=None):
    prov = json.loads((OUT/'provenance.json').read_text())
    assert sha(FINAL) == prov['output_sha256']
    for item in prov['source_files']:
        assert sha(ROOT/item['path']) == item['sha256'], item['path']
    baseline = json.loads((OUT/'baseline.json').read_text())
    for path, digest in baseline['protected_files'].items():
        assert sha(ROOT/path) == digest, 'Protected file changed: '+path
    a, b = document(SOURCE), document(FINAL)
    counts = {}
    for label, xpath, expected in [('tables','.//w:tbl',23),('cells','.//w:tc',696),('drawings','.//w:drawing',18),('math',f'.//{{{M}}}oMath',12)]:
        aa, bb = a.findall(xpath,NS), b.findall(xpath,NS)
        assert len(aa) == len(bb) == expected, label
        assert [ET.tostring(e) for e in aa] == [ET.tostring(e) for e in bb], label+' XML changed'
        counts[label] = len(bb)
    assert [ET.tostring(e) for e in a.findall('.//w:sectPr',NS)] == [ET.tostring(e) for e in b.findall('.//w:sectPr',NS)]
    with ZipFile(SOURCE) as old, ZipFile(FINAL) as new:
        assert old.namelist() == new.namelist()
        for name in old.namelist():
            if name != 'word/document.xml':
                assert old.read(name) == new.read(name), name
    oldbody, newbody = list(a.find('w:body',NS)), list(b.find('w:body',NS))
    assert [ET.tostring(e) for e in oldbody[:3]] == [ET.tostring(e) for e in newbody[:3]]
    refs = json.loads((OUT/'citation_map.json').read_text())
    newmap = {r['number']:r['id'] for r in refs}
    start = next(i for i,e in enumerate(newbody) if txt(e).startswith('II.'))
    stop = next(i for i,e in enumerate(newbody) if txt(e)=='IV KẾT LUẬN')
    new_retained = [key_text(e,newmap) for e in newbody[start:stop]]
    added = [text for item in json.loads((OUT/'revision_log.json').read_text()) for text in item['added']]
    for text in added:
        assert new_retained.count(text) == 1
        new_retained.remove(text)
    assert new_retained == [key_text(e,old_maps()) for e in oldbody[31:348]], 'II/III changed beyond additions/citations'
    bib = next(i for i,e in enumerate(newbody) if txt(e)=='TÀI LIỆU THAM KHẢO')
    text = '\n'.join(txt(e) for e in newbody[:bib])
    assert not TOKEN.search(text)
    assert list(dict.fromkeys(int(n) for n in NUM.findall(text))) == list(range(1,len(refs)+1))
    assert len(refs) == 54 and len(newbody[bib+1:-1]) == 54
    assert [txt(e) for e in newbody if e.findall(f'.//{{{M}}}oMath') and txt(e)] == [f'({i})' for i in range(1,10)]
    assert [txt(e) for e in newbody if txt(e).startswith('Hình ')] == [txt(e) for e in oldbody if txt(e).startswith('Hình ')]
    assert not any(txt(e).strip().upper() in ('ABSTRACT','KEYWORDS') or txt(e).startswith('Keywords —') for e in newbody)
    assert sum(txt(e)=='TÓM TẮT' for e in newbody) == 1
    for r in refs:
        if r.get('path'):
            assert sha(ROOT/r['path']) == r['sha256']
    result = {'status':'structural_pass_visual_pending','docx_sha256':sha(FINAL),**counts,'numbered_equations':9,'references':54,'removed_references':26,'english_abstract':False,'vietnamese_summary':True,'title_block_xml_preserved':True,'section_geometry_xml_preserved':True,'table_math_drawing_xml_preserved':True,'all_other_zip_parts_preserved':True,'section_II_original_prose_preserved':True,'section_II_added_paragraphs':len(added),'section_III_preserved_except_citation_numbers':True,'protected_files_unchanged':len(baseline['protected_files']),'citation_order_contiguous_first_occurrence':True,'simulations_run':False,'statistical_reanalysis':False}
    if render_dir:
        import fitz
        rd = Path(render_dir)
        pdf = rd/(FINAL.stem+'.pdf')
        rendered = fitz.open(pdf)
        pngs = sorted(rd.glob('page-*.png'),key=lambda p:int(p.stem.split('-')[-1]))
        review = json.loads((OUT/'visual_review.json').read_text())
        assert review['docx_sha256'] == sha(FINAL) and review['pdf_sha256'] == sha(pdf)
        assert review['page_png_sha256'] == [sha(p) for p in pngs]
        assert len(pngs) == len(rendered)
        assert [p['page'] for p in review['pages']] == list(range(1,len(rendered)+1))
        assert all(p['accepted'] and p['notes'] for p in review['pages'])
        fonts = sorted({font[3] for page in rendered for font in page.get_fonts()})
        assert any('TimesNewRoman' in font for font in fonts), fonts
        assert all(len(page.get_text().strip()) > 100 for page in rendered), 'Blank/nearly blank page'
        result.update(status='passed',rendered_pages=len(rendered),fonts=fonts,visual_review_sha256=sha(OUT/'visual_review.json'),pdf_sha256=sha(pdf))
    dump(OUT/'quality_checks.json',result)
    print(json.dumps(result,ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--init',action='store_true')
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--replace-generated',action='store_true')
    parser.add_argument('--render-dir')
    args = parser.parse_args()
    if args.init:
        init()
    elif args.check:
        check(args.render_dir)
    else:
        build(args.replace_generated)
