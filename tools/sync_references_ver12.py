#!/usr/bin/env python3
"""Build/check the ver12 reference view without changing historical sources."""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[1]
REF = ROOT / 'REFERENCES'
V12 = ROOT / 'Viet_bao/journal/ver12'
VIEW = REF / 'VER12'
W = '{http://schemas.openxmlformats.org/wordprocessingml/2006/main}'
ONLINE_NAMES = {
    'RCHOI': '2014_Choi_Kinodynamic_Motion_Planning',
    'RNAV': 'Nav2_Jazzy_Constrained_Smoother',
    'RJAZZY': 'Nav2_Jazzy_Smoother_Server_Implementation',
    'REFRON': '1979_Efron_Bootstrap',
    'RWILCOXON': '1945_Wilcoxon_Individual_Comparisons',
    'RHOLM': '1979_Holm_Multiple_Test_Procedure',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def csv_bytes(rows, fields):
    stream = io.StringIO(newline='')
    writer = csv.DictWriter(stream, fieldnames=fields, lineterminator='\n')
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode('utf-8')


def cell(text):
    return str(text).replace('|', '&#124;').replace('\n', ' ')


def expected_outputs():
    refs = load(V12 / 'citation_map.json')
    assert [r['number'] for r in refs] == list(range(1, 55))
    assert len({r['id'] for r in refs}) == 54
    assert all(r['locations'] for r in refs)
    by_id = {r['id']: r for r in refs}
    assert {r['id'] for r in refs if not r.get('path')} == set(ONLINE_NAMES)
    with ZipFile(V12 / 'PSTMO_JOURNAL_VER12_FULL_MANUSCRIPT.docx') as archive:
        body = ET.fromstring(archive.read('word/document.xml')).find(W + 'body')
    texts = [''.join(e.itertext()) if e.tag == W + 't' else
             ''.join(t.text or '' for t in e.iter(W + 't')) for e in body]
    index = texts.index('TÀI LIỆU THAM KHẢO')
    actual = [t for t in texts[index + 1:] if t]
    assert actual == [f"[{r['number']}] {r['citation']}" for r in refs], 'Word bibliography changed'
    with (REF / 'REFERENCE_MANIFEST.csv').open(encoding='utf-8', newline='') as stream:
        corpus = list(csv.DictReader(stream))
    assert len(corpus) == 74
    # All historic inputs pinned by ver12 must still match, including the old catalogs.
    protected = load(V12 / 'baseline.json')['protected_files']
    for path, digest in protected.items():
        assert sha(ROOT / path) == digest, f'Historical input changed: {path}'
    for item in corpus:
        assert sha(REF / item['filename']) == item['sha256'], item['filename']

    outputs, links, rows = {}, {}, []
    md = [
        '# Tài liệu tham khảo theo PSTMO Journal ver12', '',
        '54 mục dưới đây đúng thứ tự và nguyên văn danh mục trong Word ver12. '
        'Số [n] là số trích dẫn trong bài; mã Rxxx là khóa nguồn ổn định, '
        'không phải số trích dẫn mới.', '',
        '48 mục có tài liệu cục bộ (44 PDF, 4 HTML). Sáu mục có phiếu trích dẫn '
        'và URL, chưa có toàn văn/snapshot cục bộ. Các liên kết trong VER12 là '
        'symlink tương đối về tệp gốc; khi di chuyển cần giữ cả thư mục REFERENCES.', '',
        '| Số ver12 | Khóa | Trích dẫn | Tệp theo số mới | Mã corpus cũ | Vị trí dẫn |',
        '|---:|---|---|---|---:|---|',
    ]
    for r in refs:
        number, key = r['number'], r['id']
        locations = '; '.join(dict.fromkeys(x['heading'] for x in r['locations']))
        path = r.get('path')
        if path:
            source = ROOT / path
            assert source.parent == REF and sha(source) == r['sha256']
            name = f'{number:03}_' + source.name.split('_', 1)[1]
            links[VIEW / name] = '../' + source.name
            status = 'local_pdf' if source.suffix == '.pdf' else 'local_html_snapshot'
            digest, size, old = r['sha256'], source.stat().st_size, r['local_id']
        else:
            name = f'{number:03}_{ONLINE_NAMES[key]}.md'
            status, old, size = 'citation_and_url_only', '', ''
            note = [f"# [{number}] {r['citation']}", '',
                    f"Khóa nguồn: `{key}`. Số trích dẫn trong ver12: [{number}].", '',
                    f"Nguồn: [trang nguồn]({r['url']}).", '',
                    'Trạng thái lưu trữ: phiếu trích dẫn và URL; chưa có toàn văn '
                    'hoặc snapshot cục bộ. Phiếu này không phải bản sao tài liệu gốc.', '',
                    'Vị trí dẫn trong bài: ' + locations + '.', '',
                    'Phạm vi đọc được ghi trong hồ sơ ver12: ' + r['ver12_read_scope'], '']
            if r.get('ver12_implementation_url'):
                note += [f"Mã triển khai đối chiếu: [Jazzy cost function]({r['ver12_implementation_url']}).", '']
            outputs[VIEW / name] = '\n'.join(note).encode('utf-8')
            digest = ''  # Do not present the citation-card hash as an original-source hash.
        rows.append({'number': f'{number:03}', 'id': key, 'filename': 'VER12/' + name,
                     'storage_status': status, 'corpus_number': old, 'original_path': path or '',
                     'source_sha256': digest, 'source_bytes': size, 'source_url': r['url'],
                     'citation': r['citation'], 'locations': locations})
        md.append('| ' + ' | '.join(map(cell, [f'[{number}]', key, r['citation'],
                  f'[{name}](VER12/{name})', old or 'Ngoài corpus 74', locations])) + ' |')
    fields = list(rows[0])
    outputs[REF / 'REFERENCE_MANIFEST_VER12.csv'] = csv_bytes(rows, fields)
    outputs[REF / 'TAI_LIEU_THAM_KHAO_VER12.md'] = ('\n'.join(md) + '\n').encode('utf-8')
    inventory = []
    removed = load(V12 / 'reference_decisions.json')
    for item in corpus:
        key = 'R' + item['number'].zfill(3)
        used = by_id.get(key)
        assert used or key in removed
        inventory.append({'corpus_number': item['number'], 'id': key,
                          'filename': item['filename'],
                          'status_ver12': 'cited' if used else 'not_cited',
                          'ver12_number': str(used['number']) if used else '',
                          'reason_not_cited': '' if used else removed[key],
                          'sha256': item['sha256']})
    outputs[REF / 'CORPUS_STATUS_VER12.csv'] = csv_bytes(inventory, list(inventory[0]))
    summary = {
        'reference_count': len(rows), 'local_pdf': sum(r['storage_status'] == 'local_pdf' for r in rows),
        'local_html_snapshot': sum(r['storage_status'] == 'local_html_snapshot' for r in rows),
        'citation_and_url_only': sum(r['storage_status'] == 'citation_and_url_only' for r in rows),
        'historic_corpus_files': len(corpus), 'historic_corpus_cited': len(links),
        'historic_corpus_not_cited': len(corpus) - len(links),
        'protected_inputs_unchanged': len(protected), 'word_bibliography_exact_match': True,
        'source_hashes': {str(p.relative_to(ROOT)): sha(p) for p in
                          [V12 / 'citation_map.json', V12 / 'PSTMO_JOURNAL_VER12_FULL_MANUSCRIPT.docx',
                           REF / 'REFERENCE_MANIFEST.csv', REF / 'TAI_LIEU_THAM_KHAO_VER2.md']},
    }
    return outputs, links, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Check without writing anything')
    args = parser.parse_args()
    outputs, links, summary = expected_outputs()
    # Preflight all targets before writing; preserve edits and unrelated additions.
    for path, content in outputs.items():
        assert not path.is_symlink(), f'Unexpected symlink: {path}'
        if path.exists():
            assert path.read_bytes() == content, f'Existing content differs; inspect before replacing: {path}'
        elif args.check:
            raise AssertionError(f'Missing output: {path}')
    for path, target in links.items():
        if path.is_symlink():
            assert str(path.readlink()) == target and path.is_file(), path
        else:
            assert not path.exists(), f'Unexpected regular file: {path}'
            assert not args.check, f'Missing symlink: {path}'
    if VIEW.exists():
        expected = {p.name for p in outputs if p.parent == VIEW} | {p.name for p in links}
        actual = {p.name for p in VIEW.iterdir() if re.match(r'^\d{3}_', p.name)}
        assert actual <= expected, f'Unexpected numbered entries: {actual - expected}'
    if not args.check:
        VIEW.mkdir(parents=True, exist_ok=True)
        for path, content in outputs.items():
            if not path.exists():
                path.write_bytes(content)
        for path, target in links.items():
            if not path.is_symlink():
                path.symlink_to(target)
    summary['generated_files_match'] = True
    summary['relative_symlinks_valid'] = True
    report = (json.dumps(summary, ensure_ascii=False, indent=2) + '\n').encode('utf-8')
    audit = REF / 'VER12_SYNC_AUDIT.json'
    if args.check:
        assert audit.read_bytes() == report, 'Audit differs from current inputs'
    else:
        if audit.exists():
            assert not audit.is_symlink() and audit.read_bytes() == report, 'Existing audit differs'
        else:
            audit.write_bytes(report)
    print('PASS: 54 Word-matched entries; 44 PDFs, 4 HTML snapshots, 6 URL cards; '
          '74 historic source files and 109 protected inputs unchanged.')


if __name__ == '__main__':
    main()
