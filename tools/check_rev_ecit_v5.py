#!/usr/bin/env python3
"""Validate the generated ECIT v5 package after the required visual review."""
import argparse
import hashlib
import json
import re
from pathlib import Path
from zipfile import ZipFile

import pdfplumber
from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'Viet_bao/bao_Rev_ecit_2026/ver5'
NS = {'w': 'http://schemas.openxmlformats.org/wordprocessingml/2006/main',
      'm': 'http://schemas.openxmlformats.org/officeDocument/2006/math',
      'wp': 'http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pdf', type=Path, required=True)
    parser.add_argument('--visual-reviewed', action='store_true')
    args = parser.parse_args()
    docx = OUT / 'REV_ECIT2026_PSTMO_VER5.docx'
    template = OUT.parent / 'ecit_word_template.docx'
    audit = json.loads((OUT / 'data_audit.json').read_text())
    assert (audit['trial_count'], audit['success_count'], audit['paired_group_count']) == (175, 170, 34)
    assert len(audit['all_baseline_comparisons']) == 4
    assert len(audit['failure_details']) == 5
    provenance = json.loads((OUT / 'provenance.json').read_text())
    for name, expected in provenance['sources'].items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == expected, name
    with ZipFile(docx) as package, ZipFile(template) as original:
        xml = etree.fromstring(package.read('word/document.xml'))
        text = ' '.join(xml.xpath('//w:t/text() | //m:t/text()', namespaces=NS))
        assert len(xml.xpath('//wp:inline', namespaces=NS)) == 7
        assert len(xml.xpath('//w:tbl', namespaces=NS)) == 7
        assert len(xml.xpath('//m:oMath', namespaces=NS)) >= 6
        assert not xml.xpath('//w:ins | //w:del | //w:commentReference', namespaces=NS)
        for bad in ['planned_curvature', 'executed_curvature', 'kho điều phối', 'kho nghiên cứu', 'văn phòng']:
            assert bad not in text.lower(), bad
        for bad in ['bản thảo chưa công bố', 'Path Smoothing via Footprint-Aware Corner-Transition Optimization']:
            assert bad not in text, bad
        assert not re.search(r'\b(?:trong|theo)\s*\(\d+\)', text, re.I)
        assert not re.search(r'\b[0-9a-f]{40,64}\b', text)
        for required in ['Môi trường và quỹ đạo đặc trưng',
                         'PSTMO chưa cải thiện tỷ lệ hoàn thành trong tập thử',
                         'không phải hiệu năng tốt nhất sau hiệu chỉnh tham số',
                         'thao tác quay tại chỗ không được xem là một đường cong']:
            assert required in text, required
        paragraphs = [''.join(p.xpath('.//w:t/text()', namespaces=NS)) for p in xml.xpath('//w:p', namespaces=NS)]
        assert 'PSTMO' not in paragraphs[0]
        assert 'được ký hiệu là PSTMO' in text
        for i in range(1, 8):
            assert sum(p.startswith(f'Hình {i}.') for p in paragraphs) == 1
        refs = [p for p in paragraphs if re.match(r'^\[\d+\]\s', p)]
        assert len(refs) == 16, len(refs)
        for part in ['word/numbering.xml', 'word/fontTable.xml', 'word/theme/theme1.xml']:
            assert package.read(part) == original.read(part), part
        for sec in xml.xpath('//w:sectPr', namespaces=NS):
            dims = sec.find('w:pgSz', NS)
            assert dims.get('{'+NS['w']+'}w') == '11909'
            assert dims.get('{'+NS['w']+'}h') == '16834'
            margins = sec.find('w:pgMar', NS)
            for key, value in [('left', '1361'), ('right', '1361'), ('top', '1474'), ('bottom', '2155')]:
                assert margins.get('{'+NS['w']+'}'+key) == value
    with pdfplumber.open(args.pdf) as pdf:
        assert len(pdf.pages) == 6, len(pdf.pages)
        fonts = sorted({c['fontname'] for p in pdf.pages for c in p.chars})
        assert any('TimesNewRoman' in name.replace('-', '') for name in fonts), fonts
        # LibreOffice's editable-equation renderer uses separate symbol/math
        # fonts. Test prose font fidelity without rejecting mathematical glyphs.
        fallback_text = ''.join(c['text'] for p in pdf.pages for c in p.chars
                                if 'TimesNewRoman' not in c['fontname'])
        assert not re.search('[ăâđêôơưĂÂĐÊÔƠƯàáảãạằắẳẵặầấẩẫậèéẻẽẹềếểễệìíỉĩịòóỏõọồốổỗộờớởỡợùúủũụừứửữựỳýỷỹỵ]', fallback_text)
        for p in pdf.pages:
            assert p.chars
            assert all(62 < c['x0'] < c['x1'] < 534 and 65 < c['top'] < c['bottom'] < 745 for c in p.chars)
    results = {'pages': 6, 'figures': 7, 'tables': 7, 'numbered_equations': 6, 'references': 16,
               'data_checks': '175 source records; 170 successful measurements; 34 paired groups; four baselines; five failure logs',
               'template_geometry_and_preserved_parts': 'pass', 'source_fingerprints': 'pass',
               'terminology_and_reference_checks': 'pass', 'tracked_edits_and_comments': 0,
               'pdf_text_bounds': 'pass', 'pdf_fonts': fonts,
               'font_note': 'Times New Roman prose; editable math and superscript-minus glyphs use the listed mathematical fallback fonts in LibreOffice export',
               'all_pages_visually_reviewed': args.visual_reviewed,
               'docx_sha256': hashlib.sha256(docx.read_bytes()).hexdigest(),
               'pdf_sha256': hashlib.sha256(args.pdf.read_bytes()).hexdigest()}
    (OUT / 'quality_checks.json').write_text(json.dumps(results, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
