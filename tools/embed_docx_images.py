#!/usr/bin/env python3

"""Convert external PNG relationships in a DOCX into embedded images."""

import argparse
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET
import zipfile


REL_NS = 'http://schemas.openxmlformats.org/package/2006/relationships'
R_NS = 'http://schemas.openxmlformats.org/officeDocument/2006/relationships'
CT_NS = 'http://schemas.openxmlformats.org/package/2006/content-types'
WP_NS = 'http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing'
A_NS = 'http://schemas.openxmlformats.org/drawingml/2006/main'
EMU_PER_INCH = 914400


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('output')
    parser.add_argument(
        '--max-image-width-inches',
        type=float,
        default=6.2,
        help='Clamp inline image widths while preserving their aspect ratio.',
    )
    options = parser.parse_args()
    source = Path(options.source).resolve()
    output = Path(options.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix='embed_docx_') as directory:
        unpacked = Path(directory)
        with zipfile.ZipFile(source) as archive:
            archive.extractall(unpacked)

        relationships_path = unpacked / 'word/_rels/document.xml.rels'
        relationships = ET.parse(relationships_path)
        embedded_ids = []
        media_directory = unpacked / 'word/media'
        media_directory.mkdir(parents=True, exist_ok=True)
        image_index = 1
        for relationship in relationships.getroot():
            if not relationship.get('Type', '').endswith('/image'):
                continue
            if relationship.get('TargetMode') != 'External':
                continue
            target = relationship.get('Target', '')
            if not target.startswith('file://'):
                continue
            image_path = Path(target.removeprefix('file://'))
            suffix = image_path.suffix.lower() or '.png'
            embedded_name = f'paper_image_{image_index}{suffix}'
            shutil.copy2(image_path, media_directory / embedded_name)
            relationship.set('Target', f'media/{embedded_name}')
            relationship.attrib.pop('TargetMode', None)
            embedded_ids.append(relationship.get('Id'))
            image_index += 1
        ET.register_namespace('', REL_NS)
        relationships.write(
            relationships_path, encoding='UTF-8', xml_declaration=True
        )

        document_path = unpacked / 'word/document.xml'
        document_text = document_path.read_text(encoding='utf-8')
        for relationship_id in embedded_ids:
            document_text = document_text.replace(
                f'r:link="{relationship_id}"',
                f'r:embed="{relationship_id}"',
            )
        document_path.write_text(document_text, encoding='utf-8')

        # LibreOffice imports HTML images at their natural pixel dimensions and
        # can therefore create a valid DOCX whose pictures extend far beyond the
        # printable area. Clamp both DrawingML extents so Word and LibreOffice
        # render the same, page-width figure.
        document = ET.parse(document_path)
        max_width = round(options.max_image_width_inches * EMU_PER_INCH)
        for inline in document.getroot().iter(f'{{{WP_NS}}}inline'):
            extent = inline.find(f'{{{WP_NS}}}extent')
            if extent is None:
                continue
            width = int(extent.get('cx', '0'))
            height = int(extent.get('cy', '0'))
            if width <= max_width or width == 0:
                continue
            scale = max_width / width
            resized_width = max_width
            resized_height = round(height * scale)
            extent.set('cx', str(resized_width))
            extent.set('cy', str(resized_height))
            for drawing_extent in inline.iter(f'{{{A_NS}}}ext'):
                drawing_extent.set('cx', str(resized_width))
                drawing_extent.set('cy', str(resized_height))
        ET.register_namespace('wp', WP_NS)
        ET.register_namespace('a', A_NS)
        ET.register_namespace('r', R_NS)
        document.write(document_path, encoding='UTF-8', xml_declaration=True)

        content_types_path = unpacked / '[Content_Types].xml'
        content_types = ET.parse(content_types_path)
        root = content_types.getroot()
        has_png = any(
            item.get('Extension') == 'png' for item in root
            if item.tag.endswith('Default')
        )
        if not has_png:
            ET.SubElement(
                root,
                f'{{{CT_NS}}}Default',
                {'Extension': 'png', 'ContentType': 'image/png'},
            )
        ET.register_namespace('', CT_NS)
        content_types.write(
            content_types_path, encoding='UTF-8', xml_declaration=True
        )

        with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(unpacked.rglob('*')):
                if path.is_file():
                    archive.write(path, path.relative_to(unpacked))
    print(output)


if __name__ == '__main__':
    main()
