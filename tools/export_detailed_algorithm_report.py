#!/usr/bin/env python3

"""Export the detailed DOCX report to PDF and a self-contained HTML file."""

from __future__ import annotations

import argparse
import base64
import mimetypes
import re
import shutil
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DOCX = ROOT / "docs" / "BAO_CAO_TOAN_DIEN_ADAPTIVE_HYBRID_PIVOT_G2.docx"
DEFAULT_OUTPUT_DIR = ROOT / "docs"
IMAGE_SOURCE_RE = re.compile(
    r'(?P<prefix><img\b[^>]*\bsrc=")(?P<source>[^"]+)(?P<suffix>")',
    re.IGNORECASE,
)


def run_libreoffice(docx: Path, output_dir: Path, output_format: str) -> Path:
    subprocess.run(
        [
            "libreoffice",
            "--headless",
            "--convert-to",
            output_format,
            "--outdir",
            str(output_dir),
            str(docx),
        ],
        check=True,
    )
    output = output_dir / f"{docx.stem}.{output_format}"
    if not output.is_file() or output.stat().st_size == 0:
        raise RuntimeError(f"LibreOffice did not create {output}")
    return output


def embed_html_images(html_path: Path) -> tuple[str, int]:
    html = html_path.read_text(encoding="utf-8")
    embedded = 0

    def replace(match: re.Match[str]) -> str:
        nonlocal embedded
        source = match.group("source")
        if source.startswith(("data:", "http://", "https://")):
            return match.group(0)
        image_path = (html_path.parent / source).resolve()
        if not image_path.is_file():
            raise FileNotFoundError(
                f"HTML image reference does not exist: {source}"
            )
        media_type = mimetypes.guess_type(image_path.name)[0] or "image/png"
        payload = base64.b64encode(image_path.read_bytes()).decode("ascii")
        embedded += 1
        return (
            f'{match.group("prefix")}data:{media_type};base64,{payload}'
            f'{match.group("suffix")}'
        )

    html = IMAGE_SOURCE_RE.sub(replace, html)
    html = "\n".join(line.rstrip() for line in html.splitlines()) + "\n"
    unresolved = [
        match.group("source")
        for match in IMAGE_SOURCE_RE.finditer(html)
        if not match.group("source").startswith("data:")
    ]
    if unresolved:
        raise RuntimeError(f"Unresolved HTML image references: {unresolved}")
    return html, embedded


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--docx", type=Path, default=DEFAULT_DOCX)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--expected-images", type=int, default=35)
    args = parser.parse_args()

    docx = args.docx.resolve()
    output_dir = args.output_dir.resolve()
    if not docx.is_file():
        raise FileNotFoundError(docx)
    output_dir.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="pivot-g2-report-export-") as temp:
        temp_dir = Path(temp)
        pdf_temp = run_libreoffice(docx, temp_dir, "pdf")
        html_temp = run_libreoffice(docx, temp_dir, "html")
        html, embedded = embed_html_images(html_temp)
        if embedded != args.expected_images:
            raise RuntimeError(
                f"Expected {args.expected_images} HTML images, embedded {embedded}"
            )

        pdf_output = output_dir / f"{docx.stem}.pdf"
        html_output = output_dir / f"{docx.stem}.html"
        shutil.copy2(pdf_temp, pdf_output)
        html_output.write_text(html, encoding="utf-8")

    print(
        f"Exported {pdf_output} and self-contained {html_output} "
        f"with {embedded} embedded images"
    )


if __name__ == "__main__":
    main()
