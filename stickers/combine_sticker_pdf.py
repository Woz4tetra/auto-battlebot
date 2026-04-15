#!/usr/bin/env python3
"""Build a vendor-ready sticker PDF from artwork + cut-line PDFs.

The output places artwork and cut content on separate PDF optional-content
layers (OCGs), with the cut layer named ``cut`` by default.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import pikepdf
except ImportError as exc:  # pragma: no cover - runtime dependency guard
    raise SystemExit(
        "Missing dependency: pikepdf\n"
        "Install with: pip install pikepdf"
    ) from exc


def _page_size(page: pikepdf.Page) -> tuple[float, float]:
    mediabox = page.obj.MediaBox
    x0, y0, x1, y1 = (float(value) for value in mediabox)
    return x1 - x0, y1 - y0


def _validate_matching_size(
    artwork_page: pikepdf.Page,
    cut_page: pikepdf.Page,
    page_number: int,
    tolerance: float,
) -> None:
    aw, ah = _page_size(artwork_page)
    cw, ch = _page_size(cut_page)
    if abs(aw - cw) > tolerance or abs(ah - ch) > tolerance:
        raise ValueError(
            "Page size mismatch at page "
            f"{page_number}: artwork={aw:.3f}x{ah:.3f} pt, "
            f"cut={cw:.3f}x{ch:.3f} pt"
        )


def _build_oc_properties(
    out_pdf: pikepdf.Pdf,
    artwork_layer_name: str,
    cut_layer_name: str,
) -> tuple[pikepdf.Object, pikepdf.Object]:
    artwork_ocg = out_pdf.make_indirect(
        pikepdf.Dictionary(
            Type=pikepdf.Name.OCG,
            Name=pikepdf.String(artwork_layer_name),
        )
    )
    cut_ocg = out_pdf.make_indirect(
        pikepdf.Dictionary(
            Type=pikepdf.Name.OCG,
            Name=pikepdf.String(cut_layer_name),
        )
    )
    out_pdf.Root.OCProperties = pikepdf.Dictionary(
        OCGs=pikepdf.Array([artwork_ocg, cut_ocg]),
        D=pikepdf.Dictionary(
            Name=pikepdf.String("Layer view"),
            BaseState=pikepdf.Name.ON,
            ON=pikepdf.Array([artwork_ocg, cut_ocg]),
            Order=pikepdf.Array([artwork_ocg, cut_ocg]),
        ),
    )
    return artwork_ocg, cut_ocg


def combine_pdfs(
    artwork_pdf: Path,
    cut_pdf: Path,
    output_pdf: Path,
    artwork_layer_name: str,
    cut_layer_name: str,
    tolerance: float = 0.01,
) -> None:
    with (
        pikepdf.Pdf.open(str(artwork_pdf)) as artwork_reader,
        pikepdf.Pdf.open(str(cut_pdf)) as cut_reader,
        pikepdf.Pdf.new() as out_pdf,
    ):
        if len(artwork_reader.pages) == 0:
            raise ValueError(f"Artwork PDF has no pages: {artwork_pdf}")
        if len(cut_reader.pages) == 0:
            raise ValueError(f"Cut-line PDF has no pages: {cut_pdf}")

        # Support either one cut page for all artwork pages, or 1:1 page mapping.
        if len(cut_reader.pages) not in (1, len(artwork_reader.pages)):
            raise ValueError(
                "Cut-line PDF must have either 1 page or the same page count "
                f"as artwork PDF. artwork={len(artwork_reader.pages)}, "
                f"cut={len(cut_reader.pages)}"
            )

        artwork_ocg, cut_ocg = _build_oc_properties(
            out_pdf=out_pdf,
            artwork_layer_name=artwork_layer_name,
            cut_layer_name=cut_layer_name,
        )

        for index, artwork_page in enumerate(artwork_reader.pages):
            cut_page = (
                cut_reader.pages[0]
                if len(cut_reader.pages) == 1
                else cut_reader.pages[index]
            )

            _validate_matching_size(
                artwork_page=artwork_page,
                cut_page=cut_page,
                page_number=index + 1,
                tolerance=tolerance,
            )

            width, height = _page_size(artwork_page)
            out_page = out_pdf.add_blank_page(page_size=(width, height))

            artwork_form = out_pdf.copy_foreign(artwork_page.as_form_xobject())
            cut_form = out_pdf.copy_foreign(cut_page.as_form_xobject())

            out_page.obj.Resources = pikepdf.Dictionary(
                XObject=pikepdf.Dictionary(
                    Art=artwork_form,
                    Cut=cut_form,
                ),
                Properties=pikepdf.Dictionary(
                    ArtLayer=artwork_ocg,
                    CutLayer=cut_ocg,
                ),
            )

            # Keep each form in its own optional-content group.
            out_page.obj.Contents = out_pdf.make_stream(
                b"q /OC /ArtLayer BDC /Art Do EMC Q\n"
                b"q /OC /CutLayer BDC /Cut Do EMC Q\n"
            )

        output_pdf.parent.mkdir(parents=True, exist_ok=True)
        out_pdf.save(str(output_pdf))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Overlay a sticker cut-line PDF on top of a sticker artwork PDF "
            "and emit a layered PDF with a named cut layer."
        )
    )
    parser.add_argument(
        "artwork_pdf",
        nargs="?",
        default="stickers/Stickers - MBMK3 Top Sticker.pdf",
        help="Path to artwork/decal PDF (default: MBMK3 top sticker).",
    )
    parser.add_argument(
        "cut_pdf",
        nargs="?",
        default="stickers/Stickers - MBMK3 Top Sticker 1.pdf",
        help="Path to cut-line PDF (default: MBMK3 top sticker cut line).",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Path for output combined PDF (default: artwork name + ' vendor-ready').",
    )
    parser.add_argument(
        "--size-tolerance",
        type=float,
        default=0.01,
        help="Allowed page size difference in points before failing (default: 0.01).",
    )
    parser.add_argument(
        "--art-layer-name",
        default="artwork",
        help="Name of the artwork layer in the output PDF.",
    )
    parser.add_argument(
        "--cut-layer-name",
        default="cut",
        help="Name of the cut layer in the output PDF (default: cut).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    artwork_pdf = Path(args.artwork_pdf)
    cut_pdf = Path(args.cut_pdf)
    if args.output is None:
        output_pdf = artwork_pdf.with_name(f"{artwork_pdf.stem} vendor-ready.pdf")
    else:
        output_pdf = Path(args.output)

    if not artwork_pdf.exists():
        print(f"Artwork PDF not found: {artwork_pdf}", file=sys.stderr)
        return 2
    if not cut_pdf.exists():
        print(f"Cut-line PDF not found: {cut_pdf}", file=sys.stderr)
        return 2

    try:
        combine_pdfs(
            artwork_pdf=artwork_pdf,
            cut_pdf=cut_pdf,
            output_pdf=output_pdf,
            artwork_layer_name=args.art_layer_name,
            cut_layer_name=args.cut_layer_name,
            tolerance=args.size_tolerance,
        )
    except Exception as exc:  # pragma: no cover - CLI surface
        print(f"Failed to combine PDFs: {exc}", file=sys.stderr)
        return 1

    print(f"Wrote combined PDF: {output_pdf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
