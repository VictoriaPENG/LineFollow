#!/usr/bin/env python3
"""生成用于相机标定的可打印棋盘格。"""

from __future__ import annotations

import argparse
import math
from pathlib import Path


def build_svg(
    inner_cols: int,
    inner_rows: int,
    square_size_mm: float,
    margin_mm: float,
) -> str:
    """生成矢量棋盘格，便于无损打印。"""
    squares_x = inner_cols + 1
    squares_y = inner_rows + 1
    board_w_mm = squares_x * square_size_mm
    board_h_mm = squares_y * square_size_mm
    canvas_w_mm = board_w_mm + 2.0 * margin_mm
    canvas_h_mm = board_h_mm + 2.0 * margin_mm

    lines = [
        '<?xml version="1.0" encoding="UTF-8" standalone="no"?>',
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" '
            f'width="{canvas_w_mm}mm" height="{canvas_h_mm}mm" '
            f'viewBox="0 0 {canvas_w_mm} {canvas_h_mm}">'
        ),
        f'  <rect x="0" y="0" width="{canvas_w_mm}" height="{canvas_h_mm}" fill="white"/>',
    ]

    for row in range(squares_y):
        for col in range(squares_x):
            fill = "black" if (row + col) % 2 == 0 else "white"
            x = margin_mm + col * square_size_mm
            y = margin_mm + row * square_size_mm
            lines.append(
                f'  <rect x="{x}" y="{y}" width="{square_size_mm}" '
                f'height="{square_size_mm}" fill="{fill}"/>'
            )

    lines.extend(
        [
            (
                f'  <rect x="{margin_mm}" y="{margin_mm}" width="{board_w_mm}" '
                f'height="{board_h_mm}" fill="none" stroke="black" stroke-width="0.4"/>'
            ),
            "</svg>",
        ]
    )
    return "\n".join(lines) + "\n"


def write_png(
    output_path: Path,
    inner_cols: int,
    inner_rows: int,
    square_size_mm: float,
    margin_mm: float,
    dpi: int,
) -> None:
    """生成 Windows 打印程序更容易处理的 RGB PNG，并写入 DPI 元数据。"""
    from PIL import Image, ImageDraw

    squares_x = inner_cols + 1
    squares_y = inner_rows + 1
    px_per_mm = dpi / 25.4
    square_size_px = int(round(square_size_mm * px_per_mm))
    margin_px = int(round(margin_mm * px_per_mm))
    width = squares_x * square_size_px + 2 * margin_px
    height = squares_y * square_size_px + 2 * margin_px

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    for row in range(squares_y):
        for col in range(squares_x):
            if (row + col) % 2 != 0:
                continue
            x0 = margin_px + col * square_size_px
            y0 = margin_px + row * square_size_px
            x1 = x0 + square_size_px
            y1 = y0 + square_size_px
            draw.rectangle((x0, y0, x1 - 1, y1 - 1), fill="black")

    image.save(output_path, dpi=(dpi, dpi))


def write_pdf(
    output_path: Path,
    inner_cols: int,
    inner_rows: int,
    square_size_mm: float,
    margin_mm: float,
) -> None:
    """生成 A4 横向 PDF，适合复制到 Windows 后直接按 100% 打印。"""
    from reportlab.lib import colors
    from reportlab.lib.pagesizes import A4, landscape
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas

    squares_x = inner_cols + 1
    squares_y = inner_rows + 1
    board_w_mm = squares_x * square_size_mm
    board_h_mm = squares_y * square_size_mm
    canvas_w_mm = board_w_mm + 2.0 * margin_mm
    canvas_h_mm = board_h_mm + 2.0 * margin_mm

    page_w, page_h = landscape(A4)
    content_w = canvas_w_mm * mm
    content_h = canvas_h_mm * mm
    if content_w > page_w or content_h > page_h:
        raise ValueError(
            "pattern does not fit on A4 landscape; reduce square size or margins"
        )

    c = canvas.Canvas(str(output_path), pagesize=(page_w, page_h))
    origin_x = (page_w - content_w) / 2.0
    origin_y = (page_h - content_h) / 2.0

    c.setFillColor(colors.white)
    c.rect(0, 0, page_w, page_h, stroke=0, fill=1)
    for row in range(squares_y):
        for col in range(squares_x):
            if (row + col) % 2 != 0:
                continue
            x = origin_x + (margin_mm + col * square_size_mm) * mm
            y_from_top = margin_mm + row * square_size_mm
            y = origin_y + content_h - (y_from_top + square_size_mm) * mm
            c.setFillColor(colors.black)
            c.rect(x, y, square_size_mm * mm, square_size_mm * mm, stroke=0, fill=1)

    c.setStrokeColor(colors.black)
    c.setLineWidth(0.4 * mm)
    c.rect(
        origin_x + margin_mm * mm,
        origin_y + margin_mm * mm,
        board_w_mm * mm,
        board_h_mm * mm,
        stroke=1,
        fill=0,
    )

    label = f"{inner_cols}x{inner_rows} inner corners, {square_size_mm:g} mm squares"
    c.setFillColor(colors.black)
    c.setFont("Helvetica", 8)
    c.drawString(origin_x, max(4 * mm, origin_y - 6 * mm), label)
    c.showPage()
    c.save()


def parse_args() -> argparse.Namespace:
    """解析棋盘格规格与输出目录参数。"""
    parser = argparse.ArgumentParser(description="Generate a printable chessboard pattern.")
    parser.add_argument("--cols", type=int, default=9, help="Inner corner columns.")
    parser.add_argument("--rows", type=int, default=6, help="Inner corner rows.")
    parser.add_argument(
        "--square-size-mm",
        type=float,
        default=25.0,
        help="Physical size of one square in millimeters.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=300,
        help="DPI metadata and rasterization resolution for PNG output.",
    )
    parser.add_argument(
        "--margin-mm",
        type=float,
        default=12.0,
        help="White border margin in millimeters.",
    )
    parser.add_argument(
        "--no-pdf",
        action="store_true",
        help="Do not generate the Windows-friendly printable PDF.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("tools/usb_camera_calibration/output/pattern"),
        help="Directory to place generated pattern files.",
    )
    return parser.parse_args()


def main() -> None:
    """同时输出 SVG 和可选 PNG 棋盘格文件。"""
    args = parse_args()
    if args.cols < 2 or args.rows < 2:
        raise ValueError("cols and rows must both be at least 2")
    if args.square_size_mm <= 0:
        raise ValueError("square size must be positive")
    if args.dpi <= 0 or not math.isfinite(args.dpi):
        raise ValueError("dpi must be positive")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"chessboard_{args.cols}x{args.rows}_{args.square_size_mm:g}mm"
    svg_path = args.output_dir / f"{stem}.svg"
    png_path = args.output_dir / f"{stem}.png"
    pdf_path = args.output_dir / f"{stem}_a4_landscape.pdf"

    svg = build_svg(args.cols, args.rows, args.square_size_mm, args.margin_mm)
    svg_path.write_text(svg, encoding="utf-8")

    png_written = False
    pdf_written = False
    try:
        write_png(
            png_path,
            args.cols,
            args.rows,
            args.square_size_mm,
            args.margin_mm,
            args.dpi,
        )
        png_written = True
    except ModuleNotFoundError:
        pass

    if not args.no_pdf:
        try:
            write_pdf(
                pdf_path,
                args.cols,
                args.rows,
                args.square_size_mm,
                args.margin_mm,
            )
            pdf_written = True
        except ModuleNotFoundError:
            pass

    print(f"SVG written: {svg_path}")
    if png_written:
        print(f"PNG written: {png_path} ({args.dpi} DPI, RGB)")
    else:
        print("PNG skipped: Pillow unavailable, SVG/PDF output is still ready for printing.")
    if pdf_written:
        print(f"PDF written: {pdf_path} (A4 landscape, print at 100% / actual size)")
    elif not args.no_pdf:
        print("PDF skipped: reportlab unavailable.")
    print(f"Pattern inner corners: {args.cols} x {args.rows}")
    print(f"Physical square size: {args.square_size_mm:.3f} mm")


if __name__ == "__main__":
    main()
