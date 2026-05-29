#!/usr/bin/env python
"""Generate a parametric NTU outline target for user confirmation.

The target points come from a code-generated connected NTU solid glyph.  The
script builds N/T/U from rectangles, one diagonal polygon, and one continuous
U body with a tangent bottom arc, then samples that generated body's boundary.
No external image, screenshot, or font outline is read.
"""

from __future__ import print_function

import csv
import math
import os
import struct
import sys
import zlib


THIS_DIR = os.path.abspath(os.path.dirname(__file__))
REPRO_DIR = os.path.abspath(os.path.join(THIS_DIR, ".."))
OUTPUT_DIR = os.path.join(REPRO_DIR, "outputs")
if REPRO_DIR not in sys.path:
    sys.path.insert(0, REPRO_DIR)

from parametric_ntu_outline import sample_ntu_outline_uv, ntu_outline_program


WIDTH_M = 3.0
HEIGHT_M = 0.75
ORIGIN_X = -1.50
ORIGIN_Y = 0.375
Z_M = 0.56
STROKE_M = 0.16
LINE_STEP_M = 0.025
ARC_STEP_M = 0.018


def uv_to_xy(u, v):
    return ORIGIN_X + WIDTH_M * u, ORIGIN_Y - HEIGHT_M * v


def build_candidate_outline():
    uv_points = sample_ntu_outline_uv(
        max_linear_step=LINE_STEP_M,
        max_arc_step=ARC_STEP_M,
        scale_u=WIDTH_M,
        scale_v=HEIGHT_M,
        normalize=True,
        stroke_m=STROKE_M,
    )
    points = []
    for u, v, label in uv_points:
        x, y = uv_to_xy(u, v)
        points.append({
            "u": float(u),
            "v": float(v),
            "x": float(x),
            "y": float(y),
            "z": Z_M,
            "label": label,
        })
    return points


def path_length(points):
    total = 0.0
    for a, b in zip(points[:-1], points[1:]):
        total += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
    return total


def png_chunk(tag, data):
    payload = tag + data
    return (
        struct.pack(">I", len(data))
        + payload
        + struct.pack(">I", zlib.crc32(payload) & 0xffffffff)
    )


def write_png_file(path, width, height, pixels):
    stride = width * 3
    rows = []
    for y in range(height):
        rows.append(b"\x00" + bytes(pixels[y * stride:(y + 1) * stride]))
    raw = b"".join(rows)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(png_chunk(
            b"IHDR",
            struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0),
        ))
        f.write(png_chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(png_chunk(b"IEND", b""))


def blend_pixel(pixels, width, height, x, y, color, alpha=1.0):
    x = int(x)
    y = int(y)
    if x < 0 or y < 0 or x >= width or y >= height:
        return
    idx = (y * width + x) * 3
    inv = 1.0 - alpha
    pixels[idx] = int(pixels[idx] * inv + color[0] * alpha)
    pixels[idx + 1] = int(pixels[idx + 1] * inv + color[1] * alpha)
    pixels[idx + 2] = int(pixels[idx + 2] * inv + color[2] * alpha)


def fill_rect(pixels, width, height, x0, y0, x1, y1, color, alpha=1.0):
    x0 = max(0, int(math.floor(x0)))
    y0 = max(0, int(math.floor(y0)))
    x1 = min(width - 1, int(math.ceil(x1)))
    y1 = min(height - 1, int(math.ceil(y1)))
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            blend_pixel(pixels, width, height, x, y, color, alpha)


def draw_circle(pixels, width, height, cx, cy, radius, color, alpha=1.0):
    radius = float(radius)
    xmin = int(math.floor(cx - radius))
    xmax = int(math.ceil(cx + radius))
    ymin = int(math.floor(cy - radius))
    ymax = int(math.ceil(cy + radius))
    r2 = radius * radius
    for y in range(ymin, ymax + 1):
        for x in range(xmin, xmax + 1):
            if (x + 0.5 - cx) ** 2 + (y + 0.5 - cy) ** 2 <= r2:
                blend_pixel(pixels, width, height, x, y, color, alpha)


def draw_line(pixels, width, height, x0, y0, x1, y1, radius, color, alpha=1.0):
    length = math.hypot(x1 - x0, y1 - y0)
    steps = max(1, int(math.ceil(length * 1.5)))
    for idx in range(steps + 1):
        t = float(idx) / float(steps)
        draw_circle(
            pixels,
            width,
            height,
            (1.0 - t) * x0 + t * x1,
            (1.0 - t) * y0 + t * y1,
            radius,
            color,
            alpha,
        )


def draw_square_line(pixels, width, height, x0, y0, x1, y1, half_width,
                     color, alpha=1.0):
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    if length < 1e-9:
        fill_rect(pixels, width, height, x0 - half_width, y0 - half_width,
                  x0 + half_width, y0 + half_width, color, alpha)
        return
    nx = -dy / length * half_width
    ny = dx / length * half_width
    vertices = [
        (x0 + nx, y0 + ny),
        (x1 + nx, y1 + ny),
        (x1 - nx, y1 - ny),
        (x0 - nx, y0 - ny),
    ]
    fill_polygon(pixels, width, height, vertices, color, alpha)


def fill_polygon(pixels, width, height, vertices, color, alpha=1.0):
    if len(vertices) < 3:
        return
    ymin = max(0, int(math.floor(min(y for _, y in vertices))))
    ymax = min(height - 1, int(math.ceil(max(y for _, y in vertices))))
    edges = list(zip(vertices, vertices[1:] + vertices[:1]))
    for y in range(ymin, ymax + 1):
        scan_y = y + 0.5
        xs = []
        for (x0, y0), (x1, y1) in edges:
            if abs(y1 - y0) < 1e-9:
                continue
            if (y0 <= scan_y < y1) or (y1 <= scan_y < y0):
                t = (scan_y - y0) / (y1 - y0)
                xs.append(x0 + t * (x1 - x0))
        xs.sort()
        for left, right in zip(xs[0::2], xs[1::2]):
            for x in range(max(0, int(math.ceil(left))),
                           min(width - 1, int(math.floor(right))) + 1):
                blend_pixel(pixels, width, height, x, y, color, alpha)


DIGIT_FONT = {
    "0": ["111", "101", "101", "101", "111"],
    "1": ["010", "110", "010", "010", "111"],
    "2": ["111", "001", "111", "100", "111"],
    "3": ["111", "001", "111", "001", "111"],
    "4": ["101", "101", "111", "001", "001"],
    "5": ["111", "100", "111", "001", "111"],
    "6": ["111", "100", "111", "101", "111"],
    "7": ["111", "001", "010", "010", "010"],
    "8": ["111", "101", "111", "101", "111"],
    "9": ["111", "101", "111", "001", "111"],
}


def draw_digits(pixels, width, height, x, y, text, color=(20, 20, 20), scale=3):
    cursor = int(x)
    for char in str(text):
        glyph = DIGIT_FONT.get(char)
        if glyph is None:
            cursor += 2 * scale
            continue
        for row, bits in enumerate(glyph):
            for col, bit in enumerate(bits):
                if bit == "1":
                    fill_rect(
                        pixels,
                        width,
                        height,
                        cursor + col * scale,
                        y + row * scale,
                        cursor + (col + 1) * scale - 1,
                        y + (row + 1) * scale - 1,
                        color,
                        1.0,
                    )
        cursor += 4 * scale


def image_transform(points, width, height, margin):
    us = [p["u"] for p in points]
    vs = [p["v"] for p in points]
    umin, umax = min(us) - 0.030, max(us) + 0.030
    vmin, vmax = min(vs) - 0.070, max(vs) + 0.070

    def sx(u):
        return margin + (u - umin) / (umax - umin) * (width - 2 * margin)

    def sy(v):
        return margin + (v - vmin) / (vmax - vmin) * (height - 2 * margin)

    return sx, sy


def write_preview_png(path, points):
    width = 1400
    height = 420
    margin = 68
    pixels = bytearray([75, 77, 76] * width * height)
    sx, sy = image_transform(points, width, height, margin)
    vertices = [(sx(p["u"]), sy(p["v"])) for p in points]

    for a, b in zip(vertices[:-1], vertices[1:]):
        draw_square_line(pixels, width, height, a[0], a[1], b[0], b[1],
                         9.0, (114, 91, 0), 0.95)
    for a, b in zip(vertices[:-1], vertices[1:]):
        draw_square_line(pixels, width, height, a[0], a[1], b[0], b[1],
                         4.4, (242, 194, 48), 1.0)
    for a, b in zip(vertices[:-1], vertices[1:]):
        draw_square_line(pixels, width, height, a[0], a[1], b[0], b[1],
                         1.4, (255, 246, 176), 1.0)
    draw_circle(pixels, width, height, vertices[0][0], vertices[0][1],
                10.0, (31, 191, 117), 1.0)
    write_png_file(path, width, height, pixels)


def write_numbered_png(path, points):
    width = 1500
    height = 500
    margin = 82
    pixels = bytearray([255, 255, 255] * width * height)
    sx, sy = image_transform(points, width, height, margin)
    vertices = [(sx(p["u"]), sy(p["v"])) for p in points]

    for a, b in zip(vertices[:-1], vertices[1:]):
        draw_square_line(pixels, width, height, a[0], a[1], b[0], b[1],
                         7.0, (208, 163, 0), 0.75)
    for a, b in zip(vertices[:-1], vertices[1:]):
        draw_square_line(pixels, width, height, a[0], a[1], b[0], b[1],
                         2.4, (17, 17, 17), 1.0)

    labeled = list(range(0, len(points), 8))
    if len(points) - 1 not in labeled:
        labeled.append(len(points) - 1)
    for idx in labeled:
        x, y = vertices[idx]
        is_end = idx == 0 or idx == len(points) - 1
        draw_circle(
            pixels,
            width,
            height,
            x,
            y,
            7.5,
            (31, 191, 117) if is_end else (255, 255, 255),
            1.0,
        )
        draw_circle(pixels, width, height, x, y, 3.0, (17, 17, 17), 1.0)
        label = str(idx)
        label_w = max(12, len(label) * 12)
        fill_rect(pixels, width, height, x + 9, y - 19,
                  x + 14 + label_w, y - 1, (255, 255, 255), 0.88)
        draw_digits(pixels, width, height, x + 13, y - 17, label, scale=3)

    write_png_file(path, width, height, pixels)


def write_csv(path, points):
    with open(path, "w") as f:
        writer = csv.DictWriter(
            f, fieldnames=["idx", "x", "y", "z", "u", "v", "label"])
        writer.writeheader()
        for idx, point in enumerate(points):
            row = dict(point)
            row["idx"] = idx
            writer.writerow(row)


def write_program_csv(path):
    with open(path, "w") as f:
        writer = csv.writer(f)
        writer.writerow([
            "idx", "primitive", "name", "u0", "v0", "u1", "v1",
            "vertices_uv", "center_u", "center_v", "radius_u", "radius_v",
            "theta0_rad", "theta1_rad", "stroke_width",
        ])
        for idx, primitive in enumerate(ntu_outline_program(
                scale_u=WIDTH_M,
                scale_v=HEIGHT_M,
                stroke_m=STROKE_M)):
            kind = primitive[0]
            if kind == "rect":
                _, name, u0, v0, u1, v1 = primitive
                writer.writerow([
                    idx, kind, name, u0, v0, u1, v1,
                    "", "", "", "", "", "", "", "",
                ])
            elif kind == "poly":
                _, name, vertices = primitive
                writer.writerow([
                    idx, kind, name, "", "", "", "",
                    ";".join("{:.6f}:{:.6f}".format(u, v)
                             for u, v in vertices),
                    "", "", "", "", "", "", "",
                ])
            elif kind == "arc_stroke":
                _, name, cu, cv, ru, rv, theta0, theta1, stroke_width = primitive
                writer.writerow([
                    idx, kind, name, "", "", "", "",
                    "", cu, cv, ru, rv, theta0, theta1, stroke_width,
                ])


def write_svg(path, points):
    width_px = 1180.0
    height_px = 360.0
    margin = 54.0
    sx, sy = image_transform(points, width_px, height_px, margin)
    poly = " ".join("{:.2f},{:.2f}".format(sx(p["u"]), sy(p["v"]))
                    for p in points)

    controls = points[::8]
    if controls[-1] is not points[-1]:
        controls.append(points[-1])

    elements = []
    elements.append('<rect width="100%" height="100%" fill="#4b4d4c"/>')
    elements.append(
        '<polyline points="{}" fill="none" stroke="#6f5a00" '
        'stroke-width="16" stroke-linejoin="miter" '
        'stroke-linecap="butt"/>'.format(poly))
    elements.append(
        '<polyline points="{}" fill="none" stroke="#fff6b0" '
        'stroke-width="5" stroke-linejoin="miter" '
        'stroke-linecap="butt"/>'.format(poly))
    elements.append(
        '<polyline points="{}" fill="none" stroke="#111" '
        'stroke-width="1.5" stroke-linejoin="round" '
        'stroke-dasharray="7 6" opacity="0.85"/>'.format(poly))

    for point in controls:
        elements.append(
            '<circle cx="{:.2f}" cy="{:.2f}" r="4" fill="#101010" '
            'stroke="#fff6b0" stroke-width="1"/>'.format(
                sx(point["u"]), sy(point["v"])))

    start = points[0]
    elements.append(
        '<circle cx="{:.2f}" cy="{:.2f}" r="8" fill="#1fbf75" '
        'stroke="#ffffff" stroke-width="2"/>'.format(
            sx(start["u"]), sy(start["v"])))
    elements.append(
        '<text x="{:.2f}" y="{:.2f}" font-family="Arial" font-size="14" '
        'fill="#ffffff">start=end</text>'.format(
            sx(start["u"]) + 12.0, sy(start["v"]) - 8.0))
    elements.append(
        '<text x="54" y="32" font-family="Arial" font-size="17" '
        'fill="#f7f2d0">Boundary of generated connected NTU glyph</text>')

    with open(path, "w") as f:
        f.write(
            '<svg xmlns="http://www.w3.org/2000/svg" width="{:.0f}" '
            'height="{:.0f}" viewBox="0 0 {:.0f} {:.0f}">\n'.format(
                width_px, height_px, width_px, height_px))
        f.write("\n".join(elements))
        f.write("\n</svg>\n")


def write_numbered_svg(path, points):
    width_px = 1320.0
    height_px = 430.0
    margin = 62.0
    sx, sy = image_transform(points, width_px, height_px, margin)
    poly = " ".join("{:.2f},{:.2f}".format(sx(p["u"]), sy(p["v"]))
                    for p in points)

    labeled_indices = list(range(0, len(points), 8))
    if len(points) - 1 not in labeled_indices:
        labeled_indices.append(len(points) - 1)

    elements = []
    elements.append('<rect width="100%" height="100%" fill="#ffffff"/>')
    elements.append(
        '<polyline points="{}" fill="none" stroke="#d0a300" '
        'stroke-width="16" stroke-linejoin="miter" '
        'stroke-linecap="butt"/>'.format(poly))
    elements.append(
        '<polyline points="{}" fill="none" stroke="#111111" '
        'stroke-width="3" stroke-linejoin="miter" '
        'stroke-linecap="butt"/>'.format(poly))

    for idx in range(0, len(points) - 1, 6):
        a = points[idx]
        b = points[min(idx + 1, len(points) - 1)]
        dx = sx(b["u"]) - sx(a["u"])
        dy = sy(b["v"]) - sy(a["v"])
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            continue
        ux, uy = dx / norm, dy / norm
        cx = sx(a["u"]) + 0.55 * dx
        cy = sy(a["v"]) + 0.55 * dy
        left_x = cx - 8.0 * ux - 4.0 * uy
        left_y = cy - 8.0 * uy + 4.0 * ux
        right_x = cx - 8.0 * ux + 4.0 * uy
        right_y = cy - 8.0 * uy - 4.0 * ux
        elements.append(
            '<polygon points="{:.2f},{:.2f} {:.2f},{:.2f} {:.2f},{:.2f}" '
            'fill="#111111" opacity="0.72"/>'.format(
                cx, cy, left_x, left_y, right_x, right_y))

    for idx in labeled_indices:
        point = points[idx]
        x = sx(point["u"])
        y = sy(point["v"])
        fill = "#1fbf75" if idx == 0 or idx == len(points) - 1 else "#ffffff"
        stroke = "#0b5f3c" if idx == 0 or idx == len(points) - 1 else "#111111"
        elements.append(
            '<circle cx="{:.2f}" cy="{:.2f}" r="7" fill="{}" '
            'stroke="{}" stroke-width="2"/>'.format(x, y, fill, stroke))
        elements.append(
            '<text x="{:.2f}" y="{:.2f}" font-family="Arial" font-size="12" '
            'font-weight="bold" fill="#111111">{}</text>'.format(
                x + 9.0, y - 8.0, idx))

    elements.append(
        '<text x="62" y="34" font-family="Arial" font-size="18" '
        'font-weight="bold" fill="#111111">Generated connected NTU boundary samples</text>')
    elements.append(
        '<text x="62" y="58" font-family="Arial" font-size="13" '
        'fill="#333333">Green point is start=end. Black arrows show traversal '
        'direction. CSV contains all {} sampled points.</text>'.format(
            len(points)))

    with open(path, "w") as f:
        f.write(
            '<svg xmlns="http://www.w3.org/2000/svg" width="{:.0f}" '
            'height="{:.0f}" viewBox="0 0 {:.0f} {:.0f}">\n'.format(
                width_px, height_px, width_px, height_px))
        f.write("\n".join(elements))
        f.write("\n</svg>\n")


def main():
    if not os.path.isdir(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    points = build_candidate_outline()
    csv_path = os.path.join(OUTPUT_DIR, "ntu_outline_candidate_points.csv")
    program_csv_path = os.path.join(
        OUTPUT_DIR, "ntu_outline_parametric_program.csv")
    svg_path = os.path.join(OUTPUT_DIR, "ntu_outline_candidate_preview.svg")
    numbered_svg_path = os.path.join(
        OUTPUT_DIR, "ntu_outline_candidate_numbered.svg")
    png_path = os.path.join(OUTPUT_DIR, "ntu_outline_candidate_preview.png")
    numbered_png_path = os.path.join(
        OUTPUT_DIR, "ntu_outline_candidate_numbered.png")

    write_csv(csv_path, points)
    write_program_csv(program_csv_path)
    write_svg(svg_path, points)
    write_numbered_svg(numbered_svg_path, points)
    write_preview_png(png_path, points)
    write_numbered_png(numbered_png_path, points)

    print("generator: generated connected NTU glyph boundary")
    print("points:", len(points))
    print("length_m: {:.4f}".format(path_length(points)))
    print("closed_distance_m: {:.6f}".format(
        math.hypot(points[0]["x"] - points[-1]["x"],
                   points[0]["y"] - points[-1]["y"])))
    print("wrote:", csv_path)
    print("wrote:", program_csv_path)
    print("wrote:", svg_path)
    print("wrote:", numbered_svg_path)
    print("wrote:", png_path)
    print("wrote:", numbered_png_path)


if __name__ == "__main__":
    main()
