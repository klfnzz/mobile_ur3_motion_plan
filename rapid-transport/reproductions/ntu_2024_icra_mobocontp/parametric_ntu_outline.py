#!/usr/bin/env python
"""Parametric connected-NTU contour generator.

This module first builds a connected NTU solid glyph from code-defined
rectangles, one diagonal parallelogram, and one continuous U body whose bottom
is a tangent arc.  The T top bar overlaps the N right stroke and the U left
stroke, so the letters are one connected body.  The sampled target path is the
boundary of that generated body.  No external image, screenshot, or font
outline is read.
"""

from __future__ import print_function

import math

import numpy as np


DEFAULT_WIDTH_M = 3.0
DEFAULT_HEIGHT_M = 0.75
DEFAULT_STROKE_M = 0.10


def _segment_quad_metric(u0, v0, u1, v1, width_m, scale_u, scale_v):
    du_m = (u1 - u0) * scale_u
    dv_m = (v1 - v0) * scale_v
    norm = math.hypot(du_m, dv_m)
    if norm <= 1e-12:
        half_u = 0.5 * width_m / scale_u
        half_v = 0.5 * width_m / scale_v
        return [
            (u0 - half_u, v0 - half_v),
            (u0 + half_u, v0 - half_v),
            (u0 + half_u, v0 + half_v),
            (u0 - half_u, v0 + half_v),
        ]
    half_width = 0.5 * width_m
    nx_u = (-dv_m / norm) * half_width / scale_u
    ny_v = (du_m / norm) * half_width / scale_v
    return [
        (u0 + nx_u, v0 + ny_v),
        (u1 + nx_u, v1 + ny_v),
        (u1 - nx_u, v1 - ny_v),
        (u0 - nx_u, v0 - ny_v),
    ]


def _u_body_polygon(left_outer, left_inner, right_inner, right_outer,
                    top, bottom, outer_ry, stroke_v, arc_steps=96):
    center_u = 0.5 * (left_outer + right_outer)
    center_v = bottom - outer_ry
    outer_rx = 0.5 * (right_outer - left_outer)
    inner_rx = 0.5 * (right_inner - left_inner)
    inner_ry = max(outer_ry - stroke_v, 0.001)

    inner_arc = []
    for idx in range(arc_steps + 1):
        theta = math.pi * (1.0 - float(idx) / float(arc_steps))
        inner_arc.append((
            center_u + inner_rx * math.cos(theta),
            center_v + inner_ry * math.sin(theta),
        ))

    outer_arc = []
    for idx in range(arc_steps + 1):
        theta = math.pi * float(idx) / float(arc_steps)
        outer_arc.append((
            center_u + outer_rx * math.cos(theta),
            center_v + outer_ry * math.sin(theta),
        ))

    return (
        [
            (left_outer, top),
            (left_inner, top),
            (left_inner, center_v),
        ]
        + inner_arc[1:]
        + [
            (right_inner, top),
            (right_outer, top),
            (right_outer, center_v),
        ]
        + outer_arc[1:]
    )


def ntu_outline_program(scale_u=DEFAULT_WIDTH_M, scale_v=DEFAULT_HEIGHT_M,
                        stroke_m=DEFAULT_STROKE_M):
    """Return code-defined solid-glyph primitives before boundary extraction."""
    stroke_u = float(stroke_m) / float(scale_u)
    stroke_v = float(stroke_m) / float(scale_v)
    top = 0.000
    bottom = 1.000

    n_left_outer = 0.000
    n_right_outer = 0.300
    t_top_left = 0.315
    t_stem_left = 0.470
    u_left_outer = 0.655
    u_right_outer = 1.000
    u_left_inner = u_left_outer + stroke_u
    u_right_inner = u_right_outer - stroke_u
    t_top_right = u_left_inner
    u_outer_ry = 0.320
    return [
        ("rect", "N_left", n_left_outer, top, n_left_outer + stroke_u, bottom),
        ("poly", "N_diag", _segment_quad_metric(
            n_left_outer + 0.5 * stroke_u, top + 0.5 * stroke_v,
            n_right_outer + 0.5 * stroke_u, bottom - 0.5 * stroke_v,
            stroke_m, scale_u, scale_v,
        )),
        ("rect", "N_right", n_right_outer, top, n_right_outer + stroke_u,
         bottom),

        # T_left overlaps N_right; T_right overlaps U_left.
        ("rect", "T_top", t_top_left, top, t_top_right, top + stroke_v),
        ("rect", "T_stem", t_stem_left, top, t_stem_left + stroke_u, bottom),

        ("poly", "U_body", _u_body_polygon(
            u_left_outer, u_left_inner, u_right_inner, u_right_outer,
            top, bottom, u_outer_ry, stroke_v)),
    ]


def _uv_to_pixel(u, v, width_px, height_px, bounds):
    umin, vmin, umax, vmax = bounds
    x = (u - umin) / (umax - umin) * width_px
    y = (v - vmin) / (vmax - vmin) * height_px
    return x, y


def _pixel_to_uv(x, y, width_px, height_px, bounds):
    umin, vmin, umax, vmax = bounds
    u = umin + x / float(width_px) * (umax - umin)
    v = vmin + y / float(height_px) * (vmax - vmin)
    return u, v


def _paint_polygon(mask, vertices, bounds):
    height_px, width_px = mask.shape
    pixel_vertices = [
        _uv_to_pixel(u, v, width_px, height_px, bounds)
        for u, v in vertices
    ]
    y_min = max(0, int(math.floor(min(y for _, y in pixel_vertices))))
    y_max = min(height_px - 1, int(math.ceil(max(y for _, y in pixel_vertices))))
    edges = list(zip(pixel_vertices, pixel_vertices[1:] + pixel_vertices[:1]))
    for y in range(y_min, y_max + 1):
        scan_y = y + 0.5
        xs = []
        for (x0, y0), (x1, y1) in edges:
            if abs(y1 - y0) < 1e-12:
                continue
            if (y0 <= scan_y < y1) or (y1 <= scan_y < y0):
                alpha = (scan_y - y0) / (y1 - y0)
                xs.append(x0 + alpha * (x1 - x0))
        xs.sort()
        for left, right in zip(xs[0::2], xs[1::2]):
            x0 = max(0, int(math.ceil(left)))
            x1 = min(width_px - 1, int(math.floor(right)))
            if x0 <= x1:
                mask[y, x0:x1 + 1] = True


def _paint_rect(mask, u0, v0, u1, v1, bounds):
    _paint_polygon(mask, [
        (u0, v0),
        (u1, v0),
        (u1, v1),
        (u0, v1),
    ], bounds)


def _paint_arc_stroke(mask, center_u, center_v, radius_u, radius_v,
                      theta0, theta1, stroke_width, bounds):
    height_px, width_px = mask.shape
    arc_len = 0.5 * (abs(radius_u) + abs(radius_v)) * abs(theta1 - theta0)
    steps = max(48, int(math.ceil(arc_len / 0.003)))
    points = []
    for idx in range(steps + 1):
        alpha = float(idx) / float(steps)
        theta = (1.0 - alpha) * theta0 + alpha * theta1
        points.append((
            center_u + radius_u * math.cos(theta),
            center_v + radius_v * math.sin(theta),
        ))

    pad = 0.65 * stroke_width
    umin = min(u for u, _ in points) - pad
    umax = max(u for u, _ in points) + pad
    vmin = min(v for _, v in points) - pad
    vmax = max(v for _, v in points) + pad
    bx0, by0 = _uv_to_pixel(umin, vmin, width_px, height_px, bounds)
    bx1, by1 = _uv_to_pixel(umax, vmax, width_px, height_px, bounds)
    xlo = max(0, int(math.floor(min(bx0, bx1))))
    xhi = min(width_px, int(math.ceil(max(bx0, bx1))))
    ylo = max(0, int(math.floor(min(by0, by1))))
    yhi = min(height_px, int(math.ceil(max(by0, by1))))
    if xlo >= xhi or ylo >= yhi:
        return

    ys, xs = np.ogrid[ylo:yhi, xlo:xhi]
    umin_b, vmin_b, umax_b, vmax_b = bounds
    u = umin_b + (xs + 0.5) / float(width_px) * (umax_b - umin_b)
    v = vmin_b + (ys + 0.5) / float(height_px) * (vmax_b - vmin_b)
    min_dist = np.full((yhi - ylo, xhi - xlo), np.inf, dtype=float)
    for a, b in zip(points[:-1], points[1:]):
        u0, v0 = a
        u1, v1 = b
        du = u1 - u0
        dv = v1 - v0
        denom = du * du + dv * dv
        if denom <= 1e-18:
            dist = np.sqrt((u - u0) ** 2 + (v - v0) ** 2)
        else:
            t = ((u - u0) * du + (v - v0) * dv) / denom
            t = np.clip(t, 0.0, 1.0)
            closest_u = u0 + t * du
            closest_v = v0 + t * dv
            dist = np.sqrt((u - closest_u) ** 2 + (v - closest_v) ** 2)
        min_dist = np.minimum(min_dist, dist)
    mask[ylo:yhi, xlo:xhi] |= min_dist <= 0.5 * stroke_width


def _build_glyph_mask(width_px=2800, height_px=900,
                      scale_u=DEFAULT_WIDTH_M, scale_v=DEFAULT_HEIGHT_M,
                      stroke_m=DEFAULT_STROKE_M):
    bounds = (-0.040, -0.040, 1.040, 1.040)
    mask = np.zeros((height_px, width_px), dtype=bool)
    for primitive in ntu_outline_program(scale_u, scale_v, stroke_m):
        kind = primitive[0]
        if kind == "rect":
            _, _, u0, v0, u1, v1 = primitive
            _paint_rect(mask, u0, v0, u1, v1, bounds)
        elif kind == "poly":
            _, _, vertices = primitive
            _paint_polygon(mask, vertices, bounds)
        elif kind == "arc_stroke":
            _, _, cu, cv, ru, rv, theta0, theta1, stroke_width = primitive
            _paint_arc_stroke(
                mask, cu, cv, ru, rv, theta0, theta1, stroke_width, bounds)
        else:
            raise ValueError("unknown NTU glyph primitive: {}".format(kind))
    return mask, bounds


def _boundary_edges(mask):
    height_px, width_px = mask.shape
    edges = []
    for y in range(height_px):
        for x in range(width_px):
            if not mask[y, x]:
                continue
            if y == 0 or not mask[y - 1, x]:
                edges.append(((x, y), (x + 1, y)))
            if x == width_px - 1 or not mask[y, x + 1]:
                edges.append(((x + 1, y), (x + 1, y + 1)))
            if y == height_px - 1 or not mask[y + 1, x]:
                edges.append(((x + 1, y + 1), (x, y + 1)))
            if x == 0 or not mask[y, x - 1]:
                edges.append(((x, y + 1), (x, y)))
    return edges


def _trace_boundary_loops(edges):
    outgoing = {}
    for start, end in edges:
        outgoing.setdefault(start, []).append(end)
    loops = []
    guard_limit = len(edges) + 10
    while outgoing:
        start = next(iter(outgoing))
        current = start
        loop = [start]
        guard = 0
        while guard < guard_limit:
            guard += 1
            candidates = outgoing.get(current)
            if not candidates:
                break
            nxt = candidates.pop()
            if not candidates:
                del outgoing[current]
            loop.append(nxt)
            current = nxt
            if current == start:
                break
        if len(loop) > 4 and loop[-1] == loop[0]:
            loops.append(loop)
    return loops


def _polygon_area(points):
    area = 0.0
    for a, b in zip(points[:-1], points[1:]):
        area += a[0] * b[1] - b[0] * a[1]
    return 0.5 * area


def _point_line_distance(point, start, end):
    px, py = point
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    denom = dx * dx + dy * dy
    if denom <= 1e-18:
        return math.hypot(px - sx, py - sy)
    t = ((px - sx) * dx + (py - sy) * dy) / denom
    t = max(0.0, min(1.0, t))
    closest = (sx + t * dx, sy + t * dy)
    return math.hypot(px - closest[0], py - closest[1])


def _rdp(points, epsilon):
    if len(points) <= 2:
        return list(points)
    start = points[0]
    end = points[-1]
    max_distance = -1.0
    split_idx = 0
    for idx in range(1, len(points) - 1):
        distance = _point_line_distance(points[idx], start, end)
        if distance > max_distance:
            max_distance = distance
            split_idx = idx
    if max_distance > epsilon:
        left = _rdp(points[:split_idx + 1], epsilon)
        right = _rdp(points[split_idx:], epsilon)
        return left[:-1] + right
    return [start, end]


def _simplify_closed_loop(points, epsilon):
    closed = math.hypot(points[0][0] - points[-1][0],
                        points[0][1] - points[-1][1]) < 1e-12
    raw = list(points[:-1] if closed else points)
    start_idx = min(range(len(raw)), key=lambda idx: (raw[idx][0], -raw[idx][1]))
    rotated = raw[start_idx:] + raw[:start_idx]
    far_idx = max(
        range(1, len(rotated)),
        key=lambda idx: math.hypot(
            rotated[idx][0] - rotated[0][0],
            rotated[idx][1] - rotated[0][1],
        ),
    )
    first = rotated[:far_idx + 1]
    second = rotated[far_idx:] + [rotated[0]]
    simplified = _rdp(first, epsilon)[:-1] + _rdp(second, epsilon)[:-1]
    simplified.append(simplified[0])
    return simplified


def _rotate_start(points):
    raw = list(points[:-1])
    start_idx = min(range(len(raw)), key=lambda idx: (-raw[idx][1], raw[idx][0]))
    rotated = raw[start_idx:] + raw[:start_idx]
    rotated.append(rotated[0])
    return rotated


def _normalize_uv(points):
    u_values = [point[0] for point in points]
    v_values = [point[1] for point in points]
    umin, umax = min(u_values), max(u_values)
    vmin, vmax = min(v_values), max(v_values)
    du = max(umax - umin, 1e-12)
    dv = max(vmax - vmin, 1e-12)
    return [((u - umin) / du, (v - vmin) / dv) for u, v in points]


def _extract_glyph_outline_uv(scale_u=DEFAULT_WIDTH_M,
                              scale_v=DEFAULT_HEIGHT_M,
                              stroke_m=DEFAULT_STROKE_M):
    mask, bounds = _build_glyph_mask(
        scale_u=scale_u,
        scale_v=scale_v,
        stroke_m=stroke_m,
    )
    height_px, width_px = mask.shape
    loops = _trace_boundary_loops(_boundary_edges(mask))
    if not loops:
        raise RuntimeError("failed to extract generated NTU glyph boundary")
    loop = max(loops, key=lambda item: abs(_polygon_area(item)))
    points = [
        _pixel_to_uv(x, y, width_px, height_px, bounds)
        for x, y in loop
    ]
    if _polygon_area(points) < 0.0:
        points = list(reversed(points))
    points = _simplify_closed_loop(points, epsilon=0.0025)
    points = _rotate_start(points)
    return points


def _resample_closed_loop(points, max_step, scale_u, scale_v):
    output = []
    for a, b in zip(points[:-1], points[1:]):
        if not output:
            output.append(a)
        distance = math.hypot((b[0] - a[0]) * scale_u,
                              (b[1] - a[1]) * scale_v)
        steps = max(1, int(math.ceil(distance / float(max_step))))
        for idx in range(1, steps + 1):
            alpha = float(idx) / float(steps)
            output.append((
                (1.0 - alpha) * a[0] + alpha * b[0],
                (1.0 - alpha) * a[1] + alpha * b[1],
            ))
    return output


def sample_ntu_outline_uv(max_linear_step=0.012, max_arc_step=0.008,
                          scale_u=DEFAULT_WIDTH_M, scale_v=DEFAULT_HEIGHT_M,
                          normalize=True, stroke_m=DEFAULT_STROKE_M):
    """Sample the generated connected NTU glyph boundary into UV waypoints."""
    outline = _extract_glyph_outline_uv(
        scale_u=scale_u,
        scale_v=scale_v,
        stroke_m=stroke_m,
    )
    if normalize:
        outline = _normalize_uv(outline)
    sampled = _resample_closed_loop(
        outline,
        min(float(max_linear_step), float(max_arc_step)),
        scale_u,
        scale_v,
    )
    return [
        (u, v, "start/end" if idx == 0 or idx == len(sampled) - 1 else "")
        for idx, (u, v) in enumerate(sampled)
    ]


def uv_to_xy(u, v, origin_x, origin_y_top, width, height):
    return origin_x + width * u, origin_y_top - height * v


def sample_ntu_outline_xy(origin, width, height, max_linear_step=0.025,
                          max_arc_step=0.018, normalize=True,
                          stroke_m=DEFAULT_STROKE_M):
    uv_points = sample_ntu_outline_uv(
        max_linear_step=max_linear_step,
        max_arc_step=max_arc_step,
        scale_u=width,
        scale_v=height,
        normalize=normalize,
        stroke_m=stroke_m,
    )
    x0 = float(origin[0])
    y_top = float(origin[1])
    return [
        (uv_to_xy(u, v, x0, y_top, width, height)[0],
         uv_to_xy(u, v, x0, y_top, width, height)[1],
         label)
        for u, v, label in uv_points
    ]
