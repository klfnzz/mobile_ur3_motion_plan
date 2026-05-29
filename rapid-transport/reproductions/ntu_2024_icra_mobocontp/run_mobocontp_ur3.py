#!/usr/bin/env python
"""Reproduce the ICRA 2024 MoboConTP idea with the local UR3+base model.

This script creates a mobile printing/spraying-style task:

1. The end effector must follow a continuous, time-parametrized task-space
   trajectory.
2. The mobile base is planned in base configuration spacetime
   x = (t, base_x, base_y, base_yaw).
3. A MoboConTP-style backward dynamic-programming pass finds the minimum-cost
   base trajectory under admissible reachability, collision, and velocity
   constraints.
4. Optionally, OpenRAVE IK is used to recover a UR3 arm trajectory along the
   obtained base trajectory.

The implementation is intentionally standalone so the reproduction can live in
its own folder while reusing the project's OpenRAVE UR3/Ranger assets.
"""

from __future__ import print_function

import argparse
import csv
import json
import math
import os
import sys
import time
from collections import defaultdict

import numpy as np
import yaml

try:
    from scipy.optimize import least_squares
except ImportError:
    least_squares = None

try:
    import openravepy as orpy
except ImportError:
    orpy = None


ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
THIS_DIR = os.path.abspath(os.path.dirname(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from parametric_ntu_outline import sample_ntu_outline_xy


def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def angle_diff(a, b):
    return wrap_angle(a - b)


def normalized(vector, label="vector"):
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    if norm <= 1e-12:
        raise ValueError("{} must be nonzero.".format(label))
    return vector / norm


def tool_axis_index(axis_name):
    axis_name = str(axis_name).strip().lower()
    if axis_name == "x":
        return 0
    if axis_name == "y":
        return 1
    if axis_name == "z":
        return 2
    raise ValueError("tool axis must be x, y, or z.")


def rpy_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cr, -sr],
                   [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0],
                   [sy, cy, 0.0],
                   [0.0, 0.0, 1.0]])
    return rz.dot(ry).dot(rx)


def rotation_with_z_axis(z_axis, yaw_about_world_z=0.0):
    z_axis = normalized(z_axis, "z_axis")
    reference = np.array([math.cos(yaw_about_world_z),
                          math.sin(yaw_about_world_z), 0.0], dtype=float)
    if abs(float(np.dot(reference, z_axis))) > 0.95:
        reference = np.array([0.0, 1.0, 0.0], dtype=float)
    x_axis = reference - np.dot(reference, z_axis) * z_axis
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def rotation_with_axis(axis_name, world_axis, yaw_about_world_z=0.0):
    axis_name = str(axis_name).strip().lower()
    world_axis = normalized(world_axis, "world_axis")
    if axis_name == "z":
        return rotation_with_z_axis(world_axis, yaw_about_world_z)
    if axis_name == "x":
        x_axis = world_axis
        reference = np.array([math.cos(yaw_about_world_z),
                              math.sin(yaw_about_world_z), 0.0], dtype=float)
        if abs(float(np.dot(reference, x_axis))) > 0.95:
            reference = np.array([0.0, 1.0, 0.0], dtype=float)
        y_axis = reference - np.dot(reference, x_axis) * x_axis
        y_axis = y_axis / np.linalg.norm(y_axis)
        z_axis = np.cross(x_axis, y_axis)
        return np.column_stack((x_axis, y_axis, z_axis))
    if axis_name == "y":
        y_axis = world_axis
        reference = np.array([math.cos(yaw_about_world_z),
                              math.sin(yaw_about_world_z), 0.0], dtype=float)
        if abs(float(np.dot(reference, y_axis))) > 0.95:
            reference = np.array([1.0, 0.0, 0.0], dtype=float)
        x_axis = reference - np.dot(reference, y_axis) * y_axis
        x_axis = x_axis / np.linalg.norm(x_axis)
        z_axis = np.cross(x_axis, y_axis)
        return np.column_stack((x_axis, y_axis, z_axis))
    raise ValueError("tool axis must be x, y, or z.")


def axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm <= 1e-12:
        return np.eye(3)
    axis = axis / norm
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ], dtype=float)


def apply_tool_tilt(rotation, axis_name, tilt_deg):
    tilt = math.radians(float(tilt_deg))
    if abs(tilt) <= 1e-12:
        return rotation
    axis_name = str(axis_name).strip().lower()
    if axis_name == "z":
        tilt_axis = rotation[:, 0]
    elif axis_name == "x":
        tilt_axis = rotation[:, 1]
    else:
        tilt_axis = rotation[:, 0]
    return axis_angle_matrix(tilt_axis, tilt).dot(rotation)


def pose2d_to_transform(pose, z=0.0, yaw_offset=0.0):
    x, y, yaw = pose
    yaw = yaw + yaw_offset
    c = np.cos(yaw)
    s = np.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = np.array([[c, -s, 0.0],
                           [s, c, 0.0],
                           [0.0, 0.0, 1.0]])
    T[:3, 3] = [x, y, z]
    return T


def base_model_yaw_offset_deg(base_model_cfg):
    if "whole_body_yaw_offset_deg" in base_model_cfg:
        return float(base_model_cfg["whole_body_yaw_offset_deg"])
    return -float(base_model_cfg.get("front_yaw_at_identity_deg", 0.0))


def transform_to_pose2d(T):
    return np.array([T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0])],
                    dtype=float)


def make_transform(position, rotation):
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = position
    return T


def append_waypoint(points, point, atol=1e-9):
    point = np.asarray(point, dtype=float)
    if len(points) == 0 or np.linalg.norm(points[-1] - point) > atol:
        points.append(point)


def sample_polyline(points, samples_per_segment=8):
    points = [np.asarray(point, dtype=float) for point in points]
    samples_per_segment = max(1, int(samples_per_segment))
    sampled = []
    for a, b in zip(points[:-1], points[1:]):
        for idx in range(samples_per_segment):
            alpha = float(idx) / float(samples_per_segment)
            append_waypoint(sampled, (1.0 - alpha) * a + alpha * b)
    append_waypoint(sampled, points[-1])
    return sampled


def sample_catmull_rom(points, samples_per_segment=12):
    control = [np.asarray(point, dtype=float) for point in points]
    if len(control) < 2:
        return list(control)
    samples_per_segment = max(2, int(samples_per_segment))
    padded = [control[0]] + control + [control[-1]]
    sampled = []
    for idx in range(1, len(padded) - 2):
        p0 = padded[idx - 1]
        p1 = padded[idx]
        p2 = padded[idx + 1]
        p3 = padded[idx + 2]
        for step in range(samples_per_segment):
            t = float(step) / float(samples_per_segment)
            t2 = t * t
            t3 = t2 * t
            point = 0.5 * (
                (2.0 * p1)
                + (-p0 + p2) * t
                + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
            )
            append_waypoint(sampled, point)
    append_waypoint(sampled, control[-1])
    return sampled


def build_u_shape_layer_xy(origin_xy, width, depth, offsets):
    x0, y0 = origin_xy
    points = []
    for pass_idx, offset in enumerate(offsets):
        left_x = x0 + offset
        right_x = x0 + width - offset
        top_y = y0 - offset
        bottom_y = y0 - depth + offset
        if pass_idx % 2 == 0:
            pass_points = [
                [left_x, top_y],
                [left_x, bottom_y],
                [right_x, bottom_y],
                [right_x, top_y],
            ]
        else:
            pass_points = [
                [right_x, top_y],
                [right_x, bottom_y],
                [left_x, bottom_y],
                [left_x, top_y],
            ]
        for point in pass_points:
            append_waypoint(points, point)
    return points


def build_u_shape_layers_waypoints(path_cfg):
    origin = np.asarray(path_cfg["origin"], dtype=float)
    width = float(path_cfg["width"])
    depth = float(path_cfg["depth"])
    layers = int(path_cfg["layers"])
    if layers < 1:
        raise ValueError("u_shape_layers requires at least one layer.")
    height = float(path_cfg.get("height", 0.0))
    layer_height = float(path_cfg.get(
        "layer_height", height / float(layers) if layers > 0 else 0.0))
    offsets = [float(v) for v in path_cfg.get("offsets", [0.0])]
    if len(offsets) == 0:
        offsets = [0.0]
    alternate = bool(path_cfg.get("alternate_layers", True))

    layer_xy = build_u_shape_layer_xy(origin[:2], width, depth, offsets)
    waypoints = []
    for layer in range(layers):
        z = origin[2] + layer * layer_height
        xy_points = list(reversed(layer_xy)) if alternate and layer % 2 else layer_xy
        for xy in xy_points:
            append_waypoint(waypoints, [xy[0], xy[1], z])
    return np.asarray(waypoints, dtype=float)


def build_ntu_shape_layer_xy(path_cfg):
    origin = np.asarray(path_cfg["origin"], dtype=float)
    width = float(path_cfg["width"])
    height = float(path_cfg["letter_height"])
    gap = float(path_cfg.get("letter_gap", 0.15))
    letter_width = (width - 2.0 * gap) / 3.0
    x0, y_top = origin[:2]
    y_bottom = y_top - height

    n_left = x0
    n_right = x0 + letter_width
    t_left = x0 + letter_width + gap
    t_right = t_left + letter_width
    t_center = 0.5 * (t_left + t_right)
    u_left = x0 + 2.0 * (letter_width + gap)
    u_right = u_left + letter_width

    points = []
    for point in [
            [n_left, y_top],
            [n_left, y_bottom],
            [n_right, y_top],
            [n_right, y_bottom],
            [t_left, y_top],
            [t_right, y_top],
            [t_center, y_top],
            [t_center, y_bottom],
            [u_left, y_top],
            [u_left, y_bottom],
            [u_right, y_bottom],
            [u_right, y_top],
    ]:
        append_waypoint(points, point)

    base_length = float(np.linalg.norm(np.diff(np.asarray(points), axis=0),
                                       axis=1).sum())
    layers = int(path_cfg["layers"])
    build_height = float(path_cfg.get("build_height", 0.0))
    layer_height = float(path_cfg.get(
        "layer_height",
        build_height / float(max(1, layers - 1)),
    ))
    target_total = path_cfg.get("target_total_length")
    top_return_length = float(path_cfg.get("top_return_length", 0.0))
    if target_total is not None:
        vertical_length = max(0, layers - 1) * abs(layer_height)
        target_layer = (
            float(target_total) - vertical_length
        ) / float(max(1, layers))
        top_return_length = max(0.0, target_layer - base_length)

    if top_return_length > 1e-9:
        return_x = max(x0, min(u_right, u_right - top_return_length))
        append_waypoint(points, [return_x, y_top])
    return np.asarray(points, dtype=float)


def build_ntu_shape_layers_waypoints(path_cfg):
    origin = np.asarray(path_cfg["origin"], dtype=float)
    layers = int(path_cfg["layers"])
    if layers < 1:
        raise ValueError("ntu_shape_layers requires at least one layer.")
    build_height = float(path_cfg.get("build_height", 0.0))
    layer_height = float(path_cfg.get(
        "layer_height",
        build_height / float(max(1, layers - 1)),
    ))
    alternate = bool(path_cfg.get("alternate_layers", True))
    layer_xy = build_ntu_shape_layer_xy(path_cfg)

    waypoints = []
    for layer in range(layers):
        z = origin[2] + layer * layer_height
        xy_points = list(reversed(layer_xy)) if alternate and layer % 2 else layer_xy
        for xy in xy_points:
            append_waypoint(waypoints, [xy[0], xy[1], z])
    return np.asarray(waypoints, dtype=float)


def build_ntu_centerline_layer_xy(path_cfg):
    origin = np.asarray(path_cfg["origin"], dtype=float)
    width = float(path_cfg["width"])
    height = float(path_cfg["letter_height"])
    gap = float(path_cfg.get("letter_gap", 0.15))
    letter_width = (width - 2.0 * gap) / 3.0
    x0, y_top = origin[:2]
    y_bottom = y_top - height

    n_left = x0
    n_right = x0 + letter_width
    t_left = x0 + letter_width + gap
    t_right = t_left + letter_width
    t_center = 0.5 * (t_left + t_right)
    u_left = x0 + 2.0 * (letter_width + gap)
    u_right = u_left + letter_width

    points = []
    for point in [
            [n_left, y_bottom],
            [n_left, y_top],
            [n_right, y_bottom],
            [n_right, y_top],
            [t_left, y_top],
            [t_right, y_top],
            [t_center, y_top],
            [t_center, y_bottom],
            [t_center, y_top],
            [u_left, y_top],
            [u_left, y_bottom],
            [u_right, y_bottom],
            [u_right, y_top],
    ]:
        append_waypoint(points, point)
    return np.asarray(points, dtype=float)


def build_ntu_centerline_layers_waypoints(path_cfg):
    origin = np.asarray(path_cfg["origin"], dtype=float)
    layers = int(path_cfg["layers"])
    if layers < 1:
        raise ValueError("ntu_centerline_layers requires at least one layer.")
    build_height = float(path_cfg.get("build_height", 0.0))
    layer_height = float(path_cfg.get(
        "layer_height",
        build_height / float(max(1, layers - 1)),
    ))
    alternate = bool(path_cfg.get("alternate_layers", True))
    layer_xy = build_ntu_centerline_layer_xy(path_cfg)

    waypoints = []
    for layer in range(layers):
        z = origin[2] + layer * layer_height
        xy_points = list(reversed(layer_xy)) if alternate and layer % 2 else layer_xy
        for xy in xy_points:
            append_waypoint(waypoints, [xy[0], xy[1], z])
    return np.asarray(waypoints, dtype=float)


def build_ntu_art_layer_xy(path_cfg):
    origin = np.asarray(path_cfg["origin"], dtype=float)
    width = float(path_cfg["width"])
    height = float(path_cfg["letter_height"])
    stroke_m = float(path_cfg.get(
        "cad_stroke_width",
        path_cfg.get("visual_stroke_width", path_cfg.get("stroke_width", 0.10)),
    ))
    sampled = sample_ntu_outline_xy(
        origin[:2],
        width,
        height,
        max_linear_step=float(path_cfg.get("cad_linear_step", 0.025)),
        max_arc_step=float(path_cfg.get("cad_arc_step", 0.018)),
        normalize=True,
        stroke_m=stroke_m,
    )
    return np.asarray([[x, y] for x, y, _ in sampled], dtype=float)


def build_ntu_art_layers_waypoints(path_cfg):
    origin = np.asarray(path_cfg["origin"], dtype=float)
    layers = int(path_cfg["layers"])
    if layers < 1:
        raise ValueError("ntu_art_layers requires at least one layer.")
    build_height = float(path_cfg.get("build_height", 0.0))
    layer_height = float(path_cfg.get(
        "layer_height",
        build_height / float(max(1, layers - 1)),
    ))
    alternate = bool(path_cfg.get("alternate_layers", True))
    layer_xy = build_ntu_art_layer_xy(path_cfg)
    waypoints = []
    for layer in range(layers):
        z = origin[2] + layer * layer_height
        xy_points = list(reversed(layer_xy)) if alternate and layer % 2 else layer_xy
        for xy in xy_points:
            append_waypoint(waypoints, [xy[0], xy[1], z])
    return np.asarray(waypoints, dtype=float)


def ntu_letter_intervals(letter, y, letter_width, letter_height, stroke_width):
    intervals = []
    if letter == "N":
        intervals.extend([(0.0, stroke_width),
                          (letter_width - stroke_width, letter_width)])
        alpha = max(0.0, min(1.0, -y / letter_height))
        center = 0.5 * stroke_width + alpha * (letter_width - stroke_width)
        intervals.append((center - 0.5 * stroke_width,
                          center + 0.5 * stroke_width))
    elif letter == "T":
        if y >= -stroke_width:
            intervals.append((0.0, letter_width))
        center = 0.5 * letter_width
        intervals.append((center - 0.5 * stroke_width,
                          center + 0.5 * stroke_width))
    elif letter == "U":
        intervals.extend([(0.0, stroke_width),
                          (letter_width - stroke_width, letter_width)])
        if y <= -letter_height + stroke_width:
            intervals.append((0.0, letter_width))
    else:
        raise ValueError("Unsupported NTU letter '{}'.".format(letter))

    clipped = []
    for low, high in intervals:
        low = max(0.0, low)
        high = min(letter_width, high)
        if high > low:
            clipped.append((low, high))
    clipped.sort()
    merged = []
    for low, high in clipped:
        if not merged or low > merged[-1][1] + 1e-12:
            merged.append([low, high])
        else:
            merged[-1][1] = max(merged[-1][1], high)
    return [(low, high) for low, high in merged]


def build_ntu_filled_layer_xy(path_cfg):
    origin = np.asarray(path_cfg["origin"], dtype=float)
    width = float(path_cfg["width"])
    height = float(path_cfg["letter_height"])
    gap = float(path_cfg.get("letter_gap", 0.15))
    stroke_width = float(path_cfg.get("stroke_width", 0.16))
    rows = int(path_cfg.get("fill_rows", 4))
    rows = max(2, rows)
    group_by_letter = bool(path_cfg.get("group_by_letter", False))
    letter_width = (width - 2.0 * gap) / 3.0
    letters = [
        ("N", 0.0),
        ("T", letter_width + gap),
        ("U", 2.0 * (letter_width + gap)),
    ]
    y_values = np.linspace(0.0, -height, rows)
    points = []

    def append_segments(segments, forward):
        segments = sorted(segments)
        if not forward:
            segments = list(reversed(segments))
        for low, high, y_value in segments:
            if forward:
                append_waypoint(points, origin[:2] + np.array([low, y_value]))
                append_waypoint(points, origin[:2] + np.array([high, y_value]))
            else:
                append_waypoint(points, origin[:2] + np.array([high, y_value]))
                append_waypoint(points, origin[:2] + np.array([low, y_value]))

    if group_by_letter:
        for letter_idx, (letter, x_offset) in enumerate(letters):
            letter_rows = list(y_values)
            if letter_idx % 2 == 1:
                letter_rows = list(reversed(letter_rows))
            for row_idx, y in enumerate(letter_rows):
                segments = [
                    (x_offset + low, x_offset + high, y)
                    for low, high in ntu_letter_intervals(
                        letter, y, letter_width, height, stroke_width)
                ]
                append_segments(segments, (row_idx + letter_idx) % 2 == 0)
    else:
        for row_idx, y in enumerate(y_values):
            segments = []
            for letter, x_offset in letters:
                for low, high in ntu_letter_intervals(
                        letter, y, letter_width, height, stroke_width):
                    segments.append((x_offset + low, x_offset + high, y))
            append_segments(segments, row_idx % 2 == 0)

    target_total = path_cfg.get("target_total_length")
    layers = int(path_cfg["layers"])
    build_height = float(path_cfg.get("build_height", 0.0))
    layer_height = float(path_cfg.get(
        "layer_height",
        build_height / float(max(1, layers - 1)),
    ))
    if target_total is not None:
        layer_points = np.asarray(points, dtype=float)
        base_length = float(np.linalg.norm(np.diff(layer_points, axis=0),
                                           axis=1).sum())
        vertical_length = max(0, layers - 1) * abs(layer_height)
        target_layer = (
            float(target_total) - vertical_length
        ) / float(max(1, layers))
        extra = max(0.0, target_layer - base_length)
        if extra > 1e-9:
            direction = np.array([-1.0, 0.0]) if len(points) % 2 else np.array([1.0, 0.0])
            final = np.asarray(points[-1], dtype=float)
            x_min = origin[0]
            x_max = origin[0] + width
            remaining = extra
            while remaining > 1e-9:
                target_x = x_max if direction[0] > 0 else x_min
                travel = abs(target_x - final[0])
                if travel <= 1e-9:
                    direction *= -1.0
                    continue
                step = min(remaining, travel)
                final = final + direction * step
                append_waypoint(points, final)
                remaining -= step
                direction *= -1.0
    return np.asarray(points, dtype=float)


def build_ntu_filled_layers_waypoints(path_cfg):
    origin = np.asarray(path_cfg["origin"], dtype=float)
    layers = int(path_cfg["layers"])
    if layers < 1:
        raise ValueError("ntu_filled_layers requires at least one layer.")
    build_height = float(path_cfg.get("build_height", 0.0))
    layer_height = float(path_cfg.get(
        "layer_height",
        build_height / float(max(1, layers - 1)),
    ))
    alternate = bool(path_cfg.get("alternate_layers", True))
    layer_xy = build_ntu_filled_layer_xy(path_cfg)
    waypoints = []
    for layer in range(layers):
        z = origin[2] + layer * layer_height
        xy_points = list(reversed(layer_xy)) if alternate and layer % 2 else layer_xy
        for xy in xy_points:
            append_waypoint(waypoints, [xy[0], xy[1], z])
    return np.asarray(waypoints, dtype=float)


def resolve_path(path):
    if os.path.isabs(path):
        return path
    local_path = os.path.join(THIS_DIR, path)
    if os.path.exists(local_path):
        return os.path.abspath(local_path)
    return os.path.abspath(os.path.join(ROOT_DIR, path))


def resolve_reproduction_path(path):
    if os.path.isabs(path):
        return path
    return os.path.abspath(os.path.join(THIS_DIR, path))


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def normalize_ik_mode(mode):
    if isinstance(mode, bool):
        return "off" if mode is False else "load"
    mode = str(mode).strip().lower()
    if mode not in ("off", "load", "generate"):
        raise ValueError("Unknown IK mode '{}'. Expected off/load/generate.".format(
            mode))
    return mode


def frange_grid(vmin, vmax, step):
    count = int(round((vmax - vmin) / step))
    values = vmin + step * np.arange(count + 1, dtype=float)
    values[-1] = vmax
    return values


def build_task_waypoints(task_cfg):
    path_cfg = task_cfg["path"]
    path_type = path_cfg["type"]
    if path_type == "line":
        waypoints = np.asarray([path_cfg["start"], path_cfg["end"]], dtype=float)
    elif path_type == "polyline":
        waypoints = np.asarray(path_cfg["waypoints"], dtype=float)
        if waypoints.ndim != 2 or waypoints.shape[0] < 2 or waypoints.shape[1] != 3:
            raise ValueError("polyline path requires at least two 3D waypoints.")
    elif path_type == "u_shape_layers":
        waypoints = build_u_shape_layers_waypoints(path_cfg)
    elif path_type == "ntu_shape_layers":
        waypoints = build_ntu_shape_layers_waypoints(path_cfg)
    elif path_type == "ntu_centerline_layers":
        waypoints = build_ntu_centerline_layers_waypoints(path_cfg)
    elif path_type == "ntu_art_layers":
        waypoints = build_ntu_art_layers_waypoints(path_cfg)
    elif path_type == "ntu_filled_layers":
        waypoints = build_ntu_filled_layers_waypoints(path_cfg)
    else:
        raise NotImplementedError(
            "Unsupported path type '{}'. Use line, polyline, u_shape_layers, "
            "ntu_shape_layers, or ntu_art_layers.".format(
                path_type))
    return waypoints


def build_samples_from_waypoints(waypoints, duration, dt, orientation_rpy_deg):
    duration = float(duration)
    dt = float(dt)
    n_steps = int(round(duration / dt))
    if not np.isclose(n_steps * dt, duration):
        raise ValueError("task.duration must be an integer multiple of task.dt")

    segment_lengths = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    total_length = float(np.sum(segment_lengths))
    if total_length <= 1e-12:
        raise ValueError("End-effector path length must be positive.")
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))

    rpy = np.deg2rad(np.asarray(orientation_rpy_deg, dtype=float))
    R = rpy_matrix(rpy[0], rpy[1], rpy[2])

    samples = []
    for i in range(n_steps + 1):
        alpha = float(i) / float(n_steps)
        distance = alpha * total_length
        seg_idx = int(np.searchsorted(cumulative, distance, side="right") - 1)
        seg_idx = min(max(seg_idx, 0), len(segment_lengths) - 1)
        seg_start = cumulative[seg_idx]
        seg_len = segment_lengths[seg_idx]
        local_alpha = 0.0 if seg_len <= 1e-12 else (distance - seg_start) / seg_len
        position = (
            (1.0 - local_alpha) * waypoints[seg_idx]
            + local_alpha * waypoints[seg_idx + 1]
        )
        samples.append({
            "i": i,
            "t": i * dt,
            "alpha": alpha,
            "T_ee": make_transform(position, R),
        })
    return samples


def build_task_samples(task_cfg, waypoints=None):
    if waypoints is None:
        waypoints = build_task_waypoints(task_cfg)
    return build_samples_from_waypoints(
        waypoints,
        task_cfg["duration"],
        task_cfg["dt"],
        task_cfg["orientation_rpy_deg"],
    )


def base_grid_from_config(grid_cfg):
    x_values = frange_grid(float(grid_cfg["x_range"][0]),
                           float(grid_cfg["x_range"][1]),
                           float(grid_cfg["dx"]))
    y_values = frange_grid(float(grid_cfg["y_range"][0]),
                           float(grid_cfg["y_range"][1]),
                           float(grid_cfg["dy"]))
    yaw_values = np.deg2rad(
        frange_grid(float(grid_cfg["yaw_range_deg"][0]),
                    float(grid_cfg["yaw_range_deg"][1]),
                    float(grid_cfg["dyaw_deg"])))
    if len(yaw_values) > 1 and np.isclose(
            wrap_angle(yaw_values[0] - yaw_values[-1]), 0.0, atol=1e-12):
        yaw_values = yaw_values[:-1]
    nodes = []
    for ix, x in enumerate(x_values):
        for iy, y in enumerate(y_values):
            for iyaw, yaw in enumerate(yaw_values):
                nodes.append({
                    "key": (ix, iy, iyaw),
                    "pose": np.array([x, y, wrap_angle(yaw)], dtype=float),
                })
    return nodes


def build_admissible_controls(ctrl_cfg, dt):
    dvx = float(ctrl_cfg["dvx"])
    dvy = float(ctrl_cfg["dvy"])
    domega = math.radians(float(ctrl_cfg["domega_deg"]))
    vmax = float(ctrl_cfg["vmax"])
    omega_max = math.radians(float(ctrl_cfg["omega_max_deg"]))

    kx_max = int(math.floor(vmax / dvx + 1e-9))
    ky_max = int(math.floor(vmax / dvy + 1e-9))
    kw_max = int(math.floor(omega_max / domega + 1e-9))
    controls = []
    grid_dx = float(ctrl_cfg.get("grid_dx", dvx * dt))
    grid_dy = float(ctrl_cfg.get("grid_dy", dvy * dt))
    grid_dyaw = math.radians(float(ctrl_cfg.get(
        "grid_dyaw_deg", math.degrees(domega * dt))))
    for kx in range(-kx_max, kx_max + 1):
        for ky in range(-ky_max, ky_max + 1):
            for kw in range(-kw_max, kw_max + 1):
                vx = kx * dvx
                vy = ky * dvy
                omega = kw * domega
                if math.hypot(vx, vy) > vmax + 1e-12:
                    continue
                if abs(omega) > omega_max + 1e-12:
                    continue
                controls.append({
                    "delta_key": (
                        int(round((vx * dt) / grid_dx)),
                        int(round((vy * dt) / grid_dy)),
                        int(round((omega * dt) / grid_dyaw)),
                    ),
                    "delta_pose": np.array([vx * dt, vy * dt, omega * dt],
                                           dtype=float),
                    "velocity": np.array([vx, vy, omega], dtype=float),
                })
    return controls


def validate_discretization(cfg):
    dt = float(cfg["task"]["dt"])
    grid_cfg = cfg["base_grid"]
    ctrl_cfg = cfg["controls"]
    expected = [
        ("dx", float(grid_cfg["dx"]), float(ctrl_cfg["dvx"]) * dt),
        ("dy", float(grid_cfg["dy"]), float(ctrl_cfg["dvy"]) * dt),
        ("dyaw", math.radians(float(grid_cfg["dyaw_deg"])),
         math.radians(float(ctrl_cfg["domega_deg"])) * dt),
    ]
    for label, actual, target in expected:
        ratio = target / actual
        if not np.isclose(ratio, round(ratio), atol=1e-9, rtol=0):
            raise ValueError(
                "{} must evenly divide its velocity step times dt for "
                "grid-to-grid transitions. Got grid step {} and transition "
                "step {}.".format(label, actual, target))
    ctrl_cfg["grid_dx"] = float(grid_cfg["dx"])
    ctrl_cfg["grid_dy"] = float(grid_cfg["dy"])
    ctrl_cfg["grid_dyaw_deg"] = float(grid_cfg["dyaw_deg"])


def base_footprint_center(pose, footprint_cfg):
    offset = np.asarray(footprint_cfg.get("center_offset", [0.0, 0.0]),
                        dtype=float)
    yaw = pose[2]
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.asarray(pose[:2], dtype=float) + np.array([
        c * offset[0] - s * offset[1],
        s * offset[0] + c * offset[1],
    ], dtype=float)


def base_footprint_polygon(pose, footprint_cfg):
    shape = str(footprint_cfg.get("shape", "circle")).lower()
    center = base_footprint_center(pose, footprint_cfg)
    if shape != "box":
        radius = float(footprint_cfg.get("radius", 0.0))
        angles = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
        return np.column_stack((
            center[0] + radius * np.cos(angles),
            center[1] + radius * np.sin(angles),
        ))
    half = np.asarray(footprint_cfg["half_extents"], dtype=float)
    yaw = pose[2]
    ux = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
    uy = np.array([-math.sin(yaw), math.cos(yaw)], dtype=float)
    corners = []
    for sx in [-1.0, 1.0]:
        for sy in [-1.0, 1.0]:
            corners.append(center + sx * half[0] * ux + sy * half[1] * uy)
    return np.asarray([corners[i] for i in [0, 2, 3, 1]], dtype=float)


def oriented_box_intersects_axis_box(center, yaw, half, box_center, box_half):
    ux = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
    uy = np.array([-math.sin(yaw), math.cos(yaw)], dtype=float)
    delta = np.asarray(center, dtype=float) - np.asarray(box_center, dtype=float)
    axes = [
        np.array([1.0, 0.0], dtype=float),
        np.array([0.0, 1.0], dtype=float),
        ux,
        uy,
    ]
    half = np.asarray(half, dtype=float)
    box_half = np.asarray(box_half, dtype=float)
    for axis in axes:
        axis = axis / np.linalg.norm(axis)
        projected_delta = abs(float(np.dot(delta, axis)))
        projected_box = (
            box_half[0] * abs(float(axis[0]))
            + box_half[1] * abs(float(axis[1]))
        )
        projected_footprint = (
            half[0] * abs(float(np.dot(ux, axis)))
            + half[1] * abs(float(np.dot(uy, axis)))
        )
        if projected_delta > projected_box + projected_footprint + 1e-12:
            return False
    return True


def base_footprint_collides_obstacle(pose, obstacle, footprint_cfg):
    center = np.asarray(obstacle["center"], dtype=float)
    half = np.asarray(obstacle["half_extents"], dtype=float)
    clearance = float(obstacle.get("clearance", 0.0))
    inflated_half = half + clearance

    shape = str(footprint_cfg.get("shape", "circle")).lower()
    footprint_center = base_footprint_center(pose, footprint_cfg)
    if shape == "box":
        footprint_half = np.asarray(footprint_cfg["half_extents"], dtype=float)
        return oriented_box_intersects_axis_box(
            footprint_center, pose[2], footprint_half, center, inflated_half)

    radius = float(footprint_cfg.get("radius", 0.0))
    return np.all(np.abs(footprint_center - center) <= inflated_half + radius)


def obstacle_active_at(obstacle, t):
    active = obstacle.get("active_time", None)
    if active is None:
        return True
    start = -np.inf if active[0] is None else float(active[0])
    end = np.inf if active[1] is None else float(active[1])
    return start <= float(t) <= end


def collision_free_base(pose, collision_cfg, t=0.0):
    footprint_cfg = dict(collision_cfg.get("footprint", {}))
    if not footprint_cfg:
        footprint_cfg = {
            "shape": "circle",
            "radius": float(collision_cfg.get("base_radius", 0.0)),
            "center_offset": collision_cfg.get(
                "footprint_center_offset", [0.0, 0.0]),
        }
    for obstacle in collision_cfg.get("obstacles", []):
        if not obstacle_active_at(obstacle, t):
            continue
        if base_footprint_collides_obstacle(pose, obstacle, footprint_cfg):
            return False
    return True


def _perimeter_reference_points(T_ee, guidance_cfg):
    center = np.asarray(guidance_cfg.get("center", [0.0, 0.0]), dtype=float)
    half = np.asarray(guidance_cfg["half_extents"], dtype=float)
    track_offset = guidance_cfg.get("track_offset", 0.30)
    if np.isscalar(track_offset):
        offset = np.asarray([float(track_offset), float(track_offset)],
                            dtype=float)
    else:
        offset = np.asarray(track_offset, dtype=float)
    switch_margin = float(guidance_cfg.get("side_switch_margin", 0.08))
    clamp_margin = float(guidance_cfg.get("clamp_margin", 0.0))
    ee = np.asarray(T_ee[:3, 3], dtype=float)[:2]

    sides = {
        "left": abs(ee[0] - (center[0] - half[0])),
        "right": abs(ee[0] - (center[0] + half[0])),
        "bottom": abs(ee[1] - (center[1] - half[1])),
        "top": abs(ee[1] - (center[1] + half[1])),
    }
    best = min(sides.values())
    selected = [
        name for name, distance in sides.items()
        if distance <= best + switch_margin
    ]

    x_low = center[0] - half[0] - clamp_margin
    x_high = center[0] + half[0] + clamp_margin
    y_low = center[1] - half[1] - clamp_margin
    y_high = center[1] + half[1] + clamp_margin
    refs = []
    for side in selected:
        if side == "left":
            refs.append(np.array([
                center[0] - half[0] - offset[0],
                min(max(ee[1], y_low), y_high),
            ], dtype=float))
        elif side == "right":
            refs.append(np.array([
                center[0] + half[0] + offset[0],
                min(max(ee[1], y_low), y_high),
            ], dtype=float))
        elif side == "bottom":
            refs.append(np.array([
                min(max(ee[0], x_low), x_high),
                center[1] - half[1] - offset[1],
            ], dtype=float))
        elif side == "top":
            refs.append(np.array([
                min(max(ee[0], x_low), x_high),
                center[1] + half[1] + offset[1],
            ], dtype=float))
    return refs


def _side_band_reference_points(T_ee, guidance_cfg):
    center = np.asarray(guidance_cfg.get("center", [0.0, 0.0]), dtype=float)
    half = np.asarray(guidance_cfg["half_extents"], dtype=float)
    track_offset = guidance_cfg.get("track_offset", 0.20)
    if np.isscalar(track_offset):
        offset = np.asarray([float(track_offset), float(track_offset)],
                            dtype=float)
    else:
        offset = np.asarray(track_offset, dtype=float)
    corner_margin = float(guidance_cfg.get("corner_margin", 0.18))
    clamp_margin = float(guidance_cfg.get("clamp_margin", 0.0))
    split_y = float(guidance_cfg.get("split_y", center[1]))
    ee = np.asarray(T_ee[:3, 3], dtype=float)[:2]

    x_low = center[0] - half[0] - clamp_margin
    x_high = center[0] + half[0] + clamp_margin
    refs = []
    if ee[0] <= center[0] - half[0] + corner_margin:
        refs.append(np.array([
            center[0] - half[0] - offset[0],
            min(max(ee[1], center[1] - half[1]),
                center[1] + half[1]),
        ], dtype=float))
    if ee[0] >= center[0] + half[0] - corner_margin:
        refs.append(np.array([
            center[0] + half[0] + offset[0],
            min(max(ee[1], center[1] - half[1]),
                center[1] + half[1]),
        ], dtype=float))

    side_y = center[1] + half[1] + offset[1]
    if ee[1] < split_y:
        side_y = center[1] - half[1] - offset[1]
    refs.append(np.array([
        min(max(ee[0], x_low), x_high),
        side_y,
    ], dtype=float))
    return refs


def base_guidance_references(T_ee, guidance_cfg):
    guidance_type = guidance_cfg.get("type", "table_perimeter")
    if guidance_type == "table_perimeter":
        return _perimeter_reference_points(T_ee, guidance_cfg)
    if guidance_type == "table_side_band":
        return _side_band_reference_points(T_ee, guidance_cfg)
    raise ValueError("Unknown base_guidance type '{}'.".format(guidance_type))


def passes_base_guidance(base_pose, T_ee, guidance_cfg):
    if not guidance_cfg or not bool(guidance_cfg.get("enabled", False)):
        return True
    refs = base_guidance_references(T_ee, guidance_cfg)
    if guidance_cfg.get("mode", "hard") == "cost":
        return True
    tolerance = float(guidance_cfg.get("tolerance", 0.35))
    base_xy = np.asarray(base_pose[:2], dtype=float)
    return any(np.linalg.norm(base_xy - ref) <= tolerance for ref in refs)


def base_guidance_error(base_pose, T_ee, guidance_cfg):
    if not guidance_cfg or not bool(guidance_cfg.get("enabled", False)):
        return 0.0
    refs = base_guidance_references(T_ee, guidance_cfg)
    if not refs:
        return 0.0
    base_xy = np.asarray(base_pose[:2], dtype=float)
    return float(min(np.linalg.norm(base_xy - ref) for ref in refs))


def base_heading_error(base_pose, T_ee, heading_offset_deg=0.0):
    ee_pos = np.asarray(T_ee[:3, 3], dtype=float)
    desired_heading = math.atan2(ee_pos[1] - base_pose[1],
                                 ee_pos[0] - base_pose[0])
    heading = base_pose[2] + math.radians(float(heading_offset_deg))
    return float(abs(angle_diff(desired_heading, heading)))


def base_radial_error(base_pose, T_ee, target_radius):
    if target_radius is None:
        return 0.0
    base_xy = np.asarray(base_pose[:2], dtype=float)
    ee_pos = np.asarray(T_ee[:3, 3], dtype=float)
    radial = float(np.linalg.norm(ee_pos[:2] - base_xy))
    return radial - float(target_radius)


def passes_geometric_reachability(base_pose, T_ee, reach_cfg):
    base_xy = np.asarray(base_pose[:2], dtype=float)
    ee_pos = np.asarray(T_ee[:3, 3], dtype=float)
    radial = np.linalg.norm(ee_pos[:2] - base_xy)
    relative_height = ee_pos[2]

    if radial < float(reach_cfg["min_radius"]):
        return False
    if radial > float(reach_cfg["max_radius"]):
        return False
    if relative_height < float(reach_cfg["min_height"]):
        return False
    if relative_height > float(reach_cfg["max_height"]):
        return False

    desired_heading = math.atan2(ee_pos[1] - base_pose[1],
                                 ee_pos[0] - base_pose[0])
    heading_offset = math.radians(float(
        reach_cfg.get("heading_offset_deg", 0.0)))
    max_heading_error = math.radians(float(reach_cfg["max_heading_error_deg"]))
    if abs(angle_diff(desired_heading, base_pose[2] + heading_offset)) > max_heading_error:
        return False
    return True


class ValidVoxelCloud(object):
    """Task-independent valid voxel cloud in the mobile base frame."""

    def __init__(self, voxel_size, origin, shape, occupied, rotation_indices=None,
                 rotations=None):
        self.voxel_size = float(voxel_size)
        self.origin = np.asarray(origin, dtype=float)
        self.shape = tuple(int(v) for v in shape)
        self.occupied = np.asarray(occupied, dtype=bool)
        if rotation_indices is None:
            rotation_indices = -np.ones(self.shape, dtype=np.int16)
        self.rotation_indices = np.asarray(rotation_indices, dtype=np.int16)
        self.rotations = [] if rotations is None else [
            np.asarray(rotation, dtype=float) for rotation in rotations
        ]

    def index_for(self, relative_position):
        relative_position = np.asarray(relative_position, dtype=float)
        idx = np.rint((relative_position - self.origin) / self.voxel_size).astype(int)
        if np.any(idx < 0):
            return None
        if np.any(idx >= np.asarray(self.shape, dtype=int)):
            return None
        return tuple(int(v) for v in idx)

    def contains(self, relative_position):
        idx = self.index_for(relative_position)
        if idx is None:
            return False
        return bool(self.occupied[idx])

    def rotation_for(self, relative_position):
        idx = self.index_for(relative_position)
        if idx is None or not bool(self.occupied[idx]):
            return None
        rotation_idx = int(self.rotation_indices[idx])
        if rotation_idx < 0 or rotation_idx >= len(self.rotations):
            return None
        return self.rotations[rotation_idx]

    @property
    def valid_count(self):
        return int(np.count_nonzero(self.occupied))

    @property
    def total_count(self):
        return int(np.prod(np.asarray(self.shape, dtype=int)))


def _voxel_values(range_spec, voxel_size):
    return frange_grid(float(range_spec[0]), float(range_spec[1]), voxel_size)


def _voxel_cache_matches(data, voxel_size, origin, shape):
    return (
        np.isclose(float(data["voxel_size"]), voxel_size)
        and np.allclose(np.asarray(data["origin"], dtype=float), origin)
        and tuple(np.asarray(data["shape"], dtype=int)) == tuple(shape)
    )


def _ik_filter_options():
    if orpy is None:
        return None
    return orpy.IkFilterOptions.CheckEnvCollisions


def load_or_build_valid_voxel_cloud(cfg, samples, ik_checker):
    cloud_cfg = cfg.get("valid_voxel_cloud", {})
    if not bool(cloud_cfg.get("enabled", False)):
        return None, {"enabled": False}

    voxel_size = float(cloud_cfg.get("voxel_size", cloud_cfg.get("delta", 0.05)))
    x_values = _voxel_values(cloud_cfg["x_range"], voxel_size)
    y_values = _voxel_values(cloud_cfg["y_range"], voxel_size)
    z_values = _voxel_values(cloud_cfg["z_range"], voxel_size)
    origin = np.array([x_values[0], y_values[0], z_values[0]], dtype=float)
    shape = (len(x_values), len(y_values), len(z_values))
    cache_path = resolve_reproduction_path(
        cloud_cfg.get("cache", "outputs/valid_voxel_cloud.npz"))
    rebuild = bool(cloud_cfg.get("rebuild", False))

    if os.path.exists(cache_path) and not rebuild:
        data = np.load(cache_path, allow_pickle=False)
        if _voxel_cache_matches(data, voxel_size, origin, shape):
            cloud = ValidVoxelCloud(
                voxel_size,
                data["origin"],
                data["shape"],
                data["occupied"],
                data["rotation_indices"] if "rotation_indices" in data else None,
                data["rotations"] if "rotations" in data else None,
            )
            print("Loaded valid voxel cloud: {} / {} valid voxels from {}".format(
                cloud.valid_count, cloud.total_count, cache_path))
            return cloud, {
                "enabled": True,
                "source": "cache",
                "cache": cache_path,
                "valid_voxels": cloud.valid_count,
                "total_voxels": cloud.total_count,
            }
        print("Ignoring stale valid voxel cloud cache:", cache_path)

    if not ik_checker.available:
        raise RuntimeError(
            "valid_voxel_cloud.enabled requires OpenRAVE IK to build the cache. "
            "Run once with '--ik load' or '--ik generate'.")

    os.makedirs(os.path.dirname(cache_path), exist_ok=True)
    occupied = np.zeros(shape, dtype=bool)
    rotation_indices = -np.ones(shape, dtype=np.int16)
    base_pose = np.array([0.0, 0.0, 0.0], dtype=float)
    reference_rotation = samples[0]["T_ee"][:3, :3]
    rotations = [reference_rotation] + list(ik_checker.orientation_search)
    total = int(np.prod(np.asarray(shape, dtype=int)))
    checked = 0

    print("Building valid voxel cloud: {} voxels at delta={:.3f} m".format(
        total, voxel_size))
    for ix, x in enumerate(x_values):
        for iy, y in enumerate(y_values):
            for iz, z in enumerate(z_values):
                checked += 1
                for rotation_idx, rotation in enumerate(rotations):
                    T_ee = make_transform(np.array([x, y, z], dtype=float),
                                          rotation)
                    if ik_checker.solve(base_pose, T_ee) is not None:
                        occupied[ix, iy, iz] = True
                        rotation_indices[ix, iy, iz] = rotation_idx
                        break
        if ix == len(x_values) - 1 or (ix + 1) % 5 == 0:
            print("  voxel x-slice {}/{} checked, valid so far {}".format(
                ix + 1, len(x_values), int(np.count_nonzero(occupied))))

    np.savez_compressed(
        cache_path,
        voxel_size=np.asarray(voxel_size, dtype=float),
        origin=origin,
        shape=np.asarray(shape, dtype=int),
        occupied=occupied,
        rotation_indices=rotation_indices,
        rotations=np.asarray(rotations, dtype=float),
    )
    cloud = ValidVoxelCloud(voxel_size, origin, shape, occupied,
                            rotation_indices, rotations)
    print("Built valid voxel cloud: {} / {} valid voxels -> {}".format(
        cloud.valid_count, cloud.total_count, cache_path))
    return cloud, {
        "enabled": True,
        "source": "built",
        "cache": cache_path,
        "valid_voxels": cloud.valid_count,
        "total_voxels": cloud.total_count,
    }


def relative_ee_position_in_base(base_pose, T_ee):
    ee_pos = np.asarray(T_ee[:3, 3], dtype=float)
    dx = ee_pos[0] - base_pose[0]
    dy = ee_pos[1] - base_pose[1]
    c = math.cos(base_pose[2])
    s = math.sin(base_pose[2])
    return np.array([
        c * dx + s * dy,
        -s * dx + c * dy,
        ee_pos[2],
    ], dtype=float)


def passes_valid_voxel_cloud(base_pose, T_ee, cloud, cloud_cfg):
    relative_position = relative_ee_position_in_base(base_pose, T_ee)
    if "x_min" in cloud_cfg and relative_position[0] < float(cloud_cfg["x_min"]):
        return False
    if "x_max" in cloud_cfg and relative_position[0] > float(cloud_cfg["x_max"]):
        return False

    shoulder_offset = np.asarray(
        cloud_cfg.get("shoulder_offset", [0.0, 0.0, 0.0]), dtype=float)
    radius = np.linalg.norm(relative_position - shoulder_offset)
    if "r_min" in cloud_cfg and radius < float(cloud_cfg["r_min"]):
        return False
    if "r_max" in cloud_cfg and radius > float(cloud_cfg["r_max"]):
        return False
    if "max_heading_error_deg" in cloud_cfg:
        ee_pos = np.asarray(T_ee[:3, 3], dtype=float)
        desired_heading = math.atan2(ee_pos[1] - base_pose[1],
                                    ee_pos[0] - base_pose[0])
        heading_offset = math.radians(float(
            cloud_cfg.get("heading_offset_deg", 0.0)))
        max_heading_error = math.radians(float(
            cloud_cfg["max_heading_error_deg"]))
        if abs(angle_diff(desired_heading + heading_offset, base_pose[2])) > max_heading_error:
            return False
    return cloud.contains(relative_position)


def voxel_cloud_target_rotation(base_pose, T_ee, cloud):
    relative_position = relative_ee_position_in_base(base_pose, T_ee)
    return cloud.rotation_for(relative_position)


def _expand_degree_values(spec):
    if spec is None:
        return [0.0]
    if len(spec) == 3:
        start, stop, step = [float(v) for v in spec]
        if abs(step) <= 1e-12:
            raise ValueError("orientation search step must be nonzero.")
        values = []
        value = start
        if step > 0:
            while value <= stop + 1e-9:
                values.append(value)
                value += step
        else:
            while value >= stop - 1e-9:
                values.append(value)
                value += step
        return values
    return [float(v) for v in spec]


def _build_orientation_search(ik_cfg):
    search_cfg = ik_cfg.get("orientation_search", {})
    if not search_cfg or not bool(search_cfg.get("enabled", False)):
        return []
    if "tool_z_axis" in search_cfg:
        yaws = _expand_degree_values(search_cfg.get("yaw_deg", [-180.0, 180.0, 15.0]))
        tilts = _expand_degree_values(search_cfg.get("tilt_deg", [0.0]))
        tool_z_axis = np.asarray(search_cfg["tool_z_axis"], dtype=float)
        rotations = []
        for yaw in yaws:
            base_rotation = rotation_with_z_axis(tool_z_axis, math.radians(yaw))
            for tilt in tilts:
                rotations.append(apply_tool_tilt(base_rotation, "z", tilt))
        return rotations
    if "tool_axis" in search_cfg:
        yaws = _expand_degree_values(search_cfg.get("yaw_deg", [-180.0, 180.0, 15.0]))
        tilts = _expand_degree_values(search_cfg.get("tilt_deg", [0.0]))
        tool_axis = search_cfg["tool_axis"]
        world_axis = search_cfg.get("world_axis", [0.0, 0.0, -1.0])
        rotations = []
        for yaw in yaws:
            base_rotation = rotation_with_axis(tool_axis, world_axis,
                                               math.radians(yaw))
            for tilt in tilts:
                rotations.append(apply_tool_tilt(base_rotation, tool_axis, tilt))
        return rotations
    rolls = _expand_degree_values(search_cfg.get("roll_deg", [0.0]))
    pitches = _expand_degree_values(search_cfg.get("pitch_deg", [90.0]))
    yaws = _expand_degree_values(search_cfg.get("yaw_deg", [-180.0, 180.0, 15.0]))
    rotations = []
    for roll in rolls:
        for pitch in pitches:
            for yaw in yaws:
                rotations.append(
                    rpy_matrix(
                        math.radians(roll),
                        math.radians(pitch),
                        math.radians(yaw),
                    )
                )
    return rotations


class IkChecker(object):
    def __init__(self, env, robot, manip_name, mode, nominal_arm, ik_cfg=None):
        self.env = env
        self.robot = robot
        self.manip_name = manip_name
        self.mode = mode
        self.nominal_arm = np.asarray(nominal_arm, dtype=float)
        self.available = False
        self.cache = {}
        self.base_z = 0.0
        self.base_yaw_offset = 0.0
        self.filter_admissible = False
        self.orientation_search = []
        self.numeric_axis_refine_cfg = {}
        self.robot_collision_cache = {}

        if ik_cfg is not None:
            self.base_yaw_offset = math.radians(float(
                ik_cfg.get("base_yaw_offset_deg", 0.0)))
            self.filter_admissible = bool(ik_cfg.get("filter_admissible", False))
            self.orientation_search = _build_orientation_search(ik_cfg)
            self.numeric_axis_refine_cfg = dict(
                ik_cfg.get("numeric_axis_refine", {}))

        if self.robot is not None:
            self.robot.SetActiveManipulator(manip_name)
            self.base_z = get_robot_base_z(robot)

        if mode == "off":
            return
        if orpy is None:
            raise RuntimeError("OpenRAVE is required for IK mode '{}'.".format(mode))

        iktype = orpy.IkParameterization.Type.Transform6D
        ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(
            robot, iktype=iktype)
        loaded = ikmodel.load()
        if not loaded and mode == "generate":
            print("Generating IKFast for {}. This can take a few minutes...".format(
                manip_name))
            ikmodel.autogenerate()
            loaded = ikmodel.load()
        if not loaded:
            raise RuntimeError(
                "No IKFast cache for '{}'. Rerun with '--ik generate' or set "
                "ik.mode: off in the config.".format(manip_name))
        self.available = True

    def solutions(self, base_pose, T_ee, reference_q=None):
        if not self.available:
            return []
        rounded = tuple(np.round(np.r_[base_pose, T_ee[:3, 3], T_ee[:3, :3].ravel()], 5))
        use_cache = reference_q is None
        if use_cache and rounded in self.cache:
            cached = self.cache[rounded]
            return [] if cached is None else [cached]

        rotations = [T_ee[:3, :3]] + self.orientation_search
        q_reference = (
            np.asarray(reference_q, dtype=float)
            if reference_q is not None
            else self.nominal_arm
        )
        all_solutions = []
        with self.robot:
            self.robot.SetTransform(pose2d_to_transform(
                base_pose, self.base_z, self.base_yaw_offset))
            self.robot.SetActiveDOFValues(q_reference)
            manip = self.robot.GetManipulator(self.manip_name)
            for rotation in rotations:
                target = np.array(T_ee, dtype=float, copy=True)
                target[:3, :3] = rotation
                qsols = manip.FindIKSolutions(
                    target, orpy.IkFilterOptions.CheckEnvCollisions)
                if qsols is not None and len(qsols) > 0:
                    all_solutions.extend(np.asarray(qsols, dtype=float))
        if len(all_solutions) == 0:
            if use_cache:
                self.cache[rounded] = None
            return []
        all_solutions = np.asarray(all_solutions, dtype=float)
        rounded_solutions = {}
        for solution in all_solutions:
            rounded_solutions[tuple(np.round(solution, 8))] = solution
        all_solutions = np.asarray(list(rounded_solutions.values()), dtype=float)
        distances = np.linalg.norm(
            all_solutions - q_reference.reshape(1, -1), axis=1)
        order = np.argsort(distances)
        ordered_solutions = [all_solutions[int(idx)] for idx in order]
        if use_cache:
            self.cache[rounded] = ordered_solutions[0]
        return ordered_solutions

    def solve(self, base_pose, T_ee, reference_q=None):
        solutions = self.solutions(base_pose, T_ee, reference_q=reference_q)
        return None if len(solutions) == 0 else solutions[0]

    def has_openrave_scene(self):
        return self.env is not None and self.robot is not None

    def robot_collision_free(self, base_pose, q_arm=None, check_self=True,
                             use_cache=True):
        if not self.has_openrave_scene():
            return True
        q_values = (
            self.nominal_arm if q_arm is None
            else np.asarray(q_arm, dtype=float)
        )
        cache_key = None
        if use_cache:
            cache_key = (
                tuple(np.round(np.asarray(base_pose, dtype=float), 5)),
                tuple(np.round(q_values, 5)),
                bool(check_self),
            )
            if cache_key in self.robot_collision_cache:
                return self.robot_collision_cache[cache_key]

        with self.robot:
            self.robot.SetTransform(pose2d_to_transform(
                base_pose, self.base_z, self.base_yaw_offset))
            self.robot.SetActiveManipulator(self.manip_name)
            self.robot.SetActiveDOFValues(q_values)
            collides = self.env.CheckCollision(self.robot)
            if check_self:
                collides = collides or self.robot.CheckSelfCollision()

        ok = not collides
        if cache_key is not None:
            self.robot_collision_cache[cache_key] = ok
        return ok

    def numeric_axis_refine_enabled(self):
        return bool(self.numeric_axis_refine_cfg.get("enabled", False))

    def numeric_axis_refine(self, base_pose, T_ee, reference_q=None):
        if not self.available or not self.numeric_axis_refine_enabled():
            return None
        if least_squares is None:
            print("SciPy is not available; skipping numeric axis IK refinement.")
            return None

        cfg = self.numeric_axis_refine_cfg
        target_position = np.asarray(T_ee[:3, 3], dtype=float)
        tool_axis = cfg.get("tool_axis", "z")
        axis_idx = tool_axis_index(tool_axis)
        desired_axis = normalized(
            cfg.get("world_axis", [0.0, 0.0, -1.0]), "world_axis")
        q_seed = (
            np.asarray(reference_q, dtype=float)
            if reference_q is not None
            else self.nominal_arm
        )

        position_weight = float(cfg.get("position_weight", 30.0))
        axis_weight = float(cfg.get("axis_weight", 8.0))
        regularization_weight = float(cfg.get("regularization_weight", 0.03))
        max_nfev = int(cfg.get("max_nfev", 120))
        position_tolerance = float(cfg.get("position_tolerance", 0.004))
        axis_tolerance = math.radians(float(cfg.get("axis_tolerance_deg", 0.5)))
        collision_check = bool(cfg.get("collision_check", False))

        with self.robot:
            self.robot.SetTransform(pose2d_to_transform(
                base_pose, self.base_z, self.base_yaw_offset))
            self.robot.SetActiveManipulator(self.manip_name)
            lower, upper = self.robot.GetActiveDOFLimits()
            lower = np.asarray(lower, dtype=float)
            upper = np.asarray(upper, dtype=float)
            q_seed = np.minimum(np.maximum(q_seed, lower), upper)
            self.robot.SetActiveDOFValues(q_seed)
            manip = self.robot.GetManipulator(self.manip_name)

            def residual(q):
                self.robot.SetActiveDOFValues(q)
                T_now = manip.GetTransform()
                current_axis = T_now[:3, axis_idx]
                return np.r_[
                    position_weight * (T_now[:3, 3] - target_position),
                    axis_weight * (current_axis - desired_axis),
                    regularization_weight * wrap_angle(q - q_seed),
                ]

            result = least_squares(
                residual,
                q_seed,
                bounds=(lower, upper),
                max_nfev=max_nfev,
                xtol=1e-8,
                ftol=1e-8,
                gtol=1e-8,
            )
            q_solution = np.asarray(result.x, dtype=float)
            self.robot.SetActiveDOFValues(q_solution)
            T_solution = manip.GetTransform()
            position_error = np.linalg.norm(T_solution[:3, 3] - target_position)
            axis_dot = float(np.dot(T_solution[:3, axis_idx], desired_axis))
            axis_dot = max(-1.0, min(1.0, axis_dot))
            axis_error = math.acos(axis_dot)
            if position_error > position_tolerance or axis_error > axis_tolerance:
                return None
            if collision_check and (
                    self.env.CheckCollision(self.robot)
                    or self.robot.CheckSelfCollision()):
                return None
            return q_solution


def get_robot_base_z(robot):
    base_link = robot.GetLink("base_link")
    if base_link is not None:
        return float(base_link.GetTransform()[2, 3])
    return float(robot.GetTransform()[2, 3])


def load_openrave_scene(scene_path, robot_name):
    if orpy is None:
        raise RuntimeError("openravepy is not importable in this environment.")
    env = orpy.Environment()
    if os.environ.get("RAPID_TRANSPORT_HEADLESS", "1") == "1":
        env.SetDebugLevel(2)
    if not env.Load(scene_path):
        raise RuntimeError("Failed to load OpenRAVE scene: {}".format(scene_path))
    robot = env.GetRobot(robot_name)
    if robot is None:
        raise RuntimeError("Robot '{}' not found in {}".format(robot_name, scene_path))
    return env, robot


def build_admissible_spacetime(samples, grid_nodes, cfg, ik_checker,
                               voxel_cloud=None):
    stages = []
    cloud_cfg = cfg.get("valid_voxel_cloud", {})
    openrave_collision_cfg = cfg.get("openrave_collision", {})
    check_openrave_collision = bool(
        openrave_collision_cfg.get("enabled", False))
    openrave_collision_checks = 0
    openrave_collision_rejects = 0
    for sample in samples:
        admissible = {}
        T_ee = sample["T_ee"]
        for node in grid_nodes:
            pose = node["pose"]
            if not collision_free_base(
                    pose, cfg.get("collision", {}), t=sample["t"]):
                continue
            if not passes_base_guidance(
                    pose, T_ee, cfg.get("base_guidance", {})):
                continue
            if voxel_cloud is not None:
                if not passes_valid_voxel_cloud(pose, T_ee, voxel_cloud, cloud_cfg):
                    continue
                rotation = voxel_cloud_target_rotation(pose, T_ee, voxel_cloud)
                if rotation is not None:
                    T_ee = np.array(T_ee, dtype=float, copy=True)
                    T_ee[:3, :3] = rotation
            else:
                if not passes_geometric_reachability(
                        pose, T_ee, cfg["reachability"]):
                    continue
            q_arm = None
            if ik_checker.available and ik_checker.filter_admissible:
                q_arm = ik_checker.solve(pose, T_ee)
                if q_arm is None:
                    continue
            if check_openrave_collision:
                openrave_collision_checks += 1
                if not ik_checker.robot_collision_free(
                        pose,
                        q_arm=q_arm,
                        check_self=bool(
                            openrave_collision_cfg.get("check_self", True)),
                        use_cache=True):
                    openrave_collision_rejects += 1
                    continue
            admissible[node["key"]] = {
                "pose": pose,
                "T_ee": T_ee,
                "q_arm": q_arm,
                "guidance_error": base_guidance_error(
                    pose, T_ee, cfg.get("base_guidance", {})),
                "heading_error": base_heading_error(
                    pose, T_ee,
                    cfg["reachability"].get("heading_offset_deg", 0.0)),
                "radial_error": base_radial_error(
                    pose, T_ee, cfg["reachability"].get("target_radius")),
            }
        stages.append(admissible)
        print("stage {:02d}: {:4d} admissible base states".format(
            sample["i"], len(admissible)))
    if check_openrave_collision:
        print("OpenRAVE whole-robot collision checks: {}, rejected {}".format(
            openrave_collision_checks, openrave_collision_rejects))
    return stages


def pose_matches_constraint(pose, constraint, tolerance_xy, tolerance_yaw):
    target = np.asarray(constraint, dtype=float)
    if target.shape[0] < 2:
        raise ValueError("Base constraint must contain at least x and y.")
    if np.linalg.norm(pose[:2] - target[:2]) > tolerance_xy + 1e-12:
        return False
    if target.shape[0] >= 3:
        target_yaw = math.radians(target[2])
        if abs(angle_diff(pose[2], target_yaw)) > tolerance_yaw + 1e-12:
            return False
    return True


def apply_base_constraints(stages, cfg):
    constraint_cfg = cfg.get("base_constraints", {})
    if not constraint_cfg:
        return stages

    tolerance_xy = float(constraint_cfg.get("tolerance_xy", 0.0))
    tolerance_yaw = math.radians(float(
        constraint_cfg.get("tolerance_yaw_deg", 0.0)))
    stage_specs = defaultdict(list)
    if "start" in constraint_cfg:
        stage_specs[0].append(constraint_cfg["start"])
    if "goal" in constraint_cfg:
        stage_specs[len(stages) - 1].append(constraint_cfg["goal"])
    for item in constraint_cfg.get("stages", []):
        stage_specs[int(item["index"])].append(item["pose"])

    for stage_idx, constraints in sorted(stage_specs.items()):
        if stage_idx < 0 or stage_idx >= len(stages):
            raise ValueError("Base constraint stage {} is out of range.".format(
                stage_idx))
        before = len(stages[stage_idx])
        stages[stage_idx] = {
            key: data
            for key, data in stages[stage_idx].items()
            if any(pose_matches_constraint(
                data["pose"], constraint, tolerance_xy, tolerance_yaw)
                   for constraint in constraints)
        }
        print("constraint stage {:02d}: {:4d} -> {:4d} base states".format(
            stage_idx, before, len(stages[stage_idx])))
    return stages


def control_cost(delta_pose, dt, yaw_weight):
    vx = delta_pose[0] / dt
    vy = delta_pose[1] / dt
    omega = delta_pose[2] / dt
    return dt * (vx * vx + vy * vy + yaw_weight * omega * omega)


def mobocontp_backward_dp(stages, controls, dt, yaw_weight, num_yaw,
                          guidance_weight=0.0,
                          heading_weight=0.0,
                          radial_weight=0.0):
    n = len(stages) - 1
    if n < 1:
        raise ValueError("Need at least two trajectory samples.")
    if len(stages[-1]) == 0:
        return None, {"reason": "empty_goal_stage"}

    value = {n: {key: 0.0 for key in stages[-1]}}
    successor = {}
    feasible_counts = {n: len(stages[-1])}

    for i in range(n - 1, -1, -1):
        next_values = value[i + 1]
        value[i] = {}
        for key, data in stages[i].items():
            best_cost = np.inf
            best_next = None
            for control in controls:
                dk = control["delta_key"]
                next_key = (
                    key[0] + dk[0],
                    key[1] + dk[1],
                    (key[2] + dk[2]) % num_yaw,
                )
                if next_key not in next_values:
                    continue
                step_cost = control_cost(control["delta_pose"], dt, yaw_weight)
                if guidance_weight > 0.0:
                    guidance_error = stages[i + 1][next_key].get(
                        "guidance_error", 0.0)
                    step_cost += dt * guidance_weight * guidance_error * guidance_error
                if heading_weight > 0.0:
                    heading_error = stages[i + 1][next_key].get(
                        "heading_error", 0.0)
                    step_cost += dt * heading_weight * heading_error * heading_error
                if radial_weight > 0.0:
                    radial_error = stages[i + 1][next_key].get(
                        "radial_error", 0.0)
                    step_cost += dt * radial_weight * radial_error * radial_error
                total = step_cost + next_values[next_key]
                if total < best_cost:
                    best_cost = total
                    best_next = next_key
            if best_next is not None:
                value[i][key] = best_cost
                successor[(i, key)] = best_next
        feasible_counts[i] = len(value[i])
        print("backward stage {:02d}: {:4d} nodes can reach the goal".format(
            i, feasible_counts[i]))
        if len(value[i]) == 0:
            return None, {
                "reason": "infeasible_stage",
                "stage": i,
                "feasible_counts": feasible_counts,
            }

    start_key = min(value[0], key=lambda k: value[0][k])
    keys = [start_key]
    for i in range(n):
        keys.append(successor[(i, keys[-1])])
    return keys, {
        "cost": float(value[0][start_key]),
        "feasible_counts": feasible_counts,
    }


def recover_trajectory(keys, stages, samples):
    rows = []
    for i, key in enumerate(keys):
        data = stages[i][key]
        row = {
            "i": i,
            "t": samples[i]["t"],
            "base": data["pose"],
            "ee_position": samples[i]["T_ee"][:3, 3],
            "T_ee": data.get("T_ee", samples[i]["T_ee"]),
            "q_arm": data["q_arm"],
        }
        rows.append(row)
    return rows


def trajectory_heading_statistics(rows):
    if not rows:
        return {
            "max_deg": 0.0,
            "mean_deg": 0.0,
            "per_sample_deg": [],
        }
    errors = [
        math.degrees(base_heading_error(row["base"], row["T_ee"]))
        for row in rows
    ]
    return {
        "max_deg": float(max(errors)),
        "mean_deg": float(sum(errors) / len(errors)),
        "per_sample_deg": [float(error) for error in errors],
    }


def trajectory_openrave_collision_statistics(rows, ik_checker):
    stats = {
        "enabled": ik_checker.has_openrave_scene(),
        "env_collision_samples": [],
        "self_collision_samples": [],
    }
    if not stats["enabled"]:
        return stats
    for row in rows:
        with ik_checker.robot:
            ik_checker.robot.SetTransform(pose2d_to_transform(
                row["base"], ik_checker.base_z, ik_checker.base_yaw_offset))
            ik_checker.robot.SetActiveManipulator(ik_checker.manip_name)
            if row["q_arm"] is not None:
                ik_checker.robot.SetActiveDOFValues(row["q_arm"])
            if ik_checker.env.CheckCollision(ik_checker.robot):
                stats["env_collision_samples"].append(int(row["i"]))
            if ik_checker.robot.CheckSelfCollision():
                stats["self_collision_samples"].append(int(row["i"]))
    return stats


def fill_trajectory_ik(rows, ik_checker):
    if not ik_checker.available:
        return rows
    if ik_checker.numeric_axis_refine_enabled():
        refined = 0
        failed = []
        q_reference = ik_checker.nominal_arm
        tool_axis = ik_checker.numeric_axis_refine_cfg.get("tool_axis", "z")
        for row in rows:
            primary_seeds = []
            if row["q_arm"] is not None:
                primary_seeds.append(row["q_arm"])
            primary_seeds.append(q_reference)
            primary_seeds.append(ik_checker.nominal_arm)

            def unique(seed_candidates):
                unique_seeds = []
                seen = set()
                for seed in seed_candidates:
                    if seed is None:
                        continue
                    seed_key = tuple(np.round(np.asarray(seed, dtype=float), 7))
                    if seed_key in seen:
                        continue
                    seen.add(seed_key)
                    unique_seeds.append(np.asarray(seed, dtype=float))
                return unique_seeds

            q_refined = None
            for seed in unique(primary_seeds):
                q_refined = ik_checker.numeric_axis_refine(
                    row["base"], row["T_ee"], reference_q=seed)
                if q_refined is not None:
                    break

            if q_refined is None:
                seed_candidates = []
                seed_candidates.extend(ik_checker.solutions(
                    row["base"], row["T_ee"], reference_q=q_reference))
                seed_candidates.extend(ik_checker.solutions(
                    row["base"], row["T_ee"], reference_q=ik_checker.nominal_arm))
                for seed in unique(seed_candidates):
                    q_refined = ik_checker.numeric_axis_refine(
                        row["base"], row["T_ee"], reference_q=seed)
                    if q_refined is not None:
                        break

            if q_refined is not None:
                row["q_arm"] = q_refined
                q_reference = q_refined
                refined += 1
            else:
                row["q_arm"] = None
                failed.append(row["i"])
        print("Solved TCP {}-axis numeric IK for {}/{} trajectory samples.".format(
            tool_axis, refined, len(rows)))
        if failed:
            if bool(ik_checker.numeric_axis_refine_cfg.get(
                    "allow_partial", False)):
                fallback = ik_checker.nominal_arm
                for row in rows:
                    if row["q_arm"] is None:
                        candidates = [
                            np.asarray(fallback, dtype=float),
                            ik_checker.nominal_arm,
                        ]
                        chosen = None
                        for candidate in candidates:
                            if ik_checker.robot_collision_free(
                                    row["base"],
                                    q_arm=candidate,
                                    check_self=True,
                                    use_cache=False):
                                chosen = candidate
                                break
                        row["q_arm"] = (
                            np.asarray(chosen, dtype=float)
                            if chosen is not None
                            else np.asarray(ik_checker.nominal_arm, dtype=float)
                        )
                    else:
                        fallback = row["q_arm"]
                ik_checker.last_failed_numeric_ik = failed
                print("Keeping previous arm posture for failed samples: {}".format(
                    failed))
            else:
                raise RuntimeError(
                    "TCP {}-axis numeric IK failed for trajectory samples: {}".format(
                        tool_axis, failed))
        return rows

    candidates = []
    for row in rows:
        q_solutions = ik_checker.solutions(
            row["base"], row["T_ee"], reference_q=ik_checker.nominal_arm)
        candidates.append(q_solutions)
    solved = sum(1 for q_solutions in candidates if len(q_solutions) > 0)
    if solved == len(rows):
        values = [
            np.asarray([
                np.linalg.norm(q - ik_checker.nominal_arm) for q in candidates[0]
            ], dtype=float)
        ]
        parents = [[None for _ in candidates[0]]]
        for i in range(1, len(candidates)):
            stage_values = np.full(len(candidates[i]), np.inf, dtype=float)
            stage_parents = [None for _ in candidates[i]]
            for j, q in enumerate(candidates[i]):
                for k, prev_q in enumerate(candidates[i - 1]):
                    transition = np.linalg.norm(q - prev_q)
                    total = values[i - 1][k] + transition * transition
                    if total < stage_values[j]:
                        stage_values[j] = total
                        stage_parents[j] = k
            values.append(stage_values)
            parents.append(stage_parents)

        best_idx = int(np.argmin(values[-1]))
        chosen = [best_idx]
        for i in range(len(candidates) - 1, 0, -1):
            best_idx = parents[i][best_idx]
            chosen.append(best_idx)
        chosen.reverse()
        for row, q_solutions, idx in zip(rows, candidates, chosen):
            row["q_arm"] = q_solutions[idx]
    else:
        q_reference = ik_checker.nominal_arm
        for row in rows:
            q_arm = ik_checker.solve(
                row["base"], row["T_ee"], reference_q=q_reference)
            row["q_arm"] = q_arm
            if q_arm is not None:
                q_reference = q_arm

    print("Solved IKFast candidates for {}/{} trajectory samples.".format(
        solved, len(rows)))
    return rows


def interpolate_base_pose(a, b, alpha):
    base = (1.0 - alpha) * np.asarray(a, dtype=float) + alpha * np.asarray(b, dtype=float)
    base[2] = wrap_angle(a[2] + alpha * angle_diff(b[2], a[2]))
    return base


def interpolate_rows(rows, fine_dt):
    dense = []
    for a, b in zip(rows[:-1], rows[1:]):
        t0 = a["t"]
        t1 = b["t"]
        steps = max(1, int(round((t1 - t0) / fine_dt)))
        for j in range(steps):
            alpha = float(j) / float(steps)
            base = interpolate_base_pose(a["base"], b["base"], alpha)
            ee = (1.0 - alpha) * a["ee_position"] + alpha * b["ee_position"]
            dense.append({"t": t0 + alpha * (t1 - t0), "base": base, "ee": ee})
    dense.append({"t": rows[-1]["t"], "base": rows[-1]["base"],
                  "ee": rows[-1]["ee_position"]})
    return dense


def interpolate_animation_rows(rows, substeps):
    frames = []
    substeps = max(1, int(substeps))
    for a, b in zip(rows[:-1], rows[1:]):
        for j in range(substeps):
            alpha = float(j) / float(substeps)
            base = interpolate_base_pose(a["base"], b["base"], alpha)
            if a["q_arm"] is not None and b["q_arm"] is not None:
                q_arm = (1.0 - alpha) * a["q_arm"] + alpha * b["q_arm"]
            else:
                q_arm = a["q_arm"]
            frames.append({
                "t": (1.0 - alpha) * a["t"] + alpha * b["t"],
                "base": base,
                "q_arm": q_arm,
            })
    frames.append({
        "t": rows[-1]["t"],
        "base": rows[-1]["base"],
        "q_arm": rows[-1]["q_arm"],
    })
    return frames


def interpolate_fk_frames(rows, fine_dt):
    frames = []
    for a, b in zip(rows[:-1], rows[1:]):
        t0 = a["t"]
        t1 = b["t"]
        steps = max(1, int(round((t1 - t0) / fine_dt)))
        for j in range(steps):
            alpha = float(j) / float(steps)
            base = interpolate_base_pose(a["base"], b["base"], alpha)
            if a["q_arm"] is not None and b["q_arm"] is not None:
                q_arm = (1.0 - alpha) * a["q_arm"] + alpha * b["q_arm"]
            else:
                q_arm = a["q_arm"]
            frames.append({
                "t": t0 + alpha * (t1 - t0),
                "base": base,
                "q_arm": q_arm,
            })
    frames.append({
        "t": rows[-1]["t"],
        "base": rows[-1]["base"],
        "q_arm": rows[-1]["q_arm"],
    })
    return frames


def build_cartesian_tracking_rows(rows, waypoints, task_cfg, ik_checker, fine_dt):
    if not ik_checker.has_openrave_scene():
        return []
    target_samples = build_samples_from_waypoints(
        waypoints,
        task_cfg["duration"],
        fine_dt,
        task_cfg["orientation_rpy_deg"],
    )
    q_reference = (
        rows[0]["q_arm"]
        if rows and rows[0]["q_arm"] is not None
        else ik_checker.nominal_arm
    )
    tracking_rows = []
    for sample in target_samples:
        alpha = sample["alpha"]
        scaled = alpha * float(len(rows) - 1)
        idx = min(max(int(math.floor(scaled)), 0), len(rows) - 2)
        local = scaled - float(idx)
        base = interpolate_base_pose(rows[idx]["base"], rows[idx + 1]["base"],
                                     local)
        q_arm = ik_checker.numeric_axis_refine(
            base, sample["T_ee"], reference_q=q_reference)
        if q_arm is None:
            seed_solutions = ik_checker.solutions(
                base, sample["T_ee"], reference_q=q_reference)
            for seed in seed_solutions:
                q_arm = ik_checker.numeric_axis_refine(
                    base, sample["T_ee"], reference_q=seed)
                if q_arm is not None:
                    break
        if q_arm is None:
            raise RuntimeError(
                "Dense Cartesian TCP tracking IK failed at t={:.3f}s "
                "(alpha={:.5f}).".format(sample["t"], alpha))
        q_reference = q_arm
        tracking_rows.append({
            "i": sample["i"],
            "t": sample["t"],
            "alpha": alpha,
            "base": base,
            "ee_position": sample["T_ee"][:3, 3],
            "T_ee": sample["T_ee"],
            "q_arm": q_arm,
        })
    return tracking_rows


def compute_actual_tcp_rows(rows, ik_checker, fine_dt):
    if not ik_checker.has_openrave_scene():
        return []
    if any(row["q_arm"] is None for row in rows):
        return []
    frames = interpolate_fk_frames(rows, fine_dt)
    actual_rows = []
    with ik_checker.robot:
        ik_checker.robot.SetActiveManipulator(ik_checker.manip_name)
        manip = ik_checker.robot.GetManipulator(ik_checker.manip_name)
        for frame in frames:
            ik_checker.robot.SetTransform(pose2d_to_transform(
                frame["base"], ik_checker.base_z, ik_checker.base_yaw_offset))
            ik_checker.robot.SetActiveDOFValues(frame["q_arm"])
            T_tcp = manip.GetTransform()
            actual_rows.append({
                "t": frame["t"],
                "base": np.asarray(frame["base"], dtype=float),
                "ee": np.asarray(T_tcp[:3, 3], dtype=float).copy(),
            })
    return actual_rows


def tcp_tracking_error_statistics(actual_rows, target_dense_rows):
    count = min(len(actual_rows), len(target_dense_rows))
    if count == 0:
        return {
            "count": 0,
            "max_m": None,
            "mean_m": None,
            "max_index": None,
        }
    errors = [
        float(np.linalg.norm(actual_rows[idx]["ee"] - target_dense_rows[idx]["ee"]))
        for idx in range(count)
    ]
    max_index = int(np.argmax(errors))
    return {
        "count": int(count),
        "max_m": float(errors[max_index]),
        "mean_m": float(sum(errors) / float(count)),
        "max_index": max_index,
    }


def write_csv(path, rows, model_yaw_offset=0.0):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "i", "t",
            "base_x", "base_y", "base_yaw", "openrave_model_yaw",
            "ee_x", "ee_y", "ee_z",
            "q1", "q2", "q3", "q4", "q5", "q6",
        ])
        for row in rows:
            q_arm = row["q_arm"]
            q_values = ["" for _ in range(6)] if q_arm is None else list(q_arm)
            writer.writerow([
                row["i"], row["t"],
                row["base"][0], row["base"][1], row["base"][2],
                wrap_angle(row["base"][2] + model_yaw_offset),
                row["ee_position"][0], row["ee_position"][1], row["ee_position"][2],
            ] + q_values)


def write_dense_csv(path, rows, model_yaw_offset=0.0):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "base_x", "base_y", "base_yaw",
                         "openrave_model_yaw", "ee_x", "ee_y", "ee_z"])
        for row in rows:
            writer.writerow([
                row["t"], row["base"][0], row["base"][1], row["base"][2],
                wrap_angle(row["base"][2] + model_yaw_offset),
                row["ee"][0], row["ee"][1], row["ee"][2],
            ])


def write_actual_tcp_csv(path, actual_rows, target_dense_rows=None,
                         model_yaw_offset=0.0):
    target_dense_rows = target_dense_rows or []
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "t", "base_x", "base_y", "base_yaw", "openrave_model_yaw",
            "actual_ee_x", "actual_ee_y", "actual_ee_z",
            "target_ee_x", "target_ee_y", "target_ee_z", "error_m",
        ])
        for idx, row in enumerate(actual_rows):
            if idx < len(target_dense_rows):
                target = target_dense_rows[idx]["ee"]
                error = float(np.linalg.norm(row["ee"] - target))
                target_values = [target[0], target[1], target[2], error]
            else:
                target_values = ["", "", "", ""]
            writer.writerow([
                row["t"], row["base"][0], row["base"][1], row["base"][2],
                wrap_angle(row["base"][2] + model_yaw_offset),
                row["ee"][0], row["ee"][1], row["ee"][2],
            ] + target_values)


def waypoints_to_ee_rows(waypoints):
    return [{"ee": np.asarray(point, dtype=float)} for point in waypoints]


def write_target_path_csv(path, waypoints):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["idx", "ee_x", "ee_y", "ee_z"])
        for idx, point in enumerate(waypoints):
            writer.writerow([idx, point[0], point[1], point[2]])


def thin_sequence(sequence, max_count):
    sequence = list(sequence)
    if len(sequence) <= max_count:
        return sequence
    stride = int(math.ceil(float(len(sequence)) / float(max_count)))
    thinned = sequence[::stride]
    if thinned[-1] is not sequence[-1]:
        thinned.append(sequence[-1])
    return thinned


def write_svg(path, rows, cfg, dense_rows=None, target_rows=None,
              actual_tcp_rows=None):
    ee_rows = target_rows if target_rows is not None else dense_rows
    if ee_rows is None:
        ee_rows = [{"ee": row["ee_position"]} for row in rows]
    actual_tcp_rows = actual_tcp_rows or []
    x_values = (
        [row["base"][0] for row in rows]
        + [row["ee"][0] for row in ee_rows]
        + [row["ee"][0] for row in actual_tcp_rows]
    )
    y_values = (
        [row["base"][1] for row in rows]
        + [row["ee"][1] for row in ee_rows]
        + [row["ee"][1] for row in actual_tcp_rows]
    )
    obstacle_bounds = []
    for obstacle in cfg.get("collision", {}).get("obstacles", []):
        center = np.asarray(obstacle["center"], dtype=float)
        half = np.asarray(obstacle["half_extents"], dtype=float)
        clearance = float(obstacle.get("clearance", 0.0))
        inflated_half = half + clearance
        low = center - inflated_half
        high = center + inflated_half
        obstacle_bounds.append((low, high))
        x_values.extend([low[0], high[0]])
        y_values.extend([low[1], high[1]])
    margin = 48.0
    width = 960.0
    height = 640.0
    xmin, xmax = min(x_values) - 0.25, max(x_values) + 0.25
    ymin, ymax = min(y_values) - 0.25, max(y_values) + 0.25
    scale_x = (width - 2 * margin) / max(1e-9, xmax - xmin)
    scale_y = (height - 2 * margin) / max(1e-9, ymax - ymin)

    def sx(x):
        return margin + (x - xmin) / max(1e-9, xmax - xmin) * (width - 2 * margin)

    def sy(y):
        return height - margin - (y - ymin) / max(1e-9, ymax - ymin) * (height - 2 * margin)

    base_points = " ".join("{:.2f},{:.2f}".format(sx(row["base"][0]),
                                                  sy(row["base"][1]))
                           for row in rows)
    ee_points = " ".join("{:.2f},{:.2f}".format(sx(row["ee"][0]),
                                                sy(row["ee"][1]))
                         for row in thin_sequence(ee_rows, 6000))
    actual_tcp_points = " ".join("{:.2f},{:.2f}".format(
        sx(row["ee"][0]), sy(row["ee"][1]))
        for row in thin_sequence(actual_tcp_rows, 6000))
    path_cfg = cfg.get("task", {}).get("path", {})
    visual_stroke_m = float(path_cfg.get("visual_stroke_width", 0.0))
    print_stroke = visual_stroke_m * min(scale_x, scale_y)
    print_stroke = max(5.0, min(48.0, print_stroke))
    bead_radius = max(1.4, min(4.0, 0.12 * print_stroke))

    elements = []
    elements.append('<rect width="100%" height="100%" fill="#4a4c4b"/>')

    for low, high in obstacle_bounds:
        x = sx(low[0])
        y = sy(high[1])
        w = sx(high[0]) - sx(low[0])
        h = sy(low[1]) - sy(high[1])
        elements.append(
            '<rect x="{:.2f}" y="{:.2f}" width="{:.2f}" height="{:.2f}" '
            'fill="#5f6260" fill-opacity="0.72" stroke="#242625" '
            'stroke-width="2"/>'.format(
                x, y, w, h))

    elements.append('<polyline points="{}" fill="none" stroke="#826900" '
                    'stroke-width="{:.2f}" stroke-linecap="round" '
                    'stroke-linejoin="round" opacity="0.78"/>'.format(
                        ee_points, print_stroke + 5.0))
    elements.append('<polyline points="{}" fill="none" stroke="#f2c230" '
                    'stroke-width="{:.2f}" stroke-linecap="round" '
                    'stroke-linejoin="round"/>'.format(
                        ee_points, print_stroke))
    elements.append('<polyline points="{}" fill="none" stroke="#ffe06a" '
                    'stroke-width="{:.2f}" stroke-linecap="round" '
                    'stroke-linejoin="round" opacity="0.85"/>'.format(
                        ee_points, max(2.0, 0.28 * print_stroke)))
    if actual_tcp_points:
        elements.append('<polyline points="{}" fill="none" stroke="#05333c" '
                        'stroke-width="7" stroke-linecap="round" '
                        'stroke-linejoin="round" opacity="0.88"/>'.format(
                            actual_tcp_points))
        elements.append('<polyline points="{}" fill="none" stroke="#36e1ff" '
                        'stroke-width="3.5" stroke-linecap="round" '
                        'stroke-linejoin="round" opacity="0.96"/>'.format(
                            actual_tcp_points))
    elements.append('<polyline points="{}" fill="none" stroke="#ff6a2a" '
                    'stroke-width="4" stroke-linecap="round" '
                    'stroke-linejoin="round"/>'.format(base_points))

    footprint_cfg = cfg.get("collision", {}).get("footprint", {})

    for row in rows:
        elements.append('<circle cx="{:.2f}" cy="{:.2f}" r="4" fill="#cc4f24"/>'.format(
            sx(row["base"][0]), sy(row["base"][1])))
        elements.append('<circle cx="{:.2f}" cy="{:.2f}" r="{:.2f}" '
                        'fill="#ffd84d" fill-opacity="0.70"/>'.format(
                            sx(row["ee_position"][0]),
                            sy(row["ee_position"][1]),
                            bead_radius))
        yaw = row["base"][2]
        tip = row["base"][:2] + 0.07 * np.array([np.cos(yaw), np.sin(yaw)])
        elements.append('<line x1="{:.2f}" y1="{:.2f}" x2="{:.2f}" y2="{:.2f}" '
                        'stroke="#ffe8c7" stroke-width="2"/>'.format(
                            sx(row["base"][0]), sy(row["base"][1]),
                            sx(tip[0]), sy(tip[1])))
        if footprint_cfg:
            polygon = base_footprint_polygon(row["base"], footprint_cfg)
            points = " ".join("{:.2f},{:.2f}".format(sx(p[0]), sy(p[1]))
                              for p in polygon)
            elements.append(
                '<polygon points="{}" fill="none" stroke="#ffe8c7" '
                'stroke-width="1" stroke-opacity="0.55"/>'.format(points))

    elements.append('<text x="48" y="34" font-family="Arial" font-size="18" '
                    'fill="#f5f1df">UR3+Ranger MoboConTP reproduction</text>')
    elements.append('<text x="48" y="58" font-family="Arial" font-size="13" '
                    'fill="#d9d2b8">yellow: continuous NTU print trajectory, '
                    'cyan: actual TCP/FK trajectory, '
                    'orange: optimal base trajectory</text>')

    with open(path, "w") as f:
        f.write('<svg xmlns="http://www.w3.org/2000/svg" '
                'width="{:.0f}" height="{:.0f}" viewBox="0 0 {:.0f} {:.0f}">\n'.format(
                    width, height, width, height))
        f.write("\n".join(elements))
        f.write("\n</svg>\n")


def animate_openrave(env, robot, rows, manip_name, frame_delay=0.03, substeps=10,
                     dense_rows=None, target_rows=None, actual_tcp_rows=None,
                     base_yaw_offset=0.0):
    if orpy is None:
        return
    env.SetViewer("qtosg")
    base_z = get_robot_base_z(robot)
    robot.SetActiveManipulator(manip_name)

    ee_source = target_rows if target_rows is not None else dense_rows
    if ee_source is None:
        ee_source = [{"ee": row["ee_position"]} for row in rows]
    ee_source = thin_sequence(ee_source, 2500)
    ee_points = np.asarray([row["ee"] for row in ee_source], dtype=float)
    actual_tcp_rows = thin_sequence(actual_tcp_rows or [], 2500)
    actual_tcp_points = (
        np.asarray([row["ee"] for row in actual_tcp_rows], dtype=float)
        if actual_tcp_rows else None
    )
    base_points = np.asarray(
        [[row["base"][0], row["base"][1], base_z] for row in rows],
        dtype=float,
    )
    handles = [
        env.drawlinestrip(
            points=ee_points,
            linewidth=4.0,
            colors=np.array([0.1, 0.25, 0.85], dtype=float),
        ),
        env.drawlinestrip(
            points=base_points,
            linewidth=4.0,
            colors=np.array([0.85, 0.25, 0.1], dtype=float),
        ),
    ]
    if actual_tcp_points is not None and len(actual_tcp_points) > 1:
        handles.append(
            env.drawlinestrip(
                points=actual_tcp_points,
                linewidth=5.0,
                colors=np.array([0.0, 0.85, 1.0], dtype=float),
            )
        )

    frames = interpolate_animation_rows(rows, substeps=substeps)
    for frame in frames:
        robot.SetTransform(pose2d_to_transform(
            frame["base"], base_z, base_yaw_offset))
        if frame["q_arm"] is not None:
            robot.SetActiveDOFValues(frame["q_arm"])
        if hasattr(env, "UpdatePublishedBodies"):
            env.UpdatePublishedBodies()
        sys.stdout.write("\ranimating t={:.2f}s".format(frame["t"]))
        sys.stdout.flush()
        time.sleep(frame_delay)
    print("")
    if sys.stdin.isatty():
        input("Animation finished. Press Enter to close the OpenRAVE viewer.")
    else:
        time.sleep(2.0)
    return handles


def main():
    parser = argparse.ArgumentParser(
        description="Run the UR3+Ranger MoboConTP reproduction.")
    parser.add_argument("--config", default="config/printing_line.yaml",
                        help="Path to the reproduction YAML config.")
    parser.add_argument("--ik", choices=["off", "load", "generate"],
                        help="Override config ik.mode.")
    parser.add_argument("--view", action="store_true",
                        help="Animate the final trajectory in OpenRAVE.")
    parser.add_argument("--view-delay", type=float, default=0.03,
                        help="Seconds to sleep between OpenRAVE animation frames.")
    parser.add_argument("--view-substeps", type=int, default=10,
                        help="Interpolated animation frames between planner samples.")
    parser.add_argument("--fine-dt", type=float, default=0.1,
                        help="Interpolation step for dense CSV export.")
    args = parser.parse_args()

    cfg_path = resolve_path(args.config)
    cfg = load_config(cfg_path)
    if args.ik is not None:
        cfg.setdefault("ik", {})["mode"] = args.ik
    whole_body_yaw_offset_deg = base_model_yaw_offset_deg(
        cfg.get("base_model", {}))
    cfg.setdefault("ik", {})["base_yaw_offset_deg"] = whole_body_yaw_offset_deg

    scene_path = resolve_path(cfg["scene"])
    task_waypoints = build_task_waypoints(cfg["task"])
    target_rows = waypoints_to_ee_rows(task_waypoints)
    samples = build_task_samples(cfg["task"], waypoints=task_waypoints)
    validate_discretization(cfg)
    grid_nodes = base_grid_from_config(cfg["base_grid"])
    controls = build_admissible_controls(cfg["controls"], cfg["task"]["dt"])

    print("Loaded task samples:", len(samples))
    print("Base grid states per stage:", len(grid_nodes))
    print("Admissible controls:", len(controls))

    env = robot = None
    ik_mode = normalize_ik_mode(cfg.get("ik", {}).get("mode", "off"))
    cfg.setdefault("ik", {})["mode"] = ik_mode

    if ik_mode != "off" or args.view:
        env, robot = load_openrave_scene(scene_path, cfg["robot"])
    else:
        print("IK is off; OpenRAVE scene is not required for planning.")

    if env is not None:
        ik_checker = IkChecker(
            env,
            robot,
            cfg["manipulator"],
            ik_mode,
            cfg.get("ik", {}).get("nominal_arm", [0, 0, 0, 0, 0, 0]),
            cfg.get("ik", {}),
        )
    else:
        ik_checker = IkChecker(None, None, cfg["manipulator"], "off",
                               cfg.get("ik", {}).get("nominal_arm", [0, 0, 0, 0, 0, 0]),
                               cfg.get("ik", {}))

    voxel_cloud, voxel_report = load_or_build_valid_voxel_cloud(
        cfg, samples, ik_checker)
    stages = build_admissible_spacetime(
        samples, grid_nodes, cfg, ik_checker, voxel_cloud=voxel_cloud)
    stages = apply_base_constraints(stages, cfg)
    keys, report = mobocontp_backward_dp(
        stages,
        controls,
        float(cfg["task"]["dt"]),
        float(cfg["cost"]["yaw_weight"]),
        max(node["key"][2] for node in grid_nodes) + 1,
        float(cfg.get("base_guidance", {}).get("cost_weight", 0.0)),
        float(cfg["cost"].get("heading_weight", 0.0)),
        float(cfg["cost"].get("radial_weight", 0.0)),
    )
    if keys is None:
        print("MoboConTP reproduction is infeasible:", report)
        return 2

    rows = recover_trajectory(keys, stages, samples)
    heading_stats = trajectory_heading_statistics(rows)
    rows = fill_trajectory_ik(rows, ik_checker)
    openrave_collision_stats = trajectory_openrave_collision_statistics(
        rows, ik_checker)
    dense_rows = interpolate_rows(rows, args.fine_dt)
    tracking_rows = build_cartesian_tracking_rows(
        rows, task_waypoints, cfg["task"], ik_checker, args.fine_dt)
    tracking_collision_stats = trajectory_openrave_collision_statistics(
        tracking_rows, ik_checker)
    actual_tcp_rows = compute_actual_tcp_rows(tracking_rows, ik_checker,
                                              args.fine_dt)
    tracking_target_rows = [
        {"ee": np.asarray(row["ee_position"], dtype=float)}
        for row in tracking_rows
    ]
    tcp_tracking_stats = tcp_tracking_error_statistics(
        actual_tcp_rows, tracking_target_rows)

    output_cfg = cfg.get("output", {})
    output_dir = resolve_path(output_cfg.get("dir", "outputs"))
    basename = output_cfg.get("basename", "mobocontp_reproduction")
    os.makedirs(output_dir, exist_ok=True)
    csv_path = os.path.join(output_dir, basename + "_trajectory.csv")
    dense_csv_path = os.path.join(output_dir, basename + "_dense_base_ee.csv")
    actual_tcp_csv_path = os.path.join(output_dir, basename + "_actual_tcp.csv")
    tracking_csv_path = os.path.join(output_dir, basename + "_cartesian_tracking.csv")
    target_csv_path = os.path.join(output_dir, basename + "_target_path.csv")
    summary_path = os.path.join(output_dir, basename + "_summary.json")
    svg_path = os.path.join(output_dir, basename + "_top_view.svg")

    whole_body_yaw_offset = math.radians(whole_body_yaw_offset_deg)
    write_csv(csv_path, rows, model_yaw_offset=whole_body_yaw_offset)
    write_dense_csv(dense_csv_path, dense_rows,
                    model_yaw_offset=whole_body_yaw_offset)
    write_actual_tcp_csv(actual_tcp_csv_path, actual_tcp_rows,
                         target_dense_rows=tracking_target_rows,
                         model_yaw_offset=whole_body_yaw_offset)
    write_csv(tracking_csv_path, tracking_rows,
              model_yaw_offset=whole_body_yaw_offset)
    write_target_path_csv(target_csv_path, task_waypoints)
    write_svg(svg_path, rows, cfg, dense_rows=dense_rows,
              target_rows=target_rows, actual_tcp_rows=actual_tcp_rows)

    summary = {
        "paper_method": "MoboConTP-style backward dynamic programming in base spacetime",
        "scene": scene_path,
        "config": cfg_path,
        "ik_mode": cfg.get("ik", {}).get("mode", "off"),
        "num_task_samples": len(samples),
        "num_grid_states_per_stage": len(grid_nodes),
        "num_controls": len(controls),
        "base_constraints": cfg.get("base_constraints", {}),
        "valid_voxel_cloud": voxel_report,
        "vehicle_front_work_heading_error": heading_stats,
        "openrave_whole_robot_collision": openrave_collision_stats,
        "cartesian_tracking_openrave_collision": tracking_collision_stats,
        "actual_tcp_tracking_error": tcp_tracking_stats,
        "cost": report.get("cost"),
        "failed_numeric_ik_samples": getattr(
            ik_checker, "last_failed_numeric_ik", []),
        "feasible_counts": {
            str(k): int(v) for k, v in report.get("feasible_counts", {}).items()
        },
        "outputs": {
            "trajectory_csv": csv_path,
            "dense_csv": dense_csv_path,
            "actual_tcp_csv": actual_tcp_csv_path,
            "cartesian_tracking_csv": tracking_csv_path,
            "target_path_csv": target_csv_path,
            "top_view_svg": svg_path,
        },
    }
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2, sort_keys=True)

    print("Optimal base trajectory cost: {:.6f}".format(report["cost"]))
    print("Vehicle +X/work heading error: max {:.2f} deg, mean {:.2f} deg".format(
        heading_stats["max_deg"], heading_stats["mean_deg"]))
    if openrave_collision_stats.get("enabled", False):
        print("Whole-robot OpenRAVE collisions: env {}, self {}".format(
            openrave_collision_stats["env_collision_samples"],
            openrave_collision_stats["self_collision_samples"]))
    if tracking_collision_stats.get("enabled", False):
        print("Dense Cartesian tracking collisions: env {}, self {}".format(
            tracking_collision_stats["env_collision_samples"],
            tracking_collision_stats["self_collision_samples"]))
    if tcp_tracking_stats["count"] > 0:
        print("Actual TCP/FK tracking error: max {:.6f} m, mean {:.6f} m".format(
            tcp_tracking_stats["max_m"], tcp_tracking_stats["mean_m"]))
    print("Wrote:", csv_path)
    print("Wrote:", dense_csv_path)
    print("Wrote:", actual_tcp_csv_path)
    print("Wrote:", tracking_csv_path)
    print("Wrote:", target_csv_path)
    print("Wrote:", svg_path)
    print("Wrote:", summary_path)

    if args.view:
        if env is None or robot is None:
            env, robot = load_openrave_scene(scene_path, cfg["robot"])
        animate_openrave(
            env,
            robot,
            tracking_rows,
            cfg["manipulator"],
            frame_delay=args.view_delay,
            substeps=1,
            dense_rows=dense_rows,
            target_rows=target_rows,
            actual_tcp_rows=actual_tcp_rows,
            base_yaw_offset=whole_body_yaw_offset,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
