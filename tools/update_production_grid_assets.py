#!/usr/bin/env python3
"""Complete middle-zone labels in the production grid JSON and PNG.

The existing 1-418 numbering is intentionally immutable.  This script appends
the middle line endpoints and the missing left/right wall references, then
adds their labels to the existing deterministic map image.
"""

from __future__ import print_function

import json
import os
import shutil

from PIL import Image, ImageDraw, ImageFont


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
ROOT_JSON = os.path.join(ROOT, "production_full_grid_all_numbered.json")
PACKAGE_JSON = os.path.join(
    ROOT, "ucar_ws", "src", "ucar_2026", "config",
    "production_full_grid_all_numbered.json")
PNG_PATH = os.path.join(ROOT, "production_full_grid_all_numbered.png")


def middle_points():
    points = []
    number = 419
    for row_boundary, y_value in ((4, 1.0), (5, 0.5), (6, 0.0)):
        for column_boundary, x_value in enumerate(
                [value * 0.5 for value in range(-4, 5)], 1):
            points.append({
                "number": number,
                "type": "vertex",
                "region": "middle",
                "role": "line_endpoint",
                "x_m": x_value,
                "y_m": y_value,
                "row_boundary_from_top": row_boundary,
                "column_boundary_from_left": column_boundary,
            })
            number += 1

    for y_value in (1.0, 0.5, 0.0):
        for side, x_value in (("left", -2.5), ("right", 2.5)):
            points.append({
                "number": number,
                "type": "vertex",
                "region": "middle",
                "role": "wall_reference",
                "wall_side": side,
                "x_m": x_value,
                "y_m": y_value,
            })
            number += 1

    for y_value in (1.25, 0.75, 0.25, -0.25):
        for side, x_value in (("left", -2.5), ("right", 2.5)):
            points.append({
                "number": number,
                "type": "edge_midpoint",
                "orientation": "vertical",
                "region": "middle",
                "role": "wall_reference",
                "wall_side": side,
                "x_m": x_value,
                "y_m": y_value,
            })
            number += 1
    assert number == 460
    return points


def is_middle_wall_reference(point):
    if point.get("type") not in ("vertex", "edge_midpoint"):
        return False
    x_value = float(point["x_m"])
    y_value = float(point["y_m"])
    on_vertical = (
        abs(abs(x_value) - 2.5) < 1e-9 and -0.5 <= y_value <= 1.5)
    on_horizontal = (
        abs(y_value - 1.5) < 1e-9 or abs(y_value + 0.5) < 1e-9)
    return on_vertical or on_horizontal


def update_document(path):
    with open(path, "r", encoding="utf-8") as handle:
        document = json.load(handle)

    additions = middle_points()
    document["points"] = [
        point for point in document["points"] if int(point["number"]) <= 418
    ] + additions
    document["description"] = (
        "Full 0.5 m grid covering the complete map. Existing labels 1-418 "
        "remain unchanged; middle line endpoints use 419-445 and the missing "
        "left/right middle-wall references use 446-459.")
    document["numbering_scheme"]["middle_line_endpoints"] = [419, 445]
    document["numbering_scheme"]["middle_side_wall_vertices"] = [446, 451]
    document["numbering_scheme"]["middle_side_wall_edge_midpoints"] = [452, 459]
    document["numbering_scheme"]["notes"] = [
        "Center numbers 1-40 match the original middle zone.",
        "Center numbers 41-120 are the added outside-square centers.",
        "Existing vertex and edge-midpoint numbers 121-418 are immutable.",
        "Middle line endpoints and side-wall references append 419-459.",
    ]
    document["counts"]["all_numbered_points"] = len(document["points"])
    document["counts"]["middle_line_endpoints"] = 27
    document["counts"]["middle_side_wall_vertices"] = 6
    document["counts"]["middle_side_wall_edge_midpoints"] = 8
    document["counts"]["all_vertices"] = sum(
        point["type"] == "vertex" for point in document["points"])
    document["counts"]["all_edge_midpoints"] = sum(
        point["type"] == "edge_midpoint" for point in document["points"])

    wall_numbers = [
        int(point["number"]) for point in document["points"]
        if is_middle_wall_reference(point)
    ]
    document["wall_reference_point_numbers"] = sorted(wall_numbers)

    grouped = document.setdefault("grouped_points", {})
    grouped["middle_line_endpoints"] = [
        point for point in additions if point.get("role") == "line_endpoint"
    ]
    grouped["middle_side_wall_points"] = [
        point for point in additions if point.get("role") == "wall_reference"
    ]

    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        json.dump(document, handle, ensure_ascii=False, indent=2)
        handle.write("\n")


def load_font(size):
    candidates = (
        "C:/Windows/Fonts/arialbd.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    )
    for path in candidates:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def map_to_pixel(image, x_value, y_value):
    # The source uses an approximately 38 px margin and 300 px per map metre.
    left, right = 39.0, image.width - 39.0
    top, bottom = 38.0, image.height - 38.0
    x_pixel = left + (float(x_value) + 2.5) / 5.0 * (right - left)
    y_pixel = top + (3.0 - float(y_value)) / 6.0 * (bottom - top)
    return int(round(x_pixel)), int(round(y_pixel))


def draw_label(draw, image, point):
    x_pixel, y_pixel = map_to_pixel(
        image, point["x_m"], point["y_m"])
    is_midpoint = point["type"] == "edge_midpoint"
    color = (232, 111, 18, 255) if is_midpoint else (126, 62, 145, 255)
    text = str(point["number"])
    font = load_font(8)
    text_box = draw.textbbox((0, 0), text, font=font)
    width = max(24, text_box[2] - text_box[0] + 6)
    height = 11

    if abs(float(point["x_m"])) == 2.5:
        x0 = x_pixel + 5 if point["x_m"] < 0 else x_pixel - width - 5
        y0 = y_pixel - height // 2
    else:
        x0 = x_pixel + 7
        y0 = y_pixel - 15
    x0 = max(1, min(image.width - width - 1, x0))
    y0 = max(1, min(image.height - height - 1, y0))
    draw.rounded_rectangle(
        (x0, y0, x0 + width, y0 + height), radius=4, fill=color)
    text_x = x0 + (width - (text_box[2] - text_box[0])) / 2.0
    text_y = y0 + (height - (text_box[3] - text_box[1])) / 2.0 - 1
    draw.text((text_x, text_y), text, font=font, fill=(255, 255, 255, 255))


def update_png():
    image = Image.open(PNG_PATH).convert("RGBA")
    draw = ImageDraw.Draw(image)
    for point in middle_points():
        draw_label(draw, image, point)
    image.save(PNG_PATH)


def main():
    update_document(ROOT_JSON)
    shutil.copyfile(ROOT_JSON, PACKAGE_JSON)
    update_png()
    print("updated %s, %s, and %s" % (
        ROOT_JSON, PACKAGE_JSON, PNG_PATH))


if __name__ == "__main__":
    main()
