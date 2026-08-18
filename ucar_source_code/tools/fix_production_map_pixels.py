#!/usr/bin/env python3
"""Fix the production-field wall raster and regenerate its map previews.

The source field map uses 4 pixels for a wall.  The original rasterization
left the corner quadrant at several orthogonal joins empty and stopped three
open wall ends at the grid-point centre.  This tool applies the explicit
pixel corrections to the provincial map, applies the equivalent correction
after moving the 148-159 wall to 147-158 for the national maps, and updates
the numbered PNG previews without painting over their labels.
"""

from __future__ import print_function

import math
import os
from shutil import copyfile

from PIL import Image


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
MAP_DIR = os.path.join(ROOT, "ucar_ws", "src", "ucar_nav", "maps")
PROVINCE_PGM = os.path.join(
    MAP_DIR, "iflysse_field_walls_without_middle_vertices.pgm")
NATIONAL_PGM = os.path.join(MAP_DIR, "iflysse_field_walls_national.pgm")

ROOT_PNG = os.path.join(ROOT, "production_full_grid_all_numbered.png")
PROVINCE_PNG = os.path.join(
    ROOT, "ucar_ws", "src", "ucar_2026", "config",
    "production_full_grid_all_numbered.png")
NATIONAL_PNGS = (
    os.path.join(
        ROOT, "ucar_ws", "src", "ucar_2026_national", "config",
        "production_full_grid_all_numbered.png"),
    os.path.join(
        ROOT, "ucar_ws", "src", "ucar_2026_extra", "config",
        "production_full_grid_all_numbered.png"),
)


# Inclusive PGM rectangles.  The six 2x2 rectangles are the missing corner
# quadrants at the 136/138/140/141/149/151 orthogonal wall joins.
CORNER_PATCHES = (
    (198, 50, 199, 51),
    (300, 48, 301, 49),
    (398, 48, 399, 49),
    (450, 48, 451, 49),
    (298, 100, 299, 101),
    (400, 100, 401, 101),
)

# Each open end is extended by two rows, half of the four-pixel wall width.
# The source 139 segment stops at row 49, so its correction is rows 50-51.
# 148 is the provincial wall's upper end, and 152 is the right-hand wall's
# lower end.
PROVINCE_ENDPOINT_PATCHES = (
    (348, 50, 351, 51),  # 139
    (248, 98, 251, 99),  # 148
    (448, 100, 451, 101),  # 152
)
NATIONAL_ENDPOINT_PATCHES = (
    (348, 50, 351, 51),  # 139
    (198, 98, 201, 99),  # 147, moved from 148
    (448, 100, 451, 101),  # 152
)


def read_pgm(path):
    with open(path, "rb") as handle:
        data = bytearray(handle.read())

    offset = 0
    lines = []
    for _ in range(3):
        end = data.find(b"\n", offset)
        assert end >= 0, path
        lines.append(bytes(data[offset:end]).decode("ascii"))
        offset = end + 1
    assert lines[0] == "P5", path
    width, height = [int(value) for value in lines[1].split()]
    assert lines[2] == "255", path
    assert len(data) == offset + width * height, path
    return data, offset, width, height


def apply_pgm_patches(path, patches):
    data, offset, width, height = read_pgm(path)
    for x0, y0, x1, y1 in patches:
        assert 0 <= x0 <= x1 < width, (path, patches)
        assert 0 <= y0 <= y1 < height, (path, patches)
        for y in range(y0, y1 + 1):
            row = offset + y * width
            for x in range(x0, x1 + 1):
                assert data[row + x] in (0, 254), (path, x, y)
                data[row + x] = 0
    with open(path, "wb") as handle:
        handle.write(data)


def validate_wall_layout(path, wall_x, y0, y1, empty_x=None):
    data, offset, width, height = read_pgm(path)
    assert width == 500 and height == 600, path
    for y in range(y0, y1 + 1):
        for x in range(wall_x - 2, wall_x + 2):
            assert data[offset + y * width + x] == 0, (path, x, y)
    if empty_x is not None:
        for y in range(y0, y1 + 1):
            for x in range(empty_x - 2, empty_x + 2):
                assert data[offset + y * width + x] == 254, (path, x, y)


def png_box_from_pgm(image, x0, y0, x1, y1):
    # Keep this transform aligned with update_production_grid_assets.py.
    left, right = 39.0, image.width - 39.0
    top, bottom = 38.0, image.height - 38.0
    scale_x = (right - left) / 500.0
    scale_y = (bottom - top) / 600.0
    px0 = int(math.ceil(left + x0 * scale_x))
    py0 = int(math.ceil(top + y0 * scale_y))
    px1 = int(math.ceil(left + (x1 + 1) * scale_x)) - 1
    py1 = int(math.ceil(top + (y1 + 1) * scale_y)) - 1
    return px0, py0, px1, py1


def is_background(pixel):
    red, green, blue = pixel[:3]
    return red == green == blue and red >= 180


def paint_png_patches(image, patches):
    pixels = image.load()
    for patch in patches:
        x0, y0, x1, y1 = png_box_from_pgm(image, *patch)
        assert 0 <= x0 <= x1 < image.width, patch
        assert 0 <= y0 <= y1 < image.height, patch
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                pixel = pixels[x, y]
                if is_background(pixel):
                    pixels[x, y] = (0, 0, 0, 255) if image.mode == "RGBA" \
                        else (0, 0, 0)


def update_png(path, patches):
    image = Image.open(path).convert("RGBA" if path == ROOT_PNG else "RGB")
    paint_png_patches(image, patches)
    image.save(path)


def main():
    validate_wall_layout(PROVINCE_PGM, 250, 100, 147)
    validate_wall_layout(NATIONAL_PGM, 200, 100, 147, empty_x=250)

    apply_pgm_patches(
        PROVINCE_PGM, CORNER_PATCHES + PROVINCE_ENDPOINT_PATCHES)
    apply_pgm_patches(
        NATIONAL_PGM, CORNER_PATCHES + NATIONAL_ENDPOINT_PATCHES)

    update_png(ROOT_PNG, CORNER_PATCHES + PROVINCE_ENDPOINT_PATCHES)
    image = Image.open(ROOT_PNG).convert("RGB")
    image.save(PROVINCE_PNG)
    for path in NATIONAL_PNGS:
        update_png(path, CORNER_PATCHES + NATIONAL_ENDPOINT_PATCHES)

    validate_wall_layout(PROVINCE_PGM, 250, 98, 147)
    validate_wall_layout(NATIONAL_PGM, 200, 98, 147, empty_x=250)
    print("updated provincial and national production map pixels")


if __name__ == "__main__":
    main()
