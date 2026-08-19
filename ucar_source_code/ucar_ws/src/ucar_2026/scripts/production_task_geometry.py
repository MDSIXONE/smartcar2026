#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Pure geometry and grid-loading helpers for the 2026 production mission.

This module deliberately has no ROS imports so its coordinate and heading
contract can be tested on the development computer as well as the vehicle.
"""

from __future__ import print_function

import json
import math


DEFAULT_QR_OBSERVATION_NUMBERS = [262, 232, 295, 61, 41, 43]

DEFAULT_PRODUCTION_ROUTE = [
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    30, 29, 28, 27, 26, 25, 24, 23, 22, 21,
]
DEFAULT_PRODUCTION_OBSERVATION_HEADINGS_DEG = [
    -45, 45, -45, 45, -45, 45, -45, 45, -45, 45,
    -45, 45, -45, 45, -45, 45, -45, 45, -45, 45,
]
DEFAULT_PRODUCTION_ROUTE_GROUPS = [
    [11, 12, 21, 22],
    [13, 14, 23, 24],
    [15, 16, 25, 26],
    [17, 18, 27, 28],
    [19, 20, 29, 30],
]

# The primary zig-zag only covers the inner production lanes.  When a required
# category was not recognised there, the mission makes one complete perimeter
# pass instead of aborting: top row left-to-right, right side downward, bottom
# row right-to-left, then the left side upward.  Each target still receives a
# full 360-degree OCR turn; these headings merely start that turn facing into
# the middle production zone.
DEFAULT_FALLBACK_PRODUCTION_ROUTE = (
    list(range(1, 11)) + [20, 30, 40] + list(range(39, 30, -1)) + [21, 11]
)
DEFAULT_FALLBACK_PRODUCTION_OBSERVATION_HEADINGS_DEG = (
    [-90] * 10 + [180] * 3 + [90] * 9 + [0] * 2
)


def normalize_production_route_groups(raw_groups, route_numbers):
    """Validate and normalize the grouped OCR route definition."""
    if not isinstance(raw_groups, (list, tuple)) or not raw_groups:
        raise TaskDefinitionError("production route groups are empty")
    normalized = []
    flattened = []
    seen = set()
    for group_index, raw_group in enumerate(raw_groups):
        if not isinstance(raw_group, (list, tuple)) or not raw_group:
            raise TaskDefinitionError(
                "production route group %d is empty" % group_index)
        group = []
        for raw_number in raw_group:
            number = int(raw_number)
            if number in seen:
                raise TaskDefinitionError(
                    "production route point %d appears in multiple groups" %
                    number)
            seen.add(number)
            group.append(number)
            flattened.append(number)
        normalized.append(group)
    expected = [int(number) for number in route_numbers]
    if sorted(flattened) != sorted(expected):
        raise TaskDefinitionError(
            "production route groups do not cover production route points")
    return normalized


class TaskDefinitionError(ValueError):
    """Raised when the numbered-grid task configuration is incomplete."""


def is_finite(value):
    """Python 2-compatible finite-number check."""
    return not math.isnan(value) and not math.isinf(value)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def shortest_yaw_delta(current_yaw, target_yaw):
    """Return the signed shortest turn from current yaw to target yaw."""
    return normalize_angle(float(target_yaw) - float(current_yaw))


def bearing(source, target):
    """Return the map yaw from a source ``(x, y)`` to a target ``(x, y)``."""
    delta_x = float(target[0]) - float(source[0])
    delta_y = float(target[1]) - float(source[1])
    if abs(delta_x) < 1e-12 and abs(delta_y) < 1e-12:
        raise TaskDefinitionError("bearing requires two different points")
    return math.atan2(delta_y, delta_x)


def position_error(source, target):
    """Return planar Euclidean distance between two ``(x, y)`` positions."""
    return math.hypot(
        float(target[0]) - float(source[0]),
        float(target[1]) - float(source[1]))


def needs_recenter(error, trigger_tolerance):
    """Return whether post-turn drift is large enough to request correction."""
    return float(error) > float(trigger_tolerance)


def positive_turn_increment(previous_yaw, current_yaw, direction=1.0):
    """Return forward progress for a commanded signed turn across +/-pi."""
    signed_delta = normalize_angle(float(current_yaw) - float(previous_yaw))
    directed_delta = signed_delta if direction >= 0.0 else -signed_delta
    return max(0.0, directed_delta)


def build_straight_segments(
        route_numbers, points, angular_tolerance_radians=math.radians(1.0)):
    """Collapse consecutive forward-collinear route legs into endpoints.

    Each returned pair is ``(start_number, end_number)``.  A leg may extend
    the current segment only when its direction remains forward (positive
    dot product) and its signed cross/dot angle stays inside the supplied
    tolerance.  Repeated route points are rejected rather than silently
    producing a zero-length navigation leg.
    """
    route = [int(number) for number in route_numbers]
    tolerance = float(angular_tolerance_radians)
    if len(route) < 2:
        raise TaskDefinitionError("route requires at least two points")
    if not is_finite(tolerance) or tolerance < 0.0 or tolerance >= math.pi:
        raise TaskDefinitionError(
            "angular_tolerance_radians must be finite and in [0, pi)")

    seen = set()
    seen_coordinates = set()
    coordinates = []
    for number in route:
        if number in seen:
            raise TaskDefinitionError("duplicate route point %d" % number)
        seen.add(number)
        if number not in points:
            raise TaskDefinitionError("grid point %d is missing" % number)
        coordinate = points[number]
        try:
            x_value = float(coordinate[0])
            y_value = float(coordinate[1])
        except (IndexError, TypeError, ValueError) as exc:
            raise TaskDefinitionError(
                "invalid coordinate for point %d: %s" % (number, exc))
        if not is_finite(x_value) or not is_finite(y_value):
            raise TaskDefinitionError("grid point %d is not finite" % number)
        if (x_value, y_value) in seen_coordinates:
            raise TaskDefinitionError(
                "duplicate route coordinate at point %d" % number)
        seen_coordinates.add((x_value, y_value))
        coordinates.append((x_value, y_value))

    segments = []
    segment_start = 0
    for index in range(1, len(route) - 1):
        previous = coordinates[index - 1]
        current = coordinates[index]
        following = coordinates[index + 1]
        first_x = current[0] - previous[0]
        first_y = current[1] - previous[1]
        second_x = following[0] - current[0]
        second_y = following[1] - current[1]
        first_length = math.hypot(first_x, first_y)
        second_length = math.hypot(second_x, second_y)
        if first_length < 1e-12 or second_length < 1e-12:
            raise TaskDefinitionError(
                "route contains coincident consecutive points at %d" %
                route[index])
        dot = first_x * second_x + first_y * second_y
        cross = first_x * second_y - first_y * second_x
        angle = abs(math.atan2(cross, dot))
        if dot <= 0.0 or angle > tolerance:
            segments.append((route[segment_start], route[index]))
            segment_start = index
    segments.append((route[segment_start], route[-1]))
    return segments


def load_numbered_points(path):
    """Load and validate the numbered points in the supplied full-grid JSON."""
    try:
        with open(path, "r") as handle:
            document = json.load(handle)
    except (IOError, ValueError) as exc:
        raise TaskDefinitionError("cannot load grid file %s: %s" % (path, exc))

    raw_points = document.get("points")
    if not isinstance(raw_points, list):
        raise TaskDefinitionError("grid file has no points list")

    points = {}
    for raw_point in raw_points:
        try:
            number = int(raw_point["number"])
            x_value = float(raw_point["x_m"])
            y_value = float(raw_point["y_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise TaskDefinitionError("invalid grid point: %s" % exc)
        if number in points:
            raise TaskDefinitionError("duplicate grid point number %d" % number)
        if not is_finite(x_value) or not is_finite(y_value):
            raise TaskDefinitionError("grid point %d is not finite" % number)
        points[number] = (x_value, y_value)
    return points


def load_wall_reference_points(path):
    """Load only the explicitly approved middle-wall matching candidates."""
    try:
        with open(path, "r") as handle:
            document = json.load(handle)
    except (IOError, ValueError) as exc:
        raise TaskDefinitionError("cannot load grid file %s: %s" % (path, exc))
    raw_numbers = document.get("wall_reference_point_numbers")
    if not isinstance(raw_numbers, list) or not raw_numbers:
        raise TaskDefinitionError(
            "grid file has no wall_reference_point_numbers list")
    points = load_numbered_points(path)
    return require_points(points, raw_numbers)


def load_middle_zone_geometry(path):
    """Return ``(x_min, x_max, y_min, y_max, square_side_m)`` for the middle zone."""
    try:
        with open(path, "r") as handle:
            document = json.load(handle)
    except (IOError, ValueError) as exc:
        raise TaskDefinitionError("cannot load grid file %s: %s" % (path, exc))
    try:
        side_length = float(document["square_side_m"])
        bounds = document["middle_zone_bounds_m"]
        x_min, x_max = float(bounds["x"][0]), float(bounds["x"][1])
        y_min, y_max = float(bounds["y"][0]), float(bounds["y"][1])
    except (KeyError, IndexError, TypeError, ValueError) as exc:
        raise TaskDefinitionError(
            "grid file has invalid middle zone geometry: %s" % exc)
    if not is_finite(side_length) or side_length <= 0.0:
        raise TaskDefinitionError("square_side_m must be finite and positive")
    for value in (x_min, x_max, y_min, y_max):
        if not is_finite(value):
            raise TaskDefinitionError("middle zone bounds must be finite")
    if x_min >= x_max or y_min >= y_max:
        raise TaskDefinitionError("middle zone bounds are reversed")
    return (x_min, x_max, y_min, y_max, side_length)


def stop_point_for_wall_point(wall_coordinate, inside_offset_m, middle_bounds):
    """Return the parking pose ``(x, y)`` inside a wall point.

    ``middle_bounds`` is the ``(x_min, x_max, y_min, y_max)`` tuple returned by
    :func:`load_middle_zone_geometry`.  The wall point must lie on one of the
    four middle-zone boundaries; the stop point is the boundary coordinate
    displaced by ``inside_offset_m`` toward the inside of the zone.
    """
    x_min, x_max, y_min, y_max = middle_bounds
    wall_x = float(wall_coordinate[0])
    wall_y = float(wall_coordinate[1])
    if not is_finite(wall_x) or not is_finite(wall_y):
        raise TaskDefinitionError("wall point coordinate is not finite")
    if not is_finite(inside_offset_m) or inside_offset_m <= 0.0:
        raise TaskDefinitionError("inside_offset_m must be finite and positive")
    offset = float(inside_offset_m)
    tolerance = 1e-6
    if abs(wall_y - y_max) <= tolerance and x_min <= wall_x <= x_max:
        return (wall_x, y_max - offset)
    if abs(wall_y - y_min) <= tolerance and x_min <= wall_x <= x_max:
        return (wall_x, y_min + offset)
    if abs(wall_x - x_min) <= tolerance and y_min <= wall_y <= y_max:
        return (x_min + offset, wall_y)
    if abs(wall_x - x_max) <= tolerance and y_min <= wall_y <= y_max:
        return (x_max - offset, wall_y)
    raise TaskDefinitionError(
        "wall point (%.3f, %.3f) is not on the middle zone boundary" %
        (wall_x, wall_y))


def stop_point_for_measured_wall_hit(
        measured_hit, wall_reference_hit, inside_offset_m, middle_bounds):
    """Offset a measured wall hit inward using the map-selected wall side.

    ``wall_reference_hit`` identifies which rectangular boundary the map ray
    intersects.  The measured hit supplies both the along-wall coordinate and
    the actual wall distance, so a local map displacement does not directly
    become the processing stop position.
    """
    x_min, x_max, y_min, y_max = middle_bounds
    try:
        measured_x = float(measured_hit[0])
        measured_y = float(measured_hit[1])
        reference_x = float(wall_reference_hit[0])
        reference_y = float(wall_reference_hit[1])
        offset = float(inside_offset_m)
    except (IndexError, TypeError, ValueError) as exc:
        raise TaskDefinitionError(
            "invalid measured wall hit or wall reference: %s" % exc)
    if not all(is_finite(value) for value in (
            measured_x, measured_y, reference_x, reference_y, offset)):
        raise TaskDefinitionError(
            "measured wall hit and wall reference must be finite")
    if offset <= 0.0:
        raise TaskDefinitionError("inside_offset_m must be finite and positive")

    tolerance = 1e-6
    if (abs(reference_y - y_max) <= tolerance and
            x_min <= reference_x <= x_max):
        return measured_x, measured_y - offset
    if (abs(reference_y - y_min) <= tolerance and
            x_min <= reference_x <= x_max):
        return measured_x, measured_y + offset
    if (abs(reference_x - x_min) <= tolerance and
            y_min <= reference_y <= y_max):
        return measured_x + offset, measured_y
    if (abs(reference_x - x_max) <= tolerance and
            y_min <= reference_y <= y_max):
        return measured_x - offset, measured_y
    raise TaskDefinitionError(
        "wall reference (%.3f, %.3f) is not on the middle zone boundary" %
        (reference_x, reference_y))


def load_middle_target_guard_points(path, target_numbers):
    """Return the four numbered middle-grid vertices around each target.

    A production target is a 0.5 m middle-zone square centre.  Its guard is
    the four line endpoints or side-wall vertices one half cell away in X and
    Y.  Deriving this from the immutable grid file keeps the task route and
    guard labels in one coordinate contract instead of maintaining a second
    hand-written map.
    """
    try:
        with open(path, "r") as handle:
            document = json.load(handle)
    except (IOError, ValueError) as exc:
        raise TaskDefinitionError("cannot load grid file %s: %s" % (path, exc))

    try:
        side_length = float(document["square_side_m"])
        endpoint_range = document["numbering_scheme"][
            "middle_line_endpoints"]
        first_endpoint = int(endpoint_range[0])
        last_endpoint = int(endpoint_range[1])
        side_vertex_range = document["numbering_scheme"][
            "middle_side_wall_vertices"]
        first_side_vertex = int(side_vertex_range[0])
        last_side_vertex = int(side_vertex_range[1])
    except (KeyError, IndexError, TypeError, ValueError) as exc:
        raise TaskDefinitionError(
            "grid file has invalid middle guard metadata: %s" % exc)
    if not is_finite(side_length) or side_length <= 0.0:
        raise TaskDefinitionError("square_side_m must be finite and positive")
    if last_endpoint < first_endpoint:
        raise TaskDefinitionError("middle line endpoint range is reversed")
    if last_side_vertex < first_side_vertex:
        raise TaskDefinitionError(
            "middle side-wall vertex range is reversed")

    points = load_numbered_points(path)
    guard_numbers = (
        list(range(first_endpoint, last_endpoint + 1)) +
        list(range(first_side_vertex, last_side_vertex + 1)))
    guard_references = require_points(points, guard_numbers)
    half_side = side_length / 2.0
    tolerance = 1e-8
    result = {}
    for raw_target in target_numbers:
        target_number = int(raw_target)
        if target_number not in points:
            raise TaskDefinitionError("grid point %d is missing" % target_number)
        target_x, target_y = points[target_number]
        expected = (
            (target_x - half_side, target_y + half_side),
            (target_x + half_side, target_y + half_side),
            (target_x - half_side, target_y - half_side),
            (target_x + half_side, target_y - half_side),
        )
        guard_points = {}
        for expected_point in expected:
            matches = [
                number for number, coordinate in guard_references.items()
                if position_error(coordinate, expected_point) <= tolerance]
            if len(matches) != 1:
                raise TaskDefinitionError(
                    "target %d requires one middle endpoint at (%.3f, %.3f), "
                    "found %d" % (
                        target_number, expected_point[0], expected_point[1],
                        len(matches)))
            number = matches[0]
            guard_points[number] = guard_references[number]
        if len(guard_points) != 4:
            raise TaskDefinitionError(
                "target %d did not resolve four distinct guard points" %
                target_number)
        result[target_number] = guard_points
    return result


def require_points(points, numbers):
    """Return the requested points, rejecting a mission with missing labels."""
    resolved = {}
    for raw_number in numbers:
        number = int(raw_number)
        if number not in points:
            raise TaskDefinitionError("grid point %d is missing" % number)
        resolved[number] = points[number]
    return resolved
