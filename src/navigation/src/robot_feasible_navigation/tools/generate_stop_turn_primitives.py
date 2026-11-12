#!/usr/bin/env python3
"""Generate this project's 1 cm forward/stop/turn lattice, without resampling one.

The JSON schema matches Nav2 Jazzy's lattice reader.  ``turning_radius`` is
positive for the reader's distance heuristic; it is not a minimum radius for
the explicit zero-translation rotations.  Disable analytic expansion when this
control set is used as an ordered stop/turn execution contract.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path


RESOLUTION = 0.01
MAX_TRANSLATION_SAMPLE = 0.001
MAX_ROTATION_SAMPLE = 0.0025
METADATA_RADIUS = 0.5
GENERATION_DATE = "2026-10-31"
# Primitive endpoints are integral lattice cells, not rounded sine/cosine.
DIRECTIONS = (
    (1, 0), (2, 1), (1, 1), (1, 2),
    (0, 1), (-1, 2), (-1, 1), (-2, 1),
    (-1, 0), (-2, -1), (-1, -1), (-1, -2),
    (0, -1), (1, -2), (1, -1), (2, -1),
)


def normalized(angle):
    return angle % math.tau


def heading_angles():
    return [normalized(math.atan2(y, x)) for x, y in DIRECTIONS]


def primitive(start, end, left, length, poses):
    """Keep all fields consumed by the stock Jazzy fromJsonToMotionPrimitive."""
    return {
        "trajectory_id": 0,
        "start_angle_index": start,
        "end_angle_index": end,
        "left_turn": left,
        "trajectory_radius": 0.0,
        "trajectory_length": length,
        "arc_length": 0.0,
        "straight_length": length,
        "poses": poses,
    }


def generate():
    angles = heading_angles()
    motions = []
    for start, (dx, dy) in enumerate(DIRECTIONS):
        x, y = dx * RESOLUTION, dy * RESOLUTION
        distance = math.hypot(x, y)
        count = math.ceil(distance / MAX_TRANSLATION_SAMPLE)
        forward = [[x * i / count, y * i / count, angles[start]]
                   for i in range(count + 1)]
        motions.append(primitive(start, start, False, distance, forward))
        for direction in (1, -1):
            end = (start + direction) % len(angles)
            delta = ((angles[end] - angles[start]) % math.tau
                     if direction > 0 else
                     -((angles[start] - angles[end]) % math.tau))
            count = math.ceil(abs(delta) / MAX_ROTATION_SAMPLE)
            turns = [[0.0, 0.0, normalized(angles[start] + delta * i / count)]
                     for i in range(count + 1)]
            # Store the canonical endpoint exactly, including the 2pi -> 0 wrap.
            turns[-1][2] = angles[end]
            motions.append(primitive(start, end, direction > 0, 0.0, turns))
    for number, motion in enumerate(motions):
        motion["trajectory_id"] = number
    return {
        "version": 1.0,
        "date_generated": GENERATION_DATE,
        "lattice_metadata": {
            "motion_model": "diff",
            "turning_radius": METADATA_RADIUS,
            "grid_resolution": RESOLUTION,
            "stopping_threshold": 5,
            "num_of_headings": len(angles),
            "heading_angles": angles,
            "number_of_trajectories": len(motions),
        },
        "primitives": motions,
    }


def encoded(data):
    return (json.dumps(data, indent=2, allow_nan=False) + "\n").encode()


def write_artifacts(output):
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = encoded(generate())
    output.write_bytes(payload)
    script = Path(__file__).resolve()
    provenance = {
        "artifact": output.name,
        "sha256": hashlib.sha256(payload).hexdigest(),
        "generator": "tools/" + script.name,
        "generator_sha256": hashlib.sha256(script.read_bytes()).hexdigest(),
        "generation_date": GENERATION_DATE,
        "origin": "Project-generated analytic integer-grid motions; not an official Nav2 primitive set.",
        "schema_reference": "Nav2 Jazzy fromJsonToMetaData/fromJsonToMotionPrimitive (utils.hpp)",
        "resolution_m": RESOLUTION,
        "heading_directions_cells": DIRECTIONS,
        "motions_per_heading": ["forward_straight", "counterclockwise_in_place", "clockwise_in_place"],
        "max_translation_sample_m": MAX_TRANSLATION_SAMPLE,
        "max_rotation_sample_rad": MAX_ROTATION_SAMPLE,
        "number_of_headings": 16,
        "number_of_trajectories": 48,
        "reverse_or_lateral_displacement": False,
        "metadata_radius_contract": "Positive heuristic radius only; explicit in-place motions have radius and XY length zero.",
        "required_planner_contract": "Use explicit primitive edges; no analytic expansion or reverse expansion.",
        "acceptance": "Static schema/geometry/connectivity only; runtime planning and physical execution are separate acceptance stages.",
    }
    output.with_suffix(".provenance.json").write_bytes(encoded(provenance))
    return provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] /
                        "params" / "stop_turn_1cm_h16.json")
    args = parser.parse_args()
    print(json.dumps(write_artifacts(args.output), indent=2))


if __name__ == "__main__":
    main()
