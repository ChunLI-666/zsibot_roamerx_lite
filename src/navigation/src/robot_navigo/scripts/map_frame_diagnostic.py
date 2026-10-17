#!/usr/bin/env python3
"""Read-only local 2D map compatibility diagnostic; does not establish pose truth.

Fits occupied cell centers with trimmed nearest-neighbor SE(2) ICP initialized
at identity. Different scans, unknown cells and partial coverage limit inference.
Never apply this estimated transform automatically to localization references.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.spatial import cKDTree


def occupied_points(path):
    metadata = yaml.safe_load(path.read_text())
    image_path = path.parent / metadata["image"]
    pixels = np.asarray(Image.open(image_path).convert("L"), dtype=float)
    probability = pixels / 255.0 if metadata.get("negate", 0) else 1 - pixels / 255.0
    rows, columns = np.nonzero(probability > metadata["occupied_thresh"])
    resolution = float(metadata["resolution"])
    origin = metadata["origin"]
    points = np.column_stack(((columns + .5) * resolution,
                              (pixels.shape[0] - rows - .5) * resolution))
    c, s = np.cos(origin[2]), np.sin(origin[2])
    rotation = np.array([[c, -s], [s, c]])
    points = points @ rotation.T + np.asarray(origin[:2])
    provenance = {"yaml": str(path.resolve()), "image": str(image_path.resolve()),
                  "yaml_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                  "image_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
                  "occupied_cells": len(points), "resolution": resolution}
    if len(points) < 3:
        raise ValueError("Map needs at least three occupied cells")
    return points, provenance


def distances(tree, points):
    distance, _ = tree.query(points)
    return {"median_m": float(np.median(distance)), "p95_m": float(np.quantile(distance, .95)),
            "fraction_within_0_10m": float(np.mean(distance <= .10)),
            "fraction_within_0_25m": float(np.mean(distance <= .25))}


def compare(source, target):
    tree = cKDTree(target)
    current = source.copy()
    total_rotation, total_translation = np.eye(2), np.zeros(2)
    converged = False
    for iteration in range(50):
        distance, indices = tree.query(current)
        cutoff = min(.5, float(np.quantile(distance, .8)))
        selected = distance <= cutoff
        if np.count_nonzero(selected) < 3:
            raise ValueError("Insufficient local map correspondence")
        a, b = current[selected], target[indices[selected]]
        ac, bc = a.mean(axis=0), b.mean(axis=0)
        u, _, vt = np.linalg.svd((a - ac).T @ (b - bc))
        correction = np.eye(2)
        correction[1, 1] = np.linalg.det(vt.T @ u.T)
        rotation = vt.T @ correction @ u.T
        translation = bc - rotation @ ac
        current = current @ rotation.T + translation
        total_translation = rotation @ total_translation + translation
        total_rotation = rotation @ total_rotation
        if np.linalg.norm(translation) < 1e-6 and abs(np.arctan2(rotation[1, 0], rotation[0, 0])) < 1e-7:
            converged = True
            break
    return {"iterations": iteration + 1, "local_fit_converged": converged,
            "source_to_target_xy_yaw": [*total_translation.tolist(),
                                        float(np.arctan2(total_rotation[1, 0], total_rotation[0, 0]))],
            "identity_distances": distances(tree, source), "fitted_distances": distances(tree, current)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--target", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source, source_meta = occupied_points(args.source)
    target, target_meta = occupied_points(args.target)
    result = {"schema": "map_frame_diagnostic.v1", "reference_status": "FRAME_UNVERIFIED",
              "limitations": "Identity-initialized local 2D occupancy fit only; not a verified map-frame transform or independent pose ground truth.",
              "source": source_meta, "target": target_meta, **compare(source, target)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
