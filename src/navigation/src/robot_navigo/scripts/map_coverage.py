#!/usr/bin/env python3
"""Generate safe map-coverage routes and evaluate Odometry coverage.

The implementation deliberately has no ROS imports in its map/route core.  ROS2
bag support is loaded only when ``evaluate --trajectory-bag`` is used, so the
same script remains useful for deterministic unit tests and offline analysis.
"""

from __future__ import annotations

import argparse
import ast
import csv
from collections import deque
import hashlib
import heapq
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Sequence

try:
    import numpy as np
    from PIL import Image, ImageDraw
except ImportError as exc:  # pragma: no cover - exercised by deployment smoke tests
    np = None
    Image = None
    ImageDraw = None
    _DEPENDENCY_ERROR = str(exc)
else:
    _DEPENDENCY_ERROR = ""


SCHEMA = "robot_navigo.map_coverage"
SCHEMA_VERSION = 1


def require_dependencies() -> None:
    if np is None or Image is None:
        raise RuntimeError(
            "map_coverage.py requires numpy and Pillow; "
            f"import failed: {_DEPENDENCY_ERROR}"
        )


def _strip_yaml_value(value: str) -> str:
    value = value.split(" #", 1)[0].strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        return value[1:-1]
    return value


def read_map_yaml(path: Path) -> dict:
    """Read the ROS map fields without requiring PyYAML at runtime."""
    fields = {}
    lines = path.read_text(encoding="utf-8").splitlines()
    index = 0
    while index < len(lines):
        raw = lines[index]
        line = raw.strip()
        if not line or line.startswith("#") or ":" not in line:
            index += 1
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = _strip_yaml_value(value)
        if key == "origin" and not value:
            values = []
            lookahead = index + 1
            while lookahead < len(lines):
                item = lines[lookahead].strip()
                if not item:
                    lookahead += 1
                    continue
                if not item.startswith("-"):
                    break
                values.append(_strip_yaml_value(item[1:].strip()))
                lookahead += 1
            value = "[" + ", ".join(values) + "]"
            index = lookahead - 1
        fields[key] = value
        index += 1
    required = ("image", "resolution", "origin")
    missing = [key for key in required if key not in fields]
    if missing:
        raise ValueError(f"map YAML {path} is missing required field(s): {', '.join(missing)}")
    try:
        origin = ast.literal_eval(fields["origin"])
        if len(origin) < 3:
            raise ValueError
        result = {
            "image": str(fields["image"]),
            "resolution": float(fields["resolution"]),
            "origin": [float(origin[0]), float(origin[1]), float(origin[2])],
            "negate": int(fields.get("negate", "0")),
            "occupied_thresh": float(fields.get("occupied_thresh", "0.65")),
            "free_thresh": float(fields.get("free_thresh", "0.196")),
            "mode": fields.get("mode", "trinary"),
        }
    except (ValueError, SyntaxError, TypeError) as exc:
        raise ValueError(f"invalid ROS map YAML fields in {path}: {exc}") from exc
    if result["resolution"] <= 0:
        raise ValueError("map resolution must be positive")
    return result


@dataclass(frozen=True)
class GridMap:
    """A map in Cartesian cell order: ``cells[y, x]`` has y increasing upward."""

    cells: np.ndarray
    resolution: float
    origin: tuple[float, float, float]
    source_yaml: Path
    source_image: Path
    free_thresh: float
    occupied_thresh: float
    negate: int

    @property
    def height(self) -> int:
        return int(self.cells.shape[0])

    @property
    def width(self) -> int:
        return int(self.cells.shape[1])

    @property
    def unknown(self) -> np.ndarray:
        return self.cells == 0

    @property
    def free(self) -> np.ndarray:
        return self.cells == 1

    @property
    def occupied(self) -> np.ndarray:
        return self.cells == 2

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        ox, oy, yaw = self.origin
        dx, dy = x - ox, y - oy
        c, s = math.cos(yaw), math.sin(yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        cx = math.floor(local_x / self.resolution)
        cy = math.floor(local_y / self.resolution)
        if 0 <= cx < self.width and 0 <= cy < self.height:
            return cx, cy
        return None

    def cell_to_world(self, cx: int, cy: int) -> tuple[float, float]:
        ox, oy, yaw = self.origin
        local_x = (cx + 0.5) * self.resolution
        local_y = (cy + 0.5) * self.resolution
        c, s = math.cos(yaw), math.sin(yaw)
        return ox + c * local_x - s * local_y, oy + s * local_x + c * local_y

    def cell_is(self, mask: np.ndarray, cell: tuple[int, int] | None) -> bool:
        return cell is not None and bool(mask[cell[1], cell[0]])


def load_grid_map(map_yaml: str | Path) -> GridMap:
    require_dependencies()
    yaml_path = Path(map_yaml).expanduser().resolve()
    config = read_map_yaml(yaml_path)
    image_path = Path(config["image"])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()
    if not image_path.is_file():
        raise FileNotFoundError(f"map image does not exist: {image_path}")
    image = np.asarray(Image.open(image_path).convert("L"), dtype=np.float32)
    if image.ndim != 2:
        raise ValueError(f"map image must be grayscale: {image_path}")
    normalized = image / 255.0
    occupancy = normalized if config["negate"] else 1.0 - normalized
    free = occupancy < config["free_thresh"]
    occupied = occupancy > config["occupied_thresh"]
    # PGM rows start at the top; ROS map coordinates start at the bottom.
    state = np.zeros(image.shape, dtype=np.uint8)
    state[free] = 1
    state[occupied] = 2
    state = np.flipud(state)
    return GridMap(
        cells=state,
        resolution=config["resolution"],
        origin=tuple(config["origin"]),
        source_yaml=yaml_path,
        source_image=image_path,
        free_thresh=config["free_thresh"],
        occupied_thresh=config["occupied_thresh"],
        negate=config["negate"],
    )


def disk_structure(radius_cells: int) -> np.ndarray:
    radius_cells = max(0, int(radius_cells))
    if radius_cells == 0:
        return np.ones((1, 1), dtype=bool)
    yy, xx = np.ogrid[-radius_cells : radius_cells + 1, -radius_cells : radius_cells + 1]
    return (xx * xx + yy * yy) <= radius_cells * radius_cells


def _sample_offset(mask: np.ndarray, dx: int, dy: int) -> np.ndarray:
    """Return mask[y + dy, x + dx], treating out-of-map cells as false."""
    height, width = mask.shape
    result = np.zeros_like(mask, dtype=bool)
    x0, x1 = max(0, -dx), min(width, width - dx)
    y0, y1 = max(0, -dy), min(height, height - dy)
    if x0 < x1 and y0 < y1:
        result[y0:y1, x0:x1] = mask[y0 + dy:y1 + dy, x0 + dx:x1 + dx]
    return result


def binary_morphology(mask: np.ndarray, structure: np.ndarray, operation: str) -> np.ndarray:
    """Dependency-free binary erosion/dilation for occupancy grids."""
    if operation not in {"erosion", "dilation"}:
        raise ValueError(f"unsupported morphology operation: {operation}")
    result = np.ones_like(mask, dtype=bool) if operation == "erosion" else np.zeros_like(mask, dtype=bool)
    center_y, center_x = structure.shape[0] // 2, structure.shape[1] // 2
    for row, column in np.argwhere(structure):
        sample = _sample_offset(mask, int(column - center_x), int(row - center_y))
        if operation == "erosion":
            result &= sample
        else:
            result |= sample
    return result


def label_components(mask: np.ndarray, connectivity: int = 4) -> tuple[np.ndarray, int]:
    if connectivity == 4:
        neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1))
    elif connectivity == 8:
        neighbors = tuple(
            (dx, dy) for dy in (-1, 0, 1) for dx in (-1, 0, 1)
            if dx != 0 or dy != 0
        )
    else:
        raise ValueError("connectivity must be 4 or 8")
    labels = np.zeros(mask.shape, dtype=np.int32)
    height, width = mask.shape
    count = 0
    for seed_y, seed_x in np.argwhere(mask):
        if labels[seed_y, seed_x] != 0:
            continue
        count += 1
        labels[seed_y, seed_x] = count
        pending = deque([(int(seed_x), int(seed_y))])
        while pending:
            x, y = pending.popleft()
            for dx, dy in neighbors:
                nx, ny = x + dx, y + dy
                if (0 <= nx < width and 0 <= ny < height and mask[ny, nx]
                        and labels[ny, nx] == 0):
                    labels[ny, nx] = count
                    pending.append((nx, ny))
    return labels, count


def safe_reachable_domain(
    grid: GridMap,
    start: tuple[float, float],
    safety_radius_m: float,
    connectivity: int = 4,
) -> tuple[np.ndarray, dict]:
    if safety_radius_m < 0:
        raise ValueError("safety radius must be non-negative")
    start_cell = grid.world_to_cell(*start)
    radius_cells = int(math.ceil(safety_radius_m / grid.resolution))
    safe = binary_morphology(grid.free, disk_structure(radius_cells), "erosion")
    if not grid.cell_is(safe, start_cell):
        status = "outside_map" if start_cell is None else (
            "unknown" if grid.unknown[start_cell[1], start_cell[0]] else
            "occupied" if grid.occupied[start_cell[1], start_cell[0]] else "unsafe_after_erosion"
        )
        raise ValueError(
            f"start ({start[0]:.3f}, {start[1]:.3f}) is not in the safe free domain: {status}"
        )
    labels, count = label_components(safe, connectivity)
    component = labels == labels[start_cell[1], start_cell[0]]
    ys, xs = np.where(component)
    metadata = {
        "start_cell": [int(start_cell[0]), int(start_cell[1])],
        "safety_radius_m": float(safety_radius_m),
        "safety_radius_cells": radius_cells,
        "connectivity": connectivity,
        "free_cells": int(np.count_nonzero(grid.free)),
        "unknown_cells": int(np.count_nonzero(grid.unknown)),
        "occupied_cells": int(np.count_nonzero(grid.occupied)),
        "safe_free_cells": int(np.count_nonzero(safe)),
        "reachable_free_cells": int(np.count_nonzero(component)),
        "reachable_free_area_m2": float(np.count_nonzero(component) * grid.resolution**2),
        "reachable_bbox_cells": [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())],
        "components_after_erosion": int(count),
    }
    return component, metadata


def segment_is_free(grid: GridMap, domain: np.ndarray, p0: tuple[float, float], p1: tuple[float, float]) -> bool:
    distance = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    samples = max(1, int(math.ceil(distance / max(grid.resolution * 0.35, 1e-6))))
    for i in range(samples + 1):
        ratio = i / samples
        point = (p0[0] + ratio * (p1[0] - p0[0]), p0[1] + ratio * (p1[1] - p0[1]))
        if not grid.cell_is(domain, grid.world_to_cell(*point)):
            return False
    return True


def _astar(
    grid: GridMap,
    domain: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    if start == goal:
        return [start]
    queue = [(0.0, 0, start)]
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    cost = {start: 0.0}
    sequence = 0
    neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1))
    while queue:
        _, _, current = heapq.heappop(queue)
        if current == goal:
            result = []
            while current is not None:
                result.append(current)
                current = came_from[current]
            return list(reversed(result))
        for dx, dy in neighbors:
            nxt = (current[0] + dx, current[1] + dy)
            if not grid.cell_is(domain, nxt):
                continue
            new_cost = cost[current] + 1.0
            if new_cost >= cost.get(nxt, float("inf")):
                continue
            cost[nxt] = new_cost
            came_from[nxt] = current
            sequence += 1
            heuristic = abs(goal[0] - nxt[0]) + abs(goal[1] - nxt[1])
            heapq.heappush(queue, (new_cost + heuristic, sequence, nxt))
    raise ValueError(f"no free-space connection between cells {start} and {goal}")


def _cells_to_world(grid: GridMap, cells: Iterable[tuple[int, int]]) -> list[tuple[float, float]]:
    return [grid.cell_to_world(cx, cy) for cx, cy in cells]


def _simplify_path(grid: GridMap, domain: np.ndarray, points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if len(points) <= 2:
        return points
    result = [points[0]]
    anchor = 0
    while anchor < len(points) - 1:
        furthest = anchor + 1
        for candidate in range(anchor + 1, len(points)):
            if segment_is_free(grid, domain, points[anchor], points[candidate]):
                furthest = candidate
            else:
                break
        result.append(points[furthest])
        anchor = furthest
    return result


def _append_connection(
    grid: GridMap,
    domain: np.ndarray,
    route: list[tuple[float, float]],
    target: tuple[float, float],
) -> None:
    if route and segment_is_free(grid, domain, route[-1], target):
        if route[-1] != target:
            route.append(target)
        return
    target_cell = grid.world_to_cell(*target)
    current_cell = grid.world_to_cell(*route[-1]) if route else target_cell
    if target_cell is None or current_cell is None:
        raise ValueError("route connection endpoint is outside the map")
    path = _cells_to_world(grid, _astar(grid, domain, current_cell, target_cell))
    if route and path and path[0] != route[-1]:
        path.insert(0, route[-1])
    route.extend(_simplify_path(grid, domain, path)[1:] if route else path)


def _scanline_intervals(component: np.ndarray, y: int) -> list[tuple[int, int]]:
    xs = np.flatnonzero(component[y])
    if len(xs) == 0:
        return []
    breaks = np.flatnonzero(np.diff(xs) > 1)
    starts = np.r_[0, breaks + 1]
    ends = np.r_[breaks, len(xs) - 1]
    return [(int(xs[a]), int(xs[b])) for a, b in zip(starts, ends)]


def generate_route(
    grid: GridMap,
    start: tuple[float, float],
    safety_radius_m: float,
    row_spacing_m: float,
    waypoint_spacing_m: float,
    coverage_radius_m: float,
    connectivity: int = 4,
) -> tuple[list[dict], dict, np.ndarray]:
    if row_spacing_m <= 0 or waypoint_spacing_m <= 0 or coverage_radius_m < 0:
        raise ValueError("row/waypoint spacing must be positive and coverage radius non-negative")
    domain, domain_metadata = safe_reachable_domain(grid, start, safety_radius_m, connectivity)
    ys, xs = np.where(domain)
    row_step = max(1, int(round(row_spacing_m / grid.resolution)))
    point_step = max(1, int(round(waypoint_spacing_m / grid.resolution)))
    rows = list(range(int(ys.min()), int(ys.max()) + 1, row_step))
    if rows[-1] != int(ys.max()):
        rows.append(int(ys.max()))
    route: list[tuple[float, float]] = [start]
    left_to_right = True
    scanline_count = 0
    interval_count = 0
    for y in rows:
        intervals = _scanline_intervals(domain, y)
        if not intervals:
            continue
        scanline_count += 1
        ordered = intervals if left_to_right else list(reversed(intervals))
        for x0, x1 in ordered:
            interval_count += 1
            values = list(range(x0, x1 + 1, point_step))
            if not values or values[-1] != x1:
                values.append(x1)
            if not left_to_right:
                values = list(reversed(values))
            for x in values:
                _append_connection(grid, domain, route, grid.cell_to_world(x, y))
        left_to_right = not left_to_right

    if not route:
        raise ValueError("reachable domain produced no route")
    for a, b in zip(route, route[1:]):
        if not segment_is_free(grid, domain, a, b):
            raise AssertionError(f"generated unsafe route segment: {a} -> {b}")
    route_cells = rasterize_polyline(grid, route)
    planned = coverage_from_cells(domain, route_cells, coverage_radius_m, grid.resolution)
    waypoints = []
    for index, point in enumerate(route):
        if index + 1 < len(route):
            nxt = route[index + 1]
        elif index:
            nxt = route[index - 1]
        else:
            nxt = (point[0] + 1.0, point[1])
        yaw = math.atan2(nxt[1] - point[1], nxt[0] - point[0])
        waypoints.append({"index": index, "x": point[0], "y": point[1], "yaw": yaw})
    route_length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(route, route[1:]))
    metadata = {
        **domain_metadata,
        "start": {"x": start[0], "y": start[1]},
        "row_spacing_m": row_spacing_m,
        "waypoint_spacing_m": waypoint_spacing_m,
        "coverage_radius_m": coverage_radius_m,
        "scanline_count": scanline_count,
        "partition_interval_count": interval_count,
        "waypoint_count": len(waypoints),
        "segment_count": max(0, len(waypoints) - 1),
        "route_length_m": route_length,
        "valid_region": "free cells in the start-connected component after safety-radius erosion",
        "route_safety_check": "all segments sampled inside the safe reachable domain",
        "planned_coverage_cells": int(np.count_nonzero(planned)),
        "planned_coverage_area_m2": float(np.count_nonzero(planned) * grid.resolution**2),
        "planned_coverage_pct": float(100.0 * np.count_nonzero(planned) / max(1, np.count_nonzero(domain))),
    }
    return waypoints, metadata, domain


def rasterize_polyline(grid: GridMap, points: Sequence[tuple[float, float]]) -> np.ndarray:
    cells = np.zeros((grid.height, grid.width), dtype=bool)
    if not points:
        return cells
    for point in points:
        cell = grid.world_to_cell(*point)
        if cell is not None:
            cells[cell[1], cell[0]] = True
    for p0, p1 in zip(points, points[1:]):
        distance = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        count = max(1, int(math.ceil(distance / max(grid.resolution * 0.5, 1e-6))))
        for i in range(count + 1):
            ratio = i / count
            point = (p0[0] + ratio * (p1[0] - p0[0]), p0[1] + ratio * (p1[1] - p0[1]))
            cell = grid.world_to_cell(*point)
            if cell is not None:
                cells[cell[1], cell[0]] = True
    return cells


def coverage_from_cells(domain: np.ndarray, seed_cells: np.ndarray, radius_m: float, resolution: float) -> np.ndarray:
    radius_cells = int(math.ceil(max(0.0, radius_m) / resolution))
    return binary_morphology(seed_cells, disk_structure(radius_cells), "dilation") & domain


def _map_fingerprint(grid: GridMap) -> str:
    digest = hashlib.sha256()
    for path in (grid.source_yaml, grid.source_image):
        digest.update(str(path).encode("utf-8"))
        digest.update(path.read_bytes())
    return digest.hexdigest()


def draw_preview(
    grid: GridMap,
    output: Path,
    domain: np.ndarray | None = None,
    route: Sequence[tuple[float, float]] = (),
    covered: np.ndarray | None = None,
    uncovered: np.ndarray | None = None,
    trajectory: Sequence[tuple[float, float]] = (),
) -> None:
    rgb = np.zeros((grid.height, grid.width, 3), dtype=np.uint8)
    rgb[:, :] = (170, 170, 170)  # unknown
    rgb[grid.occupied] = (30, 30, 30)
    rgb[grid.free] = (245, 245, 245)
    if domain is not None:
        rgb[domain] = (220, 242, 220)
    if uncovered is not None:
        rgb[uncovered] = (255, 195, 120)
    if covered is not None:
        rgb[covered] = (178, 230, 178)
    image = Image.fromarray(np.flipud(rgb), mode="RGB")
    draw = ImageDraw.Draw(image)

    def pixel(point: tuple[float, float]) -> tuple[int, int] | None:
        cell = grid.world_to_cell(*point)
        if cell is None:
            return None
        return cell[0], grid.height - 1 - cell[1]

    route_pixels = [p for p in (pixel(point) for point in route) if p is not None]
    if len(route_pixels) > 1:
        draw.line(route_pixels, fill=(210, 30, 30), width=max(1, round(2 / grid.resolution)))
    trajectory_pixels = [p for p in (pixel(point) for point in trajectory) if p is not None]
    if len(trajectory_pixels) > 1:
        draw.line(trajectory_pixels, fill=(30, 80, 220), width=max(1, round(2 / grid.resolution)))
    if route_pixels:
        radius = max(2, round(0.15 / grid.resolution))
        x, y = route_pixels[0]
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=(20, 150, 30))
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def _trajectory_from_csv(path: Path) -> list[tuple[float, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = csv.DictReader(stream)
        if not rows.fieldnames:
            raise ValueError(f"trajectory CSV has no header: {path}")
        names = {name.strip().lower(): name for name in rows.fieldnames}
        x_name = next((names[key] for key in ("x", "pose_x", "position_x") if key in names), None)
        y_name = next((names[key] for key in ("y", "pose_y", "position_y") if key in names), None)
        if x_name is None or y_name is None:
            raise ValueError("trajectory CSV must contain x,y (or pose_x,pose_y) columns")
        result = []
        for line, row in enumerate(rows, 2):
            try:
                result.append((float(row[x_name]), float(row[y_name])))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"invalid trajectory at {path}:{line}") from exc
    if not result:
        raise ValueError(f"trajectory CSV is empty: {path}")
    return result


def _trajectory_from_bag(path: Path, topic: str, storage_id: str = "mcap") -> list[tuple[float, float]]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "reading --trajectory-bag requires a sourced ROS2 environment with rosbag2_py, "
            f"rclpy and rosidl_runtime_py: {exc}"
        ) from exc
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
            rosbag2_py.ConverterOptions("cdr", "cdr"),
        )
    except Exception as exc:
        raise RuntimeError(f"cannot open ROS2 bag {path}: {exc}") from exc
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topics:
        raise ValueError(f"Odometry topic {topic!r} not found; available topics: {sorted(topics)}")
    if topics[topic] != "nav_msgs/msg/Odometry":
        raise ValueError(f"topic {topic} has type {topics[topic]}, expected nav_msgs/msg/Odometry")
    message_type = get_message(topics[topic])
    result = []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, message_type)
        result.append((float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)))
    if not result:
        raise ValueError(f"Odometry topic {topic} contains no messages in {path}")
    return result


def _uncovered_regions(mask: np.ndarray) -> list[dict]:
    labels, count = label_components(mask, 8)
    regions = []
    for label in range(1, count + 1):
        ys, xs = np.where(labels == label)
        regions.append({
            "cells": int(len(xs)),
            "bbox_cells": [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())],
        })
    return sorted(regions, key=lambda item: item["cells"], reverse=True)


def evaluate_trajectory(
    grid: GridMap,
    trajectory: Sequence[tuple[float, float]],
    start: tuple[float, float],
    safety_radius_m: float,
    coverage_radius_m: float,
    connectivity: int = 4,
) -> tuple[dict, np.ndarray, np.ndarray, np.ndarray]:
    domain, domain_metadata = safe_reachable_domain(grid, start, safety_radius_m, connectivity)
    counts = {"total_points": len(trajectory), "outside_map": 0, "unknown": 0, "occupied": 0,
              "free_outside_reachable_domain": 0, "safe_reachable_points": 0}
    for point in trajectory:
        cell = grid.world_to_cell(*point)
        if cell is None:
            counts["outside_map"] += 1
        elif grid.unknown[cell[1], cell[0]]:
            counts["unknown"] += 1
        elif grid.occupied[cell[1], cell[0]]:
            counts["occupied"] += 1
        elif domain[cell[1], cell[0]]:
            counts["safe_reachable_points"] += 1
        else:
            counts["free_outside_reachable_domain"] += 1
    seeds = rasterize_polyline(grid, trajectory) & domain
    covered = coverage_from_cells(domain, seeds, coverage_radius_m, grid.resolution)
    uncovered = domain & ~covered
    valid_cells = int(np.count_nonzero(domain))
    covered_cells = int(np.count_nonzero(covered))
    metadata = {
        **domain_metadata,
        **counts,
        "coverage_radius_m": float(coverage_radius_m),
        "coverage_radius_cells": int(math.ceil(coverage_radius_m / grid.resolution)),
        "valid_region": "free cells in the start-connected component after safety-radius erosion",
        "unknown_counts_as": "not valid and never covered",
        "occupied_counts_as": "not valid and never covered",
        "valid_cells": valid_cells,
        "valid_area_m2": float(valid_cells * grid.resolution**2),
        "covered_cells": covered_cells,
        "covered_area_m2": float(covered_cells * grid.resolution**2),
        "uncovered_cells": valid_cells - covered_cells,
        "uncovered_area_m2": float((valid_cells - covered_cells) * grid.resolution**2),
        "coverage_pct": float(100.0 * covered_cells / max(1, valid_cells)),
        "uncovered_regions": _uncovered_regions(uncovered),
    }
    return metadata, domain, covered, uncovered


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def command_generate(args: argparse.Namespace) -> int:
    grid = load_grid_map(args.map_yaml)
    start = (args.start_x, args.start_y)
    waypoints, metadata, domain = generate_route(
        grid, start, args.safety_radius, args.row_spacing, args.waypoint_spacing,
        args.coverage_radius, args.connectivity
    )
    output = Path(args.output).expanduser().resolve()
    preview = Path(args.preview).expanduser().resolve() if args.preview else output.with_name("route_preview.png")
    route = [(item["x"], item["y"]) for item in waypoints]
    metadata.update({"map_fingerprint_sha256": _map_fingerprint(grid), "preview": str(preview),
                     "map_yaml": str(grid.source_yaml), "map_image": str(grid.source_image)})
    parameters = {key: value for key, value in vars(args).items() if key != "handler"}
    payload = {"schema": SCHEMA, "schema_version": SCHEMA_VERSION,
               "name": output.stem, "frame_id": args.frame_id, "return_to_start": True,
               "map": {"yaml": str(grid.source_yaml), "image": str(grid.source_image),
                       "resolution": grid.resolution, "origin": list(grid.origin)},
               "start": {"x": start[0], "y": start[1]}, "parameters": parameters,
               "metadata": metadata, "waypoints": waypoints}
    _write_json(output, payload)
    draw_preview(grid, preview, domain=domain, route=route)
    print(json.dumps({"route": str(output), "preview": str(preview), "metadata": metadata}, indent=2))
    return 0


def command_evaluate(args: argparse.Namespace) -> int:
    grid = load_grid_map(args.map_yaml)
    if bool(args.trajectory_bag) == bool(args.trajectory_csv):
        raise ValueError("provide exactly one of --trajectory-bag or --trajectory-csv")
    trajectory = (_trajectory_from_bag(Path(args.trajectory_bag), args.topic, args.storage_id)
                  if args.trajectory_bag else _trajectory_from_csv(Path(args.trajectory_csv)))
    metadata, domain, covered, uncovered = evaluate_trajectory(
        grid, trajectory, (args.start_x, args.start_y), args.safety_radius,
        args.coverage_radius, args.connectivity
    )
    output = Path(args.output).expanduser().resolve()
    preview = Path(args.preview).expanduser().resolve() if args.preview else output.with_name("coverage_preview.png")
    metadata.update({"map_fingerprint_sha256": _map_fingerprint(grid), "trajectory_source": str(args.trajectory_bag or args.trajectory_csv),
                     "trajectory_topic": args.topic if args.trajectory_bag else None, "preview": str(preview),
                     "map_yaml": str(grid.source_yaml), "map_image": str(grid.source_image)})
    _write_json(output, {"schema": SCHEMA, "schema_version": SCHEMA_VERSION, "metadata": metadata})
    draw_preview(grid, preview, domain=domain, covered=covered, uncovered=uncovered, trajectory=trajectory)
    print(json.dumps({"metadata": str(output), "preview": str(preview), "coverage_pct": metadata["coverage_pct"]}, indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    generate = sub.add_parser("generate", help="generate a safe sparse coverage route")
    generate.add_argument("--map-yaml", required=True)
    generate.add_argument("--start-x", type=float, required=True)
    generate.add_argument("--start-y", type=float, required=True)
    generate.add_argument("--output", required=True, help="route waypoint JSON")
    generate.add_argument("--preview", help="route preview PNG")
    generate.add_argument("--frame-id", default="map")
    generate.add_argument("--safety-radius", type=float, default=0.35)
    generate.add_argument("--coverage-radius", type=float, default=0.75)
    generate.add_argument("--row-spacing", type=float, default=1.0)
    generate.add_argument("--waypoint-spacing", type=float, default=0.8)
    generate.add_argument("--connectivity", type=int, choices=(4, 8), default=4)
    generate.set_defaults(handler=command_generate)

    evaluate = sub.add_parser("evaluate", help="evaluate Odometry coverage of the safe reachable region")
    evaluate.add_argument("--map-yaml", required=True)
    trajectory = evaluate.add_mutually_exclusive_group(required=True)
    trajectory.add_argument("--trajectory-bag")
    trajectory.add_argument("--trajectory-csv")
    evaluate.add_argument("--topic", default="/odom/current_pose")
    evaluate.add_argument("--storage-id", default="mcap")
    evaluate.add_argument("--start-x", type=float, required=True)
    evaluate.add_argument("--start-y", type=float, required=True)
    evaluate.add_argument("--output", required=True, help="coverage metadata JSON")
    evaluate.add_argument("--preview", help="coverage preview PNG")
    evaluate.add_argument("--safety-radius", type=float, default=0.35)
    evaluate.add_argument("--coverage-radius", type=float, default=0.5)
    evaluate.add_argument("--connectivity", type=int, choices=(4, 8), default=4)
    evaluate.set_defaults(handler=command_evaluate)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = build_parser().parse_args(argv)
        return int(args.handler(args))
    except (FileNotFoundError, RuntimeError, ValueError, AssertionError) as exc:
        print(f"map_coverage.py: error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
