#!/usr/bin/env python3
"""Build a Matrix-only point-cloud and occupancy reference map from GT poses.

Ground truth is used only while generating the static simulation test asset.
Online localization and navigation must not subscribe to the GT odometry.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import re
from pathlib import Path

import numpy as np


NSEC = 1_000_000_000
UNKNOWN_PIXEL = 128


def storage_id(bag: Path) -> str:
    metadata = bag / "metadata.yaml"
    if metadata.exists():
        match = re.search(
            r"^\s*storage_identifier:\s*([^\s#]+)",
            metadata.read_text(encoding="utf-8"),
            re.MULTILINE,
        )
        if match:
            return match.group(1)
    return "sqlite3"


def stamp_ns(message) -> int:
    stamp = message.header.stamp
    value = int(stamp.sec) * NSEC + int(stamp.nanosec)
    return value


def quaternion_rotation(quaternion) -> np.ndarray:
    x, y, z, w = (
        float(quaternion.x), float(quaternion.y),
        float(quaternion.z), float(quaternion.w),
    )
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def read_odometry(bag: Path):
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id(bag)),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/odom/mujoco_odom"]))
    stamps = []
    poses = []
    while reader.has_next():
        _, raw, received = reader.read_next()
        message = deserialize_message(raw, Odometry)
        timestamp = stamp_ns(message) or received
        position = message.pose.pose.position
        stamps.append(timestamp)
        poses.append((
            np.asarray([position.x, position.y, position.z], dtype=np.float64),
            quaternion_rotation(message.pose.pose.orientation),
        ))
    if not stamps:
        raise RuntimeError("bag has no /odom/mujoco_odom samples")
    return stamps, poses


def nearest_pose(stamps, poses, timestamp, tolerance_ns):
    index = bisect.bisect_left(stamps, timestamp)
    candidates = []
    if index < len(stamps):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    nearest = min(candidates, key=lambda item: abs(stamps[item] - timestamp))
    if abs(stamps[nearest] - timestamp) > tolerance_ns:
        return None
    return poses[nearest]


def cloud_array(message) -> np.ndarray:
    field_offsets = {field.name: field.offset for field in message.fields}
    required = ("x", "y", "z", "intensity")
    missing = [field for field in required if field not in field_offsets]
    if missing:
        raise RuntimeError(f"PointCloud2 is missing fields: {missing}")
    dtype = np.dtype({
        "names": list(required),
        "formats": ["<f4", "<f4", "<f4", "<f4"],
        "offsets": [field_offsets[field] for field in required],
        "itemsize": int(message.point_step),
    })
    return np.frombuffer(message.data, dtype=dtype, count=message.width * message.height)


def collect_points(args, stamps, poses):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=storage_id(args.bag)),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/livox/lidar"]))
    extrinsic = np.asarray(args.base_to_lidar, dtype=np.float64)
    chunks = []
    scans = []
    read_count = used_count = unsynchronized = 0
    while reader.has_next():
        _, raw, received = reader.read_next()
        read_count += 1
        if (read_count - 1) % args.frame_stride:
            continue
        message = deserialize_message(raw, PointCloud2)
        timestamp = stamp_ns(message) or received
        pose = nearest_pose(stamps, poses, timestamp, int(args.sync_tolerance * NSEC))
        if pose is None:
            unsynchronized += 1
            continue
        cloud = cloud_array(message)
        xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(np.float64)
        intensity = np.asarray(cloud["intensity"], dtype=np.float32)
        ranges = np.linalg.norm(xyz, axis=1)
        valid = (
            np.isfinite(xyz).all(axis=1)
            & np.isfinite(intensity)
            & (ranges >= args.range_min)
            & (ranges <= args.range_max)
        )
        xyz = xyz[valid] + extrinsic
        intensity = intensity[valid]
        translation, rotation = pose
        world = xyz @ rotation.T + translation
        chunk = np.column_stack((world.astype(np.float32), intensity))
        chunks.append(chunk)
        lidar_origin = rotation @ extrinsic + translation
        scans.append((lidar_origin.astype(np.float32), chunk[:, :3]))
        used_count += 1
        if used_count % 100 == 0:
            print(f"[map] used {used_count} clouds, retained {sum(len(chunk) for chunk in chunks)} points")
    if not chunks:
        raise RuntimeError("no synchronized point clouds were retained")
    points = np.concatenate(chunks, axis=0)
    voxels = np.floor(points[:, :3] / args.voxel_size).astype(np.int32)
    _, indices = np.unique(voxels, axis=0, return_index=True)
    points = points[np.sort(indices)]
    metrics = {
        "clouds_read": read_count,
        "clouds_used": used_count,
        "clouds_unsynchronized": unsynchronized,
        "points_after_voxel": int(len(points)),
    }
    return points, scans, metrics


def write_pcd(path: Path, points: np.ndarray) -> None:
    payload = np.empty(
        len(points),
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
               ("intensity", "<f4"), ("timestamp", "<f8")],
    )
    for index, field in enumerate(("x", "y", "z", "intensity")):
        payload[field] = points[:, index]
    payload["timestamp"] = 0.0
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity timestamp\n"
        "SIZE 4 4 4 4 8\n"
        "TYPE F F F F F\n"
        "COUNT 1 1 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(payload.tobytes())


def write_occupancy(output: Path, points: np.ndarray, scans, args) -> dict:
    minimum = np.floor(points[:, :2].min(axis=0) - args.map_margin)
    maximum = np.ceil(points[:, :2].max(axis=0) + args.map_margin)
    size = np.ceil((maximum - minimum) / args.map_resolution).astype(int)
    visits = np.zeros((size[1], size[0]), dtype=np.uint32)
    hits = np.zeros_like(visits)
    visits_flat = visits.ravel()
    hits_flat = hits.ravel()
    angle_count = int(round(360.0 / args.ray_angle_resolution))
    angles = np.arange(angle_count, dtype=np.float64) * math.radians(
        args.ray_angle_resolution)

    for scan_index, (origin, scan) in enumerate(scans, start=1):
        offsets = scan.astype(np.float64) - origin
        ranges = np.linalg.norm(offsets[:, :2], axis=1)
        heights = scan[:, 2] - args.floor_height
        valid = (
            np.isfinite(ranges)
            & (ranges > args.range_min)
            & (ranges <= args.range_max)
            & (heights > -args.floor_tolerance)
            & (heights < args.obstacle_max_height)
        )
        if not np.any(valid):
            continue
        ranges = ranges[valid]
        offsets = offsets[valid]
        heights = heights[valid]
        bins = np.floor(
            (np.arctan2(offsets[:, 1], offsets[:, 0]) + 2.0 * math.pi)
            / math.radians(args.ray_angle_resolution)
        ).astype(np.int32) % angle_count

        obstacle_ranges = np.full(angle_count, np.inf, dtype=np.float64)
        obstacle = heights >= args.obstacle_min_height
        np.minimum.at(obstacle_ranges, bins[obstacle], ranges[obstacle])

        floor_ranges = np.zeros(angle_count, dtype=np.float64)
        floor = heights < args.obstacle_min_height
        np.maximum.at(floor_ranges, bins[floor], ranges[floor])
        ray_ranges = np.where(np.isfinite(obstacle_ranges), obstacle_ranges, floor_ranges)

        valid_bins = np.flatnonzero(ray_ranges > args.map_resolution)
        for angle_index in valid_bins:
            distance = ray_ranges[angle_index]
            free_distance = max(0.0, distance - 1.5 * args.map_resolution)
            samples = np.arange(args.map_resolution, free_distance,
                                args.map_resolution)
            if samples.size:
                x = origin[0] + samples * math.cos(angles[angle_index])
                y = origin[1] + samples * math.sin(angles[angle_index])
                cell_x = np.floor((x - minimum[0]) / args.map_resolution).astype(int)
                cell_y = np.floor((y - minimum[1]) / args.map_resolution).astype(int)
                inside = (
                    (cell_x >= 0) & (cell_x < size[0])
                    & (cell_y >= 0) & (cell_y < size[1])
                )
                flat = cell_y[inside] * size[0] + cell_x[inside]
                np.add.at(visits_flat, flat, 1)

            if np.isfinite(obstacle_ranges[angle_index]):
                endpoint_x = origin[0] + distance * math.cos(angles[angle_index])
                endpoint_y = origin[1] + distance * math.sin(angles[angle_index])
                cell_x = int(math.floor((endpoint_x - minimum[0]) / args.map_resolution))
                cell_y = int(math.floor((endpoint_y - minimum[1]) / args.map_resolution))
                if 0 <= cell_x < size[0] and 0 <= cell_y < size[1]:
                    flat = cell_y * size[0] + cell_x
                    visits_flat[flat] += 1
                    hits_flat[flat] += 1
        if scan_index % 100 == 0:
            print(f"[grid] processed {scan_index}/{len(scans)} scans")

    observed = visits >= args.min_observations
    occupied = observed & ((hits.astype(np.float32) / np.maximum(visits, 1))
                           >= args.occupancy_ratio)
    free = observed & ~occupied
    # ROS trinary maps interpret occupancy=(255-gray)/255. With the emitted
    # free_thresh=0.25, gray 205 is incorrectly classified as free. Keep
    # unobserved cells between the free and occupied thresholds.
    image = np.full((size[1], size[0]), UNKNOWN_PIXEL, dtype=np.uint8)
    image[free] = 254
    image[occupied] = 0
    with (output / "map.pgm").open("wb") as stream:
        stream.write(f"P5\n{size[0]} {size[1]}\n255\n".encode("ascii"))
        stream.write(np.flipud(image).tobytes())
    (output / "map.yaml").write_text(
        "image: map.pgm\n"
        "mode: trinary\n"
        f"width: {size[0]}\n"
        f"height: {size[1]}\n"
        f"resolution: {args.map_resolution}\n"
        f"origin: [{minimum[0]}, {minimum[1]}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n",
        encoding="utf-8",
    )
    return {
        "width": int(size[0]),
        "height": int(size[1]),
        "resolution": args.map_resolution,
        "origin": minimum.tolist(),
        "observed_cells": int(observed.sum()),
        "free_cells": int(free.sum()),
        "occupied_cells": int(occupied.sum()),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frame-stride", type=int, default=5)
    parser.add_argument("--voxel-size", type=float, default=0.08)
    parser.add_argument("--range-min", type=float, default=0.3)
    parser.add_argument("--range-max", type=float, default=50.0)
    parser.add_argument("--sync-tolerance", type=float, default=0.02)
    parser.add_argument("--base-to-lidar", type=float, nargs=3,
                        default=(0.13011, 0.02329, 0.17598))
    parser.add_argument("--map-resolution", type=float, default=0.05)
    parser.add_argument("--map-margin", type=float, default=2.0)
    parser.add_argument("--floor-height", type=float, default=0.0)
    parser.add_argument("--floor-tolerance", type=float, default=0.10)
    parser.add_argument("--obstacle-min-height", type=float, default=0.15)
    parser.add_argument("--obstacle-max-height", type=float, default=1.2)
    parser.add_argument("--ray-angle-resolution", type=float, default=1.0)
    parser.add_argument("--min-observations", type=int, default=3)
    parser.add_argument("--occupancy-ratio", type=float, default=0.3)
    args = parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        raise SystemExit(f"output directory is not empty: {args.output}")
    args.output.mkdir(parents=True, exist_ok=True)
    stamps, poses = read_odometry(args.bag)
    points, scans, metrics = collect_points(args, stamps, poses)
    write_pcd(args.output / "0.pcd", points)
    write_pcd(args.output / "global.pcd", points)
    occupancy = write_occupancy(args.output, points, scans, args)
    (args.output / "index.txt").write_text(
        "0 0 0\n"
        "0 0 0 ./0.pcd\n"
        "# functional points\n"
        "start 0 0 0 0 0 0 1\n",
        encoding="utf-8",
    )
    report = {
        "schema": "robot_navigo.matrix_gt_reference_map",
        "schema_version": 1,
        "source_bag": str(args.bag.resolve()),
        "ground_truth_usage": "offline_map_generation_only",
        "parameters": vars(args) | {"bag": str(args.bag), "output": str(args.output)},
        "metrics": metrics | {"occupancy": occupancy},
    }
    (args.output / "reference_map_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
