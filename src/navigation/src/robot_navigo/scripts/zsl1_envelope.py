#!/usr/bin/env python3
"""Reproduce a ZSL-1 collision-mesh XY envelope for explicitly supplied postures.

No joint angle, TF, gait range, payload, or safety margin is inferred. The output
covers the supplied model samples; it does not certify unrecorded robot motion.
Dependencies: numpy, scipy, PyYAML. Does not start ROS or command a robot.
"""
import argparse
import concurrent.futures
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
import yaml


SCHEMA = 1


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def numbers(text, length):
    result = np.array([float(x) for x in text.split()], dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f"Expected {length} finite values: {text}")
    return result


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if not np.isfinite(angle) or not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid rotation axis or angle")
    x, y, z = axis / norm
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def transform(xyz, rpy):
    xyz, rpy = np.asarray(xyz, dtype=float), np.asarray(rpy, dtype=float)
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite([xyz, rpy]).all():
        raise ValueError("Expected finite xyz and rpy triples")
    matrix = np.eye(4)
    matrix[:3, :3] = rotation([0, 0, 1], rpy[2]) @ rotation([0, 1, 0], rpy[1]) @ rotation([1, 0, 0], rpy[0])
    matrix[:3, 3] = xyz
    return matrix


def origin(element):
    return transform(numbers(element.get("xyz", "0 0 0"), 3), numbers(element.get("rpy", "0 0 0"), 3)) if element is not None else np.eye(4)


def stl_vertices(path):
    data = Path(path).read_bytes()
    if len(data) >= 84 and len(data) == 84 + 50 * struct.unpack_from("<I", data, 80)[0]:
        count = struct.unpack_from("<I", data, 80)[0]
        dtype = np.dtype([("normal", "<f4", (3,)), ("points", "<f4", (3, 3)), ("attribute", "<u2")])
        points = np.frombuffer(data, dtype=dtype, count=count, offset=84)["points"].reshape(-1, 3).astype(float)
    else:
        text = data.decode("ascii")
        vertices = re.findall(r"(?m)^\s*vertex\s+(\S+)\s+(\S+)\s+(\S+)\s*$", text)
        points = np.asarray(vertices, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3 or len(points) % 3 or not np.isfinite(points).all():
        raise ValueError(f"Malformed/nonfinite STL: {path}")
    return points


class Model:
    def __init__(self, urdf):
        self.path = Path(urdf).resolve()
        self.xml = ET.parse(self.path).getroot()
        link_elements = self.xml.findall("link")
        self.links = {link.attrib["name"]: link for link in link_elements}
        if len(self.links) != len(link_elements):
            raise ValueError("Duplicate link names")
        self.joints = {}
        children = set()
        for joint in self.xml.findall("joint"):
            name, kind = joint.attrib["name"], joint.attrib["type"]
            if name in self.joints or kind not in ("fixed", "revolute", "continuous", "prismatic") or joint.find("mimic") is not None:
                raise ValueError(f"Duplicate or unsupported joint: {name}")
            child, parent = joint.find("child").attrib["link"], joint.find("parent").attrib["link"]
            if child in children or child not in self.links or parent not in self.links:
                raise ValueError(f"Invalid URDF tree at {name}")
            children.add(child)
            axis = joint.find("axis")
            limit = joint.find("limit")
            self.joints[name] = dict(kind=kind, parent=parent, child=child, origin=origin(joint.find("origin")),
                axis=numbers(axis.get("xyz", "1 0 0") if axis is not None else "1 0 0", 3),
                limit=None if kind in ("fixed", "continuous") else (float(limit.attrib["lower"]), float(limit.attrib["upper"])))
            item = self.joints[name]
            if kind != "fixed":
                norm = np.linalg.norm(item["axis"])
                if not np.isfinite(norm) or norm < 1e-12:
                    raise ValueError(f"Invalid joint axis: {name}")
                item["axis"] /= norm
            if item["limit"] is not None and (not np.isfinite(item["limit"]).all() or item["limit"][0] > item["limit"][1]):
                raise ValueError(f"Invalid joint limits: {name}")
        roots = set(self.links) - children
        if len(roots) != 1:
            raise ValueError("URDF must have one root")
        self.root = roots.pop()
        self.meshes, self.mesh_manifest = {}, []
        for name, link in self.links.items():
            collisions = link.findall("collision")
            if not collisions:
                raise ValueError(f"Missing collision geometry: {name}")
            for index, collision in enumerate(collisions):
                geometry = collision.find("geometry")
                mesh = geometry.find("mesh") if geometry is not None else None
                if mesh is None:
                    raise ValueError(f"Only explicit collision meshes are supported: {name}")
                filename = mesh.attrib["filename"]
                if "://" in filename:
                    raise ValueError("Resolve package/URI mesh paths explicitly before generating")
                path = (self.path.parent / filename).resolve()
                scale = numbers(mesh.get("scale", "1 1 1"), 3)
                if (scale <= 0).any():
                    raise ValueError("Mesh scale must be positive")
                raw = stl_vertices(path)
                points = raw * scale
                local = origin(collision.find("origin"))
                points = points @ local[:3, :3].T + local[:3, 3]
                self.meshes[(name, index)] = points
                self.mesh_manifest.append(dict(link=name, collision_index=index, filename=filename,
                    sha256=sha256(path), bytes=path.stat().st_size, triangle_count=len(raw) // 3,
                    scale=scale.tolist(), collision_origin=local.tolist()))

    def poses(self, joints):
        moving = {name for name, joint in self.joints.items() if joint["kind"] != "fixed"}
        if set(joints) != moving:
            raise ValueError(f"Every moving joint must be explicit; missing={moving-set(joints)}, unknown={set(joints)-moving}")
        result = {self.root: np.eye(4)}
        pending = dict(self.joints)
        while pending:
            progress = False
            for name, joint in list(pending.items()):
                if joint["parent"] not in result:
                    continue
                motion = np.eye(4)
                if joint["kind"] != "fixed":
                    value = float(joints[name])
                    if not np.isfinite(value) or (joint["limit"] and not joint["limit"][0] <= value <= joint["limit"][1]):
                        raise ValueError(f"Joint {name} outside URDF limits: {value}")
                    if joint["kind"] == "prismatic":
                        motion[:3, 3] = joint["axis"] * value
                    else:
                        motion[:3, :3] = rotation(joint["axis"], value)
                result[joint["child"]] = result[joint["parent"]] @ joint["origin"] @ motion
                del pending[name]
                progress = True
            if not progress:
                raise ValueError("Disconnected or cyclic URDF tree")
        return result

    def sample(self, joints, navigation_from_root):
        poses = self.poses(joints)
        return {f"{name}:{index}": points @ (navigation_from_root @ poses[name])[:3, :3].T +
                (navigation_from_root @ poses[name])[:3, 3] for (name, index), points in self.meshes.items()}


def hull(points):
    xy = np.unique(np.asarray(points)[:, :2], axis=0)
    if len(xy) < 3:
        raise ValueError("At least three distinct projected points required")
    return xy[ConvexHull(xy).vertices]


def outside_count(points, polygon, tolerance=1e-9):
    # Exact static coverage test: a convex polygon containing all mesh vertices
    # contains every triangle, not merely a random subset of surface samples.
    edges = np.roll(polygon, -1, axis=0) - polygon
    norms = np.linalg.norm(edges, axis=1)
    if (norms <= 0).any():
        raise ValueError("Degenerate polygon")
    count, maximum = 0, 0.0
    for start in range(0, len(points), 8192):
        delta = points[start:start + 8192, None, :2] - polygon[None, :, :]
        signed = (edges[None, :, 0] * delta[:, :, 1] - edges[None, :, 1] * delta[:, :, 0]) / norms
        violation = np.maximum(0.0, -signed.min(axis=1))
        count += int(np.sum(violation > tolerance))
        maximum = max(maximum, float(violation.max(initial=0.0)))
    return count, maximum


def checked_costmap_padding(polygon, padding):
    padded = np.asarray(polygon) + np.sign(polygon) * padding
    edges = np.roll(padded, -1, axis=0) - padded
    next_edges = np.roll(edges, -1, axis=0)
    cross = edges[:, 0] * next_edges[:, 1] - edges[:, 1] * next_edges[:, 0]
    if (np.linalg.norm(edges, axis=1) < 1e-12).any() or (cross < -1e-12).any() or not (cross > 0).any():
        raise ValueError("Coordinate-wise costmap padding makes this polygon nonconvex; use rectangle or a reviewed convex padded polygon")
    points = np.column_stack([polygon, np.zeros(len(polygon))])
    if outside_count(points, padded)[0]:
        raise ValueError("Padded footprint does not contain the raw footprint")
    return padded


def import_home(model, matrix_xml):
    path = Path(matrix_xml).resolve()
    xml = ET.parse(path).getroot()
    key = xml.find("./keyframe/key[@name='home']")
    if key is None or xml.find("compiler").get("angle") != "radian":
        raise ValueError("Expected an explicit radian home keyframe")
    qpos = numbers(key.attrib["qpos"], 19)
    if not np.allclose(qpos[3:7], [1, 0, 0, 0], atol=1e-12):
        raise ValueError("Only the verified level Matrix home frame is supported by this importer")
    matrix_joints = xml.findall("./worldbody//joint")
    if len(matrix_joints) != 12:
        raise ValueError("Expected 12 Matrix scalar joints")
    prefixes = {"FAR": "FR", "FBL": "FL", "RAR": "RR", "RBL": "RL"}
    mapping, values = [], {}
    parent_map = {child: parent for parent in xml.iter() for child in parent}
    for joint, value in zip(matrix_joints, qpos[7:]):
        matrix_name = joint.attrib["name"]
        source_prefix, suffix = matrix_name.split("_", 1)
        name = prefixes[source_prefix] + "_" + suffix
        urdf_joint = model.joints[name]
        body = parent_map[joint]
        parent_body = parent_map[body]
        parent_name = parent_body.get("name")
        if parent_name == "base_link":
            expected_parent = model.root
        else:
            prefix, remainder = parent_name.split("_", 1)
            expected_parent = prefixes[prefix] + "_" + remainder
        if expected_parent != urdf_joint["parent"] or not np.allclose(numbers(joint.get("pos", "0 0 0"), 3), 0):
            raise ValueError(f"Matrix/URDF parent or joint origin differs: {matrix_name}")
        # These matching origins/axes establish qpos sign/order correspondence;
        # similar names alone would not establish an angle convention.
        if not np.allclose(numbers(joint.attrib["axis"], 3), urdf_joint["axis"], atol=1e-12) or not np.allclose(
                numbers(body.get("pos", "0 0 0"), 3), urdf_joint["origin"][:3, 3], atol=1e-9):
            raise ValueError(f"Matrix/URDF kinematics differ: {matrix_name}")
        if any(k in body.attrib for k in ("quat", "euler", "axisangle")) or not np.allclose(urdf_joint["origin"][:3, :3], np.eye(3)):
            raise ValueError("Nonidentity body rotations require an explicit correspondence adapter")
        values[name] = float(value)
        mapping.append(dict(matrix_joint=matrix_name, urdf_joint=name, position_rad=float(value)))
    for leg in ("FL", "FR", "RL", "RR"):
        foot = xml.find(f".//geom[@mesh='{leg}_FOOT_LINK']")
        fixed = model.joints[leg + "_FOOT_JOINT"]
        if foot is None or not np.allclose(numbers(foot.get("quat", "1 0 0 0"), 4), [1, 0, 0, 0]) or not np.allclose(
                transform(numbers(foot.get("pos", "0 0 0"), 3), [0, 0, 0]), fixed["origin"], atol=1e-9):
            raise ValueError(f"Matrix/URDF fixed foot origin differs: {leg}")
    model.poses(values)  # Validate the imported home against official joint limits.
    meshdir = path.parent / xml.find("compiler").attrib["meshdir"]
    asset_matches = []
    for record in model.mesh_manifest:
        asset = meshdir / Path(record["filename"]).name
        digest = sha256(asset)
        if digest != record["sha256"]:
            raise ValueError(f"Matrix mesh differs from official model: {asset.name}")
        asset_matches.append(dict(filename=asset.name, sha256=digest))
    return dict(schema_version=SCHEMA, source=dict(kind="matrix_mjcf_home_keyframe", path=str(path), sha256=sha256(path),
        keyframe="home", qpos=qpos.tolist(), joint_mapping=mapping, matching_assets=asset_matches),
        root_frame=model.root, frame_id="base_link", navigation_from_root=dict(xyz=[0, 0, 0], rpy=[0, 0, 0],
        evidence="Matching Matrix base_link / URDF BASE_LINK mesh and kinematic origins; model-only identity convention",
        hardware_verified=False), footprint_padding=0.01,
        padding_reason="Existing controller experiment costmap padding, not a measured gait allowance",
        payloads=[], payloads_verified=False, motion_envelope_verified=False,
        supported_samples=["static_matrix_home"],
        excluded_modes=["physical_stance_variation", "forward_gait", "turning_gait", "lateral_gait", "recovery_gait"],
        samples=[dict(name="matrix_home", joints=values, root_rpy=[0, 0, 0], source_timestamp=None)],
        assumptions=["No sensor or payload envelope is present in this URDF.",
            "The Matrix home keyframe is model evidence, not recorded physical robot joint feedback.",
            "Navigation frame identity and level root are model conventions; verify physical TF and mounting before deployment."])


def verify_official(model, commit):
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise ValueError("Use a full immutable Git commit ID")
    root = model.path.parent.parent
    paths = [model.path, *sorted({(model.path.parent / x["filename"]).resolve() for x in model.mesh_manifest})]
    def check(path):
        relative = "zsl-1/" + str(path.relative_to(root))
        url = f"https://raw.githubusercontent.com/zsibot/genisom_model/{commit}/{relative}"
        with urllib.request.urlopen(url, timeout=60) as response:
            remote = response.read()
        local = path.read_bytes()
        if local != remote:
            raise ValueError(f"Official source mismatch: {relative}")
        return dict(path=relative, url=url, sha256=hashlib.sha256(local).hexdigest(),
            remote_sha256=hashlib.sha256(remote).hexdigest(), bytes=len(local), matches_remote=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        records = list(executor.map(check, paths))
    return dict(repo="https://github.com/zsibot/genisom_model", commit=commit, files=records)


def generate(model, postures_path, official_path, shape="rectangle"):

    postures_path, official_path = Path(postures_path), Path(official_path)
    doc = yaml.safe_load(postures_path.read_text())
    official = json.loads(official_path.read_text())
    official_files = {Path(row["path"]).name: row for row in official["files"]}
    for filename, digest in [(model.path.name, sha256(model.path)), *[(Path(x["filename"]).name, x["sha256"]) for x in model.mesh_manifest]]:
        record = official_files[filename]
        if record["sha256"] != digest or record["remote_sha256"] != digest or record["matches_remote"] is not True:
            raise ValueError(f"Model differs from the frozen official source: {filename}")
    if doc.get("schema_version") != SCHEMA or doc.get("root_frame") != model.root or not doc.get("frame_id"):
        raise ValueError("Invalid posture schema/root/frame")
    if not doc.get("samples"):
        raise ValueError("No explicit postures")
    offset = doc["navigation_from_root"]
    base = transform(offset["xyz"], offset["rpy"])
    padding = float(doc["footprint_padding"])
    if not np.isfinite(padding) or padding < 0:
        raise ValueError("Invalid costmap padding")
    clouds, summaries = [], []
    for sample in doc["samples"]:
        # A root tilt is an explicit observed/model posture input, never a guessed gait interval.
        pose = base @ transform([0, 0, 0], sample["root_rpy"])
        links = model.sample(sample["joints"], pose)
        points = np.concatenate(list(links.values()))
        clouds.append(points)
        summaries.append(dict(name=sample["name"], vertices=len(points), minimum_xyz=points.min(axis=0).tolist(),
            maximum_xyz=points.max(axis=0).tolist(), link_bounds={k: dict(minimum=v.min(axis=0).tolist(), maximum=v.max(axis=0).tolist()) for k, v in links.items()}))
    if doc.get("payloads"):
        raise ValueError("Payload geometry is not implemented; include it explicitly in the URDF before claiming coverage")
    points = np.concatenate(clouds)
    mesh_hull = hull(points)
    polygon = mesh_hull
    count, violation = outside_count(points, polygon)
    if count:
        raise ValueError(f"Envelope failed coverage: {count} vertices")
    minimum, maximum = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
    rectangle = [[float(minimum[0]), float(minimum[1])], [float(maximum[0]), float(minimum[1])],
        [float(maximum[0]), float(maximum[1])], [float(minimum[0]), float(maximum[1])]]
    if shape == "rectangle":
        polygon = np.asarray(rectangle)
    elif shape != "convex_hull":
        raise ValueError("Unsupported footprint shape")
    count, violation = outside_count(points, polygon)
    # Costmap padFootprint is a coordinate-wise sign offset, not a radial margin.
    padded = checked_costmap_padding(polygon, padding)
    padded_radius = float(np.linalg.norm(padded, axis=1).max())
    resolution = 0.05
    inflation = math.ceil((padded_radius + resolution) / resolution) * resolution
    result = dict(schema_version=SCHEMA, frame_id=doc["frame_id"], footprint=polygon.tolist(), footprint_padding=padding,
        footprint_shape=shape, convex_hull=mesh_hull.tolist(), padded_footprint=padded.tolist(),
        costmap_resolution=resolution, inflation_radius=inflation,
        inflation_reason="Rounded up to one 0.05 m grid cell beyond the padded circumscribed radius for the MPPI center-cost gate",
        rectangle=rectangle, bounds=dict(minimum_xy=minimum.tolist(), maximum_xy=maximum.tolist(),
        length_m=float(maximum[0]-minimum[0]), width_m=float(maximum[1]-minimum[1]),
        unpadded_circumscribed_radius_m=float(np.linalg.norm(polygon, axis=1).max()),
        padded_circumscribed_radius_m=padded_radius),
        envelope_kind="model_posture_union", motion_envelope_verified=False, hardware_deployment_ready=False,
        supported_samples=doc["supported_samples"], excluded_modes=doc["excluded_modes"],
        assumptions=doc["assumptions"], metadata=dict(urdf_sha256=sha256(model.path),
        official_repository=official["repo"], official_commit=official["commit"],
        official_manifest_sha256=sha256(official_path), posture_file_sha256=sha256(postures_path),
        generator_sha256=sha256(__file__), mesh_count=len(model.mesh_manifest), meshes=model.mesh_manifest,
        navigation_from_root=offset, posture_source=doc["source"], samples=summaries,
        coverage=dict(vertices=len(points), triangles=len(points)//3, outside_vertices=count,
            maximum_numerical_violation_m=violation, tolerance_m=1e-9, method="all triangle vertices against convex hull halfplanes")))
    # Same explicit polygon/padding for both costmaps. Do not enlarge scan self-filter.
    parameters = dict(footprint=json.dumps(result["footprint"], separators=(",", ":")), footprint_padding=padding,
        inflation_layer=dict(inflation_radius=inflation))
    overlay = {name: {name: {"ros__parameters": copy.deepcopy(parameters)}} for name in ("global_costmap", "local_costmap")}
    return result, overlay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    verify = sub.add_parser("verify-official")
    verify.add_argument("--urdf", required=True)
    verify.add_argument("--commit", required=True)
    verify.add_argument("--output", required=True)
    home = sub.add_parser("import-home")
    home.add_argument("--urdf", required=True)
    home.add_argument("--matrix-xml", required=True)
    home.add_argument("--output", required=True)
    build = sub.add_parser("generate")
    build.add_argument("--urdf", required=True)
    build.add_argument("--postures", required=True)
    build.add_argument("--official-manifest", required=True)
    build.add_argument("--output", required=True)
    build.add_argument("--overlay", required=True)
    build.add_argument("--shape", choices=("rectangle", "convex_hull"), default="rectangle")
    args = parser.parse_args()
    model = Model(args.urdf)
    if args.command == "verify-official":
        result = verify_official(model, args.commit)
        Path(args.output).write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(dict(files=len(result["files"]), all_match=True)))
        return
    if args.command == "import-home":
        result = import_home(model, args.matrix_xml)
    else:
        result, overlay = generate(model, args.postures, args.official_manifest, args.shape)
        Path(args.overlay).write_text("# Model-only collision overlay; not a measured gait envelope.\n" + yaml.safe_dump(overlay, sort_keys=False))
    Path(args.output).write_text(yaml.safe_dump(result, sort_keys=False))
    print(json.dumps(dict(output=args.output, mesh_count=len(model.mesh_manifest), bounds=result.get("bounds")), indent=2))


if __name__ == "__main__":
    main()
