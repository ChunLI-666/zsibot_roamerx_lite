"""Sensor-only, planar test world. It contains no navigation policy."""
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


def integrate(pose, velocity, dt):
    """Exact constant body-twist SE(2) integration, stable at zero yaw rate."""
    x, y, yaw = pose
    vx, vy, wz = velocity
    half = wz * dt / 2.0
    sinc = math.sin(half) / half if abs(half) > 1e-8 else 1.0 - half * half / 6.0
    scale, angle = dt * sinc, yaw + half
    return [x + scale * (math.cos(angle) * vx - math.sin(angle) * vy),
            y + scale * (math.sin(angle) * vx + math.cos(angle) * vy),
            math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))]


def error_to_goal(pose, goal):
    return dict(xy_m=math.hypot(pose[0] - goal[0], pose[1] - goal[1]),
                yaw_rad=abs(math.atan2(math.sin(pose[2] - goal[2]),
                                       math.cos(pose[2] - goal[2]))))


def write_open_map(directory):
    """8 m square, 5 cm cells, with physical walls around its perimeter."""
    directory = Path(directory)
    directory.mkdir(parents=True)
    image = np.full((160, 160), 254, dtype=np.uint8)
    image[:2, :] = image[-2:, :] = image[:, :2] = image[:, -2:] = 0
    Image.fromarray(image).save(directory / 'map.pgm')
    path = directory / 'map.yaml'
    path.write_text(yaml.safe_dump(dict(image='map.pgm', resolution=.05,
        origin=[0., 0., 0.], negate=0, occupied_thresh=.65, free_thresh=.196)))
    return path


class Scene:
    """Ray-cast occupied/unknown cells in the static test map."""
    def __init__(self, map_yaml):
        path = Path(map_yaml)
        metadata = yaml.safe_load(path.read_text())
        image = np.asarray(Image.open(path.parent / metadata['image']))
        if image.ndim == 3:
            image = image[:, :, :3].mean(axis=2)
        probability = (255 - image.astype(float)) / 255
        if metadata.get('negate', 0):
            probability = 1 - probability
        self.free = np.flipud(probability < metadata.get('free_thresh', .196))
        self.resolution = float(metadata['resolution'])
        self.origin = np.asarray(metadata['origin'][:2], dtype=float)
        if abs(metadata['origin'][2]) > 1e-9:
            raise ValueError('Test plant requires an axis-aligned map')
        self.ranges = np.arange(self.resolution / 2, 8., self.resolution / 2)
        self.angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)

    def scan(self, pose):
        angles = self.angles + pose[2]
        directions = np.stack((np.cos(angles), np.sin(angles)), axis=1)
        points = np.asarray(pose[:2])[None, None, :] + self.ranges[None, :, None] * directions[:, None, :]
        indices = np.floor((points - self.origin) / self.resolution).astype(int)
        ix, iy = indices[:, :, 0], indices[:, :, 1]
        valid = (ix >= 0) & (iy >= 0) & (ix < self.free.shape[1]) & (iy < self.free.shape[0])
        hit = ~valid
        hit[valid] = ~self.free[iy[valid], ix[valid]]
        return np.where(hit.any(axis=1), self.ranges[np.argmax(hit, axis=1)],
                        np.inf).astype(np.float32).tolist()
