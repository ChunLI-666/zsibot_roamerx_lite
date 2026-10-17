"""Map-backed test plant geometry. No ROS or production navigation replacements."""
import math
from pathlib import Path
import numpy as np
from PIL import Image
import yaml


class Scene:
    def __init__(self, map_yaml):
        self.path = Path(map_yaml).resolve()
        self.metadata = yaml.safe_load(self.path.read_text())
        pixels = np.asarray(Image.open(self.path.parent/self.metadata['image']))
        if pixels.ndim == 3:
            pixels = pixels[:, :, :3].mean(axis=2)
        occupancy_probability=(255-pixels.astype(float))/255
        if self.metadata.get('negate',0):occupancy_probability=1-occupancy_probability
        free=occupancy_probability<self.metadata.get('free_thresh',.196)
        occupied=occupancy_probability>self.metadata.get('occupied_thresh',.65)
        self.occupancy=np.flipud(np.where(free,0,np.where(occupied,1,2))).astype(np.uint8)
        self.free=self.occupancy==0
        self.resolution = float(self.metadata['resolution'])
        self.origin = np.asarray(self.metadata['origin'][:2], dtype=float)
        if abs(self.metadata['origin'][2]) > 1e-9:
            raise ValueError('Fixture supports axis-aligned map origins only')
        self.ranges = np.arange(self.resolution*.5, 8.0, self.resolution*.5)
        self.angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)

    def scan(self, pose):
        angles = self.angles+pose[2]
        points = np.asarray(pose[:2])[None, None, :] + self.ranges[None, :, None]*np.stack((np.cos(angles),np.sin(angles)),axis=1)[:,None,:]
        indices = np.floor((points-self.origin)/self.resolution).astype(int)
        ix, iy = indices[:,:,0], indices[:,:,1]
        valid = (ix>=0)&(iy>=0)&(ix<self.free.shape[1])&(iy<self.free.shape[0])
        occupied = ~valid
        occupied[valid] = ~self.free[iy[valid],ix[valid]]
        first = np.argmax(occupied, axis=1)
        return np.where(occupied.any(axis=1),self.ranges[first],np.inf).astype(np.float32).tolist()


def integrate(pose, command, dt):
    x,y,yaw = pose; vx,vy,wz = command
    half_angle = wz*dt*.5
    # Exact constant body twist; stable at zero angular velocity.
    scale = dt*(math.sin(half_angle)/half_angle if abs(half_angle)>1e-8 else 1.-half_angle*half_angle/6.)
    midpoint = yaw+half_angle
    return [x+(math.cos(midpoint)*vx-math.sin(midpoint)*vy)*scale,
            y+(math.sin(midpoint)*vx+math.cos(midpoint)*vy)*scale,
            math.atan2(math.sin(yaw+wz*dt),math.cos(yaw+wz*dt))]
