import math
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from scene import Scene, integrate


def test_raycast_hits_map_wall_and_treats_unknown_as_occupied(tmp_path):
    grid=np.full((40,40),255,dtype=np.uint8);grid[:,30:]=0;grid[0,:]=205
    Image.fromarray(grid).save(tmp_path/'map.pgm')
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[0.,0.,0.],free_thresh=.196)))
    scene=Scene(tmp_path/'map.yaml');ranges=scene.scan([1.,1.,0.])
    assert abs(ranges[180]-.5)<=.0251
    # A cell inside the unknown boundary row is not reported as free space.
    assert not scene.free[-1,10]
    assert np.isfinite(ranges).all()


def test_body_frame_motion_and_rotation_feedback():
    pose=integrate([1.,2.,math.pi/2],[.1,0.,0.],1.)
    assert abs(pose[0]-1.)<1e-12 and abs(pose[1]-2.1)<1e-12
    pose=integrate([1.,2.,0.],[0.,0.,.1],1.)
    assert pose[:2]==[1.,2.] and abs(pose[2]-.1)<1e-12
