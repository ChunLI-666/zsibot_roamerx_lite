"""Explicit simulation-only static-map geometry derived from the bound map image."""
import math
from pathlib import Path


def map_geometry(document, image_path):
    from PIL import Image
    image_path = Path(image_path)
    with Image.open(image_path) as image:
        width, height = image.size
    resolution = float(document['resolution'])
    origin = document['origin']
    if (width <= 0 or height <= 0 or not math.isfinite(resolution) or resolution <= 0
            or len(origin) != 3 or any(not math.isfinite(float(x)) for x in origin)
            or float(origin[2]) != 0.):
        raise ValueError('Unsupported full-map geometry')
    if any(k in document and int(document[k]) != size for k, size in (('width', width), ('height', height))):
        raise ValueError('Map YAML dimensions differ from image')
    return dict(size_x=width, size_y=height, resolution=resolution,
                origin_x=float(origin[0]), origin_y=float(origin[1]),
                width_m=width * resolution, height_m=height * resolution)


def apply_global_profile(parameters, geometry):
    """Shared by the real launcher and isolated static planner verification.

    Costmap2DROS takes integer metre dimensions; StaticLayer must resize the
    non-rolling grid to the actual image cells, verified separately from CDR.
    """
    parameters.update(rolling_window=False,width=math.ceil(geometry['width_m']),
                      height=math.ceil(geometry['height_m']),origin_x=geometry['origin_x'],
                      origin_y=geometry['origin_y'],resolution=.01)


def terminal_extent(geometry):
    return max(geometry['width_m'], geometry['height_m'])


def validate_actual_geometry(metadata, expected):
    origin = metadata.get('origin', {}).get('position', {})
    orientation = metadata.get('origin', {}).get('orientation', {})
    try:
        if (any(not math.isfinite(float(x)) for x in (metadata['resolution'],origin['x'],origin['y'],orientation['z'],orientation['w']))
                or int(metadata['size_x']) != expected['size_x'] or int(metadata['size_y']) != expected['size_y']
                or abs(float(metadata['resolution']) - expected['resolution']) > 1e-7
                or abs(float(origin['x']) - expected['origin_x']) > 1e-5
                or abs(float(origin['y']) - expected['origin_y']) > 1e-5
                or abs(float(orientation['z'])) > 1e-7 or abs(float(orientation['w']) - 1.) > 1e-7):
            raise ValueError('Actual full-map costmap geometry differs from map image')
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('Actual full-map costmap geometry incomplete') from exc
