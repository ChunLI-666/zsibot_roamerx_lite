"""Synthetic geometric boundary cases, kept distinct from recorded environments."""
import math
from pathlib import Path
import numpy as np


def minimum_width(polygon):
    edges = np.roll(polygon, -1, axis=0)-polygon
    normals = np.column_stack((-edges[:, 1], edges[:, 0]))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    return float(min(np.ptp(polygon @ normal) for normal in normals))


def cell_intersects_polygon(polygon, center, resolution):
    """SAT used only to label known synthetic initial states before executing."""
    edges = np.roll(polygon, -1, axis=0)-polygon
    axes = np.vstack((np.eye(2), np.column_stack((-edges[:, 1], edges[:, 0]))))
    for axis in axes:
        projection = polygon @ axis
        midpoint = float(np.dot(center, axis))
        radius = resolution*.5*np.abs(axis).sum()
        if projection.max() < midpoint-radius-1e-10 or projection.min() > midpoint+radius+1e-10:
            return False
    return True


def add_geometry_cases(cases, common, output, polygon):
    output = Path(output)
    resolution, size, origin = .02, 600, -6.
    coordinates = origin+(np.arange(size)+.5)*resolution
    map_spec = dict(width=size, height=size, resolution=resolution, origin_x=origin, origin_y=origin)
    path = [[float(x), 0., 0.] for x in np.linspace(-1.5, 1.5, 121)]
    span_y = float(np.ptp(polygon[:, 1]))
    min_width = minimum_width(polygon)
    # Physical gap widths are computed from the opened grid cells, not labels.
    for name, requested_width in [('door_fixed_036', .36), ('door_fixed_044', .44), ('door_fixed_060', .60), ('door_narrow', max(.2, min_width-.12)),
                                   ('door_clear', span_y+.20)]:
        grid = np.zeros((size, size), dtype=np.uint8)
        wall_columns = np.abs(coordinates) < .06
        open_rows = np.abs(coordinates) < requested_width/2
        grid[np.ix_(~open_rows, wall_columns)] = 254
        actual_width = float(open_rows.sum()*resolution)
        file = output/(name+'.costmap'); grid.tofile(file)
        impossible = actual_width < min_width-1e-6
        guard_limited = not impossible and actual_width <= span_y+4*resolution+1e-6
        expected = 'blocked_geometry' if impossible else ('clearance_limited' if guard_limited else 'reachable_feedback')
        cases.append(dict(common, name=name, initial=[-1.5, 0., 0.], path=path,
                          map=dict(map_spec, data=str(file)), steps=800,
                          expected_behavior=expected,
                          geometry=dict(kind='synthetic_wall_and_door', requested_gap_m=requested_width,
                                        actual_gap_m=actual_width, polygon_minimum_width_m=min_width,
                                        forward_width_m=span_y,
                                        guard_clearance_assumption_m=4*resolution,
                                        guard_limited=guard_limited,
                                        impossibility_basis='gap below minimum support width at every heading' if impossible else None)))
    # Find a cell that is outside the initial polygon, but inside the 90-degree
    # swept polygon. This adapts to the actual model instead of retaining .29 m.
    radius = float(np.linalg.norm(polygon, axis=1).max())
    candidate_cells = coordinates[np.abs(coordinates) <= radius+resolution]
    chosen = None
    for y in candidate_cells:
        for x in candidate_cells:
            point = np.array([x, y])
            if cell_intersects_polygon(polygon, point, resolution):
                continue
            for yaw in np.linspace(.05, math.pi/2, 32):
                rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
                if cell_intersects_polygon(polygon @ rotation.T, point, resolution):
                    chosen = (float(x), float(y), float(yaw)); break
            if chosen:
                break
        if chosen:
            break
    if chosen is None:
        raise ValueError('No rotation-only blocker found for polygon')
    grid = np.zeros((size, size), dtype=np.uint8)
    ix, iy = [int((coordinate-origin)/resolution) for coordinate in chosen[:2]]
    grid[iy, ix] = 254
    file = output/'model_rotation_sweep.costmap'; grid.tofile(file)
    cases.append(dict(common, name='model_rotation_sweep', initial=[0., 0., 0.],
                      path=[[0., float(y), math.pi/2] for y in np.linspace(0, 2, 81)],
                      map=dict(map_spec, data=str(file)), steps=50,
                      expected_behavior='intentional_hold',
                      geometry=dict(kind='synthetic_single_cell_swept_collision', obstacle_center=chosen[:2],
                                    first_sampled_intersection_yaw=chosen[2], initially_collision_free=True)))
    grid = np.zeros((size, size), dtype=np.uint8)
    grid[size//2, size//2] = 254
    file = output/'initial_overlap.costmap'; grid.tofile(file)
    cases.append(dict(common, name='initial_overlap', initial=[0.,0.,0.],
                      path=[[0.,0.,0.],[.01,0.,0.]], map=dict(map_spec,data=str(file)),
                      steps=1, expected_behavior='invalid_initial_state',
                      geometry=dict(reason='synthetic obstacle inside footprint at requested endpoint')))
    # Retain the historical fixed-cell scene, but mark an invalid start if the
    # larger model already occupies the obstacle instead of demanding arrival.
    for case in cases:
        if case['name'] == 'warehouse_blocked_rotation':
            if cell_intersects_polygon(polygon, np.array([.05, .29]), .05):
                case['expected_behavior'] = 'invalid_initial_state'
                case['geometry'] = dict(reason='historical fixed blocker lies inside the new starting footprint')
