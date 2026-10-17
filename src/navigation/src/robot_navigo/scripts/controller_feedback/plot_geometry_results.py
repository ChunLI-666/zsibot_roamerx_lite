#!/usr/bin/env python3
"""Plot frozen maps, actual padded polygons and controller feedback trajectories."""
import argparse
import csv
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True, type=Path)
    args = parser.parse_args()
    names = ['door_fixed_036', 'door_fixed_044', 'door_fixed_060',
             'recorded_context_reverse_10s', 'recorded_context_reverse_14s', 'recorded_context_reverse_20s']
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    for ax, name in zip(axes.flat, names):
        case_path = args.root/'cases'/(name+'_seed42.yaml')
        if not case_path.exists():
            ax.axis('off'); continue
        case = yaml.safe_load(case_path.read_text()); spec = case['map']
        grid = np.fromfile(spec['data'], dtype=np.uint8).reshape(spec['height'], spec['width'])
        extent = [spec['origin_x'], spec['origin_x']+spec['width']*spec['resolution'],
                  spec['origin_y'], spec['origin_y']+spec['height']*spec['resolution']]
        ax.imshow(grid, origin='lower', extent=extent, cmap='Greys', vmin=0, vmax=255, interpolation='none', alpha=.6)
        path = np.asarray(case['path'])
        ax.plot(path[:,0], path[:,1], '--', color='gray', label='frozen local path')
        ax.scatter(path[-1,0], path[-1,1], marker='*', color='black', s=70)
        for group, color in [('feature_disabled','#bf5936'), ('candidate','#2472a4')]:
            trace = args.root/group/(name+'_seed42.csv')
            rows = list(csv.DictReader(trace.open()))
            ax.plot([float(r['x']) for r in rows], [float(r['y']) for r in rows], color=color, label=group)
            metadata = yaml.safe_load(Path(str(trace)+'.footprint.yaml').read_text())
            points = np.asarray(metadata['effective_polygon'])
            for row, linestyle in [(rows[0], ':'), (rows[-1], '-')]:
                yaw = float(row['yaw'])
                rotation = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
                world = points @ rotation.T + [float(row['x']),float(row['y'])]
                ax.add_patch(Polygon(world, closed=True, fill=False, edgecolor=color, linestyle=linestyle))
        if name.startswith('door'):
            ax.set_xlim(-2,2); ax.set_ylim(-1,1)
        else:
            ax.set_xlim(extent[:2]); ax.set_ylim(extent[2:])
        ax.set(title=name, xlabel='odom x [m]', ylabel='odom y [m]', aspect='equal')
        ax.legend(fontsize=7)
    fig.suptitle('Production controller + ideal SE(2) feedback; seed 42\nSolid: final footprint; dotted: initial footprint. Frozen historical inflation retained in recorded context.')
    fig.tight_layout(); fig.savefig(args.root/'geometry_context_comparison.png', dpi=170)


if __name__ == '__main__':
    main()
