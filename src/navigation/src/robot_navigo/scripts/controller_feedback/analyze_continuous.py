#!/usr/bin/env python3
"""Generate paired policy summaries and trajectory plots from completed trials."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True)
    root=parser.parse_args().root.resolve();runs=json.loads((root/'results.json').read_text())
    assert len(runs)==36
    assert json.loads((root/'identity_before.json').read_text())==json.loads((root/'identity_after.json').read_text())
    groups=[]
    for scenario in dict.fromkeys(r['scenario'] for r in runs):
        for arm in ('legacy','forward'):
            items=[r for r in runs if r['scenario']==scenario and r['arm']==arm]
            assert len(items)==3 and all(r['footprint_verified'] for r in items)
            succeeded=[r for r in items if r['success']]
            groups.append(dict(scenario=scenario,arm=arm,successes=len(succeeded),trials=3,
                success_time_range_sec=[min(r['observed_time_sec'] for r in succeeded),max(r['observed_time_sec'] for r in succeeded)] if succeeded else None,
                reverse_distance_range_m=[min(r['reverse_distance_m'] for r in items),max(r['reverse_distance_m'] for r in items)],
                longest_reverse_range_sec=[min(r['longest_reverse_sec'] for r in items),max(r['longest_reverse_sec'] for r in items)],
                final_xy_range_m=[min(r['final_xy_error'] for r in items),max(r['final_xy_error'] for r in items)],
                final_yaw_range_rad=[min(r['final_yaw_error'] for r in items),max(r['final_yaw_error'] for r in items)],
                collision_failures=sum(r['collision_checks_failed'] for r in items)))
    (root/'summary.json').write_text(json.dumps(dict(trials=36,groups=groups),indent=2)+'\n')
    fig,axes=plt.subplots(2,3,figsize=(13,7))
    for column,scenario in enumerate(('back_facing','side_offset','overshoot_outside')):
        for arm,color in [('legacy','tab:orange'),('forward','tab:blue')]:
            run=next(r for r in runs if r['scenario']==scenario and r['arm']==arm and r['seed']==42)
            rows=list(csv.DictReader(Path(run['command'][3]).open()))
            values={k:np.array([float(r[k]) for r in rows]) for k in ('t','x','y','vx','yaw')}
            axes[0,column].plot(values['x'],values['y'],color=color,label=arm)
            axes[1,column].plot(values['t'],values['vx'],color=color,label=arm)
        axes[0,column].plot([0,4],[0,0],'k--',alpha=.3,label='full reference')
        axes[0,column].scatter([4],[0],marker='*',color='green',s=100)
        axes[0,column].set_title(scenario+' / seed 42');axes[0,column].set_xlabel('x (m)');axes[0,column].set_ylabel('y (m)')
        axes[1,column].axhline(0,color='black',linewidth=.5);axes[1,column].set_xlabel('Simulated time (s)');axes[1,column].set_ylabel('Executed body vx (m/s)')
    axes[0,0].legend();fig.suptitle('Continuous production-controller feedback / ideal SE2, synthetic free space')
    fig.tight_layout();fig.savefig(root/'comparison.png',dpi=160);plt.close(fig)
    print(json.dumps(groups,indent=2))


if __name__=='__main__':main()
