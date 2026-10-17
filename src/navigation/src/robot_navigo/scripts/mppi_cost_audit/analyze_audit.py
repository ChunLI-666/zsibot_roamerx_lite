#!/usr/bin/env python3
"""Validate cost decomposition and summarize sensitivity without claiming historical replay."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import yaml

TERMS=['ConstraintCritic','CostCritic','GoalCritic','PathAlignCritic','GoalAngleCritic',
       'PathFollowCritic','PathAngleCritic','PreferForwardCritic','VelocityDeadbandCritic','regularization']


def read_trace(directory):
    rows=list(csv.DictReader((directory/'trace.csv').open()))
    data={key:np.array([float(row[key]) for row in rows]) for key in rows[0] if key!='proposal'}
    data['proposal']=[row['proposal'] for row in rows]
    return data


def summarize(root):
    records=json.loads((root/'runs.json').read_text());protocol=json.loads((root/'inputs/protocol.json').read_text())
    snapshots={r['name']:r for r in protocol['snapshots']};summaries=[];pairs={}
    assert len(records)==180
    assert json.loads((root/'identity_before.json').read_text())==json.loads((root/'identity_after.json').read_text())
    for run in records:
        directory=Path(run['directory']);metadata=yaml.safe_load((directory/'trace.yaml').read_text());d=read_trace(directory)
        assert run['exit_code']==0 and metadata['production_manager_max_difference']<=1e-5
        assert all(np.isfinite(value).all() for key,value in d.items() if key!='proposal')
        assert np.allclose(sum(d[k] for k in TERMS),d['total'],rtol=2e-5,atol=2e-5)
        exp=np.exp(-(d['total']-min(d['total']))/metadata['temperature']);expected=exp/exp.sum()
        assert np.allclose(expected,d['weight'],rtol=2e-4,atol=2e-6)
        if not metadata['skipped_after_failure'] and metadata['goal_distance']>.5:
            # Native float32 accumulated scores quantize small additions after a large
            # collision cost. The observed increment can differ by up to one ULP.
            rounding=np.spacing(d['total'].astype(np.float32)).astype(float)+2e-5
            assert np.all(np.abs(d['PreferForwardCritic']-run['weight']*d['reverse_distance'])<=rounding)
        reverse=d['mean_control_vx']<0;forward=~reverse
        def best(mask):
            if not mask.any():return None
            i=int(np.flatnonzero(mask)[np.argmin(d['total'][mask])])
            return dict(sample=i,mean_vx=float(d['mean_control_vx'][i]),total=float(d['total'][i]),
                        **{key:float(d[key][i]) for key in TERMS})
        item={key:run[key] for key in ['name','snapshot','seed','warm','weight','fixed']}
        item.update(metadata)
        item.update(reverse_sample_fraction=float(np.mean(reverse)),reverse_weight_mass=float(d['weight'][reverse].sum()),
                    weighted_raw_control_vx=float(np.sum(d['weight']*d['first_control_vx'])),
                    best_reverse=best(reverse),best_forward=best(forward),
                    weighted_costs={key:float(np.sum(d[key]*d['weight'])) for key in TERMS})
        if run['fixed']:
            item['fixed_costs']={}
            for label in dict.fromkeys(d['proposal']):
                i=d['proposal'].index(label)
                item['fixed_costs'][label]={key:float(d[key][i]) for key in TERMS+['total']}
        else:
            pair=(run['snapshot'],run['seed'],run['warm'])
            signature=np.column_stack([d[k] for k in ['mean_control_vx','first_control_vx','end_x','end_y','end_yaw']])
            if pair in pairs:assert np.array_equal(signature,pairs[pair])
            else:pairs[pair]=signature
        summaries.append(item)
    grouped=[]
    for warm in protocol['warm_starts']:
        for weight in protocol['weights']:
            items=[s for s in summaries if not s['fixed'] and s['warm']==warm and s['weight']==weight]
            valid=[s for s in items if s['command_valid']]
            grouped.append(dict(warm=warm,weight=weight,runs=len(items),valid_runs=len(valid),
                negative_output_count=sum(s['diagnostic_command'][0]<-1e-6 for s in valid),
                reverse_at_or_above_005_deadband_count=sum(s['diagnostic_command'][0]<=-.05 for s in valid),
                mean_vx=float(np.mean([s['diagnostic_command'][0] for s in valid])),
                mean_reverse_weight_mass=float(np.mean([s['reverse_weight_mass'] for s in valid]))))
    result=dict(scope=protocol['scope'],runs=len(records),sampled_runs=162,fixed_proposal_runs=18,
                checks=dict(native_manager_equal=True,finite_costs=True,cost_sum_consistent=True,
                            softmax_consistent=True,prefer_forward_integral_consistent_with_float32_ulp=True,
                            paired_weights_have_identical_exported_rollout_summaries=True,inputs_and_runtime_unchanged=True),
                grouped=grouped,runs_detail=summaries)
    (root/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'checks':result['checks'],'grouped':grouped},indent=2))
    # Native critic costs for fixed alternatives at the start of each reverse window.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(15,5))
    labels=['stop','reverse','forward','turn_left','turn_right']
    for ax,window in zip(axes,['reverse_10s','reverse_14s','reverse_20s']):
        item=next(s for s in summaries if s['fixed'] and s['warm']=='zero' and s['snapshot']==window+'_start')
        bottom=np.zeros(len(labels))
        for key in TERMS:
            values=np.array([item['fixed_costs'][label][key] for label in labels])
            if np.max(np.abs(values))>1e-7:
                ax.bar(labels,values,bottom=bottom,label=key);bottom+=values
        ax.set_ylim(0,max(bottom)*1.15)
        for label,value in zip(labels,bottom):ax.text(label,value+.04,f'{value:.3f}',ha='center',fontsize=8)
        ax.set_title(window+' / zero nominal');ax.set_ylabel('Native total cost');ax.tick_params(axis='x',rotation=35)
    handles=[];names=[]
    for ax in axes:
        h,n=ax.get_legend_handles_labels()
        for handle,name in zip(h,n):
            if name not in names:handles.append(handle);names.append(name)
    fig.legend(handles,names,loc='lower center',ncol=4)
    fig.suptitle('Controlled fixed alternatives; recorded context, not original MPPI samples')
    fig.tight_layout(rect=(0,.14,1,.94));fig.savefig(root/'fixed_cost_comparison.png',dpi=160);plt.close(fig)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True)
    summarize(parser.parse_args().root.resolve())
