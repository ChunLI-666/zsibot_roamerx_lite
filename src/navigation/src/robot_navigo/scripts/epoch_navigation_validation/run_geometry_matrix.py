#!/usr/bin/env python3
"""Reproducible static geometry matrix for the actual isolated Nav2 stack."""
import argparse,json,math,subprocess,sys
from pathlib import Path
import numpy as np
from PIL import Image
import yaml

def prepare_maps(output):
 for name,gap in [('open',None),('door080',.8),('door035',.35)]:
  a=np.full((160,160),254,dtype=np.uint8);a[:2,:]=0;a[-2:,:]=0;a[:,:2]=0;a[:,-2:]=0
  if gap:
   a[:,79:81]=0;n=round(gap/.05);a[80-n//2:80+(n+1)//2,79:81]=254
  p=output/name;p.mkdir();Image.fromarray(np.flipud(a)).save(p/'map.pgm')
  (p/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[0.,0.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))

def main():
 p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 args.output.mkdir(parents=True,exist_ok=False);prepare_maps(args.output)
 baseline={'controller_server':{'ros__parameters':{'FollowPath':{'forward_alignment':{'exit_angle':.20}}}}}
 slow={'controller_server':{'ros__parameters':{'FollowPath':{'forward_alignment':{'exit_angle':.20}}}},'velocity_smoother':{'ros__parameters':{'max_decel':[-1.5,-1.5,-.2]}}}
 cases=[('open90','open','normal',[2,4,math.pi/2],[4,4,0],'arrive',baseline),
        ('slow_decel90','open','normal',[2,4,math.pi/2],[4,4,0],'arrive',slow),
        ('open180','open','normal',[2,4,math.pi],[4,4,0],'arrive',{}),
        ('final90','open','normal',[2,4,0],[4,4,math.pi/2],'arrive',{}),
        ('door080','door080','normal',[2.7,4,math.pi/2],[5,4,0],'arrive',{}),
        ('door035','door035','normal',[2.7,4,0],[5,4,0],'blocked',{}),
        ('loss90','open','loss_recovery',[2,4,math.pi/2],[4,4,0],'arrive',{}),
        ('curved_track','open','normal',[2,4,0],[4,5,0],'arrive',{})]
 (args.output/'protocol.json').write_text(json.dumps(dict(cases=cases,plant='SE2 chord interpolation, no inertia or SDK',map_size_m=[8,8],resolution_m=.05,hardware_io=False,domain=188),indent=2))
 results=[]
 for name,mapname,scenario,initial,goal,expected,overlay in cases:
  command=[sys.executable,str(Path(__file__).with_name('run_stack.py')),'--workspace',str(args.workspace),'--output',str(args.output/name),'--map',str(args.output/mapname/'map.yaml'),'--scenario',scenario,'--initial',*map(str,initial),'--goal',*map(str,goal),'--geometry','--expected',expected,'--observe-seconds','35','--timeout','120']
  if overlay:
   overlay_path=args.output/(name+'_overlay.yaml');overlay_path.write_text(yaml.safe_dump(overlay));command+=['--parameter-overlay',str(overlay_path)]
  # Map directories and run directories have distinct names.
  command[command.index('--output')+1]=str(args.output/(name+'_run'))
  print('START',name,flush=True)
  with (args.output/(name+'.log')).open('w') as log:
   rc=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=240).returncode
  run=args.output/(name+'_run');result=json.loads((run/'result.json').read_text()) if (run/'result.json').exists() else dict(success=False,error='No result artifact')
  results.append(dict(name=name,command=command,returncode=rc,result=result));(args.output/'matrix.json').write_text(json.dumps(results,indent=2));print('DONE',name,result['success'],flush=True)
if __name__=='__main__':main()
