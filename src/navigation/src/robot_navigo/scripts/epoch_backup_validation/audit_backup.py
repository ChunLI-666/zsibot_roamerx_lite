#!/usr/bin/env python3
"""Post-process an isolated action run with the independent wire-time observer."""
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'epoch_navigation_validation'))
from audit_events import audit,audit_callback


def main(directory):
    path=Path(directory)/'result.json'
    result=json.loads(path.read_text())
    rows=[json.loads(line) for line in path.with_name('wire_events.jsonl').read_text().splitlines()]
    verdict=audit(rows,require_motion=result.get('actual_distance_m',0.)>1e-9)
    path.with_name('wire_audit.json').write_text(json.dumps(verdict,indent=2))
    result['wire_audit_failures']=len(verdict['failures'])
    result['wire_time_basis']=verdict.get('publication_clock',{}).get('basis','legacy_callback')
    callback=verdict.get('callback_audit') or audit_callback(rows,require_motion=result.get('actual_distance_m',0.)>1e-9)
    result['callback_audit_failures']=len(callback['failures'])
    result['wire_publication_scorable']=bool(verdict.get('publication_clock',{}).get('available'))
    # Do not turn an action/geometry failure into success just because wire audit passed.
    if verdict['failures']:result['success']=False
    path.write_text(json.dumps(result,indent=2))
    print(json.dumps({key:result.get(key) for key in ('scenario','success','reason','actual_distance_m','wire_audit_failures','wire_time_basis','callback_audit_failures')}))
    return 0 if result['success'] else 1

if __name__=='__main__':sys.exit(main(sys.argv[1]))
