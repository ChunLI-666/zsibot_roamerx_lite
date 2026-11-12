#!/usr/bin/env python3
"""Summarize actual mission, recording and video evidence without inventing PASS."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import time

import mission_evidence


def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',type=Path,required=True);p.add_argument('--video-dir',type=Path);a=p.parse_args();run=a.run_dir
    video_dir=(a.video_dir or run/'phone_review').resolve()
    if not video_dir.is_relative_to(run.resolve()):raise ValueError('video output must belong to the run')
    plan=json.loads((run/'execution_plan.json').read_text());workflow=json.loads((run/'workflow_state.json').read_text())
    row,response,ref,state,state_source=mission_evidence.latest(workflow);audit=json.loads((run/'mcap_audit.json').read_text())
    qa=json.loads((video_dir/'encoded_QA/QA.json').read_text());video=json.loads((video_dir/'video_metadata.json').read_text())
    review=video.get('review') or {}
    if (review.get('schema')!='matrix.phone.independent_review.v1' or review.get('mission_state_sha256')!=ref['sha256']
            or review.get('mission_id')!=state['mission_id']):raise ValueError('Review video does not bind the current independent mission')
    video_path=Path(video['video']['path']).resolve()
    if not video_path.is_relative_to(video_dir):raise ValueError('Review video must belong to its QA directory')
    video_sha=sha(video_path)
    if (video_sha!=video['video']['sha256'] or qa.get('video_sha256_reused_from_renderer')!=video_sha
            or Path(qa['video']).resolve()!=video_path.resolve()):raise ValueError('Encoded QA is not for this exact review video')
    metrics=json.loads((run/'metrics.json').read_text())
    from workflow_inputs import checked_json
    source=checked_json(plan['input_binding']['source'])
    initialization=source.get('initialization',{}).get('description','初始化方式未在 source manifest 中单独记录；定位放行证据见本代 localization_release。')
    policy,budget=state['policy'],state['budgets'];receipts=state['progress']['accepted_receipts'];attempts=state['attempts']
    coverage=(state['result_flags']['operational_complete'] is True and state['progress']['accepted_prefix_count']==len(state['route'])
              and len(receipts)==len(state['route']) and [x['index'] for x in receipts]==list(range(len(state['route']))))
    bounded=(budget['mission_dispatches_reserved']<=policy['max_parent_dispatches'] and
             all(x<=3 for x in budget['per_index_parent_dispatches']) and
             all(x<=policy['max_attempts_per_index'] for x in budget['per_index_reserved']) and
             len({x['parent_action_uuid'] for x in attempts})==len(attempts) and
             all(x.get('terminal_confirmed') and x.get('children_quiescent') and x.get('stopped') for x in attempts))
    perf={};sample_path=run/'performance/samples.jsonl'
    if sample_path.exists():
        for line in sample_path.read_text().splitlines():
            for role,value in json.loads(line)['processes'].items():
                if value is None:continue
                s=perf.setdefault(role,dict(cpu=[],rss=[]))
                if value.get('cpu_cores') is not None:s['cpu'].append(value['cpu_cores'])
                s['rss'].append(value['rss_bytes'])
        perf={role:dict(samples=len(v['rss']),mean_cpu_cores=statistics.mean(v['cpu']) if v['cpu'] else None,
                        peak_cpu_cores=max(v['cpu'],default=None),peak_rss_bytes=max(v['rss'],default=None)) for role,v in perf.items()}
    strict=state['result_flags'].get('strict_timing_status','NOT_EVALUATED')
    result=dict(schema='terra.simulation.acceptance.v1',variant=plan['variant'],commit=plan['commit'],
        mission_id=state['mission_id'],lifecycle=state['lifecycle'],accepted_waypoints=state['progress']['accepted_prefix_count'],
        total_waypoints=len(state['route']),mission_coverage_pass=coverage,bounded_recovery_accounting_pass=bounded,
        recovery_used=state['result_flags']['recovery_used'],strict_scan_300ms_status=strict,
        scan_operational_limit_sec=.5,mission_outcome=response['outcome'],mission_state_source=state_source,
        mcap_integrity_pass=audit['integrity_pass'],mcap_required_topic_coverage_complete=audit['coverage_complete'],
        phone_encoded_checks_pass=qa['encoded_checks_pass'],phone_visual_review='PENDING',
        independent_collision_and_handoff_audit='NOT_EVALUATED',simulation_acceptance=False,physical_acceptance=False,
        budgets=budget,attempts=attempts,performance=perf,original_turns=metrics['original_turns'],
        sampled_static_footprint=metrics['sampled_static_map_footprint'],handoff_event_issues=metrics['handoff_event_issues'],
        video=video.get('video'),checked_ns=time.time_ns(),
        source_id=source['source_id'],source_manifest=plan['input_binding']['source'],initialization=initialization,
        limitation='Mission coverage, strict timing, MCAP, encoded video, collision/handoff and physical acceptance remain separate.')
    _,_,current_ref,_,_=mission_evidence.latest(json.loads((run/'workflow_state.json').read_text()))
    if current_ref['sha256']!=review['mission_state_sha256']:raise ValueError('Mission projection changed during video finalization')
    (run/'acceptance.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    text=(f"分支：{plan['variant']}，提交：{plan['commit']}\n"
          f"任务：{state['lifecycle']}；已接受 {state['progress']['accepted_prefix_count']}/{len(state['route'])} 个原始点；覆盖通过：{coverage}。\n"
          f"恢复预算核对：{bounded}；父任务次数 {budget['mission_dispatches_reserved']}/{policy['max_parent_dispatches']}；恢复使用：{state['result_flags']['recovery_used']}。\n"
          f"严格 0.3 秒扫描对照：{strict}；本次仿真运行使用 0.5 秒模式。\n"
          f"新录包扫描年龄超过 0.3 秒的样本数：{metrics['scan_header_age_vs_bag_log']['over_300ms']}；静态地图足迹采样：{metrics['sampled_static_map_footprint']['samples']}。\n"
          f"F009/F010/F011 实际旋转方向：{[(x['id'],x['direction']) for x in metrics['original_turns']]}。\n"
          f"MCAP 完整性：{audit['integrity_pass']}；必需话题覆盖：{audit['coverage_complete']}；缺失或空话题：{audit['missing_or_empty_required_topics']}。\n"
          f"手机视频编码与完整解码检查：{qa['encoded_checks_pass']}；首中尾画面人工检查待完成。\n"
          f"独立碰撞/命令交接复核尚未完成，故仿真总验收尚未判定通过；不代表实机验收。\n"
          f"初始化记录：{initialization}\n")
    (run/'report.txt').write_text(text)
    selected=['execution_plan.json','nav/runtime_manifest.json','nav/runtime_parameters.json',
              'nav/mission_identity.json','nav/final_audit.json','nav/static_layer_recording_audit.json','mcap_audit.json',
              'navigation_closed.json','performance/identity.json','performance/summary.json','planner_timing/summary.json',
              'metrics.json','video_inputs.json',
              'acceptance.json','report.txt']
    selected += [str(p.relative_to(run.resolve())) for p in (video_dir/'video_metadata.json',video_dir/'encoded_QA/QA.json',video_dir/'encoded_QA/ffprobe.json')]
    selected += [str(p.relative_to(run.resolve())) for p in sorted((video_dir/'encoded_QA').glob('encoded_keyframe_*.png'))]
    files={name:dict(sha256=sha(run/name),bytes=(run/name).stat().st_size) for name in selected if (run/name).is_file()}
    if video_path.is_file():files[str(video_path.relative_to(run.resolve()))]=dict(sha256=video_sha,bytes=video_path.stat().st_size)
    manifest=dict(schema='terra.run.hashmanifest.v1',files=files,closed_mcap_sha256=audit['mcap_sha256'],
                  mission_state=ref,source_attempt_native=[x.get('native_result') for x in attempts if x.get('native_result')])
    (run/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(dict(coverage=coverage,bounded=bounded,simulation_acceptance=False)))


if __name__=='__main__':main()
