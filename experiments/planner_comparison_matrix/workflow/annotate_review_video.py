#!/usr/bin/env python3
"""Bind the phone review's labels to the current independent mission result."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time

import mission_evidence


RENDERER_SHA='25669099d91bd6dbdf63210e2d7e173bc9393feab8bd51a6addc95c990839c9a'
RENDERER_HELPER_SHA='1049685ce24f3b52cf011fb9d0a69a173f581252586117406edc48e0d3a73f63'
FONT=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')


def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def pin(path):
    path=Path(path).resolve(strict=True)
    return dict(path=str(path),sha256=sha(path))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args();run=args.run_dir.resolve(strict=True)
    _,_,ref,state,_=mission_evidence.latest(json.loads((run/'workflow_state.json').read_text()))
    inputs=json.loads((run/'video_inputs.json').read_text())
    if inputs.get('mission_state_sha256')!=ref['sha256']:raise ValueError('Video inputs do not bind current mission state')
    original_metadata=run/'phone/video_metadata.json';meta=json.loads(original_metadata.read_text())
    video=meta.get('video') or {}
    if (meta.get('source_sha256')!=RENDERER_SHA or meta.get('helper_source_sha256')!=RENDERER_HELPER_SHA
            or video.get('width')!=1280 or video.get('height')!=720):
        raise ValueError('Review overlay requires the pinned renderer and its fixed 1280x720 layout')
    cameras=[topic for topic in meta.get('input_topics',[]) if topic in
        ('/camera/front/image/compressed','/front_camera/image/compressed')]
    if len(cameras)!=1:raise ValueError('A unique actual image topic is required')
    original=original_metadata.parent/Path(video['path']).name
    if sha(original)!=video.get('sha256'):raise ValueError('Original encoded video changed')
    count=state['progress']['accepted_prefix_count'];total=len(state['route'])
    if (type(count) is not int or not 0<=count<=total
            or count!=len(state['progress']['accepted_receipts'])):raise ValueError('Independent visit count is inconsistent')
    complete=state['result_flags']['operational_complete'] is True
    if complete and count!=total:raise ValueError('Completed mission lacks the full original prefix')
    outcome='功能完成' if complete else '已取消' if state['lifecycle']=='CANCELED' else '未通过'
    native=(state.get('attempt') or {}).get('native_status')
    native_label={4:'完成',5:'取消',6:'失败'}.get(native,'未确认')
    strict=state['result_flags'].get('strict_timing_status','NOT_EVALUATED')
    strict_label='未通过' if strict=='FAIL' else '未发现超限' if strict=='NO_STRICT_SCAN_FAULT_RECORDED' else '未确认'
    labels=dict(camera='画面话题 '+cameras[0],final=f'最终独立验收 {count}/{total}：{outcome}',
        native=f'动作层：{native_label}；严格检查：{strict_label}')
    ffmpeg=Path(shutil.which('ffmpeg') or '').resolve(strict=True)
    if not ffmpeg.is_file() or not FONT.is_file():raise ValueError('Pinned environment needs ffmpeg and the CJK font')
    output=run/'phone_review';output.mkdir(exist_ok=False)
    for name,value in labels.items():(output/(name+'.txt')).write_text(value+'\n')
    font_link=output/'review_font.ttf';font_link.symlink_to(FONT)
    target=output/'native_batch_mobile_3fps_review.mp4'
    overlay=('drawbox=x=888:y=72:w=382:h=36:color=0x0b1424:t=fill,'
        'drawtext=fontfile=review_font.ttf:textfile=camera.txt:fontcolor=white:fontsize=17:x=891:y=80,'
        'drawbox=x=888:y=584:w=382:h=86:color=0x0b1424:t=fill,'
        'drawtext=fontfile=review_font.ttf:textfile=final.txt:fontcolor=0xffcf76:fontsize=19:x=891:y=594,'
        'drawtext=fontfile=review_font.ttf:textfile=native.txt:fontcolor=0xffcf76:fontsize=18:x=891:y=625')
    command=[str(ffmpeg),'-nostdin','-hide_banner','-loglevel','warning','-n','-i',str(original),'-vf',overlay,
        '-c:v','libx264','-preset','veryfast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart','-an',str(target)]
    try:
        with (output/'ffmpeg.stdout').open('xb') as stdout,(output/'ffmpeg.stderr').open('xb') as stderr:
            subprocess.run(command,cwd=output,stdout=stdout,stderr=stderr,check=True,timeout=180)
    finally:font_link.unlink(missing_ok=True)
    result=copy.deepcopy(meta)
    result['video'].update(path=str(target),sha256=sha(target),bytes=target.stat().st_size)
    result['source_sha256']=sha(__file__);result['helper_source_sha256']=sha(mission_evidence.__file__)
    result['review']=dict(schema='matrix.phone.independent_review.v1',mission_state_sha256=ref['sha256'],
        mission_id=state['mission_id'],lifecycle=state['lifecycle'],accepted=count,total=total,
        operational_complete=complete,strict_timing_status=strict,image_topic=cameras[0],labels=labels,
        original_metadata=pin(original_metadata),original_video=pin(original),font=pin(FONT),ffmpeg=pin(ffmpeg),
        renderer_source_sha256=RENDERER_SHA,renderer_helper_sha256=RENDERER_HELPER_SHA,
        layout=dict(size=[1280,720],camera_box=[888,72,382,36],status_box=[888,584,382,86]),
        checked_unix_ns=time.time_ns(),boundary='Final independent result labels over historical frames; image topic does not prove physical camera origin. Visual label QA remains separate.')
    with (output/'video_metadata.json').open('x') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2);stream.write('\n')
    print(json.dumps(dict(video=str(target),metadata=str(output/'video_metadata.json'),labels=labels),ensure_ascii=False))


if __name__=='__main__':main()
