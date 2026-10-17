"""Read installed build facts, never the source checkout's current HEAD."""
import json
from pathlib import Path
from robot_debug_msgs.msg import BuildStamp


def stamp_from_dict(value):
    stamp = BuildStamp()
    if not isinstance(value, dict):
        return stamp
    for key in ('repository_id', 'git_commit', 'artifact_id', 'build_id'):
        item = value.get(key)
        if isinstance(item, str):
            setattr(stamp, key, item)
    dirty = value.get('dirty_state')
    if type(dirty) is int and dirty in (0, 1, 2):
        stamp.dirty_state = dirty
    stamp.known = (value.get('known') is True and len(stamp.git_commit) == 40
                   and all(c in '0123456789abcdef' for c in stamp.git_commit.lower()))
    return stamp


def load_build_stamp(package_name):
    from ament_index_python.packages import get_package_share_directory
    try:
        share = Path(get_package_share_directory(package_name))
        candidates = [share / 'debug_build.json']
        if not candidates[0].exists():
            candidates = list(share.glob('debug_build/**/debug_build.json'))
        if len(candidates) == 1 and candidates[0].is_file():
            return stamp_from_dict(json.loads(candidates[0].read_text()))
    except (OSError, ValueError, LookupError):
        pass
    return BuildStamp(artifact_id=package_name)
