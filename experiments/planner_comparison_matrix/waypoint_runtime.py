"""Explicit source/build/runtime binding for the official 1.3.13 callback guard."""
import hashlib
import json
from pathlib import Path

UPSTREAM_COMMIT = 'f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501'
PACKAGE = 'src/navigation/src/matrix_waypoint_follower'
EXECUTABLE = 'lib/matrix_waypoint_follower/waypoint_follower'
CORE = 'lib/libmatrix_waypoint_follower_core.so'
SOURCE_FILES = [PACKAGE+'/'+name for name in (
    'src/waypoint_follower.cpp', 'src/main.cpp',
    'include/nav2_waypoint_follower/waypoint_follower.hpp',
    'CMakeLists.txt', 'package.xml', 'UPSTREAM.json', 'callback_nonblocking.patch')]
UNDERLAY_FILES = ['/opt/ros/jazzy/'+name for name in (
    'lib/libwait_at_waypoint.so', 'share/nav2_waypoint_follower/package.xml',
    'include/rclcpp_action/rclcpp_action/client.hpp',
    'include/rclcpp_action/rclcpp_action/client_goal_handle_impl.hpp')]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checked_files(root, files, required):
    if not isinstance(files, dict) or not set(required) <= files.keys():
        raise ValueError('Waypoint receipt lacks required files')
    for name, digest in files.items():
        relative = Path(name)
        path = (root/name).resolve()
        if (relative.is_absolute() or '..' in relative.parts or relative.as_posix() != name
                or not path.is_relative_to(root) or not path.is_file() or sha(path) != digest):
            raise ValueError('Waypoint file differs or escapes root: '+name)


def artifacts(install, receipt_path, source_root):
    if not install or not receipt_path:
        raise ValueError('--with-waypoints requires --waypoint-install and --waypoint-build-receipt')
    prefix = Path(install).resolve()
    receipt = Path(receipt_path).resolve()
    data = json.loads(receipt.read_text())
    commit = data.get('repository_commit', '')
    if (data.get('schema') != 'matrix.waypoint.build_receipt.v1'
            or data.get('install_prefix') != str(prefix)
            or data.get('source_files_match_commit') is not True
            or data.get('upstream_commit') != UPSTREAM_COMMIT
            or len(commit) != 40 or any(c not in '0123456789abcdef' for c in commit)):
        raise ValueError('Waypoint overlay requires pinned, commit-bound build receipt')
    checked_files(prefix, data.get('artifact_sha256'), (EXECUTABLE, CORE))
    checked_files(Path(source_root).resolve(), data.get('source_sha256'), SOURCE_FILES)
    underlay = data.get('underlay_sha256', {})
    if set(underlay) != set(UNDERLAY_FILES):
        raise ValueError('Waypoint receipt lacks reviewed underlay contract')
    for name, digest in underlay.items():
        if sha(name) != digest:
            raise ValueError('Waypoint plugin/client contract changed: '+name)
    return dict(install_prefix=str(prefix), receipt_path=str(receipt), receipt_sha256=sha(receipt),
                repository_commit=commit, upstream_commit=UPSTREAM_COMMIT,
                source_sha256=data['source_sha256'], artifact_sha256=data['artifact_sha256'],
                underlay_sha256=underlay)


def verify_runtime(binding, identity, executable):
    prefix = Path(binding['install_prefix'])
    expected = binding['artifact_sha256']
    libraries = identity['selected_libraries_sha256']
    if sha(executable) != expected[EXECUTABLE]:
        raise RuntimeError('Waypoint executable differs from selected build')
    if libraries.get(str(prefix/CORE)) != expected[CORE]:
        raise RuntimeError('Guarded waypoint core not actually loaded')
    if (any(Path(name).name == 'libwaypoint_follower_core.so' for name in libraries)
            or '/libwaypoint_follower_core.so' in identity.get('maps', '')):
        raise RuntimeError('Unreviewed waypoint core also loaded')
