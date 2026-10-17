"""Build identities must distinguish dirty source bytes, independent of Git status."""
from pathlib import Path
import runpy
import subprocess

import pytest


build_stamp = runpy.run_path(str(
    Path(__file__).resolve().parents[1] / 'scripts' / 'generate_build_stamp.py'
))['build_stamp']


def git(repo, *args):
    return subprocess.run(
        ['git', '-C', str(repo), '-c', 'core.hooksPath=/dev/null',
         '-c', 'user.name=Build stamp test', '-c', 'user.email=build-stamp@example.test',
         *args], check=True, capture_output=True).stdout


@pytest.fixture
def repository(tmp_path):
    git(tmp_path, 'init', '-q')
    (tmp_path / 'tracked.py').write_text('value = 0\n')
    git(tmp_path, 'add', 'tracked.py')
    git(tmp_path, 'commit', '--no-gpg-sign', '-qm', 'Test baseline')
    return tmp_path


def identity(repo):
    return build_stamp(repo, 'navigation', 'robot_debug_tools/debug_observer')


def git_state(repo):
    return (git(repo, 'rev-parse', 'HEAD'),
            git(repo, 'status', '--porcelain=v1', '--untracked-files=normal'))


@pytest.mark.parametrize('relative', [
    'scripts/debug_observer', 'src/component/scripts/debug_observer'])
def test_untracked_entry_script_bytes_change_identity_with_same_git_state(repository, relative):
    script = repository / relative
    script.parent.mkdir(parents=True)
    script.write_text('#!/usr/bin/env python3\nprint("first")\n')
    script.chmod(0o755)
    before_state = git_state(repository)
    before = identity(repository)
    script.write_text('#!/usr/bin/env python3\nprint("second")\n')

    assert git_state(repository) == before_state
    after = identity(repository)
    assert before['known'] and after['known']
    assert before['dirty_state'] == after['dirty_state'] == 2
    assert before['build_id'] != after['build_id']


def test_untracked_python_bytes_change_identity_with_same_git_state(repository):
    source = repository / 'new_source.py'
    source.write_text('value = 1\n')
    before_state = git_state(repository)
    before = identity(repository)
    source.write_text('value = 2\n')

    assert git_state(repository) == before_state
    assert before['build_id'] != identity(repository)['build_id']


def test_tracked_patch_bytes_change_identity_with_same_git_state(repository):
    source = repository / 'tracked.py'
    source.write_text('value = 1\n')
    before_state = git_state(repository)
    before = identity(repository)
    source.write_text('value = 2\n')

    assert git_state(repository) == before_state
    assert before['build_id'] != identity(repository)['build_id']


def test_clean_checkout_remains_known_and_stable(repository):
    before = identity(repository)

    assert before['known']
    assert before['dirty_state'] == 1
    assert before['git_commit'] == git(repository, 'rev-parse', 'HEAD').decode().strip()
    assert len(before['build_id']) == 64
    assert identity(repository) == before


def test_non_git_directory_remains_unknown(tmp_path):
    assert identity(tmp_path) == {
        'repository_id': 'navigation', 'git_commit': '',
        'artifact_id': 'robot_debug_tools/debug_observer',
        'build_id': '', 'dirty_state': 0, 'known': False}


def test_unrelated_map_and_build_output_bytes_do_not_change_identity(repository):
    outputs = [repository / 'maps' / 'campus.pcd', repository / 'build' / 'output']
    for output in outputs:
        output.parent.mkdir()
        output.write_bytes(b'unrelated output before\0')
    before_state = git_state(repository)
    before = identity(repository)
    for output in outputs:
        output.write_bytes(b'unrelated output after\0')

    assert git_state(repository) == before_state
    assert identity(repository) == before
