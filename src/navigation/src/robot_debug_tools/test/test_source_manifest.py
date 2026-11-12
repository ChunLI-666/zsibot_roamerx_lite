"""Archive identity uses real commit objects and complete, byte-verified sources."""
import hashlib
import json
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import sys

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
GENERATOR = PACKAGE / 'scripts' / 'generate_build_stamp.py'
build_stamp = runpy.run_path(str(GENERATOR))['build_stamp']


def git(repo, *args, input=None):
    return subprocess.run(
        ['git', '-C', str(repo), '-c', 'core.hooksPath=/dev/null',
         '-c', 'user.name=Archive identity test', '-c', 'user.email=archive@example.test',
         *args], input=input, check=True, capture_output=True).stdout


def export_tree(repository, root, commit):
    if root.exists():
        shutil.rmtree(root)
    root.mkdir()
    files = []
    for entry in git(repository, 'ls-tree', '-rz', commit).split(b'\0'):
        if not entry:
            continue
        metadata, relative = entry.split(b'\t', 1)
        mode, _, blob = metadata.decode().split()
        relative = os.fsdecode(relative)
        data = git(repository, 'cat-file', 'blob', blob)
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        if mode == '120000':
            path.symlink_to(os.fsdecode(data))
        else:
            path.write_bytes(data)
            path.chmod(0o755 if mode == '100755' else 0o644)
        files.append(dict(path=relative, mode=mode, git_blob=blob,
                          sha256=hashlib.sha256(data).hexdigest()))
    return files


@pytest.fixture
def archive(tmp_path):
    repository = tmp_path / 'repository'
    repository.mkdir()
    git(repository, 'init', '-q')
    (repository / 'src').mkdir()
    (repository / 'src' / 'main.cc').write_bytes(b'int value = 1;\n')
    tool = repository / 'tool'
    tool.write_bytes(b'#!/bin/sh\nexit 0\n')
    tool.chmod(0o755)
    (repository / 'link').symlink_to('src/main.cc')
    git(repository, 'add', '.')
    git(repository, 'commit', '--no-gpg-sign', '-qm', 'Baseline')
    parent = git(repository, 'rev-parse', 'HEAD').decode().strip()
    (repository / 'src' / 'main.cc').write_bytes(b'int value = 2;\n')
    git(repository, 'add', '.')
    tree = git(repository, 'write-tree').decode().strip()
    commit = git(repository, 'commit-tree', tree, '-p', parent,
                 input=b'Real unmerged review commit\n').decode().strip()
    git(repository, 'update-ref', 'refs/heads/review/archive-identity', commit)
    root = tmp_path / 'snapshot'
    files = export_tree(repository, root, commit)
    proof = dict(schema='robot_debug.source_manifest.v1', repository_id='lightning',
                 git_repository=str(repository), git_commit=commit, git_tree=tree,
                 source_root=str(root), files=files)
    manifest = tmp_path / 'source_manifest.json'
    manifest.write_text(json.dumps(proof, sort_keys=True))
    return dict(repository=repository, root=root, proof=proof, manifest=manifest,
                parent=parent, commit=commit, tree=tree, tmp=tmp_path)


def identity(archive, **kwargs):
    return build_stamp(archive['root'] / 'src', 'lightning',
                       kwargs.get('artifact', 'lightning.libs'),
                       kwargs.get('context', 'compiler|flags'), archive['manifest'])


def write_proof(archive):
    archive['manifest'].write_text(json.dumps(archive['proof'], sort_keys=True))


def test_real_review_commit_snapshot_is_known_and_does_not_use_dirty_head(archive):
    stamp = identity(archive)
    assert archive['parent'] != archive['commit']
    assert git(archive['repository'], 'rev-parse', 'HEAD').decode().strip() == archive['parent']
    assert set(stamp) == {'repository_id', 'git_commit', 'artifact_id', 'build_id', 'dirty_state', 'known'}
    assert stamp['git_commit'] == archive['commit']
    assert stamp['dirty_state'] == 1 and stamp['known']
    assert len(stamp['build_id']) == 64
    assert stamp == identity(archive)
    assert not build_stamp(archive['root'], 'lightning', 'lightning.libs')['known']


@pytest.mark.parametrize('change', ['context', 'artifact', 'proof_bytes'])
def test_actual_context_artifact_and_proof_are_bound(archive, change):
    before = identity(archive)
    if change == 'proof_bytes':
        archive['manifest'].write_text(archive['manifest'].read_text() + '\n')
        after = identity(archive)
    else:
        after = identity(archive, **{change: 'different input'})
    assert before['build_id'] != after['build_id']


@pytest.mark.parametrize('change', [
    'fake_commit', 'tree_as_commit', 'wrong_tree', 'wrong_repository_id', 'wrong_schema',
    'missing_record', 'duplicate_record', 'fake_blob', 'fake_sha256', 'mode_record',
    'missing_file', 'extra_file', 'extra_hidden_file', 'source_bytes', 'source_mode',
    'unsafe_parent', 'absolute_path', 'normalized_path', 'alias_root', 'outside_repo',
    'directory_symlink', 'file_symlink', 'symlink_bytes', 'fifo', 'unknown_field',
])
def test_incomplete_or_forged_proof_fails_closed(archive, change):
    proof, root = archive['proof'], archive['root']
    row = next(row for row in proof['files'] if row['path'] == 'src/main.cc')
    if change == 'fake_commit':
        proof['git_commit'] = '0' * 40
    elif change == 'tree_as_commit':
        proof['git_commit'] = archive['tree']
    elif change == 'wrong_tree':
        proof['git_tree'] = git(archive['repository'], 'rev-parse', archive['parent'] + '^{tree}').decode().strip()
    elif change == 'wrong_repository_id':
        proof['repository_id'] = 'navigation'
    elif change == 'wrong_schema':
        proof['schema'] = 'future-or-unknown'
    elif change == 'missing_record':
        proof['files'].remove(row)
    elif change == 'duplicate_record':
        proof['files'].append(dict(row))
    elif change == 'fake_blob':
        row['git_blob'] = '0' * 40
    elif change == 'fake_sha256':
        row['sha256'] = '0' * 64
    elif change == 'mode_record':
        row['mode'] = '100755'
    elif change == 'missing_file':
        (root / 'src/main.cc').unlink()
    elif change in ('extra_file', 'extra_hidden_file'):
        (root / ('.hidden' if change == 'extra_hidden_file' else 'extra.cc')).write_bytes(b'extra')
    elif change == 'source_bytes':
        (root / 'src/main.cc').write_bytes(b'changed')
    elif change == 'source_mode':
        (root / 'src/main.cc').chmod(0o755)
    elif change in ('unsafe_parent', 'absolute_path', 'normalized_path'):
        row['path'] = {'unsafe_parent': '../escape', 'absolute_path': '/escape',
                       'normalized_path': 'src//main.cc'}[change]
    elif change == 'alias_root':
        alias = archive['tmp'] / 'alias'
        alias.symlink_to(root, target_is_directory=True)
        proof['source_root'] = str(alias)
    elif change == 'outside_repo':
        proof['source_root'] = str(root / 'src')
    elif change == 'directory_symlink':
        shutil.rmtree(root / 'src')
        (root / 'src').symlink_to(archive['repository'] / 'src', target_is_directory=True)
    elif change == 'file_symlink':
        (root / 'src/main.cc').unlink()
        (root / 'src/main.cc').symlink_to(archive['repository'] / 'src/main.cc')
    elif change == 'symlink_bytes':
        (root / 'link').unlink()
        (root / 'link').symlink_to('tool')
    elif change == 'fifo':
        (root / 'src/main.cc').unlink()
        os.mkfifo(root / 'src/main.cc')
    elif change == 'unknown_field':
        proof['claimed_commit'] = archive['parent']
    write_proof(archive)
    with pytest.raises((ValueError, OSError, subprocess.CalledProcessError)):
        identity(archive)


@pytest.mark.parametrize('target', ['../outside', '/tmp/archive-proof-escape', 'link'])
def test_even_git_committed_symlinks_cannot_escape_or_cycle(archive, target):
    repository = archive['repository']
    (repository / 'link').unlink()
    (repository / 'link').symlink_to(target)
    git(repository, 'add', '.')
    tree = git(repository, 'write-tree').decode().strip()
    commit = git(repository, 'commit-tree', tree, '-p', archive['commit'],
                 input=b'Unsafe symlink review\n').decode().strip()
    proof = archive['proof']
    proof.update(git_commit=commit, git_tree=tree)
    row = next(row for row in proof['files'] if row['path'] == 'link')
    data = os.fsencode(target)
    row.update(git_blob=hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest(),
               sha256=hashlib.sha256(data).hexdigest())
    (archive['root'] / 'link').unlink()
    (archive['root'] / 'link').symlink_to(target)
    write_proof(archive)
    with pytest.raises(ValueError, match='symlink'):
        identity(archive)


def test_git_environment_cannot_substitute_a_different_object_store(archive, monkeypatch):
    monkeypatch.setenv('GIT_DIR', str(archive['tmp'] / 'missing-git'))
    monkeypatch.setenv('GIT_WORK_TREE', str(archive['tmp']))
    monkeypatch.setenv('GIT_CONFIG_COUNT', '1')
    monkeypatch.setenv('GIT_CONFIG_KEY_0', 'core.bare')
    monkeypatch.setenv('GIT_CONFIG_VALUE_0', 'true')
    assert identity(archive)['git_commit'] == archive['commit']


@pytest.mark.parametrize('error_type', [PermissionError, OSError])
def test_unreadable_extra_directory_fails_closed(archive, monkeypatch, error_type):
    extra = archive['root'] / 'untracked_directory'
    extra.mkdir()
    (extra / 'unknown.cc').write_bytes(b'untracked source must not disappear from inventory')
    actual_scandir = os.scandir
    error = error_type('injected source inventory failure')

    def failing_scandir(path):
        if os.fsdecode(path) == str(extra):
            raise error
        return actual_scandir(path)

    monkeypatch.setattr(os, 'scandir', failing_scandir)
    with pytest.raises(error_type, match='injected source inventory failure') as caught:
        identity(archive)
    assert caught.value is error


def test_cli_failure_writes_no_identity_and_success_installs_exact_sidecar(archive):
    output = archive['tmp'] / 'output'
    command = [sys.executable, str(GENERATOR), '--repo', str(archive['root'] / 'src'),
               '--repository-id', 'lightning', '--artifact', 'lightning.libs',
               '--source-manifest', str(archive['manifest']), '--json', str(output / 'debug_build.json'),
               '--header', str(output / 'robot_debug_build.hpp'),
               '--source', str(output / 'robot_debug_build.cpp')]
    archive['proof']['git_commit'] = '0' * 40
    write_proof(archive)
    failed = subprocess.run(command, capture_output=True, text=True)
    assert failed.returncode != 0 and 'invalid source manifest' in failed.stderr
    assert not output.exists()
    archive['proof']['git_commit'] = archive['commit']
    write_proof(archive)
    subprocess.run(command, check=True, capture_output=True)
    stamp = json.loads((output / 'debug_build.json').read_text())
    assert stamp['git_commit'] == archive['commit'] and stamp['known']
    assert (output / 'source_manifest.json').read_bytes() == archive['manifest'].read_bytes()
    header = (output / 'robot_debug_build.hpp').read_text()
    source = (output / 'robot_debug_build.cpp').read_text()
    assert not any(line.startswith('stamp.') for line in header.splitlines())
    assert len([line for line in source.splitlines() if line.startswith('stamp.')]) == 6
    assert 'stamp.git_tree' not in source


def test_duplicate_json_keys_are_rejected(archive):
    text = archive['manifest'].read_text()
    archive['manifest'].write_text('{"schema":"ambiguous",' + text[1:])
    with pytest.raises(ValueError, match='duplicate'):
        identity(archive)


@pytest.mark.parametrize('selected', [True, False])
def test_cmake_hook_is_repository_specific_and_runs_real_generator(archive, selected):
    cmake = shutil.which('cmake')
    if not cmake:
        pytest.skip('CMake is not available')
    repository = archive['repository']
    (repository / 'dummy.cc').write_text('int dummy;\n')
    cmakelists = repository / 'CMakeLists.txt'
    cmakelists.write_text(
        'cmake_minimum_required(VERSION 3.16)\nproject(proof_fixture LANGUAGES CXX)\n'
        f'include("{PACKAGE / "cmake/robot_debug_build_stamp.cmake"}")\n'
        'add_library(identity STATIC EXCLUDE_FROM_ALL dummy.cc)\n'
        'robot_debug_build_stamp(identity lightning lightning.libs)\n')
    git(repository, 'add', '.')
    tree = git(repository, 'write-tree').decode().strip()
    commit = git(repository, 'commit-tree', tree, '-p', archive['commit'],
                 input=b'Real complete CMake fixture commit\n').decode().strip()
    archive['proof'].update(git_commit=commit, git_tree=tree,
                            files=export_tree(repository, archive['root'], commit))
    write_proof(archive)
    key = 'LIGHTNING' if selected else 'NAVIGATION'
    build = archive['tmp'] / 'cmake_build'
    subprocess.run([cmake, '-S', str(archive['root']), '-B', str(build),
                    f'-DROBOT_DEBUG_SOURCE_MANIFEST_{key}={archive["manifest"]}'],
                   check=True, capture_output=True)
    subprocess.run([cmake, '--build', str(build), '--target', 'identity_debug_stamp', '-j1'],
                   check=True, capture_output=True)
    directory = build / 'debug_build/identity'
    stamp = json.loads((directory / 'debug_build.json').read_text())
    assert stamp['known'] == selected
    assert (directory / 'source_manifest.json').exists() == selected
    if selected:
        assert stamp['git_commit'] == commit
