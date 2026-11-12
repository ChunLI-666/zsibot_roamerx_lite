#!/usr/bin/env python3
"""Generate immutable target input identity at build time (never at node startup)."""
import argparse
import hashlib
import json
import os
import re
import stat
import subprocess
from pathlib import Path, PurePosixPath


def git(repo, *args):
    return subprocess.run(['git', '-C', str(repo), *args], check=True,
                          capture_output=True).stdout


def proof_git(repo, *args):
    """Read actual repository objects, ignoring caller Git redirection/replacements."""
    env = {key: value for key, value in os.environ.items() if not key.startswith('GIT_')}
    env.update(GIT_NO_REPLACE_OBJECTS='1', GIT_CONFIG_NOSYSTEM='1',
               GIT_CONFIG_GLOBAL=os.devnull)
    return subprocess.run(
        ['git', '-C', str(repo), '-c', 'core.useReplaceRefs=false', *args],
        env=env, check=True, capture_output=True).stdout


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate source manifest key: ' + key)
        result[key] = value
    return result


def absolute_directory(value, label):
    if not isinstance(value, str) or not Path(value).is_absolute():
        raise ValueError(label + ' must be an absolute directory')
    path = Path(value)
    resolved = path.resolve(strict=True)
    if path != resolved or not resolved.is_dir():
        raise ValueError(label + ' must be a real directory without symlink aliases')
    return resolved


def source_path(value):
    if not isinstance(value, str) or not value or '\x00' in value:
        raise ValueError('invalid source manifest path')
    path = PurePosixPath(value)
    if not path.parts or path.is_absolute() or path.as_posix() != value or any(
            part in ('.', '..', '.git') for part in path.parts):
        raise ValueError('unsafe source manifest path: ' + value)
    return value


def inventory_error(error):
    raise error


def snapshot_inventory(root):
    paths = set()
    for directory, dirs, files in os.walk(root, followlinks=False, onerror=inventory_error):
        for name in list(dirs):
            if (Path(directory) / name).is_symlink():
                files.append(name)
                dirs.remove(name)
        for name in files:
            paths.add((Path(directory) / name).relative_to(root).as_posix())
    return paths


def archive_build_stamp(repo, repository_id, artifact, build_context, source_manifest):
    """Verify a complete Git archive against real commit objects before stamping."""
    raw = Path(source_manifest).read_bytes()
    proof = json.loads(raw, object_pairs_hook=unique_object)
    required = {'schema', 'repository_id', 'git_repository', 'git_commit',
                'git_tree', 'source_root', 'files'}
    if not isinstance(proof, dict) or set(proof) != required:
        raise ValueError('source manifest must contain exactly the documented fields')
    if proof['schema'] != 'robot_debug.source_manifest.v1' or proof['repository_id'] != repository_id:
        raise ValueError('source manifest schema/repository_id mismatch')
    for key in ('git_commit', 'git_tree'):
        if not isinstance(proof[key], str) or not re.fullmatch(r'[0-9a-f]{40}', proof[key]):
            raise ValueError('invalid source manifest ' + key)
    root = absolute_directory(proof['source_root'], 'source_root')
    actual_repo = Path(repo).resolve(strict=True)
    try:
        actual_repo.relative_to(root)
    except ValueError as error:
        raise ValueError('--repo must be inside source_root') from error
    if not actual_repo.is_dir():
        raise ValueError('--repo must be a source directory')
    git_repo = absolute_directory(proof['git_repository'], 'git_repository')
    if Path(os.fsdecode(proof_git(git_repo, 'rev-parse', '--show-toplevel')).strip()) != git_repo:
        raise ValueError('git_repository must be the actual repository root')
    commit, tree = proof['git_commit'], proof['git_tree']
    if proof_git(git_repo, 'cat-file', '-t', commit).strip() != b'commit':
        raise ValueError('git_commit is not an actual commit object')
    if proof_git(git_repo, 'rev-parse', commit + '^{tree}').decode().strip() != tree:
        raise ValueError('source manifest commit/tree mismatch')
    tracked = {}
    for entry in proof_git(git_repo, 'ls-tree', '-rz', '--full-tree', commit).split(b'\0'):
        if not entry:
            continue
        metadata, relative = entry.split(b'\t', 1)
        mode, kind, blob = metadata.decode('ascii').split()
        path = source_path(os.fsdecode(relative))
        if kind != 'blob' or mode not in ('100644', '100755', '120000'):
            raise ValueError('unsupported Git entry (including submodules): ' + path)
        tracked[path] = (mode, blob)
    if not isinstance(proof['files'], list):
        raise ValueError('source manifest files must be a list')
    declared = {}
    for row in proof['files']:
        if not isinstance(row, dict) or set(row) != {'path', 'mode', 'git_blob', 'sha256'}:
            raise ValueError('invalid source manifest file record')
        path = source_path(row['path'])
        if path in declared:
            raise ValueError('duplicate source manifest path: ' + path)
        if (row['mode'], row['git_blob']) != tracked.get(path):
            raise ValueError('source manifest mode/blob differs from Git tree: ' + path)
        if not isinstance(row['sha256'], str) or not re.fullmatch(r'[0-9a-f]{64}', row['sha256']):
            raise ValueError('invalid source SHA256: ' + path)
        declared[path] = row
    if declared.keys() != tracked.keys() or snapshot_inventory(root) != tracked.keys():
        raise ValueError('source manifest/archive must contain every tracked path and no extra files')
    for relative, row in declared.items():
        path = root / relative
        info = path.lstat()
        if stat.S_ISLNK(info.st_mode):
            mode, data = '120000', os.fsencode(os.readlink(path))
            try:
                path.resolve().relative_to(root)
            except (ValueError, RuntimeError) as error:
                raise ValueError('source symlink escapes root or cycles: ' + relative) from error
        elif stat.S_ISREG(info.st_mode):
            mode = '100755' if info.st_mode & stat.S_IXUSR else '100644'
            data = path.read_bytes()
        else:
            raise ValueError('source path is not a regular file/symlink: ' + relative)
        blob = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
        if mode != row['mode'] or blob != row['git_blob'] or hashlib.sha256(data).hexdigest() != row['sha256']:
            raise ValueError('actual source mode/blob/SHA256 mismatch: ' + relative)
    identity = dict(schema=proof['schema'], manifest_sha256=hashlib.sha256(raw).hexdigest(),
                    repository_id=repository_id, git_commit=commit, git_tree=tree,
                    artifact_id=artifact, build_context=build_context)
    build_id = hashlib.sha256(json.dumps(identity, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return dict(repository_id=repository_id, git_commit=commit, artifact_id=artifact,
                build_id=build_id, dirty_state=1, known=True), raw.decode('utf-8')


def build_stamp(repo, repository_id, artifact, build_context="", source_manifest=None):
    if source_manifest is not None:
        return archive_build_stamp(repo, repository_id, artifact, build_context, source_manifest)[0]
    try:
        top = Path(git(repo, 'rev-parse', '--show-toplevel').decode().strip())
        commit = git(top, 'rev-parse', 'HEAD').decode().strip()
        # The digest covers changes as well as commit; docs do not imply clean code.
        status = git(top, 'status', '--porcelain=v1', '--untracked-files=normal')
        patch = git(top, 'diff', '--binary', 'HEAD')
        source_digest = hashlib.sha256(commit.encode() + status + patch + artifact.encode() + build_context.encode())
        for relative in git(top, 'ls-files', '--others', '--exclude-standard', '-z').split(b'\0'):
            if not relative:
                continue
            relative_path = Path(relative.decode(errors='surrogateescape'))
            path = top / relative_path
            # Installed script entry points need not have a .py/.sh suffix.
            # Restrict this extra case to scripts directories, not arbitrary outputs.
            script_entry = not path.suffix and 'scripts' in relative_path.parts[:-1]
            if path.is_file() and (path.suffix in ('.py', '.cpp', '.cc', '.h', '.hpp', '.msg', '.cmake', '.xml')
                                   or path.name in ('CMakeLists.txt', 'Makefile') or script_entry):
                source_digest.update(relative)
                source_digest.update(path.read_bytes())
        identity = source_digest.hexdigest()
        return dict(repository_id=repository_id, git_commit=commit, artifact_id=artifact,
                    build_id=identity, dirty_state=2 if status else 1, known=True)
    except (OSError, subprocess.CalledProcessError):
        return dict(repository_id=repository_id, git_commit='', artifact_id=artifact,
                    build_id='', dirty_state=0, known=False)


def write_changed(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or path.read_text() != content:
        path.write_text(content)


def cpp_build_stamp(stamp, symbol_id, header_include='robot_debug_build.hpp'):
    """Keep identity data out of consumer headers and private to its artifact."""
    # Do not include commit, flags or absolute build paths in the namespace:
    # those would change the header and force all consumers to recompile.
    symbol_key = json.dumps([stamp['repository_id'], stamp['artifact_id'], symbol_id])
    namespace = 'build_stamp_' + hashlib.sha256(symbol_key.encode()).hexdigest()
    header = [
        '#pragma once', '#include <robot_debug_msgs/msg/build_stamp.hpp>',
        'namespace robot_debug {', f'namespace {namespace} {{',
        # Even another copy of the same target loaded with RTLD_GLOBAL must not
        # replace this implementation. Namespace isolation alone covers only
        # distinct targets; ELF hidden visibility also covers duplicate ones.
        '#if defined(__GNUC__) || defined(__clang__)',
        '__attribute__((visibility("hidden")))', '#endif',
        'robot_debug_msgs::msg::BuildStamp value();', '}',
        'static inline robot_debug_msgs::msg::BuildStamp compiled_build_stamp() {',
        f'return {namespace}::value();', '}', '}',
    ]
    source = [
        '#include ' + json.dumps(header_include), 'namespace robot_debug {',
        f'namespace {namespace} {{',
        'robot_debug_msgs::msg::BuildStamp value() {',
        'robot_debug_msgs::msg::BuildStamp stamp;',
    ]
    for key, value in stamp.items():
        literal = str(value).lower() if isinstance(value, bool) else (
            str(value) if isinstance(value, int) else json.dumps(value))
        source.append(f'stamp.{key} = {literal};')
    source += ['return stamp;', '}', '}', '}']
    return '\n'.join(header) + '\n', '\n'.join(source) + '\n'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', required=True)
    parser.add_argument('--repository-id', required=True)
    parser.add_argument('--artifact', required=True)
    parser.add_argument('--build-context', default='')
    parser.add_argument('--source-manifest', help='strict complete Git archive provenance; invalid proof fails the build')
    parser.add_argument('--header')
    parser.add_argument('--source', help='identity implementation; required together with --header')
    parser.add_argument('--symbol-id', default='', help='stable project/target key, never a commit or build path')
    parser.add_argument('--json', required=True)
    args = parser.parse_args()
    if bool(args.header) != bool(args.source):
        parser.error('--header and --source must be specified together')
    if args.source_manifest is not None:
        try:
            stamp, proof_text = archive_build_stamp(
                args.repo, args.repository_id, args.artifact, args.build_context, args.source_manifest)
        except (ValueError, OSError, subprocess.CalledProcessError, RuntimeError) as error:
            parser.error('invalid source manifest: ' + str(error))
        write_changed(Path(args.json).with_name('source_manifest.json'), proof_text)
    else:
        stamp = build_stamp(args.repo, args.repository_id, args.artifact, args.build_context)
    write_changed(args.json, json.dumps(stamp, sort_keys=True, indent=2) + '\n')
    if args.header:
        header_include = os.path.relpath(args.header, Path(args.source).parent)
        header, source = cpp_build_stamp(stamp, args.symbol_id, header_include)
        write_changed(args.header, header)
        write_changed(args.source, source)


if __name__ == '__main__':
    main()
