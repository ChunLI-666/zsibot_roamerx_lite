#!/usr/bin/env python3
"""Generate immutable target input identity at build time (never at node startup)."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path


def git(repo, *args):
    return subprocess.run(['git', '-C', str(repo), *args], check=True,
                          capture_output=True).stdout


def build_stamp(repo, repository_id, artifact, build_context=""):
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', required=True)
    parser.add_argument('--repository-id', required=True)
    parser.add_argument('--artifact', required=True)
    parser.add_argument('--build-context', default='')
    parser.add_argument('--header')
    parser.add_argument('--json', required=True)
    args = parser.parse_args()
    stamp = build_stamp(args.repo, args.repository_id, args.artifact, args.build_context)
    write_changed(args.json, json.dumps(stamp, sort_keys=True, indent=2) + '\n')
    if args.header:
        lines = ['#pragma once', '#include <robot_debug_msgs/msg/build_stamp.hpp>',
                 'namespace robot_debug {',
                 'static inline robot_debug_msgs::msg::BuildStamp compiled_build_stamp() {',
                 'robot_debug_msgs::msg::BuildStamp stamp;']
        for key, value in stamp.items():
            literal = str(value).lower() if isinstance(value, bool) else (
                str(value) if isinstance(value, int) else json.dumps(value))
            lines.append(f'stamp.{key} = {literal};')
        lines += ['return stamp;', '}', '}']
        write_changed(args.header, '\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
