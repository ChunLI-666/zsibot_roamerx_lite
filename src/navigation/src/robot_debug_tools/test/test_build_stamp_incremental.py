"""Real small C++ builds verify rebuild scope and ELF component isolation."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
HELPER = PACKAGE / 'cmake' / 'robot_debug_build_stamp.cmake'


def run(command, **kwargs):
    result = subprocess.run(command, capture_output=True, text=True, **kwargs)
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout


def git(repo, *args):
    return run(['git', '-C', str(repo), '-c', 'core.hooksPath=/dev/null',
                '-c', 'user.name=Incremental stamp test',
                '-c', 'user.email=stamp@example.test', *args]).strip()


def fixture_project(root, generator, unity=False):
    if not shutil.which('cmake') or not shutil.which('c++'):
        pytest.skip('CMake and a C++ compiler are required')
    if not shutil.which('ninja' if generator == 'Ninja' else 'make'):
        pytest.skip(generator + ' is not available')
    repo = root / 'repository'
    repo.mkdir(parents=True)
    # This test needs only the wire fields, not a running ROS installation.
    # Production targets continue to include the generated ROS BuildStamp type.
    header = repo / 'include/robot_debug_msgs/msg/build_stamp.hpp'
    header.parent.mkdir(parents=True)
    header.write_text(
        '#pragma once\n#include <string>\n#include <cstdint>\n'
        'namespace robot_debug_msgs { namespace msg { struct BuildStamp {\n'
        'std::string repository_id, git_commit, artifact_id, build_id;\n'
        'uint8_t dirty_state = 0; bool known = false;\n}; }}\n')
    (repo / 'business.cpp').write_text(
        '#include "robot_debug_build.hpp"\n'
        'extern "C" const char * read_commit() {\n'
        'static auto stamp = robot_debug::compiled_build_stamp();\n'
        'return stamp.git_commit.c_str();\n}\n'
        'extern "C" const char * read_build() {\n'
        'static auto stamp = robot_debug::compiled_build_stamp();\n'
        'return stamp.build_id.c_str();\n}\n')
    (repo / 'CMakeLists.txt').write_text(
        'cmake_minimum_required(VERSION 3.16)\n'
        'project(stamp_fixture LANGUAGES CXX)\n'
        f'include("{HELPER}")\n'
        'add_library(identity SHARED business.cpp)\n'
        'target_include_directories(identity PRIVATE include)\n'
        'robot_debug_build_stamp(identity navigation fixture/identity)\n')
    git(repo, 'init', '-q')
    git(repo, 'add', '.')
    git(repo, 'commit', '--no-gpg-sign', '-qm', 'Fixture baseline')
    build = root / 'build'
    run(['cmake', '-S', str(repo), '-B', str(build), '-G', generator,
         '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_CXX_COMPILER_LAUNCHER=',
         '-DCMAKE_UNITY_BUILD=' + ('ON' if unity else 'OFF')])
    return repo, build


def compile_target(build):
    return run(['cmake', '--build', str(build), '--target', 'identity', '--parallel', '4'])


def stamp_paths(build):
    directory = build / 'debug_build/identity'
    return [directory / name for name in (
        'robot_debug_build.hpp', 'robot_debug_build.cpp', 'debug_build.json')]


def object_times(build):
    return {path.relative_to(build).as_posix(): path.stat().st_mtime_ns
            for path in build.rglob('*.o')}


def read_stamps(*libraries):
    # RTLD_GLOBAL models components in one container. Use a separate process to
    # unload all libraries before the following test rebuilds their paths.
    return json.loads(run([
        sys.executable, '-c',
        'import ctypes,json,os,sys\n'
        'handles=[ctypes.CDLL(p, mode=os.RTLD_NOW | os.RTLD_GLOBAL) for p in sys.argv[1:]]\n'
        'result=[]\n'
        'for lib in handles:\n'
        ' lib.read_commit.restype=ctypes.c_char_p\n'
        ' lib.read_build.restype=ctypes.c_char_p\n'
        ' result.append([lib.read_commit().decode(),lib.read_build().decode()])\n'
        'print(json.dumps(result))\n',
        *(str(path) for path in libraries)]))


@pytest.mark.parametrize('generator,unity', [
    ('Ninja', False), ('Unix Makefiles', False), ('Ninja', True)])
def test_commit_only_rebuilds_stamp_and_noop_rebuilds_nothing(tmp_path, generator, unity):
    repo, build = fixture_project(tmp_path, generator, unity)
    compile_target(build)
    header, source, manifest = stamp_paths(build)
    before = {path: (path.read_bytes(), path.stat().st_mtime_ns)
              for path in (header, source, manifest)}
    objects = object_times(build)
    assert len(objects) == 2
    library = build / 'libidentity.so'
    library_time = library.stat().st_mtime_ns
    initial = json.loads(manifest.read_text())
    assert read_stamps(library) == [[initial['git_commit'], initial['build_id']]]

    compile_target(build)
    assert object_times(build) == objects
    assert library.stat().st_mtime_ns == library_time
    assert {path: (path.read_bytes(), path.stat().st_mtime_ns) for path in before} == before

    git(repo, 'commit', '--no-gpg-sign', '--allow-empty', '-qm', 'New source identity')
    compile_target(build)
    after = object_times(build)
    changed = [name for name in objects if objects[name] != after[name]]
    assert len(changed) == 1 and changed[0].endswith('robot_debug_build.cpp.o')
    assert (header.read_bytes(), header.stat().st_mtime_ns) == before[header]
    assert source.read_bytes() != before[source][0]
    assert library.stat().st_mtime_ns > library_time
    current = json.loads(manifest.read_text())
    assert current['git_commit'] == git(repo, 'rev-parse', 'HEAD')
    assert current['git_commit'] != initial['git_commit']
    assert current['build_id'] != initial['build_id']
    assert read_stamps(library) == [[current['git_commit'], current['build_id']]]

    # Real business changes must still compile, and retain dirty provenance.
    business = repo / 'business.cpp'
    business.write_text(business.read_text() + '\n// Actual source update\n')
    compile_target(build)
    assert all(object_times(build)[name] > after[name] for name in after)
    assert json.loads(manifest.read_text())['dirty_state'] == 2


@pytest.mark.skipif(os.name != 'posix', reason='ELF/RTLD_GLOBAL deployment regression')
def test_duplicate_target_in_global_dsos_keeps_own_build_stamp(tmp_path):
    first_repo, first_build = fixture_project(tmp_path / 'first', 'Ninja')
    second_repo, second_build = fixture_project(tmp_path / 'second', 'Ninja')
    git(second_repo, 'commit', '--no-gpg-sign', '--allow-empty', '-qm', 'Second artifact')
    compile_target(first_build)
    compile_target(second_build)
    # Same repository/artifact/project/target key deliberately exercises hidden
    # visibility; different namespaces alone would not protect this case.
    assert stamp_paths(first_build)[0].read_bytes() == stamp_paths(second_build)[0].read_bytes()
    expected = []
    for repo, build in ((first_repo, first_build), (second_repo, second_build)):
        stamp = json.loads(stamp_paths(build)[2].read_text())
        assert stamp['git_commit'] == git(repo, 'rev-parse', 'HEAD')
        expected.append([stamp['git_commit'], stamp['build_id']])
    assert expected[0] != expected[1]
    libraries = [build / 'libidentity.so' for build in (first_build, second_build)]
    assert read_stamps(*libraries) == expected
    assert read_stamps(*reversed(libraries)) == list(reversed(expected))
