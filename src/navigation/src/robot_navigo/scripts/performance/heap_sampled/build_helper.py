#!/usr/bin/env python3
"""Optional standalone build; explicit matching headers/library, no install/download."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parent


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--include-dir', type=Path, required=True, help='Absolute include root containing matching gperftools headers')
    parser.add_argument('--tcmalloc-lib', type=Path, required=True, help='Absolute plain tcmalloc library from the same package/version')
    parser.add_argument('--output', type=Path, required=True, help='New directory, never overwritten')
    parser.add_argument('--cxx', default=os.environ.get('CXX','c++'), help='Compiler executable; no shell options')
    parser.add_argument('--with-fixture', action='store_true', help='Also compile the small non-ROS native validation program')
    parser.add_argument('--headers-origin', type=Path, help='Optional JSON package source/version/SHA provenance')
    args = parser.parse_args()
    if not args.include_dir.is_absolute() or not args.tcmalloc_lib.is_absolute():
        parser.error('Headers and plain allocator must be absolute external paths')
    include, library, output = args.include_dir.resolve(), args.tcmalloc_lib.resolve(), args.output.resolve()
    if output.exists():
        parser.error('Preserve previous build artifacts; choose a new output directory')
    if not all((include / 'gperftools' / name).is_file() for name in ['malloc_extension.h','heap-profiler.h']):
        parser.error('Real matching gperftools development headers are required')
    compiler = shutil.which(args.cxx)
    if not compiler:
        parser.error('Explicit compiler not found; dependencies are never installed automatically')
    spec = importlib.util.spec_from_file_location('locnav_runner_build_api', ROOT.parent / 'run.py')
    runtime = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runtime)
    info = runtime.elf_info(library)
    if info['type'] != 3 or not (info['soname'] or '').startswith('libtcmalloc.so.'):
        parser.error('A plain tcmalloc shared library is required, not combined/minimal/debug variants')
    output.mkdir(parents=True)
    common = [compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pthread']
    commands = [[*common, '-O2', '-g', '-fPIC', '-shared', '-fno-omit-frame-pointer',
        '-I'+str(include), str(ROOT/'heap_sample_helper.cc'), '-o', str(output/'liblocnav_heap_sample.so'),
        '-Wl,-rpath,'+str(library.parent), '-Wl,-z,defs', '-Wl,--no-as-needed', str(library), '-ldl']]
    sources = [ROOT/'build_helper.py', ROOT/'heap_sample_helper.cc']
    if args.with_fixture:
        sources.append(ROOT/'fixture.cc')
        commands.append([*common, '-O0', '-g', '-fno-omit-frame-pointer', '-rdynamic', str(ROOT/'fixture.cc'),
            '-o', str(output/'fixture'), '-ldl'])
    report = dict(schema='locnav.heap_sampled.build.v1', success=False, started_unix=time.time(),
        compiler=compiler, compiler_version=subprocess.check_output([compiler,'--version'],text=True),
        commands=commands, source_sha256={str(path):sha(path) for path in sources},
        headers_sha256={str(path):sha(path) for path in sorted((include/'gperftools').glob('*.h'))},
        headers_origin=json.loads(args.headers_origin.read_text()) if args.headers_origin else None,
        library=str(library), library_sha256=sha(library), library_elf=info,
        downloaded_dependencies=False, system_install=False)
    try:
        with (output/'build.log').open('x') as log:
            for command in commands:
                subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
        helper = runtime.elf_info(output/'liblocnav_heap_sample.so')
        abi = lambda row:(row['bits'],row['byte_order'],row['machine'],row['arm_float_abi'])
        if abi(helper) != abi(info) or [name for name in helper['needed'] if 'tcmalloc' in name] != [info['soname']]:
            raise RuntimeError('Compiler output does not match selected plain allocator ABI/SONAME')
        names = ['liblocnav_heap_sample.so'] + (['fixture'] if args.with_fixture else [])
        report['artifacts_sha256'] = {str(output/name):sha(output/name) for name in names}
        report['helper_elf'] = helper
        report['success'] = True
    finally:
        report['finished_unix'] = time.time()
        (output/'build_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(output/'build_manifest.json')


if __name__ == '__main__':
    main()
