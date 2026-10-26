#!/usr/bin/env python3
"""Render measured collapsed stacks; stdlib only, no external SVG assets."""
import argparse
from collections import Counter
import hashlib
import html
import json
import math
import os
from pathlib import Path
import re
import subprocess


def parse_collapsed(content):
    result = Counter()
    for line in content.splitlines():
        if not line.strip():
            continue
        stack, weight = line.rsplit(None, 1)
        value = float(weight)
        if not math.isfinite(value) or value < 0:
            raise ValueError('Stack weight must be finite and nonnegative')
        if value:
            result[tuple(stack.split(';'))] += value
    if not result:
        raise ValueError('No nonzero stack samples; refusing an empty flame graph')
    return result


def summarize(stacks):
    inclusive, leaf = Counter(), Counter()
    for stack, weight in stacks.items():
        leaf[stack[-1]] += weight
        for frame in set(stack):
            inclusive[frame] += weight
    total = sum(stacks.values())
    return dict(total_weight=total, unique_stacks=len(stacks),
                inclusive=[dict(function=k, weight=v, percent=100*v/total)
                           for k, v in inclusive.most_common(80)],
                self=[dict(function=k, weight=v, percent=100*v/total)
                      for k, v in leaf.most_common(80)])


def svg(stacks, title, units):
    root = dict(weight=0., children={})
    for stack, weight in stacks.items():
        node = root
        node['weight'] += weight
        for frame in stack:
            node = node['children'].setdefault(frame, dict(weight=0., children={}))
            node['weight'] += weight
    depth = max(map(len, stacks))
    height, width = 90 + 19*depth, 1500
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" style="max-width:100%;height:auto" width="{width}" height="{height}" '
             f'viewBox="0 0 {width} {height}">', '<rect width="100%" height="100%" fill="#fcfaf5"/>',
             '<style>text{font:12px monospace}g:hover rect{stroke:#111;stroke-width:1.5}</style>',
             f'<text x="12" y="22">{html.escape(title)}</text>',
             f'<text x="12" y="43">Width = {html.escape(units)}; total {root["weight"]:g}. '
             'Hover a frame for its inclusive share. Stacks are aggregated, not a timeline.</text>']

    def render(node, x, y, available):
        position = x
        for name, child in sorted(node['children'].items()):
            size = available * child['weight'] / node['weight']
            if size >= .15:
                digest = hashlib.sha256(name.encode()).digest()
                color = f'rgb({220+digest[0]%35},{100+digest[1]%115},{45+digest[2]%80})'
                label = html.escape(f'{name}: {child["weight"]:g} {units} '
                                    f'({100*child["weight"]/root["weight"]:.2f}%)')
                parts.append(f'<g><title>{label}</title><rect x="{position:.3f}" y="{y}" '
                             f'width="{size:.3f}" height="18" fill="{color}"/>')
                if size >= 35:
                    limit = max(1, int((size-5)/7))
                    # Keep argument-heavy C++ symbols in the tooltip, and show
                    # the function component in the small on-chart label.
                    short = name.partition('(')[0].rsplit('::', 1)[-1]
                    clipped = short if len(short) <= limit else short[:limit-1] + '…'
                    parts.append(f'<text x="{position+3:.3f}" y="{y+13}">{html.escape(clipped)}</text>')
                parts.append('</g>')
                render(child, position, y-19, size)
            position += size
    render(root, 10, height-27, width-20)
    parts.append('</svg>')
    return '\n'.join(parts) + '\n'


def sampled_profile_info(profile, metadata, binary):
    """Validate complete live-only heap_v2 data; never reinterpret it as alloc_space."""
    text = profile.read_text()
    header = text.splitlines()[0] if text else ''
    match = re.fullmatch(r'heap profile:\s*(\d+):\s*(\d+)\s*\[\s*(\d+):\s*(\d+)\]\s*@ heap_v2/(\d+)', header)
    if not match or int(match[5]) <= 0:
        raise ValueError('sampled_inuse requires a real heap_v2/<positive period> profile')
    totals, period = tuple(map(int, match.groups()[:4])), int(match[5])
    buckets = re.findall(r'^\s*(\d+):\s*(\d+)\s*\[\s*(\d+):\s*(\d+)\]\s*@\s+0x', text, re.M)
    if totals[:2] != totals[2:] or tuple(sum(int(row[index]) for row in buckets) for index in range(4)) != totals:
        raise ValueError('Truncated/non-live sampled heap buckets; refusing incomplete data')
    if not totals[1]:
        raise ValueError('Valid empty live snapshot; refusing a fabricated nonempty flame graph')
    if not metadata or not metadata.is_file():
        raise ValueError('sampled_inuse requires the actual helper --sample-metadata JSON')
    witness = json.loads(metadata.read_text())
    if (not isinstance(witness, dict) or witness.get('valid') is not True or witness.get('kind') != 'sampled_live_heap_estimate' or
            witness.get('raw_header') != header or witness.get('profile_bytes') != profile.stat().st_size or
            witness.get('sample_parameter') != period or witness.get('full_heap_profiler_running') is not False or
            witness.get('cumulative_allocations_available') is not False or
            Path(witness.get('executable', '')).resolve() != binary.resolve()):
        raise ValueError('Sample metadata/executable identity does not match the raw live profile')
    mapped = text.partition('MAPPED_LIBRARIES:\n')[2]
    if str(binary.resolve()) not in mapped:
        raise ValueError('Sample lost the actual executable address map')
    return dict(sample_period_bytes=period, raw_sampled_live_objects=totals[0], raw_sampled_live_bytes=totals[1],
                raw_bucket_count=len(buckets), cumulative_allocations_available=False,
                metadata=str(metadata.resolve()), metadata_sha256=hashlib.sha256(metadata.read_bytes()).hexdigest(),
                profile_sha256=hashlib.sha256(profile.read_bytes()).hexdigest(), witness=witness,
                meaning='Raw sampled live objects; pprof subsequently applies v2 inclusion-probability correction. Not cumulative allocations.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--profile', type=Path)
    source.add_argument('--collapsed', type=Path)
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--pprof', default='google-pprof')
    parser.add_argument('--kind', choices=('cpu', 'inuse', 'alloc', 'sampled_inuse'), default='cpu')
    parser.add_argument('--sample-metadata', type=Path, help='Actual helper JSON required for a sampled live-heap graph')
    parser.add_argument('--output', type=Path, required=True, help='New report directory')
    parser.add_argument('--title', default='Measured call stacks')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new report output directory')
    if args.profile and (not args.binary or not args.binary.is_file()):
        parser.error('--profile requires the actual executable in --binary')
    sampled = None
    if args.profile:
        with args.profile.open('rb') as stream:
            first = stream.readline(1024).decode(errors='replace')
        is_v2 = first.startswith('heap profile:') and '@ heap_v2/' in first
        if is_v2 and args.kind not in ('inuse', 'sampled_inuse'):
            parser.error('heap_v2 contains only sampled live objects; cpu/alloc_space are invalid for this profile')
        if is_v2 or args.kind == 'sampled_inuse':
            try:
                sampled = sampled_profile_info(args.profile, args.sample_metadata, args.binary)
            except (OSError, ValueError, TypeError, KeyError) as error:
                parser.error(str(error))
            args.kind = 'sampled_inuse'
    elif args.kind == 'sampled_inuse':
        parser.error('sampled_inuse requires the original --profile and --sample-metadata; unproven collapsed weights are not accepted')
    args.output.mkdir(parents=True)
    flags = [] if args.kind == 'cpu' else [
        '--show_bytes', '--inuse_space' if args.kind in ('inuse', 'sampled_inuse') else '--alloc_space']
    env = os.environ.copy()
    env['PPROF_TMPDIR'] = str((args.output / 'symbolizer_tmp').resolve())
    if args.profile:
        command = [args.pprof, '--collapsed', *flags, str(args.binary), str(args.profile)]
        result = subprocess.run(command, env=env, text=True, capture_output=True, check=False)
        (args.output / 'symbolizer.log').write_text(result.stderr)
        if result.returncode:
            raise RuntimeError(f'pprof failed ({result.returncode}); see symbolizer.log')
        if sampled and ('Heap version 2' not in result.stderr or
                        f'1-in-{sampled["sample_period_bytes"]} sampling rate' not in result.stderr):
            raise RuntimeError('Cannot confirm heap_v2 probability correction; use compatible gperftools google-pprof')
        collapsed = result.stdout
        # Keep native flat/cumulative reports for cross-checking the SVG.
        with (args.output / 'top.txt').open('w') as stream:
            subprocess.run([args.pprof, '--text', '--cum', *flags, str(args.binary),
                            str(args.profile)], env=env, stdout=stream,
                           stderr=subprocess.DEVNULL, check=True)
    else:
        command = None
        collapsed = args.collapsed.read_text()
    stacks = parse_collapsed(collapsed)
    (args.output / 'stacks.folded').write_text(collapsed)
    units = 'CPU samples' if args.kind == 'cpu' else (
        'estimated live heap bytes (v2 sampling)' if sampled else
        'live heap bytes' if args.kind == 'inuse' else 'cumulative allocated bytes')
    report = summarize(stacks)
    report.update(kind=args.kind, units=units, command=command,
                  profile=str(args.profile.resolve()) if args.profile else None,
                  binary=str(args.binary.resolve()) if args.binary else None,
                  limitations=['User-space CPU samples exclude kernel stacks and waiting time.',
                               'gperftools merges thread stacks; this graph is not a per-thread timeline.',
                               'Stripped libraries may resolve to nearby exported symbols; verify native debug symbols before changing code.',
                               'Inclusive percentages overlap across callers.',
                               'Heap mode replaces the allocator with tcmalloc; run separately.',
                               'A live-heap snapshot is not process RSS, PSS, or peak heap.'])
    if sampled:
        report['sampled_live_heap'] = sampled
        report['limitations'].extend([
            'Widths are pprof-corrected live-byte estimates with sampling uncertainty, not a complete allocation census.',
            'The bracketed columns duplicate the live sample; alloc_space and summing snapshots as cumulative allocations are invalid.',
            'A largest observed snapshot is not a continuous peak or proof of a leak. Collector/allocator overhead remains part of this diagnostic.'])
    (args.output / 'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    (args.output / 'flame.svg').write_text(svg(stacks, args.title, units))
    print(args.output / 'flame.svg')


if __name__ == '__main__':
    main()
