#!/usr/bin/env python3
"""Cold ARM core/node from the isolated publication-layout worktree.

Default is read-only preparation. --execute requires an explicit source freeze;
the operator must wait for source-owner freeze and root-agent authorization.
"""
import argparse
import json
import os
import re
import subprocess
import time
from build_common import *


def command(out, name):
    source = out / 'input'
    return ['docker', 'run', '--rm', '--name', 'rgbd36-' + name,
            '--cidfile', str(out / 'container.cid'), '--network', 'none',
            '--memory', '8g', '--memory-swap', '8g', '--user', f'{os.getuid()}:{os.getgid()}',
            '-v', f'{source / "runtime/rgbd"}:/ros1_project/orb_slam:ro',
            '-v', f'{source}:/ros1_project/orb_slam3_core:ro',
            '-v', f'{BASE / "core/Thirdparty"}:/ros1_project/orb_slam3_core/Thirdparty:ro',
            '-v', f'{CACHE}:/cache:ro',
            '-v', '/home/ywl/temp/deps_ws:/ros1_project/deps_ws:ro',
            '-v', f'{REPO / "xfeat_rdk_x5/rdk_x5"}:/ros1_project/orb_slam3_dbow3/xfeat_rdk_x5/rdk_x5:ro',
            '-v', f'{out / "driver"}:/driver:ro', '-v', f'{out}:/output:rw',
            '-w', '/ros1_project/orb_slam', '--entrypoint', 'python3', IMAGE,
            '/driver/build_arm_inner.py']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--name', default='diagnostic_01')
    p.add_argument('--source-ready', type=Path, default=ART / 'preflight/SOURCE_READY.json')
    p.add_argument('--execute', action='store_true')
    p.add_argument('--check-only', action='store_true')
    a = p.parse_args()
    assert re.fullmatch(r'[A-Za-z0-9_]+', a.name)
    assert not (a.execute and a.check_only)
    out = ART / 'builds' / a.name
    actual = inputs()
    original = original_bindings()
    assert set(original) <= set(actual)
    dependencies = verify_dependencies(inspect_image=a.execute)
    preparation = dict(source_root=str(CHECKOUT), core_units=core_count(actual), original_core_units=41, node_units=2,
        original_compile_inputs=len(original), current_compile_inputs=len(actual),
        modified_original_inputs=[n for n, h in original.items() if actual[n] != h],
        added_compile_inputs=sorted(set(actual) - set(original)),
        dependencies=dependencies, command=command(out, a.name),
        executable_work_performed=False, source_freeze_required_before_build=True,
        source_ready_present=a.source_ready.is_file(),
        driver_sha256=driver_inputs())
    if not a.execute:
        write(ART / 'preflight/BUILD_DRIVER_CHECK.json', preparation)
        print(json.dumps(preparation, ensure_ascii=False, indent=2))
        return
    ready, actual = verify_ready(a.source_ready)
    out.mkdir(parents=True, exist_ok=False)
    freeze_drivers(out / 'driver', preparation['driver_sha256'])
    records = snapshot(out / 'input', actual)
    write(out / 'build_request.json', {'expected_core_compiles': core_count(actual), 'publication_inputs': actual})
    result = dict(preparation, executable_work_performed=True, inputs=actual, source_snapshot_files=records,
        source_ready_sha256=sha(a.source_ready), source_ready_file=str(a.source_ready),
        build_kind='full_cold', core_compiles=0, node_compiles=0,
        cached_core_objects_reused=0, node_objects_reused=0, build_complete=False,
        started_wall_ns=time.time_ns())
    write(out / 'RESULT.json', result)
    try:
        with (out / 'build.log').open('wb') as f:
            proc = subprocess.Popen(command(out, a.name), stdout=f, stderr=subprocess.STDOUT)
            result['client_pid'] = proc.pid
            write(out / 'RESULT.json', result)
            result['exit_code'] = proc.wait()
        assert result['exit_code'] == 0, 'cold build failed; preserve this attempt'
        manifest = json.loads((out / 'manifest.json').read_text())
        assert manifest['core_compiles'] == core_count(actual) and manifest['node_compiles'] == 2
        assert manifest['cached_core_objects_reused'] == manifest['node_objects_reused'] == 0
        assert manifest['input_stability_verified'] and manifest['publication_inputs'] == actual
        for name, digest in manifest['binaries'].items():
            assert sha(out / name) == digest
            elf(out / name, 183)
        verify_ready(a.source_ready)
        assert sha(a.source_ready) == result['source_ready_sha256']
        verify_snapshot(out / 'input', records)
        verify_drivers(out / 'driver', preparation['driver_sha256'])
        assert verify_dependencies() == dependencies
        result.update(build_complete=True, input_stability_verified=True,
                      binaries=manifest['binaries'], core_compiles=core_count(actual), node_compiles=2)
    except BaseException as e:
        result['error'] = repr(e)
        raise
    finally:
        result['driver_sha256_after'] = driver_inputs()
        try:
            verify_drivers(out / 'driver', preparation['driver_sha256'])
            result['driver_stability_verified'] = True
        except BaseException as e:
            result['driver_stability_verified'] = False
            result['driver_stability_error'] = repr(e)
            result['build_complete'] = False
        result['ended_wall_ns'] = time.time_ns()
        result['elapsed_s'] = (result['ended_wall_ns'] - result['started_wall_ns']) / 1e9
        write(out / 'RESULT.json', result)
    assert result['driver_stability_verified']
    print('ARM_FULL_COLD_BUILD_VERIFIED', flush=True)


if __name__ == '__main__':
    main()
