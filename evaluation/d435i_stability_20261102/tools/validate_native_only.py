#!/usr/bin/env python3
"""Fixture-only follow-up against an already cold-built, matching host DSO.

Preserves the failed host01 evidence and does not add to its build*.py driver
set. No core/node objects are rebuilt or relinked here.
"""
import argparse
import json
import shlex
import shutil
import subprocess
import time
from pathlib import Path

from build_common import ART, CHECKOUT, driver_inputs, elf, inputs, sha, write
from build_host import bash_command, limit_memory, run_stability_modes


def header_paths(path, cwd):
    content = path.read_text().split(':', 1)[1].replace('\\\n', ' ')
    return sorted({(Path(n) if Path(n).is_absolute() else cwd / n).resolve()
                   for n in shlex.split(content)})


def loader_paths(text):
    assert 'not found' not in text
    found = set()
    for line in text.splitlines():
        value = line.split('=>', 1)[1].strip().split()[0] if '=>' in line else line.strip().split()[0]
        if value.startswith('/'):
            found.add(Path(value).resolve())
    return sorted(found)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--name', default='native_fixture_02')
    p.add_argument('--host-build', default='diagnostic_host_01')
    p.add_argument('--fixture', type=Path, required=True)
    p.add_argument('--fixture-sha256', required=True)
    a = p.parse_args()
    assert a.name.replace('_', '').isalnum() and a.host_build.replace('_', '').isalnum()
    host = ART / 'builds' / a.host_build
    previous = json.loads((host / 'RESULT.json').read_text())
    assert previous['build_complete'] and previous['core_compiles'] == 42 and previous['node_compiles'] == 2
    expected = previous['inputs']
    assert len(expected) == 118 and inputs() == expected
    for n, h in expected.items():
        assert sha(host / 'input' / n) == h
    # Check only the original driver set, rather than retroactively extending it.
    original_drivers = previous['driver_sha256']
    assert {n: sha(Path(__file__).parent / n) for n in original_drivers} == original_drivers
    fixture = a.fixture.resolve()
    fixture.relative_to(CHECKOUT)
    assert sha(fixture) == a.fixture_sha256
    out = ART / 'builds' / a.name
    out.mkdir(parents=True, exist_ok=False)
    frozen = out / fixture.name
    shutil.copy2(fixture, frozen)
    assert sha(frozen) == a.fixture_sha256
    core = host / 'lib/liborb_slam3_ros.so'
    assert sha(core) == previous['binaries']['liborb_slam3_ros.so']
    elf(core, 62)
    compile_file = host / 'catkin_build/compile_commands.json'
    cc = json.loads(compile_file.read_text())
    template = next(x for x in cc if 'CMakeFiles/orb_slam3_ros.dir/' in x['command'])
    cwd = Path(template['directory'])
    options, skip = [], False
    for arg in shlex.split(template['command']):
        if skip:
            skip = False
            continue
        if arg == '-o':
            skip = True
        elif arg not in ['-c', template['file']]:
            options.append(arg)
    links = list((host / 'catkin_build').rglob('rgbd_mappoint_probe.dir/link.txt'))
    assert len(links) == 1
    link_file = links[0]
    dependencies = out / 'headers.d'
    obj = out / 'stability_native.o'
    binary = out / 'stability_native'
    result = dict(kind='fixture_only_against_completed_host_build', host_build=a.host_build,
        original_host_result_sha256=sha(host / 'RESULT.json'), original_core_inputs=expected,
        core_compiles=0, node_compiles=0, fixture_compiles=0, matching_core_sha256=sha(core),
        source_ready_sha256=previous['source_ready_sha256'],
        fixture_path=str(fixture), fixture_sha256=a.fixture_sha256,
        fixture_snapshot_sha256=sha(frozen), original_driver_sha256=original_drivers,
        driver_sha256=sha(Path(__file__)), compile_commands_sha256=sha(compile_file),
        reused_link_file_sha256=sha(link_file), steps=[], complete=False,
        fixtures_complete=False, stability_native_executed=False, started_wall_ns=time.time_ns())
    write(out / 'RESULT.json', result)

    def step(name, argv, cwd=None, timeout=60):
        record = dict(name=name, argv=list(map(str, argv)), cwd=str(cwd) if cwd else None,
                      timeout_seconds=timeout, started_wall_ns=time.time_ns())
        result['steps'].append(record)
        write(out / 'RESULT.json', result)
        with (out / (name + '.log')).open('wb') as log:
            try:
                record['exit_code'] = subprocess.run(bash_command(argv), cwd=cwd,
                    stdout=log, stderr=subprocess.STDOUT, timeout=timeout,
                    preexec_fn=limit_memory).returncode
            except subprocess.TimeoutExpired:
                record.update(exit_code=None, timed_out=True)
                write(out / 'RESULT.json', result)
                raise
        record['ended_wall_ns'] = time.time_ns()
        write(out / 'RESULT.json', result)
        assert record['exit_code'] == 0, name + ' failed; preserve fixture attempt'
        print('NATIVE_ONLY_STEP', name, 'passed', flush=True)
        return record

    try:
        step('preprocess_dependencies', [*options, '-M', frozen, '-MF', dependencies], cwd=cwd)
        headers = header_paths(dependencies, cwd)
        header_before = {str(v): sha(v) for v in headers}
        result['actual_compilation_dependencies_before'] = header_before
        step('fixture_compile', [*options, '-c', frozen, '-o', obj], cwd=cwd, timeout=180)
        result['fixture_compiles'] = 1
        link, following_output = [], False
        for arg in shlex.split(link_file.read_text()):
            if following_output:
                link.append(str(binary))
                following_output = False
            elif arg == '-o':
                link.append(arg)
                following_output = True
            elif not arg.endswith('.o'):
                link.append(arg)
        link.insert(1, str(obj))
        step('fixture_link', link, cwd=link_file.parents[2])
        elf(binary, 62)
        result['binary_sha256'] = sha(binary)
        loader = subprocess.check_output(bash_command(['ldd', binary]), text=True)
        (out / 'resolved_libraries.txt').write_text(loader)
        loaded = loader_paths(loader)
        assert core.resolve() in loaded, 'fixture is not bound to the verified host DSO'
        library_before = {str(v): sha(v) for v in loaded}
        result['resolved_shared_library_sha256_before'] = library_before
        run_stability_modes(binary, out, step, result)
        assert inputs() == expected
        for n, h in expected.items():
            assert sha(host / 'input' / n) == h
        assert sha(fixture) == sha(frozen) == a.fixture_sha256
        assert sha(core) == result['matching_core_sha256']
        assert sha(Path(__file__)) == result['driver_sha256']
        assert {n: sha(Path(__file__).parent / n) for n in original_drivers} == original_drivers
        assert sha(host / 'RESULT.json') == result['original_host_result_sha256']
        assert sha(compile_file) == result['compile_commands_sha256']
        assert sha(link_file) == result['reused_link_file_sha256']
        assert {n: sha(Path(n)) for n in header_before} == header_before
        assert {n: sha(Path(n)) for n in library_before} == library_before
        assert sha(binary) == result['binary_sha256']
        result.update(complete=True, fixtures_complete=True, inputs_stable_before_after=True,
                      actual_compilation_dependencies_after=header_before,
                      resolved_shared_library_sha256_after=library_before)
    except BaseException as e:
        result['error'] = repr(e)
        raise
    finally:
        result['ended_wall_ns'] = time.time_ns()
        write(out / 'RESULT.json', result)
    write(out / 'manifest.json', result)
    print('NATIVE_FIXTURE_ONLY_FOUR_MODES_VERIFIED', flush=True)


if __name__ == '__main__':
    main()
