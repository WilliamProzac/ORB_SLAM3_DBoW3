#!/usr/bin/env python3
"""Fresh publication-layout host catkin build and actual-DSO method probes."""
import argparse
import json
import os
import re
import resource
import shlex
import shutil
import subprocess
import time
from build_common import *

STABILITY_NATIVE_MODES = ('disabled', 'off', 'on', 'flush-failure')


def run_stability_modes(binary, out, step, result):
    """Four isolated processes; each fixture owns its new existing directory."""
    base = out / 'stability_native_modes'
    base.mkdir()
    result['stability_native_modes'] = {}
    for mode in STABILITY_NATIVE_MODES:
        output = base / mode
        output.mkdir()
        record = dict(output_directory=str(output), timeout_seconds=60,
                      executed=False, passed=False, exit_code=None)
        result['stability_native_modes'][mode] = record
        name = 'native_fixture_' + mode.replace('-', '_')
        try:
            completed = step(name, [binary, mode, output], timeout=60)
        except BaseException:
            if result.get('steps') and result['steps'][-1]['name'] == name:
                record.update({k: result['steps'][-1][k] for k in ['exit_code', 'timed_out']
                               if k in result['steps'][-1]})
                record['executed'] = 'exit_code' in result['steps'][-1]
            raise
        record.update(executed=True, passed=completed['exit_code'] == 0,
                      exit_code=completed['exit_code'])
    assert set(result['stability_native_modes']) == set(STABILITY_NATIVE_MODES)
    assert all(r['executed'] and r['passed'] and r['exit_code'] == 0
               for r in result['stability_native_modes'].values())
    result['stability_native_executed'] = True


def bash_command(argv):
    return ['bash', '-c', 'source /opt/ros/noetic/setup.bash\nexec "$@"', 'rgbd36-host', *map(str, argv)]


def limit_memory():
    cap = 8 * 1024 ** 3
    resource.setrlimit(resource.RLIMIT_AS, (cap, cap))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--name', default='diagnostic_host_01')
    p.add_argument('--source-ready', type=Path, default=ART / 'preflight/SOURCE_READY.json')
    p.add_argument('--execute', action='store_true')
    p.add_argument('--native-fixture', type=Path)
    a = p.parse_args()
    assert re.fullmatch(r'[A-Za-z0-9_]+', a.name)
    actual = inputs()
    out = ART / 'builds' / a.name
    source = out / 'input'
    build = out / 'catkin_build'
    devel = out / 'devel'
    generated_library = out / 'lib'
    targets = ['ros_rgbd', 'rgbd_mappoint_probe', 'rgbd_geometry_probe']
    commands = [
        ['cmake', '-S', str(source / 'runtime/rgbd/src'), '-B', str(build),
         '-DORB_SLAM3_SOURCE_DIR=' + str(source), '-DCMAKE_BUILD_TYPE=Release',
         '-DCATKIN_DEVEL_PREFIX=' + str(devel), '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON',
         '-DORB_SLAM3_LIBRARY_OUTPUT_DIRECTORY=' + str(generated_library),
         '-DPYTHON_EXECUTABLE=/usr/bin/python3', '-DBUILD_ORB_EXTRACTOR_VALIDATION=OFF',
         '-DBUILD_RGBD_QUERY_PROBES=ON', '-DENABLE_XFEAT_HYBRID=OFF',
         '-DCATKIN_WHITELIST_PACKAGES=robot_interfaces;orb_slam3_ros'],
        ['cmake', '--build', str(build), '--target', *targets, '--', '-j2']]
    preparation = dict(source_root=str(CHECKOUT), kind='host_full_cold_and_actual_dso_probes',
        command=commands, source_freeze_required_before_build=True, source_ready_present=a.source_ready.is_file(),
        core_units=core_count(actual), original_core_units=41, node_units=2,
        original_compile_inputs=116, current_compile_inputs=len(actual),
        required_stability_native_modes=list(STABILITY_NATIVE_MODES), stability_native_timeout_seconds=60,
        thirdparty_link_source='Existing x86_64 DBoW3/g2o DSOs; architecture/SHA checked, no dependency rebuild.',
        memory_limit_scope='8 GiB address space per child process, two compile jobs; not an aggregate cgroup cap',
        executable_work_performed=False, driver_sha256=driver_inputs())
    if not a.execute:
        write(ART / 'preflight/HOST_BUILD_DRIVER_CHECK.json', preparation)
        print(json.dumps(preparation, indent=2))
        return
    ready, actual = verify_ready(a.source_ready)
    fixture = (a.native_fixture or CHECKOUT / 'evaluation/d435i_stability_20261102/native/stability_native.cc').resolve()
    fixture.relative_to(CHECKOUT)
    assert fixture.is_file(), 'RGBD36 requires stability_native.cc; no silent fixture skip'
    out.mkdir(parents=True, exist_ok=False)
    freeze_drivers(out / 'driver', preparation['driver_sha256'])
    records = snapshot(source, actual)
    thirdparty = source / 'Thirdparty'
    shutil.rmtree(thirdparty)
    shutil.copytree(CHECKOUT / 'Thirdparty', thirdparty,
                    ignore=shutil.ignore_patterns('build', 'lib', '*.so', '*.so.*', '*.a', '*.o'))
    thirdparty_headers = {str(p.relative_to(thirdparty)): sha(p)
                         for p in sorted(thirdparty.rglob('*')) if p.is_file()}
    host_libraries = {}
    for name, filename in [('DBoW3', 'libDBoW3.so'), ('g2o', 'libg2o.so')]:
        library = REPO / 'Thirdparty' / name / 'lib' / filename
        elf(library, 62)
        host_libraries[str(library.resolve())] = sha(library)
        destination = thirdparty / name / 'lib'
        destination.mkdir()
        # Include versioned SONAME companions, which the loader may resolve.
        for library_entry in sorted(library.parent.glob('*.so*')):
            if library_entry.is_file():
                elf(library_entry, 62)
                host_libraries[str(library_entry.resolve())] = sha(library_entry)
                (destination / library_entry.name).symlink_to(library_entry.resolve())
    top = source / 'runtime/rgbd/src/CMakeLists.txt'
    assert not top.exists()
    top.symlink_to('/opt/ros/noetic/share/catkin/cmake/toplevel.cmake')
    fixture_inputs = {}
    frozen_fixture = out / 'native_fixture' / fixture.name
    frozen_fixture.parent.mkdir()
    shutil.copy2(fixture, frozen_fixture)
    fixture_inputs[str(fixture)] = sha(fixture)
    assert sha(frozen_fixture) == sha(fixture)
    result = dict(preparation, executable_work_performed=True, inputs=actual, source_snapshot_files=records,
        source_ready_sha256=sha(a.source_ready), host_library_dependencies=host_libraries,
        host_thirdparty_inputs=thirdparty_headers, fixture_inputs=fixture_inputs,
        core_compiles=0, node_compiles=0, cached_core_objects_reused=0, node_objects_reused=0,
        build_complete=False, fixtures_complete=False, stability_native_executed=False,
        started_wall_ns=time.time_ns(), steps=[])
    write(out / 'RESULT.json', result)

    def step(name, argv, cwd=None, timeout=7200):
        record = {'name': name, 'argv': list(map(str, argv)), 'started_wall_ns': time.time_ns(), 'timeout_seconds': timeout}
        result['steps'].append(record)
        write(out / 'RESULT.json', result)
        with (out / (name + '.log')).open('wb') as log:
            try:
                record['exit_code'] = subprocess.run(bash_command(argv), cwd=cwd, stdout=log,
                    stderr=subprocess.STDOUT, preexec_fn=limit_memory, timeout=timeout).returncode
            except subprocess.TimeoutExpired:
                record.update(exit_code=None, timed_out=True, ended_wall_ns=time.time_ns())
                write(out / 'RESULT.json', result)
                raise
        record['ended_wall_ns'] = time.time_ns()
        write(out / 'RESULT.json', result)
        assert record['exit_code'] == 0, name + ' failed; preserve attempt'
        print('HOST_STEP', name, 'passed', flush=True)
        return record

    try:
        for i, argv in enumerate(commands):
            step('build_%02d' % i, argv)
        cc = json.loads((build / 'compile_commands.json').read_text())
        core_commands = [x for x in cc if 'CMakeFiles/orb_slam3_ros.dir/' in x['command']]
        node_commands = [x for x in cc if 'CMakeFiles/ros_rgbd.dir/' in x['command']]
        assert len(core_commands) == core_count(actual) and len(node_commands) == 2
        assert {str(Path(x['file']).relative_to(source)) for x in core_commands + node_commands} == {
            n for n in actual if n.endswith(('.cc', '.cpp'))}
        for x in core_commands + node_commands:
            assert str(source) in x['file'] and str(REPO / 'src/') not in x['command']
            assert str(CACHE / 'devel') not in x['command']
            assert '-std=gnu++17' in x['command']
        library = generated_library / 'liborb_slam3_ros.so'
        binaries = {'liborb_slam3_ros.so': library, 'ros_rgbd': devel / 'lib/orb_slam3_ros/ros_rgbd',
                    'rgbd_mappoint_probe': devel / 'lib/orb_slam3_ros/rgbd_mappoint_probe',
                    'rgbd_geometry_probe': devel / 'lib/orb_slam3_ros/rgbd_geometry_probe'}
        for name, path in binaries.items():
            elf(path, 62)
            shutil.copy2(path, out / name)
        result.update(build_complete=True, core_compiles=len(core_commands), node_compiles=2,
                      binaries={n: sha(out / n) for n in binaries}, compile_commands=cc)
        write(out / 'RESULT.json', result)
        step('mappoint_contract', [binaries['rgbd_mappoint_probe'], 'contract'], timeout=60)
        geometry = out / 'geometry_results'
        geometry.mkdir()
        step('geometry_contract', [binaries['rgbd_geometry_probe'], geometry, '0'], timeout=60)
        if frozen_fixture:
            template = core_commands[0]
            argv = shlex.split(template['command'])
            stripped = []
            skip = False
            for arg in argv:
                if skip:
                    skip = False
                    continue
                if arg == '-o':
                    skip = True
                    continue
                if arg in ['-c', template['file']]:
                    continue
                stripped.append(arg)
            obj = out / 'stability_native.o'
            step('native_fixture_compile', [*stripped, '-c', frozen_fixture, '-o', obj], cwd=template['directory'])
            links = list(build.rglob('rgbd_mappoint_probe.dir/link.txt'))
            assert len(links) == 1
            argv = shlex.split(links[0].read_text())
            link = []
            next_output = False
            for arg in argv:
                if next_output:
                    link.append(str(out / 'stability_native'))
                    next_output = False
                elif arg == '-o':
                    link.append(arg)
                    next_output = True
                elif not arg.endswith('.o'):
                    link.append(arg)
            link.insert(1, str(obj))
            step('native_fixture_link', link, cwd=links[0].parents[2])
            elf(out / 'stability_native', 62)
            run_stability_modes(out / 'stability_native', out, step, result)
            result['binaries']['stability_native'] = sha(out / 'stability_native')
        verify_ready(a.source_ready)
        assert sha(a.source_ready) == result['source_ready_sha256']
        verify_snapshot(source, records)
        verify_drivers(out / 'driver', preparation['driver_sha256'])
        assert all(sha(Path(n)) == h for n, h in host_libraries.items())
        assert all(sha(Path(n)) == h for n, h in fixture_inputs.items())
        assert {str(p.relative_to(thirdparty)): sha(p) for p in thirdparty.rglob('*')
                if p.is_file() and not p.is_symlink()} == thirdparty_headers
        assert result['stability_native_executed'] is True
        result.update(fixtures_complete=True, input_stability_verified=True)
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
            result['build_complete'] = result['fixtures_complete'] = False
        result['ended_wall_ns'] = time.time_ns()
        write(out / 'RESULT.json', result)
    assert result['driver_stability_verified']
    write(out / 'manifest.json', {k: result[k] for k in ['inputs', 'binaries', 'core_compiles', 'node_compiles',
        'input_stability_verified', 'fixtures_complete', 'stability_native_executed', 'stability_native_modes',
        'cached_core_objects_reused', 'node_objects_reused']})
    print('HOST_FULL_COLD_BUILD_AND_PROBES_VERIFIED', flush=True)


if __name__ == '__main__':
    main()
