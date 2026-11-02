#!/usr/bin/env python3
"""RGBD36 build paths and source/dependency identity; no board operations."""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess

TOOLS = Path(__file__).resolve().parent
CHECKOUT = TOOLS.parents[2]
ART = CHECKOUT.parent
REPO = ART.parents[1]
OLD = REPO / 'artifacts/rgbd_tracking_optimizations_20261020'
BASE = Path('/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_scale_20261012_current')
CACHE = Path('/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_track_timing')
IMAGE = 'sha256:c073994816af4aa6c776a98adf5bb1020b92134b4f59f5dec357c5dcb3fdd349'
DEPENDENCIES = OLD / 'preflight/BUILD_DEPENDENCIES.json'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def compiled_paths(root=CHECKOUT):
    core = sorted((root / 'src').glob('*.cc')) + sorted((root / 'src').glob('*.cpp')) + sorted((root / 'src/CameraModels').glob('*.cpp'))
    node = [root / ('runtime/rgbd/src/orb_slam3_ros-master/src/' + n + '.cc') for n in ['ros_rgbd', 'common']]
    headers = sorted((root / 'include').rglob('*.h')) + sorted((root / 'include').rglob('*.hpp'))
    for d in ['src/orb_slam3_ros-master/include', 'diagnostics', 'memory']:
        headers += sorted((root / 'runtime/rgbd' / d).rglob('*.h'))
        headers += sorted((root / 'runtime/rgbd' / d).rglob('*.hpp'))
    original = {n for n in original_bindings() if n.startswith('src/') and n.endswith(('.cc', '.cpp'))}
    assert len(original) == 41 and len(node) == 2
    assert original <= {str(p.relative_to(root)) for p in core}, 'an original core TU was removed'
    paths = sorted(set(core + node + headers))
    assert all(p.is_file() and not p.is_symlink() for p in paths)
    return paths


def inputs(root=CHECKOUT):
    return {str(p.relative_to(root)): sha(p) for p in compiled_paths(root)}


def core_count(compile_inputs):
    return sum(n.startswith('src/') and n.endswith(('.cc', '.cpp')) for n in compile_inputs)


def original_bindings():
    value = json.loads((CHECKOUT / 'runtime/rgbd/PROVENANCE.json').read_text())['compiled_inputs_sha256']
    assert len(value) == 116
    return value


def verify_ready(path):
    ready = json.loads(Path(path).read_text())
    assert ready['ready'] is True, 'source freeze is not ready'
    declared = ready['files']
    for relative, digest in declared.items():
        p = CHECKOUT / relative
        p.resolve().relative_to(CHECKOUT.resolve())
        assert sha(p) == digest, 'source changed since freeze: ' + relative
    actual = inputs()
    assert set(original_bindings()) <= set(actual), 'an original compile input was removed'
    assert set(actual) <= set(declared), 'freeze misses compile inputs: ' + str(set(actual) - set(declared))
    assert all(declared[n] == h for n, h in actual.items())
    return ready, actual


def verify_dependencies(inspect_image=True):
    # Reuse the immutable audit logic, without changing its source or output.
    path = OLD / 'tools/build_dependencies.py'
    spec = importlib.util.spec_from_file_location('rgbd25_dependencies_readonly', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = module.verify()
    assert len(result['files']) == 32052
    actual = None
    if inspect_image:
        actual = subprocess.check_output(['docker', 'image', 'inspect', IMAGE, '--format', '{{.Id}}'], text=True).strip()
        assert actual == IMAGE
    return {'files_verified': len(result['files']), 'manifest_sha256': sha(DEPENDENCIES),
            'verifier_sha256': sha(path), 'image': actual, 'configured_image': IMAGE,
            'image_inspected': inspect_image}


def snapshot(destination, compile_inputs):
    destination.mkdir(parents=True, exist_ok=False)
    paths = set(CHECKOUT / n for n in compile_inputs)
    paths.add(CHECKOUT / 'CMakeLists.txt')
    paths.update(p for p in (CHECKOUT / 'runtime/rgbd').rglob('*') if p.is_file() and not p.is_symlink())
    records = {}
    for source in sorted(paths):
        relative = source.relative_to(CHECKOUT)
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        records[str(relative)] = sha(source)
        assert sha(target) == records[str(relative)]
    (destination / 'Thirdparty').mkdir(exist_ok=True)
    assert inputs(destination) == compile_inputs
    return records


def verify_snapshot(destination, records):
    for n, h in records.items():
        assert sha(CHECKOUT / n) == h, 'live source changed: ' + n
        assert sha(destination / n) == h, 'compile snapshot changed: ' + n


def elf(path, machine):
    with Path(path).open('rb') as f:
        h = f.read(20)
    assert h[:6] == b'\x7fELF\x02\x01' and int.from_bytes(h[18:20], 'little') == machine, path


def driver_inputs():
    return {p.name: sha(p) for p in sorted(TOOLS.glob('build*.py'))}


def freeze_drivers(destination, expected):
    assert driver_inputs() == expected, 'build driver changed before execution'
    destination.mkdir()
    for name, h in expected.items():
        shutil.copy2(TOOLS / name, destination / name)
        assert sha(destination / name) == h


def verify_drivers(destination, expected):
    assert driver_inputs() == expected, 'build driver changed during execution'
    assert {p.name: sha(p) for p in destination.glob('build*.py')} == expected
