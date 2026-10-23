#!/usr/bin/env python3
"""Check this published source checkpoint without requiring local raw artifacts."""
import hashlib
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    provenance = json.loads((HERE / 'PROVENANCE.json').read_text())
    for relative, entry in provenance['published_files'].items():
        assert sha(REPO / relative) == entry['sha256'], relative
    inputs = provenance['compiled_inputs_sha256']
    assert len(inputs) == 116
    for relative, expected in inputs.items():
        assert sha(REPO / relative) == expected, relative
    cmake = (HERE / 'src/orb_slam3_ros-master/CMakeLists.txt').read_text()
    library = cmake.split('add_library(${PROJECT_NAME} SHARED', 1)[1].split('\n)', 1)[0]
    declared = set(re.findall(r'\$\{ORB_SLAM3_SOURCE_DIR\}/(src/[^\s)]+)', library))
    actual = {str(p.relative_to(REPO)) for p in (REPO / 'src').glob('*.cc')}
    actual.update(str(p.relative_to(REPO)) for p in (REPO / 'src').glob('*.cpp'))
    actual.update(str(p.relative_to(REPO)) for p in (REPO / 'src/CameraModels').glob('*.cpp'))
    assert declared == actual and len(actual) == 41
    assert 'src/MapScaleObservation.cc' in declared
    assert 'set(CMAKE_CXX_STANDARD 17)' in cmake
    assert 'add_dependencies(${rgbd_target}' in cmake
    assert '${ORB_SLAM3_SOURCE_DIR}/Thirdparty/DBoW3/lib/libDBoW3.so' in cmake
    assert '${ORB_SLAM3_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so' in cmake
    for relative, entry in provenance['build_adaptations'].items():
        assert sha(REPO / relative) == entry['published_sha256']
    print(json.dumps({'source_integrity_pass': True, 'candidate': provenance['candidate'],
                      'published_inputs': len(provenance['published_files']),
                      'original_arm_compilation_inputs': len(inputs), 'core_translation_units': 41,
                      'performance_acceptance': 'not established by this source integrity check'}))


if __name__ == '__main__':
    main()
