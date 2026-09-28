#!/usr/bin/env python3
"""Export pinned Git source into a NEW root; never alter an existing checkout."""
import argparse,hashlib,io,json,re,subprocess,tarfile
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('destination',type=Path);args=p.parse_args()
    root=args.destination.resolve();root.mkdir(exist_ok=False)
    ide=Path('/home/ywl/Project/ORB_SLAM3_DBoW3')
    gf=Path('/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_modules_20260928')
    records={}
    for name,repo,ref in [('official','/home/ywl/Project/ORB_SLAM3','4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4'),('delivered','/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_memory_lifecycle/core','4d6df6a2db5afd71dc9cf666106884046a85abb3')]:
        dest=root/name;dest.mkdir()
        archive=subprocess.check_output(['git','-C',repo,'archive',ref,'src','include','Thirdparty','LICENSE','CMakeLists.txt'])
        tarfile.open(fileobj=io.BytesIO(archive)).extractall(dest)
        files={str(f.relative_to(dest)):hashlib.sha256(f.read_bytes()).hexdigest() for f in dest.rglob('*') if f.is_file()}
        if name=='delivered':
            source=ide/'artifacts/delivery_rgbd700_q8_20260917/rdk_x5_rgbd700_q8_delivery/source/current/core'
            for rel,value in files.items():
                if rel.startswith(('src/','include/')):assert hashlib.sha256((source/rel).read_bytes()).hexdigest()==value,rel
        records[name]={'repository':repo,'sha':ref,'original_files':files}
        header=dest/'include/System.h';s,n=re.subn(r'(class System\s*\{)',r'\1\n    friend class ControlsAccess;',header.read_text());assert n==1;header.write_text(s)
        loop=dest/'src/LoopClosing.cc';s=loop.read_text()
        needle='if(CheckNewKeyFrames())' if name=='official' else 'if (CheckNewKeyFrames())'
        assert s.count(needle)==1
        loop.write_text(s.replace(needle,needle.replace('CheckNewKeyFrames()', 'mbActiveLC && CheckNewKeyFrames()')))
    (root/'source_manifest.json').write_text(json.dumps(records,indent=2)+'\n')
    (root/'harness').symlink_to(Path(__file__).resolve().parent,target_is_directory=True)
    (root/'configs').mkdir();(root/'vocabulary').mkdir()
    (root/'vocabulary/ORBvoc.txt').write_bytes((ide/'Vocabulary/ORBvoc.txt').read_bytes())
    for data in ['euroc','tum']:
        for mode in ['official800','delivered800','delivered700']:
            s=(gf/'configs'/f'{data}800.yaml').read_text()+'\nCamera.type: "PinHole"\nloopClosing: 0\n'
            if mode.startswith('delivered'):s+='ORBextractor.parallelWorkers: 3\nRGBD.ExactOptimizationLevel: 4\n'
            if mode.endswith('700'):s=s.replace('ORBextractor.nFeatures: 800','ORBextractor.nFeatures: 700').replace('ORBextractor.nLevels: 8','ORBextractor.nLevels: 5')
            (root/'configs'/f'{data}_{mode}.yaml').write_text(s)
if __name__=='__main__':main()
