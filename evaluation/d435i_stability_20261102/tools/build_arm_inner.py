#!/usr/bin/env python3
"""RGBD36: full cold ARM64 publication-layout snapshot, audited RGBD25 flags/sysroot."""
from pathlib import Path
import subprocess,shlex,json,hashlib,re,sys,os,time,resource
from concurrent.futures import ThreadPoolExecutor
root=Path('/ros1_project/orb_slam'); core=Path('/ros1_project/orb_slam3_core'); out=Path('/output')
cache=Path('/cache/build/orb_slam3_ros');sysroot=Path('/cache/experiments/rgbd_fifo_20260911/build/sysroot')
compiler='/opt/arm-gnu-toolchain-11.3.rel1-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++'
commands=[]
started_wall_ns=time.time_ns();started_mono_ns=time.monotonic_ns()
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def path(a):return re.sub(r'/usr/(?:include|lib|local)|/opt/ros|/lib/aarch64-linux-gnu',lambda m:str(sysroot)+m.group(),a)
common=['-B'+str(sysroot/'usr/lib/aarch64-linux-gnu')+'/', '--sysroot='+str(sysroot),'-nostdinc++','-isystem',str(sysroot/'usr/include/c++/11'),'-isystem',str(sysroot/'usr/include/aarch64-linux-gnu/c++/11'),'-isystem',str(sysroot/'usr/include/aarch64-linux-gnu')]
def run(cmd):commands.append(cmd);subprocess.run(cmd,cwd=cache,check=True)
def compile(target,source,obj,extra=()):
 flags=dict(l.split(' = ',1) for l in (cache/f'CMakeFiles/{target}.dir/flags.make').read_text().splitlines() if ' = ' in l)
 opts=[path(a.replace('/ros1_project/orb_slam/devel/include','/cache/devel/include').replace('/ros1_project/orb_slam/tools/rdk_bpu_sysroot','/cache/tools/rdk_bpu_sysroot').replace('/ros1_project/orb_slam3_core',str(core))) for a in shlex.split(' '.join(flags[k] for k in ['CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS']))]
 assert '-DREGISTER_TIMES' not in opts
 run([compiler,*common,*opts,*extra,*(['-fPIE'] if target=='ros_rgbd' else []),'-c',str(source),'-o',str(obj)])
def link(target,objects,dest):
 args=shlex.split((cache/f'CMakeFiles/{target}.dir/link.txt').read_text());args[0]=compiler;new=[]
 for i,a in enumerate(args):
  if a.endswith('.o'):continue
  if i and args[i-1]=='-o':a=str(out/dest)
  elif a.startswith('-Wl,-rpath,'):a='-Wl,-rpath,$ORIGIN:$ORIGIN/lib'
  elif a.endswith('/liborb_slam3_ros.so'):a=str(out/'liborb_slam3_ros.so')
  else:a=path(a.replace('/ros1_project/orb_slam/src/orb_slam3_ros-master/orb_slam3/Thirdparty','/cache/src/orb_slam3_ros-master/orb_slam3/Thirdparty'))
  new.append(a)
 new[1:1]=common+['-Wl,--as-needed,--enable-new-dtags']+(['-pie'] if target=='ros_rgbd' else [])
 new[1:1]=[str(p) for p in objects]
 new += ['-L'+str(sysroot/'usr/lib/aarch64-linux-gnu'),str(sysroot/'usr/lib/aarch64-linux-gnu/libstdc++.so.6')]
 if dest == 'rgbd_system_init_allocation_probe': new += ['-ldl']
 for d in ['usr/lib/aarch64-linux-gnu','opt/ros/noetic/lib','usr/lib/aarch64-linux-gnu/blas','usr/lib/aarch64-linux-gnu/lapack']:new+=['-Wl,-rpath-link,'+str(sysroot/d)]
 run(new)

sources=sorted((core/'src').glob('*.cc'))+sorted((core/'src').glob('*.cpp'))+sorted((core/'src/CameraModels').glob('*.cpp'))
inputs=sources+sorted((core/'include').rglob('*.h'))+sorted((root/'src/orb_slam3_ros-master/include').rglob('*.h'))+sorted((root/'diagnostics').rglob('*.h'))+sorted((root/'memory').rglob('*.h'))+[root/'src/orb_slam3_ros-master/src/ros_rgbd.cc',root/'src/orb_slam3_ros-master/src/common.cc']
expected={str(p):sha(p) for p in inputs}
node_sources=[root/f'src/orb_slam3_ros-master/src/{name}.cc' for name in ['ros_rgbd','common']]
node_objects=[out/(s.stem+'.o') for s in node_sources]
objects=[out/(s.stem+'.o') for s in sources]
assert len(objects)==len(set(objects))
request=json.loads((out/'build_request.json').read_text())
assert len(sources)==request['expected_core_compiles'] and len(node_sources)==2, (len(sources),len(node_sources))
def compile_one(job):
 target,s,obj=job
 print('COMPILE',s,flush=True);compile(target,s,obj)
# Independent translation units only; retain the exact compiler options and
# deterministic object/link order. The container retains its 8 GiB ceiling.
jobs=[('ros_rgbd',s,o) for s,o in zip(node_sources,node_objects)]+[('orb_slam3_ros',s,o) for s,o in zip(sources,objects)]
with ThreadPoolExecutor(max_workers=2) as workers:
 list(workers.map(compile_one,jobs))
link('orb_slam3_ros',objects,'liborb_slam3_ros.so')
link('ros_rgbd',node_objects,'ros_rgbd')
for name,h in expected.items():assert sha(Path(name))==h,name
publication_inputs={}
for name,digest in expected.items():
 p=Path(name)
 relative=str(p.relative_to(core)) if str(p).startswith(str(core)+'/') else 'runtime/rgbd/'+str(p.relative_to(root))
 publication_inputs[relative]=digest
assert publication_inputs==request['publication_inputs'], 'container input identity mismatch'
(out/'manifest.json').write_text(json.dumps({'publication_inputs':publication_inputs,'commands':commands,'inputs':expected,'core_compiles':len(sources),'node_compiles':len(node_sources),'node_objects_reused':0,'build_kind':'full_cold','started_wall_ns':started_wall_ns,'ended_wall_ns':time.time_ns(),'elapsed_s':(time.monotonic_ns()-started_mono_ns)/1e9,'compiler_children_user_cpu_s':resource.getrusage(resource.RUSAGE_CHILDREN).ru_utime,'compiler_children_system_cpu_s':resource.getrusage(resource.RUSAGE_CHILDREN).ru_stime,'cached_core_objects_reused':0,'input_stability_verified':True,'binaries':{name:sha(out/name) for name in ['ros_rgbd','liborb_slam3_ros.so']}},indent=2)+'\n')
print('BUILD_COMPLETE',len(sources),flush=True)
