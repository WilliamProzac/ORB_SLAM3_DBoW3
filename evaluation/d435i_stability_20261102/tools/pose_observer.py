#!/usr/bin/env python3
"""Observe both pose topics independently; source identity is always original stamp."""
import argparse
import csv
import json
import math
from pathlib import Path
import signal
import threading
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--index',type=Path,required=True)
    parser.add_argument('--namespace',default='/x5_8g_test')
    args=parser.parse_args()
    with args.index.open() as f:source={int(r['source_stamp_ns']) for r in csv.DictReader(f)}
    import rospy
    from geometry_msgs.msg import PoseStamped
    rospy.init_node('stability_pose_observer',anonymous=True,disable_signals=True)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:rospy.signal_shutdown('round complete'))
    args.output.mkdir(parents=True,exist_ok=True)
    files={name:(args.output/(name+'.csv')).open('x',newline='') for name in ('raw','continuous')}
    writers={name:csv.writer(f) for name,f in files.items()}
    for writer in writers.values():writer.writerow(['stamp_ns','host_steady_s','frame','x','y','z','qx','qy','qz','qw'])
    lock=threading.Lock();closing=False;errors=[];seen={name:set() for name in files};counts={name:0 for name in files}
    def record(message,name):
        with lock:
            if closing:return
            try:
                stamp=message.header.stamp.to_nsec();p=message.pose.position;q=message.pose.orientation
                values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                if stamp not in source or stamp in seen[name] or not all(math.isfinite(v) for v in values):
                    errors.append(dict(stream=name,stamp_ns=str(stamp),reason='identity_duplicate_or_nonfinite'))
                if counts[name]>=2*len(source):raise ValueError('pose observation overflow')
                writers[name].writerow([stamp,time.monotonic(),message.header.frame_id]+values)
                seen[name].add(stamp);counts[name]+=1
                if counts[name]%64==0:files[name].flush()
            except Exception as error:errors.append(dict(stream=name,error=repr(error)))
    subscribers=[rospy.Subscriber(args.namespace+'/'+topic,PoseStamped,record,name,queue_size=1000)
                 for topic,name in [('robot_pose_slam','raw'),('robot_pose','continuous')]]
    ready=False
    try:
        deadline=time.monotonic()+25
        while not rospy.is_shutdown() and not all(s.get_num_connections() for s in subscribers):
            if time.monotonic()>=deadline:raise RuntimeError('both pose publishers did not connect')
            time.sleep(.05)
        if rospy.is_shutdown():raise RuntimeError('shutdown before pose consumer readiness')
        (args.output/'ready').write_text('both_publishers_connected\n');ready=True
        rospy.spin()
    except BaseException as error:errors.append(dict(error=repr(error)))
    finally:
        with lock:closing=True
        for subscriber in subscribers:subscriber.unregister()
        for f in files.values():f.close()
        status=dict(ready=ready,errors=errors,records=counts,unique_stamps={k:len(v) for k,v in seen.items()},
            normal_spin_return=ready and not errors,scope='Two independent subscriptions; final fresh-identity completeness assessed by reducer, not inferred from finite topic outputs')
        (args.output/'status.json').write_text(json.dumps(status,indent=2)+'\n')
    return 0 if ready and not errors else 3


if __name__=='__main__':raise SystemExit(main())
