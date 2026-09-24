#!/usr/bin/env bash
# Host entrypoint: isolated, finite live-camera validation; no source/core rebuild.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
duration=${1:-120}
timing=${2:-off}
[[ $duration =~ ^[0-9]+$ ]] && ((duration >= 10 && duration <= 1800)) || { echo 'duration must be 10..1800 seconds' >&2; exit 2; }
[[ $timing == off || $timing == on ]] || { echo 'timing must be off or on' >&2; exit 2; }
run="$(date +%Y%m%d_%H%M%S)_${timing}_$RANDOM"
remote="/userdata/d435i_slam_20260924/$run"
local_run="$root/artifacts/d435i_slam_20260924/$run"
mkdir -p "$root/artifacts/d435i_slam_20260924"
mkdir "$local_run"
ssh_opts=(-o BatchMode=yes -o ConnectTimeout=8)
ssh "${ssh_opts[@]}" sunrise@192.168.1.11 "set -e; if [ ! -e /userdata/d435i_slam_20260924 ]; then sudo -n install -d -o sunrise -g sunrise -m 775 /userdata/d435i_slam_20260924; fi; mkdir '$remote'"
scp -q "${ssh_opts[@]}" "$root/evaluation/d435i_slam.py" "$root/evaluation/rgbd_recorder.py" \
  "$root/artifacts/rdk_x5_8g_20260923/tools/resource_sample.py" "sunrise@192.168.1.11:$remote/"
rc=0
ssh "${ssh_opts[@]}" sunrise@192.168.1.11 \
  "source /opt/ros/noetic/setup.bash && exec /usr/bin/python3 '$remote/d435i_slam.py' --run-dir '$remote' --duration '$duration' --timing '$timing'" \
  | tee "$local_run/runner.log" || rc=$?
# Always fetch failed-run evidence, without deleting anything on the board.
ssh "${ssh_opts[@]}" sunrise@192.168.1.11 \
  "/usr/bin/python3 '$remote/d435i_slam.py' --run-dir '$remote' --finalize"
rsync -a -e 'ssh -o BatchMode=yes -o ConnectTimeout=8' "sunrise@192.168.1.11:$remote/" "$local_run/"
(cd "$local_run" && sha256sum -c SHA256SUMS)
echo "Evidence: $local_run"
exit "$rc"
