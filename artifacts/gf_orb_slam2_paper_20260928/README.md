# GF02 paper-semantics experiment

Isolated checkout: `upstream/`, branch `repro/paper-buckets-20260928`, parent `39e794af2ec4ba8c8fd2665a748de5551e35b161` (the frozen GF01 CPU reproduction).

This experiment restores bounded/recent hash buckets, descriptor migration and (in a separate variant) all-descriptor queries. It is not a claim of exact reconstruction of the authors' 2019 implementation. See the archived `DESIGN.md` and `REPORT.md` in `ORB_SLAM3_DBoW3/artifacts/gf_orb_slam2_paper_20260928` for scope and results.

Run commands from this directory:

```bash
cmake -S harness -B build
cmake --build build --target replay_baseline replay_mih_ots replay_paper_ots replay_paper_all test_hash_index -j4
./build/test_hash_index
bash harness/run_index_sanitizers.sh
python3 harness/test_accuracy.py
python3 harness/run_matrix.py --dataset V1_01_easy --repeats 1 --max-frames 400 --tag smoke
python3 harness/run_matrix.py --dataset V1_01_easy --repeats 3 --tag euroc_paper
python3 harness/run_matrix.py --dataset tum_large --repeats 3 --tag tum_paper
python3 harness/analyze.py runs/euroc_paper
python3 harness/analyze.py runs/tum_paper
python3 harness/export_summary.py runs/euroc_paper runs/tum_paper
python3 harness/plot_results.py runs/euroc_paper
python3 harness/plot_results.py runs/tum_paper
```

Matrix tags must be new directories; existing results are never overwritten. Run SLAM processes sequentially and finish compilation/sanitizers before timing. Inputs remain on the host and no maps are saved. Main vocabulary is `/home/ywl/Project/ORB_SLAM3_DBoW3/Vocabulary/ORBvoc.txt`. `inputs/` contains absolute local dataset filenames. `ORB_Data` is a read-only-use symlink to GF01's frozen official pose/config checkout. The host dependencies match GF01 (Ubuntu 20.04/GCC 9/OpenCV 4.2/Eigen/Pangolin/Armadillo/Boost).

`baseline` is the GF fork with hashing disabled, not canonical ORB-SLAM2. `mih_ots` is the previous unbounded/no-refresh policy plus the same index/handoff synchronization fixes as the candidates. `paper_ots` bounds buckets at 10 and refreshes revisited points, retaining unmatched-descriptor queries. `paper_all` additionally queries all current-frame descriptors. All hashed variants retain 32 index tables, at most 8 queried tables, target logDet 80, Combined covisibility intersection and the existing trigger/time budget. Loop closure, GFM, Good Graph, GPU and external priors are disabled in every mode.

`track_ms` measures the complete public API. `pose_ready_ms` measures GrabImage entry to the boundary preceding realtime pose file output and postprocessing. This is not ROS image-to-publication latency; the API returns after postprocessing. Background index insertion no longer writes into Tracking's per-frame timing structure. Do not interpret its old insertion-time column as full maintenance cost. Index diagnostic `migrations` and `evictions` count table entries; `insertions`/`refreshes` count point update calls; `queries` counts descriptor queries. `indexed_points` describes the new policy's reverse-membership table, and is zero/inapplicable in legacy mode.

Sanitizer fixtures compile the actual Hashing.cc with opaque, never-dereferenced identity tokens and throwing MapPoint stubs. They cover the new index algorithm/concurrency, not full SLAM lifecycle safety. In ptrace-based sandboxes LeakSanitizer may refuse to run; use a normal host process. No sanitizer binaries are used for performance measurements.

Original source GPL licensing remains in `upstream/`. Revert by selecting the frozen GF01 checkout/build; the production ORB3 checkout and RDK-X5 deployment were not changed.
