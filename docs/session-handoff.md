# GF02 current handoff — 2026-09-28

Active feature GF02; source implementation and bounded fixtures pass, formal performance/quality matrix is running. Production LM01 remains blocked/default off.

- Isolated root: `/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_paper_20260928`, source `upstream/`, branch `repro/paper-buckets-20260928`, commit `33e506481bf4daf8bf37b1f4377950c28a8c54a7`.
- GF01 `39e794a` source, all frozen sources and binaries preserved by SHA256 audit. IDE entry commit `4a99b5e7` was clean.
- Four builds: baseline, mih_ots (legacy buckets + common synchronization fixes), paper_ots (bounded/recent/migrating buckets), paper_all (also query every descriptor).
- Release + focused ASan/UBSan/LSan + TSan fixtures pass including partial eviction/migration. Accuracy helper 3 tests pass. Four 400-frame smoke runs complete with no loss, real bucket max=10 in repaired variants; queries were not yet triggered.
- Full matrices running sequentially: `python3 harness/run_matrix.py --dataset V1_01_easy --repeats 3 --tag euroc_paper`, then equivalent `--dataset tum_large --tag tum_paper`. Do not rebuild or change frozen runner/driver/config/binaries during these runs.
- Timing includes complete Track API and separately measured pre-postprocessing pose-ready boundary. Neither is ROS Callback latency.
- Artifact: `artifacts/gf_orb_slam2_paper_20260928`; DESIGN, README, ORB2_VS_ORB3, source/build manifests, patches, harness and fixture evidence already archived.
- TODO: finish 24 formal rounds, analyze failed runs without excluding their existence, ATE/RPE/GT coverage, capacity/CPU/BA; freeze audit, plots, final REPORT, and evidence commit.
- TODO: full-system lifecycle/concurrency is not certified by isolated index tests; retained differences in OTS and update timing prevent an exact-paper claim. No board deployment or map files.
