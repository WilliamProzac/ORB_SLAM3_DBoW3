# GF03: three-module factorial evaluation

See DESIGN.md for frozen scope, module semantics and acceptance. Isolated root: `/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_modules_20260928`.

```bash
cmake -S upstream/Thirdparty/SLAM++ -B build_slampp -DCMAKE_BUILD_TYPE=Release -DSLAM_P_P_EIGEN33=ON -DSLAM_P_P_USE_OPENMP=ON -DSLAM_P_P_FLAT_SYSTEM_ALIGNED_MEMORY=OFF -DSLAM_P_P_LIB_TYPE=SHARED -DSLAM_P_P_BUILD_EXAMPLES=OFF -DSLAM_P_P_BUILD_MAIN_APP=OFF
cmake --build build_slampp --target slampp_base csparse cxsparse cholmod camd_dlong -j2
cmake -S harness -B build
cmake --build build --target replay_baseline replay_f replay_h replay_g replay_fh replay_fg replay_hg replay_fhg -j2
python3 harness/run_matrix.py --dataset V1_01_easy --modes fhg --repeats 1 --max-frames 1200 --tag smoke_all_euroc
python3 harness/run_matrix.py --dataset tum_large --modes fh fg hg fhg --repeats 1 --max-frames 400 --tag smoke_pairs_tum
python3 harness/freeze.py
python3 harness/run_matrix.py --dataset V1_01_easy --repeats 3 --tag euroc_modules
python3 harness/run_matrix.py --dataset tum_large --repeats 3 --tag tum_modules
python3 harness/analyze.py runs/euroc_modules
python3 harness/analyze.py runs/tum_modules
python3 harness/factorial_report.py
python3 harness/audit_freeze.py
```

No production SLAM/board changes. No maps generated. Full API latency includes optional postprocessing, unlike internal pose-ready latency. G uses existing fixed-cardinality selection and 2-D projection information (including on RGB-D); subsequent g2o BA retains RGB-D constraints. Dynamic anticipation/budget prediction and the upstream explicitly unfinished stereo SLAM++ variant are not enabled. F includes its existing local-tracking skip and RGB-D keyframe rules. H retains legacy bucket policy to isolate module effects; this is not the GF02 paper-all variant.

Compatibility: SLAM++ hard-coded Eigen/Armadillo install paths use discovered system packages. Its legacy Eigen BLAS/LAPACK wrapper mixes bundled and installed Eigen; use system BLAS/LAPACK with existing native Schur/selection implementation instead. Failed build logs retained.
