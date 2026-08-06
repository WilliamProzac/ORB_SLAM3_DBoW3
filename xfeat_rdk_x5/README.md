# XFeat on RDK X5

This directory contains the local reproduction and the first conversion stage
for running the official XFeat model on the RDK X5 Bayes-e BPU.

## Source pin

- Upstream: `https://github.com/verlab/accelerated_features.git`
- Local checkout: `../Thirdparty/accelerated_features/`
- Tested revision: `e92685f57f8318b18725c5c8c0bd28c7fe188d9a`
- License: Apache-2.0 (see the upstream `LICENSE`)

The upstream checkout is intentionally kept separate from the deployment code.

## Local CPU reproduction

Python 3.8 is supported by the pinned dependencies. A virtual environment is
preferred. On hosts without the `python3-venv` package, an isolated target
directory also works:

```bash
python3 -m pip install \
  --target /tmp/xfeat-rdk-x5-pydeps \
  -r xfeat_rdk_x5/requirements-local.txt

PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps \
python3 xfeat_rdk_x5/smoke_test.py \
  --output xfeat_rdk_x5/artifacts/local_smoke.json
```

The smoke test runs the bundled XFeat weights on the two upstream sample images,
checks finite 64-D unit descriptors and mutual matches, and records warmed CPU
latency. It does not require CUDA or Kornia.

## Fixed-shape ONNX core

```bash
PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps \
python3 xfeat_rdk_x5/export_onnx.py

PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps \
python3 xfeat_rdk_x5/onnx_smoke_test.py
```

The exported graph uses opset 11 and a fixed `1x1x480x640` float32 NCHW input.
It outputs:

- `dense_descriptors`: `1x64x60x80`
- `keypoint_logits`: `1x65x60x80`
- `reliability`: `1x1x60x80`

The input must already be grayscale and per-image InstanceNorm-normalized with
`eps=1e-5`. This normalization is moved to CPU because the official RDK X5
operator table marks `InstanceNormalization` as CPU computation. Sparse NMS,
thresholding, top-k selection, descriptor `grid_sample`, and MNN matching also
remain CPU-side. This split avoids putting dynamic-length `NonZero` output into
the BPU graph and exposes a conventional keypoint/descriptor list to a future
adapter; it is not a drop-in replacement for this tree's binary ORB front end.
The exporter also folds the upstream 8x8 image unfold and its following 1x1
convolution into one mathematically equivalent 8x8 stride-8 convolution, which
removes hundreds of Slice/Transpose nodes from the deployment graph.

## RDK X5 INT8 conversion and board validation

The fixed `640x480` core was converted and run successfully on 2026-07-31 with:

- host image: `openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.6`
- converter: `hb_mapper 1.23.8`, HBDK `3.49.14`
- board runtime: DNN `1.23.10`, HBRT `3.15.54`, BPU platform `1.3.6`
- board: `root@192.168.1.10`

Prepare 100 evenly spaced left-infrared calibration tensors from the bag. The
manifest must stay outside the calibration directory because `hb_mapper` treats
every file in that directory as tensor data:

```bash
PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages \
python3 xfeat_rdk_x5/prepare_calibration.py \
  2026-06-05-09-59-04.bag \
  xfeat_rdk_x5/calibration_data/bag_2026-06-05-09-59-04_480x640 \
  --rosbag-topic /camera/infra1/image_raw --limit 100 \
  --manifest xfeat_rdk_x5/artifacts/calibration_bag_2026-06-05-09-59-04_480x640_manifest.json
```

Run PTQ from the repository root:

```bash
docker run --rm --shm-size=8g \
  -v "$PWD/xfeat_rdk_x5:/data" -w /data \
  openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.6 \
  hb_mapper makertbin --model-type onnx \
  --config config/xfeat_backbone_480x640_bayes_e.yaml
```

`hb_mapper checker` assigned all 33 reported nodes to one BPU subgraph. The O3
compiler estimate was `2.0061 ms`, while actual board execution through the DNN
API measured `13.473 ms` mean over 50 runs after 10 warmups. The generated model
is 920,712 bytes with SHA-256
`0ff4a43f8b9085c56da2173969f9e42989c79cb6c296ce484d0eb7360a693dea`.

The board runner source and CMake project are under `rdk_x5/`. It is deliberately
small and links against the board's installed `libdnn` and `libhbmem.so.1`:

```bash
cmake -S /userdata/xfeat_rdk_x5/runner \
      -B /userdata/xfeat_rdk_x5/build -DCMAKE_BUILD_TYPE=Release
cmake --build /userdata/xfeat_rdk_x5/build -j2

LD_LIBRARY_PATH=/usr/hobot/lib:/usr/lib/aarch64-linux-gnu \
/userdata/xfeat_rdk_x5/build/xfeat_dnn_runner \
  /userdata/xfeat_rdk_x5/xfeat_backbone_480x640_bayes_e.bin \
  /userdata/xfeat_rdk_x5/0050.f32 \
  /userdata/xfeat_rdk_x5/dnn_output_0050 50 10 0
```

The three board outputs are bit-identical to the corresponding dequantized
`hb_mapper infer` outputs. Against the float ONNX output on the same frame, the
cosine similarities are `0.995747` for dense descriptors, `0.990869` for
keypoint logits, and `0.997176` for reliability. Machine-readable evidence is
in `artifacts/ptq_board_validation_480x640.json`.

This first result proves the fixed `640x480` convolutional core. The subsequent
native-height end-to-end work below adds the complete CPU sparse frontend and
uses the bag's actual `640x544` image size.

## Complete board frontend and ROS LAN benchmark

`rdk_x5/xfeat_frontend.cpp` implements per-image normalization, persistent DNN
tensor allocation, BPU execution, sparse heatmap decoding, 5x5 NMS,
reliability-weighted top-k selection, bicubic 64D descriptor sampling, and
mutual-nearest-neighbour matching. `rdk_x5/ros_benchmark.cpp` runs that frontend
and an OpenCV ORB baseline on synchronized ROS stereo images.

Deploy and build natively on the board:

```bash
./xfeat_rdk_x5/deploy_board_benchmark.sh
```

Run the host ROS master, board node, and host bag replay in separate terminals:

```bash
./xfeat_rdk_x5/host_ros_lan.sh master
ssh root@192.168.1.10 \
  /userdata/xfeat_rdk_x5/benchmark_runner/run_ros_benchmark.sh
./xfeat_rdk_x5/host_ros_lan.sh play \
  2026-06-05-09-59-04.bag --delay=1.0
```

The deployed runner now defaults to the pure sparse control profile: 600
features, at most 600 guided coarse matches, `semidense_single=false`, empty
Fine model path, and both extraction/match grid quotas disabled. Pass explicit
ROS private-parameter overrides when running an XFeat* Fine experiment; do not
assume the runner's sparse defaults describe the M384 candidate.

The scripts configure `ROS_MASTER_URI=http://192.168.1.3:11311`, host
`ROS_IP=192.168.1.3`, and board `ROS_IP=192.168.1.10`. The completed full-bag
replay produced the expected 130 stereo plus 65 temporal pairs per method with
no missing sampled anchor. XFeat's median single-image extraction was 60.00 ms
versus ORB's 66.34 ms, while means were 64.85 ms and 68.67 ms. Including two
extractions, matching, and geometry, mean stereo-pair latency was 185.63 ms for
XFeat and 161.98 ms for ORB.

XFeat produced 8.9% more raw stereo matches and 48.3% more strict-inlier spatial
coverage, but 18.4% fewer strict stereo inliers. It produced 25.2% fewer strict
temporal inliers while retaining 51.2% more temporal coverage. See
`docs/xfeat-rdk-x5-board-benchmark.md` and
`artifacts/board_ros_benchmark_544x640/summary.json` for the full distributions,
tracking-state slices, exact model identifiers, and limitations.

### Parallel stereo follow-up

The board exposes only BPU0; a real `HB_BPU_CORE_1` inference fails and Linux
has no `bpu_core1` device. The current benchmark therefore uses two independent
left/right frontend buffers and worker threads, serializes only BPU0, and
overlaps it with multithreaded CPU preprocessing/postprocessing. ORB left/right
extraction is parallelized with two independent instances as well.

With eight CPU workers, full-bag XFeat stereo extraction wall time is 57.78 ms
versus parallel ORB's 74.48 ms. XFeat stereo end-to-end improves from 185.63 to
125.46 ms, but remains above ORB's 99.38 ms because float matching and RANSAC
are more expensive. All 390 quality records exactly match the serial result.
See `docs/xfeat-rdk-x5-parallel-benchmark.md` and
`artifacts/board_ros_benchmark_parallel_544x640/summary.json`.

### XFeat* semi-dense plus fixed-size Fine Matcher

The F21 candidate follows the upstream XFeat* path rather than refining
already pixel-localized sparse keypoints. `evaluate_fine_matcher.py` calls the
upstream dense extractor, keeps descriptor scales, constructs Fine Matcher
inputs in target/fixed order, and applies the predicted offset times the target
scale. `rdk_x5/xfeat_frontend.cpp` implements the deployable single-scale form:
the dense API now returns each selected cell's real reliability instead of a
placeholder score. Selection uses two deterministic grid-quota stages. First,
the complete stride-8 map is divided into 8x6 cells and contributes at most
`ceil(top_k/48)` points per cell. Then geometry-guided matches contribute at
most 16 pairs per left-image cell before one fixed-size Fine Matcher
inference refines right-image coordinates. Candidates are taken in balanced
per-cell rounds, with reliability-weighted match ranking inside each round.

Exported Fine variants use fixed inputs `1x256x128`, `1x384x128`, and
`1x512x128`. Their Bayes-e binaries are deployed alongside the backbone by
`deploy_board_benchmark.sh`. Start a board M384 run by adding these private
parameters to the benchmark command:

```bash
_top_k:=600 \
_xfeat_semidense_single:=true \
_fine_model_path:=/userdata/xfeat_rdk_x5/xfeat_fine_matcher_m384_bayes_e.bin \
_xfeat_fixed_matches:=384 \
_xfeat_grid_columns:=8 \
_xfeat_grid_rows:=6 \
_xfeat_extraction_grid_maximum_per_cell:=-1 \
_xfeat_grid_maximum_per_cell:=16 \
_xfeat_fine_confidence:=0.20 \
_minimum_patch_ncc:=0.5
```

`xfeat_extraction_grid_maximum_per_cell=-1` selects the automatic
`ceil(top_k/(columns*rows))` extraction quota. Zero disables the extraction
quota. `xfeat_grid_maximum_per_cell=0` disables the match quota.

The 9x9 patch-NCC metric is deliberately reported outside
`end_to_end_pair_ms`. Rectified single-scale semi-dense points begin on common
grid rows, so epipolar RANSAC can incorrectly make coarse correspondences look
almost perfect; NCC checks whether the selected disparity actually aligns
local image content.

The frozen F21 profile passed five alternating full-bag board runs at
54.63/59.40 ms mean/P90 versus production ORB's 70.50/81.83 ms. Its board
NCC-qualified count was 212.31 versus 215.46 production ORB valid depths. A
common real-ORB-SLAM3/XFeat NCC+PnP evaluator also passed all geometric gates.
See `docs/f21-gate-report.md` and `artifacts/f21_gate_final.json`.

To evaluate whether stereo correspondences survive into motion estimation,
run the common stereo-depth/Fine/PnP diagnostic:

```bash
PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps:/opt/ros/noetic/lib/python3/dist-packages \
python3 xfeat_rdk_x5/evaluate_stereo_vo.py \
  2026-06-05-09-59-04.bag \
  --output-dir xfeat_rdk_x5/artifacts/f21_stereo_vo_full
```

This uses identical 65 frame pairs and identical PnP settings for hard-grid
XFeat+Fine, no-hard-grid XFeat+Fine, and an OpenCV ORB control. The recorded
hard-grid policy was rejected: it returned 99.38 mean PnP inliers, one
24-inlier fragile pair, and 64/65 successful estimates. The no-hard-grid
XFeat+Fine control returned 213.0 mean inliers and 65/65 success, versus
200.25 and 65/65 for ORB. Use board evidence, not this FP32 Python program's
`extract_pair_ms`, for latency decisions.

### Same-frame feature distribution visualization

Build the helper that invokes the real ORB-SLAM3 `Frame` stereo constructor
and `ComputeStereoMatches`, then render frame 320 with the current XFeat* M384
candidate:

```bash
chmod +x xfeat_rdk_x5/build_visualization_helper.sh
./xfeat_rdk_x5/build_visualization_helper.sh
PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps:/opt/ros/noetic/lib/python3/dist-packages \
python3 xfeat_rdk_x5/visualize_feature_distribution.py \
  2026-06-05-09-59-04.bag --frame-index 320
```

The top row overlays valid/strict stereo observations on the same left image;
point color is disparity. The bottom row reports the number of observations in
each 8x6 grid cell. This separates raw detections from stereo-usable points and
shows that Fine Matcher refines the right coordinate but does not redistribute
the fixed left-image XFeat* proposals; spatial balancing must therefore happen
in extraction and coarse-candidate selection before Fine Matcher.

## Offline ROS bag evaluation

The evaluator reads synchronized infrared images directly from a ROS1 bag,
without starting a ROS master, and compares XFeat against an OpenCV ORB
baseline on identical frames:

```bash
PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps:/opt/ros/noetic/lib/python3/dist-packages \
OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 \
python3 xfeat_rdk_x5/evaluate_rosbag.py \
  2026-06-05-09-59-04.bag
```

By default it samples a pair of consecutive frames every ten source frames.
That preserves 10 Hz temporal motion for tracking metrics while covering the
whole recording. Outputs include JSON summaries, per-pair CSV metrics, and
geometry-inlier match visualizations under `artifacts/bag_*/`.
