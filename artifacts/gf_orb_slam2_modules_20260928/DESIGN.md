# GF03: existing-module factorial experiment

Parent source 33e5064; upstream 2a60498. GF01/GF02 remain frozen.

Eight modes: baseline, f, h, g, fh, fg, hg, fhg. F=Good Feature Matching; H=existing MIH+OTS (legacy bucket storage with GF02 common synchronization); G=Good Graph fixed cardinality 30, candidate cap 60. Existing selection mathematics retained. Paper bucket repair is not silently mixed into this factorial. Dynamic graph budget / external trajectory prediction disabled (uncalibrated host cost model and external inputs); report this scope explicitly.

Same 800 extracted ORB features / 8 levels / calibration / vocabulary; native full API and pose-ready boundaries; no viewer, GPU, external pose prior, loop closure or map serialization. F uses 400 scalar-constraint target (about 200 monocular 2-D matches), author time cap 15ms; F can also skip local tracking when upstream matches already exceed the constraint target; both skip and active selection are counted. Its budget affects keyframe insertion in the existing implementation and is part of the measured module. Same driver value across modes; only F consumes it. G/F branch execution is measured, not inferred from macros. No matcher/tracking threshold edits.

First build all eight and smoke test each; fix only compatibility and required integration defects with evidence. Then EuRoC V1_01_easy 2912 images and TUM fr2_large_with_loop 5057 RGB-D pairs, all eight, three repeats, sequential at 1x; cyclic/reversed order to distribute temporal bias. No simultaneous compilation or benchmark.

Report all runs including failures: full-count/sequence integrity, non-OK, mean/P95/P99/max and fixed tail25, internal pose-ready, local matching, actual F/H/G activation, graph size/selection and BA cost, input-schedule lateness, CPU/RSS/Swap, ATE and 1-second RPE. EuRoC one global Sim3; TUM SE3, ground-truth coverage/gap explicit. A useful candidate requires all repeats loss-free, >=5% full-API P95 gain without >5% P99 regression and accuracy median degradation <=5% (near-zero 5mm/0.1degree tolerance); report conflicting datasets rather than claim general effectiveness. Off builds rerun as controls.

Source/build/config/input/driver hashes frozen before timing. No board or generated maps. Keep local raw evidence; archive compact machine results and SHA256 lists to IDE repository. Save stage commits with limitations.
