# euroc_modules

F=Good Feature Matching; H=legacy MIH+OTS with common synchronization; G=fixed-cardinality Good Graph.

| mode | complete/total | tracking pass | API mean ms | API P95 ms | API P99 ms | tail25 P95 ms | ATE m* | F active / skipped | G selected | OTS calls |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | 1/1 | 1/1 | 26.157 | 36.569 | 57.313 | 43.408 | 0.0336 | 0/0 | 0 | 0 |
| f | 1/1 | 1/1 | 31.467 | 41.430 | 63.884 | 43.426 | 0.0326 | 2396/111 | 0 | 0 |
| h | 1/1 | 1/1 | 24.926 | 38.369 | 54.692 | 44.188 | 0.0345 | 0/0 | 0 | 337 |
| g | 1/1 | 1/1 | 26.016 | 35.317 | 50.323 | 39.222 | 0.0338 | 0/0 | 192 | 0 |
| fh | 1/1 | 1/1 | 26.138 | 40.441 | 55.320 | 43.049 | 0.0335 | 218/132 | 0 | 332 |
| fg | 1/1 | 1/1 | 29.984 | 39.959 | 50.791 | 42.672 | 0.0394 | 2403/94 | 218 | 0 |
| hg | 1/1 | 1/1 | 24.938 | 37.947 | 50.886 | 41.741 | 0.0325 | 0/0 | 183 | 310 |
| fhg | 1/1 | 1/1 | 25.290 | 37.703 | 46.374 | 41.857 | 0.2913 | 503/176 | 155 | 365 |

* ATE only from complete loss-free runs. Timing retains failed runs and cannot prove gains for failed groups. TUM GT covers only about 24% of frames; relative gates are not full-trajectory accuracy acceptance.
