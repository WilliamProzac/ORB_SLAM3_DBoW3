# tum_modules

F=Good Feature Matching; H=legacy MIH+OTS with common synchronization; G=fixed-cardinality Good Graph.

| mode | complete/total | tracking pass | API mean ms | API P95 ms | API P99 ms | tail25 P95 ms | ATE m* | F active / skipped | G selected | OTS calls |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | 1/1 | 1/1 | 19.278 | 26.534 | 38.904 | 30.985 | 0.7464 | 0/0 | 0 | 0 |
| f | 1/1 | 1/1 | 22.934 | 32.320 | 41.059 | 32.050 | 0.9380 | 3351/426 | 0 | 0 |
| h | 1/1 | 1/1 | 19.632 | 31.217 | 37.744 | 31.506 | 1.3497 | 0/0 | 0 | 890 |
| g | 1/1 | 1/1 | 18.875 | 25.538 | 34.028 | 27.175 | 0.9454 | 0/0 | 181 | 0 |
| fh | 1/1 | 1/1 | 20.236 | 30.786 | 38.124 | 30.496 | 0.6201 | 1141/356 | 0 | 546 |
| fg | 1/1 | 1/1 | 22.273 | 31.459 | 36.732 | 31.861 | 1.1041 | 3215/290 | 163 | 0 |
| hg | 1/1 | 1/1 | 19.336 | 30.897 | 35.935 | 31.666 | 1.1237 | 0/0 | 195 | 776 |
| fhg | 1/1 | 0/1 | 18.468 | 29.768 | 36.572 | 26.841 | — | 959/293 | 166 | 391 |

* ATE only from complete loss-free runs. Timing retains failed runs and cannot prove gains for failed groups. TUM GT covers only about 24% of frames; relative gates are not full-trajectory accuracy acceptance.
