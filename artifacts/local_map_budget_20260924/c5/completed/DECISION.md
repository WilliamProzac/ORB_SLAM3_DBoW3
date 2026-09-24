# C5 完成结论


## LM01 C5b — completed MP/statistics and tier screen

All six immutable board rounds failed input completeness and realtime:
legacy80 2436/2910, nonOK35, P95=133.027ms; attempted80 2358,35,134.134ms;
MP80 2443,0,133.633ms; adaptive96 2158,0,155.713ms; adaptive80 2446,70,136.946ms;
adaptive64 2434,0,137.711ms. Fixed-tail P95 ranges137.412–161.387ms.
The5000/6500/8000 post-filter MP limits never bind (eligible maximum2025).
All measured second/third unique-KF observations and2/3/5-KF survival deltas
are within1 percentage point of legacy80; the conditional10% young-reservation
long replay is not triggered. Its allocation/feedback mechanics passed the
purpose-built64-MP stress fixture. Different-KF/adaptive comparisons are
screening diagnostics, not isolation of a single causal change.

Cohort waiting time now distinguishes actual event wall time from dataset
progress; observing an older KF during fusion cannot turn a later observation
into a zero wait. Six analyzer tests pass; all native/board cohorts were
recomputed without changing count/survival gates. Core algorithm and tested
binaries are unchanged. Source core05e83e55, outer base9feff3c; implementation
C5a core906d870f/outer9188413. C6 matrix remains pending. DefaultOFF.
Evidence: IDE artifacts/local_map_budget_20260924/c5/completed/{board_matrix,native_analysis_v2}.json.
Conditional query-before-gather MIH and independently budgeted BA work is
specified in memory/LM01_FOLLOWUP.md; neither is implemented or accepted.
