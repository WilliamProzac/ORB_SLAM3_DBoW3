
## LM01 C5 implementation contracts (validation in progress)

The MP cap counts only deduplicated new candidates after entry-skip, frustum and
far-distance filtering. Existing valid associations stay outside the budget.
The attempted policy commits unique A-union-E visible opportunities and final
inlier found increments; retries retain the same policy. Legacy remains the
KF-only default. No historical counters or backend culling thresholds change.

The selector uses undistorted24x16 cells and camera-Z layers at mThDepth and
3*mThDepth. Grid occupancy includes protected associations; available depth
layers share capacity and empty quotas recycle. Valid unique KF observations
are counted from relationship snapshots (bounded at8), never Observations().
Score weights .50/.20/.20/.10 use smoothed found ratio, min(validKF,8)/8,
exp(-associationFrameAge/30) and normalized view cosine. Association history is
runtime-only, updated only by the selected final attempt before-outlier
associations; unknown history contributes0 and is not called successful tracking.
Candidates retain matcher order. A nonbinding cap avoids scoring entirely.
Optional young reservation is at most floor(cap/10), creation-frame age<=30 and
<3 valid unique KF observations. It is an ablation, not enabled by default.

Independent scalar lifecycle events record births, observation changes, reasoned
retirement, replacement and map transfer. ORB_LM_LIFETIME_FILE opts in before
System construction; no culling threshold or lifetime counter is fabricated.
The analyzer uses actual committed same-map KF progress, excluding imported
historical KFs; raw KF-ID exit age is reported separately. Transfers and
uncompleted tail horizons are censored. Unique observation milestones are not
credited before their KF is committed. Selection marginals separately aggregate
creation-frame-age [0,30),[30,150),>=150, grid and depth; discarded attempts are
excluded from final selection/success rates. All enabled logging costs remain
inside the timed processing path. Commit lock waits are separately measured.

Native120 four variants and2910 four variants complete all input, stateOK.
The MP6500 native long cap never binds: eligible maximum1439, gathered maximum
6168. It cannot explain any timing improvement. A64-candidate stress shadow
exercises the real budget; it is not a proposed deployment setting. Realpoint
fixtures verify score normalization, stereo-count distinction, grid/depth fair
allocation, young reservation/recycling and hard-cap/discard-retry parity.
Lifecycle fixture checks1170 events and concurrent observation updates. Its
first revision incorrectly demanded successful commit after a KF transfer;
negative logs retained, corrected test requires rejection of cross-map results.

ARM full paired build and27 inherited probes plus new budget/lifetime fixtures
pass; graph max difference7.1e-15. Full TSan race tests and lifetime race fixture
pass without race suppressions. Small transaction lock-order fixture passes
with only existing GDAL startup deadlock exceptions; full fixture disables the
Clang10 64-lock-limited deadlock detector, not race detection. ASan/UBSan/LSan fixtures also pass with the documented Boost-only UBSan
exceptions. Board six long ablations and C6 final matrix are still running.
Do not promote any configuration. Inherited full-System LSan remains open.

Native long newborn2/3 observations and2/3/5-KF survival all stay within5pp of
KF-only legacy; worst observed drop is0.122pp. Shadow weighted overall/new
recall99.959/99.877%, actual association99.936%, new recallP598.708%.
First C5 board legacy80 long fails:2436/2910,nonOK35,P95133.027ms,tail137.412ms.
C5a is a functional checkpoint; no quality/performance promotion is implied.
C6 harnesses/configs are prepared; their formal outcomes are still pending.
