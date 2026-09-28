# euroc_paper

| 模式 | 无丢跟踪 | API 平均 ms | API P95 ms | API P99 ms | 后25% P95 ms | 位姿就绪平均 ms | 位姿就绪P95 ms | ATE m | 1s RPE m / ° |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | 3/3 | 26.603 | 35.609 | 54.829 | 40.458 | 26.258 | 35.142 | 0.03329 | 0.04388 / 0.4913 |
| mih_ots | 3/3 | 25.737 | 38.627 | 55.341 | 43.731 | 24.070 | 32.310 | 0.03333 | 0.04461 / 0.5154 |
| paper_ots | 3/3 | 30.724 | 63.127 | 104.591 | 84.123 | 29.732 | 61.060 | 0.03331 | 0.04378 / 0.4898 |
| paper_all | 3/3 | 31.871 | 64.263 | 104.289 | 83.179 | 30.911 | 62.987 | 0.03306 | 0.04387 / 0.4902 |

| 模式 | 局部MP平均 | Local Map ms | 更新+搜索 ms | 优化统计 ms | 后处理 ms | RSS峰值 MiB | 平均CPU核 | 结束时桶条目 | 最大桶 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | 3739.917 | 8.993 | 7.112 | 1.872 | 0.252 | 735.203 | 1.384 | — | — |
| mih_ots | 690.394 | 6.769 | 5.318 | 1.443 | 1.565 | 769.332 | 1.421 | 1752480 | 5343 |
| paper_ots | 2118.676 | 12.674 | 10.972 | 1.695 | 0.901 | 742.504 | 1.496 | 80314 | 10 |
| paper_all | 2331.913 | 13.764 | 12.101 | 1.655 | 0.876 | 750.598 | 1.515 | 80368 | 10 |

† marks a configuration with tracking failures: its raw timing includes LOST frames and is not evidence of acceleration. All timing and quality numbers above are medians of per-run aggregates. Failed runs remain in timing; accuracy uses only eligible runs. RSS is the median of per-run sampled peaks; maximum bucket is the maximum of end-of-run diagnostics.
