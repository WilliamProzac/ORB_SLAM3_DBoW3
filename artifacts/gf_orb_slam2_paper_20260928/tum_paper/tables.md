# tum_paper

| 模式 | 无丢跟踪 | API 平均 ms | API P95 ms | API P99 ms | 后25% P95 ms | 位姿就绪平均 ms | 位姿就绪P95 ms | ATE m | 1s RPE m / ° |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | 3/3 | 19.824 | 27.550 | 37.898 | 28.832 | 19.299 | 26.820 | 1.06411 | 0.09651 / 1.2222 |
| mih_ots | 3/3 | 20.790 | 32.291 | 38.365 | 33.495 | 18.287 | 24.507 | 1.07930 | 0.08938 / 1.1981 |
| paper_ots† | 2/3 | 23.392 | 52.368 | 71.438 | 55.539 | 21.847 | 51.307 | 1.05567 | 0.14030 / 1.6712 |
| paper_all | 3/3 | 26.067 | 59.477 | 84.223 | 58.862 | 24.690 | 58.330 | 0.91054 | 0.08087 / 1.0340 |

| 模式 | 局部MP平均 | Local Map ms | 更新+搜索 ms | 优化统计 ms | 后处理 ms | RSS峰值 MiB | 平均CPU核 | 结束时桶条目 | 最大桶 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | 2268.542 | 6.902 | 4.648 | 2.235 | 0.419 | 1171.012 | 1.542 | — | — |
| mih_ots | 820.964 | 5.541 | 3.662 | 1.859 | 2.393 | 1384.270 | 1.665 | 6262848 | 11703 |
| paper_ots | 1181.226 | 9.374 | 7.401 | 1.965 | 1.405 | 1255.074 | 1.656 | 79695 | 10 |
| paper_all | 1405.490 | 11.516 | 9.465 | 2.043 | 1.267 | 1193.098 | 1.680 | 80041 | 10 |

† marks a configuration with tracking failures: its raw timing includes LOST frames and is not evidence of acceleration. All timing and quality numbers above are medians of per-run aggregates. Failed runs remain in timing; accuracy uses only eligible runs. RSS is the median of per-run sampled peaks; maximum bucket is the maximum of end-of-run diagnostics.
