# GF04：官方 ORB-SLAM3 与已交付加速版对照

本轮以官方 ORB-SLAM3 `4452a3c4` 为主 baseline。交付版使用已核验的
core `4d6df6a2` / outer `18ba684`，不使用实验 MEM18/perf20/LM01 或 IDE XFeat。
原仓库源码不修改；本机官方树仅有未跟踪 build_log.txt，保留且登记。
IDE 起点 `fa8395a5` 工作区干净，不存在需先保存的源码改动。

隔离目录 `/home/ywl/Project/RDK_X5_orb_slam/orb_slam3_controls_20260928`。
由 Git 对象导出源码，与交付归档 src/include 逐文件比对；独立完整重建。
统一 GCC、Release -O3 -march=native、OpenCV 4.2、原生 CPU API 驱动。
官方算法保持原样；通过仅授予诊断访问权限的 friend 声明记录 Frame/Atlas，
官方构建脚本 OpenCV 4.4 下限以独立 CMake 适配现有 4.2，记录此构建差异。

每组每数据仅一轮：official800、delivered800、delivered700，另跑 GF03
冻结 baseline 的桥接，共 8 轮。EuRoC V1_01_easy 2912 帧单目；TUM
freiburg2_large_with_loop 5057 对 RGB-D。配置均保留相同标定/词典树。
800/8 对比版本；交付配置 700/5 单列，不将降低特征数收益归因于代码。
交付版均使用 3 ORB workers，RGB-D L4；官方原提取线程机制。
1×原始时间戳，不跳帧；Track API 全调用计时，图像解码、诊断输出在计时外，
积压包含解码等调用间开销。OpenCV/BLAS/OpenMP 线程数 1，显式算法线程保留。
Viewer、回环、自动地图保存关闭，无板卡部署。loop-off 在这些项目中均会
抑制常规 KF 数据库填充，故本轮不验收重定位/回环。

编译后每个实际目标短测，再冻结源码、配置、驱动、库/可执行文件哈希。
正式测试不与编译并行。记录全输入/初始化后/固定后25%的 API 分位数、
deadline 超时、输入积压、状态、地图数量和资源；失败跟踪耗时不当作收益。
ATE/RPE 为质量防线：单目单次全局 Sim(3)，RGB-D SE(3)，不逐地图对齐。
TUM GT 仅覆盖约24.3%，保留精度结论边界。单轮不声称统计显著。

GF03 8组合旧结果保留，增加相对官方 baseline 的参考表；桥接有明显漂移
则明确旧表为历史参考，不能当同批严格对照。不覆写旧实验或旧 baseline。
交付源/二进制、工具、机器可读结果、报告及 Git 提交作为最终证据。
