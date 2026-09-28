# 对照版本和计时边界

| 项目 | 官方 ORB-SLAM3 | 交付加速 ORB-SLAM3 | GF03 原 baseline |
|---|---|---|---|
| 来源 | UZ-SLAMLab/ORB_SLAM3 `4452a3c4` | core `4d6df6a2` / outer `18ba684` | GF03 `1ad2246`，F/H/G 全关闭 |
| 词袋 | DBoW2 | DBoW3 | DBoW2 |
| 相同参数组 | 800 特征、8 层 | 800 特征、8 层 | 800 特征、8 层 |
| 交付配置组 | 无 | 700 特征、5 层 | 无 |
| ORB 提取 | 原线程机制 | 3 workers | GF 原线程机制 |
| RGB-D 专用优化 | 无交付版优化 | L1 网格查询、L2 距离计算、L3 描述子快照、L4 无 Viewer 路径 | 无交付版优化 |
| IMU | 不输入 | 不输入 | 不输入 |
| 回环/合并 | 本次关闭 | 本次关闭 | 旧构建 DISABLE_LOOP_CLOSURE |
| 地图生命周期 | 官方原实现 | 当时交付实现 | GF 原实现 |
| LM01 / MEM18 / perf20 | 无 | 无 | 无 |

交付版 L4 配置只在纯 RGB-D 构造分支读取；单目不会得到 L4 优化。
两种交付配置都明确设置 3 workers，因此 800/8 vs 700/5 用于观察配置差异；
官方 vs 交付版是整版本比较，不能将结果全部归因于单个线程池或单项 L4。
交付源码 src/include 的 69 项与原交付归档逐字节一致后才导出测试适配。
归档中 `source/current/core` 是实际 core，嵌套 ROS 目录中的旧 core 不参与构建。

三项一致性约束：

1. 两版 ORB3 使用同一份已核验的 replay.cc、相同编译工具链和 OpenCV 4.2，
   实际库通过 ldd 核验位于本轮隔离构建目录。官方的构建文件原要求 OpenCV4.4，
   本轮独立 CMake 使用现有4.2；未声称复现官方原始二进制。
2. 统一从调用 Track API 前到返回后计时。输入时间戳按1×调度，全部图像依次提交，
   不跳帧。PNG 解码、诊断 CSV 和轨迹输出在计时之外；积压指标包含这些调用间开销。
   词典加载、由文本生成 DBoW3 缓存、Shutdown 也不计入逐帧时间。
3. 原版及交付版均关闭 Viewer、回环、地图自动保存；GF 桥接原二进制也关闭回环。
   OpenCV/BLAS/OpenMP/MKL 单线程，算法自身显式线程仍保留。未调整主机调频策略。

## 必要的关闭回环适配

初始未修补短测中，交付版单目和官方版 RGB-D 都发生 SEGV。
两个 core 的崩溃 PC 经符号解析均为 `LoopClosing::Run()`；官方 PC
`liborb3_official.so+0x1eeebd` 位于对 `mpLastCurrentKF` 的候选列表读取。
源码中 `NewDetectCommonRegions()` 在 `!mbActiveLC` 时先返回，未为
`mpCurrentKF` 赋值；Run 仍执行 `mpLastCurrentKF = mpCurrentKF`，下次迭代
可能解引用未初始化地址。交付单目在 GDB 下未复现，不能据此排除该未定义行为。

两版仅将 Run 中的 `if (CheckNewKeyFrames())` 改为
`if (mbActiveLC && CheckNewKeyFrames())`。未停止整个 loop 线程，因为 reset
仍需它响应；未修改 Tracking、LocalMapping、优化器或匹配器。另在 System.h
授予诊断访问权限，不改变类内数据布局。修改前后源码哈希、最小补丁、崩溃
栈文本与全部失败轮保留。此适配不提供新的重定位数据库填充功能。

本轮严格范围是「官方/交付源码 + 相同关闭回环基准适配」的主机跟踪建图对照。
其成功结果不等同于官方完整回环系统、RDK-X5 ROS 或长期生命周期验收。
