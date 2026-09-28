# GF03：论文模块与现有实现逐项对照

核对对象为官方 `catkin` 提交 `2a60498cb889965bd5b4ab05c918ab28b433e882` 的本地派生版本 `1ad2246b2aac4357a2990c80ced84a41ef6d0d64`。以下源码路径相对于 `upstream/`；行号对应这个冻结版本。已有 GF01/GF02 的编译适配、观测与公共同步修复保留，不能称为未经修改的官方二进制。

三个模块的核心代码都存在。判断必须区分“源码存在”“本轮编译启用”“运行实际进入”和“带来有效收益”。第三项由运行日志核实，第四项由完整输入、延时及精度共同判断。本轮不以开启宏代替运行证据。

## F：Good Feature Matching

论文以加权测量雅可比的信息量指导主动匹配，并采用随机化贪心减少选择成本；单纯匹配后再删点无法省下已经发生的数据关联工作。参见 [Good Feature Matching，2020](https://arxiv.org/pdf/2001.00714)。以下实现细节来自冻结源码核对。

| 对照项 | 现有实现位置 | 本轮处理及边界 |
|---|---|---|
| 信息矩阵构造 | `src/Tracking.cc:2731`、`:2916`，`src/Observability.cc` 的 `runMatrixBuilding` | 使用现有 `USE_INFO_MATRIX` 为候选构造信息；7 维状态模型保持原样，未重写数值模型 |
| 已有匹配信息先验 | `src/Tracking.cc:2754`、`:2785`，`include/Tracking.h:125` | `FRAME_MATCHING_INFO_PRIOR` 代码存在，但原默认注释且本轮未启用；主动选择从小对角正则项开始，不能说本轮已纳入全部已有匹配信息 |
| Max-logDet 评分 | `src/Observability.cc:956`，`logDet(curMat + ObsMat)` | 已实现，不是按响应值取 Top-N |
| 随机候选、贪心主动匹配 | `src/Observability.cc:831` 的 `runActiveMapMatching` | 随机抽样、选择候选后调用单点投影匹配，成功才累加匹配信息；该分支有实际进入计数 |
| 时间和匹配数量限制 | `src/Tracking.cc:2805`，`src/Observability.cc:948` | 现有局部搜索总预算 15 ms；driver 设 400 个标量约束。单目每个成功匹配主要计 2，带深度可能计 3，因此不是统一 400 个 MP |
| 初始化/恢复和小集合回退 | `src/Tracking.cc:2902` | 最近重定位、初始化期、可见候选不足 400 时走现有普通投影匹配；F 开启不代表每帧执行主动选择 |
| 已有匹配足够则跳过局部跟踪 | `src/Tracking.cc:912` | 保留，并单独统计 `gfm_skip_local`。速度变化不能全部归因于主动选点 |
| 关键帧补充匹配 | `src/Tracking.cc:1217` 附近 | `DELAYED_MAP_MATCHING` 保留，在 pose-ready 之后仍可能发生；计入完整 API 耗时 |
| RGB-D 关键帧规则 | `src/Tracking.cc:2432` | 启用 F 会改变近点条件，属于现有模块的实际作用；同时报告 KF/MP 和 BA 变化 |
| 跨帧预计算 | `src/Tracking.cc:1136` 附近 | `PRECOMPUTE_WITH_MOTION_MODEL` 本轮全部关闭；未声称验证该并发变体 |

新增两个布尔诊断字段只记录主动分支/跳过分支，不替换算法。原有 `lmk_num_BA` 在跳过局部跟踪时可能为零，不能直接解释为该帧没有有效跟踪点；另记录最终 Frame 的有效关联数量。后者也不等同于优化器内点数，可能包含后处理新关联。

`WITH_OCT_LEVELED_NOISE` 保留开启，投影/质量/稳健权重扩展未启用。矩阵构建仍使用原生多线程（硬件并发数），并未因为 BLAS/OpenMP 设为单线程而被强制串行。这些是本轮固定配置，不等同于穷举 F 的所有可选变体。

## H：Local Map Hashing 与在线选表

论文提出用外观先验约束共视局部地图，通过 MIH 检索和在线选表降低候选及查询开销。参见 [Appearance-Enhanced Local Map Building，2019](https://arxiv.org/pdf/1905.07797)。以下列出当前实现与此次配置，而非认定已精确复现论文所有细节。

| 对照项 | 现有实现位置 | 本轮处理及边界 |
|---|---|---|
| ORB 分段索引与查询 | `src/Hashing.cc:469`、`:839` | 32 张表；初始/最多 8 张活动表，保留既有查询接口及公共同步修复 |
| 共视与外观集合结合 | `src/Tracking.cc:3530` 的 `UpdateLocalPointsByHashing` | `Combined` 路径先扫描共视 KF 点槽，再查询并取交集；不能消除前面的共视扫描成本 |
| 查询输入 | `src/Tracking.cc:3615` | 使用 `GetUnMatchedKPbyBucketing`，不是每个描述子都查询；GF02 的 `GF_PAPER_ALL_DESCRIPTORS` 未启用 |
| 小地图回退 | `src/Tracking.cc:3588` 附近 | 共视点少于 2000 返回共视路径；短测试可能编译启用而没有真正触发 H |
| OTS 信息构造 | `src/Tracking.cc:3753` 的 `UpdateQueryNumByHashTable` | 根据本帧有效候选匹配及其分段相等关系形成每表信息，不是空壳函数 |
| 贪心选表及重叠扣除 | `src/Tracking.cc:3905`、`:3954` | `logDet` 比较，选定一表后扣除重叠点贡献；保留作者实现，未做新的数学等价证明 |
| 停止条件和提交 | `src/Tracking.cc:3982`、`:4000` | 最多 8 表，信息阈值 80，时间不足可不提交新列表；分别统计调用与完成次数 |
| OTS 调用时机 | `src/Tracking.cc:1187` | 关键帧补充匹配后、剩余时间允许时调用；耗时在完整 API 内，通常在 pose-ready 之后 |
| 桶容量、最近性和迁移 | `src/Hashing.cc:123`、`:469` | 本轮保留 legacy 桶策略；GF02 的有界桶/刷新/迁移实验未混入。已知名义容量不等同于实际存储硬上限 |

因此不能再把“没有 OTS”作为当前开源版本与论文效果不同的解释。更准确的待验证差异是查询范围、桶行为、信息构造与调用时序。GF02 对其中部分差异的修补没有证明整体更快，本轮专门固定 H 的这一版本来比较模块组合。

## G：Good Graph

论文针对 BA 子图，在保持条件性的目标下选择较小图，并结合未来估计需求分配计算预算。参见 [Good Graph to Optimize，2020](https://arxiv.org/pdf/2008.10123)。当前源码既有固定规模分支，也有预期需求和动态预算代码；它们的验证状态不同。

| 对照项 | 现有实现位置 | 本轮处理及边界 |
|---|---|---|
| BA 候选图构建 | `src/Optimizer.cc:615` 起，`include/Optimizer.h:44` | 原作者候选上限 60；默认目标 30 个自由 KF |
| Schur 相机系统 | `Thirdparty/SLAM++/include/slam/NonlinearSolver_GoodGraph.h:978` 起 | 现有 SLAM++ 实现，未用简单“最近 N 个 KF”替代 |
| 随机贪心选子图 | `src/Optimizer.cc:1156`，`NonlinearSolver_GoodGraph.h:1054` | `Find_Subgraph(...,1,...)` 进入 lazier selection；记录触发、返回和分阶段耗时 |
| 必保留节点与选择后重建 | `src/Optimizer.cc:1174` 起 | 将所选 ID 映射回 KF，重新收集点及固定相机，再交给 g2o |
| 固定规模控制 | `src/Optimizer.cc:1131` 起 | 本轮测试此分支：大于 30 个自由 KF 时触发选择 |
| 未来虚拟帧与可见性 | `src/Optimizer.cc:927`、`src/Tracking.cc:2561` | `ENABLE_ANTICIPATION_IN_GRAPH` 代码存在，本轮未启用 |
| 时间预算预测与规模反解 | `src/Optimizer.cc:1011`、`:1123`，`estimateKFNum` | `ENABLE_ANTICIPATION_IN_BUDGET` 代码存在；需要硬件耗时模型和运动预测标定，本轮未验证 |
| 单目/RGB-D 几何 | `src/Optimizer.cc:911`，后续 g2o 边构造 | 选图阶段使用现有二维投影信息；最终 RGB-D BA 保留深度约束。上游标注未完成的 stereo SLAM++ 变体关闭 |
| 构建依赖 | `Thirdparty/SLAM++/CMakeLists.txt`，`harness/CMakeLists.txt` | 修正 Eigen/Armadillo 路径；原 Eigen BLAS/LAPACK 包装器混用旧/新 Eigen 构建失败，改链接系统 BLAS/LAPACK，保留选图代码 |

所以本轮 G 的结论只能覆盖“现有固定规模 Good Graph”，不能外推到论文完整动态预算策略，也不能将动态策略未测试说成源码缺失。

## 组合与解释规则

| 模式 | F | H | G | 必须看到的运行证据 |
|---|---|---|---|---|
| baseline | 关 | 关 | 关 | 三类增强计数为零 |
| f | 开 | 关 | 关 | 主动匹配/跳过局部跟踪计数 |
| h | 关 | 开 | 关 | 哈希查询、OTS 调用与完成 |
| g | 关 | 关 | 开 | Good Graph 触发、返回、选后 BA 规模 |
| fh | 开 | 开 | 关 | F/H 均实际进入，注意 F 跳过局部跟踪会减少 H 机会 |
| fg | 开 | 关 | 开 | F/G 均实际进入，注意 F 改变关键帧和 BA 负载 |
| hg | 关 | 开 | 开 | H/G 均实际进入，注意地图变化反过来改变查询集合 |
| fhg | 开 | 开 | 开 | 三者均实际进入；收益不能按单模块百分比相加 |

两套数据每个模式一次是用户明确要求。各数据内顺序均为上表顺序，不能再称为三轮交错对照；时间漂移、线程调度、随机贪心和地图分岔均限制单轮归因。完整全局回环、动态预算、ARM/ROS、NewCollege 长序列和完整系统并发认证不在本轮已验证范围内。

另一个需要单独核对的组合接口是 F 跳过局部跟踪之后的暂存状态：`mbMapHashTriggered/mbMapHashOTS` 在构造函数和 `UpdateLocalPointsByHashing()` 中清零，并非在每个 `Track()` 入口清零；`mLeftMapPoints` 主要在 `SearchLocalPoints()` 清理，而关键帧后处理仍可能读取剩余候选。ONLINE_TABLE_SELECTION 分支会重新打开视锥检查，因此“用了前帧候选”本身不能直接定性为错误。但它说明三个模块并不是互不影响的独立开关，后续修复前应对跳过帧和关键帧后处理做专门的状态一致性测试。本轮没有相应冻结输入消融，不能把观察到的 FHG 精度下降归因于这一个接口，也没有在正式测试中途修改它。

最终失跟诊断还确认：`src/LoopClosing.cc:72` 的 `DISABLE_LOOP_CLOSURE` 分支同时跳过了关键帧入库路径（`:138`、`:168`、`:237`），因此虽然 Tracking 打印重定位启用，本轮没有验证完整可用的重定位数据库。FHG 之后持续未恢复受此共同设置限制，首次失跟仍单独记为失败。另有三个 driver 缺少输入滞后列的构建诊断差异，详见 REPORT.md 的证据缺口；不将文件冻结成功等同于所有诊断列一致。
