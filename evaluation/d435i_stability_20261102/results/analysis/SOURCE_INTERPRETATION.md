# 新增源码观测的解释边界

本文只解释冻结 RGBD36 源码与观测口径，不分析正在进行的试验结果，不预判参数或 eager BoW 的收益。实际效果须等全部回放结束，核对输入覆盖、诊断扰动和重复轮次后判断。

## 匹配、内点与快照

| 字段 | 可以解释什么 | 不能直接解释什么 |
| --- | --- | --- |
| `motion_first/final`、`reference_coarse` | 原生搜索得到的初始关联数量，运动模型扩大窗口是否增加关联 | 优化后的几何正确性；数量增加不等于可靠性提高 |
| `motion_map/reference_map` | 各自优化并排除外点后，关联点 `Observations()>0` 的槽位数 | 全局位姿精度；它们也不是最终局部跟踪内点 |
| `local_points`、`search_seen/bad/projected/visible/matches` | 从局部点集合、跳过、实际视锥测试，到新增关联的数量变化 | 投影失败的细分原因、描述子拒绝原因；`projected` 是尝试数，不是全部成功投影 |
| `local_inliers/local_threshold` | 局部优化后参与成功判定的原生计数与实际源码门槛 | 没调用局部跟踪时的内点；这些帧为 `NA`，不能沿用上一帧数值 |
| `pose_calls/pose_last_*` | 优化方法调用次数及本帧最后一次调用的前后关联/外点数 | 求解次数；`pose_last_after` 仍包括外点，不是内点数，也不能替代逐阶段统计 |

上述地图匹配与局部内点按关联槽位计数，不是地图点全集大小或全局唯一点计数。局部内点还要求该点有观测；地图点的加权观测数不是关键帧数量，RGB-D 有有效虚拟右像素时一次观测加2，否则加1。纯 RGB-D 正常门槛30，重定位保护窗50；非保护窗、源码状态为 `RECENTLY_LOST` 时，另有 `>10` 提前成功分支，最小值11，不能概括为永远只有30/50。[计数与门槛](../checkout/src/Tracking.cc#L3777)、[观测加权](../checkout/src/MapPoint.cc#L217)。

窗口 `kind=1` 取实际 `isInFrustum` 写出的局部搜索投影；`kind=2/3` 保存优化调用前后的真实关联、外点标记、边界位姿和当时地图点重投影，附 `camera_z/input_depth`（米）。它们适合看关联分布、距离和外点集中区域；后台可能改变地图点位置，因此不能当优化器内部固定边的逐轮残差或 chi²。`stage` 区分运动模型、参考、局部与重定位。[实现](../checkout/src/StabilityDiagnostics.cc#L132)。

输入 `frame_id/input_keypoints` 在 `Track()` 前捕获；清图后 `current_frame_valid=0` 只表示结束时的当前 Frame 已被替换，输入身份仍有效。精确源时间和序号来自 ROS header，不由 double 时间戳反推。[快照入口](../checkout/src/StabilityDiagnostics.cc#L102)。

## 关键帧与点生命周期

`kf_tracked_close/untracked_close/need_close` 直接说明近点专用条件是否满足；`kf_ref_matches/c1a/c1b/c1c/c2/mapper_idle` 说明普通比例、帧间隔和后台接收条件。近点触发不成立不等于不插关键帧。`kf_queue` 仅在原代码实际查询队列的分支有值；提前返回未计算的字段为 `NA`，不能解释成条件为假。[原生条件](../checkout/src/Tracking.cc#L3903)。

`kf_depth_slots` 是后续关键帧排序深度循环处理的槽位数，包含已有点；`kf_new_points` 才是该创建过程实际新生点数。初始化遍历全部正深度建立种子点，未执行此限槽循环，故初始化槽位数为 `NA`。两阶段的高低不能直接解释为持续补点能力衰退。[后续创建](../checkout/src/Tracking.cc#L4068)。

生命周期记录可区分出生、替换、清图和不同退出原因。`found/visible` 是退出时已有计数，`extra` 为加权观测数；KF龄是 ID 差，没有墙钟寿命。近期点 `found/visible<0.25` 或 KF龄至少2且观测数不超过3会被剔除；KF龄至少3时“毕业”仅退出近期检查表，**不删除地图点**。记录未细分 Local BA 的每条删边原因，不能把所有删观测都归为同一种退化。[近期点剔除](../checkout/src/LocalMapping.cc#L554)。

## eager BoW 修复与性能

原路径先将新关键帧发布到 Atlas 或 Tracking 参考，再由 LocalMapping 出队计算 BoW。Tracking 可能在后台计算前读取空向量，也可能在计算写入时无锁读取。开启 eager 后，在初始化和普通关键帧的最早发布前完成真实 `ComputeBoW()`；正常非空 ORB 词袋就绪后，后台重复调用只检查容器，不再改写。默认开关为0，便于同一 binary 对照；空/float 描述子的原语义保留。[两个发布前入口](../checkout/src/Tracking.cc#L2894)、[普通创建入口](../checkout/src/Tracking.cc#L4068)、[原后台计算](../checkout/src/LocalMapping.cc#L488)。

此修复会把部分 BoW 工作移到前台关键帧回调，也会改变后台排队和线程时序。`bow_prepares` 是准备方法调用次数，未必每次真的执行词典变换；`bow_prepare_ms` 是调用耗时。即使减少空 BoW，也不能预先推断总回调更快或所有失跟消失。诊断开关本身还增加计数、锁和窗口重投影，须独立评估其扰动。

## 局部点变多、内点变少与 ThDepth

局部集合增大只说明候选库存增大。可先沿链路区分：更多点被 seen/bad 跳过；投影可见比例下降；可见点新增匹配比例下降；已有匹配在优化中更多成为外点；或关键帧补点、点存活与当前视野需求不匹配。阶段计数与窗口能定位数量损失发生在哪一段，不能单独证明模糊、旋转、遮挡、描述子混淆、错误深度或位姿预测中的某一项为根因。

`mThDepth=(bf/fx)×ThDepth` 是近远点分界，用于近点插帧条件和关键帧补点策略，**不是全链路最大深度截断**。例如基线 `bf/fx=0.05m` 时，ThDepth40对应2m，80对应4m。后续关键帧创建会保留全部近点，并在近点不足时继续处理较近的远点；由于处理后才检查 `depth>mThDepth && nPoints>100`，不能写成“最多新建100点”。增加 ThDepth 会同时改变近点统计、插帧与补点负载，会纳入更多较远距离点，其稳定性须由观测判断。[换算](../checkout/src/Tracking.cc#L1436)、[槽位循环](../checkout/src/Tracking.cc#L4121)。

最后，`summary.complete` 只表示 CSV 刷盘成功。必须分别检查输入覆盖、各类 drops、每窗口 stored/dropped 与 `workers_joined`。寿命明细满后只保留前缀，原因聚合继续累计；坐标缺失或 `NA` 不能当零。原始深度65535也不能仅凭取值认定无效。[停止与输出](../checkout/src/StabilityDiagnostics.cc#L170)。
