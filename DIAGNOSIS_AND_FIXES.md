# 深入诊断报告: ORB_SLAM3_DBoW3 三大问题及修复方案

## 问题背景
在 MS-SLAM 迁移到 ORB_SLAM3_DBoW3 后，立体惯性建图中出现了三类关联症状：
1. `Fail to track local map!` 频繁出现。
2. `Covisible list empty` 反复出现。
3. 新关键帧加入后，旧关键帧上的地图点似乎“被清空”或不可见。

这几个现象不是独立的，它们共享同一条控制链：关键帧连接不足会影响局部建图，局部建图又会影响局部跟踪可投影点的数量，最终触发跟踪失败。

## 已确认的根因

### 1. 共可见图在关键帧级别断开
问题的最初根因在 [src/KeyFrame.cc](src/KeyFrame.cc) 的 `UpdateConnections()`。迁移后的稀疏地图状态下，某些关键帧的有效 `MapPoint` 太少，`KFcounter` 为空后函数直接返回，导致共可见图没有建立。

这个问题的实际表现是：
- `GetBestCovisibilityKeyFrames()` 经常返回空。
- `LocalMapping::CreateNewMapPoints()` 找不到合适邻接关键帧。
- 后续三角化建点不足，局部地图维持在很弱的状态。

### 2. 局部跟踪失败的直接触发点在投影/视锥阶段
后续调试把问题进一步缩小到 [src/Tracking.cc](src/Tracking.cc) 的 `SearchLocalPoints()`。最新日志显示，局部地图点数量并不低，但经常出现：

```text
nToMatch=0! All local map points out of frustum!
```

这说明失败点不是最终的优化步，而是更早的视锥/投影筛选阶段。换句话说，局部地图里有点，但当前位姿下这些点没有通过 `isInFrustum()`，所以根本没有进入匹配阶段。

### 3. `Covisible list empty` 不是假象，而是图和邻接关系真的断过
这个日志最初反映的是共可见连接空缺，不只是打印问题。后续通过 fallback 让图能 bootstrap 起来后，这条日志已经不再是主要阻塞点。

### 4. 地图点“消失”更像是局部可视范围和稀疏化生命周期问题
目前没有证据表明每次新关键帧都会真正删除前一关键帧的全部地图点。更合理的解释是：
- 地图稀疏化会清理一部分坏点和描述子。
- 局部跟踪只展示当前视锥内可见的局部点。
- 当位姿或投影筛选偏严时，旧关键帧上的点会“看起来像消失了”。

## 已实施的修复

### 1. `UpdateConnections()` 增加 fallback
在 [src/KeyFrame.cc](src/KeyFrame.cc) 中，当 `KFcounter` 为空时，不再直接放弃连接，而是：
- 优先尝试 `mPrevKF` / `mNextKF`。
- 如果仍然没有邻接关键帧，再回退到最近的关键帧集合。

目的很直接：保证即使共享观测太少，共可见图也能先连起来。

### 2. `CreateNewMapPoints()` 增加 fallback
在 [src/LocalMapping.cc](src/LocalMapping.cc) 中，当 `GetBestCovisibilityKeyFrames()` 为空时，改为：
- 先尝试时间邻居。
- 再尝试最近关键帧集合。

这样局部建图不会因为共可见列表空而完全停摆。

### 3. `Tracking.cc` 增加更细的诊断输出
在 [src/Tracking.cc](src/Tracking.cc) 里增加了多处调试信息，重点看：
- `mvpLocalMapPoints.size()`
- 有效 `MapPoint` 数量
- `TrackLocalMap stats`
- `SearchLocalPoints` 的 `nToMatch`
- 重定位后局部跟踪的拒绝原因

同时把最近重定位后的跟踪门槛从 50 适当放宽到 25，避免在恢复阶段过早拒绝。

### 4. 修复了 `SearchLocalPoints()` 的语法问题
之前在插入调试代码时，`SearchLocalPoints()` 里少了一个闭合花括号，导致编译错误。这个已修正并重新通过编译。

## 当前验证结果

短时运行日志显示：
- 共可见图已经不再长期空着。
- `mvpLocalKeyFrames` 和 `mvpLocalMapPoints` 会逐步增长。
- `TrackLocalMap stats` 里 `aux1` 和 `mnMatchesInliers` 常常并不低。
- 但 `SearchLocalPoints()` 仍然经常给出 `nToMatch=0`。

这意味着当前主瓶颈已经从“图不连通”转移到了“投影/视锥筛选过严，导致局部点进不了匹配阶段”。

## 新增确认（2026-05-12）

### A. 导致崩溃/异常退化的高优先级回归点

对比 MS-SLAM 与当前 ORB_SLAM3_DBoW3 后，新增确认了两处高优先级回归：

1. `GetFeaturesInArea()` 并发访问风险（已修复）
	- 现象：历史 gdb 栈显示 `ORBmatcher::Fuse -> KeyFrame::GetFeaturesInArea` 段错误。
	- 根因：`KeyFrame::EraseBadDescriptor()` 在稀疏化后会释放 `mGrid`，而 `GetFeaturesInArea()` 原实现未加 `mMutexFeatures` 锁，存在并发访问窗口。
	- 影响：可表现为随机段错误、跟踪状态污染、后续退出阶段崩溃。

2. `UpdateConnections()` fallback 过激（已修复）
	- 现象：日志反复出现 `KFcounter empty`、`SearchLocalPoints: nToMatch=0`，随后跟踪退化并新建 map。
	- 根因：空连接时曾回退到“尝试连接所有现存 KF”，这会把非局部/几何不相关关键帧引入共视图，污染局部地图候选。
	- 影响：局部地图点虽然数量不低，但可投影点显著下降，出现 `All local map points out of frustum`。

### B. 已落地修复清单

已在代码中完成如下最小修复（可编译通过）：

1. `include/KeyFrame.h`
	- `mMutexFeatures` 改为 `mutable`，允许在 `const` 查询函数里加锁。

2. `src/KeyFrame.cc`
	- `GetFeaturesInArea()` 入口增加 `mMutexFeatures` 互斥保护，避免稀疏化与匹配并发冲突。
	- `UpdateConnections()` 增加与 MS-SLAM 对齐的保守逻辑：
	  - 若 `mbSparsified` 且 `KFcounter` 为空，先复用已有连接权重。
	  - 若仍为空，仅使用时间邻接（`mPrevKF/mNextKF`）做 fallback，不再“连接全部 KF”。
	  - fallback 分支会同步更新 `mConnectedKeyFrameWeights`，保持图结构自洽。

### C. 对三个现象的对应解释（更新）

1. 特征点集中在下半图
	- 更接近“可投影/可跟踪地图点分布异常”的结果，不是 ORB 提取器本体失效。
	- 由局部图污染 + 可见性筛选收缩共同导致，系统偏向近处/地面短时点。

2. 建图中频繁跟丢并新建 map
	- 直接链路：共视图连接异常 -> 局部地图候选偏离当前视角 -> `nToMatch` 变小甚至为 0 -> TrackLocalMap 失败。

3. 中断建图后保存阶段段错误
	- 高概率不是 `SaveAtlas` 逻辑单点故障，而是运行期结构破坏（并发/越界）在 shutdown/save 期显化。
	- 本次已优先修复最关键的并发访问缺口与过激连接策略。

## 结论
目前可以把问题拆成两层：
1. 之前的共可见图断裂问题已通过 fallback 基本打通。
2. 现在真正卡住局部跟踪的是 `SearchLocalPoints()` 的可见性筛选，而不是最后的 inlier 阈值。

## 后续排查方向
下一步最值得继续查的是：
- 当前位姿是否在重定位后仍然偏差较大。
- `isInFrustum()` 是否过于严格。
- 相机内参、基线或深度/尺度参数是否和 bag 实际数据不匹配。
- 是否需要在初始化和恢复阶段进一步放宽局部点搜索策略。

## 涉及文件
- [src/KeyFrame.cc](src/KeyFrame.cc)
- [src/LocalMapping.cc](src/LocalMapping.cc)
- [src/Tracking.cc](src/Tracking.cc)

## 参考测试
```bash
cd /home/ywl/Project/ORB_SLAM3_DBoW3
timeout 40 ./run_stereo_inertial.sh mapping stereo 2>&1 | grep -E "DEBUG Tracking|Fail to track local map|Relocalization: accepted|TrackLocalMap stats|TrackLocalMap rejected"
```
