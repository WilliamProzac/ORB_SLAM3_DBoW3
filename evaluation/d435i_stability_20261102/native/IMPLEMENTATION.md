# RGBD36 核心实现与观测契约

本实现仅针对冻结 f75a3ffd 的 RGB-D 实验。修改在隔离 checkout；不复用当前主树 XFeat 源码。尚未由实现代理编译或板测，构建与真实回放由独立代理执行。

## 固定启动开关

- `ORB_RGBD_EAGER_BOW=0|1`，默认 `0`。System 创建工作线程前读取一次；进程中修改环境不会改变模式。`1` 仅在纯 RGBD 的两个关键帧创建点、任何 Atlas/reference/queue 发布之前调用真实 `KeyFrame::ComputeBoW()`；之后正常 LocalMapping 的重复调用只检查已就绪向量。没有给未同步读者追加锁，没有发布后补算。空描述子、float 描述子的既有语义保留；本实验 ORB 非空描述子才能形成非空 BoW。
- `ORB_RGBD_STABILITY_DIR=<已存在、可写的独立目录>`：未设/空表示诊断关闭；指定后启动即验证写入，失败直接报错。此开关独立于旧 FieldDiagnostics，兼容 `_scale_enabled=true`。不启用旧 `ORB_LM_LIFETIME_FILE`。
- `ORB_RGBD_STABILITY_WINDOWS=1-120,650-710`：可选，最多 64 段，闭区间；按 Scale 源索引的精确 `source_ordinal` 判断。未设时仍有逐帧计数/寿命聚合，没有坐标。重叠窗口的行计入第一个匹配窗口。

Scale wrapper 从原始 ROS header 传入 `toNSec()` 和两路 seq，保留源索引 ordinal；没有经过 double 时间戳。普通 v2 同样传精确身份；普通非索引 callback 的 ordinal=0，不能使用 ordinal 窗口。`frame_id` 与 `input_keypoints` 在真实 Frame 构造完成后、Track 之前捕获。新图重置后仍输出本次输入身份，`current_frame_valid=0` 表示 Track 结束对象已经被清空，不能把其空对象内容当本次输入。默认 Frame 仅补初始化 N=0 与 mnId=ULONG_MAX，不能推断其余标量都可读。现有 scale 的 frame_id/keypoints 也改为使用这一捕获值。

## 输出与 NA

目录写入 `stability_frames.csv`、`stability_points.csv`、`stability_lifetime.csv`、`stability_summary.json`。CSV 精确列名见源码首行。未执行阶段的整数输出 `NA`；真实执行所得 0 保留 0。`pose_calls` 是方法调用次数，不能当优化器求解次数。`pose_last_*` 是本帧最后一次 PoseOptimization 的边界计数；同一帧先参考、再局部等多调用不会冒充逐阶段完整计数，坐标窗口通过 stage/kind 保留每次快照。

`stability_frames.csv` 包括输入身份、前后状态、清空有效性、有限正深度/近远点、运动模型初始/扩大搜索/优化后地图匹配、参考粗匹配/优化后地图匹配、局部搜索 seen/bad/投影/可见/新增匹配、局部内点及实际源码门槛、关键帧 ID/处理深度槽位/新生点数、前台 BoW 次数与耗时。NeedNewKeyFrame复用原局部值保存tracked/untracked close、bNeedToInsertClose、nRefMatches、c1a/c1b/c1c/c2、mapperIdle；queue仅在原本实际执行的KeyframesInQueue处保存，不增加查询，提前返回未算值为NA。门槛纯 RGBD 正常为30、重定位保护窗50，非保护窗且源码状态 RECENTLY_LOST 的提前成功分支最小11；其他传感器写 NA。budgeted LocalMap 的独立路径没有这些新增分项，写 NA。初始化 `kf_depth_slots=NA`，因为它遍历全部正深度，未执行后续 KF 的排序限槽循环。后续槽位数包括已有点与新点，不能当新建点数。

坐标 `stage`：1运动模型、2参考、3局部、4重定位。`kind`：1局部 isInFrustum 实际结果（观察坐标/外点/距离写NA）；2 PoseOptimization 调用前；3调用后。kind2/3 保留原生匹配 MapPoint ID、实际关联 keypoint index/坐标、调用后外点标记、当前位姿和实时 MP 位置重投影、相机 z 与该输入 keypoint 的 depth（m）。这是阶段边界快照；后台可能在两次读取之间改变地图，不能当优化器内部固定边的逐轮残差或 chi²。仅采单相机 RGB-D，深度负值是原槽位未关联值，不能冒充有效深度。没有同步图片编码。

寿命 `kind`：1出生；2其他退出；3 found/visible 比值退出；4 KF龄/低观测退出；5删观测退出；6清图退出；7替换；8/9 KF加入/删除；10/11观测加入/删除；12近期检查表毕业。12不是删除地图点。记录 map_generation/map_id、point_id、KF ID；出生 value=firstFrame，退出 value=KF龄（没有cull_key则NA）、extra=加权 nObs，found/visible 是退出当时已有计数；替换 value=目标点ID、extra=原观测数；观测变化 value=变化后 nObs；KF加入 value=来源frame_id。未采墙钟寿命、未按原因拆 LocalBA outlier 的每条删边，不能声称逐点完整墙钟生命周期。ID可与源帧/KF关联。

## 有界缓冲、停止与完整性

硬上限：20000帧、262144坐标、100000寿命记录；预留值结构缓冲约33MiB（准确值 `retained_capacity_bytes`），正常本包输出约几十MB。无图像缓存、无后台持有 MapPoint/KeyFrame 裸指针。热路径不刷盘；诊断关闭不额外遍历点容器或调用重投影。

所有 append 与 StopAndFlush 使用同一 mutex；停止先关 admission，写盘期间不读仍被写的 vector，后续 append 被拒绝；重复 flush 返回第一次结果，不覆盖文件。调用前应停止 callback ingress。真正 join 路径标记 workers_joined=true；现有 scale 的 legacy Shutdown 路径标记 false，不虚构 join 或后台所有工作已完成。即使仍有后续生产者也只能追加值并被关闭入口拒绝，不能在刷盘中改 vector。

summary 的 `complete` 仅表示三份CSV成功写完，不等于无截断或后台join。必须分别检查 frames 是否覆盖5878个实际输入、各 drops、workers_joined。每个坐标窗口有 attempted/stored/dropped；坐标满后仍累计窗口缺失量。寿命满后逐点明细只剩前缀，但 `life_totals[0..12]` 仍按原因统计所有已准入事件。`closed_appends_at_flush` 仅截至停止锁取得时；后续拒绝次数不会反写已封存文件。

## 原生夹具入口

`native/stability_native.cc` 链接**本次 matching 完整核心 DSO**，不要仅编译一个替代算法或使用当前主树 DSO。依赖与已有 geometry_probe 相同（OpenCV/Eigen/Sophus/DBoW3/g2o/核心DSO/pthread）。四个模式各用新建空输出目录，返回非0表示失败：

```
stability_native disabled OUTPUT_DISABLED
stability_native off OUTPUT_OFF
stability_native on OUTPUT_ON
stability_native flush-failure OUTPUT_FAILURE
```

使用显式 Require/throw，Release/NDEBUG 不会吞检查。disabled 验证没有记录/文件；off/on 用真实 RGBD Frame+ORBextractor、小词典与真实 KeyFrame，核发布前开关、发布后并发读/重复 ComputeBoW、空/float描述子；真实 Frame reset 后检查CSV保留本次输入且current_frame_valid=0；真实 MapPoint birth/observation/retirement；并发寿命生产者和容量耗尽；生产者尚在运行时 Stop(false) 关入口，再 join，重复停止不改文件；真实路径打开失败被返回。该并发夹具验证的是发布先后关系与容器只读，不是TSan结果；没有故意在off分支制造已知数据竞争。静态核对两个真实 Tracking 发布点的 guard 同样必需。


单文档 TF-IDF 的初版夹具使全部训练词 IDF=log(1/1)=0，导致 BoW/Feat 都为空；host01 的失败保留在原归档，不能当核心 eager 修复失败。修正版把256个训练描述子分成8个32行独立文档，保持真实 TF-IDF；在 eager 前对实际 Frame 描述子独立 transform 到局部容器，检查词袋/特征非空，不预先填写 Frame 容器；off 延后真实 ComputeBoW 后也必须检查非空才进入并发读测试。`native_fixture_02` 已链接 host01 的实际 DSO，通过 disabled/off/on/flush-failure 四模式，因此将相同 idf2 内容同步为 canonical `stability_native.cc`，并保留 `stability_native_idf2.cc`。本次仅修夹具及对应元数据，ARM01/host01 已验证核心与节点编译输入不变；不重写 host01 原失败结果。
