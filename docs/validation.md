# BAG03 D435i ZIP 导入归档 — 2026-10-30

## RGBD36 — Git保存与16轮板卡稳定性对照（2026-11-02）

passing仅指实验与交付。代码/参数检查点a131e3c2及隔离ORB实现5c8b936f已上传相应checkpoint/research分支并核远端SHA；未混用根XFeat源码。主机/ARM冷编译42核心+2节点，118实际输入保持；原host01单文档TF-IDF夹具失败保留，8文档修正夹具链接同一真实DSO后disabled/off/on/flush-failure通过。主机MapPoint11/几何4411/并发探针及runner13检查通过。

原003015包完整SHA、5878精确输入、1×与8/8/10队列冻结，观测4轮、BoW4轮、矩阵8轮按预定顺序全部执行，无重跑。全部94048回调、77716 fresh；840封存文件SHA、54冻结项和1044独立数值检查通过，原RGBD31/35与旧部署保持。最终owned进程退出、11431可复用。14个ON帧表完整，但坐标/寿命截断及workers_joined=false保留。

P3均值有效率87.02%对P0 82.77%，nonfresh741/785对951/1075，平均少24.68%，均未达每轮≤709.1的30%门；成本60.68对46.85ms、P95两轮80.92/84.78ms，第二轮P99 115.07ms。P3新图/reset减少但onset更多，且有2/3组合图日志未定位/正确性未验证，不能拼世界轨迹或授予精度资格。四参数候选均未过预定质量门，条件深度/独立场景试验未触发。

主报告artifacts/rgbd_stability_trials_20261102/REPORT.md；数值复算analysis/compare_results.py、诊断analysis/analyze_diagnostics.py；独立审计validation/BOARD_TRIALS_INDEPENDENT_REVIEW.json。新图artifacts/rgbd_stability_charts_20261102/callback_breakdown.html，全16轮/82数据检查及129浏览器检查、桌面/窄屏目视通过。原8768网页由另一工作流维护，本实验未修改。没有待运行板测或后台构建；后续先核几何/合图与深度质量，不把本轮筛选失败写成算法验收通过。


## RGBD35 — 新地图内点衰减与参数实验方案（2026-11-02）

passing仅指分析和方案交付，报告为artifacts/rgbd_map_decay_20261102/REPORT.md，计划为NEXT_STEPS.md。没有算法改动、新板测或参数收益验收。原5878行输入身份/时间边界/阶段缺失/8分量和、5个原归档SHA核验通过；31个新图边界无效快照、36初始化未调用TLM按口径排除。首次失跟前首末5帧35组32降、localMP33增；固定年龄26组19降；独立原CSV逐行复核一致。全fresh相邻4655对中1984内点上升，不称单调下降或年龄因果。

执行：`/usr/bin/python3 artifacts/rgbd_map_decay_20261102/tools/analyze.py`、`python3 artifacts/rgbd_map_decay_20261102/trend_review/analyze_trends.py`、`/usr/bin/python3 -B artifacts/rgbd_map_decay_20261102/vision_review/analyze_orb_depth.py`。12个冻结源码文件与atomic_02编译输入相同；原depth5878张只读扫描，450离线ORB坐标取样覆盖36首帧及63起始，分箱和/cap单调检查通过。原bag全SHA复用RGBD34已验证记录，本轮未重复全包散列；每张取样depth身份/负载SHA保留。不得把离线ORB/F对应当历史板端地图内点。

四组700/1000×5/8候选YAML可读，P0字节相同，其他仅nFeatures/nLevels变更，类型保持；均prepared_not_replayed。4幅PNG目视与SVG解析、21本地链接核验通过。独立validation/INDEPENDENT_REVIEW.json passed，无待解决发现，包含报告版本SHA；浮点YAML应写80.0的建议已修正。首轮分析脚本对未执行阶段None求和报错，已修正为仅核已测分量，原空值保留；失败与成功证据均保留。

只读交付复核：`/usr/bin/python3 artifacts/rgbd_map_decay_20261102/tools/verify.py`。该入口检查链接、配置、图像及存在时的SHA256SUMS，不重生成封存报告。analyze/独立分析脚本为派生重建入口，会重写本轮输出。未执行build.sh/build_ros.sh或新SLAM回放，因为本轮仅诊断与方案；30/50门槛、近点100/70、100槽位不是本冻结版YAML参数，ThDepth是近远分类不是全链路depth cap。

## RGBD34 — 原图同步特征与匹配中间图（2026-11-02）

passing：artifacts/rgbd_feature_visuals_20261102/README.md。编译命令记录在build/compile_command.txt；prepare读取原bag 5878 RGB身份并保存450原始/灰度输入，render完成324组特征/金字塔及323组真实相邻帧匹配。独立重算323组Hamming双向ratio/距离/互最近邻、逐点/坐标/mask/全数与前120绘制数量、1942资产SHA及尺寸均通过。293个有效板测快照数量全部吻合；31个新图边界Frame重置后快照身份不可靠，页面标为不可用，原始27差异全在这些边界（COUNT_BOUNDARY_REVIEW.json）。未宣称逐点等价。BAG_SHA256.txt为原bag本轮全SHA；RGBD33_SHA_VERIFY.log为旧封存全部保持。

实际浏览器同origin验证43/44/1/45/90/674/1370/5878、4类图像同步、模态放大和关闭/Esc、390×844窄屏无横向溢出，截图及BROWSER_VALIDATION.json留档。独立INDEPENDENT_REVIEW.json passing；package_site.py逐ZIP成员与dist SHA一致，2618文件/262132299字节。原SLAM数据data.js保持，离线F模型内点与板端tracking_inliers分列。

仅构建独立主机提取工具，无需根build.sh/build_ros.sh，无板端重放、算法修改或外部发布。未获取原板端未落盘的真实逐点MapPoint/优化残差信息；F估计未检查所有几何退化，不将其视为定位成功。运行中的127.0.0.1:8768新owned进程见本轮LOCAL_SERVER.json。


用户请求的 `~/bag.zip` 实际文件为 `/home/ywl/bags.zip`（7269481634 字节）。已完整解压到 `bags/d435i_local_20261029/`，保留原始ZIP。252个ZIP条目中有41目录、201普通文件和10个源机绝对日志链接；211个非目录成员CRC全部通过，普通文件SHA在解压后及归档后复核一致。10个ros_logs/latest转换为同目录UUID相对链接，原绝对目标及ZIP原始字段保存在ZIP_MANIFEST.json。ZIP扩展字段的UTC mtime已保留；未执行包内脚本。

5个bag共9832118927字节，物理全扫描/解压/消息边界、ROS原生索引逐话题计数与起止时间、全部47416消息反序列化均通过；RGB与深度header逐项相等，共11852对。每包完整SHA均匹配ZIP提取、SHA256SUMS、session_result及bag_check四处记录；与原22包和新包之间没有整文件SHA重复。以上只证明导入与文件完整性，不改变原采集门：10会话仅1个原检查通过，4个有包会话有停流/低帧率失败记录，5个无包会话的日志也完整保留。

综合CATALOG.json、CATALOG.csv、TOPICS.csv、ALIASES.csv已增量加入5包，现27包；原22条JSON记录和三个CSV的原字节前缀保持，旧bag的inode/大小/mtime保持。录制时间用bag UTC epoch换算Asia/Shanghai，首包为2026-10-29 15:30:39，不能把源会话名的003015直接当成北京时间；源会话名与无时区JSON原文未改。

运行证据位于 `artifacts/bag_import_20261030/`：import_zip.py extract、audit、finalize、verify均通过，最终verification.json为passed；复核入口为：

```bash
/usr/bin/python3 artifacts/bag_import_20261030/import_zip.py verify
```

该verify只读，不会重写综合索引。旧20261029的organize.py verify会按当时22包重写索引，不应再用来更新当前综合目录。会话明细在 `bags/d435i_local_20261029/SESSIONS.csv`，来源与结果在同目录IMPORT.json及证据目录RESULT.json。未运行算法构建、相机、ROS master、板端回放或Git操作，因为本次仅做离线数据归档。

以下历史记录保留。

# STOR01 DBoW3 旧产物归档与压缩 — 2026-10-29

按用户要求执行与 RDK 类似的清理。既有 current_only/full 两个交付压缩包 SHA 与原发布记录一致；逐成员核对解压文件的 SHA、类型、权限和链接目标后，移除 73916 个已归档文件/链接，释放 10.36 GiB。保留两个不同版本的原压缩包、1315 份小型报告/配置/清单，以及外部验证脚本读取的 source/current/core/src、include。历史交付重新编译前需先恢复解压内容。

旧独立构建删除 926 个 ELF 对象（0.477 GiB），lockdiag04→05、E2 sanitizer 续链接、C5 重现及跨实验复用对象保留。35 个大型 CSV、23 份地图、8 个 perf_dump、4 个 perf.data、48 个 ORB 输出和 1 个符号文本共 119 文件无损 gzip，节省 6.006 GiB；所有地图数据仍可恢复。实际 gzip 解压安装与 tar 成员提取抽检通过，原模式和 SHA 一致。

`python3 artifacts/dbow3_storage_cleanup_20261029/CLEANUP.py verify` 已通过：全部新压缩文件的压缩包 SHA/解压 SHA 一致，原交付压缩包完整 SHA 未变，13057 个保留文件/链接的 inode、大小、mtime 及指定内容指纹保持。当前源码、根 build/lib/Thirdparty、Git、原始 bag、图表和当天分享包均保留。项目约 108.67→92.01 GiB，计入约 193 MiB 审计记录后 home 可用净增 16.65 GiB，达 53.34 GiB（82% 使用率）。

证据：[RESULT](../artifacts/dbow3_storage_cleanup_20261029/RESULT.json)、delivery_review.json、objects_review.json、compression_review.json、两个 journal、protected_files.json、restore_smoke_test.json。恢复工具为同目录 RESTORE.py，命令见 session-handoff。清理核验针对本次归档布局；恢复原文件后不再满足“原路径已移除”的断言，历史原 manifest 复核也应先恢复所需文件。

未运行 build.sh、ROS/相机/板端回放、Git 提交或上传：本次仅处理生成产物。源文件与原实验结果的资格结论不变。

以下历史记录保留。

# BAG02 根目录与 bags 数据归档完成 — 2026-10-29

本次按用户要求整理根目录和 bags/ 中全部 ROS1 bag，不修改算法、不启动相机/ROS master/板端，也不执行 Git 提交或上传。24 个旧路径对应 22 份独立数据，约 49.18 GiB；19 份已索引、2 份尾部不完整、1 份 4117 字节空录制。SHA256 与包含话题/类型/记录时间/完整消息负载的指纹没有发现额外完整重复包；原有两组硬链接仅改为兼容软链接，未宣称释放其 4.17 GiB 数据块。

16 份文件同分区原地迁移到 bags/recordings、bags/incomplete、bags/empty_or_failed 下，按实测话题类型和实际首条记录的北京时间分组；保留原 basename。既有 camera_20261021 和 loop955 会话目录及其配置/日志保留。17 个根目录旧名与 bags 下空录制旧名共 18 个入口改为相对软链接，冻结脚本、旧 manifest、RDK 恢复 bag 均未修改。9/8 和 8/6 09:50 两原件不重新建立索引，不改指向恢复版；残缺包的消息数和时长只覆盖完整可读记录。

验证已完成：none/bz2/lz4 三种压缩小包自检；19 个已索引包与 ROS 原生元数据逐话题计数/起止时间一致；5 个 camera 包与原归档 SHA 一致；22 份迁移后完整重算 SHA、inode/字节数/mtime 检查通过；全部旧入口解析正常，根目录普通 bag 为 0，bags 中 canonical 普通 bag 为 22；数据块总量不变。首尾预览已经目视，只用于场景描述，不代表全序列质量或相机型号。

交付：bags/CATALOG.csv 与 CATALOG.json、TOPICS.csv、ALIASES.csv；审计、迁移计划、逐步日志、首尾预览、最终核验在 artifacts/bag_catalog_20261029/。运行命令为 /usr/bin/python3 artifacts/bag_catalog_20261029/audit_and_organize.py selfcheck 与 audit，及 organize.py plan、apply、verify。apply 已完成，不重复执行；未来只读复核可运行 organize.py verify。BAG02 passing 仅指整理与一致性验证，不表示残缺原件已修复或 SLAM 精度通过。

未运行根 build.sh 或板端测试：本次仅做数据归档。修改范围为新审计工具/清单、bag 路径布局及三个共享 docs 的 BAG02 记录；此前用户的录制工具、文档和其他未提交工作保留。

TODO：如需使用两份不完整原件，应在独立位置另行恢复并保留原始 SHA；本次不包含恢复。没有待执行删除，也没有新增完整录制副本。

以下历史记录保留。

# RGBD27 GitHub发布完成 — 2026-10-24

用户已明确确认允许将f75a3ff推送到WilliamProzac/ORB_SLAM3_DBoW3的research/rgbd-mappoint-atomic-20261023。同一已准备提交已成功push，远端refs/heads/research/rgbd-mappoint-atomic-20261023精确为f75a3ffd9addd94d23224574c4fbced2b58f4224；发布checkout无未提交改动。未force push，未合并主分支，未新增板测或部署。

发布含最新推荐atomic_02核心、匹配ROS源码/服务与构建适配、实际方法探针、源SHA及验证说明；共49个修改/新增文件。完整commit message已写入Git提交，并保存在artifacts/rgbd_github_publish_20261023/COMMIT_MESSAGE.txt。130发布文件/116原ARM输入、398最终独立检查、主机41core+2node及2fixture构建、MapPoint11契约与几何4411用例均保持上一轮已验证结果，本次未无故重编译或重测。

之前自动审批拒绝和LOCAL_COMMIT.json保留为历史；具体目的地授权记录为operations/USER_AUTHORIZATION_20261024.json，成功及远端核验为REMOTE_PUSH.json。RGBD27发布状态passing；RGBD25等原性能/完整输入/90ms/生命周期资格仍按原结论保留。当前XFeat主工作树和旧实验源没有算法改动。

以下原发布准备及历史记录保留。

# RGBD27 GitHub发布交接 — 2026-10-23（本地提交完成，远端推送待具体目的地确认）

用户授权上传最新代码。主XFeat树没有算法改动，推荐RGBD25 atomic_02在隔离checkout按core5ea9afb6整理，匹配ROS放runtime/rgbd；不混入几何候选，原归档/现有工作树保留。本地提交f75a3ffd9addd94d23224574c4fbced2b58f4224，分支research/rgbd-mappoint-atomic-20261023，目标git@github.com:WilliamProzac/ORB_SLAM3_DBoW3.git。

130发布文件/116原ARM编译输入保持，398最终独立核对通过。新ROS构建声明补齐MapScaleObservation、核心/Thirdparty路径、service生成顺序和GNU17；主机实际41core+2node及2fixture编译成功，MapPoint11契约与几何4411用例及并发检查通过。已详细记录原八轮性能、工作量/完整输入/90ms等限制，没有新板测/生产晋升。

自动审批在执行前拒绝git push，理由为用户未在可信内容明确具体GitHub仓库，存在源码/文档外发风险。已通过异步问题请求明确授权该repo和branch；不得换协议或间接绕过。剩余仅在用户确认后重试同一commit push并核对远端SHA。不要重编译/重测或修改已封存代码来处理授权问题。状态见artifacts/rgbd_github_publish_20261023/LOCAL_COMMIT.json，完整message在同根COMMIT_MESSAGE.txt，checkout工作树clean。

以下历史交接保留。

## RGBD26 MapPoint原子查询加入历史交互图（2026-10-20）

用户指定rgbd-callback-charts技能，将RGBD25单独MapPoint原子查询的a_01/a_02与同期b_01/b_02加入旧36轮，生成独立40轮入口 [callback_breakdown.html](../artifacts/rgbd_mappoint_charts_20261020/viewer/callback_breakdown.html)。默认mp_atomic_a_01；旧图与归档只读，不启动回放、编译、部署或发布。

新增11626/11640回调、缺14（原子3/基线11）；新增已完成均state2/fresh/valid/map0，全40轮113745/116400、缺2655。共同基线含容器＋KeyFrame原子＋ORB P，新增MapPoint原子查询，未加入几何或几何＋原子候选。完整detail计时及初始化未执行null保留；历史M0未测、失跟/恢复、源号缺失及八色语义不变，A1差额不称锁等待。旧36run对象按原值保留。

原技能生成器5918来源SHA和完整源身份/同包预览校验通过。真实Chrome155检查512项通过，无异常/consoleerror/外网请求/失败资源；桌面/窄屏四图已目视。首次浏览器在沙箱内因setsockopt权限启动失败，原browser_check保留，独立临时Chrome的browser_check02为通过结果。输入/技能和共享docs前快照已保存，生成器/模板未改、技能字节快照在新档。

单独原子为本次新增候选中更有支持的研究方向，回调均值/P95中位约−3.08%/−4.31%，但工作量、控制漂移、完整输入及90ms资格未过。图表交付不改变RGBD25 blocked或历史性能结论，也不能作跨批次因果排名。原生/长包无需重跑；未运行根build.sh，因为只生成图表。

只读封存核验入口：

```bash
python3 -B artifacts/rgbd_mappoint_charts_20261020/verify_delivery.py
```

实际配置、来源清单、命令/首次失败及截图复核见新根config.json、viewer/VALIDATION.json、operations/COMMANDS_AND_VISUAL_REVIEW.json和README。预览相对复用旧frame_preview；移动时保留目录关系。独立复核418835项通过，HTML完整内嵌数据、新四轮分量/百分比/导出及旧36run逐值一致，最大分量求和误差约2.84e-14ms。独立校验器首轮清单对象/字符串比较错误已修正，首轮脚本/结果保留在checks/independent/attempt01；生成器与图未改。RGBD26 passing仅指图表交付，不表示性能通过。

以下旧验证记录保留。

## RGBD25 几何合并读取与MapPoint原子查询（2026-10-20研究完成，性能／质量资格未过）

用户授权的两项修改及组合版本已实现、独立冷构建、板端测试；入口 [REPORT](../artifacts/rgbd_tracking_optimizations_20261020/REPORT.md)。新归档及板端根均为rgbd_tracking_optimizations_20261020。保留组合＋ORB P基线，普通RGBD投影采用一次现成几何快照，MapPoint状态release/acquire发布查询，锁与生命周期语义单独审核；未切LocalMapMode。

四版本4411静态几何用例逐字段/字节一致，退休／替换／生命周期／持锁查询／TEXT与BINARY互读及异常加载原生进程共168项通过。最后32项bool后尾字段截断在全部长包结束后补测，异常后状态与退休计数四版一致。固定单点查询CPU约94.68→7.34ns、全部通过投影约741→364ns，仅证明局部成本，不外推完整回调。无OOM注入、完整Atlas在线保存或惯性/合图生命周期资格。

冻结664项后B→G→A→GA→GA→A→G→B八轮完整2910前缀、1×，原700/5/3、队列8/8/10及LM/LC保持；无短包替代或择优重跑。处理23255/23280、缺25。单独原子两配对同方向改善，轮统计等权中位回调均值77.502→75.118ms、P95 96.734→92.564ms，缺失两轮合计11→3；A3收集CPU8.642→7.076ms，Search13.287→12.483ms。几何回调均值77.747ms，无净收益。GA1完整且P9589.395ms、质量通过；GA2源2339–2373共35无效新位姿，2374换图，不能采纳低耗时或仅剔除35帧比较。

每候选74判据，G/A/GA分别21/23/27失败；控制均值差1.700ms超过1.550ms预算，原子局部KF/MP/扫描槽也变化，未证明固定工作量全回调3.08%因果收益。三个候选均未通过全部资格；后续研究优先单独原子，几何及组合暂不采纳。本轮研究交付完成，RGBD25 blocked不表示待批准。

独立原始CSV3308核对无差异，112回传文件SHA/文件集合一致，246板端包/依赖前后一致，16owned身份退出，仅TIME_WAIT。1169旧输入保持。26登记任务与整个八轮区间无交集；首轮输入前有一次只读metadata查询无精确区间，计时回放无本任务编译/绘图/归约。温度45.928–50.911°C、频率1.2–1.5GHz；有系统换出、少量majorfault，不能称恒频或零分页。采样丢失、部分cgroup/fulljoin及GT精度仍有限。

实际命令、失败及参数均在新归档operations/builds：构建/部署/native、freeze_run、run_sequence、analyze_candidates及独立审查。主分析exit1表示数值门未过，无解析失败；整数状态导出修订与原判据完全一致，早期误判已撤回。根build.sh对应另一工作树，未运行；使用匹配runtime隔离完整ARM构建。只读封存核验不回放：

```bash
python3 -B artifacts/rgbd_tracking_optimizations_20261020/verify_archive.py
```

TODO：完整输入与90ms、控制/局部工作量可比性、GA2失跟机制、GT和完整生命周期。没有改旧图或生产包；未来重算用新输出。以下原验证记录逐字保留。

## RGBD24 A3／Search CPU成本拆分（2026-10-19研究完成，低扰动资格未过）

用户要求拆分两个阶段具体成本，已在新归档 `artifacts/rgbd_tracking_cost_20261019` 完成，不修改生产或旧归档。原组合＋ORB P成套二进制/130源文件直接复用；19Hz cpu-clock:u、16KiB DWARF、CLOCK_MONOTONIC，仅Tracking，原基础Scale/detail两侧相同。真实线程main调用栈、PID/TID/startticks、detach及5秒静置每轮确认。原生四阶段夹具171样本与31项解析测试通过；第一次本地路径错误未启动板端且保留。

实际四完整2910前缀1×长轮OFF→ON→ON→OFF，700/5/3、队列8/8/10、LM/LC保持。完成2892/2910/2905/2897，缺18/0/5/13；完成state2/fresh/valid/map0、两路pose完整。ON两轮A3clear/gather/Search CPU约1.4/9.3–9.4/14.2–14.3ms，合计占CB CPU约32%。CPU采样4342/4347，raw/script与目标身份/时钟/EXIT覆盖逐条通过、LOST0；MapPoint::isBad、几何getter、网格查询热点重复。条件占比、内联/未知leaf与栈上限缺口详见REPORT，不转成精确逐调用或锁等待时间。

77门28失败：4均值、9观测工作量、15控制工作量波动；控制均值中位77.284→ON79.773ms，不能归为纯采样开销。四轮P95均>90ms，原完整输入与性能资格未过。独立77门一致，阶段1246项一致（浮点末位容差）、另476算术项通过。234冻结、78raw、1599旧文件及已记录依赖SHA保持，10owned退出；四轮完整外层无已登记重任务交集。温度峰51.107°C，有1.2GHz，系统映射after-only、资源采样丢失/cgroup/full-join未知保留。

已运行入口：`tools/run_sequence.py`（四轮）、`tools/decode_profiles.py --attempt 01`（仅四轮后）、`analysis_tools/check_profile_influence.py tracking`（结果exit1预期）、冻结 `analysis_tools/analyze_tracking_profile.py`（两ON exit0）、`tools/post_audit.py`（技术通过）。所有脚本位于上述新归档，完整参数/起止/失败见operations/jobs、PLAN与运行记录。补充叶PC脚本首版demangled解析失败和attempt02修正均保留，冻结主解析器未变。

只读封存核验：

```bash
python3 -B artifacts/rgbd_tracking_cost_20261019/verify_archive.py
```

不回放、不检查未来共享docs当前值。重算选新输出，保留原证据；无C++改动故不运行根build.sh或新算法等价测试。TODO：A3内联操作精确分账、原基线低扰动可比性、完整输入及90ms、GT及完整生命周期。本轮有限诊断已结束，不追加回放；RGBD24 blocked表示资格未过，不是待批准。

# Validation

Source basis: `README.md`, `build.sh`, `build_ros.sh`,
`run_stereo_inertial.sh`, and repository tree scan on `master`.

## RGBD23 组合版＋ORB P完整长包（2026-10-19研究完成，正式性能／工程资格未过）

已按用户要求合并并完成板端测试，入口 [REPORT](../artifacts/rgbd_a1_orbp_20261019/REPORT.md)、[主结果](../artifacts/rgbd_a1_orbp_20261019/analysis/SUMMARY.json)、[独立复算](../artifacts/rgbd_a1_orbp_20261019/analysis/independent_long/REPORT.md)。只新增实验归档和更新三份共享docs，旧归档1514项SHA及生产保持。

从RGBD20组合源最小移植RGBD13最终P行并行＋C1 FAST工作区复用，只差六文件；保留组合版Frame／Tracking／System／KeyFrame与三批次计时布局。对照复用原匹配成套构建，候选核心41＋节点2单元全冷编、零旧对象复用。探针只编译fixture并链接各自实际新DSO；四loader＋四main核对加载身份。2910张ARM灰度输入的全部关键点原位字段、顺序及描述子字节一致，每版60边界输出、空输入／受控线程池异常恢复／重复终止与候选真实FAST异常恢复通过。不是System全生命周期或真实OOM证明。

事前冻结322项，B→P→P→B四轮完整2910前缀、1×、700/5/3、队列8/8/10、LM／LC开启；四轮总区间不运行本任务编译分析绘图，没有短包替代。完成2846/2903/2898/2849，共11496/11640、缺144；对照两轮缺125，候选缺19。完整回调轮均值中位85.615→79.672ms（−6.94%），逐轮P95中位106.998→97.286ms（−9.08%），ORB均值36.824→29.355ms（−20.28%）；两配对同方向改善。全部完成行state2/fresh/valid/map0，两位姿消费者完整；无真值ATE／RPE结论。

78项工程判断53过25失败：控制均值跨度1.951ms>1.712ms预算，局部KF／MP／gather槽位24项可比性失败；全局规模、速度／缺失及ORB四支持通过。局部地图工作量变化，不能将端到端改善解释为固定工作量的纯单位成本收益。四轮完整输入且P95≤90ms目标均未通过。独立原始复算2522项与主结果一致。

56项raw、8个owned进程退出、原始身份／冻结／旧输入核对通过。严格post工具technical_audit_pass=false原样保留，唯一失败为11411的四条TIME_WAIT；原始记录无LISTEN／ESTABLISHED，补充解释没有改写严格结果或追加板读。峰温51.058°C，有1.2GHz样本，不称恒频；B2 majorfault+1，swap／OOM无增量。系统解释器仅after SHA、采样丢失／部分cgroup／逐后台join证据有限。共享docs前后快照封存，未来文档可继续变化。

执行入口：`tools/build_pair.py`、`tools/build_keyframe_probe.py`、`tools/native_execute.py`、`tools/freeze_run.py`、`tools/run_sequence.py`、`analysis_tools/check_candidate.py`、独立脚本及`tools/post_audit.py`；实际参数、起止、失败与修正保留在operations/builds。门合成最终33/33、比较器9项正反检查通过，早期fixture常量／map0检查／输出目录／冻结别名失败均保留。只读封存核验：

```sh
python3 -B artifacts/rgbd_a1_orbp_20261019/verify_archive.py
```

此命令不回放；重算分析须另建输出目录，避免改写封存。TODO：完整输入与90ms、控制／局部工作量可比性、完整System生命周期／保存重载／真值精度仍未通过或未覆盖。RGBD23 blocked指正式资格，不是等待用户批准；本次合并及有限测试交付已完成，无生产替换或追加回放。

## RGBD22 A1下游因果与八完整长包（2026-10-19研究完成，因果／低扰动资格未过）

入口 [REPORT](../artifacts/rgbd_a1_causal_20261019/REPORT.md)、[主结果](../artifacts/rgbd_a1_causal_20261019/analysis/RESULTS_SUMMARY.json)、[实际方法干预](../artifacts/rgbd_a1_causal_20261019/preflight/CAUSAL_NATIVE_VALIDATION.json)、[独立长轮](../artifacts/rgbd_a1_causal_20261019/analysis/long_independent/REPORT.md)。仅新归档与三份共享docs变更，旧RGBD20的1510文件及生产未改。

两版成套ARM冷构建各40+2、零旧对象复用；匹配新DSO的6个main和4个loader检查实际执行通过。真实A1/A3/Need/退休受控交错显示80→81选中KF、参考有效点40→0、插帧true→false，但A3为同一41MP集合；系统后台已先join，夹具不等同自然LM/LC并发或全Track/Search。原语记录增量约0.099us，整轮仍另验收。23项解析正反例通过，原失败保留。

八轮按C关→C开→A开→A关→A关→A开→C开→C关，原2910完整前缀／1×／700-5-3／8-8-10／LM-LC开。完成2845/2844/2836/2837/2846/2843/2844/2845，共22740/23280、缺540；全部已完成state2/fresh/valid/map0，两种位姿数量完整。P95 107.216–110.463ms，8轮正式完整输入和90ms均失败。56个预定扰动判断中C失败10、A失败2；C关自身均值跨度1.820ms，不能把开关差异全部归因记录器。两模式共有104388608B预分配预触页缓冲／公共钩子／末尾join，无本轮原二进制锚点，资格只涉及增量记录。

四ON共1308628过滤调用全false、无SetBadFlag入口；不是配置关闭退休。源30的Track内map-update后、当前TrackLocalMap/A1前snapshot已754/753MP，而A1输入指纹／关联数／候选票数相同；容器重复也分叉。仅能排除本次源30过滤为该计数差异起点，更早时序反馈未知。选集顺序首次差异不等于成员集合首次差异。11134共同帧的A3算术分解精确闭合，共有创建源KF槽位变化为0；仅定位数量，不能充当性能验收或因果。旧四轮A1外CPU+2.186ms、自然控制差−2.158ms同量级。

离线原始独立复算968字段一致；318冻结、128raw本地/板端SHA、227lib/Vocab、旧1510以及16owned退出／11411通过。八轮总区间2493.485s暂停编译分析绘图，已知duration账本无交集；VERIFY_JOB只有检查时刻、duration未知。峰温51.302°C，三个ON各majorfault+1，系统swap/OOM无增量；两轮有1.2GHz样本在回调整体包络，不称恒频。没有新sched_wakeup、全持锁者、完整点状态或真值精度证据。板端原始数据保留，停止8轮，不关闭LM、不追加优化或生产晋升。

实际命令及失败均保存于新归档。只读离线核验（不回放）：

```sh
python3 -B artifacts/rgbd_a1_causal_20261019/verify_archive.py
```

重算脚本会写派生结果，不应直接在封存目录重跑。TODO：原约2ms增量的全局因果、完整长包低扰动资格、最早地图分叉前可恢复状态和真实唤醒证据仍未补齐。RGBD22 blocked表示这些目标未证实，有限分析测试交付已完成。

## RGBD21 A1长包加入历史交互图（2026-10-19图表交付通过）

按用户指定 `/home/ywl/.codex/skills/rgbd-callback-charts/SKILL.md`，新增低扰动4轮、CPU/profile与sparse8轮、独立候选8轮、组合4轮；旧orb_delivery的12轮对象逐值保留，共36轮。入口：[新版HTML](../artifacts/rgbd_callback_charts_20261019/viewer/callback_breakdown.html)、[说明](../artifacts/rgbd_callback_charts_20261019/README.md)。默认combined_on_02，旧锁诊断四轮已有未重复。未运行SLAM回放、编译、部署或改算法。

每轮2910源位置，新增68103完成/1737缺失；全图102119完成/2641缺失。原包ordinal/stamp字符串/RGB/depth seq全匹配，同包RGB及固定0–5m深度预览相对复用。基础六阶段在额外稀疏/CPU观测轮中仍逐回调完整；零calls且-1为未执行，不是假设抽样缺测。数据无负A1差额、非法时钟或负残余，无钳零/丢异常；M0未测分量保持null。两新增失跟轮及旧失跟底色、缺失留白保留，M1标签不表示观测侵入合格。

skill原生成器及模板未修改。实际命令：

```sh
python3 -B /home/ywl/.codex/skills/rgbd-callback-charts/scripts/generate.py --config artifacts/rgbd_callback_charts_20261019/config.json --output artifacts/rgbd_callback_charts_20261019/viewer
python3 -B /home/ywl/.codex/skills/rgbd-callback-charts/scripts/check_browser.py --html artifacts/rgbd_callback_charts_20261019/viewer/callback_breakdown.html --output artifacts/rgbd_callback_charts_20261019/browser_check
python3 -B artifacts/rgbd_callback_charts_20261019/checks/verify_data.py
```

生成器核5973来源SHA；2807856数据项对照raw CSV/导出百分比/回调端点/版本SHA/缺失留空，12历史对象保持，旧图及来源5999快照前后相同。组合源722的A1差额10.695675ms人工对照一致。Chrome实际469项通过（所有36轮、边界/缺失悬停、失跟、滚轮/拖动、图像解码与锁定快速切换），四份桌面/窄屏截图目视通过，页面异常/consoleerror/失败资源/页面外请求均0。生成时VALIDATION.browser_status仍not_run，实际浏览器结果独立存browser_check。

根目录新图是旧交互图的扩展副本，原HTML/CSV/静态图/测试归档均未覆盖。未改变任何性能验收结论，RGBD17–20原blocked保持。只读交付核验：`python3 -B artifacts/rgbd_callback_charts_20261019/checks/verify_delivery.py`；原生成/浏览器入口要求新/空输出，复现用新目录。新before/after保留本次根文档快照，不把未来正常文档演进当作旧数据变化。

## RGBD20 组合候选完整长包对照（2026-10-16完成，工程门未过）

入口：[REPORT](../artifacts/rgbd_a1_combined_20261016/REPORT.md)。用户授权后独立部署container_02精确复用控制与combined_01全量ARM冷构建40核心＋2节点，原128文件中仅KeyFrame.h/.cc为两补丁组合。实际DSO516正常／全部OOM／own锁与原子查询、42次退休发布与归档测试通过；611项事前冻结。有限visual RGBD研究不等于完整System／GBA／惯性／Atlas或真值精度认证。

四轮固定container→combined→combined→container，全部2910源／1×／700／5／3／8/8/10，LM/LC保留，无新增细观测。完成2847／2842／2844／2858，缺63／68／66／52；合计11391/11640，缺249。已完成state2/fresh/valid/map0，两pose流一致，原完整输入与90ms门四轮均失败。采集正常结束不代表全部输入完成，join字段仍未知。

A1尖峰25→1，过滤P95两轮中位0.197→0.072ms；回调mean中位84.585→86.584ms、P95 106.815→107.580ms，缺失中位+9.5。66工程门17失败：控制均值与局部工作量12项、均值/P95/缺失5项；local KF/MP均值中位+8.95%/+10.26%，不可把整体差异全归因atomic。唯一组合尖峰源722落在过滤外粗残差，具体锁/调度原因未知。没有追加回放，生产保持。

主归约：`python3 -B artifacts/rgbd_a1_combined_20261016/analysis_tools/check_candidate.py combined`，预期exit1，封存后在完整副本中执行。独立补充01对401544项raw及同66门一致，首次独立结果错误额外要求backend_joined=true的4失败保留，补充只撤销该无依据前提，未放宽原门。只读交付核验：

```sh
python3 -B artifacts/rgbd_a1_combined_20261016/analysis_supplement/verify_delivery.py
```

收尾及限制见operations/POST_LONG与FINAL_STATE.json；全部重型job在四轮总区间外，idle声明＋连续采样辅助，无全程恒频或排他系统负载主张。准备/审计工具的目录权限、重复stage、Python3.8收集校验及软链期望问题均保留，未重跑fixture或图像。RGBD20 blocked为性能／可比性未通过，授权验证工作已完成。

## RGBD19 板端完整长包候选验证（2026-10-16完成，性能门未过）

入口：[REPORT](../artifacts/rgbd_a1_candidates_20261015/REPORT.md)、FINAL_STATE.json。用户明确允许后部署到NEW独立路径；3个成套pair/config全SHA核验，两个候选源与40核心＋2节点冷构建保持。实际ARM container516正常案例／mutex／逐分配失败验证，atomic42进程／发布与重复true窗口／TEXT-BINARY双向及截断异常验证通过；两个独立源码审核支持限定visual RGB-D研究，不等于生产或完整System／活动GBA／惯性／Atlas安全认证。

两个427项事前冻结ABBA矩阵全部完成，八轮均2910源／1×／700／5／3／队列8/8/10／基础阶段统计，LM与LC保留，无新增CPU或锁细观测。全部主机接收23280对，完成22714回调、缺566；已完成均state2/fresh/valid/map0，无reset，两个pose消费者对应完整。normal exit和RESULT.complete是测量执行完成，不是输入处理完整；8轮原2910完整＋回调P95≤90ms门全部失败。

主66门与独立原CSV归约逐项一致：container7项失败（基线均值及local KF/MP漂移）；atomic5项（工作量／控制3项、中位P95改善0.693%不足2%、第二配对P95+3.761ms）。容器P95中位112.550→110.226ms，原子108.738→107.985ms；A1尖峰311→25／254→7。过滤改善不替代可比性和整机收益，不能据粗残差断定持锁者或调度；两候选保持独立，不晋升生产。

实际归约命令：`python3 -B artifacts/rgbd_a1_candidates_20261015/analysis_tools/check_candidate.py container`（atomic同理），预期exit1；它会写分析文件，封存后请用只读核验器或复制整个目录后再归约。完整只读交付核验：

```sh
python3 -B artifacts/rgbd_a1_candidates_20261015/analysis_supplement/verify_delivery_20261016.py
```

审计证据在operations/POST_LONG_20261016：427×2冻结与raw transfer、exe/maps、输入和两pose身份通过；全部重型job与8轮总外层区间无重叠。initial/final idle均为放包前两次检查，final不是结束声明。连续host_load禁标志全0、末次16个owned身份退出，无目标进程/11411残留。共享227库＋词典、3新包及原baseline前后SHA一致；实际227映射路径中只有系统解释器缺独立pre SHA。保留C_on02两个1.2GHz覆盖窗采样、两B1各一次majorfault、swap/OOM增量0；cgroup空字段及资源采样无显式loss计数不当作全0。

native01错库loader guard与audit外链枚举错误、首次序列缺runs父目录（回放0）及描述图表Path/旧Matplotlib接口准备失败均原样保留，未改变冻结工具／候选／原始数据／数值门。最终8次真实回放后停止，没有择优追加或短包替代。RGBD19 blocked指实时性／可比性门未通过，授权测试和有限研究交付已完成；旧RGBD18因果证据缺口保持。

## RGBD19 本地准备历史（2026-10-15，已由上述板测续接）

实验入口：`artifacts/rgbd_a1_candidates_20261015/REPORT.md`。源于 current_long_01/02 的原128文件；container只改KeyFrame.cc，atomic只改KeyFrame.h/.cc，分别成套40核心＋2节点ARM全量冷编。baseline精确复用原pair，不称本轮冷编。三个节点SHA一致、库各自绑定；四个ARM fixture均已编译，但执行数为0。

已运行的核验：container_host_attempt03的1028正常案例、6受控读取异常、真实mutex链及逐分配失败；atomic_verify_source.py的10项边界、host syntax05两变体及83个KF查询点审查；analysis_tools的9个判定测试通过。ENTRY_GUARD_TEST.json确认两候选pending状态均拒绝RUN_FREEZE。真实ARM反汇编确认atomic query为LDARB且没有mutex调用，只证明生成代码。

容器版正常顺序保持，分配压力下调用前缀可变；原子版保留晚发布，提前false、重复退休旧true、对象寿命与静止归档前提均有明确契约。宿主提取方法与静态审核不能代替整库／System执行。实际ARM DSO并发与TEXT/BINARY双向／截断归档42次进程计划尚未运行，无生产安全或性能通过结论。

本地只读完整性复核命令：

```sh
python3 -B artifacts/rgbd_a1_candidates_20261015/tools/verify_review.py
```

自动审批拒绝NEW/container_02板端部署，需用户明确授权；命令未执行，后续上传／探针／回放均停止，原文见operations/DEPLOY_APPROVAL_REJECTION01.json。只有先前独立获批的实验父目录创建。现有主机／镜像无QEMU，ARM loader本地执行ENOEXEC，未伪称模拟测试。可审查staging、拟执行具体命令及全部输入SHA均在新目录，旧归档／生产保持。

待授权后，先部署3个隔离成套包和4探针，完成实际库前置；按evidence SHA填写ENTRY_VALIDATION并冻结正式矩阵。两个独立B→C→C→B，每轮完整2910输入／1×／700特征／5层／3执行者／原队列，最多8轮，无短包替代或追加择优。只保留原基础阶段统计；每轮all-agent显式idle、账本及实际进程屏障，分析和绘图置于矩阵之后。全源缺失、P95、尖峰、局部工作量、质量、基线波动与正式90ms门分开保留；A1残差不能命名为锁等待或精确邻居阶段。

未运行：实际ARM fixture、所有本轮长包、完整System生命周期／活动GBA／完整Atlas保存重载／真值精度，前两项受部署审批阻止，后几项没有被定向测试覆盖。RGBD19状态blocked，不把本地准备完成写成优化有效；RGBD18原因果证据缺口及历史结论保持。

## First-Party Validation Entry Points Found

- `./build.sh`
- `./build_ros.sh`
- `./run_stereo_inertial.sh <mode> <sensor> [--log]`
- `./collect_slam_debug.sh [attach|launch-slam] [options]` for collecting
  board-side ROS runtime evidence from the RDK X5 catkin deployment.
- `./record_rgbd_loop.sh` for standalone S316 RGB-D capture from the laptop;
  see `docs/rgbd-loop-recording.md`.
- `./record_d435i_10fps_bag.sh` for ROS1 D435i 30 FPS capture and synchronized
  10 FPS RGB-D recording; use `--board user@host` or `D435I_BOARD` to select
  the SSH target. See `docs/d435i-10fps-recording.md`.

## REC01 Standalone RGB-D Recording

- Syntax: `bash -n record_rgbd_loop.sh` and
  `/usr/bin/python3 -m py_compile evaluation/rgbd_recorder.py evaluation/test_rgbd_recorder.py`.
- Offline checks: source `/opt/ros/noetic/setup.bash`, then run
  `/usr/bin/python3 evaluation/test_rgbd_recorder.py`. Uses real LZ4 bags to
  cover valid data, missing depth, zero depth, non-monotonic stamps, malformed
  payloads, mismatched RGB-D stamps, truncation and unfinished `.active` files.
- Board: `./record_rgbd_loop.sh --duration 6 --output artifacts/rgbd_recorder_validation`;
  repeat without `--duration` and press Ctrl+C after recording begins. Require
  successful preflight, finalized bag, matching SHA-256, a full message read,
  valid RGB-D pairs and no remaining session camera/ROS processes.
- Retry: use `--fetch <run_id>` with the same `--output`, and require another
  successful verification without starting camera processes. After the
  user-requested cleanup extension, successful verification deletes only the
  corresponding board `rgbd.bag`; verify the local copy and board logs remain.
- Cleanup regression: the same offline test script now has 6 test groups,
  including successful deletion/retry, failure preservation for mismatched
  hashes or invalid data/recording status, and refusal to delete changed
  remote files or sessions with `.active` bags.
- Passed on 2026-09-18: runs `rgbd_loop_20260918_145800_5dc9d3` and
  `rgbd_loop_20260918_150021_125c59` under
  `artifacts/rgbd_recorder_validation/`. The latter manually stopped via Ctrl+C:
  84 exact-stamp RGB-D pairs at 11.11 Hz, 1688 total messages, 172202854 bytes.
  `board_result.json` and `bag_check.json` record completion and validation;
  `fetch_test.log` records recovery and `final_board_processes.txt` is empty.
- Cleanup extension passed real `--fetch` on the first run: local 134217703-byte
  bag and 1311-message full read remained valid, the selected board bag was
  removed, and its logs plus the other session's bag remained. Evidence:
  `fetch_cleanup_test.log` and `remote_cleanup_check.txt` in the same directory.
- Not exercised: actual network failure, disk exhaustion, power loss, or a
  physical closed-loop walk. Basic data checks do not establish SLAM loop
  detection or geometric calibration accuracy. No C++ build is needed.

## REC02 D435i 10 FPS RGB-D Recording

- Syntax/regression: run `bash -n record_d435i_10fps_bag.sh`, compile
  `evaluation/rgbd_recorder.py`, and run
  `/usr/bin/python3 evaluation/test_rgbd_recorder.py` after sourcing Noetic.
- Hardware: run `./record_d435i_10fps_bag.sh --duration 8 --output
  artifacts/d435i_10fps_recorder_validation`. Require a stable post-warmup
  preflight above 9.5 Hz, exact RGB/depth pairing, finalized LZ4 bag, matching
  SHA-256, complete sequential read, verified board-bag cleanup, retained logs,
  and no remaining camera/ROS/recorder process.
- Passed 2026-09-24 with `d435i10_20260924_111100_130a26`: preflight RGB and
  depth were both 9.99 Hz with 100% pairing. The 7.906-second, 48,284,291-byte
  bag contains 79 RGB, 79 aligned-depth, 79 color-info, 79 depth-info and one
  TF-static message. Full read covered all 317 indexed messages; both image
  streams were 9.9929 Hz, 79/79 paired within 0.1 ms, with 87.94% sampled valid
  depth. SHA-256 is `e34f0df7f7583f7da17c51f254a59cbeb6cf14867c57e4205a1a79f41343f1c4`.
- The first hardware attempt correctly stopped before recording because the
  initial five-second header-rate window was 9.477 Hz. The recorder now waits
  for both topics, discards a configurable two-second D435i startup warmup,
  then measures a fresh five-second window. Entire recorded bags remain fully
  checked without discarding startup frames.
- Evidence is under `artifacts/d435i_10fps_recorder_validation/`. Physical
  closed-loop coverage, network failure, disk exhaustion and power loss were
  not exercised.

## REC03 D435i SSH 目标配置（2026-10-28）

- 本次仅修改专用 shell 入口与说明。`--board user@host` 由现有 Python
  解析器接收，入口增加 `D435I_BOARD` 环境变量；优先级为命令行、环境变量、
  原默认 `sunrise@192.168.1.11`。未改动共享录制程序。
- `bash -n record_d435i_10fps_bag.sh` 通过，实际入口的 `--help` 可正常运行。
  离线检查捕获真实 shell 入口传给 Python 的参数，再用真实解析器解析，
  将 host 调度和 rsync 调用替换为本地捕获，未执行 SSH/录包。
- 6 种场景通过：默认值、环境变量、命令行 `--board user@host`、
  `--board=user@host` 覆盖环境变量、SSH 别名配合 `--fetch`、空环境变量回退。
  同时确认 SSH/rsync 目标一致，原 RGB-D 话题、相机启动路径及校验阈值保持。
- source `/opt/ros/noetic/setup.bash` 后运行
  `/usr/bin/python3 -B evaluation/test_rgbd_recorder.py -v`：既有 6 组回归通过。
- 修改前脚本和文档逐字备份在
  `artifacts/d435i_recorder_portability_20261028/before/`；SHA-256 清单为
  `BACKUP.json`，检查结果为 `wrapper_checks.json` 和 `recorder_regression.log`。
- passing 仅指参数选择与离线回归；未进行新机器的 SSH 连接或相机实录，
  本次不涉及 C++/ROS 节点修改，无需核心构建。

## REC04 D435i 单文件录包（2026-10-28）

- 交付 `record_d435i_10fps_standalone.py`，只复制该文件即可在新 Linux 主机使用。
  主机不依赖 ROS 或仓库；板端需预装 ROS1/RealSense ROS1 驱动。内置相机 launch、
  Python ExactTime 同步后每三对取一、录制 supervisor、全量读包与验证/回传流程。
- 隔离目录中用 `/usr/bin/python3 -I -S <复制后的脚本> --help` 通过；
  同样隔离的主机校验没有导入 rosbag 或 NumPy，验证报告绑定到完全相同 SHA 的 bag。
- source `/opt/ros/noetic/setup.bash` 后运行：

  ```bash
  /usr/bin/python3 -B evaluation/test_d435i_standalone.py -v
  /usr/bin/python3 -B evaluation/test_rgbd_recorder.py -v
  ```

  最终结果为 17/17 和 6/6。新测试复用真实 LZ4 good/bad/truncated/active bag，覆盖
  无主机 ROS、报告缺失/损坏/哈希不匹配时不删板端原包、重复 fetch、真实 rsync 修复
  同大小同 mtime 的损坏副本，以及内置 XML/bash、ExactTime 同步/1、4、7 对取样和
  CameraInfo 原时间戳不变。板端 supervisor 测试替换相机/录制进程和 socket，
  实际执行启动资产生成与最终 bag 全读校验，未启动相机/ROS master。
- `--fetch` 环境自检不 source ROS、不导入 ROS Python 包，不会因旧 ROS 环境路径失效
  阻止回传。主机/板端均有 rsync 时使用 `--checksum`，否则使用 scp。
- 修改前八文件备份与 SHA 清单、最终日志和 `VALIDATION.json` 位于
  `artifacts/d435i_standalone_20261028/`。两次早期测试夹具失败日志保留，
  最终完整测试通过。原入口和 `evaluation/rgbd_recorder.py` 与任务前备份逐字一致。
- passing 只覆盖单文件交付和离线行为。TODO：新目标的依赖、USB 权限、实录、
  Python 抽帧吞吐和实际退出清理尚未板测，不继承旧 C++ 抽帧器的硬件性能结论。
  正常 SIGINT 清理路径沿用原 supervisor；强制杀死 roslaunch 后的节点残留风险未扩展验证。
  未运行核心/ROS C++ 构建，无算法或现有 ROS 包改动。

## REC05 D435i USB 本机录包（2026-10-28）

- 新增 `record_d435i_10fps_local.py`，D435i USB 接本机，内置 ROS1 launch、ExactTime
  每3对取1抽帧、手动/定时录包、完整读包与 SHA。全部数据保留在本机，不依赖项目文件。
- `/usr/bin/python3 -I -S record_d435i_10fps_local.py --help` 与脚本/测试 AST 检查通过。
- source `/opt/ros/noetic/setup.bash` 后运行：

  ```bash
  /usr/bin/python3 -B evaluation/test_d435i_local.py -v
  ```

  最终 17/17 通过。复用真实 LZ4 good/bad/truncated/active bag；验证 Ctrl+C 处理、
  Enter、定时停止、先封包再停相机/master、预检失败和磁盘不足不启动录制、
  单个退出失败仍继续其余清理、最终校验失败保留文件，以及 ExactTime 1/4/7对输出和
  CameraInfo。相机/录制子进程和端口替换为夹具，没有启动相机/ROS master。
- ROS bootstrap 用带空格路径的实际 bash/setup 和驱动查询夹具执行 --check-env，
  确认 source 时清空位置参数、参数原样传回系统 Python、只检查依赖、不生成录制目录。
  新单文件可在空目录搬移启动帮助；使用 bash 启动生成脚本以支持 noexec 数据盘。
- 任务前十文件备份/SHA 清单与最终日志、`VALIDATION.json` 在
  `artifacts/d435i_local_20261028/`。旧远程/共享程序和旧测试逐字保持，因此未重复旧套件。
- passing 仅指本机脚本交付与离线验证。TODO：本机 USB 权限/真实出图、30→10 FPS
  Python 抽帧吞吐、实际录制与退出尚未实测；沿用实时预检/最终配对率95%阈值，
  不将抽帧器 ExactTime 语义当成录包完全无丢帧或几何精度证明。
  不涉及核心/现有 ROS 包修改，未运行核心或 ROS C++ 构建。

## RGBD01 D435i + MEM18/perf20 Integration

- Syntax: `bash -n run_d435i_slam.sh`; compile `evaluation/d435i_slam.py`.
- Offline: `/usr/bin/python3 evaluation/test_d435i_slam.py -v` checks calibration,
  depth units, rejected models, sequence gaps and histogram summaries.
- Hardware: `./run_d435i_slam.sh 120 off`, then `./run_d435i_slam.sh 60 on`.
  Require live CameraInfo, matched binary SHA256/file/ldd/loaded core, complete
  10 FPS input, initialized and valid tracking, no resets, latency distribution,
  memory/paging evidence, graceful owned-process cleanup and transfer hashes.
- Static-only scope. Do not mark tracking passed when the scene fails the
  unchanged initialization requirements. Timing-on is diagnostic only.
- Instructions: `docs/d435i-slam.md`; evidence: `artifacts/d435i_slam_20260924/`.

## RGBD02 host live launcher

- Run `bash -n start_rdk_rgbd_slam.sh` and
  `/usr/bin/python3 evaluation/test_rdk_rgbd_live.py -v`.
- Launch `./start_rdk_rgbd_slam.sh`, verify actual SLAM stdout/stderr appears
  before shutdown, then press Ctrl+C. Require owned camera/SLAM/master cleanup,
  host log preservation and board-log fetch. Optional `--duration 60` is a
  convenience timeout; this launcher is not a tracking acceptance test.
- EOF, explicit stop and heartbeat timeout have offline control tests. Actual
  LAN packet loss, power failure and forced process kills are not asserted tested.

## Build Validation

- `F01`: run `./build.sh` from repo root.
- Success means the script completes its third-party builds, extracts
  `Vocabulary/ORBvoc.txt.tar.gz`, and finishes the top-level CMake build.
- `F02`: export `ROS_PACKAGE_PATH` as documented in `README.md`, then run
  `./build_ros.sh`.
- Success means the rosbuild package under `Examples_old/ROS/ORB_SLAM3/`
  finishes building.

## Runtime Validation With `run_stereo_inertial.sh`

- Valid modes on this branch are `mapping`, `mapping_save`, and
  `localization`.
- Valid sensors on this branch are `stereo_inertial`, `rgbd`, and `stereo`.
- The wrapper writes terminal logs to `logs/` only when `--log` is used.
- The wrapper always starts `pidstat` monitoring and `rosbag record`.
- No checked-in `--no-bag` switch exists on this branch.
- Before treating a runtime attempt as meaningful, confirm the required ROS
  inputs are already live for the selected sensor, because the wrapper records
  bags but does not replay them.
- If ROS, camera topics, IMU topics, `pidstat`, or bag-recording prerequisites
  are missing, record the validation as `blocked` or `skipped` instead of
  inventing an unsupported workaround.

## RDK X5 Runtime Evidence Collection

- `collect_slam_debug.sh` is an auxiliary board-side collection script. Copy it
  to the RDK X5 deployment, usually `/userdata/orb_slam/`, and run it from the
  board.
- Use `./collect_slam_debug.sh attach` when `s316_camera camera.launch` and
  `orb_slam3_ros kitti_stereo.launch` are already running.
- Use `./collect_slam_debug.sh launch-slam` to let the script start
  `roslaunch orb_slam3_ros kitti_stereo.launch` and save its stdout/stderr to
  the run directory.
- The script records per-run evidence under
  `/userdata/orb_slam/debug_runs/<run_id>/`, including metadata, environment,
  ROS topic/node/param snapshots, `dmesg` before/after, ROS latest logs, topic
  frequency logs, selected message samples, resource logs, and optional rosbag
  files.
- Use `--no-images` for lower disk and network cost when image frames are not
  required. Use `--no-bag` for log-only diagnosis.
- A passing collection run proves only that evidence was captured. It does not
  prove SLAM runtime correctness unless the captured ROS topics and logs are
  inspected against the feature-specific criteria below.

## Feature-Specific Runtime Notes

- `F03`: run `./run_stereo_inertial.sh mapping stereo_inertial --log`.
- During `F03`, confirm the script prints the selected mapping YAML, creates a
  log file under `logs/`, creates a resource log under `logs/`, starts bag
  recording under `bags/`, and keeps the ROS node alive until the session is
  intentionally stopped or a real runtime error occurs.
- `F05`: run `./run_stereo_inertial.sh localization <sensor> [--log]`.
- During `F05`, confirm the script prints the expected `*_load.yaml` path for
  the chosen sensor and launches the matching ROS executable without extra
  undocumented flags.
- `F11`: first run `./build_ros.sh`, then run either
  `./run_stereo_inertial.sh mapping stereo --log` or
  `./run_stereo_inertial.sh localization stereo --log`.
- During `F11`, confirm `/odometry` is advertised as `nav_msgs/Odometry`,
  confirm `header.frame_id` is `odom`, confirm `child_frame_id` is
  `left_camera_link`, confirm the first published pose after successful
  tracking is near a fresh local origin instead of inheriting the loaded map
  origin, and confirm `twist` is derived from consecutive fresh tracked poses
  instead of staying permanently zero once the robot is moving.
- For `F11`, low-confidence twist is acceptable, but the implementation should
  make that uncertainty explicit through covariance or another documented
  contract instead of pretending the differential estimate is high confidence.
- For `F11`, also confirm held-pose samples during tracking loss use unknown
  twist covariance instead of reporting a confident zero velocity.
- For `F11`, also confirm the node broadcasts a TF chain that makes
  `continuous_map`, `odom`, `left_camera_link`, and `left_camera` visible to
  RViz. The 2026-06-05 Scheme B follow-up changed the root from raw `map` to
  external `continuous_map`.
- The 2026-05-29 implementation session reran `./build_ros.sh` successfully
  after the local-origin odometry change, but host-side runtime validation was
  blocked because `rosnode list` and `rostopic list` could not communicate
  with a ROS master before `run_stereo_inertial.sh` could be exercised.
- The 2026-06-05 Scheme B implementation reran `./build.sh` and
  `./build_ros.sh` successfully after changing the root frame to
  `continuous_map`, but runtime validation still requires a live or replayed
  stereo session that exercises tracking loss and new-map recovery.
- `F12`: first run `./build_ros.sh`, then run
  `./run_stereo_inertial.sh localization stereo --log`.
- During `F12`, confirm `/relocalization_status` is advertised at
  approximately `10 Hz`, confirm the built message carries only
  `timestamp_ns` and `status`, and confirm the runtime output can emit exactly
  `RelocalizationRunning`, `RelocalizationSucceed`, and
  `RelocalizationFailed`.
- For `F12`, `grid_map` already provides a migration template, but validation
  must reject a straight port if it still emits `RelocalizationSucceeded`
  instead of the requested `RelocalizationSucceed`.
- The 2026-05-25 validation session measured `/relocalization_status` at about
  `10.0 Hz` and directly sampled `RelocalizationSucceed`; because the
  localization session converged quickly, the short topic captures did not
  separately observe `RelocalizationRunning` or `RelocalizationFailed` before
  the steady-state publish loop resumed.
- `F13`: first run `./build_ros.sh`, then run either
  `./run_stereo_inertial.sh mapping stereo --log` or
  `./run_stereo_inertial.sh localization stereo --log`.
- During `F13`, confirm `/robot_pose` is advertised as
  `geometry_msgs/PoseStamped`, confirm `header.frame_id` is
  `continuous_map`, confirm `/robot_pose_slam` is advertised as
  `geometry_msgs/PoseStamped`, confirm `/robot_pose_slam.header.frame_id` is
  `map`, confirm `/robot_pose_tracking_ok` is advertised as `std_msgs/Bool`,
  and confirm the published pose follows the live tracked position during
  successful tracking.
- During `F13`, force or capture a tracking-loss interval after the first
  valid pose and confirm `/robot_pose` continues publishing at image callback
  cadence with the last valid continuous pose while `/robot_pose_tracking_ok`
  is `false`.
- During `F13`, confirm `Fail to track local map!` and Atlas new-map recovery
  do not produce a visible reset to the raw internal map origin or optimized
  map frame in `/robot_pose`. Ordinary `map_change` increments should not spam
  correction warnings, and later loop-closure or merge events should not start
  a `/robot_pose` convergence window.
- During `F13`, if the active map id changes during recovery, confirm the log
  uses the map-switch continuity warning rather than restarting any
  convergence window on every routine update.
- During `F13`, inspect `/robot_pose_slam` separately around map merge, loop
  closure, and return-to-start. This topic is allowed to jump with ORB-SLAM3
  optimization; use it to evaluate the optimized SLAM map pose while keeping
  `/robot_pose` reserved for downstream continuous control feedback.
- `F14`: first run `./build_ros.sh`, then run either
  `./run_stereo_inertial.sh mapping stereo --log` or
  `./run_stereo_inertial.sh localization stereo --log`.
- During `F14`, confirm `/robot_pose_map` is advertised as
  `geometry_msgs/PoseStamped`, confirm `header.frame_id` is `planning_map`,
  confirm `/pose_correction/set_enabled` is advertised as `std_srvs/SetBool`,
  and
  confirm both `MyD435i_stereo.yaml` and `MyD435i_stereo_load.yaml` carry the
  documented `PoseCorrection.EnableCorrectedMapPose`,
  `PoseCorrection.CorrectionHorizonFrames`, and
  `PoseCorrection.AutoEnableOnMapEvent` keys.
- During `F14`, ordinary `map_change` increments should not repeatedly restart
  correction. After a tracking-loss interval starts, confirm `/robot_pose_map`
  keeps publishing but does not converge toward raw SLAM again until the
  subsequent merge-equivalent big map event. Then confirm the runtime log emits
  `Starting gradual /robot_pose_map planning_map correction after ORB-SLAM big map event`.
- During `F14`, confirm `/robot_pose_map` transitions over the configured
  `PoseCorrection.CorrectionHorizonFrames` instead of stepping immediately to
  the new raw SLAM pose, confirm `/robot_pose_slam` remains the unblended
  raw/optimized `map` pose for comparison, and confirm `planning_map -> map`
  stays visible in TF for downstream planning consumers.
- During `F14`, test the runtime toggle with
  `rosservice call /pose_correction/set_enabled "data: true"` and
  `rosservice call /pose_correction/set_enabled "data: false"`, then confirm
  the response text and the observed output behavior match the requested state.
- For `F14`, also validate the one-shot path explicitly: with
  `PoseCorrection.EnableCorrectedMapPose` false and
  `PoseCorrection.AutoEnableOnMapEvent` true, a loop-closure or merge event
  should still start and complete one correction window.
- `F04`: validate by reading the documented entrypoints and confirming this
  branch's governance docs do not claim unsupported wrapper options or
  branch-only runtime topics.
- `F10`: validate by confirming `docs/grid-map-port-plan.md` is clearly marked
  as historical reference material rather than current `master` behavior.

## Validation Done Criteria

- Record the exact command that was run.
- Record whether the validation covered the native build, ROS build, or ROS
  runtime path.
- If `run_stereo_inertial.sh` was used, record the selected mode and sensor.
- Record whether live ROS inputs were present.
- If validation was not run, say so explicitly and explain why.

## Known Gaps

- No top-level unit-test entrypoint was found.
- No repository CI workflow file was found.
- No checked-in automated replay wrapper was found for the modified ROS flow.
- `run_stereo_inertial.sh` couples runtime launch with `pidstat` and bag
  recording, so runtime validation currently depends on those external tools
  being available.

## XFeat / RDK X5 Validation

- `F15` official local path:
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps python3 xfeat_rdk_x5/smoke_test.py --output xfeat_rdk_x5/artifacts/local_smoke.json`.
- `F15` fixed-shape export:
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps python3 xfeat_rdk_x5/export_onnx.py`.
- `F15` split-pipeline parity:
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 python3 xfeat_rdk_x5/onnx_smoke_test.py --threads 1`.
- Success requires finite 64D descriptors, non-empty mutual matches, valid
  opset-11 ONNX, raw-output error below `1e-4`, equal selected keypoint sets,
  coordinate-aligned descriptor error below `1e-4`, and equal match-coordinate
  pairs.
- The local checks do not prove BPU execution. BPU completion additionally
  requires `hb_mapper checker`, PTQ/makertbin accuracy evidence, `.bin`
  verification, and a real RDK X5 runtime measurement.

### F16 bag evaluation

- Run from the repository root:
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps:/opt/ros/noetic/lib/python3/dist-packages OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 python3 xfeat_rdk_x5/evaluate_rosbag.py 2026-06-05-09-59-04.bag`.
- The validation reads the bag directly and does not require a ROS master.
- Success means both methods process identical sampled stereo and temporal
  pairs, all aggregate metrics are finite, `summary.json` and
  `per_pair_metrics.csv` are produced, and representative match images exist.
- Verified on 2026-07-30: 130 stereo pairs, 65 temporal pairs, and a 100%
  robust-pair rate for both XFeat and the OpenCV ORB comparison.
- This validates feature/matching behavior only. It does not validate an
  XFeat-integrated ORB-SLAM3 trajectory, BPU execution, or ATE/RPE.

### F18 RDK X5 INT8 deployment

- Generate 100 calibration tensors from the bag with
  `xfeat_rdk_x5/prepare_calibration.py`, passing
  `--rosbag-topic /camera/infra1/image_raw` and keeping `--manifest` outside
  the tensor directory.
- In `openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.6`, run
  `hb_mapper checker --model-type onnx --march bayes-e` on the exported core,
  then run `hb_mapper makertbin --model-type onnx --config
  config/xfeat_backbone_480x640_bayes_e.yaml` from `/data`.
- Success requires all reported core nodes in BPU subgraphs, a generated
  Bayes-e `.bin`, final-output cosine similarity of at least 0.99, successful
  loading by board DNN 1.23.10 / HBRT 3.15.54, finite board outputs, and a
  recorded warmed inference latency.
- Build the board runner from `xfeat_rdk_x5/rdk_x5/` against the installed DNN
  headers/libraries, then run it with `LD_LIBRARY_PATH=/usr/hobot/lib`.
- Run `xfeat_rdk_x5/compare_ptq_outputs.py` on float, quantized, and returned
  board outputs. The board-vs-quantized values must match and every
  board-vs-float output cosine must be at least 0.99.
- Verified 2026-07-31: all 33 reported nodes were on one BPU subgraph; the
  920,712-byte model loaded on `root@192.168.1.10`; 50 warmed BPU0 runs averaged
  13.473 ms; all three board outputs exactly matched the quantized simulation;
  the minimum board-vs-float cosine was 0.990869. Evidence is in
  `xfeat_rdk_x5/artifacts/ptq_board_validation_480x640.json` and
  `docs/xfeat-rdk-x5-research.md`.

### F19 complete frontend and ROS LAN replay

- Export and calibrate the native bag shape with
  `config/xfeat_backbone_544x640_bayes_e.yaml`; checker success requires one
  BPU subgraph with no CPU fallback and PTQ output cosine above 0.99.
- Deploy and compile with `./xfeat_rdk_x5/deploy_board_benchmark.sh`.
- Start `./xfeat_rdk_x5/host_ros_lan.sh master`, run the deployed board
  `run_ros_benchmark.sh`, and verify `rosnode info /xfeat_orb_benchmark`
  reports `192.168.1.10` plus both infrared subscriptions before replay.
- Replay with `./xfeat_rdk_x5/host_ros_lan.sh play
  2026-06-05-09-59-04.bag --delay=1.0`, copy the CSV back, and run
  `xfeat_rdk_x5/summarize_board_benchmark.py`.
- Success requires the expected original indices across 130 stereo pairs and
  65 temporal pairs per method, finite latency/matching/geometry aggregates,
  and no missing sampled anchors. This is a feature frontend test; do not claim
  trajectory accuracy or ORB-SLAM3 integration from it.
- Verified 2026-07-31: 390 result rows covered the expected pairs. XFeat median
  extraction was 60.00 ms/image versus ORB 66.34 ms, while means were 64.85
  and 68.67 ms. Mean stereo end-to-end latency was 185.63 versus 161.98 ms.
  Both methods had 100% robust-pair rate; XFeat had wider inlier coverage but
  fewer strict stereo and temporal inliers. Evidence:
  `xfeat_rdk_x5/artifacts/board_ros_benchmark_544x640/summary.json` and
  `docs/xfeat-rdk-x5-board-benchmark.md`.

### F20 single-BPU stereo/CPU parallelization

- Run `xfeat_dnn_runner` with CLI core `1`; success criteria for BPU1 require a
  completed inference, not merely the presence of `HB_BPU_CORE_1` in headers.
  Also inspect `/dev/bpu_core*` and `/sys/devices/system/bpu/`.
- This board correctly fails the BPU1 probe with `-6000001` and exposes only
  BPU0. The implementation must use separate left/right tensor buffers and
  serialize the BPU0 critical section; concurrent calls on shared buffers are
  invalid.
- Build on board with `cmake --build
  /userdata/xfeat_rdk_x5/benchmark_build -j1`. Replay two anchors using
  `_cpu_threads:=1`, `4`, and `8`; select the measured fastest configuration.
- For the final run, explicitly pass `_max_anchors:=0` because private ROS
  parameters survive node restarts on a persistent master. Replay with the
  one-second connection delay and require 390 result rows over frames 0–641.
- Compare all parallel CSV quality fields with F19. Success requires zero
  mismatches for timestamps, keypoint/match counts, RANSAC/strict inliers,
  ratios, coverage and residual metrics.
- Verified 2026-07-31: CPU=8 was fastest; XFeat stereo extraction wall time
  was 57.78 ms versus parallel ORB 74.48 ms. XFeat stereo end-to-end was
  125.46 ms versus ORB 99.38 ms, a 32.4% improvement over serial XFeat. All
  390 quality rows matched F19 exactly. Evidence:
  `docs/xfeat-rdk-x5-parallel-benchmark.md` and
  `xfeat_rdk_x5/artifacts/board_ros_benchmark_parallel_544x640/`.

### F21 XFeat* semi-dense and fixed-size Fine Matcher

- Export the official `128 -> 512 -> 512 -> 512 -> 512 -> 64` Fine Matcher
  with `xfeat_rdk_x5/export_fine_matcher.py` for `M=256/384/512`. Validate
  fixed input `1xMx128`, output `1xMx64`, and float parity before conversion.
- Convert each model with OpenExplore v1.2.6 / `hb_mapper 1.23.8`. Checker
  success requires a single BPU subgraph and no CPU fallback. Compare float,
  quantized simulation, and board outputs with
  `xfeat_rdk_x5/compare_fine_ptq_outputs.py`; then run 1000 board iterations
  to check determinism, memory stability, and task release.
- The correctness evaluator must use upstream `detectAndComputeDense`, retain
  the per-point scale, concatenate descriptors in the upstream target/fixed
  order, and apply `offset * scale` to the target point. Its single- and
  dual-scale refinement outputs must agree with upstream within `1e-4 px`.
- Do not use rectified RANSAC/vertical error alone to judge XFeat* stereo
  quality. Single-scale dense points share their source-grid rows, so wrong
  disparity identities can still appear as perfect epipolar inliers. Record a
  normalized 9x9 patch NCC side metric and require the geometry test as well.
- Current offline command family:
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps:/opt/ros/noetic/lib/python3/dist-packages python3 xfeat_rdk_x5/evaluate_fine_matcher.py 2026-06-05-09-59-04.bag --feature-mode semi_dense_single --fine-size <256|384|512> --top-k 600 --anchor-parity all --minimum-ncc 0.5 --output <json>`.
- Board M384 full replay uses `_top_k:=600`, `_anchor_stride:=10`,
  `_temporal_gap:=1`, `_max_anchors:=0`, `_cpu_threads:=8`,
  `_xfeat_semidense_single:=true`, the M384 Fine `.bin`, confidence `0.25`,
  and patch NCC threshold `0.5`. Success of this component run requires 520
  rows: 130 each for XFeat* coarse/Fine and ORB stereo, plus 65 each for XFeat
  and ORB temporal controls, spanning source frames 0 through 641.
- Verified 2026-08-03: the full M384 board replay produced all 520 rows. Fine
  raised mean NCC-qualified stereo inliers from `248.29` to `289.42`
  (`+16.6%`) and their ratio from `0.650` to `0.767`; mean patch-NCC median
  rose from `0.654` to `0.792`. Fine stereo latency was `54.89 ms` mean and
  `59.37 ms` P90; its minimum strict and photometric inliers were `330` and
  `176`. The same harness's OpenCV ORB control produced `286.28` photometric
  inliers, `0.383` photometric grid coverage, and `88.70/101.43 ms` mean/P90.
  Evidence: `xfeat_rdk_x5/artifacts/f21_semidense_m384_board_full.csv` and
  `xfeat_rdk_x5/artifacts/f21_semidense_m384_board_full_summary.json`.
- Cross-build the real S316 baseline from
  `/home/ywl/Project/RDK_X5_orb_slam` with
  `RDK_BUILD_JOBS=1 ./tools/build_rdk_arm64_docker.sh`. The local derived image
  `rdk-orb-slam3-arm64-build:v3` is based on the supplied complete
  `ros1_arm64_ubuntu22.04:v3` archive and adds the missing `libepoxy-dev`.
  Stage `ros_stereo` and `liborb_slam3_ros.so` on the board, require zero
  missing `ldd` entries, and verify the uploaded SHA-256 values before replay.
- Verified one production run on 2026-08-03. The instrumented node processed
  all 644 bag callbacks with S316 `600` features and `5` pyramid levels. After
  discarding 30 warmups and one invalid default-Frame statistics row at an
  Atlas map reset, ORB extraction/stereo/frontend/TrackStereo means were
  `57.73/13.24/70.96/88.15 ms`; frontend P90 was `81.76 ms`, mean valid depths
  `215.51`, and mean depth coverage `0.755`. Tracking had one 30-frame,
  `2.97 s` loss segment.
- The run-1 comparison uses `end_to_end_pair_ms` for XFeat Fine, including its
  BPU Fine stage. XFeat mean/P90 was `55.05/59.37 ms`, satisfying the speed
  gate against this run, and mean strict count was `375.4`; strict grid
  coverage was only `0.580` versus ORB `0.755`, so the preliminary frontend
  gate fails and promotion remains false. Evidence:
  `xfeat_rdk_x5/artifacts/f21_orb_production_run1.csv`,
  `f21_preliminary_production_gate_run1.json`, and
  `f21_orb_crossbuild_run1_manifest.json`.
- Superseded by the formal result below. The preliminary comparison mixed all
  ORB depths with strict-filtered XFeat pairs and therefore did not own the
  final coverage decision. EuRoC trajectory gates belong to F23/F24 after
  integration, not to this pre-integration F21 gate.
- Same-frame visualization was verified on bag frame `320` with the real
  ORB-SLAM3 `Frame::ComputeStereoMatches` and official host XFeat* M384 path.
  ORB detected `603` points and returned `270` valid depths over `37/48` cells.
  XFeat* coarse returned `384` strict candidates over `29/48` cells; Fine
  retained `369` strict candidates and the same `29/48` coverage while changing
  median disparity from `0.0` to `3.00 px`. Evidence is under
  `xfeat_rdk_x5/artifacts/feature_visualization/`.
- A second representative-frame check used frame `440` of
  `2026-06-05-09-56-06.bag`. ORB detected `605` left points and returned `224`
  valid depths over `43/48` cells. XFeat* coarse produced `384` candidates over
  `20/48` cells and Fine retained `371` over the same cells; median disparity
  changed from `0.0` to `2.45 px`. The rendered PNG and metrics JSON are under
  `xfeat_rdk_x5/artifacts/feature_visualization_bag_2026-06-05-09-56-06/`.
- Reliability/grid-quota regression verified 2026-08-03. Run
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps:/opt/ros/noetic/lib/python3/dist-packages python3 xfeat_rdk_x5/test_grid_quota.py -v`; all three tests must pass,
  including dense reliability shape/order, deterministic hard quota, and
  reliability-aware guided matching.
- With extraction grid `8x6`, automatic per-cell maximum `13`, match maximum
  `8`, top-k `600`, and M384, bag frame `440` produced raw/coarse/Fine coverage
  `48/48`, `47/48`, and `47/48`, with coarse/Fine strict counts `325/285`.
  This replaces neither the full board replay nor the formal F21 gate.
- Five-anchor smoke evidence in
  `xfeat_rdk_x5/artifacts/f21_grid_quota_host_smoke_09-56-06.json` requires
  Fine parity `passing`, no empty pair, mean Fine strict count `298.2`, and
  mean/min Fine coverage `0.971/0.938`. The RDK-X5 CMake build reached 100% for
  `xfeat_ros_benchmark`; a runtime smoke was not completed because no ROS
  master was listening at the test URI.
- The completed hard-grid board replay is
  `xfeat_rdk_x5/artifacts/f21_grid_quota_m384_board_full_20260803.csv`.
  It must contain 520 data rows with exactly 130 stereo rows per method and 65
  temporal rows per method, span source frames 0 through 641, and align by
  `(frame_index, stamp_ns)` with the pre-grid replay. Its CSV SHA-256 is
  `eb59d7d63ed450a5742c9904b7e5727b0d28e7f8f655def3e93aec68c18850c2`.
- Run the common stereo-depth/temporal-Fine/PnP diagnostic with:
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps:/opt/ros/noetic/lib/python3/dist-packages python3 xfeat_rdk_x5/evaluate_stereo_vo.py 2026-06-05-09-59-04.bag --output-dir xfeat_rdk_x5/artifacts/f21_stereo_vo_full`.
  It samples the same 65 `(anchor, anchor+1)` pairs for hard-grid XFeat+Fine,
  no-hard-grid XFeat+Fine, and an OpenCV ORB control; forms previous-frame 3D
  points from NCC-qualified stereo depth; refines current XFeat observations
  with the official Fine Matcher; and sends all methods through identical
  `solvePnPRansac` settings. Require 65 rows per method, finite metrics, no
  fewer-than-30-inlier pair, and 65/65 PnP success before treating a candidate
  as tracking-robust.
- The host evaluator's `extract_pair_ms` is diagnostic only: it runs FP32
  Python inference and includes different selection overheads. Use the board
  replay for latency gates. Its ORB rows are also a common-PnP OpenCV control,
  not the production ORB-SLAM3 trajectory baseline.
- Verified 2026-08-03: hard-grid XFeat+Fine failed the tracking diagnostic
  with `99.38` mean PnP inliers (`49.6%` of the ORB control), `98.5%` success,
  and one 24-inlier pair at frame 561. No-hard-grid XFeat+Fine returned `213.0`
  mean PnP inliers (`106.4%` of ORB), 65/65 success, and no fragile pair.
  This rejects the current hard quota but does not reject Fine Matcher. A
  softened minimum-per-cell plus global reliability fill must be evaluated
  before another board promotion attempt.
- The four-way extension additionally enables `xfeat_sparse_coarse`: official
  sparse XFeat with 600 features, no Fine Matcher, no quota, MNN/ratio-tested
  rectified stereo matching, and 16-pixel local temporal matching. The full
  result under `xfeat_rdk_x5/artifacts/f21_stereo_vo_fourway/` must contain 65
  aligned finite rows for each of the four methods (260 data rows total).
  Verified 2026-08-03: sparse XFeat averaged `83.0` PnP inliers (`41.4%` of
  ORB), succeeded on 63/65 pairs, and had 9/6 inliers at fragile frames
  511/561. No-hard-grid XFeat+Fine remained the only XFeat variant with 65/65
  success, no fragile pair, and mean PnP inliers above the ORB control.
- Deployed sparse-600 validation uses the board runner defaults: `_top_k:=600`,
  `_xfeat_fixed_matches:=600`, `_xfeat_semidense_single:=false`, empty
  `_fine_model_path`, and both quota parameters zero. The startup log must say
  `top_k=600 semidense_single=0 ... max_per_cell=0
  extraction_max_per_cell=0 fine=disabled`. Replay one anchor and require both
  XFeat keypoint columns to equal 600 and every Fine timing column to remain
  zero. Verified 2026-08-03 in
  `xfeat_rdk_x5/artifacts/f21_sparse600_board_smoke.csv`: both stereo rows and
  the temporal row contain `600/600` XFeat points; the temporal row has 429
  matches and 383 RANSAC inliers.
- Formal F21 verification passed 2026-08-03 with the frozen configuration in
  `xfeat_rdk_x5/config/f21_xfeat_star_m384.yaml`: extraction quota `-1`, loose
  match quota `16`, M384, and Fine confidence `0.20`. Five alternating runs
  each produced 644 ORB callbacks and 520 XFeat records. Combined mean/P90
  frontend latency was `70.505/81.827 ms` for ORB and `54.633/59.402 ms` for
  XFeat. The common real-S316-ORB/XFeat estimator passed NCC count/coverage,
  PnP count/coverage, 65/65 success, and zero-fragile-pair gates. The M384
  threshold-0.20 PTQ check passed, and two sequential 1000-iteration board
  runs returned identical output hashes and stable peak RSS. Authoritative
  evidence: `docs/f21-gate-report.md` and
  `xfeat_rdk_x5/artifacts/f21_gate_final.json`.

## F25 RDK X5 stereo gate diagnostic (2026-08-06)

- Cross-build command: `RDK_BUILD_JOBS=4 ./tools/build_rdk_arm64_docker.sh`
  from `/home/ywl/Project/RDK_X5_orb_slam`. The ARM64 node/library hashes were
  `d6df0d5e...f71ccb` and `3f4de91f...e085c54f`. They were copied to the
  independent board directory
  `/userdata/orb_slam/f25_stereo_gate25_sparse_20260806`; the existing staging
  and devel artifacts were not replaced.
- Runtime controls: live S316 camera threads on CPUs 0-1, ORB-SLAM3 on CPUs
  2-7, Pangolin viewer disabled, host bag playback at 1.0x, and image topics
  remapped under a unique `/baggate25_sparse_*` prefix so the node could not
  subscribe to the live camera image topics.
- `2026-06-05-09-59-04.bag`: 643 callbacks, all tracking state 2, one Atlas
  map with 135 keyframes and 2822 saved map points. Frame 523 reached exactly
  25 local-map inliers and was accepted; its pose step was 0.062627 m and
  1.613700 deg, with inlier reprojection mean/P90 1.264135/2.374674 px and
  chi-square mean/P90 1.390776/2.646152. The former threshold-30 run entered
  state 3 at frame 528 and created a new map after 30 lost frames.
- `2026-06-05-09-56-06.bag`: 883 callbacks, 30 state-3 frames, then a new map.
  Failure began at frame 432 when `TrackReferenceKeyFrame` found only 9 BoW
  matches. The preceding frame had 50 local-map inliers, and no 25-29-inlier
  diagnostic occurred before failure, so changing the 30-inlier local-map gate
  could not have caused or prevented this loss. The threshold-30 CPU-isolated
  run had remained state 2 throughout, exposing asynchronous LocalMapping/map
  state repeatability as a remaining runtime variable.
- Mean callback time changed from 84.002 to 84.565 ms on `09-59-04` and from
  96.592 to 97.366 ms on `09-56-06` versus the threshold-30 CPU-isolated runs.
  The sparse diagnostic overhead is therefore below 1%, but repeat runs are
  still required before promoting 25 as a production threshold.
- Host evidence and SHA-256:
  `/tmp/orb_slam3_095606_rate1_camera_cpuiso_gate25_sparse.csv`
  (`1dff179a...91650bf`),
  `/tmp/orb_slam3_095606_gate25_sparse_diag.log`
  (`a2ce0700...65d6a9e`),
  `/tmp/orb_slam3_095904_rate1_camera_cpuiso_gate25_sparse.csv`
  (`d9f51beb...ebb8d50`), and
  `/tmp/orb_slam3_095904_gate25_sparse_diag.log`
  (`cecd50cd...7b74e5d`).

## F22 RDK X5 Hybrid stereo replay (2026-08-10)

- Host core build: `./build.sh` passed after adding the provider seam and
  frontend statistics. Standalone `-std=c++17 -Wall -Wextra` compilation also
  passed for `xfeat_frontend.cpp` and `xfeat_hybrid_depth_provider.cpp`.
- RDK cross-build: `RDK_BUILD_JOBS=1
  /home/ywl/Project/RDK_X5_orb_slam/tools/build_rdk_arm64_docker.sh` passed.
  The first `-j4` attempt was rejected as evidence because `cc1plus` was killed
  while compiling `Optimizer.cc`; the retained single-job retry completed both
  catkin packages. Node/library hashes are `5a507a46...1eaf9b` and
  `56987b52...7766f9`; board `ldd` reported no missing dependency.
- Board smoke: an 8-second slice of `09-46-39` produced 89/89 state-2 and
  pose-valid callbacks. XFeat Fine added 14-17 depths per sampled frame and
  initialized both BPU models without error.
- Full replay command: `./xfeat_rdk_x5/run_f22_hybrid_ab.sh`, using unique ROS
  topic prefixes, viewer disabled, CPU affinity 2-7, and 0.5x playback. The
  slower rate is required because the validation Hybrid intentionally computes
  both complete ORB and XFeat frontends; it prevents queue drops from becoming
  a confounder. Source image-pair counts are 996, 811 and 715, while recorded
  ORB/Hybrid callbacks are `994/994`, `803/810`, and `704/713`.
- Common-timestamp verification covers 2,496 frames. ORB keypoint mismatch is
  zero, valid-depth regression count is zero, and mean Hybrid depth deltas are
  `+18.39/+17.12/+16.13`. Mean tracked-inlier deltas are
  `+18.79/+13.88/+17.82`.
- Tracking-loss frames/segments are ORB versus Hybrid `0/0` and `0/0` for
  `09-46-39`, `34/1` and `34/1` for `09-48-14`, and `34/1` and `0/0` for
  `09-50-14`. Map counts are `1/1`, `2/2`, and `2/1`. No runtime error marker
  was found in any log.
- Trajectory evidence: first-bag maximum consecutive translation/rotation is
  ORB `0.064 m/2.91 deg` and Hybrid `0.069 m/3.40 deg`. Both second-bag runs
  reset maps. On the third bag ORB reaches `2.945 m/95.25 deg` across its reset,
  while Hybrid remains at `0.100 m/3.14 deg` and stays in one map.
- Reprojection evidence uses the existing sparse near-gate diagnostic and is
  conditional on its trigger. Mean of per-row reprojection mean/P90 is
  ORB/Hybrid `1.24/2.30` versus `1.45/2.76` px, `1.32/2.45` versus
  `1.27/2.40` px, and `1.35/2.53` versus `1.24/2.34` px for the three bags.
- Timing: Hybrid itself averages `53.42/53.26/53.34 ms`; total callback mean is
  `161.85/162.41/155.92 ms`, versus ORB `106.46/107.11/100.72 ms`. F22 passes
  the specified loss/observation gate, but this dual-frontend bridge is not an
  11 Hz real-time production mode.
- Machine-readable evidence:
  `xfeat_rdk_x5/artifacts/f22_hybrid_ab/summary.json` plus the six paired CSV
  and log files. The original unindexed `2026-08-06-09-50-14.bag` was not
  modified; the tested indexed copy is `/tmp/f22_2026-08-06-09-50-14.bag`.

## F23 Full XFeat frontend validation (2026-08-10)

- Descriptor unit check:
  `g++ -std=c++17 -Wall -Wextra -Werror -Iinclude
  xfeat_rdk_x5/test_feature_descriptor.cpp src/FeatureDescriptor.cc
  $(pkg-config --cflags --libs opencv4) -o
  /tmp/f23_test_feature_descriptor && /tmp/f23_test_feature_descriptor`.
  Success requires exact ORB Hamming checks, finite float cosine distances,
  descriptor-family rejection, and the expected float medoid.
- Host build check: `cmake --build build --target ORB_SLAM3 -j1` from the
  repository root. Script check: `bash -n
  xfeat_rdk_x5/run_f23_xfeat_smoke.sh`. Also require `git diff --check`.
- RDK cross-build check: from `/home/ywl/Project/RDK_X5_orb_slam`, run
  `RDK_BUILD_JOBS=1 ./tools/build_rdk_arm64_docker.sh`. The package must use
  `-DORB_SLAM3_SOURCE_DIR=/ros1_project/orb_slam3_dbow3` so the tested core is
  this repository rather than a stale mirror.
- Board smoke/full replay helper:
  `F23_ARTIFACTS=<output> F23_DURATION=<seconds>
  ./xfeat_rdk_x5/run_f23_xfeat_smoke.sh <bag>`. It uses the isolated stage
  `/userdata/orb_slam/f23_xfeat_20260810`, 0.5x playback, CPUs 2-7, a unique
  topic prefix, Pangolin disabled, and copies the per-callback CSV back.
- The 8-second temporal-Fine smoke passed 87/87 state-2 callbacks with no
  `Fail to track local map!`. Evidence:
  `xfeat_rdk_x5/artifacts/f23_xfeat_smoke_temporal_fine/`.
- The complete `2026-08-06-09-46-39.bag` temporal-Fine run is failing
  evidence, not a pass: 981 callbacks, 682 state-2, 299 state-3, 29 loss
  segments, 2.97 s maximum loss and eight maps. Evidence:
  `xfeat_rdk_x5/artifacts/f23_lawn_full_094639_temporal_fine/`.
- The authoritative F23 gate remains five runs each of MH_01, MH_03 and MH_05,
  requiring 5/5 initialization, ATE/RPE no greater than 1.05x ORB, and no
  worse loss duration. No matching local dataset directories were found on
  2026-08-10, so that gate is blocked and F23 must not be marked passing.

## BR01 branch and archive validation (2026-09-07)

The separation check compares inherited XFeat source bytes against `419878d`,
the four runtime core modifications against their pre-migration backup, and all
54 saved source/config files against `backup_manifest.json`. The archived 7,626
file/link records were verified after relocation against SHA-256, size, mode or
symlink target. Existing branch refs are compared with the saved ref inventory.

Run the existing F23 descriptor test on its branch:

```bash
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude \
  xfeat_rdk_x5/test_feature_descriptor.cpp src/FeatureDescriptor.cc \
  $(pkg-config --cflags --libs opencv4) -o /tmp/f23_branch_descriptor_check
/tmp/f23_branch_descriptor_check
```

Run `bash -n` on migrated shell scripts and use Python `ast.parse` on migrated
Python files. Check `git diff --check`, the source/artifact ownership, unique
feature IDs, expected branch ancestry, ignore rules and final clean statuses in
both core worktrees and the separate ROS/build repository. Validate build-script
default and override paths without starting Docker or ROS.

Results are recorded in the local archive's `validation.json`; branch/source
routing is documented in `docs/branch-layout.md`.

Full host/ARM builds and board replays were not rerun: this task preserves the
existing C++ source bytes and organizes branch ownership. Only runner data-root
selection and build-script default workspace/core routing were adjusted. The
F23 long-run and EuRoC limitations remain blocked; imported runtime passing
states describe the earlier recorded experiments, not new runtime verification.

Verified 2026-09-07: 79 inherited source files match `419878d`; all four pending
runtime core files and four unchanged ROS/build files match their saved bytes.
The descriptor test passed, 9 shell scripts and 10 Python modules passed syntax
checks, and build defaults/overrides, feature ownership and ignore rules passed.
The new local branch commit IDs and clean statuses are recorded in the archive
after committing. Original branch refs are retained.

## LM01 Active local map (runtime worktree)

The implementation lives in `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_memory_lifecycle/core`,
not this IDE checkout's XFeat core. Run `bash memory/tools/build_localmap_host.sh c1`
from the runtime outer repository; the fixture is `memory/devel-host-lm01-c1/lib/orb_slam3_ros/local_map_budget_probe`.
The runtime `memory/LM01.md` records contracts, stage gates and board artifact ownership.
Source/binary provenance and results are archived here under `artifacts/local_map_budget_20260924`.
C1 native/ARM probes and transaction fixtures pass; sanitizer qualifications are recorded in c1/checks.json. Production integration and later quality/realtime gates remain in progress.


## LM01 C2 — functional gate only (2026-09-24)

Runtime RGB-D full/KF/shadow transaction integration, true KF cap and legacy
counters now pass native full/KF80/shadow 120-frame tests, original 27 ARM probes,
new cap/competition/invalidation fixtures, TSan races/small-lock checks and
ASan/UBSan/LSan fixture with the recorded C1 dependency exceptions. A narrow
recursive identity barrier serializes retirement/fusion/transfer with active
transactions; ordinary metadata updates remain concurrent. This fixes actual
extra non-OK frames found during integration. Full-System LSan remains open.

Board full and KF80 short runs are 120/120 OK, but P95 112.448/115.138 ms
versus perf20 prefix 90.928 ms: performance FAIL, feature default OFF. Next is
C2.1 transaction overhead reduction, then C3 event-based controller, C4 selector
comparison, C5 MP/statistics/cohorts and C6 formal matrix. Preserve all failures.
Evidence: IDE artifacts/local_map_budget_20260924/c2/checks.json and runs/.
Runtime memory/LM01.md specifies the refined synchronization contract.

## GF01 upstream local-map hashing reproduction (2026-09-28)

- Independent source/build/inputs are under
  `/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_repro_20260928`.
  Run commands and exact upstream/local hashes are in the archived
  `artifacts/gf_orb_slam2_repro_20260928/README.md` and `REPORT.md`.
- Four CPU targets built; 3 accuracy evaluator tests pass; input regeneration
  matches frozen manifests; bucket-container diagnostic verifies the current
  unbounded storage / bounded query behavior.
- Completed 15 EuRoC + 9 corrected-input TUM rounds, all source frames consumed.
  One narrow-OTS EuRoC run has 68 LOST frames. Failed measurements are retained;
  speed/robustness claims require the full configuration to pass, not just
  selected successful runs. No stable end-to-end speed benefit was found.
- `validation.json`, `comparison_summary.json`, per-matrix `analysis.json`,
  `run_summary.csv`, plots and SHA256 raw indices are archived. TUM accuracy
  only covers 24.3% of online poses because of a long ground-truth gap.
- No ARM/ROS callback gate, GF concurrent-lifecycle sanitizer gate, full-paper
  benchmark or combined Good Feature Matching/Good Graph gate was run.
  GF01 passing refers to reproducible evaluation delivery only.

## GF02: paper-described MIH policy experiment

Independent root: `/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_paper_20260928`; source `33e5064`, GF01 parent `39e794a`. Exact commands are archived in `artifacts/gf_orb_slam2_paper_20260928/README.md`.

- Four CPU modes built; release plus focused ASan/UBSan/LSan and TSan fixtures passed for capacity/refresh/eviction/migration/partial membership/concurrent query/reset. These fixtures do not certify the full System/MapPoint lifecycle. Three accuracy-tool tests passed.
- Four 400-frame smoke runs passed, but did not yet activate querying. Formal EuRoC and TUM matrices each completed 12 rounds, 95,628 input frames in total. All records are contiguous/finite, timestamps increase, and frozen source/binary/config/driver/input-list hashes match.
- 23/24 rounds remained OK after initialization. TUM `1_paper_ots` lost from index 28 through 5056, with zero hash queries throughout; its fast LOST frames are retained and excluded from acceleration claims.
- EuRoC baseline / bounded buckets / bounded+all-descriptor API P95: 35.609 / 63.127 / 64.263 ms. TUM: 27.550 / 52.368 / 59.477 ms; bounded-only has a failed repeat. Production promotion fails. GF02 `passing` means the implementation/evaluation/evidence task is delivered, not performance or full-paper acceptance.
- Report, complete per-run JSON/CSV, raw-file SHA256 indices, plots, patches and provenance are in `artifacts/gf_orb_slam2_paper_20260928`. `harness/audit_freeze.py` reproduces the final integrity audit.
- Not executed: full SLAM sanitizers, ARM/ROS timing, loop closure, map merge/save/reload, long-duration NewCollege or all EuRoC sequences. TUM GT covers only 1227/5057 poses. Production ORB3/LM01 and board files are unchanged.

## GF03: module factorial evaluation

Independent root `/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_modules_20260928`; source `1ad2246`, parent `33e5064`. Build/runtime workflow and prerequisites are in `artifacts/gf_orb_slam2_modules_20260928/README.md`.

- All eight CPU target builds completed; actual F active/skip, H query/OTS and G selection counts exist for every enabled combination on both full sequences. G is fixed-cardinality30/candidate60, not the dynamic anticipation variant. 16 functional runs /7600 frames passed; their timing is excluded. Three trajectory-tool tests passed.
- User requested one pass per mode/dataset. `harness/run_matrix_once.py` completed EuRoC2912×8 and TUM5057×8 = 63,752 frames. Existing baseline retained; cancelled partial F archived separately. All16 input sequences complete/contiguous/finite with increasing timestamps; 15 remain OK after initialization.
- `analyze.py`, `factorial_report.py`, `detail_tables.py`, `diagnose_euroc.py`, `audit_freeze.py` and `archive_evidence.py` completed. Original source/config/binary/input-list hashes and previous GF01/GF02 binaries are unchanged. No candidate passes all single-run latency/accuracy gates; delivery passing is not optimization acceptance.
- TUM FHG fails at index4126, remaining931 frames LOST. First failure is in upstream motion/reference tracking, with declining prior inliers. Loop-off also prevents normal keyframe database population, so recovery is not validated. Failed timing is retained and excluded from gains; partial ATE is not comparable to complete trajectories.
- Diagnostic gap: FG/HG/FHG executables have historical replay driver d85f25dc; other modes have 2a5138f5 with input-lateness column. Incremental build did not recompile those three. File-freeze audit alone does not prove uniform compiled-source provenance. Primary tracking clocks/poses/counters exist; missing lateness remains unmeasured. See diagnostic_gaps.json, preserved source snapshots and driver_diagnostic_delta.patch.
- TUM GT covers only24.3% of complete outputs, maxgap129.865s. Single-pass fixed order/ondemand host cannot establish repeated-run stability. No GF03 full-System sanitizer, dynamic Good Graph, complete relocalization/loop, save/reload, full benchmark suite or board/ROS gate. No maps or board files generated.

## GF04: official and delivered ORB-SLAM3 controls

See artifacts/orb_slam3_controls_20260928/README.md and REPORT.md. Separate GCC9.4/
OpenCV4.2 builds from official4452a3c4 and delivered4d6df6a2; both use identical loop-off
crash protection plus a diagnostic friend, without frontend/backend algorithm edits.

- `harness/verify.py --phase smoke/formal`:8 smokes1200 frames and8 full rounds31876
  rows pass input/schema/source/dependency checks. Official RGB-D has92 nonOK and a map
  split; input PASS is not tracking acceptance.589 frozen files remain unchanged.
- `harness/fullsystem.py build/smoke/formal`, `verify_fullsystem.py`,
  `analyze_fullsystem.py`:3 loop-on RGB-D smokes450 rows;3 full rounds15171 rows, zero
  nonOK and one loop each. All3 exit7(60s backend/GBA shutdown wait); complete shutdown FAIL.
  GBA NORMAL-level logs are suppressed by QUIET; no final per-thread-state diagnostic.
- `test_accuracy.py`:3 tests pass. `analyze.py`, `tables.py`, `all_results.py`, `archive.py`
  produce JSON/CSV/plots and184 SHA-indexed compressed raw files. Matched800 loop-on
  delivered ATE+5.5% vs official on sparse TUM truth; quality gate NOT accepted.
- Total11 formal attempts47047 input rows. GF03 bridge change+0.44%/-5.48% P95 on EuRoC/TUM;
  old factorial data stays historical. Single pass, host native API, PNG decode excluded.
- Not run: ARM/ROS callback90ms target, repeats, full-System sanitizers, full-GT suite,
  save/reload or clean GBA lifecycle. No board deployment or generated Atlas maps.


## RGBD09 有界研究交付验证（2026-10-10）

运行 `python3 -B artifacts/rgbd_a1_a3_impl_20261010/analysis/verify_delivery.py` 检查本阶段成对来源、39 TU冷构建、实际加载与每轮证据SHA、独立提交补丁、历史主文档保留及完整归档索引。实际方法、ASan/UBSan/定向LSan、ARM固定工作量和T1–T6由归档命令及原始结果证明，不能用此完整性脚本替代运行测试。

RGBD09 passing只表示实施、一次诊断和唯一原型有界筛选交付完整；A1 writer/scheduler未决、输入缺失、观测侵入以及正式性能/精度/生命周期/完整System LSan门分别保留，不代表正式晋升通过。证据入口为 `artifacts/rgbd_a1_a3_impl_20261010/REPORT.md`。


## RGBD11 加速交付版与 B_common 地图规模对照（2026-10-12）

研究入口：[REPORT](../artifacts/rgbd_scale_compare_20261012/REPORT.md)。隔离双仓各自加入一致core观测和共同完整回调Recorder，旧/当前ARM冷编译32/40个核心源；外层CMake源清单/依赖修正与已编译输入一致，但未另跑完整catkin。

- 定向观测/Recorder/分析自测、四短轮120/120以及ABBA四长轮均完成；每轮实际库/输入/记录SHA及身份核对通过。
- 全程/固定四段/100源分箱/阶段嵌套/两路pose和描述性Spearman独立复核263094项通过，报告数值复核223项通过。
- 只读归档验证：`python3 artifacts/rgbd_scale_compare_20261012/tools/verify_archive.py`。
- 在独立目录重新归约：`python3 -B artifacts/rgbd_scale_compare_20261012/tools/analyze_scale.py --output /tmp/rgbd_scale_recheck_20261012`。
- 八轮精确参数、退出与回传证据在实验runs目录，冷构建命令在builds/RESULT和manifest，源码恢复/元数据修正在source/DELIVERY.md；收尾及最终验证见FINAL_VERIFICATION.json。

旧/当前长轮处理2882/2881与2847/2825对，全部不足2910，P95都大于90ms。passing仅指完成相关性研究，不代表生产验收。完整System退出泄漏、真值轨迹精度、活动GBA/保存加载及正式重复矩阵未验证，不改写历史结论。统计相关不等于因果，网络/队列时间不在完整回调计时内，CPU不含ORB辅助线程。


## RGBD12 ORB 速度与波动稳定化研究（2026-10-12，研究完成）

固定方案与门见 `artifacts/orb_stability_20261012/PLAN.json`。独立探针固定700/1.2/5/20/7，比较全KeyPoint字段、输出顺序、monoIndex、描述子形状/类型/全部字节；禁止用数量相同替代质量证明。完整operator wall与process CPU分开，图片I/O及结果归档在计时之外；首帧保留，末10帧分箱保留。

构建入口 `python3 artifacts/orb_stability_20261012/tools/build_probe.py <baseline|row_tiles_01> <host|arm> <new_build_id>`；运行入口 `python3 artifacts/orb_stability_20261012/tools/run_board_from_host.py <new_run_id> <baseline|row_tiles_01> --list all`。每次使用新ID，不覆盖旧结果。板端默认OpenCV线程策略保持，当前实际为8线程；ORB执行者仍为3。probe、统计器、run结果均保存完整命令与SHA。

比较使用 `tools/compare_probe.py --list ... --baseline-csv ... --baseline-bytes ... --candidate-csv ... --candidate-bytes ... --out <new_json>`；三对完整结果齐后运行 `python3 artifacts/orb_stability_20261012/tools/analyze_stability.py`。事前筛选要求每对P95及固定100源分段均值极差均至少改善10%、均值不退化、P99不退化超过3%；这不是整机90ms正式验收门。三对ARM完整提取均通过该门；host全输入用于质量，其分箱极差改善未达到10%，不能推广为host性能通过。

安全测试保留严格LSan失败：仅baseline控制和候选均观测到宿主TBB 792 B/3次分配，未关闭检测/加suppression。不能从行为一致或未报地址错误得出完整系统无泄漏。旧失败和正式验收缺口继续保持。


完整SLAM已完成候选120短轮与基线→候选一对2910长轮。两侧scale_detail=false，阶段／地图计数未采。长轮处理2831→2881，P95113.340→104.974ms，但相邻绝对差P9518.099→20.642ms、最大值及第三段均值退化；完整输入与90ms正式门均失败。研究完成不等于整体稳定化成功。逐帧归约见 `integration/analysis/pair_01/SUMMARY.json`，独立复核94164项通过。

只读复核命令（不重建、不连接板端、不重写历史归档）：

```bash
python3 -B artifacts/orb_stability_20261012/tools/verify_delivery.py --integration-summary integration/analysis/pair_01/SUMMARY.json --history rgbd11 --history a1
```

核验器首次真实集成检查将字符串传入Path API而失败；修正Path参数后保留新的FINAL_VERIFICATION_02，原失败另列。该退出0仅指证据与研究筛选一致，严格LSan、整机正式性能及全生命周期缺口不变。未来回放须另建run_id；不得覆盖冻结源码、原始记录或既有归档清单。


## RGBD13 ORB 25 ms 实施（2026-10-13，整机门受阻）

最终保留 C1 FAST 工作区复用；C2按有限诊断跳过，C3/C4性能门拒绝，C5原生ARM浮点门拒绝。C1 host/ARM FAST4265差分、198提取器回归、100生命周期及异常恢复通过；host全2910对B_current/B_row一致，最终ARM八对所有字段及描述子字节一致。定向TSan通过；严格LSan保留基线/候选TBB同栈792B/3分配失败，完整System安全及正式质量门不具备。

唯一final01实际16轮22180调用完成。长包运行指标中位数mean30.454849→27.537488ms、P9538.037288→34.227269ms；候选三轮超25ms数量1771/1760/1764。两压力轮各1000/1000超时，三冷轮各120/120超时；首帧及慢值保留，严格25ms目标失败。66PNG+66SVG和全部逐帧CSV已归档。

双方整库零旧对象冷构建、成套隔离部署通过（B_row40+2，P41+2）。I0八轮均120/120新鲜有效输出、自然退出；B_row观测P95增量3.976455ms超过1.693411ms门，总开销门失败，I1/I2未运行。实际ARM每帧记录1000B、短轮记录容量227328B，OpenCV4.5.4默认8线程。共有槽位诊断计数在Moff/M0仍写入，相关列不作为有效地图工作量快照；未测收益，不靠此推断I0失败原因。

入口：[REPORT](../artifacts/orb_25ms_impl_20261013/epoch01/REPORT.md)、[源码与回退](../artifacts/orb_25ms_impl_20261013/epoch01/source/DELIVERY.md)、[FINAL_STATE](../artifacts/orb_25ms_impl_20261013/epoch01/FINAL_STATE.json)。旧研究、原生产部署和已有文档内容保留，没有生产晋升。


## RGBD14 A1逐调用与锁持有者诊断（2026-10-14，研究交付完成）

入口：[REPORT](../artifacts/rgbd_a1_lockdiag_20261013/REPORT.md)、[运行核验](../artifacts/rgbd_a1_lockdiag_20261013/operations/DELIVERY_RUNTIME_REVIEW.md)、[独立1101时间线核验](../artifacts/rgbd_a1_lockdiag_20261013/analysis/baseline_audit/FINAL_ALL_TIMELINE_REVIEW05.md)。阈值严格A1 wall−Tracking CPU>10ms用于诊断，不放宽原全部2910输入/90ms完整回调P95及质量门。

覆盖全部25个connections获取位置、每一次过滤isBad计时/逐帧汇总、慢细与全部锁记录/嵌套/持锁CPU。host与ARM probe03各十例通过，涵盖持锁计算/暂停/嵌套、早返回、时钟失败、容量/全池、精确压缩、并发快照和gzip写入失败；定向包装探针不等同完整System安全通过。04 ARM41+2冷构建，05只改私有存储源，全部公共头/其他对象/依赖与参数身份一致，匹配增量重编译＋成套重链接，ABI1004强符号检查通过。

八短轮均120/120与正常退出，最终05短P95影响+20.203%；四长轮严格关→开→开→关，完整发送2910对，处理2812/2690/2701/2816、缺失98/220/209/94，开启两轮额外45/35个非新鲜输出。原始约4200万事件独立流式核对，1101时间线6853995项独立核对通过，时钟/溢出/未完成记录为零；快速函数细记录按阈值未存不是记录丢失。全帧及全部11640源行均保留，输入缺失单列。

主证据1045持锁CPU占主导、5持有者低CPU原因未知、51混合未知；第二持有者嵌套2例累计0.057833ms。CPU是被测整段，含观测成本；所有持锁/嵌套/请求相交按并集，释放只给前后区间。未解释差额下界累计2249.321ms，不把下界或剩余时间当精确调度/锁睡眠归因。长P95+30.258%、缺失+4.072个百分点、地图规模改变，观测侵入失败；原运行机制未确认，正式性能不通过。板端可访问调度追踪和schedstat不可用，明确保留调度、二级锁覆盖和快速nested第二持有者缺失未逐请求独立检索的证据缺口。

只读复核归档：

```bash
python3 -B artifacts/rgbd_a1_lockdiag_20261013/tools/verify_archive.py
```

重新归约使用新输出目录：

```bash
python3 -B artifacts/rgbd_a1_lockdiag_20261013/tools/analyze_lockdiag.py --run long_on_01 --output /tmp/rgbd14_reduction_recheck
```

实际收集超时、离线SQL全扫与修订/终止143均保留；修订逐字段等价、不删短嵌套、不增加回放。12轮来源/六退出/回传/完整host观察及24板端进程收尾已核验。TODO：原基线因果确认、合格低开销观测及每调用调度证据仍未完成；本轮按两诊断长轮结束，没有生产修复或去锁。


## RGBD15 逐帧回调构成可视化（2026-10-14，离线交付通过）

入口：[交互图](../artifacts/rgbd_callback_breakdown_20261014/callback_breakdown.html)、[主图PNG](../artifacts/rgbd_callback_breakdown_20261014/long_off_01.png)、[数据核验](../artifacts/rgbd_callback_breakdown_20261014/VALIDATION.json)、[浏览器核验](../artifacts/rgbd_callback_breakdown_20261014/BROWSER_VALIDATION.json)。六轮来源为RGBD14四长轮与RGBD11 current两长轮，主图long_off_01；全部输入身份/缺失/非新鲜保留。开启细观测两轮标明侵入失败，不能代表原基线性能。

```bash
python3 -B artifacts/rgbd_callback_breakdown_20261014/generate_charts.py
python3 -B artifacts/rgbd_callback_breakdown_20261014/check_browser.py
git diff --check
```

生成器使用本机numpy/Matplotlib和Noto CJK字体，只读原CSV并在生成前后核对12个输入SHA与历史归档一致。17460个原定源位置、16691回调均完成身份/计时端点/阶段有效性/互斥加总检查；时间误差最大5.685e-14ms，百分比误差最大4.264e-14。未执行阶段CSV为空、缺失回调留空且无插值；非新鲜输出未剔除。阶段按已测wall互斥相加，A1分CPU与差额，其他为完整回调减已测项，不叠加父阶段或后台BA。

生成六轮全图及两个细节共8组PNG/SVG、六份逐源CSV和不依赖网络的HTML；静态图、浏览器全图与505尖峰细图目视通过。浏览器脚本依赖本机google-chrome与Python websocket；临时本地Chrome验证35项（六轮切换、悬停数值、未执行/缺失/非新鲜、区间表单、缩放、真实指针拖动、复位），无JavaScript异常。首次沙箱socket限制导致浏览器启动失败，保留browser_attempt01.log；同脚本获自动批准后在非沙箱本地运行通过。没有执行SLAM构建/回放或更改旧归档，原性能与因果证据限制保持。


### RGBD15 补充原 ORB 加速交付版（2026-10-14）

用户进一步要求原加速交付版数据图，新增目录 [orb_delivery](../artifacts/rgbd_callback_breakdown_20261014/orb_delivery)，原六轮图及其36项文件清单保持不变。新[交互图](../artifacts/rgbd_callback_breakdown_20261014/orb_delivery/callback_breakdown.html)默认old_long_01，可切换八轮；新增[同轴四轮对照](../artifacts/rgbd_callback_breakdown_20261014/orb_delivery/old_vs_bcommon_same_batch.png)，共9组PNG/SVG及8份逐源CSV。

```bash
python3 -B artifacts/rgbd_callback_breakdown_20261014/orb_delivery/generate_delivery_charts.py
python3 -B artifacts/rgbd_callback_breakdown_20261014/orb_delivery/verify_delivery.py
python3 -B artifacts/rgbd_callback_breakdown_20261014/orb_delivery/check_browser.py
```

执行结果：八轮22454回调、23280源位置；16个原CSV SHA与历史归档一致，8份真实加载身份SHA核验通过；180458项值/空值检查通过。浏览器使用此前确认可用的临时本地非沙箱Chrome，51项切换、版本身份、悬停、缩放与拖动检查通过，无JS异常。静态旧版全图、同轴图与默认旧版浏览器图目视通过；git diff --check通过。

原加速版为outer18ba684db428/core4d6df6a2db5a＋RGBD11统一观测，两轮old_long_01/02处理2882/2881，均值78.377/80.460ms、P95102.809/103.016ms。旧版与B_common的同批对比保留各自完整已处理集合和缺失，不作单改动因果结论。上一主图long_off_01为B_common＋RGBD11观测＋diagnostic_05（锁细观测关），并非row_tiles或C1。row_tiles长轮未开阶段观测，C1未运行整机长轮，不能补造相同分解图。版本与二进制见新目录SOURCE_PROVENANCE.json；源输入、输出校验与浏览器记录见VALIDATION.json、OUTPUT_CHECK.json、BROWSER_VALIDATION.json。


### RGBD15 交互图失跟底色（2026-10-14）

用户要求在已有交互图标注失跟范围。已更新根目录与 `orb_delivery` 两份 HTML 及共享模板：粉紫半透明色覆盖耗时/占比两图，区间按钮可放大，悬停区分失跟、失跟后未初始化与缺失未知。由 state3/4 开启事件，仅将尚未恢复期间的 state0/1 纳入，恢复帧不着色；2634 缺失留白。on01 四段2615–2617、2619–2621、2623–2627、2630–2664共45实测帧；on02一段781–815共35实测帧，合计78个state3及2个失跟后state0。

```bash
python3 -B artifacts/rgbd_callback_breakdown_20261014/generate_charts.py --html-only
python3 -B artifacts/rgbd_callback_breakdown_20261014/orb_delivery/generate_delivery_charts.py --html-only
python3 -B artifacts/rgbd_callback_breakdown_20261014/check_browser.py
python3 -B artifacts/rgbd_callback_breakdown_20261014/orb_delivery/check_browser.py
python3 -B artifacts/rgbd_callback_breakdown_20261014/orb_delivery/verify_delivery.py
git diff --check
```

两份图桌面/窄屏 Chrome 检查117/141项通过，无JS异常；包含实际像素、区间边界、缺失空隙、恢复、轮次切换、区间按钮及缩放检查，既有拖动/悬停检查保留。22454行跟踪状态与原CSV一致，16输入SHA及8加载身份核验通过，180458项既有数据检查通过；50份数据/静态图文件SHA保持不变。首次沙箱Chrome因setsockopt权限退出；自动批准后的本地非沙箱检查通过。证据：[失跟标注核验](../artifacts/rgbd_callback_breakdown_20261014/loss_marking/VALIDATION.json)、[效果截图](../artifacts/rgbd_callback_breakdown_20261014/browser_loss_on01.png)。原HTML/模板/验证脚本/文件清单及文档快照保留在loss_marking/before；两套FILES.json更新为本次交互图版本。未改SLAM源码，未构建、回放或操作板卡。


### RGBD15 光标联动 RGB／深度图预览（2026-10-14）

根目录及 `orb_delivery` 两份交互图使用共享 `frame_preview` 素材，光标移到源帧时同时显示该输入的 RGB／深度伪彩图和蓝色选帧线。支持锁定预览、输入源帧号、前后按钮及图表方向键；窄屏为可收起的浮窗。持续移动时最多约每45ms发起当前帧加载，过期响应不覆盖新选择，RGB/depth成对提交；图片失败清空旧图并明确提示。

```bash
/usr/bin/python3 -B artifacts/rgbd_callback_breakdown_20261014/frame_preview/extract_previews.py
/usr/bin/python3 -B artifacts/rgbd_callback_breakdown_20261014/frame_preview/extract_previews.py --verify-only
python3 -B artifacts/rgbd_callback_breakdown_20261014/generate_charts.py --html-only
python3 -B artifacts/rgbd_callback_breakdown_20261014/orb_delivery/generate_delivery_charts.py --html-only
python3 -B artifacts/rgbd_callback_breakdown_20261014/image_preview_update/verify_preview_binding.py
python3 -B artifacts/rgbd_callback_breakdown_20261014/check_browser.py
python3 -B artifacts/rgbd_callback_breakdown_20261014/orb_delivery/check_browser.py
python3 -B artifacts/rgbd_callback_breakdown_20261014/orb_delivery/verify_delivery.py
git diff --check
```

实际原包2910对输入按纳秒时间戳与RGB/depth各自序号核对；bag、索引与配置SHA提取前后均匹配历史CONTROL。原编码rgb8/32FC1、640×544；DepthMapFactor=1.0，深度单位米。输出320×272的2910张RGB JPEG及2910张深度PNG，约148.47 MiB。深度采用固定0–5m TURBO，<=0/非有限为黑，>=5m饱和，先最近邻缩小再着色；RGB为有损缩略图，深度图不提供原始数值测量。提取中2910深度PNG逐像素回读，5820图片尺寸/解码和5824文件SHA/大小检查通过。证据在frame_preview/SOURCE_VERIFICATION.json、VERIFICATION.json、MANIFEST.json。

八轮原CSV与源状态共45734行timestamp/双seq绑定一致，16原CSV SHA核验通过；50份既有图表数据、CSV及静态PNG/SVG未改，两个HTML嵌入数据一致，11640个相对图片路径存在。根目录/八轮图Chrome193/217项通过，无JS异常；包括桌面/窄屏、持续悬停、精确帧号、首末帧、缺失与失跟、换版、锁定、前后/键盘、异步乱序、加载失败、收起和既有图表交互。原性能数据180458项及8份加载身份复核通过。Chrome沙箱socket限制后使用获自动批准的本地非沙箱验证；首次浏览器检查发现预览底部超出1050px窗口，缩紧图片布局后通过。最终桌面/窄屏截图目视通过。

缺失回调帧仍展示原包输入，页面明确不表示该轮已经接收或处理；所有版本共用同一输入，不能把预览当成SLAM输出。可直接用浏览器打开离线HTML，无需服务或重新回放；移动文件需保留整个图表目录内的相对位置。变更前模板、HTML、脚本、清单和文档在image_preview_update/before保留。两套FILES.json已更新。未修改SLAM源码、构建、回放或访问板卡。


## RGBD16 最新候选 P 板上长包研究（2026-10-14）

按用户新指令固定 B_row→P 各一轮，复用原冻结成套 ARM 包与监督工具。两轮实际执行和分析均退出0；每轮全部发布2910对，处理2881/2898，全部已处理帧新鲜有限且state2、map0、reset/active_reset0；两侧双pose消费者与处理源一致。完整回调mean82.385→79.044ms、P95106.196→101.023ms，ORB P9540.930→36.438ms，峰值PSS1017.82→1011.13MiB。P尾段727/727，但全程仍缺12源，回调最大值150.351→151.192ms。

源码/构建/输入/实际加载身份、前后依赖、回传SHA和记录器/时钟/全程资源核验通过；独立脚本重算全部5779帧的时钟、源集合及分位数。旧I0观测侵入失败保持，原I1/I2未续跑；RGBD16 passing仅为用户指定研究测试交付，不代表性能或生命周期验收。板端4、主机12个登记进程均已退出。未构建，因本次测试原冻结候选二进制；未运行正式重复/真值精度/生命周期，因本轮是固定一对研究。

入口：`artifacts/orb_25ms_impl_20261013/epoch01/board_test_20261014/REPORT.md`，精确命令在同目录PLAN/EXECUTION，原始记录在runs。复算：`python3 -B artifacts/orb_25ms_impl_20261013/epoch01/board_test_20261014/tools/summarize_pair.py`（不连接板端，重写本研究汇总JSON）。统计全实际帧、不裁剪、不取交集；四段按728/727/728/727划分，历史分析器原边界保留。


### RGBD16 补充原交付版历史对比（2026-10-14）

用户要求加入交付版，已在RGBD16 REPORT增加旧old_long_01/02和本次B_row/P四列。复核命令：`python3 -B artifacts/orb_25ms_impl_20261013/epoch01/board_test_20261014/delivery_comparison/compare_delivery.py`。六份旧帧/源状态/内存CSV SHA核验，两个回调/ORB序列的均值/P95/P99/max、内存峰值及源数量独立重算，与归档一致；新板测SUMMARY只读引用。峰值PSS旧1252.93/1248.54MiB、P1011.13MiB；旧回调mean78.38/80.46ms、P79.04ms。两批观测detail开关、内存采样起点与实现不同，明确只作历史参照。未构建、回放或访问板端。原报告/清单和三份文档存delivery_comparison/before；原始运行数据未改。


## RGBD15：P 最新板测及两轮 M1 细观测加入交互图（2026-10-14）

按用户指令复用 `P_integration_r3` 冻结包，执行且仅执行两轮新 M1 完整长包：`research_20261014_P_detail_long01/02`，均1×、2910源、700特征、5层、3执行者、原8/8/10队列。实际命令、监督期限、环境、退出和SHA在 [新回放目录](../artifacts/orb_25ms_impl_20261013/epoch01/board_detail_20261014/PLAN.json)、`EXECUTION.json`、各轮`RESULT.json`和`MANIFEST.json`。未重编译、改算法、重部署或调整性能参数；新输出与历史记录分开。

两轮处理2891/2892，缺19/18；已处理均state2、新鲜有效、map0/reset0；回调均值82.213/77.755ms，P95为105.926/105.535ms；ORB均值30.365/30.438ms。两轮自然退出、16个自有进程身份均已退出；124份新归档SHA通过，原M0目录148文件及12个冻结工具未改。M1开销未单独量化，完整输入、90ms及严格ORB25ms均未通过，旧I0失败不变。见 [SUMMARY](../artifacts/orb_25ms_impl_20261013/epoch01/board_detail_20261014/analysis/SUMMARY.json)、[FINAL_STATE](../artifacts/orb_25ms_impl_20261013/epoch01/board_detail_20261014/FINAL_STATE.json)。

[原交互图](../artifacts/rgbd_callback_breakdown_20261014/callback_breakdown.html)现在10轮，[包含原交付版的图](../artifacts/rgbd_callback_breakdown_20261014/orb_delivery/callback_breakdown.html)现在12轮，均默认P M1 01；同时加入此前完成的P/B_row M0对照。M0仅ORB及总回调实测，其他阶段保留null并显示“未测”，灰色明确“其余回调（未细分）”；M1保持八色互斥分解。新增轮没有已记录失跟，缺失保留空白；同一源图像预览、导出、缩放和历史失跟底色均保留。

复核命令（只更新图表交付，不重跑板端）：

```bash
python3 -B artifacts/rgbd_callback_breakdown_20261014/generate_charts.py --add-board-runs
python3 -B artifacts/rgbd_callback_breakdown_20261014/orb_delivery/generate_delivery_charts.py --add-board-runs
python3 -B artifacts/rgbd_callback_breakdown_20261014/p_board_update/verify_board_addition.py
python3 -B artifacts/rgbd_callback_breakdown_20261014/image_preview_update/verify_preview_binding.py
python3 -B artifacts/rgbd_callback_breakdown_20261014/check_browser.py
python3 -B artifacts/rgbd_callback_breakdown_20261014/orb_delivery/check_browser.py
python3 -B artifacts/rgbd_callback_breakdown_20261014/p_board_update/refresh_manifests.py --scope root
python3 -B artifacts/rgbd_callback_breakdown_20261014/orb_delivery/verify_delivery.py
python3 -B artifacts/rgbd_callback_breakdown_20261014/p_board_update/refresh_manifests.py --scope delivery
git diff --check
```

新增4轮11562行、两个归档124/147文件SHA通过；旧6/8轮完整对象及48份旧CSV/静态图保持。全12轮273032项值、24份原始CSV、12份加载身份、68936行时间戳/双seq绑定通过。Chrome桌面/窄屏275/303项通过，0未捕获异常；截图`browser_p_board*.png`目视通过。两套输出清单5941/52文件已更新。首次独立核验器因Python3.8不支持dict合并运算退出1，已修正为兼容写法；加入长M1轮次名后浏览器发现选择框挤出第二行、预览越界，限制桌面选择框宽度后保留原检查并通过；失败记录及截图在`p_board_update/VALIDATION_FAILURES.json`、`browser_failure.png`。未改原始数据来通过检查。


## RGBD17：低扰动锁／调度观测（2026-10-14，因果目标 blocked）

入口：[REPORT](../artifacts/rgbd_a1_lowperturb_20261014/REPORT.md)、[冻结计划](../artifacts/rgbd_a1_lowperturb_20261014/PLAN.json)、[短轮总门](../artifacts/rgbd_a1_lowperturb_20261014/analysis/SHORT_GATE.json)、[原始独立复核](../artifacts/rgbd_a1_lowperturb_20261014/analysis/SHORT_RAW_INDEPENDENT_REVIEW01.json)。新实现只在本次隔离source/diagnostic，原RGBD11/14归档及生产源不变。

原current源码128文件与原ARM节点/库冻结；新增25处connections锁边界、退休二级锁、UBC粗CPU、五组线程独占环及三线程同CLOCK_MONOTONIC perf切出/切入。无新增热路径IO/日志锁/逐锁CPU读钟；保留原算法/锁顺序/参数/基础统计。完整冷构建43核心单元、0缓存对象复用，成套库/节点已上板；首次Map.h循环包含失败和修复后的attempt02均保留。host解析/区间/门46项通过；实际ARM生产适配器已知计算/休眠/嵌套等待同轴重构守恒。单锁额外约0.510us，20k嵌套增加外持锁约9.980ms；应用+perf预留63.672MiB。

四短轮B→ON→ON→B全部120/120，正常跟踪、新鲜有效位姿、map0/reset0、自然退出；每轮119个A1测量无wall−CPU>10ms。六条perf零丢失/时钟/身份/解析错误，容量门通过。两轮中位ON−B：mean+0.628730ms（限0.770320）、P95+1.046863ms（限1.859428），地图等门通过；唯一失败是B自身P95跨度4.695390ms>3.718855ms。总门不通过，不能判定低扰动已合格或归因于观测过重；按冻结规则未跑四长轮，原基线A1原因未知。

独立原始复核61,988项无问题，包含175冻结输入、132构建源、ARM ELF/加载/回传身份、480帧和6份perf原始二进制。不把已知原因fixture当成SLAM原因。当前板内核未取得sched_wakeup，nonrunnable-out到in不能拆阻塞/唤醒后排队；perf on-CPU不代替线程CPU。其他features/pose/Map锁持有者和未覆盖窗口仍未知。

离线复核命令（不重跑板端）：

```bash
python3 -m unittest discover -s artifacts/rgbd_a1_lowperturb_20261014/tests -p 'test_*.py' -v
python3 artifacts/rgbd_a1_lowperturb_20261014/tools/verify_core_instrumentation.py
python3 artifacts/rgbd_a1_lowperturb_20261014/tools/check_influence.py short
python3 artifacts/rgbd_a1_lowperturb_20261014/tools/analyze_capture.py current_short_on_01
python3 artifacts/rgbd_a1_lowperturb_20261014/tools/analyze_capture.py current_short_on_02
git diff --check
```

总门脚本预期exit1；本次未执行长轮因前置门失败，未执行真值精度、内核变更或生产修复。板端8个登记进程已消失，无SLAM残留。TODO：新实验需先验证基线稳定性及对照长度，另需真实唤醒边界才能分开阻塞与醒后排队；不修改这次门槛、不从0实际尖峰推出LM/LC冲突。


## RGBD17：用户授权的四完整长包补测（2026-10-14，低扰动原基线因果仍 blocked）

用户明确纠正仅短包不足，本轮隔离目录 [rgbd_a1_lowperturb_long_20261014](../artifacts/rgbd_a1_lowperturb_long_20261014/REPORT.md) 新冻结178项，复用旧基线/diagnostic_02成套二进制；不改C++源码、不重编。旧短门失败、旧归档1314项全部保持，长数值门不放宽。用户授权只替代长轮入口条件，详见PLAN和LONG_ENTRY_INDEPENDENT_REVIEW01。

实际命令为 `python3 artifacts/rgbd_a1_lowperturb_long_20261014/tools/run_round.py <run_id> current --package <package>`，按 current_long_b_01/baseline_01 → current_long_on_01/diagnostic_02 → current_long_on_02/diagnostic_02 → current_long_b_02/baseline_01 顺序；使用原ROS环境完整1×长包2910对、700特征/5层/3执行者/8-8-10队列、两后台开启。四轮全部发送2910，完成2833/2811/2820/2843，缺失77/99/90/67；已完成均state2/新鲜有效/map0，所有节点/回放器/观察器正常退出。

A1尖峰110/241/240/154；两ON共481，475获取累计>10ms而仅9最大单次>10ms。按预定四段保留的8详细窗口全部匹配LocalMapping，主要UpdateBestCovisibles，7多次短获取累计、1单次长持锁；持有者匹配段无切出。源1803第78次获取12.802ms，KF739出生代次741的UBC wall12.999/CPU12.926ms，首邻居检查前12.832ms，78次子锁累计0.031ms。只能定位到reserve/复制/sort等范围，不能命名精确sort/malloc时间；8窗口之外473尖峰持有者详情未知。六perf共356884 SWITCH原始/CSV一致，零perf丢失；应用环有明确旧历史覆盖，仅选定窗口保证覆盖，无wakeup仍不能拆阻塞/醒后排队。

`tools/check_influence.py long` 正常计算后预期exit1；67检查48通过/19失败，与独立工具逐值一致。P95中位+5.246ms>2ms、两对缺失+22/+23、尖峰率+3.894pp和局部地图变化；B均值自身不稳定，均值变快也不构成同负载收益。ON2曾有未隔离主机离线绘图并已停止，保留实验限制；不把全部变化单独归因于观测。四轮原始209655项及ON二进制4793935项独立复核零问题。所有2910完整输入及P95≤90ms的原正式门四轮均失败，未以采集完整性替代性能验收。

已执行两轮 `tools/analyze_capture.py`、四轮 `analysis/enrich_all_frames.py`、`analysis/summarize_long.py`、两个候选的 `analysis/plot_timelines.py`；八幅PNG/SVG生成，源1803和2189目视通过。逐源每轮2910行，缺失不筛除。主机绘图首尝试因逐span创建绘图对象慢而中止，改离线批量渲染后在全部回放结束后生成，未改采集工具/参数。

`operations/final_audit.py` 在主机非隔离/proc和已授权SSH环境只读核验通过：旧1314/新178冻结输入一致，远端二进制及配置相同，本实验主机进程无残留，11411可绑定，板端8登记进程均退出。没有重跑更多长轮、关闭LocalMapping、改锁或生产部署；不重复未变二进制的冷构建/旧探针，不做本轮范围外的真值精度验收。TODO仍为降低锁内观测成本并通过完整长包对照、取得真正唤醒边界；本次完整长包补测交付完成，原基线因果目标blocked。


## RGBD18 原基线低频 CPU 与粗观测（2026-10-14启动，10-15完成；原基线因果 blocked）

隔离目录 `artifacts/rgbd_a1_sparse_profile_20261014`，入口 [REPORT](../artifacts/rgbd_a1_sparse_profile_20261014/REPORT.md)、[SUMMARY](../artifacts/rgbd_a1_sparse_profile_20261014/analysis/SUMMARY.json)。实际完成两组关→开→开→关共8次完整2910对长包，0短包替代、无额外择优重跑；700特征/5层/3执行者/队列8/8/10/1×完整前缀，原LM/LC保持。profile全部使用原成套二进制，粗观测OFF使用原二进制，ON为独立纯观测版；基础阶段统计一致，生产源码及部署未覆盖。

板perf19Hz cpu-clock:u仅采实际GDB识别的LM TID，DWARF4096、MONOTONIC、FIFO启用及target EXIT关闭。板端已知复制/排序/分配、控制FIFO与自然退出探针完成。原isBad逐字不变，Tracking每次只两次wall读钟且帧汇总，新增邻居粗wall/CPU；LM1/64 UBC先判断再读钟，未采样零新读钟，停止逐内层记录。固定缓冲3473664 B。真实host与ARM适配器、时钟失败/溢出/活跃未知/计算睡眠嵌套及源码逆变换核对完成；ARM七批微测增量中位192.789ns/过滤调用、191.269ns/100邻居UBC外层（不是整机上界）。ARM成套41核心＋2节点全量冷编，0对象复用，实际载入SHA与编译输入一致。

八轮23280源位，完成22691、缺589；全部正常封口退出/传输校验。profile B2源2345–2379与sparse ON1源258–292各35非新鲜无有效输出、34state3+1state0并换图；其余六轮已处理帧正常，未补缺失或宣称真值精度。独立原始437866项与背景4254项通过，正常结构不等于质量/扰动通过。profile67门25失败，sparse67门19失败；控制波动、地图工作量及质量使原基线因果资格失败。两个粗观测CSV自身均无时钟/溢出/计数问题；ON1完整质量失败、ON2通过。严格尖峰128/102，获取累计>10ms120/101，单次>10ms2/7，另8/1邻居例外；ON2仅23个最大获取局部同锁匹配、涉及5尖峰，不能外推全量或拆唤醒后调度。

首次native解码两轮180s超时，失败输出留存；后续仅离线关闭inline名称展开，原data/SHA/采样及冻结解析器不改。原core无GNUbuild-id、node只有ABI note，冻结解析器的硬身份条件无法通过；实际SHA/符号映射正面证据单列，前置未覆盖该条件的工具缺口不掩盖数值失败。CPU样本只作研究线索，不跨矩阵拼接时间轴。当前kernel CONFIG_FTRACE/SCHEDSTATS未设置，tracefs主入口及schedstat不存在、debug tracing同用户拒绝；没有wakeup分解。候选入场false，0优化实施/验证，不修改mbBad，容器失败边界与退休/生命周期/序列化审核独立留存。

回放期间无本任务编译/绘图；sparse B1回放期仍有主机stdin perf raw扫描约18.6/22.3s完成，精确起止未记录，离线分析隔离不完整。开始该轮前未确认扫描结束，额外主机影响未知；补录analysis/HOST_ANALYSIS_OVERLAP01.json。八轮host禁止工作字段仍空，文件名检测漏掉stdin分析，不能证明全部分析隔离；温度/频率/host负载每秒留存，不证明系统独占或作事后修正。新363冻结输入、旧两归档1973文件、原源码128与5外部输入最终SHA通过。实际host与board20登记进程退出、无活动SLAM、11411释放、6包文件SHA一致；board原始数据保留。

离线复核（根目录执行，不触发回放；归约更新派生结果）：

```sh
python3 -B artifacts/rgbd_a1_sparse_profile_20261014/tools/analyze_profile.py profile_on_01
python3 -B artifacts/rgbd_a1_sparse_profile_20261014/tools/analyze_profile.py profile_on_02
python3 -B artifacts/rgbd_a1_sparse_profile_20261014/tools/analyze_sparse.py sparse_on_01
python3 -B artifacts/rgbd_a1_sparse_profile_20261014/tools/analyze_sparse.py sparse_on_02
python3 -B artifacts/rgbd_a1_sparse_profile_20261014/tools/check_comparability.py profile
python3 -B artifacts/rgbd_a1_sparse_profile_20261014/tools/check_comparability.py sparse
python3 -B artifacts/rgbd_a1_sparse_profile_20261014/operations/archive_verify.py
```

两个比较exit1、sparse ON1解析exit2是保留失败；只读封存核验使用最后一条。原始执行/全冷编/微测/恢复和全部失败命令在实验operations/builds/preflight。未运行候选长轮、生产修复、真值精度及完整保存重载/活动GBA生命周期，因为进入条件未达且本次为纯观测。TODO：质量与工作量可比的原基线、持有者覆盖及真实唤醒证据仍缺；不追加本轮回放凑门。


## BAG01 外置盘2026-10-21 bag分析与归档（2026-10-24）

完整报告：[REPORT](../bags/camera_20261021/REPORT.md)。五包15,995,077,322字节；20,813消息全量反序列化、物理边界/计数和目标20,813项索引偏移/时间及SHA均通过。4包为640×480约15Hz的RGB与原始16UC1深度，配对98.154%–99.966%，最长帧间隔2.269秒；另1包只有0.999秒/16帧RGB。没有CameraInfo、TF、IMU、深度比例或已确认的深度对齐，不宣称可直接用于现有RGB-D/惯性流程或SLAM精度通过。

执行`audit_bags.py`、`import_bags.py --copy`、`verify_import.py`，均exit 0，证据位于`artifacts/bag_import_20261024/`；临时副本截断/magic/索引偏移负例被正确拒绝。5包已按包内北京时间重命名到`bags/camera_20261021/`，manifest/SHA256SUMS/逐帧时间戳/JSON/预览齐备。复用本地14,759,763字节完整副本及3,982,229,504字节精确匹配前缀，后者补齐479,878,609字节；两个原本地路径为归档文件的硬链接，原文件内容前缀未丢失，剩余约3.3GB。不更改算法、构建配置或其他既有数据，未运行构建/板测/SLAM（本任务为数据整理与离线分析）。

源删除命令曾在执行前被自动审批拒绝；随后用户明确回复“保留源文件，仅复制归档”。已按该选择完成收尾，5个外置盘原包均保留，源删除未执行，也没有待执行删除或待授权步骤。BAG01状态passing仅指数据检查、重命名复制归档完成，不代表SLAM运行资格通过。manifest及TRANSFER_STATUS/COMPLETION均更新为copied_verified_source_retained_by_user_choice；原审批拒绝作为历史记录保留，当前用户选择优先，不应再次请求或执行删除。共享文档原改动快照在`preflight/`。


## RGBD28 camera_20261021 全包板测（2026-10-24）

本次active feature仅RGBD28，已完成授权的4个RGB/原始深度包1×全段板测和报告交付。运行`python3 -B artifacts/rgbd_camera_20261021_20261024/tools/prepare.py`，全包SHA与BAG01一致，5ms一对一匹配2903/3109/1010/3338对，原始未配对RGB/深度共47/30帧另存；RGB-only包缺深度，未作为RGB-D输入。`/usr/bin/python3 -B artifacts/rgbd_camera_20261021_20261024/tools/run_all.py`在sunrise@192.168.1.11运行最新atomic_02成套程序，与发布f75a3ffd的116个实际ARM编译输入对应；原D435i YAML逐字复用，700特征/5层/3 workers、队列8/8/10。仅在主机发布时统一配对header时间戳和重编号seq，原bag及图像负载不变，逐条保留原身份/记录时间。Camera.fps=10而源约15Hz，原始深度未确认对齐、标定及比例，不能据此做尺度/轨迹精度验收。

四轮10360/10360全部处理，fresh分别2895/2618/878/2809，共9200；非fresh1160（RECENTLY_LOST1142、NO_IMAGES_YET17、LOST1），map切换0/9/3/5。回调均值46.31/46.64/43.97/45.31ms，P95 53.31/57.19/55.34/62.74ms；最后一包max276.72ms，不以P95单项宣称实时性通过。raw与continuous独立消费者各10360条全有限，1160条对应非fresh回调，不算有效新位姿。1Hz采样RSS最大848.05MiB/PSS850.13MiB，各轮VmSwap0，采样窗口major fault/整机换页/OOM增量均0。

`tools/summarize.py`复算原CSV、原身份、缺失/状态/资源、主机输入status和56个回传文件SHA，all_execution_complete/all_paired_input_complete/all_measurement_integrity_ok均true；只读独立复核核心数一致。`tools/post_audit.py`核实8个owned PID/start_ticks均退出、没有SLAM残留、程序SHA保持、无新增Atlas，板端剩余2731810816字节。`artifacts/rgbd_github_publish_20261023/checkout/runtime/rgbd/verify_source.py`源码完整性通过。首次4次目录设置失败均发生在发布前，完整保留preflight/setup_failed_attempt_01*，未计正式回放。未重建/修改算法、运行相机或替换既有部署；本次无需构建/算法单元测试。passing仅表示板测与报告交付完成，不表示当前参数下跟踪质量通过。报告：artifacts/rgbd_camera_20261021_20261024/REPORT.md；机器结果SUMMARY.json与BOARD_FINAL_AUDIT.json。TODO：若后续要求解释失跟或精度，需匹配实际相机内外参、深度比例及RGB对齐；本轮不追加调参或择优重放。


## RGBD29 camera_20261021失跟离线分析（2026-10-24）

已完成用户要求的详细原因分析，主报告artifacts/rgbd_tracking_failure_analysis_20261024/REPORT.md。原板测SHA227项复核保持；使用实际冻结发布源并核对7文件与atomic归档逐字一致，不用cwd旧XFeat核心解释运行。只读原bag逐条检查10360对原图，vision_metrics.py完成4包图像指标，extract_case.py额外定点抽取1473病例；视觉聚合10360行、96个起始前后CSV与四张标准matplotlib六面板PNG通过身份/状态联表与目视检查。LK位移只在有效点>=30且存活率>=50%时汇总，指标为诊断代理、深度保持原单位，同像素角点采样不冒充实际ORB或空间配准。

两条独立工作流分别复算源码/日志和时序/资源，主报告再次独立只读复核。96段非fresh中76首次失败未调用局部地图；实际局部地图失败20次，11次<30内点，9次30–49受50保护门槛拒绝。日志7次参考KF词袋未就绪且got0，定位到先选为参考、后台才计算词袋、备用SearchByBoW缺就绪同步；日志无逐帧stamp，不能将7条精确归源序号或解释全部失跟。内部Relocalization门槛20与外层近期50之间有27次accepted未最终OK。17切图之外161410第469对8KF小图ResetActiveMap一次。相关Tracking/KeyFrame/LocalMapping/ORBmatcher与优化前baseline逐字一致，不能断言最近MapPoint原子化引入该逻辑问题，也不能无对照排除其时序间接影响。

17持续失跟起始10个header gap<100ms，仅3个>=1s，前5帧无>100ms回调；共9个>1s gap有5个当帧及后两帧仍fresh。1610152137源gap1.868s且LK1/139存活，源间断/视野大跳有具体证据；2277/2278区分原录制延迟与真实采集跨度。1605362854快速滚转、709特征/29内点；161015302清晰慢动仍前端失败；1615501473逆光低纹理，555→459特征、28内点，回调23.64ms。96起始仅5个Lap<300/1个<100，不作普遍模糊结论。资源没有分页/OOM/Swap共同模式。RGB与原深度空间对齐/标定/比例仍未知；源码直接按RGB像素取depth、d>0接受，65535在factor1000下会成为65.535m，若其为无效值才应掩膜，不猜设备语义。FPS10改变mMaxFrames与恢复/KF策略、不丢弃15Hz的每秒5帧。

执行脚本与子报告均在新归档，未修改原bag、旧证据、核心算法或部署，未重跑板测/构建（本任务是既有证据分析）。passing仅指分析交付完成，不代表算法修复或精度通过。后续先核实输入几何与BoW发布就绪契约，再用同包单变量复测；不建议直接降低内点门槛或同时改多个参数。共享docs原改动已在preflight快照保留。


## RGBD30 帧率与单帧回调成本对比（2026-10-24）

已完成用户所问15fps新包45–46ms、历史低帧率长包高耗时的离线对比。主报告artifacts/rgbd_fps_cost_comparison_20261024/REPORT.md；独立原始CSV归约workload_review/recompute.py、METRICS.json、REPORT.md保留全段/fresh/OK/四分段、阶段执行/零填充和局部工作量。十个历史/当前轮次CSV源SHA、行数与每帧ORB+TLM不超过回调的残差检查通过；相同atomic_02旧2轮/新4轮的core/nodeSHA一致。原bag只读索引及首条消息核对，旧2910对header11.111Hz/配置11、640x544 rgb8+32FC1，新640x480 rgb8+16UC1、源中位15Hz/沿用配置10。

同版本旧5817回调75.118ms→新10360回调45.860ms，ORB29.332→21.988，TLM每回调33.036→11.980、仅执行33.048→13.455；约72%的观测差额落在TLM。局部KF/MP/gather中位旧114/8236/80371，新29/1502/20417；首包无换图仍38/1741/26772及46.309ms，说明失跟重置不是唯一地图差异。新fresh-only45.818、nonfresh46.192ms，失跟跳阶段不是总体低均值主因。旧组合整包86.531/86.638、尾四分段95.191/93.523ms，约34–37%单帧>90ms；历史高耗时包含旧版本及重地图后段。线程CPU旧73.560/新44.483ms，MapWait仅0.095/0.111ms，不支持主要地图更新锁等待。较小图像少11.8%像素，内容/几何/轨迹和地图演化亦不同；不将阶段分账说成帧率或某单项的因果收益。

passing仅表示对比分析交付。帧率决定输入时间预算，单帧工作量决定实际回调成本；45ms不代表旧大地图也可15Hz。未修改算法、旧证据、参数或部署，未新跑板测/构建。本任务仅离线分析，无需算法单元测试；复核与源SHA/语法/算术闭合已完成。后续若要隔离帧率效应，需同序列同版本同几何做受控抽样率测试，同时检查质量与工作量；仅降播放速度是另一类墙钟/后台调度实验。共享文档既有改动快照在本归档preflight。


## RGBD31 D435i 003015_7ea2c9 最新程序板测（2026-10-30）

报告：[REPORT](../artifacts/rgbd_d435i_003015_20261030/REPORT.md)，机器结果为同目录SUMMARY.json。使用sunrise@192.168.1.11上最新已发布f75a3ffd对应的atomic_02成套程序，实载node/core SHA分别20a17635…/471f66f7…，与既有部署一致；本轮未重编译或替换程序。原包SHA dda9cc56ac095d6bfa5c7c2265995b6e8791615f4ecc880ccbe63a176c4bcbf1，约589.678秒、640×480、header均值9.969Hz；23515条消息完整读取，5878对原stamp精确同步，全部CameraInfo恒定且两流K/D/R/P相同。按实际包内内参生成新配置，RGB+已对齐16UC1深度、factor1000；ORB700特征/5层/3执行者、队列8/8/10、fps10，1×原记录时间回放，无时间戳/seq改写。输入原bag和其他会话保持。

执行命令（run_all为已完成的单次采集，不重复执行）：

```bash
/usr/bin/python3 -B artifacts/rgbd_d435i_003015_20261030/tools/prepare.py
/usr/bin/python3 -B artifacts/rgbd_d435i_003015_20261030/tools/run_all.py
/usr/bin/python3 -B artifacts/rgbd_d435i_003015_20261030/tools/post_audit.py
/usr/bin/python3 -B artifacts/rgbd_d435i_003015_20261030/tools/host_final_audit.py
/usr/bin/python3 -B artifacts/rgbd_d435i_003015_20261030/tools/summarize.py
```

5878/5878均收到/配对/返回，无缺失或同步drop；fresh4754（80.88%），RECENTLY_LOST1089、NO_IMAGES_YET31、LOST4，失跟状态连续63段、map编号0至31变化31次。回调均值44.837754、P50 42.652512、P95 58.940474、P99 76.673932、最大191.388661ms，10次>100ms；fresh-only均值43.892723ms，非fresh48.834796ms，未剔除失跟改善均值。两路独立位姿消费者各5878条有限，其中1124条匹配非fresh状态，不计有效新位姿。RSS/PSS约1Hz采样最大878.96/880.60MiB，VmSwap0；进程major fault0、整机pswpin增量0/pswpout增量1/OOM增量0，不把历史OOM=1算本轮异常。

归约all_execution_complete/all_paired_input_complete/all_measurement_integrity_ok均true；独立只读复核原CSV、全部14个回传文件SHA、分位数与阶段/四分段一致。BOARD_FINAL_AUDIT核实owned进程退出、无活动SLAM、程序SHA未变、无新增Atlas；HOST_FINAL_AUDIT核实冻结10文件保持、回放/观察器/ROS master无残留、11421已释放。首轮prepare因Python3.8不支持dict合并写法失败，修正准备工具后全量验证成功，未在失败尝试启动SLAM，日志保留preflight/prepare_attempt01.log；首次sandbox内端口检查被系统限制，授权环境重做通过。没有额外择优重跑。

RGBD31 passing仅指整包测试与报告交付，当前跟踪仍不稳定。无真值轨迹，本轮不宣称ATE/RPE、尺度精度、稳定性能或后端线程完全join；completion的null保持原意。未运行算法构建/单元测试：本轮无算法、ROS节点源码或既有部署变更，仅新增隔离测试工具和证据。TODO：如后续要求定位此次失跟，按原ordinal/stamp联查图像、前端内点/日志与地图重建；本轮未实施修复或调参。最终清单校验见本归档SHA256SUMS与SHA256_VERIFY.log。


## RGBD32 D435i 003015_7ea2c9失跟诊断（2026-10-30）

详细报告：[REPORT](../artifacts/rgbd_d435i_failure_analysis_20261030/REPORT.md)。本轮只做离线分析，原RGBD31的73项SHA与输入4,815,717,371字节bag全量SHA保持；未改算法、配置或旧证据，未连接板端或重新回放。实际atomic_02冻结源与实际ARM编译输入9个核心文件SHA一致。

原5878对图像全部读出并按原stamp/ordinal联表，320×240角点/前后向LK只作诊断代理；原对齐深度保持16UC1值。63个OK→非fresh起始：33前端未进局部地图（12参考BoW粗匹配<10、21按控制流推断参考优化后地图匹配<10），22局部地图实测内点<30，8最近重定位保护窗内点30–49。63个起始中11次两帧保护直接走参考，只有其中5次未进局部地图，独立审查已修正初稿把11都写前端失败的计数。1124失败marker与1124非fresh回调按顺序交叉校验：KF175空参考BoW唯一事件顺序推断到674；28内部accepted只有1368未最终OK（内部49→局部41），不用旧包27事件。结局27原图恢复、31新建图、4同ID重置（94/114/2253/3446）和1段EOF右截断。

63起始当帧回调最大56.879ms、前5帧均无>100ms；全段10个>100ms均fresh，多为恢复/重初始化之后。3个原header较大间隔3135/4072/5321当帧均fresh；USB2.1及watchdog确有证据，5321的1.172s间断附近仍OK，不作全部失跟根因。资源无Swap/major fault/OOM共同前兆。LK代理经≥30存活且≥50%存活率过滤后，失跟起始位移P90跨帧中位47.14px，对fresh24.43px；原始位移不等于运动模型预测残差。全段81.35%帧无2m内诊断角点，但正常帧远点比例也很高，作为背景条件不独立归因。65535会按65.535m入图，其硬件语义未确认；SDK标准无效深度为0，没有依据将65535全部命名无效值或宣称单位错误。本轮CameraInfo/对齐/颜色/因子匹配，物理曝光时差和远距测深准确性仍未知。

已执行的分析命令：

```bash
/usr/bin/python3 -B artifacts/rgbd_d435i_failure_analysis_20261030/tools/vision_metrics.py
/usr/bin/python3 -B artifacts/rgbd_d435i_failure_analysis_20261030/tools/analyze_vision.py
/usr/bin/python3 -B artifacts/rgbd_d435i_failure_analysis_20261030/source_review/analyze_source.py
/usr/bin/python3 -B artifacts/rgbd_d435i_failure_analysis_20261030/timing_review/analyze_timing.py
```

首条全量读取约88秒并生成病例图，输出目录已存在时拒绝覆盖，不直接重复；其余归约仅写本分析派生结果。图像指标与实际SLAM阶段/状态逐帧身份检查通过；6面板PNG/SVG及60/94/323/1370/2222/5155病例已目视。源码、时序、几何分工后独立审查原CSV和主报告，必要数字修正完成，无剩余阻断。证据verification/FINAL_CHECK.json、INDEPENDENT_REVIEW.json、ORIGINAL_BAG_SHA256.json及最终SHA256SUMS。未运行构建、算法单元测试或性能复测，因为无算法改动且本任务仅诊断。RGBD32 passing仅表示分析交付；TODO：修复BoW就绪同步并同包单变量验证、补每帧参考/运动模型拒绝原因、确认65535与远距深度可靠性；本轮未实施修复或宣称真值精度通过。


## RGBD33 D435i失跟诊断网页（2026-10-30，本地交付完成；线上发布待授权）

产物artifacts/rgbd_d435i_web_20261030/site/dist/index.html，本机预览http://127.0.0.1:8768/，完整本地包d435i_diagnostics_offline.zip。采用rgbd-callback-charts的八分量/身份/状态语义，按本轮RESULT.id与loaded实际布局做独立适配；Sites项目身份保存在site/.openai/hosting.json。只读RGBD31/32原归档，旧73/108项SHA核验保持，不运行SLAM或修改算法。

执行tools/build_data.py、extract_previews.py、render_reports.py：全部5878帧以原ordinal/stamp字符串/双seq联表，63事件及33/22/8原因、27恢复/31新图/4清图/1EOF一致；324组关键帧/648图从原bag精确提取，固定0–30m、0黑/65535紫，648文件SHA及尺寸/解码通过。A1差额保留原值、无钳零；8互斥分量加和最大误差1.42e-14ms，父TrackLocalMap不再叠加，null保持未执行。36坐标代次包含4次清图，只有fresh有限pose，非fresh和非相邻源号处断线，失跟标记为此前有效位姿；不拼全局地图。门槛按28accepted/4reset及真实帧ID重建，界面明确非直接打点。

独立14项数据审查通过，见validation/INDEPENDENT_DATA_REVIEW.json。node --check app.js/data.js、Python AST、全部静态相对链接及离线ZIP完整性通过。实际Edge/CUA验证筛选8保护/33前端/1EOF、1370的49/50、674空BoW与未执行TLM、44缺失预览占位、94→95同ID坐标代次变化、5878末帧按钮禁用。发现数值框按Enter未跳转后补明确keydown处理并复测通过。390×844窄屏pageWidth375无横向溢出，桌面/手机截图已目视，viewport已恢复。初始化浏览器未捕获error/warn。

浏览器下载事件捕获超时，未拿到浏览器交付文件路径；改为从实际不变的downloadCSV/exportEvents/exportFrames函数捕获Blob，标准CSV解析核对8保护事件、1350–1390共41帧及19位纳秒字符串完整，见EXPORT_FUNCTION_CHECK和导出样本。file://自动导航被浏览器URL策略拒绝，没有绕过；HTTP实际交互验证完成，离线静态资源已核验。首次origin审批检查超时，随后发现原预览进程结束；在同一批准origin重启PTY服务后恢复检查，没有使用其他浏览器控制通道。

线上发布命令site-workflow.mjs在进程创建前被自动审批拒绝：用户只授权生成web，未明确允许上传本地RGB/深度、轨迹与诊断数据到具体远端Sites仓库。拒绝后未绕过、未上传/推源/保存版本/发布。空私有Site已创建，ID appgprj_6ae45ab7efb0819194ee646aacc0e483，未来如用户明确同意应复用、重核受众并刷新凭据，不重复create_site。状态PUBLISH_STATUS.json；凭据只在会话内存，未落文件。保留仅127.0.0.1预览供用户审阅，owned PID/start_ticks在LOCAL_SERVER.json；这不是公网发布。本地功能passing，线上仍blocked待授权。未做算法构建、重放或性能/真值验收，因为本轮仅网页派生与交互。
