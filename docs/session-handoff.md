# RGBD35 地图内点衰减分析与参数计划 — 2026-11-02

状态passing仅指分析交付，主报告artifacts/rgbd_map_decay_20261102/REPORT.md；NEXT_STEPS.md包含分阶段观测/BoW修复/四组参数/深度与采集侧对照、建议筛选标准。未启动新板测或修改算法，不能称参数已提升。

35个首次失跟前可比代次32个内点下降，窗口中位210→64，但33个localMP增加、582→1334；固定0–1s/4–5s26组19降，避免把失败终点偏倚说成地图年龄因果。4655实际连续fresh局部跟踪相邻对42.62%上升。36首帧离线ORB深度33无2m内点；8/12m cap仍有4/3个首帧不足50候选点，不是可靠深度或初始化门槛结论。初始化全正深度建点、后续约100深度槽位（含已有MP）、近点专用触发、正常剔点/BA及BoW问题的源码范围均已核。

执行主analyze.py、独立trend_review/analyze_trends.py、vision_review/analyze_orb_depth.py和只读tools/verify.py。5878输入身份与5冻结SHA、12源码/编译输入、450深度坐标取样、35/26趋势独立复核、4配置与4PNG/SVG/21本地链接通过。validation/INDEPENDENT_REVIEW.json passed无剩余发现；配置浮点读取要求已明确80.0。原bag全SHA复用RGBD34已完成记录，本轮未重算4.8GB文件；只读原深度并记录payload SHA。初始None求和脚本错误已修正，失败记录保留。

修改范围仅本轮新归档及features/validation/session-handoff三份增量文档；用户已有README/录制代码/其他修改保留。旧板测、旧图及127.0.0.1:8768网页服务未更改。没有Git提交、远端上传、硬件修改或发布。根算法构建不适用；只读封存检查入口为`/usr/bin/python3 artifacts/rgbd_map_decay_20261102/tools/verify.py`，重建分析脚本会改本轮派生文件，封存后不要作为只读检查直接跑。

下一步按计划先取得前端/局部搜索/优化与点寿命证据，独立修BoW后冻结同一代码基线，再进行P0→P1→P2→P3→P3→P2→P1→P0八完整轮；4组YAML已准备未回放。TODO：真实MapPoint逐点历史、65535设备语义/远深度真值、参数收益、跨场景稳定性及ATE/RPE仍未验证。不得将离线F内点、初始化种子点或错误边界数量当原跟踪内点。

以下历史记录保留。

# RGBD34 特征与匹配中间图交接 — 2026-11-02

状态passing，本机http://127.0.0.1:8768/#event=1&frame=43&from=23&to=67已切到artifacts/rgbd_feature_visuals_20261102/site/dist；原RGBD33静态网页、离线包和封存SHA保持。旧owned PID1508081/459018638核准后停止；新owned PID1637165/start_ticks483817445，仅127.0.0.1，为用户审阅保留。实际身份以本轮LOCAL_SERVER.json为准，管理前需核PID代次。

新增324关键帧特征/5层金字塔，323组真实前帧n−1→n候选及F筛选，原RGB主图640×480，可点击放大/关闭/Esc，支持窄屏。原运行无逐点落盘，故使用冻结ORB源码+原D435i参数在主机离线复算，统计与板端原内点分列；31个新图边界的Frame默认N/mnId未初始化而被后置快照读取，板测数量显示不可用；其余293有效对照全部数量相同，原数保留在证据，不声称原运行逐点重现。匹配图各画前120对，完整数量与坐标JSON保留。当前未选中帧仍占位，不借图。

实际运行g++独立提取构建、build_visuals.py prepare/render、两JS的node --check、package_site.py与旧SHA核验。450原始字节/灰度身份、323组完整匹配及1942资源独立复核、实际浏览器8帧/模态/390px响应式通过；ZIP2618文件逐成员SHA通过。源码/编译指令与证据见validation/BUILD.json、INDEPENDENT_REVIEW.json、BROWSER_VALIDATION.json及PACKAGE_VALIDATION.json。首次绘图NumPy坐标TypeError转int修复后复用提取输出重画通过，失败日志保留。

改动仅本轮新归档及features/validation/session-handoff三文档；保留所有用户已有未提交工作。根算法构建和板端SLAM不在此轮范围，无新板测、Git提交或远端上传。下一步直接刷新原URL查看；TODO：原历史真实MapPoint匹配、位姿优化外点逐帧图因未持久化无法还原，需要未来单独带可视化采集的回放，不能由本次相邻帧图替代。

以下历史记录保留。

# RGBD33 D435i失跟诊断网页交接 — 2026-10-30

本地网页passing，线上发布被审批阻止，尚待用户明确上传授权。打开artifacts/rgbd_d435i_web_20261030/site/dist/index.html或本机http://127.0.0.1:8768/；d435i_diagnostics_offline.zip包含全部相对资源。预览服务仅127.0.0.1，PID1508081/start_ticks459018638，准确身份以LOCAL_SERVER.json为准；为本次用户审阅保留，后续管理需先核PID代次。原RGBD31/32证据及bag/算法保持，未新板测。

功能：5878逐帧/63事件检索、原因分级证据、324组RGB/深度关键帧、8分量耗时及百分比、内点与30/50门槛、36坐标代次断线轨迹、1Hz资源和两种CSV导出/深链接。全部数据身份、阶段可加性、预览SHA/解码和独立14项复核通过；实际桌面/390px手机UI、1370保护门槛、674空BoW、44占位、94→95清图、EOF最后一帧验证通过。数值框Enter跳转已修并复测。

执行tools/build_data.py、extract_previews.py、render_reports.py及Node语法检查、离线ZIP/链接校验，完整证据在validation。下载事件捕获超时，实际导出函数Blob/CSV内容独立验证通过，不宣称验证浏览器下载路径。自动化file协议被URL策略拒绝，HTTP测试通过；原preview进程结束后同origin重启PTY，不绕过安全控制。未编译/修改算法，无真值精度结论。

线上源同步命令在启动前被automatic approval review拒绝，理由：尚无向具体远端上传RGB/深度图、轨迹和诊断数据的明确许可。没有上传、推源、保存版本或部署。已创建空的私有Site元数据，project_id appgprj_6ae45ab7efb0819194ee646aacc0e483，slug orb-slam3-d435i-diagnostics-003015；site/.openai/hosting.json已持久化，凭据未落盘。TODO：如用户授权上传到该私有目的地，复用ID，重读get_site确认受众并刷新credential，完成source helper同步/打包后保存部署；禁止重复建Site或绕过拒绝。PUBLISH_STATUS.json是权威未发布状态。

修改范围：新web归档和三份共享docs的RGBD33记录，用户原有录包工具/README等未提交改动保持。

以下历史记录保留。

# RGBD32 D435i 003015_7ea2c9失跟分析交接 — 2026-10-30

状态passing仅指诊断交付，未修复或重测。报告：[artifacts/rgbd_d435i_failure_analysis_20261030/REPORT.md](../artifacts/rgbd_d435i_failure_analysis_20261030/REPORT.md)。原5878对全量图像扫描、9个实际源码/编译输入核对、原73项及bag全SHA复核通过。仅新分析归档和三份共享docs加入RGBD32记录，旧结果/算法/相机/板端保持；既有录制工具等未提交改动保留。

63起始直接机制为33前端、22局部内点<30、8最近重定位50门槛拒绝；1空参考BoW按1124失败marker顺序关联674，28内部接受仅1368未最终OK。结局31新图/4同ID清图/27原图恢复/1EOF截断，不能只数31次map变化。63起始前5帧无>100ms，运动代理起始47.14px对fresh24.43px；深度远点为全段脆弱背景、65535设备语义未知。旧15Hz原depth/借内参疑点本次不成立；USB2.1/watchdog造成的最长源间断附近仍fresh。无因果干预或GT，不分摊物理根因百分比。

已运行vision_metrics/analyze_vision、source_review/analyze_source、timing_review/analyze_timing；命令见docs/validation.md的RGBD32。图像11组病例与总览已生成，代表病例目视；独立复核修正初稿11次两帧保护的口径（11总起始，其中5前端失败）后无剩余阻断。verification目录、各子报告和最终SHA清单保留。首条全量图像工具拒绝覆盖已存在输出，不直接重复运行；旧归档只读验证可用原SHA256SUMS。

未执行build.sh/build_ros.sh、算法单元测试或新板测，因为本次没有算法/构建变更。TODO：BoW在参考发布前就绪与线程同步；逐帧细分前端拒绝条件；核验远距深度及65535，再做同包单变量验证。增加特征/尺度层或降低内点门槛尚无收益证据，不应直接当修复；没有后台join、ATE/RPE或米制尺度通过结论。

以下历史记录保留。

# RGBD31 D435i 003015_7ea2c9 板测交接 — 2026-10-30

状态passing，仅指整包板测和报告交付完成。主报告为[artifacts/rgbd_d435i_003015_20261030/REPORT.md](../artifacts/rgbd_d435i_003015_20261030/REPORT.md)。sunrise@192.168.1.11使用最新发布f75a3ffd对应atomic_02成套程序，node/core实际加载SHA保持。使用包内恒定CameraInfo内参及aligned_depth_to_color，640×480、fps10、700特征/5层/3执行者、队列8/8/10，原生rosbag play按原记录时间1×全段回放，保留原stamp/seq及图像负载；没有启动相机、复制4.8GB包上板或覆盖既有部署。

原23515消息全读/整包SHA及5878精确RGB-D对身份核验通过。实际5878对全部处理，无缺失；有效新位姿4754（80.88%），失跟状态63段，map编号变化31次。完整回调均值44.84/P95 58.94/P99 76.67/max191.39ms，10次超过100ms。两路消费者的1124条非fresh输出不算有效新位姿；RSS采样最大878.96MiB、VmSwap0，整机pswpout增量1、OOM增量0。跟踪仍不稳定，没有真值精度结论。

已执行prepare.py、run_all.py、summarize.py、post_audit.py、host_final_audit.py，实际命令在docs/validation.md的RGBD31条目。独立CSV/四分段/阶段/消费者/资源和14回传文件SHA复核通过；两端登记进程均退出、无活动SLAM和新增Atlas，host端口11421释放、10项冻结输入保持。首轮prepare的Python3.8兼容失败日志保留，修正后验证成功，该失败未启动SLAM；sandbox内端口检查受限后在授权主机环境通过。正式全段仅一轮，无择优重放。原始板端证据仍在/userdata/rgbd_d435i_003015_20261030/003015_7ea2c9，主机归档有SUMMARY/BOARD_FINAL_AUDIT/HOST_FINAL_AUDIT/INDEPENDENT_REVIEW和最终SHA清单。

修改范围：新隔离artifacts/rgbd_d435i_003015_20261030及features/validation/session-handoff的RGBD31记录；此前录制工具、README、bag索引与其他既有改动保持。未运行build.sh/build_ros.sh或算法单元测试，因为未修改算法/节点源码或构建配置；未提交/上传Git。TODO：本次未定位每段失跟根因、无独立米制真值、没有证明所有后台线程自然排空或join。下一步如用户要求诊断，用原ordinal和stamp关联图像、内点与日志，保留本轮证据再做单变量修改。

以下历史记录保留。

# BAG03 D435i ZIP 归档交接 — 2026-10-30

状态passing，仅指解压、原始字节与bag完整性验证通过。实际源文件为 `/home/ywl/bags.zip`，原ZIP已保留。归档到 [bags/d435i_local_20261029](../bags/d435i_local_20261029/)，约9.16GiB，包含10组原会话、5个bag及全部配置/日志。5包原SHA一致，47416消息/11852精确RGB-D对全读通过；原采集结果仍为1通过、4有包但低帧率/停流失败、5无包。会话表为SESSIONS.csv，整体来源为IMPORT.json。

使用已有20261029物理扫描器完成逐块核验，并用ROS原生接口全读对照。新导入工具和证据在 [artifacts/bag_import_20261030](../artifacts/bag_import_20261030/)，extract/audit/finalize/verify已完成，最终verification.json为passed。综合bag目录由22增至27，原条目、旧路径及数据身份保持；10个源机ros_logs/latest已转换为归档内相对链接，原目标仍在ZIP_MANIFEST.json。

今后只读复核本批数据：

```bash
/usr/bin/python3 artifacts/bag_import_20261030/import_zip.py verify
```

extract/audit/finalize是本次一次性导入步骤，不要重复执行。旧20261029 organize.py verify会重建旧22包索引，当前综合索引应使用新只读verify检查。会话名保留源机器本地时间；索引时间按bag epoch换算北京时间。TODO：源机器具体IANA时区没有记录，不能把目录00:30当北京时间；这不影响使用bag UTC时间归档。

修改范围：新导入工具/证据、bags/d435i_local_20261029、四个综合索引及features/validation/session-handoff。用户原有录包代码与其他未提交改动保留。未启动硬件、构建、回放、Git提交或上传，本任务没有算法变更；未验证SLAM跟踪质量。下一步可按SESSIONS.csv选择原检查通过的003015_7ea2c9作为常规数据，其他会话的故障日志保留作诊断。

以下历史记录保留。

# STOR01 DBoW3 存储清理交接 — 2026-10-29

状态 passing，仅指归档完整性与存储清理通过。项目 108.67→92.01 GiB；文件释放合计约 16.84 GiB，包含审计记录增长后的 home 可用空间净增 16.65 GiB，当前约 53.34 GiB。旧交付副本省 10.36 GiB，926 个旧对象省 0.477 GiB，119 个实验文件无损压缩省 6.006 GiB。

已运行 CLEANUP.py 的 review-delivery、objects、compress、delivery、verify；最终验证 passed，实际 gzip/tar 解压抽检 passed。完整证据与逐文件散列在 [RESULT](../artifacts/dbow3_storage_cleanup_20261029/RESULT.json) 所在目录。两个交付 tar.xz 均保留，原始 bag、源码工作树、共享对象、当前构建、关键地图和在线图表未动。

从项目根目录恢复旧实验所需内容：

```bash
python3 artifacts/dbow3_storage_cleanup_20261029/RESTORE.py --list
python3 artifacts/dbow3_storage_cleanup_20261029/RESTORE.py --category csv
python3 artifacts/dbow3_storage_cleanup_20261029/RESTORE.py --category map
python3 artifacts/dbow3_storage_cleanup_20261029/RESTORE.py --delivery
```

还支持类别 perf_dump、perf_raw、raw_binary、symbol_dump，或 --path 指定单个原路径。恢复后保留压缩包，空间会相应增加；交付恢复会先在 /tmp 临时解包再回填本次移除的成员。旧编译对象需用对应源码版本重新构建。重跑性能/并发实验不能保证与当时数值或地图完全一致。

修改范围为 artifacts/dbow3_storage_cleanup_20261029、新产物布局及 features/validation/session-handoff 三文档。用户原有录制工具、README 和其他未提交改动保留。未运行算法构建、ROS/相机/板端或 Git 提交上传，因为本次没有算法与部署变更。下一步如需复算历史实验，先按需恢复原路径；TODO：旧实验完整硬件/依赖环境与数值复现未在本轮重测。

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

# RGBD26 当前交接 — 2026-10-20（MapPoint原子查询历史交互图）

用户指定rgbd-callback-charts技能，要求把单独MapPoint原子查询长包加入之前交互图。新增RGBD25 a_01/a_02及同期b_01/b_02，合并旧36轮成为40轮，入口 [HTML](../artifacts/rgbd_mappoint_charts_20261020/viewer/callback_breakdown.html)，默认mp_atomic_a_01。两轮原子均完整保留，不按最快选择默认。旧图/原档未覆盖，无板端/回放/构建/部署/发布。

新增11626回调/14缺失（原子5817/5820，基线5809/5820）；全图113745/116400、缺2655。新增已完成state2/fresh/valid/map0，初始未执行阶段null保留；历史未测/失跟/恢复/缺失按原对象保留。六阶段detail是基础统计，无新细锁观测；A1 wall−CPU仍只称差额，后台时间不加入堆叠。图内说明清楚共同基线已有容器、KeyFrame原子及ORB P。

原技能完整校验5918来源文件及完整2910源身份/预览，Chrome155的512检查和四幅桌面/窄屏目视通过，页面异常/consoleerror/失败资源/外网请求均0。首次Chrome沙箱socket启动失败原样保留browser_check，独立临时Chrome结果在browser_check02；未改技能/模板，skill_snapshot保存执行字节。实际命令及scope在operations，数据在viewer。

单独原子是本轮新增候选中更值得继续研究的方向，均值/P95中位改善3.08%/4.31%；不能称所有历史方案统一排名或生产通过。控制漂移/地图工作量与完整输入/90ms限制不变，RGBD25维持blocked。没有加入几何及失跟的两项组合，不需要重跑长包来画图。

只读核验：python3 -B artifacts/rgbd_mappoint_charts_20261020/verify_delivery.py 。未来生成须新空输出，保留旧图/原数据；不要求未来共享docs不变。修改仅新归档和features/validation/session-handoff三份docs，所有原文及前后快照保留。根build.sh与测试回放未运行，原因是本次只生成已完成数据的离线图。预览相对链接旧frame_preview，移动需维持目录。独立复核418835项通过，HTML完整内嵌数据、新四轮分量/百分比/导出及旧36run逐值一致，最大分量求和误差约2.84e-14ms。独立校验器首轮清单对象/字符串比较错误已修正，首轮脚本/结果保留在checks/independent/attempt01；生成器与图未改。RGBD26 passing仅指图表交付，不表示性能通过。

以下历史交接保留。

# RGBD25 当前交接 — 2026-10-20（两项修改及八完整长包完成，资格blocked）

用户要求“完成前两项修改并测试是否有效”，已经完成普通RGBD几何合并读取G、MapPoint原子查询A和精确并集GA。入口 [REPORT](../artifacts/rgbd_tracking_optimizations_20261020/REPORT.md)，源码/补丁/成套构建/原生测试/冻结计划/全部逐源数据均在新根。板端同名根下baseline_01、geometry_01、atomic_02、both_01；生产和旧归档未替换。基线是既有容器＋KeyFrame原子＋ORB P，新增原子化的是MapPoint。

候选各41+2全冷编，基线精确复用原包；所有fixture链接匹配实际DSO，四版4411静态几何逐字一致。原生168进程case通过，覆盖退休替换、强引用、状态查询锁行为、双向存档和截断异常；最后32项post-bool在八长轮结束后补做，false/0及true/1状态/退休计数四版一致。实际空观测isBad为LDARB无mutex。单点微测证明局部降成本，不能直接外推SLAM收益；OOM/完整Atlas并发保存/惯性合图仍未覆盖。

冻结664项后八完整2910、1×、全建图前缀B/G/A/GA/GA/A/G/B，原700/5/3、8/8/10、LM/LC和基础统计保持。23255/23280、缺25。A两轮均改善，轮统计中位mean77.502→75.118ms（−3.08%）、P9596.734→92.564ms（−4.31%）、两轮缺11→3；A3收集及Search CPU下降。G无长包净收益；GA1完整/89.395ms/正常，GA2有35无效新位姿并于源2374换图，不能采用其低耗时。单独A仍未满足完整输入与90ms，控制均值漂移及局部工作量判据失败，三个候选无一全部合格。RGBD25 blocked表示资格失败，研究交付已结束，不等待批准或追加择优回放。

独立CSV3308项无差异、原生和运行审查完成；112raw/246板端文件一致、16owned退出仅TIME_WAIT、1169旧输入未改。26登记任务无长序列交集，首轮输入前短metadata查询时间不全；计时回放无本任务编译绘图分析。系统有换出及少量majorfault、频率1.2–1.5GHz，不称系统独占/零分页/恒频，cgroup/loss/fulljoin/GT证据限制保留。早期目录权限/参数/父目录错误均在回放前；atomic01停止构建和原始失败记录保留。分析类型误判已撤回，v1/v2所有门相同。

可用只读命令：python3 -B artifacts/rgbd_tracking_optimizations_20261020/verify_archive.py 。不启动板端/回放，不要求未来共享docs不变；重算另建输出。修改仅新归档和三份docs，原文及前后快照保留。未运行根build.sh（当前主树为其他feature），使用隔离runtime冷构建；未新增perf/热路径读钟或调用制图skill。下一步研究优先单独MapPoint原子候选；TODO为完整输入/90ms与可比性、真实几何早退负载、GA2失跟机制、GT及完整生命周期。以下历史交接保留。

# RGBD24 当前交接 — 2026-10-19（A3／Search四完整长包拆分完成，低扰动资格blocked）

用户要求拆清两个阶段具体成本，已完成授权范围，入口 [REPORT](../artifacts/rgbd_tracking_cost_20261019/REPORT.md)。新根 `artifacts/rgbd_tracking_cost_20261019`，板端 `/userdata/rgbd_tracking_cost_20261019`；复用RGBD23原组合＋ORB P core/node/config，无C++修改、无新核心构建。原700/5/3、队列8/8/10、LM/LC、完整前缀1×与基础阶段统计保持，ON仅外部Tracking 19Hz CPU采样。

四轮关开开关完成11604/11640，缺36；两ON clear/gather/Search CPU约1.4/9.3–9.4/14.2–14.3ms，约32%回调CPU。A3gather内MapPoint::isBad及子调用约37–39%，余约61–63%内联/self未细拆。Search状态检查约27%、几何getter20–23%、网格10–11%、描述子距离约1%，均为已识别阶段条件样本占比。4342/4347 raw/script样本通过，LOST0；栈上限/未知leaf保留，不能当逐调用ms或锁等待。此前A1改的是KeyFrame::isBad，本轮MapPoint状态锁是另一条路径。

77门28失败、4轮原完整输入且P95≤90ms门均失败；对照均值本身漂移，ON局部工作量也增加，不能宣称原基线无扰动精确分解或把差值全归perf。独立77门和1246阶段核对一致，476额外算术项通过；234冻结/78raw/1599旧文件/已记录依赖一致，10owned退出，整个四轮无已登记重任务交集。研究交付完成，RGBD24 blocked不是待批准，也不继续择优重跑。

新归档保留源码/成套二进制、源成本审查、原生探针171样本/31解析测试、PLAN、4runs、raw/perf解码、独立复核、报告和起止ledger。实际命令见validation/operations：run_sequence、decode_profiles、check_profile_influence、analyze_tracking_profile、post_audit；首本地主控路径错误与后补叶PC解析失败保留。没有执行新优化、生产替换、GT或完整System生命周期测试；没有调用制图skill或改旧交互图。

下一步可独立审查普通RGBD投影所需几何字段的一次读取候选，先审同步/等价语义，再测试；不要直接切整个LocalMapMode。MapPoint原子状态查询另行审查，不能照搬KeyFrame补丁。TODO仍为内联成本、低扰动、完整输入/90ms、GT/full-lifecycle。只读核验：`python3 -B artifacts/rgbd_tracking_cost_20261019/verify_archive.py`；后续重算用新输出。修改仅新归档与docs/features、validation、session-handoff；before/after及旧内容保留核验随档。以下原交接保留。

# RGBD23 当前交接 — 2026-10-19（组合＋ORB P四完整长包完成，正式资格 blocked）

用户要求“将组合版和ORB P加速版结合后测试”，已完成合并、隔离部署、实际库等价／功能测试及四轮完整长包。入口 [REPORT](../artifacts/rgbd_a1_orbp_20261019/REPORT.md)。候选完整源`source/orbp`，六文件差异；板端新根`/userdata/rgbd_a1_orbp_20261019/{control_01,orbp_01}`，原生产和旧归档未改。头文件／节点／DSO按各自成套匹配，不能混用历史P计时布局。

对照复用原组合成套构建，候选41+2全冷编；匹配实际库四loader与四main通过。全部2910图像关键点逐字段／顺序／描述子字节相同，每版60输出与恢复边界通过。四轮B→P→P→B完整前缀1×、700/5/3、原队列／LM／LC，完成2846/2903/2898/2849，共11496/11640，缺144。两轮统计中位：完整回调均值85.615→79.672ms（−6.94%）、P95 106.998→97.286ms（−9.08%），ORB均值36.824→29.355ms；对照缺125、候选缺19。全部完成state2/fresh/valid/map0，双位姿完整。

78门25失败：控制均值波动1项、局部工作量24项；四轮完整输入且P95≤90ms门均失败。独立原始复算2522字段与主结果一致，无固定工作量单位成本或原2ms因果结论。feature RGBD23 blocked指工程／正式性能未过，研究交付完成，不需要继续重跑或批准。

322冻结、56raw、1514旧输入、227共享库／词典等身份核对通过，8owned退出，整个四轮无已登记重任务交集。原严格收尾technical_audit_pass=false仅11411四条TIME_WAIT，原记录无监听／活动连接；失败及补充解释均保留，没有等待其消失重测。峰温51.058°C，有低频样本；B2 majorfault+1，swap／OOM无增量。完整后台join、系统解释器before SHA、采样丢失及部分cgroup未知，不称全生命周期安全。

执行的构建、部署、native、冻结、四轮控制、归约、独立审查和收尾命令均在归档账本，失败原样保留；没有新增短包、生产替换、LM／LC关闭或GT／活动GBA／Atlas保存重载／完整System生命周期测试，后几项超出本次提取器组合与有限长包范围。只读封存验证：`python3 -B artifacts/rgbd_a1_orbp_20261019/verify_archive.py`；后续测试或重算必须用新输出，保留本轮原始。

修改仅新归档及docs/features.md、docs/validation.md、docs/session-handoff.md，前后快照与保留核验随档。TODO仍为完整输入及90ms、负载可比性和完整生命周期／真值质量。下一步若继续提速，应先针对下游CPU工作量独立选候选，不能直接用此轮均值认定固定单位收益。当前用户请求已完成；以下历史交接保留。

# RGBD22 当前交接 — 2026-10-19（八完整长轮与局部干预完成，因果／低扰动 blocked）

本轮已完成用户授权的深入分析测试，入口 [REPORT](../artifacts/rgbd_a1_causal_20261019/REPORT.md)。没有追加长包、修改生产、关闭LocalMapping或改变退休语义。新归档 `artifacts/rgbd_a1_causal_20261019`，旧RGBD20归档1510文件保持。feature唯一RGBD22，blocked指全局因果／低扰动／性能未过，不是等待批准。

两版ARM完整冷编40+2、匹配实际DSO定向案例及23项解析验证完成。真实A1/A3/Need受控退休场景：选中80→81，参考有效点40→0，插帧true→false，A3仍同一41MP集合。只能证明局部机制，夹具已先join系统后台，没有完整Track/Search/图像输入。四ON自然1308628查询全false、退休入口0，没有在本长包观察该机制；未配置禁用culling，具体门条件未采到。

八轮C关/开、A开/关、A关/开、C开/关全部2910输入完整发送，完成22740/23280，缺540，处理状态均2/fresh/valid/map0，位姿消费者完整；回调P95都>107ms，原完整输入及90ms均失败。观测28门各失败10/2；C关自身均值差1.820ms。源30的Track内当前A1前已754/753MP，同版本也有同类分叉；不能归到当前过滤，更早时序反馈未排除。A3选集／槽位精确分账和旧四轮CPU分账完成，不称全局直接因果。

主／独立归约、自然事件复核与板端收尾通过技术核验，968字段一致；318冻结、128raw、227lib/词典、16owned退出／11411与旧1510一致。八轮总区间无已登记重任务交集；只有checked的历史点检查duration未知。三个ON各majorfault+1，系统swap/OOM无增量；不称恒频、零缺页或系统独占。系统解释器只有after SHA，cgroup／采样丢失计数／物理驻留证据有限。

实际执行：`tools/build_pair.py`最终container03/combined01、`tools/native_execute.py`两次、`tools/freeze_run.py`、`tools/run_sequence.py`一次八轮、`tools/analyze_causal.py`、`analysis/workload_contributions.py`、两个独立归约以及`tools/post_audit.py`。入口和参数以operations/builds账本为准。早期源码匹配／Docker访问／container02停止／新目录权限／独立Python3.8兼容失败均保留，不覆写冻结工具来通过。只读核验命令：`python3 -B artifacts/rgbd_a1_causal_20261019/verify_archive.py`；该命令不回放，不要求未来共享docs不变。

修改仅新实验归档、docs/features.md、docs/validation.md、docs/session-handoff.md；before/after快照在新档。未运行追加图像回放、完整System生命周期／活动GBA／保存重载／真值精度，不以局部测试代替。TODO仍为最早地图分叉前可恢复点状态及单独等待边界干预、完整唤醒证据和整轮低扰动资格；本轮停止，不靠重复相同长包补因果缺口。以下历史交接保留。

# RGBD21 当前交接 — 2026-10-19（36轮交互图交付通过）

用户指定rgbd-callback-charts skill，要求此前长包结果加到旧交互图。已只读使用4个A1归档新增24轮（低扰动4、CPU/profile+sparse8、独立候选8、组合4），合并旧orb_delivery 12轮，共36。默认combined_on_02；旧锁诊断4轮已在旧图，不重复。入口 `artifacts/rgbd_callback_charts_20261019/viewer/callback_breakdown.html`，配置/说明/校验/浏览器截图在同新根目录。旧图/静态图/原归档未覆盖，无新回放/编译/板端动作。

新增68103回调+1737缺失=69840源位置；全36轮102119+2641=104760，每轮2910横轴。完整输入身份及共享RGB/深度预览逐源核对，stamp保持字符串。八色前台分量保留，父子不重复、后台不叠加，A1差额不命名为锁等待；基础detail逐回调完整，额外UBC稀疏与CPU样本未混入。原M0未测仍null；失跟、恢复与缺失未知位置忠实保留。图通过不改变RGBD17–20性能/扰动/质量门失败。

skill脚本/模板原值使用；5973输入SHA、2807856项raw/CSV/百分比/身份、5999原图/来源快照及12历史对象保留核验通过。Chrome469项真实交互与四幅desktop/narrow截图目视通过，无页面异常/consoleerror/失败资源/外请求。源码语义由只读agent审核，新增12旧观测轮33998回调基础阶段完整，76帧未执行局部阶段而非抽样未测。具体检查见checks/MEASUREMENT_REVIEW.json、DATA_VALIDATION.json、browser_check/BROWSER_VALIDATION.json。

命令见README和validation。只读入口：`python3 -B artifacts/rgbd_callback_charts_20261019/checks/verify_delivery.py`。原生成与browser CLI需要新/空输出目录，重做请另选目录。准备时第一次快照误读旧FILES.json包装键并在生成前停，prepare.py适配后正常，未修改输入；记录在operations/PREPARATION_NOTE.json。

修改仅新图归档、docs/features.md/validation.md/handoff；根文档原有工作保留，before/after保存本次快照。RGBD21 passing只指制图和验证交付。图片以相对路径复用旧frame_preview，移动时保留相对目录；没有打包复制149MiB图片。下一步无必需工作。以下历史交接保留。

# RGBD20 当前交接 — 2026-10-16（组合候选四完整长包完成，工程门未过）

用户要求组合容器优化＋原子查询，并已授权板端部署长包对照。已在 `artifacts/rgbd_a1_combined_20261016` 完成两补丁精确组合、40核心＋2节点ARM全量冷编、匹配DSO516正常/OOM/锁行为及atomic42次验证、611项事前冻结，固定container_02→combined_01→combined_01→container_02四长轮。每轮原2910完整前缀、1×、700/5/3、队列8/8/10，LM/LC及原基础阶段统计保持，未加采样/细锁记录。旧归档/生产未改，无短包替代或追加择优。

四轮完成2847/2842/2844/2858，缺63/68/66/52，合计11391/11640、缺249，已完成全state2/fresh/valid/map0，无reset/换图，两pose消费者完整。A1尖峰25→1、过滤P95中位0.197→0.072ms；整回调mean中位84.585→86.584ms、P95 106.815→107.580ms、缺失中位+9.5。66门17失败，local KF/MP中位+8.95%/+10.26%、控制自身均值超波动门，不能归因补丁单独成本。四轮正式完整输入及90ms均失败。组合可限定研究运行，未晋升；RGBD20 blocked表示性能/可比性失败，用户授权验证已完成。

主归约与补充01独立raw401544项/66门一致，首份独立脚本错误追加backend_joined=true造成4额外失败已保留并更正，实际join仍未知。唯一组合尖峰源722过滤差额0.001667ms、A1粗残差10.691302ms，不能命名精确邻居、锁等待或调度。原子退休false窗口、repeat true未来写入、lifetime及静止归档前提保持，未认证完整System/GBA/惯性/Atlas/GT精度。

实际命令、失败保留和退出审计在builds/tests/preflight/operations。最初新root权限不足后sudo仅创建实验目录；stage一次并发重复被拒；native全通过后本地Path.readlink Python3.8错误只重核原收集数据；native staged driver与稍后template两行集合检查差异绑定；收尾Vocabulary多级软链期望错误仅据原SHA/路径更正。无fixture或图像重跑、无候选/门变更。四轮期间各代理idle，登记重型任务与全序列不交叠；ON1/B2有1.2GHz采样，资源/cgroup/loss未知项未虚报0。

复核入口：REPORT.md、analysis/COMBINED_GATE.json、analysis/RESULTS_SUMMARY.json、analysis/INDEPENDENT_COMBINED_LONG_SUPPLEMENT01_20261016.md、operations/POST_LONG、FINAL_STATE.json。主checker写衍生文件且预期exit1；封存后使用 `python3 -B artifacts/rgbd_a1_combined_20261016/analysis_supplement/verify_delivery.py` 只读核验。根目录不同feature的build.sh未运行；使用匹配runtime源和固定离线ARM工具链。修改仅新实验归档与features/validation/handoff，原三文档已在before备份。

下一步若用户另授权优化，应针对剩余整回调成本和地图工作量立新研究；本轮停止于既定四轮，不因A1下降直接采用组合版。以下历史交接保留。

# RGBD19 补充交接 — 2026-10-16（8完整长轮完成，无性能晋升）

用户已明确允许板端部署测试，原授权阻塞解除。有限研究及测试交付完成；RGBD19继续blocked，原因改为可比性／性能门失败。入口 [REPORT](../artifacts/rgbd_a1_candidates_20261015/REPORT.md)、FINAL_STATE.json、[源码运行补充](../artifacts/rgbd_a1_candidates_20261015/source/DELIVERY_RUNTIME_ADDENDUM.md)。生产与旧归档未改；3实验包及全部数据留NEW供复核。

实际ARM container516案例+mutex/分配边界、atomic42进程与发布/重复true/归档对照通过；359项原子入场核对及容器独立审核通过。native01被旧依赖core loader guard拦截未main，新native02只改隔离依赖搜索，44次实际main均加载正确库，源码和ELF不变。3包部署SHA、227共享库＋词典指纹存档。

两矩阵各427冻结项，8完整长包B→C→C→B连续完成，首次缺本地runs目录在节点/回放前退出0输入的准备失败保留，补空目录后执行原冻结8轮；没有追加或短包。控制／候选完成数为container2820/2829/2835/2833、atomic2836/2859/2843/2859，缺90/81/75/77/74/51/67/51，总22714/23280。已完成帧state2/fresh/valid/map0、无reset；不对缺失推断状态，也没有真值精度认证。

两个独立raw归约与主66门一致：container7失败（基线均值及local KF/MP波动），atomic5失败（工作量/控制3、P95中位改善不足2%、第二对回退3.761ms）。容器P95中位112.550→110.226ms（2.064%），原子108.738→107.985ms（0.693%）；A1尖峰311→25／254→7，过滤P95约降98%／99%。所有8轮完整输入和90ms原门均未过。保留容器作较小同步改动研究起点，不合并两补丁、不生产晋升；原子剩余7尖峰在粗残差，不能命名为精确邻居锁等待或调度。

所有代理/编译/CPU采样和离线分析在8轮前结束，全部8轮后才恢复。独立审计确认heavy job与总外层运行区间不相交，host_load禁标全0。initial/final idle均是放包前两次检查，final不冒充结束声明。C_on02覆盖窗口有2次1.2GHz样本；两B1各1 majorfault，观察窗swap/OOM增量0，空cgroup字段不当作零。末次16个owned PID退出，板/host11411无残留；226/227映射文件可与pre指纹对照，系统解释器仅post SHA。详见operations/POST_LONG_20261016及preflight/RUNTIME_DEPENDENCIES02.json。

复核命令：`python3 -B artifacts/rgbd_a1_candidates_20261015/analysis_supplement/verify_delivery_20261016.py`。主归约命令为analysis_tools/check_candidate.py container/atomic，预期1且会写分析输出；封存后复制目录再重算，优先只读验证。图表、597尖峰、全部23280源位置与固定四段、逐帧非重叠回调成本及outside-A1条件算术界限已交付；未用阶段P95相加减。

未运行根目录其他feature构建／生产部署、完整System/LSan/TSan／活动GBA／惯性／完整Atlas保存重载／真值精度。TODO仅供后续另开范围：稳定局部地图工作量对照、A1外A3/局部点搜索成本，以及原基线逐锁持有者/唤醒调度证据；本轮8/8次数已用完，不自动继续重跑。以下准备阶段与历史交接保留。

# RGBD19 当前交接 — 2026-10-15（两个候选已可审查，板端待授权）

状态blocked。新实验 [REPORT](../artifacts/rgbd_a1_candidates_20261015/REPORT.md)、[两独立源码补丁](../artifacts/rgbd_a1_candidates_20261015/source/DELIVERY.md)；container_02及atomic_01已各40核心＋2节点ARM完整冷编、0复用，baseline_01复用原成套pair。原128文件逐份对照：容器仅KeyFrame.cc，原子仅KeyFrame.h/.cc。未采用row_tiles/C1/细锁记录；生产及旧RGBD18保持。

本地container实际类／提取方法1028正常案例、6受控异常、真实mutex链与逐分配失败通过，6邻居分配15→3只是宿主次数。atomic10项源边界、两变体语法、83查询点及30项最终源码／ARM产物绑定通过；实际库反汇编确认LDARB无mutex。原子false退休窗口／重复true旧mTcp、寿命pin缺失及静止归档前提均已记录，未作严格图输出等价或完整System安全承诺。四ARM fixtures编译完成，42次原子进程计划和两个container linked实际库运行尚未执行。判定器9测试通过；pending entry会被冻结工具拒绝。

阻塞：自动审批拒绝NEW/container_02远程部署并要求用户明确授权（operations/DEPLOY_APPROVAL_REJECTION01.json），命令未执行，没有换路径重试。先前仅独立获批创建NEW父目录；本轮上传0／板端fixture0／回放0。主机及既有镜像无QEMU，本地ARM loader ENOEXEC。已完成本地完整staging、具体提议命令和SHA，正式RUN_FREEZE尚未创建。

只读核验：`python3 -B artifacts/rgbd_a1_candidates_20261015/tools/verify_review.py` 已通过1590项固定输入；三个旧归档3307文件SHA一致，真实主机进程与重型任务账本均空。当前状态见新目录FINAL_STATE.json。构建／宿主测试／反汇编的实际命令及失败尝试保留在builds、tests、preflight和operations/jobs。container_01误用41 TU断言在0编译时失败，核实原CMake40后container_02成功；其他宿主依赖／语法宏／符号选择／判定fixture计数准备失败没有删除或冒充性能失败。未运行根目录不同feature的build.sh/catkin；使用匹配原运行源和固定依赖的成套ARM构建。

下一步仅待用户批准板端隔离实验。批准后按PROPOSED_REMOTE_ACTIONS及对应最新staging执行3包／4探针，审核实际输出、DSO加载身份和归档，填写绑定证据的ENTRY_VALIDATION；再生成正式PLAN、RUN_FREEZE、逐轮全代理idle声明。分别完整B→container→container→B和B→atomic→atomic→B，共最多8长轮，2910／1×／700／5／3／队列8/8/10，先实际库前置，无短包替代、额外择优重跑。回放期间所有编译、stdin分析和绘图停止，矩阵后才离线归约；这是对RGBD18隔离遗漏的修正。若任一具体安全反例出现，停止相应候选。不得据阶段差额宣称调度或LocalMapping／LoopClosing根因。

TODO：实际库并发／双向TEXT-BINARY／截断归档、完整长包效果与质量、基线波动、正式90ms及完整输入门。完整System／活动GBA／Atlas重载／真值质量和可用唤醒调度证据未认证。以下旧交接保留。

# RGBD18 当前交接 — 2026-10-15（8完整长包完成，原基线低扰动因果 blocked）

用户四步研究已完成授权范围的采样、粗观测和两组关→开→开→关完整长包，共8轮/23280源位，无短包替代或额外重跑。入口：[REPORT](../artifacts/rgbd_a1_sparse_profile_20261014/REPORT.md)、[SUMMARY](../artifacts/rgbd_a1_sparse_profile_20261014/analysis/SUMMARY.json)、[FINAL_STATE](../artifacts/rgbd_a1_sparse_profile_20261014/FINAL_STATE.json)。研究交付完成；RGBD18保持blocked，原基线机制与不影响实时性资格没有通过。

原baseline128文件与成套二进制保留。LM原二进制19Hz用户CPU采样由实际GDB TID/start_ticks确认；全部profile侧同前置流程。粗观测仅两个ON换隔离纯观测版：原isBad不动、Tracking全调用2wall读钟帧汇总/最大/累计＋邻居粗阶段，LM每64次UBC外层与少量内部边界，抽样先于时钟/打包。缓冲3473664B；ARM七批新增成本中位192.789ns/过滤调用、191.269ns/100邻居UBC外层；不当整机上界。实际适配器host/ARM、解析器、源码逻辑/锁析构位置核对通过；41核心＋2节点ARM完整冷构建，0对象复用。

八轮完成22691、缺589，缺失全部单列，各轮完整输入与90ms P95均未过。profile B2源2345–2379、sparse ON1源258–292各35非fresh/valid（34state3+1state0）且换图；两消费流均保留例外。profile67门25失败，sparse67门19失败（OFF控制P95/缺失波动、ON1质量与地图、配对均值/地图），不能用较小中位均值变化宣布低扰动。独立原始437866项及背景4254项检查通过。

CPU原始约4300样本/轮，首次离线perf script180s超时原样留存；仅decoder关闭inline展开恢复，原raw/采样/冻结解析器不改。原core没有GNUbuild-id、node只有ABI note；冻结硬身份门缺口在前置未审全，实际SHA/映射可信线索与该失败分开保存。热点不是逐A1原因，不拼接不同matrix。粗观测严格尖峰128/102，过滤累计>10ms120/101、单次>10ms2/7、邻居例外8/1；ON2仅5尖峰少量局部同锁交叠，其余holder未知，ON1禁跨reset身份关联。

两矩阵回放无本任务编译/绘图，记录board温度频率及host负载；但sparse B1期间仍有主机stdin perf raw扫描约18.6/22.3s完成，分析隔离不完整，精确扫描起止未记录。开始该轮前未确认扫描结束，额外影响未知；analysis/HOST_ANALYSIS_OVERLAP01.json补录。禁止工作字段8轮仍空，检测器漏掉stdin分析，不代表全部分析隔离或系统独占。当前FTRACE/SCHEDSTATS配置未设、入口/权限限制有原始只读证据，缺wakeup无法拆阻塞/醒后调度。候选进入false，0选择实施验证；容器正常语义/分配失败差异、mbBad退休中间态/发布/裸指针/delete/序列化单独审核，没有去锁或关后台。

收尾：363新冻结项、旧1314＋659归档、baseline128及5外部输入SHA通过；真实host/board20登记身份退出、无活动SLAM、11411释放、6包文件SHA匹配。生产源码/部署没变，board原始数据保留。本仓库仅改features/validation/handoff，旧内容在新before存档；其他新增均在rgbd_a1_sparse_profile_20261014。只读清单核验：`python3 -B artifacts/rgbd_a1_sparse_profile_20261014/operations/archive_verify.py`；其余实际复算/执行命令见REPORT和validation的RGBD18节。

未执行候选/生产修复、真值精度及完整保存重载/GBA生命周期：本次纯观测且候选前置失败。TODO：原基线质量/地图工作量可比性，覆盖逐尖峰的holder及实际唤醒证据；后续另开研究，不追加本轮择优回放。以下历史交接原样保留。

# RGBD17 当前交接 — 2026-10-14（四完整长包完成，原基线低扰动因果 blocked）

用户要求补长包，已在新隔离归档完成B→ON→ON→B四轮原长包、1×完整前缀2910对。入口：[REPORT](../artifacts/rgbd_a1_lowperturb_long_20261014/REPORT.md)、[SUMMARY](../artifacts/rgbd_a1_lowperturb_long_20261014/analysis/SUMMARY.json)、[LONG_GATE](../artifacts/rgbd_a1_lowperturb_long_20261014/analysis/LONG_GATE.json)。旧短轮失败及1314项旧归档完全保留，不再将旧入口条件阻止此次明确授权长轮；原长数值门未放宽。

四轮处理2833/2811/2820/2843、缺失77/99/90/67、A1尖峰110/241/240/154。两个ON八窗口均匹配LocalMapping，主要UpdateBestCovisibles，七个多次短获取累计、一个单次长持锁；匹配持有区间无切出。源1803第一次邻居检查前12.832ms，完整UBC CPU12.926ms，78次子锁获取仅0.031ms；不能单独指定sort/malloc，更不能外推为原基线已证根因。其他473候选尖峰无完整持有者历史。无sched_wakeup，阻塞/醒后排队未拆。

六perf流完整，356884 SWITCH与CSV一致。独立原始209655项、应用/perf二进制4793935项通过；67长门48通过19失败，P95+5.246ms、两对缺失+22/+23、尖峰频率与局部工作量改变且B自身有不稳定。ON2期间未隔离主机绘图负载限制保留；最终图在全部回放结束后生成。候选机制可核对，原基线低扰动因果没有资格，RGBD17保持blocked，不宣称性能通过。

本轮只复用原基线和diagnostic_02配套二进制，未改C++或生产配置、未重建、未追加长轮。新增目录内保留PLAN、178项RUN_FREEZE、源码镜像、二进制、tools、完整runs、独立analysis、operations与before文档副本；根文档只更新features/validation/handoff。最终主机真实/proc和SSH只读核验：新旧冻结项及板端包一致，实验进程全部退出、11411释放。复算命令见REPORT；总门exit1预期。

TODO：后续若继续原基线目标，应先缩减高频持锁观测成本并通过长包对照，真正拆醒后调度还需唤醒事件；不得把现有结论转成去锁/原子bool或关闭后台线程的生产修复。当前预定两诊断长轮已耗尽，停止重跑。以下旧交接按原样保留，其“未长轮”仅指上一归档。

# RGBD17 当前交接 — 2026-10-14（工具验证完成，原基线因果 blocked）

用户要求在实时性约束下补齐同轴锁边界、持有者与调度证据。本轮隔离目录 `artifacts/rgbd_a1_lowperturb_20261014`，入口 [REPORT](../artifacts/rgbd_a1_lowperturb_20261014/REPORT.md)。新工具已实现并完整冷编译上板，四短轮全完成；因冻结控制稳定性门失败，未进入四长轮。原基线原因仍未知，不能认定LM/LC冲突或低扰动验收通过。当前feature RGBD17=`blocked`。

保留原current二进制B，ON为diagnostic_02；175输入冻结，PLAN revision5 SHA0175aab4...在首次回放前确定。43核心单元冷编、0缓存复用，132构建输入和最终源一致。Map.h包含问题首次失败保留，后续完整构建成功。四短B→ON→ON→B共480/480，正常跟踪/位姿/地图/退出，六条perf完整；ON均值差+0.628730ms、P95差+1.046863ms在限额内，唯一失败B的P95跨度4.695390>3.718855ms。因此本批无法可靠分离基线波动与观测影响，0长轮、0实际A1>10ms尖峰。独立61,988项原始检查通过，证据analysis/SHORT_RAW_INDEPENDENT_REVIEW01.json。

新增观测仅source/diagnostic：所有connections路径、退休二级锁、UBC粗CPU、五组预分配独占环、三目标perf切出/切入。每锁约0.510us额外；20k嵌套仍增外持锁9.980ms；总预算63.672MiB。host46项解析/门测试和真实ARM已知原因探针通过。无sched_wakeup；nonrunnable-out后阻塞与醒后排队无法拆分；perf onCPU非CPU计量，features/pose其余持有者/Map锁与覆盖外仍未知，Shutdown不假称all-join。

已运行：build_pair.py diagnostic --attempt 01（失败保留）/02（成功）、deploy.py两隔离包、run_round.py四个current_short_*、check_influence.py short（预期exit1）、analyze_capture.py两ON（0timeline）、单元测试/源操作逆变换、能力与ARM探针。精确命令在构建RESULT、运行RESULT、preflight和operations；离线复核见validation末节。板端8个登记进程均消失，无残留SLAM；没有改内核/governor/算法/参数或生产运行包。

修改文件：新实验目录全部实现/工具/证据，docs/features.md、validation.md及本交接；修改前三文档在新目录before，下面历史交接原样保留。未执行四长轮、真值精度、生产修复，因为短门未通过。TODO：下次另立计划先验证基线稳定性/对照长度；若要求精确醒后延迟，还需支持调度唤醒事件的测试环境。禁止通过重跑挑选结果或改本次判据续跑，也不能据此去掉isBad锁或关闭后台线程。

# RGBD15 当前交接 — 2026-10-14（P M0及两轮新M1加入交互图）

用户先要求加入刚完成P长包，随后追加“再额外跑两轮开启细观测”。两项均完成：旧P/B_row M0数据加入，两轮P M1实际板测完成并进入两份交互图。入口：[包含旧交付版的12轮图](../artifacts/rgbd_callback_breakdown_20261014/orb_delivery/callback_breakdown.html)，[原10轮图](../artifacts/rgbd_callback_breakdown_20261014/callback_breakdown.html)。默认P M1细观测01，可切换02/M0/B_row/历史版本，RGB与深度预览及失跟底色保留。M0未采集的A1/A3等显示未测；M1显示互斥阶段分解，后台BA不相加。

新实验目录`artifacts/orb_25ms_impl_20261013/epoch01/board_detail_20261014`：P冻结包不变，仅开M1；两轮处理2891/2892、缺19/18，已处理均正常跟踪，新鲜有效、map0/reset0、自然退出；回调mean82.213/77.755ms、P95105.926/105.535ms。PLAN/EXECUTION保存完整运行命令，SUMMARY/FINAL_STATE及124项平铺MANIFEST已封存；16个自有PID身份均已退出。旧M0归档148文件及12冻结工具未改。没有额外重跑、重编译、算法/参数修改或生产部署；M1开销未量化，正式完整输入/90ms/25ms不通过，RGBD13原I0失败保持。

RGBD15 passing仅表示研究与图表交付完成，RGBD16既有结论保留。本次修改共享HTML模板、生成器、数据/浏览器核验脚本及输出；新增p_board_update归档/验证/清单工具、新板测目录，更新features/validation/handoff。源CSV/包身份/源图像关联、旧对象与静态图不变均核验；4新增轮11562行、12轮273032项值、24CSV/12加载身份、68936行图像关联、Chrome275/303项通过。两份页面桌面/窄屏目视通过，两套输出清单5941/52项。完整复核命令见validation末节；两轮原始命令见新实验EXECUTION。

工具问题已保留：Python3.8核验器合并字典写法修正；长轮次名导致桌面工具栏换行使预览窗越界，限制选择框宽度后原断言通过，未放宽检查或改数据。图表变更前快照在p_board_update/before；失败原因及截图仍保留。未运行本轮之外的性能矩阵、精度/生命周期测试；TODO：M1开销与持续缺输入的原因尚未由本次两轮隔离归因。以下历史交接原样保留。

# RGBD16 当前交接 — 2026-10-14（最新 P 板测完成）

RGBD16 补充（用户要求加入交付版）：REPORT新增10/12原交付版两轮与本次B_row/P四列历史对比，数据与口径见 `board_test_20261014/delivery_comparison/SUMMARY.json`。旧六份CSV SHA、独立统计核验通过；PSS相对旧两轮中位低19.16%，回调均值79.04ms接近旧78.38/80.46ms，最大值未改善。不同批次/detail观测/内存采样范围，不能作为正式同口径收益。此次仅整理现有证据，无新板测；原报告和清单已在delivery_comparison/before备份，原始数据保持，feature仍passing仅指研究交付。

按用户要求对最新 P（行并行＋FAST 工作区复用）完成 B_row→P 固定一对真实板端长包研究，入口 [REPORT](../artifacts/orb_25ms_impl_20261013/epoch01/board_test_20261014/REPORT.md)。每轮完整发送2910对，处理2881→2898，回调mean82.385→79.044ms、P95106.196→101.023ms，ORB P9540.930→36.438ms；采样峰值PSS1017.82→1011.13MiB。已处理帧均state2、新鲜有限、map0、无reset；P尾段727/727，全程仍缺12帧。两轮自然退出且证据完整，独立5779帧复算通过。

RGBD16 passing仅指研究测试交付。原I0观测开销失败与RGBD13 blocked保持，原I1/I2没有续跑；本次新指令的独立输出在epoch01/board_test_20261014及板端runs/research_20261014_*，不覆盖历史结果。完整输入/90ms/严格ORB25ms未过，最大回调未改善，单对不能证明稳定重复收益或修复全部历史失跟。M0共有槽位列未用于地图工作量解释。

复用冻结成套库/节点，未改运行源码、重编译或生产部署。新PLAN/EXECUTION、原始输入/位姿/资源/退出记录、SHA、汇总脚本和报告均在新目录；原工具只读复用并快照。板端4个及主机12个登记进程已退出，板端原始数据保留。实际复核：`python3 -B artifacts/orb_25ms_impl_20261013/epoch01/board_test_20261014/tools/summarize_pair.py`；源码/构建/依赖/实际加载/回传SHA均核验。未运行正式重复矩阵、真值精度、完整生命周期；旧开销问题与输入缺失原因仍待解决。修改仅features/validation/handoff及新研究记录，原三份文档在before备份，下方历史内容原样保留。

# RGBD15 补充交接 — 2026-10-14（RGB／深度图同步预览完成）

已更新[用户当前八轮图](../artifacts/rgbd_callback_breakdown_20261014/orb_delivery/callback_breakdown.html)和[原六轮图](../artifacts/rgbd_callback_breakdown_20261014/callback_breakdown.html)：光标联动原包RGB及深度伪彩缩略图，蓝色选帧线、锁定、源帧输入、前后按钮、方向键、窄屏收起浮窗；固定0–5m深度色标，黑色无效，>=5m饱和。按source_ordinal对应精确stamp和双seq，缺失回调仍显示原包输入并明确本轮未完成回调。所有轮次共享frame_preview目录，直接离线打开，移动页面要保留相对目录结构。

RGBD15 passing仅指图表交付。完整提取2910对/5820缩略图（320×272，约148.47MiB），原bag/index/config SHA前后匹配，2910深度PNG精确回读；5824文件SHA/大小及5820尺寸解码通过。16原CSV/45734行关联核验通过，50份既有统计与静态图SHA不变；180458项值及8份加载身份复核通过。Chrome193/217项通过，包含连续悬停刷新、异步旧响应防覆盖、两图成对提交、缺图错误清理及全部旧图表检查；桌面和窄屏截图目视通过。命令见validation.md最新RGBD15条目。证据：frame_preview/SOURCE_VERIFICATION.json、VERIFICATION.json、image_preview_update/VALIDATION.json、两份BROWSER_VALIDATION.json和browser_preview_*.png。

修改共享模板、两个HTML生成器及浏览器检查脚本，新增frame_preview提取脚本/素材/清单、image_preview_update绑定核验及浏览器测试；更新HTML、验证产物、两份FILES.json和features/validation/handoff。变更前文件在image_preview_update/before。首次沙箱Chrome因socket权限失败，获自动批准后本地检查完成；首次UI检查暴露预览过高，布局已调整并重新通过。无SLAM代码修改，无构建、重新回放或板端操作；原性能及因果结论保持。以下历史交接保留。

# RGBD15 补充交接 — 2026-10-14（交互图失跟底色完成）

按用户要求更新[原六轮交互图](../artifacts/rgbd_callback_breakdown_20261014/callback_breakdown.html)和[含旧交付版的八轮图](../artifacts/rgbd_callback_breakdown_20261014/orb_delivery/callback_breakdown.html)。共享模板依据实际状态标出5个失跟后尚未恢复区间，80实测帧（78个state3、2个失跟后state0）；2634等缺失帧留白，恢复帧不着色。两张子图同步粉紫半透明底色，区间标签可点击放大，悬停解释重建/未知状态；默认轮次均无失跟，选择long_on_01或long_on_02查看标注。

仅重生成HTML，chart_data、逐帧CSV及静态PNG/SVG共50文件SHA不变，16历史输入SHA一致；22454行状态核对、180458项值检查和8份加载身份核验通过。根目录/八轮图Chrome117/141项通过（桌面/窄屏、实际像素、缺失与恢复边界、切换/缩放/拖动/悬停），无JS异常；目视通过。核验命令见validation.md的RGBD15失跟底色条目，证据在loss_marking/VALIDATION.json、两份BROWSER_VALIDATION.json及browser_loss_*.png。首次沙箱Chrome因本地socket权限失败，已用获自动批准的非沙箱本地Chrome完成。两套FILES.json已更新，变更前文件保留在loss_marking/before。

RGBD15维持passing，仅指图表交付。修改共享viewer.template.html、两个生成器（新增--html-only）、两个浏览器检查脚本、两份HTML及验证产物和本仓库features/validation/handoff；新增共用loss_marking/browser_checks.js。未修改或运行SLAM源码、构建、回放或板端。既有性能/失跟原因结论保持，图中半透明色不是对缺失帧状态的推断。以下历史交接原样保留。

# RGBD15 补充交接 — 2026-10-14（原 ORB 加速交付版图表完成）

用户询问上一图的版本并要求之前加速交付版的图，已补 [八轮交互图](../artifacts/rgbd_callback_breakdown_20261014/orb_delivery/callback_breakdown.html)，默认old_long_01，及old_long_01/02完整回调/百分比图、同批old/current四轮同轴对照。原六轮交付36项SHA不变，新文件全部在原图目录的orb_delivery子目录，变更前文档另存其before目录；旧源码/数据归档未改。

明确版本：old为原按层并行加速交付outer18ba684db428/core4d6df6a2db5a＋RGBD11观测；current为B_common outer2dc402ce0c08/core5ea9afb68322＋RGBD11观测；long_off_01为此current再加diagnostic_05包装和缓冲，ORB_A1_LOCK_DIAG关闭。后者不是后来网格行并行row_tiles或C1工作区复用候选。row_tiles整机长轮只有完整回调，阶段未测；C1尚无整机长轮，不能从独立ORB调用数据补成整机回调。

运行generate_delivery_charts.py、verify_delivery.py及本地Chrome check_browser.py均通过：八轮22454回调/23280源位置，16原CSV和8加载身份SHA一致，180458项值核验，浏览器51项及目视通过。新目录含9组PNG/SVG、8逐源CSV、HTML/JSON和SOURCE_PROVENANCE；同轴对照仅使用2026-10-12同批old/current四轮。RGBD15 passing仅指可视化交付，既有性能/因果/完整输入限制保持。更新本仓库features/validation/handoff；本次只做离线绘图和核对，没有运行SLAM构建、回放或访问板卡。以下历史交接保留。

# RGBD15 当前交接 — 2026-10-14（逐帧回调构成图完成）

用户要求按帧展示总回调耗时及各部分占比，已生成 [离线交互图](../artifacts/rgbd_callback_breakdown_20261014/callback_breakdown.html)、六轮全程及两个细节共8组PNG/SVG、六份完整2910源位置CSV。主图为最新锁诊断long_off_01，可切换long_off_02、long_on_01/02和原地图规模对比current_long_01/02。开观测轮有明确侵入标签，关观测轮仍带新缓冲/包装，与原基线二进制不同；未混称同版本性能对照。

按ORB、reference copy、A1线程CPU、A1 wall−CPU、A3、搜索局部点、局部跟踪位姿优化和其他回调耗时互斥堆叠，黑线总回调、虚线90ms，下图逐帧百分比。A1 CPU与差额不是帧内先后顺序；差额未认定为锁等待。其他保留尚未单独计时的关键帧插入等开销；后台LocalMapping/局部BA不相加。缺失留空并标红、非新鲜标黄、阶段未执行保持空值，未平滑或筛除慢值。

已执行 `python3 -B artifacts/rgbd_callback_breakdown_20261014/generate_charts.py`：12个输入CSV与原manifest的SHA一致，16691个已完成回调的8项和与总时间一致（最大浮点误差5.685e-14ms），17460源位置保留。已执行 `python3 -B artifacts/rgbd_callback_breakdown_20261014/check_browser.py`：Chrome六轮切换、缺失/未执行/非新鲜提示、505尖峰、输入区间、滚轮、真实指针拖动和全程恢复等35项通过，无JS异常；全图和细图目视通过。首次沙箱Chrome因socket权限启动失败，日志browser_attempt01.log保留，同工具在获自动批准的本地非沙箱环境通过。验证证据在VALIDATION.json和BROWSER_VALIDATION.json。

RGBD15 passing仅为可视化交付通过。新增文件均在artifacts/rgbd_callback_breakdown_20261014；仅更新本仓库features/validation/handoff，旧文档在新目录before留存。本次没有修改SLAM源码、运行构建、板端检查或回放；无需硬件即可查看图表，未新增因果或性能验收结论。以下历史交接原样保留。

# RGBD14 当前交接 — 2026-10-14（有界诊断交付完成，原基线原因未确认）

已从产生 `current_long_01/02` 的128文件源码归档隔离增加纯观测，最终05版本冻结130文件。入口：[REPORT](../artifacts/rgbd_a1_lockdiag_20261013/REPORT.md)、[源码补丁](../artifacts/rgbd_a1_lockdiag_20261013/source/DIAGNOSTIC_05.patch)、[源码与构建说明](../artifacts/rgbd_a1_lockdiag_20261013/source/DELIVERY.md)、[最终统计](../artifacts/rgbd_a1_lockdiag_20261013/analysis/final_long/SUMMARY.json)、[逐尖峰索引](../artifacts/rgbd_a1_lockdiag_20261013/analysis/final_long/TIMELINE_INDEX.csv)。RGBD14 passing仅指完成工具验证、有限诊断和证据缺口交付，观测开销、原运行因果确认和正式性能均未通过。

04／05各四短轮均120/120输入、新鲜有效输出和正常退出；04容量外推不足后在首个长轮前无损压缩记录并重做05短轮，旧结果保留。最终同05源码／二进制／700特征／5层／ORB执行者3／队列8/8/10，1×完整建图前缀，关→开→开→关四长轮全部发送2910对：处理2812/2690/2701/2816、缺失98/220/209/94，两个开启长轮有45/35个非新鲜无有效跟踪输出被两个pose流接收。四长完整输入与90ms P95门失败，不能拿共同成功帧验收。

开启长轮全部1101尖峰已生成JSON/Markdown时间线，独立6853995项复核通过；两轮约4200万原始事件独立流式核验、全部过滤调用序号/CPU/慢细关联、时钟和溢出检查通过。互斥主证据1045个持锁CPU占主导、5个持有者低CPU停顿未知、51个混合未知；嵌套第二持有者仅2例，累计重叠0.057833ms。源294捕获Tracking第69次获取70.441ms，LoopClosing的只读GetConnectedKeyFrames持锁wall79.496ms/CPU0.181ms，未确定其阻塞／调度原因。观测长轮P95+30.258%、缺失+4.072个百分点且地图改变，不能用上述机制直接解释原基线。

实际执行命令与记录在实验tools/builds/runs/operations。04为41核心源＋2节点源ARM全量冷构建；05公共头/其余源/依赖/参数相同，先重链接复现04 SHA，只重编译私有记录源并成套重链接，不能称05全量冷构建。主机与ARM probe03各十项定向探针通过；本轮未重做完整System sanitizer、保存/重载/活动GBA、正式真值精度或完整catkin，因为它们不在本次观测诊断范围。冻结工具记录两个离线SQL查询计划修订，真实逐帧精确等价后完成归约；部分结果/退出143与原收集超时恢复证据保留，不改变板端采集源码或增加回放。

板端24个本实验进程身份记录均已退出，未生成Atlas；原生产部署与旧归档未覆盖。精确核验本地原始gzip后仅删除板端两份同一gzip副本，本地完整记录保留。新增源码、工具、原始数据、图和报告均在 `artifacts/rgbd_a1_lockdiag_20261013`；本仓库只更新features/validation/handoff文档，原文副本在实验before目录且历史内容保持。

只读归档验证命令：`python3 -B artifacts/rgbd_a1_lockdiag_20261013/tools/verify_archive.py`。独立新输出归约：`python3 -B artifacts/rgbd_a1_lockdiag_20261013/tools/analyze_lockdiag.py --run long_on_01 --output /tmp/rgbd14_reduction_recheck`。本轮两诊断长轮已经用完，停止，不自动追加回放、关闭LocalMapping或修改mbBad。

TODO（未决，后续另建实验范围）：降低并校准持锁读钟/记录成本，使观测侵入合格；取得每调用可用调度证据；补二级独立持锁路径及快速嵌套第二持有者缺失的独立复检；在合格观测下确认原基线机制和非新鲜输出/输入缺失原因。当前可访问板端配置FTRACE/SCHEDSTATS未启用、schedstat不可读，其他内核追踪能力未被否认。以下历史交接原样保留。

# RGBD13 当前交接 — 2026-10-13（C1交付，整机门受阻）

按25ms计划完成有限候选与独立矩阵，最终P只保留C1工作区复用。源码及六文件补丁、成套核心/节点和回退包见 [DELIVERY](../artifacts/orb_25ms_impl_20261013/epoch01/source/DELIVERY.md)，完整证据见 [REPORT](../artifacts/orb_25ms_impl_20261013/epoch01/REPORT.md)。未覆盖IDE主库或生产部署。

ARM最终16轮22180调用的输入/时钟/逐字段及描述子字节核验有效；长包三轮指标中位数mean30.455→27.537ms、P9538.037→34.227ms，严格25ms未达。I0八短轮全部120/120自然退出，但B_row P95观测开销+3.976ms超过1.693ms门，I1/I2未运行。C2跳过，C3/C4性能拒绝，C5 ARM float bit拒绝；没有追加择优筛选或第二个最终冻结。

RGBD13状态blocked：S7整机受I0门阻止，完整System泄漏/并发/生命周期及正式TUM、闭环质量门未齐备。共有槽位诊断计数的Detailed门控仅为未实施建议，M0相关计数列不能用作有效地图工作量。研究回退选择B_row_integration_r3的匹配核心库/节点/配置，生产版本保持原状态。创建时清单和失败日志不改，最终结论以 [FINAL_STATE](../artifacts/orb_25ms_impl_20261013/epoch01/FINAL_STATE.json)为准。

# RGBD12 当前交接 — 2026-10-12（ORB 稳定化研究完成）

已完成论文和开源工程调研、按网格行并行的一个原型、三对 ARM 全输入提取及预设整机短轮／一对长轮。入口 [REPORT](../artifacts/orb_stability_20261012/REPORT.md)、[源码交付](../artifacts/orb_stability_20261012/source/DELIVERY.md)、[事前 PLAN](../artifacts/orb_stability_20261012/PLAN.json)。RGBD12 passing 仅表示研究交付完成；整机全面稳定化未证实，无生产晋升。IDE 主库、旧归档和生产部署保持。

候选 row_tiles_01 仅改 RGBD11 current 冻结的 ORBextractor.h/.cc：5层检测拆为49个网格行任务，3执行者不变，按原顺序合并再执行原八叉树。198定向、host2910和ARM三对2910均全字段／描述子字节一致；三对P95改善18.30–18.54%，100源分箱均值极差改善21.82–22.51%，均通过事前门。首帧、慢值及末10帧箱保留。light诊断支持均衡负载缩短关键路径，未直接采每task／屏障区间。无地图独立输入仍重现画面相关波动，候选量变化是主要依据。

候选40核心＋2节点冷构建完成，编译命令同RGBD11，仅两个提取器输入不同。核心SHA c20d4ff0aa66286ab511f13932539e2fc7ae087c3ddf9a2114d9cfeece760514；节点重编后仍为c2bf17c62ab656b535d92b73b4aa2dc8755e0dee737a03a7194f9b54e56bdba5。隔离板端包为 `/userdata/orb_stability_20261012/integration/{baseline_02,row_tiles_01_01}`。候选short120完整新鲜有效输出；long两轮全部发布2910，实际处理2831→2881，平均84.122→83.433、P95113.340→104.974、标准差14.864→11.616ms。但最大145.690→152.633，相邻源帧绝对差P9518.099→20.642，第三段均值72.960→80.915ms，不能宣称整机全面稳定。两侧detail=false，core阶段／地图计数未采；map_id全部−1，不能据此证明无reset或无地图分裂。Shutdown返回不等于独立后台join证明。

严格LSan仍失败：候选和baseline-only宿主TBB均792B／3分配，未关闭检测或增加suppression。早期两份诊断为转录，原二进制SHA未知。准备失败、词典二级symlink误判和最终核验器Path参数错误保留在PREPARATION_FAILURES；后两项仅修工具，原始数据不改。集成依赖228项SHA前后一致，独立复核94164项通过。资源采样不覆盖完整启动；系统pswpout基线＋6、原型0，不能声称全批无系统换页。

只读交付核验：`python3 -B artifacts/orb_stability_20261012/tools/verify_delivery.py --integration-summary integration/analysis/pair_01/SUMMARY.json --history rgbd11 --history a1`。最终检查记录为FINAL_VERIFICATION_02.json，完整新归档见ARCHIVE_MANIFEST.json；事前PLAN和RUN_PLAN的创建时status保留，收尾以FINAL_STATE.json为准。板端本轮活动进程和构建容器均无，研究ROS端口已释放，原RGBD09 1391项及RGBD11 693项历史SHA不变。

后续如另开研究，优先解释整机第三段退化与相邻尖峰：同口径有限观测ORB／局部跟踪／A1／A3的wall和CPU、局部KF／MP／槽量及后台活动，再决定调度实验。ORB内下一候选为固定FAST判据的小ROI工作区复用，已有NEON，不能把简单开NEON当新收益。完整输入、90ms、真值精度、正式重复矩阵和生命周期仍未验收。本轮未自动追加试验。以下历史交接保留。

# RGBD11 当前交接 — 2026-10-12（地图规模对照研究完成）

已完成用户要求的旧特征提取加速交付版与当前 B_common 同口径观测、成套 ARM 冷构建、四短轮和旧→当前→当前→旧四长轮。入口：[REPORT](../artifacts/rgbd_scale_compare_20261012/REPORT.md)、[代码差异](../artifacts/rgbd_scale_compare_20261012/CODE_COMPARISON.md)、[源码交付](../artifacts/rgbd_scale_compare_20261012/source/DELIVERY.md)。RGBD11 passing 仅指研究交付；四长轮缺输入且 P95>90ms，没有生产替换。

双版本隔离目录为 `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_scale_20261012_old` 和 `_current`，嵌套实际库为 `core`。旧基 outer18ba684d/core4d6df6a2；当前基 outer2dc402ce/core5ea9afb6。本次补丁及三个新增文件、构建后 CMake/package.xml 声明修正均在 source 归档中，工作树保留未提交研究修改；不以外层旧 orb_slam3 目录代替实际 core。未合并或打发布标签。

旧/当前重新编译32/40个核心源，零旧核心对象复用；节点/库及每轮实际 maps/SHA 一致。core/Recorder/分析定向验证、四短轮120/120、四长轮全部正常退出和回传校验通过。独立长轮263094项断言通过。首部署因父目录权限失败保留，仅创建本次板端 `/userdata/rgbd_scale_compare_20261012` 后重试成功。两成套包 old_01/current_01 和八轮原始记录保留。最后登记16个板端PID均已不存在，实验容器和主机回放均结束。

旧长轮处理2882/2881，当前2847/2825；全程轮均值中位数79.419/88.320ms，差8.901ms（11.21%）。ORB差约0.057ms，主要差异在局部跟踪。第三段旧65.566/68.975ms、当前85.549/82.135ms，局部工作集合不同；回调与局部规模关联更强，不能断言全局累计增长单独导致全程变慢。分箱和逐帧CSV/PNG/SVG全部归档。没有删除缺帧或取交集验收。

只读核验：`python3 artifacts/rgbd_scale_compare_20261012/tools/verify_archive.py`。另目录重算：`python3 -B artifacts/rgbd_scale_compare_20261012/tools/analyze_scale.py --output /tmp/rgbd_scale_recheck_20261012`。原始八轮命令/退出码见 runs 下 RESULT.json；构建命令见 builds 下 RESULT/manifest；源码恢复顺序见 source/DELIVERY.md。本次未跑完整catkin、完整System LSan、真值精度、正式三对S316/TUM/闭环矩阵或完整保存重载，均不在本轮相关性研究验收内。

TODO：输入缺失具体环节、两版中段局部工作集合为何不同、相同合法对象工作量下引用维护净成本、A1具体等待写者仍未归因。旧退出泄漏/活动生命周期/精度缺口保持。后续如另开研究优先隔离局部集合与reference/A3成本；不自动调裁剪参数或回退所有权修复。以下原交接逐字保留。

# RGBD10 文档汇总 — 2026-10-12（完成）

按用户要求将上一轮 RGBD09 的修复、全部测试和回放记录、分析结果集中到一个独立文件：[S316_RGBD_REPAIR_AND_ANALYSIS_SUMMARY.md](../artifacts/rgbd_a1_a3_summary_20261012/S316_RGBD_REPAIR_AND_ANALYSIS_SUMMARY.md)。前面用直白语言说明修改和结果，后面汇总源码/八提交、构建、九回放、63份RESULT、完整缺输入源号、A1诊断、A3固定工作量/维护/内存、全部失败、未决和回退，以及原始证据入口。日志窗口修复通过，具体等待原因未决，唯一位图原型拒绝；研究完成和正式性能未通过分别记录。

本次只新增上述Markdown及本条交接/RGBD10 feature行。编辑前已有features/session-handoff/validation修改逐字保留，validation未增加修改；原RGBD09归档和历史状态均保持。未修改运行源码，未构建、部署、访问板端或回放，因为用户本次只要求整理。

验证命令：`python3 /tmp/verify_rgbd_summary_20261012.py`，对照原JSON/补丁和完整原SHA索引，2043项校验、221处本地链接、33个表格通过；原1391文件SHA一致、历史feature行与原交接后缀保留。`git diff --check`通过。可长期复核的只读基础命令已写入汇总第17节；临时验证脚本不作为新运行或新实验交付。

TODO仍以汇总第14节及原KNOWN_ISSUES为准：完整System退出泄漏、共同TUM构建差异、统计侵入、A1具体等待原因、活动GBA/完整保存重载/C1、精度及正式重复矩阵均未关闭。本次未重新核验板端当前环境，不把上一轮快照当今日事实。建议保留共同日志修复，以共同版本作为后续研究起点；新实验需另登记范围，不自动续跑。以下原交接逐字保留。

# RGBD09 当前交接 — 2026-10-10（有界研究实施交付完成，正式性能未通过）

已实际实施并完成用户指定L1修正、一次A1诊断和唯一A3位图原型筛选；入口：[REPORT](../artifacts/rgbd_a1_a3_impl_20261010/REPORT.md)、[A1_DIAGNOSIS](../artifacts/rgbd_a1_a3_impl_20261010/A1_DIAGNOSIS.md)、[A3_SCREENING](../artifacts/rgbd_a1_a3_impl_20261010/A3_SCREENING.md)、[SOURCE](../artifacts/rgbd_a1_a3_impl_20261010/SOURCE.json)。RGBD09 passing仅指本阶段有界研究交付，正式晋升未通过；候选决定为拒绝，无生产替换、merge或发布。本轮已无构建/回放进程、容器或待回传日志。

共同双仓 `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_a1_a3_20261010`：outer2dc402ce0c08d0658eba508f87436ecf7da69c33/core5ea9afb683224d2245ce31455c41187f54c67b42，成对tag `rgbd-a1-a3-common-20261010-01`。候选双仓同根加 `_p`：outer1899d5891e52c5263bffb932d163da9b8b6379d9/core53ebbcfca17b2b448ef39e425dcddccddd2cebc8，成对tag `rgbd-a3-bitmap-20261010-01`。两组clean、共同epoch `rgbd_a1_a3_common_20261010_01`；W/D/M/P独立提交，初始化修复保留。回退使用已保留共同工作树、成套节点/库及独立板端 `/userdata/rgbd_a1_a3_20261010/a1a3_common_01` 包，候选包另置 `a3_bitmap_01`。

L1统一选择/expected/reserve/JSON，2910输入源窗2621–2910、expected290；实际缺输入与overflow分开，短120前后各6，空/短边界保留。T3完整前缀实际248细记录/42缺输入、overflow0。A1真实总段/投票/过滤端点与CPU、有限轮转isBad样本已实施；约99.90%的wall−CPU差额在KF过滤，具体writer/scheduler unknown，有限样本未覆盖主差额。按范围不追加A1诊断。

唯一位图按KF逆序/槽升序只跳空identity，expired非零identity仍执行weak；强owner、marker、原锁顺序和原释放范围保留。host/ARM成套39 TU冷构建完成，host20/ARM18真实ELF；实际方法写者/地址复用/重叠并发/失败回退、严格ASan/UBSan/定向LSan与双向单KF归档通过。原88B注入先命中OpenCV的工具错误及静态ASan链接/沙箱LSan失败保留，仅修正独立测试工具；实际性能源码与epoch不变。完整System保存重载、PostLoad缺失ID/特定OOM和ThreadSanitizer没有冒充通过。

T1三个短轮及唯一事前触发反序对全部120/120；L0侵入≤2%未证实。T2/T6共同与候选长轮分别2823/2910、2818/2910，双方完整源集合、callback/A3工作量、关键帧规模、资源与异常连续段保留；不以共同成功行交集验收，也不以较少输入/建图宣称收益。固定夹具首次/预热/正式与维护/释放成本另列，只支持其测量范围。没有第二原型或连续参数搜索。

已知完整System LSan 16,803,462B/6612、共同TUM reset差异、活动GBA/完整保存重载/C1及完整轨迹精度/正式重复矩阵仍在KNOWN_ISSUES。它们是研究非阻断或后续晋升要求，不自动展开System析构工程。下一步由用户明确后续研究或晋升范围；本提示词授权的有界实验已经结束，旧RGBD06/07 blocked和RGBD08提示词交付事实不变。以下历史交接逐字保留。

# RGBD08 当前交接 — 2026-10-10（下一会话提示词交付）

用户已调整后续优先级：降低 S316 长程后段持续处理成本与异常尾延迟；不开展完整 System 所有权/析构重构，约16 MB退出泄漏保留记录，出现运行期增长/资源压力/慢帧直接证据才重评。下一会话按 [SESSION_IMPLEMENTATION_PROMPT.md](../artifacts/rgbd_a1_a3_plan_20261010/SESSION_IMPLEMENTATION_PROMPT.md) 执行。该新范围取代以下历史交接中的相关研究前置阻断与“A1未决则停止整个性能主线”，不改写旧实验或验收结果。

执行顺序：保留初始化修复→L1窗口语义修正独立提交→既定一次A1投票/KF过滤诊断，未决即归档并继续→一个可回退A3原型及有界筛选。各项改动独立提交，控制和候选同含共同修复/观测并冻结新epoch。完整精度、退出泄漏清理、活动生命周期与正式重复矩阵仍是后续晋升要求，本次提示词不自动启动正式矩阵。

父双仓仍为 `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_system_init_fix_20261009`：core `96a912876c1602afdd8d9f0c2e851b671ae665b1`、outer `637915d72eba10361b031e51fa48aa149b724ad6`，共同标签 `rgbd-v3-c0-init-fixed-20261009-01`。2026-10-10本地核验clean；板端可用性由执行会话重新核验。

只读归约旧C0固定尾669条已处理行：每帧slots约93568.90、null identity约59262.90（63.34%）。提示词优先审查只跳过空identity槽的有序位图原型，覆盖实际槽写者且保留weak/bad/marker/owner语义；若需超局部生命周期重构，在实施前一次性改选固定容量的帧内两路hint，只实现一个。空槽比例不能换算为CPU收益。证据见新目录 `A3_WORKLOAD_REVIEW.json` 与可复现归约脚本。

本轮仅生成文档、来源清单及旧数据归约；未修改运行源码、未建新实施工作树、未构建、部署或回放。RGBD08通过仅指提示词交付；L1/A1/A3实施尚未开始，下一会话另登记单个实施feature。本轮编辑前的主文档快照在新目录 `before/docs/`，原修改保留。以下历史记录保留。

# RGBD07 当前交接 — 2026-10-09（初始化修复实施验证完成，完整验收仍blocked）

用户授权按修订方案实施，已完成本轮代码、三种构建、实际入口/分配失败与真实System短序列。入口：[实施报告](../artifacts/rgbd_system_init_fix_impl_20261009/REPORT.md)、[最终核验](../artifacts/rgbd_system_init_fix_impl_20261009/FINAL_VERIFICATION.json)、[源码交付](../artifacts/rgbd_system_init_fix_impl_20261009/source_delivery/README.md)。本轮无正在运行的构建/测试、待回传日志或权限阻塞。

隔离双仓 `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_system_init_fix_20261009`，core96a912876c1602afdd8d9f0c2e851b671ae665b1、outer637915d72eba10361b031e51fa48aa149b724ad6，两仓fix/rgbd-system-init-20261009、共同标签rgbd-v3-c0-init-fixed-20261009-01。父C0仍core0041da6f/outerf13df6cc且clean，新双仓clean。IDE运行源码未改，保留上一轮docs修改，新增本条与RGBD07记录。

修复完整IMU模式隔离、Tracking初值、实际关键帧决策短路及共用重置窗口；两入口矩阵/频率/噪声/float平方校验、局部RAII完整成对发布。各入口key/type、200Hz=0.005s、30fps=30帧、零噪声、原float公式及既有所有权不变。普通host/ARM各232项通过，含两入口实际第二new失败、Calib恰好回收一次、未发布成员；49示例外参各经两入口，不能外推完整原配置或真实六模式跟踪。

ASan/host/ARM各39核心单元冷构建，0旧核心对象复用，host17/ARM15产物。三轮真实TUM均120进/返并自然排空/join，ASan33正常/1reset、host120正常/0reset、ARM34正常/1reset。ASan无非法bool/UB/address错误，但最终LSan16803462B/6612分配、进程-6；最大10,720,840B来自未释放LocalMapping观测缓存。入口ASan230行为断言成立，严格43通过/187相机泄漏失败。跟踪差异尚未归因，不能用Release短序列或CPP退出码0宣称全系统通过或正式性能达标。

板端新package `/userdata/rgbd_system_init_fix_20261009/c0_init_fixed_01`，251产物/配置、241输入、480输出回传SHA核验完成，实际新库maps一致，已无本轮进程/容器。准备失败及回传自动审批超时一次重试成功均归档；没有自动审批遗留拒绝。

RGBD07 blocked表示完整验收失败、实施已完成。原RGBD06性能停止继续有效，不恢复N03/N04/N05/R7/R8。TODO：另批处理System所有权与析构、TUM不同构建结果差异；纯视觉载入惯性地图、单目既有初值、保存重载、C1竞争、L1窗口和活动GBA覆盖保持未关闭。不要通过新增suppression或直接统一delete回避这些问题。以下保留原交接内容。

# RGBD06 System初始化修复方案评审 — 2026-10-09

用户本次要求评审附件建议、更新解决方案并确定修改基线。已交付 [REVISED_SOLUTION.md](../artifacts/rgbd_system_init_fix_plan_20261009/REVISED_SOLUTION.md)，原文和只读来源核验分别为同目录USER_REVIEW.txt、SOURCE_REVIEW.json。RGBD06仍blocked，尚未实施修复、建立工作树或执行新构建/回放。

基线已确定并核验干净：C0 core `0041da6f4cbcce78628cefef43c5cad2abb481df`、outer `f13df6cc58ac16a9dec71844206952f15433916f`，从这两个commit分别建新隔离工作树。它们继承B0并保留实际失败的观测/驱动来源，不包含A1/C1/E2性能候选；C0作为修复父版本不等于合格性能控制。拟建根rgbd_system_init_fix_20261009、分支fix/rgbd-system-init-20261009及新epoch目前均只是方案。

采纳模式隔离、Tracking默认值、两个入口后的统一重置帧窗口，保留合法IMU模式已有mImuPer频率表达式、键名/类型和噪声换算。补充矩阵转换前校验、非负且有限噪声及平方结果校验、两个对象局部RAII后发布；RAII不覆盖exit/abort、发布后的构造失败或整个System最终清理，运行期共享预积分段不新增delete。L1导出状态修正另作观测提交。

只读重算44项仓内IMU外参，全部float32/4×4/有限；最大正交误差1.1912230913563636e-6，现有1e-6辅助检查会拒绝6项，因此方案提出旋转1e-5、齐次末行1e-6的明确输入规则，尚需真实loader及边界测试。本轮命令为git HEAD/status/log/diff/祖先核验、rg/sed源码审查、OpenCV读取与numpy矩阵重算、官方机制核对及文档检查。验证见新目录VERIFICATION.json；未编译、测试运行代码、访问板卡、部署或回放，因为当前任务是方案评审。

下一步按修订方案从已定C0双仓实施正确性修复，完整120输入的System插桩检查与Release性能分别结算。TODO：修复尚未实现验证；新错误、全局泄漏与其他历史质量缺口继续单列。旧源码与归档未改；本次编辑前features/session-handoff快照在新目录before/docs，原文档补充清单仍表示其当时版本。以下历史交接原文保留。

# RGBD06 文档补充 — 2026-10-09

按用户要求补充报告及相关文件中的术语。新增 [TERMS.md](../artifacts/rgbd_next_round_impl_20261008/TERMS.md)，详细解释hint的当前帧去重、完整身份比较、碰撞回退、清零和非拥有语义；补充owner/weak/marker、阶段与候选编号、wall/CPU、锁与观测、输入与统计、后台终结、构建和sanitizer等术语。

本轮核对冻结C0的RetainedPointCache、WeakIdentitySlot、MapPointHandle、Tracking和KeyFrame源码，给REPORT、NEXT_CANDIDATE_DECISION、DIAGNOSIS、MEASUREMENT_SCHEMA、QUALITY_STATUS、PLAN以及2026-10-08实施计划、2026-09-28优化方案添加阅读链接或简短释义。仅更新Markdown与本次文档补充清单，RGBD06仍blocked；原实验数值、源码、二进制、CSV/JSON证据及验收结论不变。

检查为文档差异、原数字与实验表格保留、本地链接及git diff --check，记录在 [UPDATE_MANIFEST.json](../artifacts/rgbd_next_round_impl_20261008/documentation_20261009/UPDATE_MANIFEST.json)。修改前10份文档逐字节备份，主归档内6份原件SHA与原索引相符。原ARCHIVE_MANIFEST/DELIVERY_VERIFICATION保留，当前文档另列校验值。未运行构建、测试套件、部署或回放，因为没有运行代码变动；原校验脚本会重写旧归档，未用它覆盖历史记录。未新增运行证据，原TODO和下一项判别补测保持。以下2026-10-08及更早交接内容保留。

# RGBD06 当前交接 — 2026-10-08（首批实施停止归档）

已读取并执行用户指定SESSION_IMPLEMENTATION_PROMPT。实际交付见 [REPORT](../artifacts/rgbd_next_round_impl_20261008/REPORT.md)、[DIAGNOSIS](../artifacts/rgbd_next_round_impl_20261008/DIAGNOSIS.md)、[QUALITY_STATUS](../artifacts/rgbd_next_round_impl_20261008/QUALITY_STATUS.md)、[NEXT_CANDIDATE_DECISION](../artifacts/rgbd_next_round_impl_20261008/NEXT_CANDIDATE_DECISION.md)。RGBD06验收blocked，诊断交付已完成；RGBD04仍blocked，RGBD05 passing只指计划评审。没有待运行的本轮构建/回放或候选，没有push/merge/替换生产部署。

隔离双仓 `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_optimization_v3_20261008`，分支perf/rgbd-v3-20261008，core0041da6f、outerf13df6cc（先前共同outer观测63b2ffb）。C0=B0+必要观测，epoch rgbd_v3_c0_observed_20261008_01。源码/驱动/依赖已冻结，原B0双仓和原runtime/review.json保持。不要原地更改NEW源码或ART/tools后沿用旧source/build/run；单独新epoch修复再实验。源码差异在新ART/patches/c0_core.patch、c0_outer.patch，source目录为完整输入关联。

host/ARM均39核心TU冷编，15/13冻结产物，各39进程探针与ORB字节一致通过；51工具测试通过。板端新包 /userdata/rgbd_optimization_v3_20261008/c0_observed_01，DSO SHA168180a2f46146827bb880fe831f0d010551ecddc782fe46fcfd701954cc4eaa；旧配置SHA a6736839…保持700/5/3、LM01/Viewer off、loop on、8/8/10、1×。实际加载maps和回传SHA均保存。最终板端无SLAM/master/rosbag，free9201741824B，新/旧源码干净，IDE两文档既有dirty已备份续接。

T1 off/on/L1各120/120，L0 P95=93.556/92.849ms，无反序触发；L1 P95=96.900ms，只是功能归因。T2长L0=2814/2910，P95全段113.751/固定尾121.153ms，缺96（RGBonly21/depthonly46/neither29），前67有ExactTime drop；T3 L1=2802/2910，尾P95=124.898ms。全长已处理帧正常单图，发布/消费一致、自然结束，不抵消missing/90ms失败。正式有效控制P99门未建立，不从旧ACE或非法控制放宽。

A3所采241尾窗行：hint清零CPU0.02541ms（A3总CPU0.1585%）；旧owner clear2.03381ms（12.6874%），不能延迟释放藏成本。A1最大49.715ms wall/2.065ms CPU，GetObservations累计0.733ms，voting KF180使邻居扩展跳过；未采MP/KF isBad与调度仍unknown。成员版本不足以缓存。唯一后续判别项是预定同窗A1投票/KF过滤整段与有限isBad锁采样，未实现。不得自动启动N03/N04/N05/R7/R8或扩大队列/缓存/线程矩阵。

本轮观测缺陷：L1实际2621–2910、241细记录，状态JSON却打印旧145/2766。原始数据/源码未改，语义验收失败；审计和未应用修正补丁在 analysis/L1_WINDOW_STATUS_AUDIT.md / patches/N01_DETAIL_WINDOW_STATUS_FIX_UNAPPLIED.patch。不能把拟议修正写成当前epoch已修复。

Q1旧TUM记账分支夹具确认可能复用旧stamp，真实5057失跟/两图/最终重复仍未修。Q4真实当前System ASan/UBSan冷构建成功，但构造期 Settings::insertKFsWhenLost_ 未初始化bool两次触发UBSan（0/120到达TrackRGBD）；沙箱外同二进制复核确认。core address+undefined/g2o仅address，LSan开启，不新增suppression；自然析构/System LSan未覆盖。全955活动轮完整、loop/GBA1且0→2→3→5→4自然结束，但前台结束时worker已返回；唯一事前补充1–499全返回却35 invalid/两图/GBA0，结束时活动未覆盖，已停止补轮。stop-owner完整trace、MergeLocal/reset/cancel、C1旧TSan、历史LSan3379036B/7205分配和保存重载101/120均不关闭。

后续先独立共同正确性/观测修复、建立新epoch并实际复验，再只做决定文档指定判别项。全部失败、原命令、源码/二进制SHA、进程/maps、退出、消费者、容量和资源保存在新ART；最终校验索引ARCHIVE_MANIFEST.json。不要管理历史PID/容器，不删除旧冻结产物。以下为原交接快照，按原文保留。

# RGBD05 当前交接 — 2026-10-08（下一轮计划评审完成）

交接提示词：按用户准备交给其他session实施的要求，已生成[SESSION_IMPLEMENTATION_PROMPT.md](../artifacts/rgbd_next_round_plan_20261008/SESSION_IMPLEMENTATION_PROMPT.md)。包含项目/双仓来源、已知结果与源码结论、host/ARM实际构建方式、镜像/sysroot/依赖、板端SSH和部署/监督runner、数据配置、旧驱动硬编码需适配、阶段门与交付要求。本次仅只读核对脚本和归档命令并整理文档，未构建、部署、访问板卡或开始下一轮实施；原计划及其冻结核验未修改。

本轮用户要求分析并重整附件计划，已交付[评审与新版实施计划](../artifacts/rgbd_next_round_plan_20261008/IMPLEMENTATION_PLAN.md)。RGBD05的passing仅指计划交付；RGBD04完整输入/90ms目标仍blocked。未修改runtime源码、编译、部署、访问板卡或新跑SLAM；两个只读子任务核对A3/锁/既有字段及ROS/TUM/生命周期。原始建议无改动副本与来源/旧CSV重算在同目录。

主要修正：关键帧/BA详细记录已经存在；A3当前是KF锁内直接遍历，并无每KF快照复制；map-update等待在A1外，identity barrier不由普通A1/A3读者获取；现有Received事件在成功反序列化后的FieldInput，不能称真实出队；ExactTime drop已经接入但覆盖不全。TUM是native/OpenCV=1，失跟和重复轨迹记账分开查。原硬门仅全段P99×1.05、尾P99单列，第1个源输入不豁免。

只读复算bad=missing OR callback>90：B0全段最长115/尾102，ACE全段及尾112（2765–2876，17missing+95超时）。这是诊断补充，不新增硬门。下一批N00–N02只补判别所需观测，Q质量任务拆小；条件触发ROS依赖hook、writer trace、C1插桩和跨帧缓存。下一项只选一个候选；有望联合达标才恢复固定R7/R8。

核对源码：outer823fd074/core B0 59ac3847/ACE61f25aa0仍干净，IDE84e6ac3f保留原docs修改。本轮新增artifacts/rgbd_next_round_plan_20261008/文档与证据，更新features和本交接；旧冻结产物未变。验证为源CSV SHA、确定义重算、20个本地链接/源码行号、Markdown与git diff --check。未跑测试套件，因为本轮没有运行代码改动。TODO：实际板端工具/依赖能力、最小新增观测与质量恢复仍需后续实施验证，不能用旧板端快照替代。

# RGBD04 历史交接 — 2026-10-06（本轮实施与筛选结束，目标未达标）

文档补充：按用户要求新增[本轮修改、测试与后续工作汇总](../artifacts/rgbd_optimization_impl_20261006/IMPLEMENTATION_TEST_SUMMARY.md)，集中说明具体修改、测试范围、八版本回放对照、分析、未关闭问题及后续推进条件。本次仅整理文档，核对43个本地证据链接及候选表48个数值；未新增源码修改、构建、回放或板端操作。RGBD04状态与原始冻结证据保持不变。

用户已授权并已执行修订计划。最终入口：[实施报告](../artifacts/rgbd_optimization_impl_20261006/REPORT.md)、[覆盖清单](../artifacts/rgbd_optimization_impl_20261006/FINAL_TEST_COVERAGE.md)、[源码交付](../artifacts/rgbd_optimization_impl_20261006/source_delivery/README.md)。RGBD04 为 blocked，含义是性能与完整输入目标未达成，不是等待权限。当前没有本轮构建、回放或专项测试待执行。

实际隔离runtime为 `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_optimization_v2_20261006`，外层823fd074、共同core59ac3847，双仓标签rgbd-v2-b0-20261006。六候选均已单独实现/筛选；ACE=A1+C1+E2-entry core61f25aa0位于memory/candidate_sources/combo_ace_01。固定B0不滚动晋升。10份补丁包含共同core/outer、独立候选、D2增量/outer及组合，不包含bag/地图/二进制。

ACE host/ARM均39核心TU全新构建，15/13产物；host39项、ARM原27项及v2、A1/C1实际组合库、E2实际图/真实求解后取消通过。ARM核心SHA e3529baae801dd9afba4d365e421fc4ecb24ccd3edc88acc789c1071e4e8619a，板端combo_ace_01目录13产物最终哈希一致。120短包完整；2910长包2863处理/缺47，P95 106.233166ms、固定尾114.047535ms，尾P99 137.289754ms较B0 132.023297ms恶化。全程已处理帧正常、单图、自然drain/join、测量/实际消费完整，不能抵消源输入缺失。所有输入轮均未发生loop/GBA。

依据R6停止R7/R8三对长包/三对TUM/两对闭环正式矩阵，ACE不晋升。B1/D1/D2未进入组合。C1仅受控实验组合：默认TBB TSan两项未消除，B0亦复现但不算通过；固定输入全进程ORB CPU中位−3.02%，墙时+2.75%，Mat代理峰值+481403 B。E2仅入口已有请求取消，完整P07中途构图取消未做。实际求解取消通过限于受控post-iteration交错及整图提交，不能推广所有调度。

共同TUM返回5057但91 non-OK、两图及轨迹重复，控制资格失败。完整System历史LSan 3,379,036B/7,205分配、保存重载101/120缺19仍未关闭；现有定向通过不替代整机覆盖。正式质量、实际活动GBA自然退出、完整System sanitizer均未通过。下一项只优先A3汇集归因；现有wall−CPU差不能直接视为锁等待，不自行扩大成新的缓存/预算改造。

原runtime outer9c43a6c3/corebe52247f、分支和原review.json均未变，见ORIGINAL_RUNTIME_FINAL_AUDIT.json。板端最终只读快照无本次目录活动可执行进程，余9,258,090,496B。保留全部失败/旧构建，不覆盖原部署或清未知文件。最后E2上传曾自动审批拒绝，经用户原始板卡/实施授权补证，同一未改动作重审获准并执行；现无权限阻塞。历史准备/构建RESULT里的deployed=false等字段是生成时快照，最终状态以本报告、部署/回传/筛选证据为准。冻结源与四个构建驱动不得原地更改。

# RGBD03 历史交接 — 2026-10-06（评审修订完成）

- 状态：`passing`仅指分析与计划交付，未修改runtime源码、构建、部署或运行新SLAM/测试；LM01仍默认off且未验收。
- 最新入口：[评审修订实施计划](../artifacts/rgbd_optimization_plan_20261006/IMPLEMENTATION_PLAN.md)。评审意见逐项取舍、完整执行规则和证据链接集中在该文件；用户评审原文及原计划无修改副本同目录保存。旧IMPLEMENTATION_PLAN已加替代提示。
- 三仓HEAD仍为IDE `84e6ac3f`、outer `9c43a6c3`、core `be52247f`；126项当前来源文件匹配，outer已有review.json未动，core干净。新来源记录 `SOURCE_RECHECK.json`；本轮未重新检查板端，9月28日板端快照不能当今天环境。
- 主要决定：O0原off归档→E1原子请求/同队列锁任务门闩→共同测量/评价/结束协议→固定B0→独立A/B/C/D/E2→组合。候选不滚动提升B0；最终S316固定三对、TUM三对、闭环两对，单项仍一轮筛选。
- 新核对：g2o addEdge返回false可能已经注册；marker跨多函数使用但不能直接断言并发写故障；TUM原有在线/最终双评价，原生API在线与ROS重复/连续发布不同；现有sleep/finished不证明drain或真实join。E1仅修取消通道，不宣称全系统无竞争/泄漏。
- 只读评价脚本：`python3 artifacts/rgbd_optimization_plan_20261006/recheck_evaluation_mask.py --output /tmp/<新的唯一文件名>.json`，输出须不存在。固定5057输入关联1227个GT，原1130对RPE，禁止跨>100ms GT缺口后保留1113对；这是掩码预览，未评价新轨迹精度。源码/链接/文档核验记入新目录VERIFICATION.json及SHA256SUMS。
- 改动范围：新20261006计划、来源/评价预览与脚本、评审/旧计划副本；旧计划替代提示及对应哈希索引；features与本交接。历史失败保留，不提交、切分支或清理地图。
- 下一步：用户进入实施阶段时按P00/P01保存三仓工作后修E1。TODO：ARM候选收益、有效端到端、诊断开销、实际drain/join及现有System泄漏仍待实施验证；workspace容量值是工程预算，不是实测保证。

# RGBD03 历史交接 — 2026-09-28（只读分析完成）

- 活动项：RGBD03，`passing`仅指分析与计划交付；没有实施/部署候选，没有新的实时性或质量通过声明。LM01仍blocked/default off。
- 交付：优先阅读面向新读者的[单篇分析与实施方案](../artifacts/rgbd_optimization_plan_20260928/OPTIMIZATION_PROPOSAL.md)，已整合背景、实测证据、修改理由、并发约束、分阶段验证及Git计划；另有[技术主报告](../artifacts/rgbd_optimization_plan_20260928/REPORT.md)、[详细实施计划](../artifacts/rgbd_optimization_plan_20260928/IMPLEMENTATION_PLAN.md)、前后端源码审计、原始时序重算JSON/脚本、来源审计JSON/脚本和板端只读记录。
- 三仓起点：IDE `84e6ac3f`；runtime outer `9c43a6c3`、core `be52247f`。runtime为 `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_memory_lifecycle`；core独立Git。outer已有未跟踪 `memory/tools/mem14_board/review.json` 保留不动（SHA256 `a4589af28b4afa54ad734ba13ed2a6340d83c91f3232aa789979fba5d9e8d6e4`），没有提交/切分支。
- 已执行：指定10份证据与F27阅读，三个独立只读审计工作流；源码/构建manifest哈希核验；运行本目录 `audit_sources.py` 与 `reanalyze_timing.py --root /home/ywl/Project/ORB_SLAM3_DBoW3 --output artifacts/rgbd_optimization_plan_20260928`；SSH只读检查环境/既有文件哈希/进程；文档链接、统计与源码不变核对。
- 单篇整理补充：按用户要求新增 `OPTIMIZATION_PROPOSAL.md`，核对31个本地链接（含14个源码行号）和35个前台表格数值；结果记入 `VERIFICATION.json` 的 `consolidated_proposal`，更新 `SHA256SUMS`。此次仅整理文档与索引，没有新增实验或修改runtime。
- 核验结果：冻结perf20源/头81/81、二进制3/3；当前C6来源126/126、ARM源/头98/98、二进制4/4匹配。当前与perf20共同清单仅62/81相同。板端perf20/C6-off核心哈希匹配；无运行SLAM进程，不能证明当前实际加载版本。可用空间约9.01GiB仅为检查时快照。
- 新发现：完整TrackRGBD mean83.305ms、Tracking内部44.016ms；最慢5%主要贵在KF更新/MP汇集/搜索，pose反而较轻；同帧local_update差额mean1.852ms，reference强引用vector复制待细分。170/162是BA执行与轮尾flag，不是精确中止原因；BA MP列未赋值；旧header_age无有效端到端意义。811帧>90ms，严格连续源序列最长41帧。
- 推荐下一步：进入实施时先保存三仓dirty/untracked并建立隔离控制，补ORB/后台/端到端计时；A1观测连续化、B1兼容单锁几何、C1同层blur→descriptor各自验证，D精确medoid与E同步/早停分开提交。不要直接开启LM01或移植GF组合。
- 未执行：生产源码修改、任何编译/SLAM回放/测试套件/部署/地图保存/清理。原因：用户限定本轮分析。新的ARM收益、真实取消延迟、完整端到端/队列占用、全System LSan及完整真值仍TODO；历史失败保留。
- 本轮改动仅IDE分析产物、docs/features.md与本交接；生成目录受既有ignore规则影响，后续若提交需显式选择小型报告/脚本/JSON，不能加入bag/地图/build或整个memory目录。

# GF04 historical handoff — 2026-09-28 (completed evaluation)

User requested official ORB-SLAM3 as baseline plus the previously delivered acceleration;
one pass per configuration persists. GF04 is passing for evaluation delivery, not production.
No subagents or board access. Main checkpoints95b26786/c1c7d760/679c9fc2; final report commit
is the Git history of artifacts/orb_slam3_controls_20260928/REPORT.md.

- Isolated root `/home/ywl/Project/RDK_X5_orb_slam/orb_slam3_controls_20260928`.
  Official4452a3c4; delivered core4d6df6a2 /outer18ba684, NOT MEM18/perf20/LM01/XFeat.
  Delivered69 src/include files match the archived source. Original checkouts untouched;
  official untracked build_log.txt preserved. Custom native GCC9.4/OpenCV4.2/C++14 builds.
- First unpatched loop-off smokes crashed in BOTH versions. NewDetectCommonRegions returns
  before initializing mpCurrentKF; Run then copies/dereferences it. Both isolated sources
  get the same one-line mbActiveLC guard, plus diagnostic friend in System.h. No Tracking,
  LocalMapping, matcher or optimizer change. Never call this an untouched upstream binary.
- 8 main rounds: 4 modes(official800/delivered800/delivered700/frozen GF03 baseline bridge)
  ×EuRoC2912/TUM5057 =31876 frames; complete input/finite fields/exit0. 800/8 matched;
  700/5 delivery configuration separate; delivery3workers/RGBD L4. Viewer/loop/mapsave off.
  Official TUM has92 nonOK(indices1715–1806), map split. Remaining7 loss-free.
- EuRoC P95 official/del800/del700/GF=24.564/21.778/20.148/36.728ms; all ATE≈0.033–0.034m.
  TUM P95=17.431/13.764/11.873/25.079ms. Official split invalidates a whole-map quality/late
  workload comparison; clean prefix[0,1715) and common-start metrics are explicitly stored.
- Failure-triggered3 RGB-D loop-on rounds reuse frozen core libraries, separate driver
  differing only by shutdown GBA wait. All5057 each, zero nonOK, one map, one detected loop.
  P95 official/del800/del700=17.062/14.148/12.371ms. ALL exit7:60s combined backend/GBA wait
  times out. Do not equate all input rows with clean shutdown. Exact final thread states
  unrecorded. QUIET suppresses NORMAL GBA logs; missing log messages do NOT mean no GBA.
  ATE0.47729/0.50370/0.43030m; matched800 ATE/RPE degrade≈5.5/5.4/5.2%, outside5% gate.
- All11 formal attempts47047 input rows;11 validated smokes1650 rows;3 accuracy tests pass.
  Main589 frozen files unchanged even after fullsystem driver builds/runs.184 raw artifacts
  compressed+byte-checked,6.6MiB evidence. Raw runs retained. No Atlas maps generated.
- GF bridge P95 change vs GF03:+0.44% EuRoC,-5.48% TUM; old8 combinations are historical
  references, not new matched A/B. Never rescale old runs to remove drift. TUM GT1227/5057,
  maxgap129.865s. One global Sim3 for mono, SE3 for RGBD; no per-map alignment.
- Commands: harness/run.py smoke/formal, verify.py, analyze.py, tables.py; fullsystem.py
  build/smoke/formal, verify_fullsystem.py, analyze_fullsystem.py; all_results.py/archive.py.
  Sessions61367(main)/27070(loop-on) finished. No active replay remains.
- TODO not run: RDK-X5/ROS/network callback gate, repeats, full-System sanitizers, full-GT
  precision, clean GBA shutdown, map save/reload or longer scenes. Existing deployments and
  production source/binaries unchanged. Next work should diagnose background exit and
  validate board realtime before declaring a new production optimization accepted.

# GF03 historical handoff — 2026-09-28 (completed)

GF03 evaluation delivery is complete (`passing` for the research workflow), not an acceleration or production acceptance. User explicitly reduced the matrix to one pass per mode/dataset. No subagents were used.

- Independent root `/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_modules_20260928`; source branch `repro/module-factorial-20260928`, clean source commit `1ad2246b2aac4357a2990c80ced84a41ef6d0d64`; parent GF02 `33e5064`, official base `2a60498`. Main design/build checkpoints `39e5ebaf`, `29c41fa3`; final evidence commit is the Git history of the GF03 REPORT.
- Modes baseline/F/H/G/FH/FG/HG/FHG. F existing active matching + skip-local + RGB-D KF rules, 400 scalar constraints, ORB setting 800/8 levels; optional frame-match information prior and cross-frame precompute off. H legacy MIH+OTS with inherited synchronization, not GF02 bounded/all-query policy. G fixed30/candidate60 native SLAM++ selection; dynamic anticipation/budget and unfinished stereo SLAM++ variant off. Selection uses 2-D information; downstream RGB-D BA keeps depth constraints.
- All eight CPU targets built. 16 short functional runs / 7600 frames were loss-free; these overlapped builds and are not timing evidence. Three accuracy-tool tests passed. Full single-pass matrices completed 16 runs / 63,752 frames; all input indices/timestamps/finite fields and immutable-file checks pass, 15 runs stayed OK after initialization.
- EuRoC all 8 loss-free. Baseline/G/FHG API P95 36.569/35.317/37.703 ms; ATE 0.0336/0.0338/0.2913 m. TUM baseline/G API P95 26.534/25.538 ms, ATE 0.7464/0.9454 m. No enhanced candidate passes all screening gates on either dataset.
- TUM FHG lost at index 4126 and remained LOST through 5056 (931 frames). First loss reports motion and reference tracking failure, after prior inliers 41/38/30/25. Subsequent empty relocalization candidates have a shared test limitation: DISABLE_LOOP_CLOSURE skips normal KeyFrameDatabase insertion. Initial loss and absent recovery are separated in failure_diagnostic.json; no repeat or root-cause proof claimed.
- IMPORTANT correction to preflight handoff: the final incremental build recompiled only five replay drivers. FG/HG/FHG retain historical driver d85f25dc without input_lateness_ms; other modes use 2a5138f5. Source snapshots differ only in pre-call lateness calculation and CSV column; exact eight-way driver parity is NOT established. All primary tracking/trajectory/module measurements exist. Keep missing lateness as unmeasured, preserve frozen binaries and original metadata; see diagnostic_gaps.json/driver_diagnostic_delta.patch. Do not claim a file hash audit proves every binary derives from latest replay.cc.
- Source/config/input-list/runtime hashes and 24 new binary/dependency hashes remain fixed; GF01/GF02 16 prior binary hashes unchanged. User-reduction cancellation retained under runs/cancelled_on_user_repeat_reduction; completed initial baseline reused. No active batch remains; session 37229 completed exit0.
- Evidence in artifacts/gf_orb_slam2_modules_20260928: REPORT, CODE_AUDIT, DETAILS, JSON/CSV summaries, PNG/PDF figures, patches, diagnostic ledgers, source/build manifests and raw SHA256 indices. Raw runs remain at the independent root. TUM GT covers only 1227/5057 poses (24.3%), with 129.865s max gap; failed FHG has only 590 associated poses and is excluded from accuracy acceptance.
- Commands run: all target builds; run_matrix_once.py on both tags; analyze.py, factorial_report.py, detail_tables.py, diagnose_euroc.py, audit_freeze.py, archive_evidence.py. Do not rerun freeze.py or alter old tags. README documents local prerequisites and workflow.
- No production ORB3/MEM18/perf20 edits, board access, map generation or board cleanup. No full-System sanitizer certification, dynamic Good Graph, functioning relocalization, loop/save/reload, full EuRoC suite, NewCollege or ARM/ROS acceptance. Host native API timing cannot establish board callback performance. LM01 remains blocked/default off.
- Follow-up only if requested: diagnose first-loss/accuracy interactions with fresh diagnostic-parity builds, preserve KF database population independently of loop correction, and profile selection/postprocessing before proposing a production port. No blanket three-repeat matrix is needed by default.

---

# GF02 current handoff — 2026-09-28

GF02 evaluation delivery is complete (`passing` for the research workflow). The repaired hashing candidate is **not accepted for acceleration or production**. LM01 remains blocked/default off.

- Isolated root: `/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_paper_20260928`; source worktree `upstream/`, branch `repro/paper-buckets-20260928`, commit `33e506481bf4daf8bf37b1f4377950c28a8c54a7`. Git object storage is shared with GF01; preserve that original checkout. Full official-base and GF01-delta patches are archived.
- IDE started clean at `4a99b5e7`; implementation/harness checkpoint `ee1c6bec`; final evidence commit is the history of the GF02 REPORT. GF01 source/build hashes are unchanged.
- Four modes: baseline, legacy mih_ots with common synchronization fixes, paper_ots bounded/recent/migrating buckets, paper_all also querying every descriptor. No feature/matcher/tracking threshold reduction.
- Release and focused ASan/UBSan/LSan/TSan index fixtures pass; accuracy helper 3 tests pass. Four 400-frame smoke runs pass (no hash queries in that short interval).
- Full `euroc_paper` and `tum_paper`, 12 rounds each, completed 95,628 frames. `harness/audit_freeze.py` passes all record and freeze checks. 23/24 loss-free; TUM first bucket-only run lost at frame 28 (24 inliers, early-window threshold 25), before any hash query. Root cause beyond this trigger is unresolved; later two bucket-only rounds succeeded.
- EuRoC baseline / old hash / bounded / bounded+all-descriptor API mean: 26.603 / 25.737 / 30.724 / 31.871 ms; P95: 35.609 / 38.627 / 63.127 / 64.263 ms. ATE about 3.3 cm throughout.
- TUM corresponding mean: 19.824 / 20.790 / 23.392 / 26.067 ms; P95: 27.550 / 32.291 / 52.368 / 59.477 ms. Bucket-only numbers include a failed run, so cannot establish gains. GT covers 24.3%; successful-only accuracy is explicitly annotated.
- Internal pose-ready timing improves for old hashing, but full API tails do not. Current public ROS wrappers publish after Track returns. Core pose-ready timing is not ROS publication latency.
- Evidence: `artifacts/gf_orb_slam2_paper_20260928/REPORT.md`, raw indices, machine summaries, PNG/PDF figures, sources and build manifests, fixture logs, startup failure diagnostic. No maps generated or board files changed.
- Remaining research: independently profile/update index synchronization and OTS actual bucket membership/information semantics; do not interpret this partial semantic repair as exact reconstruction of the paper. No full-System sanitizer, board/ROS, loop/save/reload or long-NewCollege acceptance.
- Do not rerun or alter existing frozen tags; use a new experiment tag for any later candidate.

---

## Historical handoffs retained for provenance

The current instructions are above. The following earlier state is preserved as historical evidence.

# GF01 current handoff — 2026-09-28

GF01 host reproduction/evidence delivery is `passing`; stable acceleration and
full-paper reproduction are **not** demonstrated. Production LM01 stays blocked
and default off after its completed C6 matrix; do not continue the stale C1
instructions from earlier handoffs.

Independent directory:
`/home/ywl/Project/RDK_X5_orb_slam/gf_orb_slam2_repro_20260928`.
Official catkin base: `2a60498cb889965bd5b4ab05c918ab28b433e882`;
local compatibility/isolated-hashing source commit: `39e794a`.
IDE initial checkpoint: `9fa86c71`; final evidence commit is the history of
`artifacts/gf_orb_slam2_repro_20260928/REPORT.md`.

- Four CPU modes built using `cmake -S harness -B build`, then
  `cmake --build build --target replay_baseline replay_mih32 replay_mih_ots replay_mih_ots_default -j3`.
- Completed sequential interleaved matrices: `euroc_formal` (9),
  `euroc_default` (6), `tum_bgr_formal` (9). All 24 inputs complete,
  89,193 frames total. One EuRoC narrow-OTS run lost its last 68 frames;
  all other formal runs remained OK after initialization.
- EuRoC first batch baseline/OTS mean 27.52/26.34 ms, P95 36.10/38.20 ms;
  narrow OTS fails the three-run tracking gate. Wider parameters are slower.
  TUM baseline/8-table/24-table means 18.70/20.16/20.71 ms,
  P95 26.42/31.13/32.46 ms. Native API timing is not board ROS latency.
- TUM input adaptation: OpenCV PNG is BGR, so `Camera.RGB=0`; depth factor
  5000 for uint16 PNG. Initial `tum_formal` with RGB interpretation is
  explicitly invalidated: 3 completed + 1 interrupted runs retained.
- Ran `python3 -m unittest discover -s harness -p 'test_*.py' -v` (3 pass),
  `python3 harness/prepare_inputs.py --check-existing`,
  `python3 harness/analyze.py runs/<matrix>`, export/plot/archive tools,
  and `bash harness/run_bucket_probe.sh` after timing finished.
  Probe confirms the current upstream deque stores 25 entries despite
  nominal MAX_BUCKET_SIZE=10; query limit 10 still returns 10.
- Frozen binary/config/driver hashes audited against current or preserved
  historical files. Source checkout is clean. Raw frames, poses and logs
  remain local with per-file SHA256 indices in the IDE archive.
- EuRoC uses global Sim(3); RGB-D uses SE(3). TUM GT covers only 1227/5057
  poses with a 129.865 s gap; metre-scale ATE is not full-sequence accuracy
  acceptance. Read the report before using any aggregate.

Touched: only GF reproduction source checkout, independent harness/configs,
`artifacts/gf_orb_slam2_repro_20260928/`, and these evaluation docs.
No production ORB-SLAM3 edit, deployment, board process or board-file cleanup.
No map files generated.

Not run: 11-sequence/10-repeat EuRoC or NewCollege 43-minute loop SLAM;
combined Good Feature Matching/Good Graph; ARM deployment; GF sanitizer /
concurrency certification. Current published source is not a confirmed 2019
paper revision. Hardware, libraries, timing boundary and bucket semantics
limit claims of exact paper reproduction.

TODO for a separate follow-up, if authorized: identify the paper revision and
resolve bounded-bucket/refresh semantics before attributing a performance gap
to the algorithm; profile query/maintenance/postprocessing costs separately.
For production LM01, snapshot/commit cost and nonbinding MP limits remain the
measured issues. Do not port GF solely on the paper's average-latency figure.

Historical handoffs follow.

# Session Handoff

## Summary

- Active feature: BR01; final state: `passing` for branch/source/archive separation.
- Scope: organize existing changes into the requested branches and archive
  generated evidence. See `docs/branch-layout.md` for exact ownership and bases.
- The original branch tips and frozen board production binaries are preserved.
- Next recommended step: Continue F23 tracking-quality work on `feature/xfeat-f23`; the EuRoC/full-lawn gate remains blocked.

## Validation and evidence

- Pre-change snapshots: `backups/` and `backup_manifest.json` in the local
  `20260907_git_reorganization` archive.
- Archive move mapping and fingerprints: `artifact_moves.json` and
  `artifact_manifest.json`; all 7,626 records verified after relocation.
- Descriptor, syntax, source-preservation, ancestry, ignore and status checks:
  `validation.json` in the same archive. Descriptor checks passed; all 9 shell
  scripts and 10 Python modules passed syntax checks. Build-path defaults and
  explicit overrides resolved to the expected source/workspace.
- No host/ARM full build, board replay, new ATE/RPE gate or deployment was run
  during this organization. C++ logic is inherited or preserved byte-for-byte;
  build/data-root routing was adjusted and checked separately.

## Files and remaining limitations

- Branch routing, feature list, progress, validation and handoff documentation;
  ignore rules; branch-owned tests or runtime scripts/core fixes.
- The outer ROS/build repository has its own same-named runtime branch.
- TODO: F23 full-lawn tracking and the required EuRoC gate remain blocked.
- TODO: archived board evidence needs to be supplied separately in a fresh clone.
- The local runtime core includes changes absent from the frozen production
  library. A clean Git branch does not mean those changes are deployed.

## Addendum 2026-06-06

- Follow-up scope: make `Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc` accept
  configurable stereo image topics so the node can consume bags that publish
  `/camera/infra1/image_raw` and `/camera/infra2/image_raw` without rewriting
  the bag itself.
- Commands run:
  `sed -n '700,780p' Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`,
  `git diff -- Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`,
  two one-off `python3` assertions for topic-param coverage before/after the
  patch, and `./build_ros.sh`.
- Result: `./build_ros.sh` passed and relinked `Stereo` successfully after
  adding private ROS params `~left_image_topic` and `~right_image_topic`,
  keeping the existing `image_rect_raw` defaults while allowing runtime
  override to `image_raw`.
- Touched files:
  `Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc`,
  `Examples_old/ROS/ORB_SLAM3/S316_stereo.yaml`,
  `docs/session-handoff.md`.
- TODO: runtime playback with the original bag is still unverified in this
  session; use `_left_image_topic:=/camera/infra1/image_raw` and
  `_right_image_topic:=/camera/infra2/image_raw` during the next replay test.

## Addendum 2026-07-22

- Follow-up scope: add a board-side debug collection helper for the RDK X5
  catkin deployment found at `/userdata/orb_slam`, so `orb_slam3_ros
  kitti_stereo.launch` sessions can be captured and brought back to the laptop
  for Codex analysis.
- Commands run:
  `sed -n '1,260p' docs/features.md`,
  `sed -n '1,240p' docs/validation.md`,
  `sed -n '1,220p' docs/repo-map.md`,
  `git status --short`,
  `chmod +x collect_slam_debug.sh`,
  `bash -n collect_slam_debug.sh`, and
  `./collect_slam_debug.sh --help`.
  Board-side follow-up commands copied the script to
  `/userdata/orb_slam/collect_slam_debug.sh`, ran `bash -n`, and ran a short
  `timeout 8s ... attach --no-bag --no-images --no-grid --no-future-debug`
  smoke test writing to `/tmp/codex_slam_debug_smoke`.
- Result: added `collect_slam_debug.sh` with `attach` and `launch-slam` modes,
  per-run output under `/userdata/orb_slam/debug_runs/<run_id>/`, optional
  image/grid/future-debug topic recording, ROS state snapshots, topic hz logs,
  selected message samples, resource logs, `dmesg` before/after, and ROS latest
  log copying. `bash -n` passed locally and on the board, `--help` printed
  expected usage, and the short board-side smoke run produced metadata,
  ROS/topic snapshots, and `dmesg.before.txt`/`dmesg.after.txt` before timeout
  stopped the collector.
- Touched files:
  `collect_slam_debug.sh`, `docs/validation.md`, and
  `docs/session-handoff.md`.
- TODO: full board-side runtime execution with active camera image topics and
  `orb_slam3_ros kitti_stereo.launch` is still unverified; the smoke test used
  `--no-bag --no-images` and only proved collection/setup/cleanup mechanics.

## Addendum 2026-07-30 — F15 XFeat / RDK X5

- Final state: `passing` for the requested local reproduction and BPU
  feasibility assessment. PTQ, `.bin` generation, and board runtime remain a
  clearly separated follow-up rather than being claimed from ONNX inspection.
- Commands run included the governance initialization reads and `git status`,
  official source clone, dependency inspection/install into
  `/tmp/xfeat-rdk-x5-pydeps`, official CPU smoke test, fixed-shape ONNX export,
  ONNX Runtime parity test, split-pipeline test, calibration helper smoke test,
  Python syntax compilation, graph/operator inspection, local Docker/toolchain
  checks, and read-only SSH probing of `root@192.168.1.10`.
- Passing evidence:
  `xfeat_rdk_x5/artifacts/local_smoke.json`,
  `xfeat_rdk_x5/artifacts/onnx_validation.json`, and
  `xfeat_rdk_x5/artifacts/onnx_pipeline_smoke.json`.
- Generated evidence: the gitignored 2.6 MiB ONNX model at
  `xfeat_rdk_x5/artifacts/xfeat_backbone_480x640_opset11.onnx`, SHA-256
  `479da17c7d8ab12cfb21125b5a85e617f9aa849ea606fa2bcd2bb085c7c1ca26`.
- Touched files/directories:
  `Thirdparty/accelerated_features/`, `xfeat_rdk_x5/`,
  `docs/xfeat-rdk-x5-research.md`, `docs/repo-map.md`, `docs/features.md`,
  `docs/validation.md`, `docs/progress.md`, and `docs/session-handoff.md`.
- Commands not run: `./build.sh`, `./build_ros.sh`, and ROS runtime commands
  were outside F15's isolated evaluation lane. `hb_mapper checker` and
  `hb_mapper makertbin` could not run because `hb_mapper` is absent and the
  current user cannot access the Docker daemon. Board inference could not run
  because SSH to `192.168.1.10` timed out during banner exchange.
- TODO: acquire about 100 representative camera images, confirm the candidate
  YAML against the exact OpenExplorer release paired with the board runtime,
  run checker/PTQ/model verifier, deploy `.bin`, and measure BPU-only plus
  end-to-end latency and INT8 feature/matching quality.
- TODO: do not connect XFeat descriptors directly to the current ORB-SLAM3
  front end. The existing tree assumes 32-byte binary ORB descriptors,
  Hamming matching, and an ORB-specific DBoW3 vocabulary; a separate design
  and validation feature is required for that integration.

## Addendum 2026-07-30 — F16 XFeat bag evaluation

- Final state: `passing` for the offline feature/matching comparison on
  `2026-06-05-09-59-04.bag`.
- Commands run included `rosbag info`, direct ROS bag metadata/image reads,
  Python syntax compilation, a three-anchor smoke test, two full evaluator
  runs, JSON/CSV inspection, and visual inspection of XFeat/ORB stereo and
  temporal inliers at representative frames including tracking-failure frame
  521.
- The final full run sampled 130 stereo pairs and 65 adjacent temporal pairs.
  Both methods achieved a 100% robust-pair rate. XFeat had substantially
  broader spatial coverage, but fewer strict stereo and temporal geometry
  inliers and 13.40x OpenCV ORB's host CPU extraction latency.
- Correlation with `/robot_pose_tracking_ok` found 7 sampled stereo times in
  the false state. In those samples XFeat averaged 364.3 strict stereo inliers
  versus ORB's 343.0, but XFeat remained weaker on temporal inliers (339.7
  versus 504.7).
- Touched files:
  `xfeat_rdk_x5/evaluate_rosbag.py`, `xfeat_rdk_x5/README.md`,
  `docs/xfeat-bag-evaluation.md`, `docs/features.md`, `docs/validation.md`,
  `docs/progress.md`, and `docs/session-handoff.md`.
- Evidence:
  `xfeat_rdk_x5/artifacts/bag_2026-06-05-09-59-04/summary.json`,
  `per_pair_metrics.csv`, and the `frame_*.png` visualizations.
- Commands not run: ORB-SLAM3 was not rebuilt or launched because F16 is an
  isolated evaluation-lane script and does not modify the SLAM implementation.
  `hb_mapper` and RDK X5 board inference remain unavailable as recorded in F15.
- TODO: export and calibrate a native `640x544` BPU model before using this
  bag for an end-to-end latency comparison; the existing F15 ONNX artifact is
  fixed at `640x480`.
- TODO: a trajectory-level comparison requires a deliberately designed hybrid
  integration. This bag has no external ground truth, so ATE/RPE cannot be
  computed from the current evidence alone.

## Addendum 2026-07-30 — F17 Git storage cleanup

- Final state: `passing` for safe cleanup. `.git` decreased from about 87 GiB
  to 20 GiB, and `/home` ended at 159 GiB used with 124 GiB available. The
  remaining 19.07 GiB is recent unreachable data protected by Git's default
  recovery grace period; confirmed temporary-pack garbage is zero.
- Large-object cause: normal heads/remotes contained 0.467 GiB of blobs, while
  six `refs/codex` worktree snapshots contained 30.405 GiB. The largest blobs
  were rosbag files between roughly 2 and 5.6 GB each.
- Those six stale snapshot refs were removed. Codex later created one new
  current-session checkpoint with about 483 MiB of blobs and no ignored bags;
  most of its objects are shared with normal branch history.
- Main commands run:
  `git count-objects -vH`,
  `git rev-list --objects --branches --remotes`,
  `git for-each-ref ... refs/codex`,
  `git fsck --connectivity-only --no-progress`,
  six exact `git update-ref -d refs/codex/...` commands,
  `git gc`,
  removal of the five exact `tmp_pack_*` paths still reported as garbage,
  `git fsck --full --no-progress`,
  representative `git check-ignore -v` checks,
  `du -sh .git bags`, `df -h /home`, and `git status --short --branch`.
- Validation: both fsck passes exited 0. The final full pass reported dangling
  objects only, not missing or corrupt objects. `master` and `origin/master`
  stayed at `7f22eb93ef034449ef11d96440eee1b8bdae58d8`; the other local and remote
  branch refs were also unchanged. The index contained no staged changes.
- Ignore coverage added in `.gitignore`: `*.bag`, `/bags/`, `*.log`, `/logs/`,
  `/.ros_runtime/`, root trajectory outputs, `/.cache/`, `/.codex`, generated
  reports, `compile_commands.json`, `*.bak`, ROS-generated Python/message
  bindings, temporary dependency checkouts, `Thirdparty/DBoW3.zip`, and
  `Vocabulary/*.dbow3`.

## Addendum 2026-07-31 — F19 complete board frontend benchmark

- Final state: `passing`. The native-height INT8 model, full XFeat CPU/BPU
  frontend, ROS LAN transport, full bag replay, and aggregate comparison all
  completed successfully.
- Commands run included governance initialization reads/status, native
  `640x544` ONNX export and parity validation, 100-frame calibration
  extraction, OpenExplore `hb_mapper checker`/`makertbin`, SHA checks, `scp`
  deployment, board CMake build, two three-second ROS replay smoke tests, ROS
  node/subscription inspection, the complete 65.95-second replay, CSV transfer,
  summary generation, shell syntax checks, and Python syntax compilation.
- Runtime coverage: host ROS Noetic master at `192.168.1.3`, board ROS Noetic
  node at `192.168.1.10`, full 644-frame replay with the expected original
  sample indices, 130 stereo plus 65 temporal pairs per method, and 390 rows.
- Main result: XFeat median extraction 60.00 ms/image versus ORB 66.34 ms;
  mean extraction 64.85 versus 68.67 ms; mean stereo end-to-end 185.63 versus
  161.98 ms. XFeat retained substantially wider spatial coverage but returned
  fewer strict geometry inliers. Both methods passed the robust-pair rule on
  every sample.
- Touched files: `xfeat_rdk_x5/export_onnx.py`, native-height mapper config and
  evidence, `xfeat_rdk_x5/rdk_x5/`, `host_ros_lan.sh`,
  `deploy_board_benchmark.sh`, `summarize_board_benchmark.py`,
  `xfeat_rdk_x5/README.md`, `docs/xfeat-rdk-x5-board-benchmark.md`,
  `docs/xfeat-rdk-x5-research.md`, `docs/features.md`, `docs/validation.md`,
  `docs/progress.md`, and this handoff.
- Evidence:
  `xfeat_rdk_x5/artifacts/board_ros_benchmark_544x640/raw.csv` (SHA-256
  `25603920f73b2c408127eb661129c8ae772e3ad2be64d403812ce9877a03c5f1`),
  its `summary.json`, native-height ONNX validation/calibration manifests, and
  `docs/xfeat-rdk-x5-board-benchmark.md`.
- Commands not run: `./build.sh` and `./build_ros.sh` were not needed because
  F19 is isolated under the evaluation lane and does not modify ORB-SLAM3.
  The board benchmark was built natively rather than cross-compiled.
- TODO: trajectory-level XFeat versus ORB is still unverified. The current
  SLAM frontend cannot consume 64D float descriptors or use them with the ORB
  DBoW3 vocabulary, and this bag has no independent ground truth for ATE/RPE.
- Next recommended step: retain ORB temporal tracking/place recognition and
  prototype XFeat only as an auxiliary stereo/recovery source; then measure
  tracking-loss duration and map continuity on this bag before attempting a
  complete frontend replacement.

## Addendum 2026-07-31 — F20 parallel XFeat stereo frontend

- Final state: `passing`. A real BPU1 inference failed with DNN error
  `-6000001`; device/sysfs inspection confirms this board exposes only BPU0.
- Implemented two independent XFeat instances and tensor buffers, two stereo
  workers, one BPU0-only mutex, OpenCV CPU parallel loops, two independent
  parallel ORB instances, BPU queue timing, and stereo extraction wall timing.
- Board CMake build passed. Two-anchor CPU=1/4/8 replays selected eight workers.
  The full replay generated 390 expected rows with frames 0–641 represented.
- XFeat stereo extraction wall time is 57.78 ms versus parallel ORB 74.48 ms.
  XFeat stereo end-to-end is 125.46 ms versus 99.38 ms; serial-to-parallel
  XFeat end-to-end improvement is 32.4%. Temporal end-to-end is 153.22 versus
  169.40 ms.
- A field-by-field comparison of all 390 records against F19 found zero quality
  mismatches. The final CSV SHA-256 is
  `caba91000f6b3fd8a06fe3cc8f86ad55ea9a2c4275ed9e7e4033b36c48346359`.
- Touched files: `xfeat_rdk_x5/rdk_x5/xfeat_frontend.{hpp,cpp}`,
  `ros_benchmark.cpp`, `run_ros_benchmark.sh`,
  `summarize_board_benchmark.py`, XFeat README, F20 evidence, feature,
  validation, progress, report and handoff documents.
- Commands not run: top-level `./build.sh` and `./build_ros.sh` remain outside
  this isolated evaluation feature. The board-specific target was compiled and
  exercised directly on RDK X5.
- TODO: global 1024x1024 float matching and RANSAC now dominate stereo latency.
  Add rectified row/disparity candidate gating before considering batch=2 or
  image concatenation; spatial concatenation remains unsuitable because it
  introduces a false convolution boundary and does not create another BPU.
- Recovery event: ignored `bags/` files unexpectedly disappeared during Git
  maintenance. `git restore --source=61fa312b... --worktree -- bags` restored
  them from the fully validated dangling tree. A `diff` between tree entries
  and filesystem filenames/byte sizes passed for all 15 files; `bags/` is
  again 28 GiB and is reported as ignored. The exact trigger remains unknown;
  ordinary `git gc` is not expected to remove untracked worktree files.
- Commands not run: `git gc --prune=now` was rejected because it would
  irreversibly delete every unreachable local object, not only known rosbag
  snapshots. `./build.sh`, `./build_ros.sh`, and ROS runtime commands were not
  run because F17 changed repository hygiene/docs only.
- Touched files: `.gitignore`, `docs/features.md`, `docs/progress.md`, and
  `docs/session-handoff.md`. Git-internal snapshot refs and garbage objects
  were removed. Existing source/config changes were preserved.
- TODO: after the default recovery grace period, run ordinary `git gc` again
  to reclaim the remaining unreachable objects. If immediate reclamation is
  required, obtain explicit approval for the irreversible scope before using
  `git gc --prune=now`.

## Addendum 2026-08-03 — F21 XFeat* semi-dense Fine retest

- Active feature/state: F21 remains `active`; F22–F24 were not started.
- Implemented official-contract single/dual-scale host evaluation, selectable
  `sparse|semi_dense_single|semi_dense_dual` modes, target-scale coordinate
  refinement, M256/M384/M512 fixed Fine export/PTQ, board single-scale
  semi-dense decoding, online BPU Fine execution, and 9x9 patch-NCC reporting.
- Host parity: official single-scale refinement maximum coordinate difference
  was `3.05e-5 px`; dual-scale was `1.53e-5 px`.
- Host full-bag M384: Fine raised mean NCC-qualified inliers from `251.98` to
  `295.42`, inlier ratio from `0.659` to `0.811`, and mean patch-NCC median
  from `0.664` to `0.813`. M256 failed the quality target; M512 was slower and
  was not promoted over the quality-sufficient M384 candidate.
- Board full-bag M384: exactly 520 CSV rows, 130 stereo frames per method,
  65 temporal pairs per control method, and source frame range 0–641. Fine
  latency was `54.887 ms` mean / `59.367 ms` P90. It produced `376.50` mean
  strict inliers, `289.42` mean NCC-qualified inliers, `0.529` photometric
  grid coverage, and no fragile frame (minimum strict/photometric `330/176`).
  Against coarse, Fine added `41.12` NCC-qualified inliers (`+16.6%`).
- Same-process OpenCV ORB control: `88.699/101.427 ms` mean/P90, `286.28`
  photometric inliers, and `0.383` photometric coverage. It is not the
  production ORB-SLAM3 baseline and must not be used to mark F21 passing.
- Main commands included Python syntax checks; official parity and all-anchor
  evaluations; OpenExplore conversion/board DNN checks; board CMake build
  with `-j1`; ROS LAN node launch; full `rosbag play --delay=1`; `wc`, method
  count, frame-range and SHA checks; CSV download; summary generation; and
  `git diff --check`.
- Evidence:
  `xfeat_rdk_x5/artifacts/f21_semidense_single_m256_all.json`,
  `f21_semidense_single_m384_all.json`,
  `f21_semidense_single_m512_all.json`,
  `f21_semidense_m256_board_ncc_smoke_summary.json`,
  `f21_semidense_m384_board_ncc_smoke_summary.json`,
  `f21_semidense_m384_board_full.csv` (SHA-256
  `8b3b6d419082f266ceb5f9951fcecc1c991c2581cc30938b99fcdd74c62eed70`),
  and `f21_semidense_m384_board_full_summary.json`.
- Touched files:
  `xfeat_rdk_x5/evaluate_fine_matcher.py`,
  `xfeat_rdk_x5/rdk_x5/xfeat_frontend.{hpp,cpp}`,
  `xfeat_rdk_x5/rdk_x5/ros_benchmark.cpp`,
  `xfeat_rdk_x5/deploy_board_benchmark.sh`,
  `xfeat_rdk_x5/summarize_semidense_board.py`,
  `xfeat_rdk_x5/README.md`, and F21 governance/evidence files.
- Superseded blocker: a complete ARM64 archive was supplied later on
  2026-08-03; the cross-build and first production S316 replay are recorded in
  the following addendum. Five alternating runs, XFeat TrackStereo evidence,
  EuRoC ATE/RPE, and SLAM integration are still not complete.
- Next recommended step: fix coverage and the map-reset statistics snapshot,
  then repeat ORB/XFeat M384 in alternating order. Only if the formal 90%
  speed and 95% quality gates pass should F22 begin.

## Addendum 2026-08-03 — complete ARM64 image and production ORB run 1

- Archive `/home/ywl/temp/ros1_arm64_ubuntu22.04.tar` is complete and loaded
  as `ros1_arm64_ubuntu22.04:v3`; archive SHA-256 is
  `66a8b417b8f74dcce80029b6eabff9b143176abef716d58707775126b61c6354`.
- Modified `/home/ywl/Project/RDK_X5_orb_slam/tools/build_rdk_arm64_docker.sh`
  for the new image/entrypoint and ROS `set -u` compatibility. Added
  `Dockerfile.rdk-arm64-orb-build`, which installs only `libepoxy-dev`, and
  built local image `rdk-orb-slam3-arm64-build:v3`.
- Build result: `-j4` was OOM-killed at `Optimizer.cc` under the 9 GiB limit;
  `-j1` reused completed objects and passed both catkin packages. New hashes:
  `ros_stereo` `71198355...057e`, core library `cada3095...957e`.
- Staged, without overwriting existing deployment, under
  `/userdata/orb_slam/f21_crossbuild_staging_20260803`. Board `ldd`, hashes,
  S316/vocabulary loading, ROS initialization, full replay, and clean Shutdown
  all passed.
- Run 1 processed all 644 callbacks. With 30 warmups and one invalid
  map-reset row excluded: ORB extract 57.73 ms, stereo 13.24 ms, frontend
  70.96 ms mean / 81.76 ms P90, TrackStereo 88.15 ms, valid depths 215.51,
  depth coverage 0.755, and one 2.97 s loss segment.
- M384 Fine run-1 comparison: 55.05/59.37 ms mean/P90, so speed passes;
  strict count 375.4 passes, but coverage 0.580 is only 76.8% of ORB and fails.
  F21 remains active and F22 promotion remains false.
- Evidence:
  `xfeat_rdk_x5/artifacts/f21_orb_crossbuild_run1_manifest.json`,
  `f21_orb_production_run1.csv` (SHA-256 `dd4f0a7e...61f1`), and
  `f21_preliminary_production_gate_run1.json`.
- Unverified/remaining: final Cortex-A55-specific rebuild, safe stats snapshot
  across Atlas map reset, four additional alternating runs, XFeat SLAM
  tracking-loss comparison, and EuRoC trajectory metrics.

## Addendum 2026-07-31 — F18 XFeat INT8/BPU deployment

- Final state: `passing` for the fixed `1x1x480x640` XFeat BPU core.
- Toolchain: Docker image
  `openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.6`, `hb_mapper 1.23.8`,
  HBDK 3.49.14, and `horizon-nn 1.0.6.3`.
- Calibration: 100 evenly spaced mono8 left-infrared frames from
  `2026-06-05-09-59-04.bag`, converted to normalized float32 tensors. The
  manifest is stored outside the tensor directory because `hb_mapper` reads
  every directory entry as calibration data.
- Conversion: `hb_mapper checker` placed all 33 reported nodes on one BPU
  subgraph. `hb_mapper makertbin` completed and produced a 920,712-byte model
  with SHA-256
  `0ff4a43f8b9085c56da2173969f9e42989c79cb6c296ce484d0eb7360a693dea`.
- Board: files and the compiled runner are under `/userdata/xfeat_rdk_x5/` on
  `root@192.168.1.10`. DNN 1.23.10 / HBRT 3.15.54 loaded the model and reported
  builder version 1.23.8. Fifty BPU0 inferences after ten warmups averaged
  13.473 ms (13.429–13.584 ms); model load took 125.450 ms.
- Accuracy: all three board outputs exactly matched `hb_mapper infer` on the
  quantized ONNX. Board-vs-float cosine was 0.995747 for dense descriptors,
  0.990869 for keypoint logits, and 0.997176 for reliability.
- Main commands run: `hb_mapper checker`, `hb_mapper makertbin`,
  `hb_model_info`, float/quantized `hb_mapper infer`, board CMake build, the
  repository DNN runner, SHA-256 checks, and `compare_ptq_outputs.py`.
- Touched files: `xfeat_rdk_x5/prepare_calibration.py`,
  `xfeat_rdk_x5/compare_ptq_outputs.py`,
  `xfeat_rdk_x5/config/xfeat_backbone_480x640_bayes_e.yaml`,
  `xfeat_rdk_x5/rdk_x5/`, `xfeat_rdk_x5/.gitignore`,
  `xfeat_rdk_x5/README.md`, `docs/xfeat-rdk-x5-research.md`,
  `docs/features.md`, `docs/validation.md`, and `docs/progress.md`.
- Evidence: `xfeat_rdk_x5/artifacts/ptq_board_validation_480x640.json` and
  `xfeat_rdk_x5/artifacts/calibration_bag_2026-06-05-09-59-04_480x640_manifest.json`.
- Not run: complete sparse XFeat postprocessing/matching on the board and an
  XFeat-integrated ORB-SLAM trajectory replay. The tested core resizes native
  `640x544` bag images to `640x480`.
- TODO: export and calibrate a native `640x544` core, implement/tune the CPU
  sparse postprocessing path, then measure full frontend latency and quantized
  stereo/temporal inliers before ORB-SLAM integration.

## Addendum 2026-08-03 — same-frame ORB/XFeat visualization

- Added `xfeat_rdk_x5/orb_slam3_stereo_dump.cpp`, its build wrapper, and
  `visualize_feature_distribution.py`. ORB uses the real
  `Frame::ComputeStereoMatches`; XFeat uses official `semi_dense_single`,
  top-k 600, M384, cosine 0.82, ratio 0.9, and Fine confidence 0.25.
- Bag frame 320 matches the production ORB CSV at 603 raw left points and 270
  valid depths. ORB covers 37/48 cells; XFeat* coarse/Fine cover 29/48. Fine
  changes median disparity from 0.0 to 3.00 px and filters 384 coarse
  candidates to 369 strict points, but cannot fill empty left-image cells.
- Evidence is under `xfeat_rdk_x5/artifacts/feature_visualization/`.

## Addendum 2026-08-03 — 09-56-06 bag feature visualization

- Active feature/state: F21 remains `active`; no later feature was started.
- Rendered frame 440 of `2026-06-05-09-56-06.bag` with the real ORB-SLAM3
  stereo path and official host-FP32 XFeat* semi-dense M384 coarse/Fine path.
- ORB: 605 raw left points, 224 valid stereo depths, 43/48 occupied cells.
  XFeat*: 600 raw points, 384 coarse candidates, 371 Fine-accepted candidates,
  and 20/48 occupied cells for both coarse and Fine.
- Evidence directory:
  `xfeat_rdk_x5/artifacts/feature_visualization_bag_2026-06-05-09-56-06/`.
- Verification completed: JSON parse, PNG type (`2880x1600 RGBA`), and SHA-256
  `d4126d5389282c42b271d114dd8a41d61ee6d0b5029a34bd0f0ed88a5a9e01d4`.
- TODO: repeat with board INT8 coordinate dumps before treating the host-FP32
  XFeat positions as exact BPU deployment evidence.

## Addendum 2026-08-03 — reliability and grid-quota implementation

- Active feature/state: F21 remains `active`; F22–F24 remain gated.
- Changed the nested official XFeat checkout's dense API to return selected
  reliability values. Host and board adapters now use the same score semantics.
- Added balanced extraction and match quotas. Defaults are 8x6 cells,
  automatic extraction maximum `ceil(top_k/48)` (13 for top-k 600), and eight
  coarse matches per cell. Zero disables the corresponding quota.
- Verification commands completed: Python compile, three regression tests,
  frame-440 visualization/JSON validation, five-anchor Fine smoke, two RDK-X5
  CMake builds, and final deployment under
  `/userdata/xfeat_rdk_x5/benchmark_build/`.
- Final board executable is 439,368 bytes with SHA-256
  `9c16d6d5d1ef2332af842fd816a8b57e4d885276c78e574f72e1f273ef1bcd42`.
- Frame 440 result: XFeat raw/coarse/Fine coverage is 48/48, 47/48, and 47/48;
  coarse/Fine strict counts are 325/285. Before this change coarse/Fine covered
  only 20/48 cells on the same frame.
- Evidence:
  `xfeat_rdk_x5/artifacts/feature_visualization_bag_2026-06-05-09-56-06_grid_quota_v3/`
  and `xfeat_rdk_x5/artifacts/f21_grid_quota_host_smoke_09-56-06.json`.
- Not run: final board bag replay. A six-second node start could not reach a
  ROS master at `localhost:11311`, so it does not count as runtime validation.
- Next: start the configured LAN ROS master, replay the full bag against the
  deployed binary, and compare board latency/NCC/coverage with the frozen ORB
  production baseline before changing the F21 state.

## Addendum 2026-08-03 — hard-grid full replay and tracking diagnostic

- Active feature/state: F21 remains `active`; F22–F24 remain gated.
- Completed the final board replay of the current hard-grid M384 binary over
  the LAN ROS master. The board produced all 520 expected records (130 each
  for XFeat coarse, XFeat Fine, and ORB stereo; 65 each for XFeat and ORB
  temporal), covering source frames 0–641. The fetched CSV SHA-256 is
  `eb59d7d63ed450a5742c9904b7e5727b0d28e7f8f655def3e93aec68c18850c2`.
- Board result: hard-grid Fine stayed fast at `54.53/61.27 ms` mean/P90, but
  NCC-qualified stereo correspondences fell from the pre-grid `289.42` to
  `186.03`, and temporal RANSAC inliers fell from `357.52` to `270.80`.
- Added and ran `xfeat_rdk_x5/evaluate_stereo_vo.py` over all 65 sampled frame
  pairs. It computes previous-frame stereo depth, temporal XFeat Fine
  refinement, and a shared PnP RANSAC for hard-grid XFeat, no-hard-grid XFeat,
  and an OpenCV ORB control.
- PnP result: ORB control `200.25` mean inliers, 65/65 success; no-hard-grid
  XFeat+Fine `213.0`, 65/65 success; hard-grid XFeat+Fine `99.38`, 64/65
  success. Frame 561 was fragile with 45 candidates and 24 inliers, while the
  same frame had 59 XFeat-global and 58 ORB inliers.
- Interpretation: strict round-robin selection confuses occupancy with useful
  information. It admits weak points from sparse cells and caps repeatable
  high-quality cells, so coverage rises while stereo photometric validity and
  temporal/PnP constraints fall. Fine Matcher itself remains viable because
  the no-hard-grid control meets this diagnostic.
- Verification commands completed: evaluator full run, `python3 -m py_compile
  xfeat_rdk_x5/evaluate_stereo_vo.py`, `python3 xfeat_rdk_x5/test_grid_quota.py
  -v` (3/3 passed), row/frame/timestamp and finite-value checks, SHA-256
  checks, and `git diff --check`.
- Evidence:
  `xfeat_rdk_x5/artifacts/f21_grid_quota_m384_board_full_20260803.csv`,
  `f21_grid_quota_m384_board_full_20260803_summary.json`,
  `f21_grid_quota_preliminary_gate_run1.json`, and
  `xfeat_rdk_x5/artifacts/f21_stereo_vo_full/{per_pair.csv,summary.json}`.
- Commands not run: no second production ORB replay and no EuRoC trajectory
  run were performed. The common-PnP ORB row is an OpenCV control and must not
  replace the existing real S316 production baseline.
- Next recommended step: replace hard equalization with a soft selector that
  first guarantees a small minimum per occupied cell, then globally fills the
  remaining budget by reliability while enforcing only a loose maximum. Tune
  it offline against stereo NCC and PnP, then rebuild/replay on the board only
  after it removes the frame-561 failure and reaches at least 95% of ORB
  quality. Do not start F22 before the five alternating production runs and
  trajectory gates also pass.

## Addendum 2026-08-03 — four-way frontend comparison

- Active feature/state: F21 remains `active`; F22–F24 remain gated.
- Extended `xfeat_rdk_x5/evaluate_stereo_vo.py` with a 600-point official
  sparse XFeat method that bypasses Fine Matcher and quotas. The previous
  three methods retain their behavior.
- Full four-way run produced 260 aligned finite data rows. Sparse XFeat had
  83.0 mean PnP inliers, 63/65 success, and two fragile pairs (frame 511: 9;
  frame 561: 6). No-hard-grid XFeat+Fine retained 213.0 mean inliers and 65/65
  success, versus the ORB control's 200.25 and 65/65.
- Verification: Python compilation, a two-pair four-way smoke run, full 65-pair
  run, row/frame/timestamp/finite checks, SHA-256 checks, and `git diff
  --check`. Evidence is in
  `xfeat_rdk_x5/artifacts/f21_stereo_vo_fourway/`; detailed comparison is
  `docs/xfeat-fourway-comparison.md`.
- The existing deployed sparse result used 1024 points and global matching;
  it remains valid as a deployment reference but is not conflated with this
  600-point local-matching PnP result.
- Next: retain the no-hard-grid M384 candidate, implement a softened quota only
  as an optional challenger, and proceed to SLAM/EuRoC gates only after the
  frontend result is reproduced in alternating board runs.

## Addendum 2026-08-03 — deployed sparse-600 profile

- Active feature/state: F21 remains `active`; F22–F24 remain gated.
- Updated `rdk_x5/run_ros_benchmark.sh`, `ros_benchmark.cpp`, and
  `xfeat_frontend.hpp` so the deployed sparse/no-Fine control defaults to 600
  features. The run profile also uses a 600 coarse-match limit, no Fine model,
  and zero extraction/match grid quotas.
- `bash -n` and `git diff --check` passed. The board native CMake build reached
  100%, and the deployed program printed the exact expected runtime parameters.
- One-anchor replay wrote six data rows. Every XFeat row reported 600/600
  keypoints; Fine timings were zero. The temporal row recorded 429 matches,
  383 RANSAC inliers, and 104.09 ms end-to-end.
- Evidence: `xfeat_rdk_x5/artifacts/f21_sparse600_board_smoke.csv` (SHA-256
  `473263d93c84c6f36e504a1698dacef56b61de7390d1a68fd0c71efbaaf8546f`).
  Deployed binary/script hashes are `3fa4f964...adb98` and
  `6b961c23...83fd` respectively. No benchmark process was left running.
- Not run: a complete 65-pair/full-bag sparse-600 board benchmark. The smoke
  proves the deployed feature budget and profile, not its aggregate latency or
  quality distribution.

## Addendum 2026-08-03 — F21 formal gate passing

- Active feature final state: F21 is now `passing`. No F22 code was started;
  F22 remains `not_started` and is no longer gated by F21.
- Frozen candidate: `xfeat_rdk_x5/config/f21_xfeat_star_m384.yaml` with
  XFeat* single scale, 600 points, extraction quota `-1`, match quota `16`,
  M384 Fine, confidence `0.20`, cosine `0.82`, and eight CPU workers on BPU0.
- Five alternating production runs completed. ORB recorded 644 callbacks in
  every run; XFeat recorded all 520 expected rows in every run. Combined
  frontend mean/P90 was ORB `70.505/81.827 ms` and XFeat `54.633/59.402 ms`.
- Common real-S316-ORB/XFeat NCC+PnP verification passed every gate; both had
  65/65 PnP success and zero fragile pairs. The formal machine result is
  `xfeat_rdk_x5/artifacts/f21_gate_final.json` with SHA-256
  `f07e1a445443015fb8846b7ab406379295e03f860457194bbc092f6dade6acba`.
- Cortex-A55 cross-build passed. Board staging hashes are `c15c99cb...ebe4`
  for `ros_stereo` and `26b833f0...f836` for `liborb_slam3_ros.so`; no
  production file was overwritten. Source backups are in
  `/home/ywl/Project/RDK_X5_orb_slam/f21_backups/20260803_gate_close/`.
- Two sequential 1000-iteration Fine runs passed with identical output hash
  and stable peak RSS. No `ros_stereo` or `xfeat_ros_benchmark` process was
  left on the board.
- Commands verified: Python compilation, helper build, threshold-0.20 PTQ
  comparison, common 65-pair estimator, five-run gate summarization, shell
  syntax, `git diff --check`, board hashes/`ldd`, and process cleanup.
- Not run by design: F22 hybrid integration, F23 EuRoC ATE/RPE, and F24 XFeat
  vocabulary/relocalization/Atlas compatibility. Next recommended step is F22
  hybrid stereo-depth integration using the frozen F21 profile.

## Addendum 2026-08-04 — feature-count sensitivity after F21

- Active feature/state: no implementation feature was activated. F21 remains
  `passing` with its frozen 600-point M384 profile; F22 remains `not_started`.
- Ran the common 65-pair XFeat tracking diagnostic at M384 for top-k
  576/560/512/480/384/320, plus 576 with M256. Outputs are under
  `xfeat_rdk_x5/artifacts/feature_count_sweep/`.
- M384 top-k 576 and 560 both returned 65/65 PnP success and zero fragile
  pairs, with minimum inliers 35 and 31. Top-k 512 and below failed the
  existing no-fragile-pair requirement. M256 at top-k 576 also failed with
  64/65 success and a 24-inlier minimum.
- No build, board replay, source modification, or deployment was performed.
  Before considering 576 M384, run a complete board replay and compare
  tracking loss, P90 latency and tail coverage against frozen 600 M384. Do not
  adopt 560 based on the current one-inlier worst-case margin.

## Addendum 2026-08-04 — frame 561 top-k 512 diagnosis

- Active feature/state remains unchanged: F21 `passing`, F22 `not_started`.
- Exported bag frames 560/561 and reconstructed the same XFeat* M384 pipeline
  for top-k 600 and 512. The 512-point solver returned a plausible pose but
  only 28 inliers, below the 30-inlier diagnostic gate; this is a gate failure,
  not a numerical PnP failure or proof that integrated ORB-SLAM3 must lose.
- The top-k reduction removed 14 of the 600-profile's 70 PnP candidates and 7
  of its 37 PnP inliers directly. Subsequent consensus changes produced the
  final `70/37 -> 59/28` candidate/inlier reduction. Dynamic people, repeated
  texture and weakly distributed static geometry make this a tail-risk frame.
- 500 RANSAC iterations did not improve the result. A 2.5 px threshold reached
  33 inliers but materially changed the estimated motion, so thresholds were
  not modified. No source, build, deployment, or frozen configuration changed.
- Evidence: `xfeat_rdk_x5/artifacts/feature_count_sweep/frame561_analysis/`.

## Addendum 2026-08-06 — branch publication verification

- Active feature/state remains unchanged: F21 is `passing`; F22 is
  `not_started`. This session packaged the existing ROS pose-continuity and
  XFeat/RDK X5 work for publication and did not start F22 implementation.
- Added the missing direct `std_srvs` rosbuild dependency required by the
  `/pose_correction/set_enabled` service in `ros_stereo.cc`.
- Verification completed successfully with `./build.sh`, `./build_ros.sh`,
  `PYTHONPATH=/tmp/xfeat-rdk-x5-pydeps:/opt/ros/noetic/lib/python3/dist-packages
  python3 xfeat_rdk_x5/test_grid_quota.py -v` (3/3 passed), and `bash -n` for
  the staged shell helpers. The ROS build linked `Stereo` successfully.
- Staged-scope checks covered 215 files (about 63.2 MB); the largest file was
  4.9 MB, and a credential-pattern scan found no secrets. Fifteen generated
  CSV evidence files retain their original line endings, so `git diff
  --check` reports trailing whitespace for those data files only.
- The generated `.ros_runtime_11312/` PID directory was intentionally left
  untracked and excluded from publication.
- Not run: no live camera, rosbag replay, RDK X5 board session, loop-closure,
  map-merge, or tracking-loss runtime validation was repeated. Existing
  runtime TODOs for `/robot_pose`, `/robot_pose_map`, TF continuity, and the
  correction service remain applicable.
- Publication target: branch
  `agent/ros-pose-continuity-xfeat-rdk-evaluation` based on `master`.

## Addendum 2026-08-06 — F25 stereo local-map gate diagnostic

- Active feature/final state: F25 `passing` as a temporary diagnostic. The
  tested threshold remains isolated and is not a production recommendation.
- Changed the RDK cross-build source at
  `/home/ywl/Project/RDK_X5_orb_slam/src/orb_slam3_ros-master/orb_slam3/src/Tracking.cc`:
  the pure-stereo local-map gate is 25, inertial gates are unchanged, and
  pose-step/reprojection diagnostics run only near the gate or on a large pose
  step. No source under this repository's `src/` or `include/` was changed.
- The user ran `RDK_BUILD_JOBS=4 ./tools/build_rdk_arm64_docker.sh`; both catkin
  packages passed. The ARM64 node/library hashes are `d6df0d5e...f71ccb` and
  `3f4de91f...e085c54f`. Deployment used the independent board directory
  `/userdata/orb_slam/f25_stereo_gate25_sparse_20260806` and did not replace
  the previous staging or `/userdata/orb_slam/devel` artifacts.
- Runtime controls: camera PID 39105 and all its threads were temporarily
  pinned to CPUs 0-1; ORB-SLAM3 was pinned to CPUs 2-7; Pangolin was disabled;
  bags were played at 1.0x from host `192.168.1.3`; left/right images were
  remapped beneath isolated `/baggate25_sparse_*` topics. Camera affinity was
  restored to CPUs 0-7 and no test `ros_stereo` process remained afterward.
- `09-59-04` passed in one map: 643 state-2 callbacks, 135 KFs and 2822 MPs.
  Frame 523 was accepted at exactly 25 local-map inliers with 0.062627 m,
  1.613700 deg, and 1.264135/2.374674 px reprojection mean/P90. Atlas:
  `ORB_SLAM3_Map_1786001967.955328.osa` on the board.
- `09-56-06` produced 853 state-2 plus 30 state-3 callbacks and split into
  74-KF and 61-KF maps. Frame 432 failed before the local-map gate because the
  reference KF produced only 9 BoW matches; frame 431 still had 50 local-map
  inliers. Atlas: `ORB_SLAM3_Map_1786001728.921163.osa` on the board.
- Evidence copied to the host:
  `/tmp/orb_slam3_095606_rate1_camera_cpuiso_gate25_sparse.csv`,
  `/tmp/orb_slam3_095606_gate25_sparse_diag.log`,
  `/tmp/orb_slam3_095904_rate1_camera_cpuiso_gate25_sparse.csv`, and
  `/tmp/orb_slam3_095904_gate25_sparse_diag.log`. Exact SHA-256 values are in
  `docs/validation.md`.
- Commands not run: no production deployment, install-space build, IMU mode,
  or viewer-enabled replay was performed. The local RDK diagnostic source is
  still at 25 for follow-up experiments; restore it to 30 before using that
  workspace for a production build.
- Remaining uncertainty: one replay per final sparse-logging configuration is
  insufficient to separate threshold effects from asynchronous LocalMapping
  scheduling. The next meaningful step is an alternating repeated A/B test of
  threshold 30 and 25 from separately staged, otherwise identical libraries.

## Addendum 2026-08-10 — F22 Hybrid XFeat stereo depth passing

- Active feature/final state: F22 `passing`. F23 remains `not_started` and is
  now unblocked. The RDK target used was `root@192.168.1.10`.
- Implemented a provider seam that refines only stereo depth on existing ORB
  keypoints. ORB binary descriptors and all temporal/MapPoint/BoW/loop paths
  remain unchanged. The provider uses the frozen F21 600-point backbone and
  M384 Fine model, patch NCC acceptance, add-only default behavior and guarded
  replacement.
- Verification passed: host `./build.sh`; standalone frontend/provider syntax
  compilation; shell/Python checks; ARM64 catkin cross-build at `-j1`; board
  `ldd`, hashes and model-init smoke; and six complete board runs over the
  three user-provided lawn bags at controlled 0.5x playback.
- The third bag was unindexed. Its original at the repository root was left
  byte-for-byte untouched; `rosbag reindex` was run only on
  `/tmp/f22_2026-08-06-09-50-14.bag`.
- On all 2,496 paired timestamps, ORB keypoint counts were identical and no
  frame lost valid stereo depth. Hybrid added 16.13-18.39 mean depths and
  13.88-18.79 mean tracked inliers. Loss frames were equal on the first two
  bags and improved from 34 to zero on the third.
- Cross-build artifacts are isolated at
  `/userdata/orb_slam/f22_hybrid_20260810`; node/library SHA-256 values are
  `5a507a46...1eaf9b` and `56987b52...7766f9`. Existing devel, F21 and F25
  staging artifacts were not replaced.
- Evidence is under `xfeat_rdk_x5/artifacts/f22_hybrid_ab/`; the authoritative
  aggregate is `summary.json`. No F22 `ros_stereo` process remained after the
  run. The existing host ROS master was reused and left running.
- Touched current-repository files: `include/StereoDepthProvider.h`,
  `include/{Frame,System,Tracking}.h`, `src/{Frame,System,Tracking}.cc`, the
  Hybrid provider, deploy/replay/summary helpers under `xfeat_rdk_x5/`, F22
  artifacts, and the four project progress/validation documents. The RDK
  build workspace additionally changed its package `CMakeLists.txt`,
  `src/ros_stereo.cc`, `config/Stereo/S316.yaml`, mirrored core headers/sources,
  and `tools/build_rdk_arm64_docker.sh`; a minimal board-derived BPU link
  sysroot is under `tools/rdk_bpu_sysroot/` there.
- Remaining limitation: Hybrid callback mean is 155.92-162.41 ms, about 53 ms
  slower than ORB, so it cannot sustain the 11 Hz S316 stream. This is expected
  for a bridge that runs both extractors. The next step is F23 descriptor-aware
  tracking, beginning with an explicit feature/descriptor abstraction and
  initialization path before touching relocalization or vocabulary handling.
- Not run: 1.0x/11 Hz Hybrid replay, EuRoC MH_01/MH_03/MH_05 ATE/RPE, XFeat
  temporal descriptor integration, XFeat vocabulary, relocalization, loop
  closing, or Atlas model-tag compatibility. Those belong to F23/F24 and must
  not be inferred from the passing F22 depth-only bridge.

## Addendum 2026-08-10 — F23 full XFeat tracking blocked

- Active feature/final state: F23 is `blocked`. F24 was not started because it
  remains gated on F23 passing.
- Implemented descriptor-aware ORB/XFeat distance and medoid utilities; a full
  stereo feature-provider seam; an RDK-X5 XFeat provider; float-descriptor
  Frame/MapPoint/reference-keyframe tracking; single-scale-aware projection;
  and last-frame/current-frame mutual-nearest-neighbor plus M384 Fine temporal
  matching. Float BoW is intentionally bypassed and loop closing is explicitly
  disabled until F24 supplies a float vocabulary and descriptor-tagged Atlas.
- Host commands passed: the `-Wall -Wextra -Werror` descriptor test,
  `cmake --build build --target ORB_SLAM3 -j1`, `bash -n
  xfeat_rdk_x5/run_f23_xfeat_smoke.sh`, and `git diff --check`.
- RDK cross-build passed with `RDK_BUILD_JOBS=1
  ./tools/build_rdk_arm64_docker.sh` in
  `/home/ywl/Project/RDK_X5_orb_slam`. Final ARM64 node/library SHA-256 values
  were `1d1f2ae6f1c8f218ebcae036431e92a2bb5082beab2383a6c4c6a2ffc42ebd8b`
  and `7cad809094747f0ec4d64e7e9ed9bd24574939405c907c464ef6f2f18e595029`.
- Board stage: `/userdata/orb_slam/f23_xfeat_20260810`. The test stage contains
  the final core library. The final ROS node was not copied after a combined
  upload approval-channel failure, but no ROS-facing ABI was changed in that
  iteration; the staged node loaded and exercised the final core library.
- Best short evidence:
  `xfeat_rdk_x5/artifacts/f23_xfeat_smoke_temporal_fine/smoke.csv` and
  `smoke.log`; 87/87 state-2 and pose-valid callbacks, zero loss segment,
  158.25 mean inliers, 72.23 ms frontend mean. SHA-256 values are
  `9ed85cb222016c9f8a1e5bc0cf376fedae4028e1acd8cc1e9e79570f8e5d5e82`
  and `74dd7bbbf2f82c3b8f647e5d3a63f7c03cf959bc49cc9e12dd314101bfc964af`.
- Complete first-lawn evidence:
  `xfeat_rdk_x5/artifacts/f23_lawn_full_094639_temporal_fine/smoke.csv` and
  `smoke.log`; 981 callbacks, 682 state-2, 299 state-3, 29 loss segments,
  2.97 s maximum loss and eight maps. SHA-256 values are
  `49cfbba2c226190117eb1d19a04cffef43eeb277f0945d1feb362878bab1079c`
  and `096cb1f8051817a1acbf7e403dcf9cc725618e3a48f27765bf8731d85cdb7946`.
- Commands not run: the formal MH_01/MH_03/MH_05 five-run ORB/XFeat gate,
  because no matching dataset directories were found under the local project
  roots; the other two lawn bags were not promoted as substitutes because
  the first complete bag already fails long-run continuity and none supplies
  external ground truth for ATE/RPE. `./build.sh` was not rerun end-to-end;
  the directly affected `ORB_SLAM3` target and ARM64 ROS package both passed.
- Touched current-repository areas: `CMakeLists.txt`,
  `include/{FeatureDescriptor,StereoFeatureProvider,Frame,LoopClosing,ORBmatcher,System,Tracking}.h`,
  `src/{FeatureDescriptor,Frame,KeyFrame,MapPoint,ORBmatcher,System,Tracking}.cc`,
  F23 provider/test/replay files and evidence under `xfeat_rdk_x5/`, and the
  four governance documents. The RDK workspace additionally changed its
  package CMake/source/config/launch and cross-build wrapper.
- Preserved user/runtime content: `.ros_runtime_11312/` remains untracked and
  untouched; all three original lawn bags remain unchanged. The existing host
  ROS master at `192.168.1.3:11311` was reused and left running.
- Next recommended step: inspect why long-run LocalMapping culls the map to
  zero optimization vertices despite strong short-run temporal tracking.
  Add per-keyframe MapPoint creation/culling and covisibility evidence, then
  validate a targeted fix on the complete first lawn bag before acquiring and
  running the formal EuRoC five-run gate. Do not lower the 30-inlier tracking
  gate or mark F23 passing from the 8-second smoke alone.

2026-09-07: RGB-D keyframe policy research (analysis only)

- Scope: literature/source review and historical-log analysis, not a new
  implementation feature. Existing feature states were not changed.
- Audited runtime core `fc1c50c8cf1c096725126d88510ed4c626bdddd3` in
  `/home/ywl/Project/RDK_X5_orb_slam/debug_branches/rgbd_track_timing/core`;
  compared the relevant source lineage against F27 commit `84a964b`.
- Recomputed insertion intervals and same-map relative motion from
  `handheld_oom_runs/handheld_20260904_154857/`: 966/1396 insertion gaps were
  one input frame; 816/1390 valid consecutive KF pairs were below 5 cm and
  3 degrees. These are online-pose statistics, not ground-truth accuracy.
- Corrected the prior interpretation of LBA logging: 281/287 stop flags at
  the end of LM cycles do not establish only six effective BA updates.
  The flag is sampled after optimization/culling; an interrupted optimizer
  can still write estimates. Actual iterations/writeback need instrumentation.
- Evidence and proposed policy: local generated report
  `evaluation/artifacts/keyframe_policy_study_20260907/report.md`, including
  primary literature/official-code links, exact local source references,
  initial parameter ranges and replay/accuracy verification design.
- Recommended policy: lightweight visual novelty and risk signals, ordinary
  insertion budget with explicit turn/near-point exceptions, and nonblocking
  backend feedback before `InterruptBA`. Actual enqueue success commits the
  budget. Map/reset events invalidate geometry history. Map-object reclamation
  is a separate requirement for bounded long-term memory.
- Touched: this handoff and the generated report only; no algorithm, board,
  configuration, dataset or process state changed. Runtime worktree clean.
- Validation: read-only source/log checks, CSV calculations and documentation
  whitespace/link checks. Builds and ROS replay not run because no runtime
  implementation was requested or produced.
- TODO: exact production compile commit/dirty diff is unavailable; no raw
  images or ground truth were recorded for this handheld run. Shadow-policy
  logs, real BA progress telemetry, same-bag 1.0x repeats and outdoor turn
  accuracy tests remain necessary before claiming a best operating point.

2026-09-08: bag integrity audit (analysis only)

- Scope: inspect `2026-09-08-09-56-57.bag`; no implementation feature was
  selected or changed. Existing user YAML and earlier handoff content preserved.
- Result: original is truncated, 5,188,853,760 bytes with header index pointer
  6,370,758,821; 4257 complete chunks, one partial, 969 absent relative to
  header declaration. Original left in place; independent reindexed copy
  contains 4257 readable image/depth messages spanning 191.524 seconds.
- Verification: original `rosbag info --yaml` fails unindexed; physical record
  scan and 4257 local index checks pass up to truncated tail; output reindex
  and info pass; full message deserialization, payload checks and timestamp
  analysis completed. Retained raw chunk/index byte region hashes identical.
- Evidence and touched generated outputs:
  `evaluation/artifacts/bag_integrity_20260908_095657/` (report, JSON, NPZ,
  logs, scripts and recovered bag). This handoff was appended.
- Not run: builds, ROS publishing replay or SLAM; this task only audits data.
- TODO: compare original recording-device file size/hash and recopy full bag;
  source checksum and original end time unavailable. Depth units, outlier
  conventions, camera calibration and geometric alignment not verified.

2026-09-08: recovered bag playback follow-up

- User playback error: connection refused / failed to contact master at
  localhost:11311. Advised starting roscore before replay.
- Actual full-file `rosbag play --immediate --quiet` passed with exit code 0
  and Done, using isolated localhost master port 11329; test master stopped.
- Evidence: bag integrity artifact directory `play_check.log`,
  `play_check.json`, `play_check_roscore.log`, `verify_play.py`; report updated.
- No bag, SLAM source, camera YAML, or feature-state changes. Fast publishing
  verified, but subscriber delivery rate and 1x real-time playback untested.

2026-09-08: recovered bag RDK-X5 1.0x mapping baseline

- Scope: one RGB-D baseline replay under the existing F27 production behavior;
  no algorithm implementation or feature-state changes. Preserved user YAML
  and all prior handoff content. Bag stayed on the laptop.
- Ran isolated host ROS master 192.168.1.3:11332 and board production_w3,
  frozen 700 features / 5 levels / 3 workers, with namespaced RGB/depth.
  Generated run_once.sh adapts F33 with owned PID handling and persistent
  logs. Corrected its preflight because rosnode info returns zero for an
  unknown node; that rejected preflight never started SLAM or playback.
- Replay and SLAM exits 0; 2105/2128 paired inputs processed, 23 missing
  callbacks; 2070 OK, 34 RECENTLY_LOST and one new-map state0. One interruption
  at source 126.720 s, restored by new map after 3.150 s. No loop or merge.
- Atlas has 2 actual maps, 1288 KFs and 49770 MPs. No OOM/crash/reboot evidence.
  Full-process sampled RSS peak 1009.45 MiB, Swap peak 257.39 MiB,
  same-sample sum peak 1232.51 MiB; saving contributes additional memory.
- Save service and board collector exited 0. SCP directory '/.' collection
  failed (controller exit 1); rsync recovery copied all 41 regular files,
  175620425 bytes. Preserved failure evidence; matching host/board SHA256
  verified for Atlas and both timing CSVs. Test processes/master stopped.
- Found PreSave appends to mvpBackupMaps without clearing; second save reports
  4 backup entries while live-map set remains 2. Do not double-count map/KF/MP
  totals. No core fix was made. TUM KF export covers only active map; frame
  TUM mixes map references and includes RECENTLY_LOST, so not ground truth.
- Evidence/touched generated outputs:
  evaluation/artifacts/board_recovered_20260908_095657/ (run/analysis scripts,
  logs, summary.json, report.md, three plots, recovered board results).
  This handoff was appended; no SLAM source, binary or camera settings edited.
- Validation: bash -n, actual 1.0x board replay, exact source seq/stamp matching,
  CSV analysis, plot inspection, map/timing transfer hash checks, clean stop.
  Builds not run: frozen binary baseline, no source changes. Atlas reload,
  ground-truth accuracy, failure-image review and longer/full-bag OOM testing
  remain TODO. Original bag tail is missing; source only lasts 191.524 s.

2026-09-08: failure-image follow-up for recovered-bag baseline

- Analysis-only continuation of the F27 baseline; no feature-state or SLAM
  changes. Extracted 151 exact RGB/depth pairs (seq1350-1500, 121.500-135.000s)
  from host recovered bag, including the five locally missing callbacks.
- Added RGB PNGs, lossless float32 depth NPZ, previews, HTML gallery, phase
  montages, timing detail plot and evidence report under the run's
  failure_images/ directory. Compiled a standalone host ORBextractor helper;
  146/146 delivered-frame keypoint counts agree with board CSV (not a
  byte-identical descriptor claim across OpenCV versions).
- Evidence supports right turn plus missing intermediate observations:
  source1403->1408 spans450ms, host-runtime ORB mutual/H-inliers45/30,
  compared with81-156 H-inliers in each omitted adjacent pair. Surviving
  jump H-inliers all lie in upper half; near floor features are numerous
  but poorly persistent at that span. No blackout/depth-wide dropout seen.
- First failure executes motion-model and reference-KF paths, not LocalMap;
  shared failure text is misleading. KF856 is last OK seq1403, so this is
  not sparse KF insertion. Pure RGB-D relocalizes unsuccessfully for3s,
  then seq1443 initializes new map/KF857. Stale inliers99 hidden in plots.
- OOM/Swap not observed at loss. Input backlog/queue loss specific layer is
  still TODO; no full-delivery counterfactual replay, internal MapPoint
  correspondence replay or metric ground truth. Preliminary OpenCV-ORB
  PnP fields are unvalidated/degenerate and explicitly excluded from claims.
- Validation: source pair count, timing/seq alignment, host extractor build
  and run, offline matching, PNG/NPZ generation and visual inspection.
  Preserved existing user YAML changes. No board process actions this turn.

2026-09-08: missing-input queue forensic follow-up

- Read-only board ELF disassembly verified frozen production_w3 subscribers
  use queue_size1 each and TimeSynchronizer10, with single-thread ros::spin.
  SHA256 matches baseline a41fd835...; current source20/20/20 is NOT the
  frozen deployment. Outer runtime a28ac28 has1/1/10; a71c1b1 changed20/20/20.
- Source interval90ms versus observed callback interval101-104ms accumulates
  ~350ms relative age;1403->1408 skips450ms source time in101.407ms wall time.
  Combined with depth/RGB independent size1 queues, ROS oldest-message eviction
  plus exact-sync incomplete-pair cleanup is now the leading explanation.
- Do not overclaim: raw arrival/drop events absent, so exact per-message
  eviction sequence and which stream dropped remain TODO. INFO-only logs do
  not exclude queue overflow (roscpp reports it atDEBUG).
- Evidence: run/input_drop_forensics/report.md and production_main_disassembly.txt;
  failure_images/report.md linked update. No source/config/deployment changes,
  no new replay. Future diagnostic should preserve core and instrument queues.

## 2026-09-08: Docker source provenance follow-up (analysis only)

- Verified archived August 17 production_w3/ros_rgbd SHA-256 is exactly
  a41fd8359313a0c78bb690743c08b964a18491b94c21d797019986666a99cc7b,
  matching the board replay. Current runtime devel node is 4962aece...1f684,
  a different artifact; do not infer its queue values from source alone.
- Docker bind-mounts WORKSPACE to /ros1_project/orb_slam and CORE_SOURCE to
  /ros1_project/orb_slam3_core. Generated build.make explicitly compiles
  WORKSPACE/src/orb_slam3_ros-master/src/ros_rgbd.cc; queue constants belong
  to this executable, not the core shared library or Docker image.
- Outer a28ac28 versus a71c1b1 ros_rgbd.cc diff is exclusively 1/1/10 to
  20/20/20. September 7 commit preserves pre-existing changes; it does not
  establish their original edit date. Original top-level RDK workspace still
  has 1/1/10. Runtime script now defaults to its own workspace/core; historical
  DEBUG_BRANCH.md documents explicit overrides for that same isolated tree.
- Checked historical/current build scripts, CMakeCache/build.make, deployment
  README, archived hashes and both Git histories. Historical full build env
  and immutable source manifest remain unverified; source baseline similarity
  must not be represented as byte-exact provenance of every compiled file.
- No compilation, Docker execution, replay, deployment or source changes.
  Only this handoff was appended; unrelated user YAML changes preserved.

2026-09-12: RDK X5 RGB-D map-memory research (analysis only)

- User confirmed 2 GB RAM, 2–4 GB swap, RGB-D, and substantial swap use
  around 1400 keyframes. Historical Linux-visible RAM was about 1307.55 MiB;
  current board memory/other workloads were not remeasured.
- Reviewed primary papers, author repositories, a firsthand Quanser blog,
  upstream ORB-SLAM memory issues, current/runtime source, and prior X5 reports.
- Report: docs/map-memory-research-20260912.md. Proposes measured attribution,
  observation-based GBA capacity, idempotent save preparation, safe retirement,
  compact feature grids, quality-aware sparsification and disk-backed submaps.
- Verified runtime source still has nine KF×MP vector reserves, pointer-only
  Map erasure, disabled bad-map deletion, and PreSave backup-list appending.
  Capacity estimates are not measured RSS or a diagnosis of prior OOM.
- No implementation feature selected or feature states changed. Only the
  research document and this appended handoff were written; existing user
  YAML and prior handoff changes preserved.
- Validation: read-only source/report checks, static arithmetic, and document
  link/whitespace checks. No builds, replay, SSH, deployment or runtime tests:
  this request was for research and a plan, not an implementation.
- TODO: current memory budget, per-object lifetime telemetry, target ABI,
  loop/GBA long-run data, independent accuracy/area evidence and storage I/O.
- Next step: P0 attribution in the runtime core with frozen binary/config
  provenance, then implement and validate one optimization at a time.

2026-09-12: Map-object reclamation clarification (analysis only)

- Clarified that delete and shared_ptr are both usable with a complete
  lifetime protocol; immediate deletion and all-strong pointer replacement
  are the problematic shortcuts.
- Read stella_vslam map/keyframe/landmark/graph/frame-statistics source:
  explicit shared ownership, weak reverse/graph links, retirement cleanup,
  and historical frame-reference migration provide a concrete reference.
- Read Buffalo thesis sections 4.3/4.4 on custom counting and deferred
  collection; no verified drop-in author patch was found in this review.
- Extended docs/map-memory-research-20260912.md with evidence, proposed
  ORB-SLAM3 ownership roles, trajectory rebinding and staged migration.
- Validation: source reading and document whitespace checks only. No build,
  runtime test, source implementation or deployment; feature states unchanged.
- TODO: complete lifetime audit, map-merge/serialization contracts and ARM
  runtime overhead before enabling physical reclamation.

2026-09-14: MS-SLAM omission correction (analysis only)

- User identified missing MS-SLAM (JFR, online 2024-09-04, 2025 issue).
  Acknowledged incomplete candidate coverage, missing author-followup search
  and failure to inspect the already available local MS-SLAM checkout.
- Read publication method/evaluation sections, official README and local
  MS-SLAM 51142b3 sparsification, KF/MP retirement, descriptor compaction and
  historical reference migration paths.
- Corrected docs/map-memory-research-20260912.md at the top and in the source
  table; added concrete local references and revised prioritization.
- MS-SLAM uses shared_ptr with explicit relation teardown; weak_ptr is an
  alternative design, not a mandatory condition for reclaiming cyclic graphs.
- Prioritize MS-SLAM implementation review and separate lifetime, payload and
  selection ablations; retain stella_vslam as an ownership-design comparison.
- No source/build/deployment changes. Existing MS-SLAM build-file changes,
  compare_results/, current-repo YAML changes and artifacts/ were preserved.
- Validation: online and local source checks, document whitespace checks.
  No compilation/replay: scope is correcting research and candidate ranking.
- TODO: full concurrency/reset/merge audit, RGB-D, ARM solver feasibility,
  DBoW3/serialization integration, same-data memory and accuracy experiments.

2026-09-14: camera_field_03 transfer and save-stall analysis

- Analysis only; feature states unchanged. Copied the user-specified board run
  `/userdata/rd53_camera_delivery_20260914/camera_field_03` to local
  `artifacts/save_stall_20260914/camera_field_03`; board data retained.
- All 31 source regular files (239278752 bytes) match fresh remote SHA-256.
  Final status complete, camera/slam/control/log exit codes zero. Earlier
  first-save snapshot is also preserved separately; no map reload performed.
- First save service 351.715 s; shutdown to final map mtime about 903.986 s.
  Logs prove repeated SaveAtlas and duplicate PreSave map processing (1 -> 2
  backup entries). Swap peak 1842.1 MiB; available minimum 13.8 MiB. Runtime
  source corroborates uncleared backup list and shutdown save filename reuse.
- Existing diagnostic analyzer and local summarize.py generated report.md,
  evidence.json and memory.png under artifacts/save_stall_20260914/analysis.
  Chart visually checked; 1706 memory samples, 2 parsed diagnostic events.
  Recorder dropped 92; event_1 prewindow only 8.446 s; event_2 triggered after
  normal camera shutdown, so it is not an independent camera-failure event.
- Benchmark has 4559 paired callbacks, max 23.393 s, 265 unobserved seq IDs
  within 0..4823. Loop, lost tracking/new map and merge are logged; their
  precise causal relationship to memory pressure remains unverified.
- No code build, deployment, board cleanup or process interruption. Existing
  YAML changes preserved. TODO: save idempotence and atomic replacement,
  thread-consistent snapshot, reload/accuracy validation and memory attribution.

2026-09-14: camera_field_03 map visualization

- Generated local standalone read-only Atlas exporter and PNG/HTML/PLY views
  under artifacts/save_stall_20260914/visualization; original map unchanged.
- Native archive v18 header/ABI validated explicitly; host Boost 1.71 uses
  headerless decoding with input library version18. Full237410143-byte consume,
  2 backup entries/1 unique map,73953 points/1801 KFs, finite coordinates.
  KF positions match saved TUM to max6.07e-6m; timestamps within5.01e-6s.
- Main view displays72994 points,959 outside crop retained in full view and
  HTML/PLY.18 points beyond100m from origin; farthest3185m. Do not infer real
  site extent or metric accuracy from outliers. Display -Y is not gravity up.
- Exporter compiled/ran; Python plots generated; local headless Chrome page
  rendering and static/browser screenshots inspected. Initial compiler flags
  and old Matplotlib3D import issues resolved; commands recorded in README.
- No main runtime rebuild/deployment or board changes. Atlas PostLoad and
  localization replay not performed; this validates geometric extraction,
  not all map references or tracking accuracy. Existing user changes retained.

2026-09-14: 500m² mower memory sizing (analysis only)

- User confirmed0.5m lane spacing and0.3m/s speed for estimates; with assumed
  10% turning overhead gives1100m and61.1min. Actual KF trajectory161.757m XZ,
  1801KFs/422.008s; single-run density11.134KF/m,4.27KF/s.
- Fit318 pre-late-transient samples: RSS+swap MiB=523.631+0.459593×KF,
  R²0.9953. This includes correlated map/history costs, not per-object size.
  Distance/time extrapolations yield12.2–15.6kKFs and roughly6–8GiB steady
  process footprint. A separately labeled linear peak stress scenario gives
  16–20GiB; neither is a verified physical-RAM requirement or upper bound.
- Report: artifacts/save_stall_20260914/analysis/lawn_500m2_memory_estimate.md.
  Current2GB insufficiently supported;8GB optimization target,32GB baseline
  test host suggestion conditional on current growth. No purchase or deployment.
- Primary ORB-SLAM paper checked for redundant map selection principle;
  exact lawn memory depends on overlap/culling/reclamation and global peaks.
  Feature states unchanged; no new runtime tests or algorithm changes.


2026-09-14: MS-SLAM reuse audit for RDK X5 RGB-D

- Analysis/docs only; no coding feature selected and no feature state changed.
- Reviewed MS-SLAM51142b3, timing outerc6acc9e/core705c710, then latest field
  outer18ba684/core4d6df6a including existing worktree changes and delivery
  provenance. Preserve all existing user/source/artifact changes.
- Report: /home/ywl/Project/RDK_X5_orb_slam/reviews/ms_slam_reuse_20260914/README.md.
  source-audit.json records52 source/evidence fingerprints, ownership/API/build
  inventory and timing/field map-core equality. review-validation.json:52
  hashes unchanged,49 local links valid. Updated research document with link.
- Findings: pure RGB-D sparsification enters via LocalMapping; Tracking still
  retains an unused shared KF candidate list in that mode. Explicit graph
  teardown incomplete on clear/reset; Replace clears replacement reference
  while Tracking still reads it. KF compaction requires index/DB consistency.
  Gurobi has no resource/status controls; shutdown batches all remaining KFs
  of CURRENT map (Atlas getter is current-map only), before other workers stop.
  Save/load and ROS API integration missing. ARM64 Gurobi is officially
  supported; locally inspected MS/Gurobi libraries are x86-64 only.
- Latest field delivery is direct8/8/10, not archived FIFO16. Its incremental
  cross build replaces Tracking/LocalMapping objects and assumes unchanged
  class layout; ownership/payload port requires full core/node rebuild.
  Existing camera_field_03 evidence confirms long-run/save pressure and
  repeated PreSave entries; it is not a new replay from this session.
- Verification: rg/sed and lexical inventories, file architecture inspection,
  local hash/link validation, primary paper/repo/Gurobi documentation checks.
  Initial one-off inventory emitter had a parenthesis syntax error, fixed
  before artifact generation; final inventory/validation completed.
- Not run: new build, Gurobi/license initialization, sanitizers, RGB-D replay,
  board experiment/deployment or map reload. This audit does not certify
  runtime correctness. Next: full field-source build baseline and solver/model
  budget probe, then ownership, payload/persistence and bounded sparsification
  as separate validated implementation stages.


2026-09-14: Gurobi commercial licensing correction

- User identified a missing deployment consideration. Earlier audit mentioned
  license initialization but incorrectly retained Gurobi as the default P0.
- Verified official commercial/academic licensing descriptions, Academic WLS
  restrictions, Gurobi standard EULA deployment scope, and HiGHS MIT license.
- Rewrote report summary, dependency assessment and P0: default evaluation is
  HiGHS MILP model port plus bounded greedy; Gurobi is optional only with an
  applicable license. No license pricing or device entitlement inferred.
- Updated docs/map-memory-research-20260912.md and the RDK audit report and
  review-validation.json. Existing runtime sources and source fingerprints
  unchanged by this docs-only correction. All local report links valid.
- No license credentials read, software installed, build, solver benchmark,
  board connection or deployment. Replacement performance remains unverified.


2026-09-14: MS-SLAM native RGB-D bag memory validation

- User requested actual MS-SLAM validation with the previous bag. Evaluation
  lane lives outside this repo at /home/ywl/Project/MS-SLAM/evaluation/bag_memory_20260914;
  no ORB_SLAM3_DBoW3 feature state or runtime source changed. Preserve existing edits.
- Losslessly extracted all 2910 RGB8/32FC1 exact-stamp pairs from
  2026-09-10-11-35-57.bag. Same 700-feature/5-level YAML, native 1x timestamps,
  no ROS/viewer, DBoW2, isolated full rebuild of MS51142b3 and ORB4452a3c.
  Actual loaded libraries and input/source/binary hashes captured.
- Gurobi initialized in host environment; sandbox host identity failed.
  Existing installation/license unchanged; no commercial entitlement inferred.
- Completed orb_1, ms_1, ms_2, orb_2: each2910 frames, all state2, single map,
  no logged loop/merge, swap0. Mean pre-shutdown PSS1226.90 ->985.98MiB (-19.64%);
  baseline-subtracted growth773.29 ->529.92MiB (-31.47%). Full-lifetime peak
 1227.06 ->1256.29MiB (+2.38%); shutdown batches569/602KFs, solve1.96/2.14s.
- Online MS destructed327137/324420 MapPoints; liveMP36897/39026 versus
  ORB370137/372063. Descriptor rows roughly halved. Candidate KF strong refs
  still1407/1410; no KF destruction observed. Not a bounded-memory guarantee.
- Optional keep_1 ablation skips only solver-result SetBadFlag, still solves
  and compacts KFs. Stalled after last complete statistics at frame2136;
  three futex-wait threads. Ptrace denied and sudo -n required password;
  no system setting changed. Terminated only this owned process after176s
  without progress; exit-15 and partial buffers preserved. Excluded from
  complete comparisons; contribution of MIP-only deletion remains unresolved.
- Commands: extraction/prepare scripts; cmake configure and full -j2 build;
  run_suite.py --smoke, run_suite.py, --diagnose, --stop-stalled-keep,
  --finish-repeats; analyze.py, compare_trajectories.py, verify_results.py,
  probe_sizes.py. Initial hook compilation issue fixed in isolated sources;
  full build passed. Postprocessing handled truncated failed-run output;
  size probe linked required Pangolin support. Four main evidence checks pass;
  ablation explicitly unusable. Counter adds8 bytes/MP,0 bytes/KF in both cores.
- Report, plot, CSV, manifests, patches, logs, verification and size probes
  retained in the evaluation directory. RDK reuse report links new evidence.
- Not run: X5/ARM/DBoW3 integration, alternative solver, sanitizers, Atlas
  save/reload/reset or larger new area. No truth in bag: pairwise SE3 pose
  disagreement is not ATE. No deployed runtime files modified.
- Next: independent reclamation/payload controls, diagnose ablation stall,
  fix lifecycle and shutdown budgets, then open-source solver and X5 validation.

2026-09-18: REC01 standalone S316 RGB-D recording — passing

- Added laptop entrypoint `record_rgbd_loop.sh`, worker/checker
  `evaluation/rgbd_recorder.py`, offline regression
  `evaluation/test_rgbd_recorder.py`, and `docs/rgbd-loop-recording.md`.
  Updated README, repo map, validation, feature list and the narrow docs
  ignore exception. No SLAM/ROS C++ node or driver source was modified.
- Board inspection confirmed `root@192.168.1.10`, Noetic,
  `/ros1_project/camera_ws/install/setup.bash`, and `/home/metoak/mo_init.sh`.
  `/home/mo` does not exist. Source initialization in the camera's shell,
  retaining SDK paths and skipping only already-loaded kernel modules.
- Flow: isolated board-local ROS master, live RGB-D preflight, local board
  LZ4 recording, explicit recording notification, Enter/Ctrl+C stop, recorder
  flush before camera shutdown, resumable rsync, SHA-256 and full bag reads.
  Board originals and logs are retained. `--fetch` retries without opening
  the camera. Disk reserve and control heartbeat trigger cleanup.
- Passed `bash -n record_rgbd_loop.sh`, Python compilation, `--help`, and
  `/usr/bin/python3 evaluation/test_rgbd_recorder.py` after sourcing Noetic.
  Offline cases cover healthy, absent depth, all-zero depth, non-monotonic
  timestamps, malformed payload, unsynchronized streams, truncated and active bags.
- Hardware command: `./record_rgbd_loop.sh --duration 6 --output
  artifacts/rgbd_recorder_validation`; run `rgbd_loop_20260918_145800_5dc9d3`
  returned 134217703 bytes, 1311 messages, 65 RGB / 66 depth frames, 65 pairs.
- Manual Ctrl+C command: `./record_rgbd_loop.sh --output
  artifacts/rgbd_recorder_validation`; run `rgbd_loop_20260918_150021_125c59`
  returned 172202854 bytes, 1688 messages, 84/84 exact-stamp pairs at 11.11 Hz.
  Both board results and local bag checks passed; second initialization
  confirmed all four previously loaded modules were handled successfully.
- `--fetch rgbd_loop_20260918_145800_5dc9d3 --output
  artifacts/rgbd_recorder_validation` passed. Evidence under that directory:
  per-run logs, snapshots, board_result.json, stream_check.json, bag_check.json,
  fetch_test.log, and empty final_board_processes.txt (no camera/ROS leftovers).
- Existing user's S316_stereo.yaml trailing blank line and prior handoff
  additions were preserved. Whole-repo `git diff --check` reports that
  pre-existing YAML whitespace; the task's changed files pass scoped checks.
- Not run: C++ builds (no compiled source change), real connection loss,
  disk exhaustion, power loss or physical closed-loop SLAM evaluation.
  TODO: user records the actual route; basic integrity/pairing checks cannot
  establish geometric calibration or successful SLAM loop detection.

2026-09-18: REC01 follow-up — delete board bag after verified transfer (passing)

- User requested automatic board-space reclamation. `verify_download` now
  gates deletion on matching SHA-256, full bag/RGB-D validation and successful
  recording/SSH status. Fsync the local bag/directory, then recheck the remote
  hash and delete only that session's `rgbd.bag`. Preserve logs, configuration,
  other sessions and `.active` files; failures keep the source for retry.
- Updated `evaluation/rgbd_recorder.py`, `evaluation/test_rgbd_recorder.py`,
  README, recording guide, feature list, validation notes and this handoff.
  Existing user changes remain untouched.
- Passed shell syntax, Python compilation and all 6 offline test groups.
  New cases cover successful cleanup/retry, failed validation/recording/SSH,
  changed remote contents, and unfinished recording protection.
- Real command: `./record_rgbd_loop.sh --fetch
  rgbd_loop_20260918_145800_5dc9d3 --output artifacts/rgbd_recorder_validation`.
  Revalidated 1311 messages and deleted only its 134217703-byte board bag;
  the matching local bag, board logs, and second session bag remain.
  Evidence: `fetch_cleanup_test.log` and `remote_cleanup_check.txt` under
  `artifacts/rgbd_recorder_validation/`.
- No camera restart or C++ build needed for this post-transfer change.
  TODO: actual network loss during deletion and storage hardware failure were
  not induced; cleanup errors report uncertainty and allow --fetch retry.

2026-09-18: F17 follow-up — ignore root artifacts (passing)

- Added `/artifacts/` to `.gitignore` after user approval. The 45,683
  untracked delivery/evaluation files no longer appear in Git status;
  local artifacts and existing user edits are preserved.
- Verified `git check-ignore -v` on delivery and recorder evidence paths,
  `git status --short --untracked-files=all` (10 remaining changes), and
  representative local file existence. Scoped `git diff --check` passed.
- Touched `.gitignore` and this handoff only. No builds or hardware tests
  were needed for this ignore-rule change; no commit, push or Git cleanup ran.
- Next: review the remaining 10 changes before committing. TODO: remote
  status remains unrefreshed; this check used local Git metadata only.

2026-09-24: REC02 D435i synchronized 10 FPS bag recorder — passing

- Added laptop entrypoint `record_d435i_10fps_bag.sh` and
  `docs/d435i-10fps-recording.md`. It connects to `sunrise@192.168.1.11`, starts
  `/userdata/ros1_realsense_ws/start_rgbd10.sh`, validates the exact-pair
  `/camera_10fps` RGB/aligned-depth streams, records those images plus their
  CameraInfo and TF, then reuses REC01 finalization, transfer, full-read,
  SHA-256 and verified remote cleanup logic.
- Generalized `evaluation/rgbd_recorder.py` with an optional camera start
  script, busy-process guards, configurable CameraInfo topics, snapshots,
  run prefix and retry wrapper while retaining S316 defaults. Added a
  configurable preflight warmup; D435i discards two startup seconds before its
  independent five-second stability sample.
- Passed shell syntax, Python compilation, scoped diff checks, `--help`, and
  all 6 existing offline regression groups. No ORB-SLAM C++ build was needed.
- Real command: `./record_d435i_10fps_bag.sh --duration 8 --output
  artifacts/d435i_10fps_recorder_validation`. Passing run
  `d435i10_20260924_111100_130a26` preflighted at 9.99 Hz/100% pairing and
  produced a 48,284,291-byte indexed LZ4 bag with 317 fully read messages:
  79 each of RGB, aligned depth and both CameraInfo streams, plus one TF static.
  RGB/depth were both 9.9929 Hz and paired 79/79 within 0.1 ms. SHA-256:
  `e34f0df7f7583f7da17c51f254a59cbeb6cf14867c57e4205a1a79f41343f1c4`.
- The verified board bag was removed; board logs remain and no RealSense,
  adapter, rosbag, roscore or roslaunch process remained. Evidence is under
  `artifacts/d435i_10fps_recorder_validation/d435i10_20260924_111100_130a26/`.
- Initial directory creation required one-time ownership setup for
  `/userdata/d435i_10fps_bags`. An earlier attempt correctly rejected the
  startup-transient 9.477 Hz window before recording; its logs are retained.
  TODO: the user still needs to record the physical closed-loop route. Network
  loss, disk exhaustion and power loss were not induced.

2026-09-24: RGBD01 D435i + MEM18/perf20 adaptation — implemented, tracking blocked by scene

- User requested adaptation and real joint validation, choosing static scene first.
  Added `run_d435i_slam.sh`, `evaluation/d435i_slam.py`,
  `evaluation/test_d435i_slam.py`, `evaluation/summarize_d435i_slam.py`, and
  `docs/d435i-slam.md`; updated README, docs routing/validation/features, and
  added only the new guide to the existing .gitignore whitelist.
- Live CameraInfo generates 640x480/10 FPS PinHole settings with measured K/D,
  rgb8 ordering and 16UC1 millimetre scale 1000. Input subscribes directly to
  `/camera_10fps/color/image_raw` and aligned depth. No C++ change or rebuild.
  Pinned perf20 node/core hashes, file/ldd and loaded core path were verified;
  existing deployments, YAMLs, Swap and governors were unchanged.
- Ran `./run_d435i_slam.sh 120 off` and `./run_d435i_slam.sh 60 on`.
  Final off run `20260924_114152_off_3817` processed all 1200 measurement-window
  pairs; on run `20260924_113803_on_730` processed all 600. About 9.99 Hz,
  exact pairing, no input-window loss, graceful stop. Off peak PSS 487.03 MiB,
  VmSwap=0, pswpin/out deltas=0; all transferred off files passed SHA256 checks.
- Tracking did NOT pass: all callbacks non-OK, no keyframe/map point/valid pose.
  On run reports state=1 throughout, keypoints 268–320 (median 292), against
  existing `Tracking::StereoInitialization` >500 gate. Image points mainly at
  ceiling/light/plain walls. Asked user to aim at textured objects and hold
  still; no completed adjustment confirmation yet. Do not lower the gate.
  TODO: re-run off/on after scene adjustment; motion/loop/accuracy/navigation
  TF remain unvalidated. Low callback times are uninitialized-path cost only.
- Fixed validation harness issues: use actual ROS subscriptions rather than
  buffered stdout for readiness; separate resource stderr and allow natural
  sampler exit; generate remote SHA manifest after process/rospy atexit ends.
  Initial failed runs and their stale manifests remain documented; separate
  `analysis_review.json` reanalyses preserve original results.
- Checks: 6 adaptation tests and all 6 prior recorder regression groups pass;
  bash/Python syntax and scoped diff checks pass. Global diff-check still
  flags pre-existing S316_stereo.yaml line 87 whitespace, left untouched.
  No git commit/reset/cleanup/push/merge. Final board check shows no camera,
  adapter, SLAM, roslaunch or ROS master remaining; new board outputs ~2.9 MiB.
- Evidence/report/JSON/CSV/log index and final archive checksum manifest:
  `artifacts/d435i_slam_20260924/`. Active feature RGBD01 is `blocked` only for
  actual initialized tracking validation, not for input/config adaptation.

2026-09-24: RGBD02 laptop live launcher — passing (launcher, not tracking gate)

- Added executable `start_rdk_rgbd_slam.sh` and `evaluation/rdk_rgbd_live.py`.
  Laptop needs only Python3 and ssh/scp; starts a unique board-local ROS master,
  D435i exact-pair 10 FPS input and pinned MEM18/perf20 with live CameraInfo YAML.
  Reuses existing calibration/process helpers without modifying finite validation.
- SLAM stdout/stderr uses stdbuf line buffering and is forwarded immediately to
  the terminal with [SLAM] prefix; original node log stays on board and combined
  terminal output is saved locally. Latest tracking status is shown every 5 s.
  Default runs until Ctrl+C; optional `--duration N` stops after N ready seconds.
  No unbounded Python per-frame statistics, timing, benchmark, bag or atlas save.
- Host 2-second heartbeats plus board EOF/stop/15-second timeout control stop only
  owned process groups; busy camera/SLAM and lock guards reject double starts.
- Real run `live_20260924_131622_eb6ff1`: actual settings/vocabulary/subscription
  output appeared while process was live; sent host terminal Ctrl+C after 232
  tracking-status messages. camera/SLAM exited 0, all cleanup checks true, final
  board process scan empty. Evidence `logs/rdk_rgbd/live_20260924_131622_eb6ff1/`.
  Tracking was false in the same low-texture scene: RGBD01 remains blocked.
- 4 new offline tests passed (live line forwarding before EOF, control EOF,
  timeout, explicit stop); previous 6 calibration tests passed. Shell/Python
  syntax, help and scoped diff checks passed. No C++ rebuild required.
- Updated README, docs/d435i-slam.md, local repo-map, features and validation.
  No existing deployment or system setting modified. TODO: real network-fault
  injection/power loss not tested; successful initialized tracking still needs
  a textured scene, independently of the launcher feature.


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


## LM01 C2.1 — reduce transaction collection overhead (2026-09-24)

Replace per-KF repeated weak ownership promotions with attempt-private identity
hint/exact deduplication and retained owners. Snapshot deltas use a reserved
contiguous vector and an identity index; projection pointers are sealed only
after all candidates are known. Batched snapshot timing removes per-point clock
calls. Commit avoids redundant map locks under the existing identity barrier,
but retains all-before-any-write feature/position validation and counter semantics.

Native full/KF80/shadow 120-frame tests, 27 original ARM probes, new frozen
parity/cap/competition/invalidation fixtures, TSan race/small-lock fixtures and
ASan/UBSan/LSan fixture pass with previously documented dependency exceptions.
The private collector fixture evicts hints and checks exact dedup/order without
shared marker writes. Board full/KF80 short tests both process120/120, non-OK0;
Callback P95 is100.200/99.795ms, Local Map P95 is43.001/41.127ms. This improves
C2's112.448/115.138ms callback, but DOES NOT meet90ms and is not a promotion.
Long2910 screening is running; its results are not included in this commit gate.
Default staysOFF. C3–C6 remain required. Evidence: IDE
artifacts/local_map_budget_20260924/c21/r2/checks.json and immutable runs/.

Tools now freeze native binary pairs before replay, record governor/frequency,
CPU time and temperature, report full-path residence including budget recovery,
and provide a TUM PNG replay driver (20/20 reader smoke only, no precision claim).


## LM01 C3 — typed events and adaptive probes (2026-09-24)

Map event generations/versions are runtime-only. Loop correction, committed
GBA, merge, active-map switch and reset publish explicit types; ordinary
IncreaseChangeIndex does not. Controller separates budget and final health,
uses undistorted8x6 relative/absolute coverage, keeps one retry, escalates budgets,
and requires actual5-frame bounded probes after recovery. Startup/event safety
is separate from budget-induced cooldown, so fallback accounting cannot hide
budget recovery. Both health records and event/controller deadlines are logged.

Native120 and2910 KF80 runs process all inputs in stateOK. Original27 ARM probes
and new transaction/controller/event/grid fixtures pass (graph delta<=7.1e-15).
Controller event concurrency passes TSan and ASan/UBSan/LSan without suppressions.
The inherited full-System LSan issue is unchanged. C3r1's stale stationary-view
inlier reference caused repeated healthy probes to fail; negative evidence is
retained. C3r2 starts a fresh budget-health reference after entering recovery,
then still requires20 protected frames and5 real bounded successes. Full retry
inliers never establish budget health. Scene-change regression tests pass.

First boardKF80 long screen:2450/2910, non-OK0, callbackP95=136.285ms,
fixedlast25P95=145.490ms. OrdinaryOK full/retry fraction13.54%, last25=19.01%:
REALTIME,INPUT and RESIDENCE gates FAIL. DefaultOFF; no production promotion.
KF96/KF64 screening remains running on the immutable C3r2 ARM pair. C4–C6 remain.
C2.1 fixedKF80 long also failed:2490/2910,non-OK0,P95=123.895ms,last25=126.206ms;
cap violations/retries/commit failures0. Do not infer a win from smaller maps.
Evidence: IDE artifacts/local_map_budget_20260924/c3/r2/checks.json and runs/.


## LM01 C4a — experimental coverage and association/geometry diagnostics

Votes stays default. Coverage uses entry matched-point/grid bitsets, normalized
marginals, one pose read per candidate and incremental minimum diversity. It
never scans candidate KF point slots to score; chosen KFs retain original gather
order. Logs distinguish mandatory coverage, first saturation and post-saturation
vote/diversity/tie decisions. Optional shadow diagnostics compare actual feature
associations, optimized pose delta, residual and6x6 information spectrum. Their
cost is included in tracking; they do not change Tracking success criteria.

Native120 coverage/shadow,27 ARM probes, saturation/order and controlled frozen
transaction tests, geometry-rank fixture and ASan/UBSan/LSan fixture pass. The
sanitizer builds now read a frozen source snapshot, so later stages cannot alter
an in-flight build. C1 Boost UBSan-only exceptions remain; full-SystemLSan stays
open. Live shadow is separately labeled from exact frozen fixture parity.

First native2910 pair is2910/2910 OK in both modes. New-match recall votes99.833%
versus coverage99.885% (+0.052pp), new-recallP5 98.571/98.750%. Tail new recall
99.757/99.817%. SelectionP95 .229/1.883ms. Coverage saturates2879/2909 frames,
1456 at the mandatory set. This does not justify promotion. Second native pair
and interleaved board pairs remain running on immutable binaries/configs; C4b
will retain final decision evidence. C4a is an implementation checkpoint, not
final performance acceptance. First boardvotes2612/2910,non-OK35,P95=120.518ms.

Completed C3 adaptive tier screen also rejects every tier: KF96 processed2495,
non-OK70,P95=143.660ms; KF80 processed2450,non-OK0,P95=136.285ms; KF64 processed2577,
non-OK35,P95=129.535ms. KF96's first failure is before Local Map execution, after
a successful full-path frame and an input-sequence gap; cause remains unproven.
No candidate is eligible for production. C5 MP/statistics/cohort work and C6
formal verification/delivery remain. Evidence: IDE artifacts/local_map_budget_20260924/c4/.

## LM01 C4b — completed strategy decision

Two interleaved board pairs and two native shadow pairs completed on frozen C4a
binaries. Votes callback P95=120.518/123.913 ms; coverage=142.096/141.289 ms.
Processed counts were votes 2612/2547 and coverage 2279/2413 of 2910.
Non-OK counts were votes 35/0 and coverage 0/34. All board rounds FAIL.
Native four runs processed 2910/2910, all OK. New-inlier recall improvement was
only 0.029–0.052 percentage points (<0.5 pp promotion threshold), with selection
P95 about 1.9 ms vs votes 0.23 ms. Coverage remains experimental; votes remains
default. These results establish no realtime or tracking-quality acceptance.
Ordinary rounds generated no Atlas files; manifests and SHA checks were retained.
C5 implementation/validation is in progress, C6 remains.
Evidence: artifacts/local_map_budget_20260924/c4/{board,native}_comparison.json.
Core implementation b7ebd840; outer 6c495b5; evidence checkpoint bf514150.

## LM01 C5a — MP budgets/statistics/lifetime checkpoint

KF/MP transaction fixtures, normalized unique-KF scoring, grid/depth allocation,
young-reservation tests, lifetime concurrency and27 ARM probes pass.
ASan/UBSan/LSan fixtures and TSan full/lifetime/small-lock checks pass with
previously documented Boost/GDAL exceptions; inherited full-SystemLSan open.
Four native2910 runs are complete/allOK. MP6500 never binds (eligible max1439);
shadow new recall99.877%, association99.936%. Newborn gate worst drop0.122pp.
First boardC5legacy80 fails2436/2910,nonOK35,P95133.027ms. Six board ablations
and C6 formal/TUM/fault/save-reload matrix remain. DefaultOFF, votes default,
coverage experimental. See memory/LM01_C5_CONTRACT.md and IDE artifacts/c5.
Parent core64d073fd / outerc0ddab1 / IDE39ccbf6c; manifests archive dirty-build
source hashes and paired binaries. This is functional acceptance only.


## LM01 C5b — completed MP/statistics and tier screen

All six immutable board rounds failed input completeness and realtime:
legacy80 2436/2910, nonOK35, P95=133.027ms; attempted80 2358,35,134.134ms;
MP80 2443,0,133.633ms; adaptive96 2158,0,155.713ms; adaptive80 2446,70,136.946ms;
adaptive64 2434,0,137.711ms. Fixed-tail P95 ranges137.412–161.387ms.
The5000/6500/8000 post-filter MP limits never bind (eligible maximum2025).
All measured second/third unique-KF observations and2/3/5-KF survival deltas
are within1 percentage point of legacy80; the conditional10% young-reservation
long replay is not triggered. Its allocation/feedback mechanics passed the
purpose-built64-MP stress fixture. Different-KF/adaptive comparisons are
screening diagnostics, not isolation of a single causal change.

Cohort waiting time now distinguishes actual event wall time from dataset
progress; observing an older KF during fusion cannot turn a later observation
into a zero wait. Six analyzer tests pass; all native/board cohorts were
recomputed without changing count/survival gates. Core algorithm and tested
binaries are unchanged. Source core05e83e55, outer base9feff3c; implementation
C5a core906d870f/outer9188413. C6 matrix remains pending. DefaultOFF.
Evidence: IDE artifacts/local_map_budget_20260924/c5/completed/{board_matrix,native_analysis_v2}.json.
Conditional query-before-gather MIH and independently budgeted BA work is
specified in memory/LM01_FOLLOWUP.md; neither is implemented or accepted.


## LM01 C6 final — implemented, production gate blocked

All planned experiment groups were executed and negative outcomes retained.
Final ARM/core/node/probe sources are the126-file snapshot for core05e83e55
and outer9feff3c; later changes are analysis/tools/docs. Full six-binary native
reproduction freeze matches the tested manifest. Default LocalMapMode remains off.

Formal long A-B/B-A/A-B: A processed2752/2825/2831, P95=113.661/110.157/108.010ms;
B processed2282/2316/2431, P95=148.547/146.167/146.365ms, nonOK=0/36/111.
Fixed-tail B P95=153.786/172.585/148.022ms; ordinary tail full/retry=18.51/12.38/7.41%.
Formal loop: A953/955/955, B900/930/907 of955; B P95=123.417/110.799/116.878ms.
All six candidate board rounds fail input/P95/P99. All six loop rounds retain
one known loop and complete GBA writeback/finish/join. Pose topic integrity
checks pass for all processed OK callbacks, not ground-truth accuracy.

Final TUM: six attempts; five5057-pair completions, off1 interrupted after
merge cleanup stalled (4693 complete flushed rows). candidate1/3 allOK;
candidate2 and off2/3 split maps, corrected trajectory timestamps invalid.
No valid three-run ATE/RPE comparison; precision gate FAIL. Initial six TUM
runs are also retained. GT has129.865s gap; typically1227/5057 associations.
Final blackout+5-pair gap: both recover but split maps, candidate295 inputs
with52 nonOK; reset and intentional map-switch A/B240 each allOK. Save120/120
OK, loaded existing map101/120 OK; reload input-completeness FAIL. The8914885B
map was fetched/SHA-verified before manifest-owned deletion. Final board free
9775378432B; no experiment SLAM process or registered map remains.

Scoped transaction/statistics/KF/MP/controller/concurrency fixtures,27 ARM
probes and numeric max7.1e-15 remain PASS. C5 sanitizer exceptions/limits apply;
whole-System LSan remains inherited open. A merge progress failure is open,
not fixed or attributed solely to this feature. C5/C6 MP caps never bind;
no MP-cut speedup claim. All newborn gates within5pp. Coverage not promoted.

Analyzer audits: relative callback timestamps cannot align exactly to absolute
resource CPU samples; report whole-process and BA thread CPU. The inherited
visual BA num_MPs output is unpopulated; use separately scoped MemoryStats
candidate MPs and actual graph vertices/edges, never interpret timer zeros as
zero landmarks. No raw evidence was rewritten. Runtime algorithms were frozen
throughout formal testing. Follow-up MIH/BA and diagnostic-overhead work remains
conditional and unimplemented; see memory/LM01_FOLLOWUP.md.

Primary delivery: /home/ywl/Project/ORB_SLAM3_DBoW3/artifacts/local_map_budget_20260924/REPORT.md
Machine gates: c6/acceptance.json (production_pass=false); full data:
c6/formal_details.json, c6/final_tum, c6/final_faults, c6/save_reload,
c6/pose_sidecars/analysis.json, c6/board_final_audit.json.

Final LM01 runtime commits: core `be52247f`, outer `9c43a6c`; IDE final evidence is the commit containing artifacts/local_map_budget_20260924/DELIVERY.json.


## BAG01 外置盘2026-10-21 bag分析与归档（2026-10-24）

完整报告：[REPORT](../bags/camera_20261021/REPORT.md)。五包15,995,077,322字节；20,813消息全量反序列化、物理边界/计数和目标20,813项索引偏移/时间及SHA均通过。4包为640×480约15Hz的RGB与原始16UC1深度，配对98.154%–99.966%，最长帧间隔2.269秒；另1包只有0.999秒/16帧RGB。没有CameraInfo、TF、IMU、深度比例或已确认的深度对齐，不宣称可直接用于现有RGB-D/惯性流程或SLAM精度通过。

执行`audit_bags.py`、`import_bags.py --copy`、`verify_import.py`，均exit 0，证据位于`artifacts/bag_import_20261024/`；临时副本截断/magic/索引偏移负例被正确拒绝。5包已按包内北京时间重命名到`bags/camera_20261021/`，manifest/SHA256SUMS/逐帧时间戳/JSON/预览齐备。复用本地14,759,763字节完整副本及3,982,229,504字节精确匹配前缀，后者补齐479,878,609字节；两个原本地路径为归档文件的硬链接，原文件内容前缀未丢失，剩余约3.3GB。不更改算法、构建配置或其他既有数据，未运行构建/板测/SLAM（本任务为数据整理与离线分析）。

源删除命令曾在执行前被自动审批拒绝；随后用户明确回复“保留源文件，仅复制归档”。已按该选择完成收尾，5个外置盘原包均保留，源删除未执行，也没有待执行删除或待授权步骤。BAG01状态passing仅指数据检查、重命名复制归档完成，不代表SLAM运行资格通过。manifest及TRANSFER_STATUS/COMPLETION均更新为copied_verified_source_retained_by_user_choice；原审批拒绝作为历史记录保留，当前用户选择优先，不应再次请求或执行删除。共享文档原改动快照在`preflight/`。


## RGBD28 camera_20261021 最新版本板测交付（2026-10-24）

状态passing仅指测试和报告已完成，运行质量负面结果全部保留。用户要求最新ORB-SLAM3在板上测试bags/camera_20261021并先用既有D435i参数；使用与最新发布f75a3ffd对应的板端/userdata/rgbd_tracking_optimizations_20261020/atomic_02原成套程序，未修改算法或部署。旧D435i YAML（20260924_114152_off_3817）原样复用，源15Hz/配置10FPS及原始深度未确认对齐/比例明确记录。

prepare.py、run_all.py、summarize.py、post_audit.py及发布源verify_source.py均成功。4轮全段1×处理10360/10360对，有效新位姿9200；各包fresh=2895/2903、2618/3109、878/1010、2809/3338，map切换0/9/3/5，P95=53.31/57.19/55.34/62.74ms。采样RSS最大848.05MiB、VmSwap0，整机换页/OOM增量0。原始未配对47RGB/30depth帧明确排除且逐条留档，1秒RGB-only包缺深度跳过。4轮正常退出，56回传文件SHA复核，8个owned进程退出，旧程序SHA不变，无新增Atlas，板端余2731810816字节。scale记录不证明所有后台join；结果不支持轨迹精度结论。

交付artifacts/rgbd_camera_20261021_20261024/{REPORT.md,SUMMARY.json,PLAN.json,BOARD_FINAL_AUDIT.json,runs,inputs,tools}。板端结果保留/userdata/rgbd_camera_20261021_20261024/，约13.4MB；没有待运行会话。初次目录权限失败无发布，日志保留preflight/setup_failed_attempt_01*；修复只为本次新目录设置sunrise属主。共享文档原有用户改动的快照在preflight/。只改docs/features.md、validation.md、session-handoff.md及新本地归档；源bag、原REPORT和旧实验全部保留。没有运行核心/ROS构建，因为本次复用已冻结、核验来源的成套程序；不是代码更改任务。下一步如需提升稳定性，先核实实际相机参数和原深度对齐，再定义独立调参实验；不自动修改原始bag或替换旧部署。


## RGBD29 camera_20261021失跟离线分析（2026-10-24）

已完成用户要求的详细原因分析，主报告artifacts/rgbd_tracking_failure_analysis_20261024/REPORT.md。原板测SHA227项复核保持；使用实际冻结发布源并核对7文件与atomic归档逐字一致，不用cwd旧XFeat核心解释运行。只读原bag逐条检查10360对原图，vision_metrics.py完成4包图像指标，extract_case.py额外定点抽取1473病例；视觉聚合10360行、96个起始前后CSV与四张标准matplotlib六面板PNG通过身份/状态联表与目视检查。LK位移只在有效点>=30且存活率>=50%时汇总，指标为诊断代理、深度保持原单位，同像素角点采样不冒充实际ORB或空间配准。

两条独立工作流分别复算源码/日志和时序/资源，主报告再次独立只读复核。96段非fresh中76首次失败未调用局部地图；实际局部地图失败20次，11次<30内点，9次30–49受50保护门槛拒绝。日志7次参考KF词袋未就绪且got0，定位到先选为参考、后台才计算词袋、备用SearchByBoW缺就绪同步；日志无逐帧stamp，不能将7条精确归源序号或解释全部失跟。内部Relocalization门槛20与外层近期50之间有27次accepted未最终OK。17切图之外161410第469对8KF小图ResetActiveMap一次。相关Tracking/KeyFrame/LocalMapping/ORBmatcher与优化前baseline逐字一致，不能断言最近MapPoint原子化引入该逻辑问题，也不能无对照排除其时序间接影响。

17持续失跟起始10个header gap<100ms，仅3个>=1s，前5帧无>100ms回调；共9个>1s gap有5个当帧及后两帧仍fresh。1610152137源gap1.868s且LK1/139存活，源间断/视野大跳有具体证据；2277/2278区分原录制延迟与真实采集跨度。1605362854快速滚转、709特征/29内点；161015302清晰慢动仍前端失败；1615501473逆光低纹理，555→459特征、28内点，回调23.64ms。96起始仅5个Lap<300/1个<100，不作普遍模糊结论。资源没有分页/OOM/Swap共同模式。RGB与原深度空间对齐/标定/比例仍未知；源码直接按RGB像素取depth、d>0接受，65535在factor1000下会成为65.535m，若其为无效值才应掩膜，不猜设备语义。FPS10改变mMaxFrames与恢复/KF策略、不丢弃15Hz的每秒5帧。

执行脚本与子报告均在新归档，未修改原bag、旧证据、核心算法或部署，未重跑板测/构建（本任务是既有证据分析）。passing仅指分析交付完成，不代表算法修复或精度通过。后续先核实输入几何与BoW发布就绪契约，再用同包单变量复测；不建议直接降低内点门槛或同时改多个参数。共享docs原改动已在preflight快照保留。


## RGBD30 帧率与单帧回调成本对比（2026-10-24）

已完成用户所问15fps新包45–46ms、历史低帧率长包高耗时的离线对比。主报告artifacts/rgbd_fps_cost_comparison_20261024/REPORT.md；独立原始CSV归约workload_review/recompute.py、METRICS.json、REPORT.md保留全段/fresh/OK/四分段、阶段执行/零填充和局部工作量。十个历史/当前轮次CSV源SHA、行数与每帧ORB+TLM不超过回调的残差检查通过；相同atomic_02旧2轮/新4轮的core/nodeSHA一致。原bag只读索引及首条消息核对，旧2910对header11.111Hz/配置11、640x544 rgb8+32FC1，新640x480 rgb8+16UC1、源中位15Hz/沿用配置10。

同版本旧5817回调75.118ms→新10360回调45.860ms，ORB29.332→21.988，TLM每回调33.036→11.980、仅执行33.048→13.455；约72%的观测差额落在TLM。局部KF/MP/gather中位旧114/8236/80371，新29/1502/20417；首包无换图仍38/1741/26772及46.309ms，说明失跟重置不是唯一地图差异。新fresh-only45.818、nonfresh46.192ms，失跟跳阶段不是总体低均值主因。旧组合整包86.531/86.638、尾四分段95.191/93.523ms，约34–37%单帧>90ms；历史高耗时包含旧版本及重地图后段。线程CPU旧73.560/新44.483ms，MapWait仅0.095/0.111ms，不支持主要地图更新锁等待。较小图像少11.8%像素，内容/几何/轨迹和地图演化亦不同；不将阶段分账说成帧率或某单项的因果收益。

passing仅表示对比分析交付。帧率决定输入时间预算，单帧工作量决定实际回调成本；45ms不代表旧大地图也可15Hz。未修改算法、旧证据、参数或部署，未新跑板测/构建。本任务仅离线分析，无需算法单元测试；复核与源SHA/语法/算术闭合已完成。后续若要隔离帧率效应，需同序列同版本同几何做受控抽样率测试，同时检查质量与工作量；仅降播放速度是另一类墙钟/后台调度实验。共享文档既有改动快照在本归档preflight。


## REC03 D435i 录包 SSH 目标配置（2026-10-28）

状态 passing，仅表示参数选择和离线回归通过。根据用户最新要求，脚本仍在主机执行，通过 SSH 控制所选板卡；更换登录用户名或地址使用 `--board user@host`，新增 `D435I_BOARD` 环境变量作为默认目标。优先级为命令行 > 环境变量 > 原默认 `sunrise@192.168.1.11`。底层原已支持 `--board`，本次复用已有解析与 SSH/rsync 流程，只补入口配置和用法，没有修改 `evaluation/rgbd_recorder.py` 或相机采集/抽帧/录包逻辑。

修改前已逐字备份 5 个目标文件到 `artifacts/d435i_recorder_portability_20261028/before/`，SHA 清单在 `BACKUP.json`。保留原有 `docs/features.md`、`docs/validation.md` 和 `docs/session-handoff.md` 未提交改动，本次只增加 REC03 记录及相关入口说明。实际修改文件为 `record_d435i_10fps_bag.sh`、`docs/d435i-10fps-recording.md`、上述 3 个治理文档；新建本次备份和验证归档。

验证：`bash -n record_d435i_10fps_bag.sh` 通过；真实 shell 入口 `--help` 的参数经真实 Python 解析器检查，6 种场景全部通过（默认、环境变量、命令行、命令行等号写法覆盖环境变量、SSH 别名与 fetch、空环境变量回退），并核对 SSH 与 rsync 使用同一目标及原话题/阈值保持。该检查替换了 host 调度与 rsync 调用，没有联网。source `/opt/ros/noetic/setup.bash` 后 `/usr/bin/python3 -B evaluation/test_rgbd_recorder.py -v` 的既有 6 组回归全部通过。证据为 `wrapper_checks.json`、`recorder_regression.log`。

未运行板卡 SSH、相机实录、核心或 ROS 构建：本次只修改 SSH 目标配置，不涉及算法/节点；用户尚未指定要连接的新目标。TODO：新机器的 ROS1/相机工作空间、免密认证和目标录制目录权限尚未实机验证。使用时将示例用户名/IP 换为实际目标；另一台主机需同时具备 shell 入口、`evaluation/rgbd_recorder.py`、ROS1 Python 依赖、ssh 和 rsync。无待执行部署或录包任务。


## REC04 D435i 单文件录包（2026-10-28）

用户要求只复制一个新脚本到新机器即可运行，避免依赖本地 `evaluation/rgbd_recorder.py` 和板端自定义脚本。已交付根目录 `record_d435i_10fps_standalone.py`，保留原包装脚本/共享程序。本机只需 Linux、Python 3.8+、SSH 和 rsync/scp，不需要 ROS/NumPy/仓库；板卡仍需 ROS1 Noetic、Python ROS 消息/rosbag/LZ4/NumPy 及 RealSense ROS1 驱动和 USB 权限。新脚本通过 SSH 上传自身，在独立会话内生成 launch 和启动脚本，用内置 Python ExactTime 节点按1/4/7…对取样，无需预装旧 start_rgbd10.sh 或编译 rgbd_rate_adapter。

SSH 目标仍支持 `--board` / `D435I_BOARD`；默认板端目录改为登录用户家目录下的 d435i_10fps_bags，本地默认运行目录/bags，可用 --remote-root/--output 覆盖；--ros-setup/--camera-setup 控制板端环境。--check-env 只检查依赖和登录，不启动相机，不安装软件。--fetch 只用于此单文件版本已有会话，回传不要求 ROS 环境仍有效。板端先完成全量消息反序列化/同步/深度校验，校验报告绑定 bag SHA；主机只核对完整 SHA 和报告并落盘，全部通过才删除板端对应 rgbd.bag，日志保留。rsync 使用 checksum 修复同大小同mtime损坏副本，缺 rsync 时使用 scp。

验证：source /opt/ros/noetic/setup.bash 后 `/usr/bin/python3 -B evaluation/test_d435i_standalone.py -v` 最终17/17通过，原 `/usr/bin/python3 -B evaluation/test_rgbd_recorder.py -v` 6/6通过；新脚本/测试 AST 及 git diff --check 通过。新测试包括空目录搬移与 -I -S 无 site-packages 帮助、主机无 ROS 的真实bag SHA校验、真实LZ4 good/bad/truncated/active、报告缺失/伪配对与失败不删除/重复fetch、实际本地rsync损坏修复、ExactTime严格配对后每3对抽1和CameraInfo、生成XML/bash、替换进程/socket的板端监督最终全读路径。没有SSH到板卡或启动相机/ROS master。读写分离审查纠正了生成launch复制自身、fetch环境耦合和损坏副本重试问题；未改原算法/既有部署。

状态passing仅指单文件交付与离线验证。TODO：新机器上的依赖/USB权限、Python抽帧节点实际30→10FPS吞吐、实录及实际进程清理尚未验证，不能继承旧C++节点的性能结果。原 supervisor 的强杀 roslaunch 后节点独立进程组残留边界未重构。未运行核心/ROS构建，因为没有C++/现有ROS包更改；未连接新目标，因为本次要求是生成可搬移脚本，未提供实际目标。下一步在目标基础环境具备时用 --check-env，再 --duration 10 做实录。

改动：新增独立脚本和 evaluation/test_d435i_standalone.py，更新README、repo-map、录制说明、features、validation和本交接；原shell入口/共享录制程序字节未变。任务前八文件备份、SHA清单、17/6测试日志和VALIDATION.json保存在 artifacts/d435i_standalone_20261028/；前两次测试夹具问题（None进程对象误处理、沙箱socket限制）的失败日志保留，最终测试已修正夹具并全部通过。既有未提交文档改动保留，无待执行部署/录包。


## REC05 D435i USB 本机录包（2026-10-28）

用户要求另生成相机直接插主机、本机采集/抽帧/录包的脚本。已新增根目录 record_d435i_10fps_local.py，单文件内置 ROS1 相机 launch、ExactTime RGB/对齐深度每3对取1、录制 supervisor 和完整 bag 校验。主机需 Linux、Python3.8+、ROS1 Noetic、Python ROS依赖和 RealSense ROS1 驱动及 USB 权限。默认640×480@30采集，直接录制10FPS产品与CameraInfo/TF，关闭IMU/红外/点云。原远程脚本和共享程序保留，未改算法或部署。

命令为 python3 record_d435i_10fps_local.py --check-env / --duration 10，省略duration时按Enter/Ctrl+C停止。脚本自动source本机--ros-setup，可通过--camera-setup指定自编译驱动工作空间；bootstrap保存参数并清空source的位置参数。使用系统Python重进，本机ROS master独立端口/日志与localhost通信。默认生成当前目录/bags/d435i10_local_<时间>_<标识>/，可--output更改，--reserve-gb默认1GiB，--extra-topic可追加。预热和稳定检查后计时；先封包再停相机/master，最终完整读包/图像时间戳与同步/深度检查并保存SHA256SUMS、bag_check.json和session_result.json。已有bag/.active和日志均保留，不执行文件传输或删除。生成的启动脚本通过bash读取，以支持noexec数据盘。

验证：source /opt/ros/noetic/setup.bash 后 /usr/bin/python3 -B evaluation/test_d435i_local.py -v 共17项全部通过。包括真实LZ4有效/缺深度/零深度/错误时间/错误负载/不同步/截断/active、Ctrl+C/Enter/定时、预检失败不录制、磁盘不足不启动进程、清理异常仍处理master、失败保留文件和最终SHA、ExactTime的1/4/7对和CameraInfo，以及带空格setup/参数的实际bash bootstrap和只读自检。过程夹具替换相机/录制进程和socket，没有打开相机、启动ROS master或网络连接。空目录中-I -S帮助及AST、git diff --check通过。旧远程和共享程序/测试与任务前逐字一致，未重复旧套件或运行核心/ROS构建。

状态passing仅指单文件交付和离线验证。TODO：本机USB相机实际出图、Python30→10FPS吞吐、实录及实际退出尚未验证。抽帧器严格按相同时间戳选对，最终基础校验仍沿用95%配对率阈值，不能宣称bag零丢帧或几何/SLAM精度通过。原进程组强杀下的roslaunch独立子节点边界未重构。下一步在连接D435i的ROS1主机执行--check-env，再--duration 10实录。没有待执行录制/部署。

本次新增record_d435i_10fps_local.py和evaluation/test_d435i_local.py，更新README、repo-map、录制说明、features、validation及本交接。任务前十文件备份和SHA清单位于artifacts/d435i_local_20261028/before及BACKUP.json，测试记录local_tests.log和VALIDATION.json同目录；保留既有未提交改动。
