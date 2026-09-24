# D435i 与 MEM18/perf20 联合运行

## 日常一键启动（实时回传 SLAM 输出）

在本计算机执行：

```bash
cd /home/ywl/Project/ORB_SLAM3_DBoW3
./start_rdk_rgbd_slam.sh
```

自动通过免密 SSH 连接 `sunrise@192.168.1.11`，启动 D435i、10 FPS 配对流
和现有 MEM18/perf20 RGB-D SLAM。SLAM 的 stdout/stderr 以 `[SLAM]` 前缀
实时打印在当前终端，并每 5 秒显示一次跟踪状态。默认持续运行，按 **Ctrl+C**
停止本次启动的进程并等待清理结束；不会结束板上其他用户已有的会话。
启动时发现相机或 SLAM 被占用会拒绝启动。

可选自动停止：`./start_rdk_rgbd_slam.sh --duration 60`，从 SLAM 订阅就绪起计时。
本机不需要 ROS，仅需 Python3、ssh/scp；板端使用既有 ROS Noetic 和程序。
本机日志：`logs/rdk_rgbd/live_<时间>_<标识>/terminal.log`，结束后回传板端日志到
同目录 `board/`。板端新会话位于 `/userdata/d435i_slam_20260924/live_<...>/`。
主机每 2 秒发心跳，EOF/停止命令/15 秒未收到心跳触发板端本会话清理。

日常模式关闭逐帧 timing、benchmark 和内存明细，不保留无限增长的 Python
逐帧统计列表；默认不录 bag、不自动保存地图、不启动图形窗口。仅回传文本日志，
不是远程图像/地图显示。ROS master 绑定板端 localhost，非本机 ROS 可视化入口。
不修改 Swap、governor 或既有部署。内存优化版长期实时性仍受既有报告限制。

测试：`/usr/bin/python3 evaluation/test_rdk_rgbd_live.py -v`。

## 有限时长联合验证（详细采样）

在笔记本仓库根目录执行（相机接在 `sunrise@192.168.1.11`）：

```bash
./run_d435i_slam.sh 120 off
./run_d435i_slam.sh 60 on
```

第一个参数是正式测量秒数（10–1800），第二个参数为 `off` 或 `on`。
默认 120 秒、timing 关闭。该入口是有限时长验证工具，不是后台常驻服务。
正常结束后停止本次相机、SLAM 和独立 ROS master，并回传结果。不会录制 bag
或自动保存 Atlas；不会改动已有部署、Swap、调频、内核或系统服务。

## 适配内容

- 固定使用 `/userdata/rgbd_memory_lifecycle_8g_20260923/mem18_perf20` 的配套
  `ros_rgbd` / `liborb_slam3_ros.so`，运行前后核对 SHA256、检查 file/ldd，
  检查实际进程加载的 core 路径。不从其他版本借用节点或 core。
- 相机仍用 `/userdata/ros1_realsense_ws/start_rgbd10.sh`：30 FPS 采集，
  严格时间戳配对后每三对发布一对，SLAM 直接订阅：
  `/camera_10fps/color/image_raw` 和
  `/camera_10fps/aligned_depth_to_color/image_raw`。
- 每次从实时 `/camera_10fps/color/camera_info` 生成独立 `d435i.yaml`，
  并核对对齐深度的内参和 frame_id。仅接受 640×480、plumb_bob、4/5 项畸变。
  使用 `PinHole` 模型和实际 K/D，非零畸变不会被当成已校正图像忽略。
- RGB 为 rgb8，`Camera.RGB=1`；`Camera.fps=10`。
  板端 RealSense ROS1 的 `fix_depth_scale` 将深度规范为毫米，
  `16UC1` 对应 `RGBD.DepthMapFactor=1000.0`。
- `Stereo.b=0.05` 是 D435 标称基线，用于 RGB-D 近点阈值（2 m）；
  不声称该数值是本次实测的双目标定。几何深度来自已对齐的深度图。
- 保持 700 features、5 层、3 个 ORB workers、原匹配/初始化阈值、
  rgb/depth queue=8、exact sync queue=10；关闭 Pangolin。

## 产物与验收范围

板端每轮新建 `/userdata/d435i_slam_20260924/<run>/`，主机归档到
`artifacts/d435i_slam_20260924/<run>/`。保留 YAML、标定 JSON、单帧场景图、
启动命令、node/camera 日志、队列 1000 的独立输入序号 CSV、跟踪/位姿 CSV、
RSS/PSS/VmSwap/major fault/系统换页采样、benchmark 和结果 JSON。
timing-on 额外保存逐帧耗时及 Local BA。回传后逐文件验证 SHA256。

输入完整性按就绪后 2 秒起的固定测量窗口比较独立旁路与 SLAM 序号，
不会以“最后一个已处理帧”截断预期输入来隐藏尾部丢帧。耗时直方图和 non-OK
覆盖全部 SLAM 回调（包含启动），off 分位数是 1 ms bin 中点近似，max 精确。
全机 pswpin/pswpout 与 SLAM VmSwap 分开解释。

`static_integration_ok` 联合检查输入帧率/完整性、初始化及跟踪、地图重置、
有限位姿、正常退出、清理和二进制一致性。严格 100 ms 回调门独立报告；
timing-on 绝对耗时不作为生产验收值。当前任务仅验证静止场景，不证明运动、
闭环、定位精度或长期地图增长性能。

当前 ROS wrapper 的 odometry child frame 仍沿用旧 `camera_infra1_frame`
命名；本次验证 RGB-D 跟踪及原始 SLAM 位姿，不宣称完成导航 TF/机器人外参接入。

离线回归：

```bash
bash -n run_d435i_slam.sh
/usr/bin/python3 evaluation/test_d435i_slam.py -v
```
