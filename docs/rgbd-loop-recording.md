# 板卡 RGB-D 闭环录制

在笔记本的仓库根目录执行：

```bash
./record_rgbd_loop.sh
```

默认连接 `root@192.168.1.10`。笔记本需要 ROS Noetic、Python 3 的
`rosbag`/`numpy`、`ssh`、`rsync`，以及现有的 SSH 密钥登录；脚本采用
`BatchMode=yes`，不会保存密码。板卡需要已安装的 S316 驱动、ROS Noetic、
`rsync` 及初始化权限。当前笔记本和板卡已完成一次实际短录验证。

看到 **“已开始录制”** 后再开始走闭环路线。回到起点、恢复相近朝向并保持
几秒，然后在这个终端按 **Enter 或 Ctrl+C**。脚本会等待录制器写完索引，
再关闭相机和本次启动的 ROS master，回传文件；本地校验通过后自动删除
板卡上本次录制的 `rgbd.bag`，最后打印检查结果。
停止后请等到“完成”或错误提示再关闭终端。

## 实际配置

- 初始化：优先查找 `/home/mo/mo_init.sh`，其次 `/home/metoak/mo_init.sh`。
  2026-09-18 实机检查发现 `/home/mo` 不存在，使用的是后者。
  在启动驱动的同一个 shell 内 `source` 初始化脚本，保留 SDK 库路径。
  已加载模块跳过重复 `insmod`，其余初始化错误会阻止驱动启动。
- 相机环境：`/ros1_project/camera_ws/install/setup.bash`；
  启动命令：`roslaunch s316_camera camera.launch`。
- 图像：`/camera/infra1/image_raw`，实测 `640×544 rgb8`。
- 深度：`/camera/depth/image_raw`，实测 `640×544 32FC1`，单位米，
  和左图同时间戳、同 `camera_infra1_frame`。
  沿用当前 S316 驱动的 RGB-D 输入，话题名虽然带 `infra1`，消息编码为 `rgb8`。
- 同时订阅录制 `/camera/imu/data`、`/tf`、`/tf_static` 和两路图像对应的
  `/camera/infra1/camera_info`、`/camera/depth/camera_info`。
  可选话题未发布不会阻塞；当前驱动没有发布 CameraInfo，脚本会一并保存
  `x5_camera.cfg` 和 `camera.launch` 快照，供后续核对内参和深度单位。
- 录制数据写到板卡磁盘，采用 LZ4 无损压缩，停止后才通过 SSH 回传。
  使用通过回环地址通信的独立 ROS master，不要求两台设备配置 ROS 网络互通。
- 拒绝同时运行第二个录制会话，发现现有 `s316_camera_node` 时会退出，
  避免重复打开相机。结束时仅管理本次启动的进程。

默认位置：

```text
板卡：/userdata/rgbd_bags/rgbd_loop_<时间>_<标识>/rgbd.bag
笔记本：<仓库>/bags/rgbd_loop_<时间>_<标识>/rgbd.bag
```

每个目录还包含启动/录制日志、`stream_check.json`、`board_result.json`、
`config.json` 和配置快照。本地增加 `bag_check.json`。
本地 SHA-256、完整读取和 RGB-D 检查全部通过、录制流程成功后，自动删除
板卡对应的 `rgbd.bag`，节约空间；板卡上的小体积日志和配置继续保留。
回传或校验失败时保留原包，方便重试。删除前会再次核对板卡 bag 的哈希，
确认对应本次已验证的本地文件；不会删除 `.active` 或其他会话的 bag。
默认保留 1 GiB，空间接近阈值会停止录制并尝试回传，返回异常状态而不会
静默宣称成功。当前短录约 128 MiB/6 秒，采集时请留意终端的空间提示。

## 检查内容

录制前：等待图像启动后采样约 5 秒，要求两路至少 10 帧、接收帧率及
时间戳帧率至少 3 Hz、正且严格递增的时间戳、有效的 Image 编码及数据长度、
相同尺寸和 frame_id。默认 5 ms 内一对一配对，配对率至少 95%；
抽样深度中有限正值的比例均值至少 1%。未通过则关闭本次相机并返回错误。

回传后：比对板卡与本地 SHA-256，使用 ROS `rosbag` 完整读取并反序列化
每条消息，比对索引数量，再检查整个 RGB-D 序列的消息结构、时间戳、配对率
和抽样深度值。未封包 `.active`、缺索引/截断、缺少 RGB 或深度以及数据异常
都返回非零退出码。不会自动 reindex 或修改原始 bag。

这些检查覆盖文件完整性和基础数据可用性；实际闭环是否被 SLAM 检出，需要
之后回放评估。相同尺寸/frame_id 也不能替代几何标定验证。

## 可选用法

```bash
# 改板卡地址、初始化位置和笔记本保存目录
./record_rgbd_loop.sh --board root@192.168.1.10 \
  --init-script /home/metoak/mo_init.sh --output /path/to/local/bags

# 正式开始录制后 10 秒自动停止，用于设备检查
./record_rgbd_loop.sh --duration 10

# 回传/清理中断后重试；校验通过会清理板卡 bag，不会再次打开相机
./record_rgbd_loop.sh --fetch rgbd_loop_20260918_145800_5dc9d3

# 添加一个额外录制话题，可多次传入
./record_rgbd_loop.sh --extra-topic /your/topic

# 独立检查已有文件，不需要启动 ROS master
source /opt/ros/noetic/setup.bash
/usr/bin/python3 evaluation/rgbd_recorder.py check /path/to/rgbd.bag \
  --report /tmp/bag_check.json

# 离线验证正常/异常 bag 检查逻辑
/usr/bin/python3 evaluation/test_rgbd_recorder.py
```

`--fetch` 如果使用过非默认的 `--board`、`--remote-root` 或 `--output`，
重试时也应传入它们。清理完成后，bag 只保存在笔记本上，无法从板卡重新
下载到其他目录；原本地目录仍支持重复校验。断开 SSH 控制连接时，板卡会尽快封包并关闭相机；
网络恢复后用 `--fetch` 拉回。如果电脑进程卡住，20 秒收不到心跳也会停止。
初始化失败等早期错误可能没有生成 bag，此时检查日志中的具体原因。

全部选项见 `./record_rgbd_loop.sh --help`。当前默认配置针对已验证的 S316。
