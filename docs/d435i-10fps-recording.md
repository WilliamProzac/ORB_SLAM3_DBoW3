# D435i 10 FPS RGB-D 录制

## 本机单文件版：D435i 直接插在主机 USB

将 [`record_d435i_10fps_local.py`](../record_d435i_10fps_local.py) 复制到连接 D435i 的
Linux 主机，在该主机上执行：

```bash
# 检查本机 ROS1、Python 包和 RealSense 驱动，不打开相机
python3 record_d435i_10fps_local.py --check-env

# 本机录制，按 Enter 或 Ctrl+C 停止
python3 record_d435i_10fps_local.py

# 正式录制 10 秒后自动停止
python3 record_d435i_10fps_local.py --duration 10

# 指定本机保存根目录
python3 record_d435i_10fps_local.py --duration 10 --output /data/d435i_bags
```

该版本在本机完成相机启动、640×480@30 FPS 采集、RGB/对齐深度严格时间戳配对、
每三对保留一对、10 FPS bag 录制和全量校验。代码、相机 launch 和抽帧节点全部内置，
只需复制这一个文件。默认只保存抽帧后的 10 FPS RGB-D、两路 CameraInfo、`/tf` 和
`/tf_static`；相机 IMU、红外图和点云默认关闭。

主机需要 Python 3.8+、ROS1 Noetic、Python ROS 依赖（rosbag/rospy/sensor_msgs/
message_filters/NumPy/roslz4）和 RealSense ROS1 驱动，并具备相机 USB 访问权限。
脚本自动 source `/opt/ros/noetic/setup.bash`；驱动从自编译工作空间安装时，明确指定：

```bash
python3 record_d435i_10fps_local.py \
  --ros-setup /opt/ros/noetic/setup.bash \
  --camera-setup /home/your_user/catkin_ws/devel/setup.bash \
  --duration 10
```

两路图像到达后预热约 2 秒，再检查约 5 秒稳定数据，检查通过才正式录制。
`--duration` 从正式录制开始计时，不含启动、预检、封包和最终读包检查时间。
`--check-env` 检查基础依赖，不验证相机实际出图；实际访问和数据稳定性由录制前预检确认。
用 `--reserve-gb` 设置磁盘保留空间（默认 1 GiB），`--extra-topic /话题` 可增加录制话题。

默认生成目录：

```text
<运行命令时的目录>/bags/d435i10_local_<时间>_<标识>/
  rgbd.bag                 # 本机 10 FPS RGB-D bag，LZ4 压缩
  config.json              # 本次参数
  stream_check.json        # 录制前数据检查
  bag_check.json           # 完整消息读取、RGB-D 检查和 SHA-256
  session_result.json      # 录制/退出/校验整体结果
  SHA256SUMS               # bag 的 SHA-256
  camera.log / record.log / master.log
```

停止时先正常封包，再关闭本次启动的相机、抽帧节点和 ROS master，最后完整读取 bag，
检查消息数量、图像结构、时间戳、帧率、RGB-D 配对和深度有效数据，并保存 SHA-256。
成功或失败均保留已有 bag、`.active` 文件和日志。该版本已通过 17 项离线测试；
TODO：本机 USB 相机实际录制、Python 抽帧吞吐和实际进程退出尚未实测。

## 单文件版：复制到新机器使用

只需复制仓库根目录的 [`record_d435i_10fps_standalone.py`](../record_d435i_10fps_standalone.py)。
在新 Linux 主机的终端运行，将示例用户名和地址换成实际板卡：

```bash
# 检查本机/板卡依赖和 SSH 登录，不启动相机
python3 record_d435i_10fps_standalone.py --board robot@192.168.1.20 --check-env

# 开始录制，按 Enter 或 Ctrl+C 结束
python3 record_d435i_10fps_standalone.py --board robot@192.168.1.20

# 录制 10 秒后自动停止
python3 record_d435i_10fps_standalone.py --board robot@192.168.1.20 --duration 10
```

主机需要 **Python 3.8+、SSH，以及 rsync 或 scp**，无需 ROS、NumPy、
`evaluation/rgbd_recorder.py` 或整个项目。SSH 使用密钥或代理进行非交互登录。
`D435I_BOARD` 环境变量和 SSH 主机别名同样可用；命令行 `--board` 优先，
都未指定时仍连接 `sunrise@192.168.1.11`。

板卡需要 ROS1 Noetic、Python ROS 包（`rosbag`、`rospy`、`sensor_msgs`、
`message_filters`、`roslz4`、`numpy`）、提供 `rs_camera.launch` 的 RealSense ROS1
驱动，以及相机 USB 访问权限。脚本内置相机 launch、Python 严格同步抽帧节点和
完整录制/校验程序，每次自动上传到独立会话目录，不依赖板卡上原有的
`start_rgbd10.sh` 或预编译的 `rgbd_rate_adapter`，也无需编译抽帧节点。
相机保持 640×480、30 FPS 采集，按完全相同的时间戳配对后每三对保留一对。

ROS 或驱动安装位置不同时，可通过参数指定板端环境文件：

```bash
python3 record_d435i_10fps_standalone.py \
  --board robot@192.168.1.20 \
  --ros-setup /opt/ros/noetic/setup.bash \
  --camera-setup /home/robot/catkin_ws/devel/setup.bash
```

默认只 source ROS 安装和已存在的 `/userdata/ros1_realsense_ws/devel/setup.bash`；
其他驱动工作空间使用上面的 `--camera-setup` 明确指定。环境自检会检查文件、
ROS1 Python 依赖和 RealSense 驱动是否可找到，不会安装软件或修改系统配置；
相机是否可打开、数据是否稳定由开始录制前的数据流检查确认。

默认保存位置：

```text
板卡：<SSH 登录用户的家目录>/d435i_10fps_bags/d435i10_<时间>_<标识>/rgbd.bag
主机：<运行命令时的目录>/bags/d435i10_<时间>_<标识>/rgbd.bag
```

可以用 `--remote-root /可写的板端目录` 和 `--output /本地目录` 覆盖。
板端完成封包、全量消息反序列化和 RGB-D 校验，主机回传后再核对 bag 的 SHA-256
与对应校验报告；全部通过并落盘后才删除板端对应 bag，日志和配置保留。
主机和板卡均有 rsync 时使用校验和传输，否则使用 scp。
失败或中断后，使用同一板卡及保存目录重试：

```bash
python3 record_d435i_10fps_standalone.py --board robot@192.168.1.20 \
  --fetch d435i10_YYYYMMDD_HHMMSS_xxxxxx
```

若录制时覆盖了保存路径，重试时也传入同一 `--remote-root` / `--output`。
`--fetch` 用于此单文件版本产生的会话，无需启动 ROS 或相机。
单文件版已完成离线真实 bag、配对、目录搬移和无主机 ROS 的测试；
TODO：内置 Python 抽帧节点在新机器上的实际吞吐和相机录制尚未实测。

## 原仓库包装脚本

在笔记本仓库根目录执行：

```bash
./record_d435i_10fps_bag.sh
```

更换 SSH 登录用户名或地址时，用 `--board 用户名@地址` 指定目标：

```bash
./record_d435i_10fps_bag.sh --board robot@192.168.1.20

# 同样适用于短录和回传重试
./record_d435i_10fps_bag.sh --board robot@192.168.1.20 --duration 10
./record_d435i_10fps_bag.sh --board robot@192.168.1.20 --fetch d435i10_YYYYMMDD_HHMMSS_xxxxxx
```

也可以设置环境变量，后续执行时自动使用该目标：

```bash
export D435I_BOARD=robot@192.168.1.20
./record_d435i_10fps_bag.sh
```

优先级为：`--board` > `D435I_BOARD` > 默认 `sunrise@192.168.1.11`。
`--board` 也接受 SSH 配置中的主机别名。脚本使用 SSH 非交互认证，运行前需配置好目标的密钥或 SSH 代理。

在另一台电脑上使用时，需同时复制本脚本和 `evaluation/rgbd_recorder.py`，保持目录关系；
本机需要 ROS1 Noetic（含 Python `rosbag`、`numpy`）、`ssh` 和 `rsync`。
本机 ROS 环境路径不同时可通过 `LOCAL_ROS_SETUP=/path/to/setup.bash` 指定。
更换 SSH 目标后，目标板卡仍需预先具备下面使用的 ROS1 相机工作空间和启动脚本。

脚本默认连接 `sunrise@192.168.1.11`，在所选板卡上启动
`/userdata/ros1_realsense_ws/start_rgbd10.sh`。D435i 以
`640×480@30 FPS` 采集彩色和深度，`rgbd_rate_adapter` 只接收时间戳完全
相同的 RGB/对齐深度帧，每三对发布一对。录包只保存以下 10 FPS 产品：

```text
/camera_10fps/color/image_raw
/camera_10fps/color/camera_info
/camera_10fps/aligned_depth_to_color/image_raw
/camera_10fps/aligned_depth_to_color/camera_info
/tf
/tf_static
```

脚本等待两路首次到达后预热 2 秒，再连续检查约 5 秒稳定数据，要求两路接收帧率至少 9.5 Hz、图像结构和
时间戳有效、0.1 ms 内配对率至少 95%、深度有效像素不少于 1%。检查通过
后终端显示“已开始录制”。完成路线后按 **Enter 或 Ctrl+C**，脚本依次：

1. 停止并正常封包 `rosbag record`；
2. 关闭本次启动的 D435i、10 FPS adapter 和独立 ROS master；
3. 将会话目录回传到笔记本；
4. 比对 SHA-256，完整读取并反序列化全部消息，再检查整段 RGB-D；
5. 仅在本地验证全部通过后删除板卡上的对应 `rgbd.bag`，保留日志和配置。

默认路径：

```text
板卡：/userdata/d435i_10fps_bags/d435i10_<时间>_<标识>/rgbd.bag
笔记本：<仓库>/bags/d435i10_<时间>_<标识>/rgbd.bag
```

录制目录还包含相机/ROS/录制日志、流检查、本地 bag 检查、启动脚本快照、
launch 快照和同步降频节点源码快照。录制或回传失败时板卡 bag 会保留。

常用命令：

```bash
# 10 秒自动停止，用于短录检查
./record_d435i_10fps_bag.sh --duration 10

# 指定本地保存目录
./record_d435i_10fps_bag.sh --output /path/to/bags

# 回传或清理中断后重试
./record_d435i_10fps_bag.sh --fetch d435i10_YYYYMMDD_HHMMSS_xxxxxx

# 增加其他话题
./record_d435i_10fps_bag.sh --extra-topic /your/topic
```

脚本拒绝在 RealSense ROS1/ROS2 驱动或 10 FPS adapter 已运行时再次打开
相机。当前 launch 关闭 IMU；如后续启用 RGB-D+IMU，需要先配置 IIO 权限，
再显式增加 IMU 话题。所有选项见 `./record_d435i_10fps_bag.sh --help`。
