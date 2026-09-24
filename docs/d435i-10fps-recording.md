# D435i 10 FPS RGB-D 录制

在笔记本仓库根目录执行：

```bash
./record_d435i_10fps_bag.sh
```

脚本连接 `sunrise@192.168.1.11`，在板卡上启动
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
