#!/bin/bash

# ORB-SLAM3 Stereo Inertial 快捷测试脚本
# 用法: ./run_stereo_inertial.sh [模式] [传感器] [--log]
# 模式:
#   mapping      - 默认建图模式
#   mapping_save - 建图模式并保存地图
#   localization - 纯定位模式，加载已有地图
# 传感器:
#   stereo_inertial - 双目惯性模式 (默认, 使用 MyD435i.yaml)
#   rgbd            - RGB-D 模式 (使用 RealSense_D435i.yaml)
#   stereo          - 双目模式 (使用 MyD435i_stereo.yaml)
# 选项:
#   --log           - 将所有输出同时保存到 logs/ 目录下的日志文件
#   --no-bag        - 禁用自动 rosbag 录制（默认启用）

# 检查当前目录是否是 ORB_SLAM3_DBoW3 根目录
if [ ! -f "Vocabulary/ORBvoc.bin" ]; then
  echo "错误：请在 ORB_SLAM3_DBoW3 根目录下运行此脚本"
  exit 1
fi

# --log 参数可放在任意位置，先过滤出来
SAVE_LOG=false
RECORD_BAG=true
ARGS=()
for arg in "$@"; do
  if [ "$arg" == "--log" ]; then
    SAVE_LOG=true
  elif [ "$arg" == "--no-bag" ]; then
    RECORD_BAG=false
  else
    ARGS+=("$arg")
  fi
done

MODE=${ARGS[0]}
SENSOR=${ARGS[1]}

if [ -z "$MODE" ]; then
  echo "请指定运行模式！"
  echo "用法: ./run_stereo_inertial.sh [mapping | mapping_save | localization] [stereo_inertial | rgbd | stereo] [--log] [--no-bag]"
  exit 1
fi

if [ -z "$SENSOR" ]; then
  echo "未指定传感器模式，默认使用: stereo_inertial"
  SENSOR="stereo_inertial"
fi

# 基础路径配置
#VOCAB_PATH="./Vocabulary/ORBvoc.bin"
VOCAB_PATH="./Vocabulary/orbvoc.dbow3"

if [ "$SENSOR" == "rgbd" ]; then
  MAPPING_YAML="./Examples_old/ROS/ORB_SLAM3/MyD435i_rgbd.yaml"
  LOCALIZATION_YAML="./Examples_old/ROS/ORB_SLAM3/MyD435i_rgbd_load.yaml"
  NODE_NAME="RGBD"
elif [ "$SENSOR" == "stereo" ]; then
  MAPPING_YAML="./Examples_old/ROS/ORB_SLAM3/MyD435i_stereo.yaml"
  LOCALIZATION_YAML="./Examples_old/ROS/ORB_SLAM3/MyD435i_stereo_load.yaml"
  NODE_NAME="Stereo"
elif [ "$SENSOR" == "stereo_inertial" ]; then
  MAPPING_YAML="./Examples_old/ROS/ORB_SLAM3/MyD435i.yaml"
  LOCALIZATION_YAML="./Examples_old/ROS/ORB_SLAM3/MyD435i_load.yaml"
  NODE_NAME="Stereo_Inertial"
else
  echo "未知的传感器类型: $SENSOR"
  echo "可用传感器: stereo_inertial | rgbd | stereo"
  exit 1
fi

# 日志配置
if [ "$SAVE_LOG" == "true" ]; then
  mkdir -p ./logs
  LOG_FILE="./logs/${MODE}_${SENSOR}_$(date +%Y%m%d_%H%M%S).log"
  # 将后续所有输出（stdout + stderr）同时写入日志文件和终端
  # 使用 trap '' INT 免疫 Ctrl+C 信号，防止 tee 进程过早退出导致主进程触发 SIGPIPE 异常中断
  exec > >(trap '' INT; tee "$LOG_FILE") 2>&1
  echo "日志已开启，保存路径: $LOG_FILE"
fi

echo "=================================================="
echo "启动 ORB-SLAM3 Stereo_Inertial ROS 节点"
echo "模式: $MODE"
echo "日志: $([ "$SAVE_LOG" == "true" ] && echo "$LOG_FILE" || echo "未开启")"
echo "录制: $([ "$RECORD_BAG" == "true" ] && echo "开启" || echo "禁用")"
echo "=================================================="

# 启动系统资源监控 (使用 pidstat)
mkdir -p ./logs
RES_LOG_FILE="./logs/resource_usage_${MODE}_${SENSOR}_$(date +%Y%m%d_%H%M%S).txt"
echo "资源监控已开启，数据保存路径: $RES_LOG_FILE"
pidstat -u -r -C "$NODE_NAME" 1 > "$RES_LOG_FILE" &
PIDSTAT_PID=$!

# 启动 rosbag 录制（可被 --no-bag 禁用）
mkdir -p ./bags
ROSBAG_FILE="./bags/${MODE}_${SENSOR}_$(date +%Y%m%d_%H%M%S).bag"

if [ "$RECORD_BAG" == "true" ]; then
  if [ "$SENSOR" == "rgbd" ]; then
    RECORD_TOPICS="/camera/color/image_raw /camera/aligned_depth_to_color/image_raw"
  elif [ "$SENSOR" == "stereo" ]; then
    RECORD_TOPICS="/camera/infra1/image_rect_raw /camera/infra2/image_rect_raw"
  elif [ "$SENSOR" == "stereo_inertial" ]; then
    RECORD_TOPICS="/camera/infra1/image_rect_raw /camera/infra2/image_rect_raw /camera/imu"
  fi

  echo "rosbag录制已开启，保存路径: $ROSBAG_FILE"
  echo "录制的话题: $RECORD_TOPICS"
  rosbag record -O "$ROSBAG_FILE" $RECORD_TOPICS &
  ROSBAG_PID=$!
else
  echo "rosbag录制已禁用 (使用 --no-bag)"
  ROSBAG_PID=""
fi

# 当脚本退出时（包括按 Ctrl+C），自动发送 SIGINT 确保 rosbag 正常保存退出，并杀死 pidstat 进程
trap 'if [ "$RECORD_BAG" = "true" ] && [ -n "$ROSBAG_PID" ]; then kill -INT $ROSBAG_PID 2>/dev/null; fi; kill $PIDSTAT_PID 2>/dev/null' EXIT INT TERM

case "$MODE" in
"mapping")
  echo "使用配置文件: $MAPPING_YAML"
  if [ "$SENSOR" == "rgbd" ]; then
    rosrun ORB_SLAM3 $NODE_NAME "$VOCAB_PATH" "$MAPPING_YAML"
  elif [ "$SENSOR" == "stereo" ]; then
    rosrun ORB_SLAM3 $NODE_NAME "$VOCAB_PATH" "$MAPPING_YAML" false
  else
    rosrun ORB_SLAM3 $NODE_NAME "$VOCAB_PATH" "$MAPPING_YAML"
  fi
  ;;

"mapping_save")
  echo "注意：请确保 $MAPPING_YAML 中 System.SaveMap 配置正确，或者在关闭前手动在Viewer中保存"
  if [ "$SENSOR" == "rgbd" ]; then
    rosrun ORB_SLAM3 $NODE_NAME "$VOCAB_PATH" "$MAPPING_YAML" false
  elif [ "$SENSOR" == "stereo" ]; then
    rosrun ORB_SLAM3 $NODE_NAME "$VOCAB_PATH" "$MAPPING_YAML" false false
  else
    rosrun ORB_SLAM3 $NODE_NAME "$VOCAB_PATH" "$MAPPING_YAML" false false
  fi
  ;;

"localization")
  echo "使用配置文件: $LOCALIZATION_YAML"
  echo "确保该YAML中配置了 System.LoadMap 选项"
  if [ "$SENSOR" == "rgbd" ]; then
    rosrun ORB_SLAM3 $NODE_NAME "$VOCAB_PATH" "$LOCALIZATION_YAML"
  elif [ "$SENSOR" == "stereo" ]; then
    rosrun ORB_SLAM3 $NODE_NAME "$VOCAB_PATH" "$LOCALIZATION_YAML" false
  else
    rosrun ORB_SLAM3 $NODE_NAME "$VOCAB_PATH" "$LOCALIZATION_YAML"
  fi
  ;;

*)
  echo "未知模式: $MODE"
  echo "可用模式: mapping | mapping_save | localization"
  exit 1
  ;;
esac
