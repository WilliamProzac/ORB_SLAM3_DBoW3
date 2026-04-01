#!/bin/bash

# ORB-SLAM3 Stereo Inertial 快捷测试脚本
# 用法: ./run_stereo_inertial.sh [模式] [传感器]
# 模式:
#   mapping      - 默认建图模式
#   mapping_save - 建图模式并保存地图
#   localization - 纯定位模式，加载已有地图
# 传感器:
#   stereo_inertial - 双目惯性模式 (默认, 使用 MyD435i.yaml)
#   rgbd            - RGB-D 模式 (使用 RealSense_D435i.yaml)
#   stereo          - 双目模式 (使用 MyD435i_stereo.yaml)

# 检查当前目录是否是 ORB_SLAM3_DBoW3 根目录
if [ ! -f "Vocabulary/ORBvoc.bin" ]; then
  echo "错误：请在 ORB_SLAM3_DBoW3 根目录下运行此脚本"
  exit 1
fi

MODE=$1
SENSOR=$2

if [ -z "$MODE" ]; then
  echo "请指定运行模式！"
  echo "用法: ./run_stereo_inertial.sh [mapping | mapping_save | localization] [stereo_inertial | rgbd | stereo]"
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

echo "=================================================="
echo "启动 ORB-SLAM3 Stereo_Inertial ROS 节点"
echo "模式: $MODE"
echo "=================================================="

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
