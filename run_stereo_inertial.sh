#!/bin/bash

# ORB-SLAM3 Stereo Inertial 快捷测试脚本
# 用法: ./run_stereo_inertial.sh [模式]
# 模式:
#   mapping      - 默认建图模式 (使用 MyD435i.yaml)
#   mapping_save - 建图模式并保存地图 (修改yaml中的 System.SaveMap 选项或手动在viewer中保存)
#   localization - 纯定位模式，加载已有地图 (使用 MyD435i_load.yaml)

# 检查当前目录是否是 ORB_SLAM3_DBoW3 根目录
if [ ! -f "Vocabulary/ORBvoc.bin" ]; then
    echo "错误：请在 ORB_SLAM3_DBoW3 根目录下运行此脚本"
    exit 1
fi

MODE=$1

if [ -z "$MODE" ]; then
    echo "请指定运行模式！"
    echo "用法: ./run_stereo_inertial.sh [mapping | mapping_save | localization]"
    exit 1
fi

# 基础路径配置
VOCAB_PATH="./Vocabulary/ORBvoc.bin"
MAPPING_YAML="./Examples_old/ROS/ORB_SLAM3/MyD435i.yaml"
LOCALIZATION_YAML="./Examples_old/ROS/ORB_SLAM3/MyD435i_load.yaml"

echo "=================================================="
echo "启动 ORB-SLAM3 Stereo_Inertial ROS 节点"
echo "模式: $MODE"
echo "=================================================="

case "$MODE" in
    "mapping")
        echo "使用配置文件: $MAPPING_YAML"
        # rosrun ORB_SLAM3 node_name vocabulary_path settings_path do_rectify do_equalization
        rosrun ORB_SLAM3 Stereo_Inertial "$VOCAB_PATH" "$MAPPING_YAML" false false
        ;;
    
    "mapping_save")
        echo "注意：请确保 $MAPPING_YAML 中 System.SaveMap 配置正确，或者在关闭前手动在Viewer中保存"
        rosrun ORB_SLAM3 Stereo_Inertial "$VOCAB_PATH" "$MAPPING_YAML" false false
        ;;
        
    "localization")
        echo "使用配置文件: $LOCALIZATION_YAML"
        echo "确保该YAML中配置了 System.LoadMap 选项"
        rosrun ORB_SLAM3 Stereo_Inertial "$VOCAB_PATH" "$LOCALIZATION_YAML" false false
        ;;
        
    *)
        echo "未知模式: $MODE"
        echo "可用模式: mapping | mapping_save | localization"
        exit 1
        ;;
esac
