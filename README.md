# ORB-SLAM3_DBoW3 (Modified Version)

本项目在原有的 ORB-SLAM3 基础上进行了多项定制化修改，主要针对工程实用性、性能监控、快速启动与测试流程进行了优化。

## 本项目主要修改内容 (Modifications)

### 1. 词袋库替换 (DBoW3 Support)
* 将原本项目内聚的 DBoW2 替换与迁移，支持使用更高性能的 DBoW3 词典文件 (`Vocabulary/orbvoc.dbow3`)，提升了加载速度和词汇树处理的灵活性。

### 2. 纯定位模式 (Localization) 启动优化
* **快速全局重定位**：优化了 ORB-SLAM3 在纯定位模式下的启动追踪逻辑。消除了在纯定位加载地图时默认“盲目”从上次地图最后一帧起始追踪引发的长时间丢失问题。强制改变系统逻辑使其在启动初始瞬间直接强制进行全局重定位 (Global Relocalization)，显著提升了任意位置冷启动时的快速鲁棒恢复能力。

### 3. 一键运行与综合测试脚本 (`run_stereo_inertial.sh`)
* **多功能模式**：可以通过命令行快速切换 `mapping`（正常建图）、`mapping_save`（建图后保存）以及 `localization`（加载地图纯定位）模式。
* **多传感器解耦选项**：自动适配了不同的实验设备场景。可以在启动时直接传入传感器参数（`rgbd`, `stereo`, `stereo_inertial`），系统将自动加载对应该设定的 `.yaml` 文件或读取指定的默认配置（如 Realsense D435i 配置文件）。
* **自动化日志归档 (`--log`)**：增加了日志选项，自动同步接管所有 `stdout` 与 `stderr`，自动将其备份到 `./logs/` 文件夹中并带入时间戳，且不影响前台屏幕检视。
* **节点资源消耗监控**：每次运行程序时，自动在后台利用 `pidstat` 挂载实时资源消耗监控程序（评估 CPU 与系统内存占用），并在退出时自动终止，生成分析记录到 `./logs/` 目录下。
* **自动 RosBag 录制功能**：与节点的启动和退出完全同步。根据实际所选相机的不同类型（如 RGBD 时录制 color + depth_aligned；Stereo_Inertial 时录制 infra1 + infra2 + imu），在后台静默录制当前运行时的话题。进程优雅退出并保存至 `./bags/`。

### 4. 重构了 ROS 通信接口层
* 取消了死板的默认话题代码绑定。适配了真实世界相机和传感器的深度/红外流特征（比如 `/camera/infra1/image_rect_raw`），并在 ROS 处理节点的 `SyncWithImu()` 中加入了优雅的主动 Shutdown 钩子，从源头避免由于 Ctrl+C 引发的宕机或者进程成僵尸。
* **Stereo ROS wrapper 新增实时位姿输出**：`Stereo` 节点会发布 `/robot_pose`（`geometry_msgs/PoseStamped`），`header.frame_id = "map"`，位姿直接复用 `TrackStereo()` 返回的 `Tcw.inverse()`。
* **Stereo ROS wrapper 新增里程计输出**：`Stereo` 节点会发布 `/odometry`（`nav_msgs/Odometry`），`header.frame_id = "map"`、`child_frame_id = "left_camera"`。其中 pose 同样来自 `Tcw.inverse()`，而 `twist` 使用连续有效跟踪位姿做差分估计，不再长期填零，并通过较大的 covariance 显式表达低置信度。
* **定位模式新增重定位状态输出**：当 `Stereo` 节点运行在 localization 模式时，会以约 `10 Hz` 发布 `/relocalization_status`（`ORB_SLAM3/RelocalizationStatus`）。该消息仅包含 `timestamp_ns` 与 `status` 两个字段，状态字符串严格为 `RelocalizationRunning`、`RelocalizationSucceed`、`RelocalizationFailed`。

### 5. 编译与运行指南 (Build & Run Instructions)

#### 编译项目 (Build)
1. **编译核心库与第三方库 (DBoW3, g2o)**
```bash
cd ORB_SLAM3_DBoW3
chmod +x build.sh
./build.sh
```
2. **编译 ROS 节点**
由于 ROS 目录更名为 `Examples_old/ROS`，您需要将其所在的路径添加到您的 `ROS_PACKAGE_PATH` 中（您可以将其加入您的 `~/.bashrc`）：
```bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/您的绝对路径/ORB_SLAM3_DBoW3/Examples_old/ROS
chmod +x build_ros.sh
./build_ros.sh
```
该步骤会同时生成并编译 `ORB_SLAM3/RelocalizationStatus.msg`。

#### 运行与测试 (Run)
本项目推荐直接使用修改后的 `run_stereo_inertial.sh` 脚本进行日常启动，它封装好了底层传参并附带扩展功能。

**使用语法：**
```bash
./run_stereo_inertial.sh <模式> <传感器> [--log]
```

* **模式 `<模式>`:** 
  * `mapping`（默认建图）
  * `mapping_save`（建图模式，程序中开启了保存地图逻辑支持）
  * `localization`（纯定位模式，程序会自动加载已有地图 `System.LoadMap`）
* **传感器 `<传感器>`:** 
  * `stereo_inertial`（双目IMU，默认）
  * `rgbd`（RGB-D模式）
  * `stereo`（纯双目模式）
* **选项:**
  * `--log`: 将屏幕标准输出同步写入到 `logs/`，并执行后续的一系列录包 (`rosbag record`) 和监控进程联动。

**运行实例：以双目IMU模式开始建图，同时记录终端日志、系统资源与录制 RosBag 数据：**
```bash
./run_stereo_inertial.sh mapping stereo_inertial --log
```

**运行实例：利用此前建立好的地图，开启 RGB-D 纯定位任务：**
```bash
./run_stereo_inertial.sh localization rgbd
```

#### Stereo ROS 话题补充说明

当使用 `Stereo` 节点时，本仓库当前额外提供以下话题：

* `/robot_pose`
  * 类型：`geometry_msgs/PoseStamped`
  * 语义：实时反馈当前左目相机在 `map` 坐标系下的位姿
* `/odometry`
  * 类型：`nav_msgs/Odometry`
  * 坐标系：`map -> left_camera`
  * 说明：pose 来自 SLAM 跟踪位姿，twist 来自相邻有效位姿差分估计
* `/relocalization_status`
  * 类型：`ORB_SLAM3/RelocalizationStatus`
  * 生效条件：`localization` 模式下的 `Stereo` 节点
  * 发布频率：约 `10 Hz`

---

*以下为官方原版 ORB-SLAM3 的 README 内容*

---

# ORB-SLAM3

### V1.0, December 22th, 2021
**Authors:** Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/).

The [Changelog](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Changelog.md) describes the features of each version.

ORB-SLAM3 is the first real-time SLAM library able to perform **Visual, Visual-Inertial and Multi-Map SLAM** with **monocular, stereo and RGB-D** cameras, using **pin-hole and fisheye** lens models. In all sensor configurations, ORB-SLAM3 is as robust as the best systems available in the literature, and significantly more accurate. 

We provide examples to run ORB-SLAM3 in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) using stereo or monocular, with or without IMU, and in the [TUM-VI dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) using fisheye stereo or monocular, with or without IMU. Videos of some example executions can be found at [ORB-SLAM3 channel](https://www.youtube.com/channel/UCXVt-kXG6T95Z4tVaYlU80Q).

This software is based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) developed by [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)).

<a href="https://youtu.be/HyLNq-98LRo" target="_blank"><img src="https://img.youtube.com/vi/HyLNq-98LRo/0.jpg" 
alt="ORB-SLAM3" width="240" height="180" border="10" /></a>

### Related Publications:

[ORB-SLAM3] Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós, **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**, *IEEE Transactions on Robotics 37(6):1874-1890, Dec. 2021*. **[PDF](https://arxiv.org/abs/2007.11898)**.

[IMU-Initialization] Carlos Campos, J. M. M. Montiel and Juan D. Tardós, **Inertial-Only Optimization for Visual-Inertial Initialization**, *ICRA 2020*. **[PDF](https://arxiv.org/pdf/2003.05766.pdf)**

[ORBSLAM-Atlas] Richard Elvira, J. M. M. Montiel and Juan D. Tardós, **ORBSLAM-Atlas: a robust and accurate multi-map system**, *IROS 2019*. **[PDF](https://arxiv.org/pdf/1908.11585.pdf)**.

[ORBSLAM-VI] Raúl Mur-Artal, and Juan D. Tardós, **Visual-inertial monocular SLAM with map reuse**, IEEE Robotics and Automation Letters, vol. 2 no. 2, pp. 796-803, 2017. **[PDF](https://arxiv.org/pdf/1610.05949.pdf)**. 

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://arxiv.org/pdf/1610.06475.pdf)**.

[Monocular] Raúl Mur-Artal, José M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](https://arxiv.org/pdf/1502.00956.pdf)**.

[DBoW2 Place Recognition] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp. 1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License

ORB-SLAM3 is released under [GPLv3 license](https://github.com/UZ-SLAMLab/ORB_SLAM3/LICENSE). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM3 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

If you use ORB-SLAM3 in an academic work, please cite:
  
    @article{ORBSLAM3_TRO,
      title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
               and Multi-Map {SLAM}},
      author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
              Jos\'e M. M. AND Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics}, 
      volume={37},
      number={6},
      pages={1874-1890},
      year={2021}
     }

# 2. Prerequisites
We have tested the library in **Ubuntu 16.04** and **18.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0 and 4.4.0**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## Python
Required to calculate the alignment of the trajectory with the ground truth. **Required Numpy module**.

* (win) http://www.python.org/downloads/windows
* (deb) `sudo apt install libpython2.7-dev`
* (mac) preinstalled with osx

## ROS (optional)

We provide some examples to process input of a monocular, monocular-inertial, stereo, stereo-inertial or RGB-D camera using ROS. Building these examples is optional. These have been tested with ROS Melodic under Ubuntu 18.04.

# 3. Building ORB-SLAM3 library and examples

Clone the repository:
```
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM3*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM3.so**  at *lib* folder and the executables in *Examples* folder.

# 4. Running ORB-SLAM3 with your camera

Directory `Examples` contains several demo programs and calibration files to run ORB-SLAM3 in all sensor configurations with Intel Realsense cameras T265 and D435i. The steps needed to use your camera are: 

1. Calibrate your camera following `Calibration_Tutorial.pdf` and write your calibration file `your_camera.yaml`

2. Modify one of the provided demos to suit your specific camera model, and build it

3. Connect the camera to your computer using USB3 or the appropriate interface

4. Run ORB-SLAM3. For example, for our D435i camera, we would execute:

```
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D435i.yaml
```

# 5. EuRoC Examples
[EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) was recorded with two pinhole cameras and an inertial sensor. We provide an example script to launch EuRoC sequences in all the sensor configurations.

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Open the script "euroc_examples.sh" in the root of the project. Change **pathDatasetEuroc** variable to point to the directory where the dataset has been uncompressed. 

3. Execute the following script to process all the sequences with all sensor configurations:
```
./euroc_examples
```

## Evaluation
EuRoC provides ground truth for each sequence in the IMU body reference. As pure visual executions report trajectories centered in the left camera, we provide in the "evaluation" folder the transformation of the ground truth to the left camera reference. Visual-inertial trajectories use the ground truth from the dataset.

Execute the following script to process sequences and compute the RMS ATE:
```
./euroc_eval_examples
```

# 6. TUM-VI Examples
[TUM-VI dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) was recorded with two fisheye cameras and an inertial sensor.

1. Download a sequence from https://vision.in.tum.de/data/datasets/visual-inertial-dataset and uncompress it.

2. Open the script "tum_vi_examples.sh" in the root of the project. Change **pathDatasetTUM_VI** variable to point to the directory where the dataset has been uncompressed. 

3. Execute the following script to process all the sequences with all sensor configurations:
```
./tum_vi_examples
```

## Evaluation
In TUM-VI ground truth is only available in the room where all sequences start and end. As a result the error measures the drift at the end of the sequence. 

Execute the following script to process sequences and compute the RMS ATE:
```
./tum_vi_eval_examples
```

# 7. ROS Examples

### Building the nodes for mono, mono-inertial, stereo, stereo-inertial and RGB-D
Tested with ROS Melodic and ubuntu 18.04.

1. Add the path including *Examples/ROS/ORB_SLAM3* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:
  ```
  gedit ~/.bashrc
  ```
and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM3:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
  ```
  
2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### Running Monocular Node
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM3/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.

  ```
  rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

### Running Monocular-Inertial Node
For a monocular input from topic `/camera/image_raw` and an inertial input from topic `/imu`, run node ORB_SLAM3/Mono_Inertial. Setting the optional third argument to true will apply CLAHE equalization to images (Mainly for TUM-VI dataset).

  ```
  rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE [EQUALIZATION]	
  ```

### Running Stereo Node
For a stereo input from topic `/camera/left/image_raw` and `/camera/right/image_raw` run node ORB_SLAM3/Stereo. You will need to provide the vocabulary file and a settings file. For Pinhole camera model, if you **provide rectification matrices** (see Examples/Stereo/EuRoC.yaml example), the node will recitify the images online, **otherwise images must be pre-rectified**. For FishEye camera model, rectification is not required since system works with original images:

  ```
  rosrun ORB_SLAM3 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```

### Running Stereo-Inertial Node
For a stereo input from topics `/camera/left/image_raw` and `/camera/right/image_raw`, and an inertial input from topic `/imu`, run node ORB_SLAM3/Stereo_Inertial. You will need to provide the vocabulary file and a settings file, including rectification matrices if required in a similar way to Stereo case:

  ```
  rosrun ORB_SLAM3 Stereo_Inertial PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION [EQUALIZATION]	
  ```
  
### Running RGB_D Node
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM3/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.

  ```
  rosrun ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

**Running ROS example:** Download a rosbag (e.g. V1_02_medium.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab for a Stereo-Inertial configuration:
  ```
  roscore
  ```
  
  ```
  rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml true
  ```
  
  ```
  rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu
  ```
  
Once ORB-SLAM3 has loaded the vocabulary, press space in the rosbag tab.

**Remark:** For rosbags from TUM-VI dataset, some play issue may appear due to chunk size. One possible solution is to rebag them with the default chunk size, for example:
  ```
  rosrun rosbag fastrebag.py dataset-room1_512_16.bag dataset-room1_512_16_small_chunks.bag
  ```

# 8. Running time analysis
A flag in `include\Config.h` activates time measurements. It is necessary to uncomment the line `#define REGISTER_TIMES` to obtain the time stats of one execution which is shown at the terminal and stored in a text file(`ExecTimeMean.txt`).

# 9. Calibration
You can find a tutorial for visual-inertial calibration and a detailed description of the contents of valid configuration files at  `Calibration_Tutorial.pdf`
