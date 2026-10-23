cmake_minimum_required(VERSION 3.10)
project(orb_slam3_ros)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(RDK_TARGET_CPU "" CACHE STRING "Explicit ARM CPU passed to -mcpu for board builds")
option(ENABLE_XFEAT_HYBRID "Build the RDK X5 XFeat hybrid stereo provider" OFF)
option(BUILD_ORB_EXTRACTOR_VALIDATION
  "Build deterministic ORB workspace and parallelism validation tools" OFF)
option(ORB_EXTRACTOR_LEGACY_REFERENCE
  "Compile ORBextractor with the pre-optimization allocation/copy paths" OFF)
option(ENABLE_ORB_FINE_TIMING
  "Collect detailed per-level ORBextractor timing counters" OFF)
option(ENABLE_ORB_OUTPUT_HASH
  "Hash ordered ORB keypoints, descriptors, and mono/lapping assembly" OFF)
option(ENABLE_ORB_DIRECT_ISOLATED_BLUR
  "Experimentally blur pyramid ROIs directly with BORDER_ISOLATED" OFF)
set(XFEAT_SOURCE_DIR "" CACHE PATH "Directory containing xfeat_frontend sources")
get_filename_component(RGBD_REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
set(ORB_SLAM3_SOURCE_DIR "${RGBD_REPOSITORY_ROOT}" CACHE PATH
  "Authoritative ORB-SLAM3 core source tree")
set(RDK_BPU_SYSROOT "" CACHE PATH "Minimal RDK BPU development sysroot")
if(RDK_TARGET_CPU)
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mcpu=${RDK_TARGET_CPU}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mcpu=${RDK_TARGET_CPU}")
else()
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
endif()

# Match the validated ARM compiler language mode.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Prefer the camera SDK OpenCV when that workspace is present, but keep the
# package reproducible on a deployed board where only the system OpenCV exists.
set(RDK_CAMERA_OPENCV_DIR "/home/jinsy/camera_ws/src/s316_camera/sdk/3rdparty_libs/opencv_libs/lib/cmake/opencv4")
if(EXISTS "${RDK_CAMERA_OPENCV_DIR}/OpenCVConfig.cmake")
  set(OpenCV_DIR "${RDK_CAMERA_OPENCV_DIR}")
endif()
find_package(OpenCV 4 REQUIRED)   # change OpenCV version to 4 by ywl 2026-01-28
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 4 not found.")
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  message_filters
  roscpp
  rospy
  tf
  tf2
  message_generation
  robot_interfaces
  dynamic_reconfigure
  visualization_msgs
  tf2_geometry_msgs
  tf2_ros
  pcl_conversions
  pcl_ros
)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)  # 添加PCL支持

add_service_files(
  FILES
  SaveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package (
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs cv_bridge image_transport tf sensor_msgs dynamic_reconfigure message_runtime robot_interfaces
  LIBRARIES ${PROJECT_NAME}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${ORB_SLAM3_SOURCE_DIR}
  ${ORB_SLAM3_SOURCE_DIR}/include
  ${ORB_SLAM3_SOURCE_DIR}/include/CameraModels
  ${ORB_SLAM3_SOURCE_DIR}/Thirdparty
  ${ORB_SLAM3_SOURCE_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}  # 添加PCL头文件目录
  ${catkin_INCLUDE_DIRS}
)

if(ENABLE_XFEAT_HYBRID)
  if(NOT XFEAT_SOURCE_DIR OR NOT EXISTS "${XFEAT_SOURCE_DIR}/xfeat_frontend.cpp")
    message(FATAL_ERROR "ENABLE_XFEAT_HYBRID requires XFEAT_SOURCE_DIR")
  endif()
  if(NOT EXISTS "${XFEAT_SOURCE_DIR}/xfeat_stereo_feature_provider.cpp")
    message(FATAL_ERROR "XFEAT_SOURCE_DIR has no F23 stereo feature provider")
  endif()
  find_path(HB_DNN_INCLUDE_DIR hb_dnn.h
    PATHS "${RDK_BPU_SYSROOT}/include/dnn" /usr/include/dnn
    NO_DEFAULT_PATH)
  set(HB_DNN_LIBRARY "${RDK_BPU_SYSROOT}/lib/libdnn.so")
  set(HB_MEM_LIBRARY "${RDK_BPU_SYSROOT}/lib/libhbmem.so.1.0.0")
  if(NOT HB_DNN_INCLUDE_DIR OR NOT EXISTS "${HB_DNN_LIBRARY}" OR
     NOT EXISTS "${HB_MEM_LIBRARY}")
    message(FATAL_ERROR "RDK BPU headers/libraries (hb_dnn, dnn, hbmem) were not found")
  endif()
  include_directories(${XFEAT_SOURCE_DIR} ${HB_DNN_INCLUDE_DIR})
endif()

# PCL库目录设置
link_directories(${PCL_LIBRARY_DIRS})  # 添加PCL库目录
add_definitions(${PCL_DEFINITIONS})    # 添加PCL定义

# 不包含第三方库的CMakeLists.txt，直接链接预编译的库
# include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW3/CMakeLists.txt)
# include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/CMakeLists.txt)

set(ORB_SLAM3_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib" CACHE PATH
    "Isolated candidate library output")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${ORB_SLAM3_LIBRARY_OUTPUT_DIRECTORY})

add_library(${PROJECT_NAME} SHARED
  ${ORB_SLAM3_SOURCE_DIR}/src/System.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/MapScaleObservation.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/MemoryStats.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/Tracking.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/LocalTrackingAttempt.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/LocalMapBudget.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/LocalMapPointBudget.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/LocalMapLifetime.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/LocalMapDiagnostics.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/LocalMapEvents.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/TrackingLocalMap.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/LocalMapping.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/LoopClosing.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/ORBextractor.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/FastGridKernel.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/ORBmatcher.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/FrameDrawer.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/Converter.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/FeatureDescriptor.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/MapPoint.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/KeyFrame.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/Atlas.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/Map.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/MapDrawer.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/Optimizer.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/Frame.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/KeyFrameDatabase.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/Sim3Solver.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/Viewer.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/ImuTypes.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/G2oTypes.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/CameraModels/Pinhole.cpp
  ${ORB_SLAM3_SOURCE_DIR}/src/CameraModels/KannalaBrandt8.cpp
  ${ORB_SLAM3_SOURCE_DIR}/src/OptimizableTypes.cpp
  ${ORB_SLAM3_SOURCE_DIR}/src/MLPnPsolver.cpp
  ${ORB_SLAM3_SOURCE_DIR}/src/GeometricTools.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/TwoViewReconstruction.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/Config.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/Settings.cc
  ${ORB_SLAM3_SOURCE_DIR}/src/Random.cpp
  ${ORB_SLAM3_SOURCE_DIR}/src/Timestamp.cpp
)

# Keep these experiment-only definitions local to ORBextractor.cc.  Changing
# an A/B option then recompiles one translation unit instead of the complete
# ORB-SLAM3 library.
if(ORB_EXTRACTOR_LEGACY_REFERENCE)
  set_property(SOURCE ${ORB_SLAM3_SOURCE_DIR}/src/ORBextractor.cc APPEND PROPERTY
    COMPILE_DEFINITIONS ORB_SLAM3_ORB_LEGACY_REFERENCE=1)
endif()
if(ENABLE_ORB_FINE_TIMING)
  set_property(SOURCE ${ORB_SLAM3_SOURCE_DIR}/src/ORBextractor.cc APPEND PROPERTY
    COMPILE_DEFINITIONS ORB_SLAM3_ORB_FINE_TIMING=1)
endif()
if(ENABLE_ORB_OUTPUT_HASH)
  set_property(SOURCE ${ORB_SLAM3_SOURCE_DIR}/src/ORBextractor.cc APPEND PROPERTY
    COMPILE_DEFINITIONS ORB_SLAM3_ORB_OUTPUT_HASH=1)
endif()
if(ENABLE_ORB_DIRECT_ISOLATED_BLUR)
  set_property(SOURCE ${ORB_SLAM3_SOURCE_DIR}/src/ORBextractor.cc APPEND PROPERTY
    COMPILE_DEFINITIONS ORB_SLAM3_ORB_DIRECT_ISOLATED_BLUR=1)
endif()

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORB_SLAM3_SOURCE_DIR}/Thirdparty/DBoW3/lib/libDBoW3.so
  ${ORB_SLAM3_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lcrypto
)

if(BUILD_ORB_EXTRACTOR_VALIDATION)
  add_executable(orb_blur_equivalence
    ${ORB_SLAM3_SOURCE_DIR}/evaluation/orb_extractor/blur_equivalence.cc)
  target_link_libraries(orb_blur_equivalence ${OpenCV_LIBS})

  add_executable(orb_output_parity
    ${ORB_SLAM3_SOURCE_DIR}/evaluation/orb_extractor/orb_output_parity.cc)
  target_link_libraries(orb_output_parity ${PROJECT_NAME} ${OpenCV_LIBS})

  add_executable(orb_fast_parallel_probe
    ${ORB_SLAM3_SOURCE_DIR}/evaluation/orb_extractor/fast_parallel_probe.cc)
  target_link_libraries(orb_fast_parallel_probe ${OpenCV_LIBS})
endif()


## ROS node
# Other ORB_SLAM3_SOURCE_DIR overrides may not carry this diagnostic test.
if(EXISTS "${ORB_SLAM3_SOURCE_DIR}/evaluation/rgbd_exact/parity.cc")
  add_executable(rgbd_exact_parity EXCLUDE_FROM_ALL
    ${ORB_SLAM3_SOURCE_DIR}/evaluation/rgbd_exact/parity.cc)
  target_link_libraries(rgbd_exact_parity ${PROJECT_NAME} ${OpenCV_LIBS})
endif()

add_executable(ros_stereo 
  src/ros_stereo.cc
  src/common.cc
)
if(ENABLE_XFEAT_HYBRID)
  target_compile_features(ros_stereo PRIVATE cxx_std_17)
  target_sources(ros_stereo PRIVATE
    ${XFEAT_SOURCE_DIR}/xfeat_frontend.cpp
    ${XFEAT_SOURCE_DIR}/xfeat_hybrid_depth_provider.cpp
    ${XFEAT_SOURCE_DIR}/xfeat_stereo_feature_provider.cpp
  )
  target_compile_definitions(ros_stereo PRIVATE ORB_SLAM3_WITH_XFEAT=1)
endif()
target_link_libraries(ros_stereo
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
if(ENABLE_XFEAT_HYBRID)
  target_link_libraries(ros_stereo ${HB_DNN_LIBRARY} ${HB_MEM_LIBRARY})
  # The minimal cross sysroot intentionally contains only the public DNN and
  # memory libraries. Their HBRT/CNN dependencies are supplied by the board.
  set_property(TARGET ros_stereo APPEND_STRING PROPERTY LINK_FLAGS
    " -Wl,--allow-shlib-undefined")
endif()

add_executable(ros_rgbd
  src/ros_rgbd.cc
  src/common.cc
)
target_link_libraries(ros_rgbd
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(ros_rgbd_inertial
  src/ros_rgbd_inertial.cc
  src/common.cc
)
target_link_libraries(ros_rgbd_inertial
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(occupancy_grid_publisher_node
  src/occupancy_grid_publisher_node.cpp 
  src/local_map.cpp
)

target_link_libraries(occupancy_grid_publisher_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}  # 添加PCL库链接
)

option(BUILD_MEMORY_VALIDATION "Build isolated RGB-D memory validation tools" OFF)
if(BUILD_MEMORY_VALIDATION)
  set(RGBD_V2_CORE_TARGET ${PROJECT_NAME} ${catkin_LIBRARIES})
  include(${CMAKE_CURRENT_SOURCE_DIR}/../../memory/tools/rgbd_v2_targets.cmake)
  add_executable(rgbd_v2_cancel_probe ../../memory/tools/rgbd_v2_cancel_probe.cc)
  target_compile_definitions(rgbd_v2_cancel_probe PRIVATE RGBD_V2_CANCEL_WITH_CORE)
  target_link_libraries(rgbd_v2_cancel_probe ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_executable(local_map_budget_probe ../../memory/tools/local_map_budget_probe.cc)
  target_link_libraries(local_map_budget_probe ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_executable(memory_probe ../../memory/tools/probe.cc)
  target_link_libraries(memory_probe ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_executable(memory_replay ../../memory/tools/replay.cc)
  target_link_libraries(memory_replay ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_executable(tum_localmap_replay ../../memory/tools/tum_localmap_replay.cc)
  target_link_libraries(tum_localmap_replay ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_executable(memory_slot_probe ../../memory/tools/slot_semantics_probe.cc)
  target_compile_definitions(memory_slot_probe PRIVATE SLOT_SEMANTICS_API)
  target_link_libraries(memory_slot_probe ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../memory/tools/check_archive.cc")
  add_executable(memory_archive_check EXCLUDE_FROM_ALL ../../memory/tools/check_archive.cc)
  target_link_libraries(memory_archive_check ${PROJECT_NAME} ${OpenCV_LIBS})
endif()


# Fresh catkin builds must generate service headers before compiling the nodes.
foreach(rgbd_target ros_rgbd ros_rgbd_inertial ros_stereo occupancy_grid_publisher_node)
  add_dependencies(${rgbd_target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endforeach()

option(BUILD_RGBD_QUERY_PROBES "Build method-level MapPoint and geometry probes" OFF)
if(BUILD_RGBD_QUERY_PROBES)
  find_package(Threads REQUIRED)
  add_executable(rgbd_mappoint_probe ../../tests/map_point_probe.cc)
  target_compile_definitions(rgbd_mappoint_probe PRIVATE ATOMIC_CANDIDATE=1)
  target_link_libraries(rgbd_mappoint_probe ${PROJECT_NAME} ${OpenCV_LIBS} Threads::Threads)
  add_executable(rgbd_geometry_probe ../../tests/geometry_probe.cc)
  target_link_libraries(rgbd_geometry_probe ${PROJECT_NAME} ${OpenCV_LIBS} Threads::Threads)
endif()
