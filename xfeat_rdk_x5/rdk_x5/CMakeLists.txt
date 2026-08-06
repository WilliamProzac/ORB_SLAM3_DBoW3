cmake_minimum_required(VERSION 3.16)
project(xfeat_dnn_runner LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)

add_executable(xfeat_dnn_runner dnn_runner.cpp)
target_compile_features(xfeat_dnn_runner PRIVATE cxx_std_17)
target_compile_options(xfeat_dnn_runner PRIVATE -O3 -Wall -Wextra -Wpedantic)
target_include_directories(xfeat_dnn_runner PRIVATE /usr/include/dnn)
target_link_directories(xfeat_dnn_runner PRIVATE /usr/lib /usr/hobot/lib)
target_link_libraries(
  xfeat_dnn_runner PRIVATE dnn /usr/hobot/lib/libhbmem.so.1 pthread)

add_executable(xfeat_ros_benchmark ros_benchmark.cpp xfeat_frontend.cpp)
target_compile_features(xfeat_ros_benchmark PRIVATE cxx_std_17)
target_compile_options(xfeat_ros_benchmark PRIVATE -O3 -Wall -Wextra -Wpedantic)
target_include_directories(xfeat_ros_benchmark PRIVATE
  /usr/include/dnn
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_directories(xfeat_ros_benchmark PRIVATE /usr/lib /usr/hobot/lib)
target_link_libraries(xfeat_ros_benchmark PRIVATE
  dnn
  /usr/hobot/lib/libhbmem.so.1
  pthread
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
