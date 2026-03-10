cmake_minimum_required(VERSION 2.8)
project(DBoW3)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -std=c++11")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(src)

# DBoW3 核心文件
set(SRCS_DBOW3
    src/BowVector.cpp
    src/Database.cpp
    src/DescManip.cpp
    src/FeatureVector.cpp
    src/QueryResults.cpp
    src/ScoringObject.cpp
    src/Vocabulary.cpp
    src/quicklz.c
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_library(DBoW3 SHARED ${SRCS_DBOW3})
target_link_libraries(DBoW3 ${OpenCV_LIBS})
