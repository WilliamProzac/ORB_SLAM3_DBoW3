#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "CameraModels/Pinhole.h"
#include "Frame.h"
#include "ORBextractor.h"

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "usage: orb_slam3_stereo_dump LEFT.png RIGHT.png OUTPUT.csv\n";
    return 2;
  }

  const cv::Mat left = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  const cv::Mat right = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
  if (left.empty() || right.empty() || left.size() != right.size()) {
    std::cerr << "input images are missing or have different sizes\n";
    return 3;
  }

  // Production S316.yaml parameters used by the board baseline.
  constexpr int features = 600;
  constexpr float scale_factor = 1.2F;
  constexpr int levels = 5;
  constexpr int initial_fast = 20;
  constexpr int minimum_fast = 7;
  constexpr float fx = 293.225983F;
  constexpr float fy = 293.225983F;
  constexpr float cx = 312.983093F;
  constexpr float cy = 274.322723F;
  constexpr float baseline = 0.060087482F;
  constexpr float close_depth = baseline * 40.0F;

  ORB_SLAM3::ORBextractor extractor_left(
      features, scale_factor, levels, initial_fast, minimum_fast);
  ORB_SLAM3::ORBextractor extractor_right(
      features, scale_factor, levels, initial_fast, minimum_fast);
  std::vector<float> camera_parameters{fx, fy, cx, cy};
  ORB_SLAM3::Pinhole camera(camera_parameters);
  cv::Mat calibration =
      (cv::Mat_<float>(3, 3) << fx, 0.0F, cx, 0.0F, fy, cy, 0.0F, 0.0F,
       1.0F);
  cv::Mat distortion = cv::Mat::zeros(4, 1, CV_32F);
  const float bf = fx * baseline;
  ORB_SLAM3::Frame frame(left, right, 0.0, &extractor_left, &extractor_right,
                         nullptr, calibration, distortion, bf, close_depth,
                         &camera);

  std::ofstream output(argv[3], std::ios::out | std::ios::trunc);
  if (!output) {
    std::cerr << "cannot open output: " << argv[3] << '\n';
    return 4;
  }
  output << "index,left_x,left_y,right_x,depth,octave,response,valid_depth,"
            "descriptor_hex\n";
  output << std::fixed << std::setprecision(6);
  for (int index = 0; index < frame.N; ++index) {
    const auto &keypoint = frame.mvKeysUn[index];
    const float right_x = frame.mvuRight[index];
    const float depth = frame.mvDepth[index];
    std::ostringstream descriptor;
    descriptor << std::hex << std::setfill('0');
    if (!frame.mDescriptors.empty() && index < frame.mDescriptors.rows) {
      for (int column = 0; column < frame.mDescriptors.cols; ++column) {
        descriptor << std::setw(2)
                   << static_cast<unsigned int>(
                          frame.mDescriptors.at<unsigned char>(index, column));
      }
    }
    output << index << ',' << keypoint.pt.x << ',' << keypoint.pt.y << ','
           << right_x << ',' << depth << ',' << keypoint.octave << ','
           << keypoint.response << ',' << (depth > 0.0F ? 1 : 0) << ','
           << descriptor.str() << '\n';
  }
  std::cout << "detected=" << frame.N << " valid_depth=";
  int valid = 0;
  for (const float depth : frame.mvDepth) valid += depth > 0.0F;
  std::cout << valid << '\n';
  return 0;
}
