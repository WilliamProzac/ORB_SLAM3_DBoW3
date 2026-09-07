#include "FeatureDescriptor.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

void Require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

cv::Mat FloatRow(float x, float y) {
  cv::Mat descriptor(1, 2, CV_32F);
  descriptor.at<float>(0, 0) = x;
  descriptor.at<float>(0, 1) = y;
  return descriptor;
}

} // namespace

int main() {
  using ORB_SLAM3::FeatureDescriptorDistance;
  using ORB_SLAM3::FeatureDescriptorMedoid;

  cv::Mat orb_a = cv::Mat::zeros(1, 32, CV_8U);
  cv::Mat orb_b = orb_a.clone();
  orb_b.at<unsigned char>(0, 0) = 0xff;
  Require(FeatureDescriptorDistance(orb_a, orb_a) == 0,
          "identical ORB distance must be zero");
  Require(FeatureDescriptorDistance(orb_a, orb_b) == 8,
          "ORB distance must remain exact Hamming distance");

  const cv::Mat x_axis = FloatRow(1.0F, 0.0F);
  const cv::Mat y_axis = FloatRow(0.0F, 1.0F);
  const cv::Mat opposite = FloatRow(-1.0F, 0.0F);
  Require(FeatureDescriptorDistance(x_axis, x_axis) == 0,
          "identical float distance must be zero");
  Require(FeatureDescriptorDistance(x_axis, y_axis) == 256,
          "orthogonal float descriptors must have scaled distance 256");
  Require(FeatureDescriptorDistance(x_axis, opposite) == 512,
          "opposite float descriptors must have scaled distance 512");

  const float cosine = 0.82F;
  const cv::Mat gate = FloatRow(cosine, std::sqrt(1.0F - cosine * cosine));
  Require(std::abs(FeatureDescriptorDistance(x_axis, gate) - 46) <= 1,
          "F21 cosine gate must map close to the legacy low threshold");

  const cv::Mat near_a = FloatRow(0.98F, 0.20F);
  const cv::Mat near_b = FloatRow(0.96F, 0.28F);
  const cv::Mat medoid = FeatureDescriptorMedoid({x_axis, near_a, near_b});
  Require(cv::norm(medoid, near_a, cv::NORM_L2) < 1e-6,
          "float MapPoint medoid must select the central observation");

  bool rejected_mixed = false;
  try {
    (void)FeatureDescriptorDistance(orb_a, x_axis);
  } catch (const std::invalid_argument &) {
    rejected_mixed = true;
  }
  Require(rejected_mixed, "mixed ORB/XFeat descriptors must be rejected");

  std::cout << "feature descriptor checks passed" << std::endl;
  return 0;
}
