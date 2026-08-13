#include "FeatureDescriptor.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace ORB_SLAM3 {
namespace {

void ValidateDescriptorRow(const cv::Mat &descriptor) {
  if (descriptor.empty() || descriptor.rows != 1 || descriptor.channels() != 1) {
    throw std::invalid_argument("feature descriptor must be one non-empty row");
  }
  if (descriptor.type() != CV_8UC1 && descriptor.type() != CV_32FC1) {
    throw std::invalid_argument(
        "feature descriptor must use CV_8UC1 or CV_32FC1 storage");
  }
}

int HammingDistance(const cv::Mat &a, const cv::Mat &b) {
  const std::uint8_t *pa = a.ptr<std::uint8_t>();
  const std::uint8_t *pb = b.ptr<std::uint8_t>();
  int distance = 0;
  for (int column = 0; column < a.cols; ++column) {
    const unsigned int value =
        static_cast<unsigned int>(pa[column] ^ pb[column]);
#if defined(__GNUC__) || defined(__clang__)
    distance += __builtin_popcount(value);
#else
    unsigned int bits = value;
    while (bits != 0U) {
      bits &= bits - 1U;
      ++distance;
    }
#endif
  }
  return distance;
}

int ScaledCosineDistance(const cv::Mat &a, const cv::Mat &b) {
  const float *pa = a.ptr<float>();
  const float *pb = b.ptr<float>();
  double dot = 0.0;
  double norm_a = 0.0;
  double norm_b = 0.0;
  for (int column = 0; column < a.cols; ++column) {
    const double av = pa[column];
    const double bv = pb[column];
    if (!std::isfinite(av) || !std::isfinite(bv)) {
      throw std::invalid_argument("float feature descriptor contains NaN or Inf");
    }
    dot += av * bv;
    norm_a += av * av;
    norm_b += bv * bv;
  }
  if (norm_a <= 1e-12 || norm_b <= 1e-12) {
    return 512;
  }
  const double cosine = std::max(
      -1.0, std::min(1.0, dot / std::sqrt(norm_a * norm_b)));
  return static_cast<int>(std::lround(256.0 * (1.0 - cosine)));
}

} // namespace

FeatureDescriptorType GetFeatureDescriptorType(const cv::Mat &descriptor) {
  ValidateDescriptorRow(descriptor);
  return descriptor.type() == CV_8UC1
             ? FeatureDescriptorType::ORB_BINARY
             : FeatureDescriptorType::XFEAT_FLOAT;
}

bool FeatureDescriptorsCompatible(const cv::Mat &a, const cv::Mat &b) {
  if (a.empty() || b.empty() || a.rows != 1 || b.rows != 1 ||
      a.channels() != 1 || b.channels() != 1) {
    return false;
  }
  return a.type() == b.type() && a.cols == b.cols &&
         (a.type() == CV_8UC1 || a.type() == CV_32FC1);
}

int FeatureDescriptorDistance(const cv::Mat &a, const cv::Mat &b) {
  ValidateDescriptorRow(a);
  ValidateDescriptorRow(b);
  if (!FeatureDescriptorsCompatible(a, b)) {
    throw std::invalid_argument("incompatible feature descriptor types or widths");
  }
  if (a.type() == CV_8UC1) {
    return HammingDistance(a, b);
  }
  return ScaledCosineDistance(a, b);
}

cv::Mat FeatureDescriptorMedoid(const std::vector<cv::Mat> &descriptors) {
  if (descriptors.empty()) {
    return cv::Mat();
  }
  for (const cv::Mat &descriptor : descriptors) {
    ValidateDescriptorRow(descriptor);
    if (!FeatureDescriptorsCompatible(descriptors.front(), descriptor)) {
      throw std::invalid_argument(
          "MapPoint observations contain incompatible descriptors");
    }
  }

  const std::size_t count = descriptors.size();
  std::vector<std::vector<int>> distances(count, std::vector<int>(count, 0));
  for (std::size_t i = 0; i < count; ++i) {
    for (std::size_t j = i + 1; j < count; ++j) {
      const int distance =
          FeatureDescriptorDistance(descriptors[i], descriptors[j]);
      distances[i][j] = distance;
      distances[j][i] = distance;
    }
  }

  int best_median = 0;
  std::size_t best_index = 0;
  bool has_best = false;
  for (std::size_t i = 0; i < count; ++i) {
    std::vector<int> row = distances[i];
    std::sort(row.begin(), row.end());
    const int median = row[(count - 1) / 2];
    if (!has_best || median < best_median) {
      best_median = median;
      best_index = i;
      has_best = true;
    }
  }
  return descriptors[best_index].clone();
}

} // namespace ORB_SLAM3
