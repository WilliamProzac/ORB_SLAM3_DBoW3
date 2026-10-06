#include "FeatureDescriptor.h"
#include "MedoidTiming.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace ORB_SLAM3 {
namespace {
thread_local MedoidTiming* medoid_timing = nullptr;

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

MedoidTiming* CurrentMedoidTiming() { return medoid_timing; }
MedoidTimingScope::MedoidTimingScope(MedoidTiming* timing)
    : previous_(medoid_timing) { medoid_timing = timing; }
MedoidTimingScope::~MedoidTimingScope() { medoid_timing = previous_; }

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

int FeatureDescriptorDistanceExact(const cv::Mat &a, const cv::Mat &b) {
  // Retain public validation and all non-ORB32 behavior.
  ValidateDescriptorRow(a);
  ValidateDescriptorRow(b);
  if (!FeatureDescriptorsCompatible(a, b))
    throw std::invalid_argument("incompatible feature descriptor types or widths");
#if defined(__aarch64__)
  if (a.type() == CV_8UC1 && a.cols == 32) {
    const uint8_t *pa = a.ptr<uint8_t>();
    const uint8_t *pb = b.ptr<uint8_t>();
    const uint8x16_t lo = vcntq_u8(veorq_u8(vld1q_u8(pa), vld1q_u8(pb)));
    const uint8x16_t hi = vcntq_u8(veorq_u8(vld1q_u8(pa+16), vld1q_u8(pb+16)));
    // Lane sums <=16, full sum <=256: widening reduction avoids overflow.
    return static_cast<int>(vaddlvq_u8(vaddq_u8(lo, hi)));
  }
#endif
  return a.type() == CV_8UC1 ? HammingDistance(a, b) : ScaledCosineDistance(a, b);
}

cv::Mat FeatureDescriptorMedoid(const std::vector<cv::Mat> &descriptors,
                                MedoidTiming* timing) {
  if (timing) ++timing->calls;
  MedoidTiming* detail = timing && timing->detail_window ? timing : nullptr;
  if (descriptors.empty()) {
    return cv::Mat();
  }
  RgbdMeasurementSpan validation(detail, MedoidPhase::Validation);
  for (const cv::Mat &descriptor : descriptors) {
    if (timing) ++timing->validated_rows;
    ValidateDescriptorRow(descriptor);
    if (!FeatureDescriptorsCompatible(descriptors.front(), descriptor)) {
      throw std::invalid_argument(
          "MapPoint observations contain incompatible descriptors");
    }
  }

  validation.Stop();
  const std::size_t count = descriptors.size();
  if (timing) {
    timing->n_sum += count;
    timing->n_squared_sum += static_cast<std::uint64_t>(count) * count;
    timing->n_max = std::max(timing->n_max, static_cast<std::uint64_t>(count));
    if (descriptors.front().type() == CV_8UC1 && descriptors.front().cols == 32)
      ++timing->orb32_calls;
    else ++timing->other_calls;
  }
  RgbdMeasurementSpan allocation(detail, MedoidPhase::MatrixAllocation);
  std::vector<std::vector<int>> distances(count, std::vector<int>(count, 0));
  allocation.Stop();
  std::uint64_t matrix_capacity = distances.capacity() * sizeof(std::vector<int>);
  if (timing) timing->matrix_outer_capacity_sum += distances.capacity();
  RgbdMeasurementSpan distance_time(detail, MedoidPhase::Distances);
  for (std::size_t i = 0; i < count; ++i) {
    if (timing) {
      timing->matrix_row_capacity_sum += distances[i].capacity();
      matrix_capacity += distances[i].capacity() * sizeof(int);
    }
    for (std::size_t j = i + 1; j < count; ++j) {
      const int distance =
          FeatureDescriptorDistance(descriptors[i], descriptors[j]);
      if (timing) ++timing->pair_distances;
      distances[i][j] = distance;
      distances[j][i] = distance;
    }
  }

  distance_time.Stop();
  if (timing) timing->matrix_capacity_bytes_max =
      std::max(timing->matrix_capacity_bytes_max, matrix_capacity);
  RgbdMeasurementSpan median_time(detail, MedoidPhase::Median);
  int best_median = 0;
  std::size_t best_index = 0;
  bool has_best = false;
  for (std::size_t i = 0; i < count; ++i) {
    std::vector<int> row = distances[i];
    if (timing) {
      ++timing->median_rows;
      timing->median_row_capacity_sum += row.capacity();
    }
    std::sort(row.begin(), row.end());
    const int median = row[(count - 1) / 2];
    if (!has_best || median < best_median) {
      best_median = median;
      best_index = i;
      has_best = true;
    }
  }
  median_time.Stop();
  RgbdMeasurementSpan clone_time(detail, MedoidPhase::Clone);
  cv::Mat representative = descriptors[best_index].clone();
  if (timing) {
    ++timing->cloned_rows;
    timing->clone_bytes += representative.total() * representative.elemSize();
  }
  return representative;
}

} // namespace ORB_SLAM3
