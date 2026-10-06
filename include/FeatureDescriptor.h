#ifndef FEATURE_DESCRIPTOR_H
#define FEATURE_DESCRIPTOR_H

#include <opencv2/core.hpp>

#include <vector>

namespace ORB_SLAM3 {

struct MedoidTiming;

enum class FeatureDescriptorType {
  ORB_BINARY,
  XFEAT_FLOAT,
};

FeatureDescriptorType GetFeatureDescriptorType(const cv::Mat &descriptor);

bool FeatureDescriptorsCompatible(const cv::Mat &a, const cv::Mat &b);

// Returns a distance in the legacy ORB matcher scale. Binary descriptors use
// exact Hamming distance. Float descriptors use 256 * cosine distance, so the
// existing TH_LOW=50 gate corresponds to cosine similarity about 0.805.
int FeatureDescriptorDistance(const cv::Mat &a, const cv::Mat &b);
// Exact ORB32 fast path; validates inputs and preserves generic fallback.
int FeatureDescriptorDistanceExact(const cv::Mat &a, const cv::Mat &b);

// Selects the observed descriptor whose median distance to the other
// observations is smallest. This is the MapPoint representative descriptor
// for both binary ORB and float XFeat observations.
cv::Mat FeatureDescriptorMedoid(const std::vector<cv::Mat> &descriptors,
                                MedoidTiming* timing = nullptr);

} // namespace ORB_SLAM3

#endif // FEATURE_DESCRIPTOR_H
