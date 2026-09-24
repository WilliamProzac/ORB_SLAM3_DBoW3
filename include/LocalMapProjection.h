#pragma once

#include "MapPointHandle.h"
#include <Eigen/Core>
#include <array>
#include <cstdint>

namespace ORB_SLAM3 {
class Map;

// A single position-lock snapshot. Distances are the raw scale-invariance
// bounds; PredictScale uses max_distance, not the expanded 1.2 bound.
struct LocalPointGeometry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Vector3f normal = Eigen::Vector3f::Zero();
  float min_distance = 0, max_distance = 0;
};

enum class LocalProjectionRejection { None, NonFinite, Depth, Image, Distance, View };
struct LocalProjection {
  bool visible = false;
  // Even a rejected legacy projection writes x/y once image bounds pass.
  float x = -1, y = -1, right = 0, z = 0, distance = 0, view_cos = 0;
  int level = 0;
  LocalProjectionRejection rejection = LocalProjectionRejection::None;
};

struct LocalPointSnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MapPointPtr owner;
  Map* map = nullptr;
  LocalPointGeometry geometry;
  std::array<unsigned char, 32> descriptor{};
  unsigned long id = 0, entry_seen = 0, entry_reference = 0;
  int observations = 0, visible = 0, found = 0;
  bool valid = false;
};

struct LocalProjectedPoint {
  const LocalPointSnapshot* snapshot = nullptr;
  LocalProjection projection;
  bool queried = false;
};
} // namespace ORB_SLAM3
