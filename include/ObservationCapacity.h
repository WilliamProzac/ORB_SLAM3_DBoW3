#pragma once
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>
namespace ORB_SLAM3 {
// Reserve only when a qualifying observation is about to append an edge.
// Geometric growth is necessary: exact reserve(size+1) would be quadratic.
inline size_t NextObservationCapacity(size_t size, size_t capacity, size_t maximum) {
  if (size >= maximum) throw std::length_error("BA observation capacity exhausted");
  const size_t required = size + 1;
  if (capacity >= required) return capacity;
  return std::max(required, capacity > maximum / 2 ? maximum : capacity * 2);
}
template<class Edge, class KF, class MP>
void ReserveObservation(std::vector<Edge>& edges, std::vector<KF>& keys, std::vector<MP>& points) {
  const size_t maximum = std::min(edges.max_size(), std::min(keys.max_size(), points.max_size()));
  const size_t target = NextObservationCapacity(edges.size(), edges.capacity(), maximum);
  if (edges.capacity() < target) edges.reserve(target);
  if (keys.capacity() < target) keys.reserve(target);
  if (points.capacity() < target) points.reserve(target);
}
}
