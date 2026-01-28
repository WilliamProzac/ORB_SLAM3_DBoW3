/**
 * DBoW3 Boost Serialization Support
 *
 * Provides Boost serialization for DBoW3::BowVector and DBoW3::FeatureVector.
 * Include this header wherever Boost serialization of these types is needed.
 */

#ifndef DBOW3_SERIALIZATION_H
#define DBOW3_SERIALIZATION_H

#include "BowVector.h"
#include "FeatureVector.h"
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

namespace boost {
namespace serialization {

// Serialization for DBoW3::BowVector (inherits from std::map<WordId,
// WordValue>)
template <class Archive>
void serialize(Archive &ar, DBoW3::BowVector &bv, const unsigned int version) {
  // Serialize the underlying std::map
  ar &boost::serialization::base_object<
      std::map<DBoW3::WordId, DBoW3::WordValue>>(bv);
}

// Serialization for DBoW3::FeatureVector (inherits from std::map<NodeId,
// std::vector<unsigned int>>)
template <class Archive>
void serialize(Archive &ar, DBoW3::FeatureVector &fv,
               const unsigned int version) {
  // Serialize the underlying std::map
  ar &boost::serialization::base_object<
      std::map<DBoW3::NodeId, std::vector<unsigned int>>>(fv);
}

} // namespace serialization
} // namespace boost

#endif // DBOW3_SERIALIZATION_H
