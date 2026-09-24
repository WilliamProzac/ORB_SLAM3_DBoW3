#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace ORB_SLAM3 {
// Non-owning membership hints for points retained in Tracking's output vector.
// A collision evicts a hint: the caller then weak-locks and checks the original
// frame marker. False negatives cost time; exact identity checks prevent false
// positives. Size is fixed regardless of map age, with no per-frame allocation.
class RetainedPointCache {
public:
  void clear() noexcept { identities_.fill(0); accepted_ = 0; }
  bool count(std::uintptr_t identity) const noexcept {
    return identity && identities_[index(identity)] == identity;
  }
  void insert(std::uintptr_t identity) noexcept {
    identities_[index(identity)] = identity;
    ++accepted_;
  }
  // Counts accepted strong owners, not occupied cache slots.
  std::size_t accepted_count() const noexcept { return accepted_; }
private:
  static std::size_t index(std::uintptr_t identity) noexcept {
    const auto value = identity >> 4;
    return (value ^ (value >> 15) ^ (value >> 30)) & (kSlots - 1);
  }
  static constexpr std::size_t kSlots = 32768;
  std::array<std::uintptr_t, kSlots> identities_{};
  std::size_t accepted_ = 0;
};
}
