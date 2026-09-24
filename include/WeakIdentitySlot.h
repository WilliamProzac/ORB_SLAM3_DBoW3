#pragma once
#include <cstdint>
#include <memory>

namespace ORB_SLAM3 {
// The identity is only a hint for deduplication against already strongly-owned
// points. Never dereference it: a slot must still lock() before accessing data.
// Expired slots retain their weak owner identity, preserving assigned semantics.
template<class T> class WeakIdentitySlot {
public:
  WeakIdentitySlot() = default;
  WeakIdentitySlot(const std::shared_ptr<T>& owner)
      : weak_(owner), identity_(reinterpret_cast<std::uintptr_t>(owner.get())) {}
  WeakIdentitySlot& operator=(const std::shared_ptr<T>& owner) {
    weak_ = owner;
    identity_ = reinterpret_cast<std::uintptr_t>(owner.get());
    return *this;
  }
  std::shared_ptr<T> lock() const noexcept { return weak_.lock(); }
  bool expired() const noexcept { return weak_.expired(); }
  bool owner_before(const WeakIdentitySlot& other) const noexcept {
    return weak_.owner_before(other.weak_);
  }
  std::uintptr_t identity() const noexcept { return identity_; }
private:
  std::weak_ptr<T> weak_;
  std::uintptr_t identity_ = 0;
};
}
