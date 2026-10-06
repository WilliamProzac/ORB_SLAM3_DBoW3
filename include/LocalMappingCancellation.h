#pragma once

#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <mutex>

namespace ORB_SLAM3 {

// One LocalMapping thread owns refinement attempts. Requests share its existing
// queue mutex; optimizer reads use only the atomic flag. This class does not
// change the queue, stop, reset, finish, or optimization policy.
class LocalMappingCancellation {
public:
  enum class Source : unsigned { NewKeyFrame = 0, Tracking = 1, Stop = 2 };

  struct RequestRecord {
    std::uint64_t request_id = 0;
    std::uint64_t attempt_id = 0; // Zero means no active refinement attempt.
    Source source = Source::Tracking;
    std::uint64_t request_ns = 0;
  };

  struct Snapshot {
    std::uint64_t attempt_id = 0;
    bool active_attempt = false;
    bool requested = false;
    bool began_with_stop = false;
    bool began_with_queued_keyframes = false;
    std::uint64_t request_count = 0;
    std::uint64_t no_active_attempt_count = 0;
    std::uint64_t attempt_request_count = 0;
    std::uint64_t first_request_ns = 0;
    std::array<std::uint64_t, 3> requests_by_source{{0, 0, 0}};
    RequestRecord last_request;
  };

  LocalMappingCancellation(std::atomic<bool>& requested,
                           std::mutex& stop_mutex,
                           std::mutex& queue_mutex)
      : requested_(requested), stop_mutex_(stop_mutex),
        queue_mutex_(queue_mutex) {}

  LocalMappingCancellation(const LocalMappingCancellation&) = delete;
  LocalMappingCancellation& operator=(const LocalMappingCancellation&) = delete;

  // Callers already changing the queue or stop state use this entry point while
  // retaining their existing lock. Never recursively acquire the queue mutex.
  RequestRecord RequestLocked(Source source,
                             const std::unique_lock<std::mutex>& queue_lock,
                             bool record_time = false) {
    assert(queue_lock.owns_lock() && queue_lock.mutex() == &queue_mutex_);
    (void)queue_lock;
    requested_.store(true);
    ++state_.request_count;
    ++state_.requests_by_source[static_cast<unsigned>(source)];
    if (state_.active_attempt)
      ++state_.attempt_request_count;
    else
      ++state_.no_active_attempt_count;
    state_.last_request = {state_.request_count,
                          state_.active_attempt ? state_.attempt_id : 0,
                          source, record_time ? NowNs() : 0};
    if (state_.active_attempt && state_.attempt_request_count == 1)
      state_.first_request_ns = state_.last_request.request_ns;
    return state_.last_request;
  }

  RequestRecord Request(Source source, bool record_time = false) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    return RequestLocked(source, lock, record_time);
  }

  Snapshot GetSnapshot() const {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    Snapshot snapshot = state_;
    snapshot.requested = requested_.load();
    return snapshot;
  }

  // Lifetime covers neighbor fusion, local BA, culling and result recording.
  // The contained optimizer/token must be destroyed before this guard closes.
  class Attempt {
  public:
    template <typename Queue>
    Attempt(LocalMappingCancellation& owner, const bool& stop_requested,
            const Queue& pending_keyframes)
        : owner_(owner) {
      owner_.BeginAttempt(stop_requested, pending_keyframes);
    }
    ~Attempt() { owner_.EndAttempt(); }
    Attempt(const Attempt&) = delete;
    Attempt& operator=(const Attempt&) = delete;

  private:
    LocalMappingCancellation& owner_;
  };

private:
  template <typename Queue>
  void BeginAttempt(const bool& stop_requested,
                    const Queue& pending_keyframes) {
    // Match RequestStop's lock order. Both state reads and arming occur inside
    // this critical section; taking a stop snapshot before locking is unsafe.
    std::unique_lock<std::mutex> stop_lock(stop_mutex_);
    std::unique_lock<std::mutex> queue_lock(queue_mutex_);
    assert(!state_.active_attempt);
    state_.began_with_stop = stop_requested;
    state_.began_with_queued_keyframes = !pending_keyframes.empty();
    requested_.store(state_.began_with_stop ||
                     state_.began_with_queued_keyframes);
    ++state_.attempt_id;
    state_.attempt_request_count = 0;
    state_.first_request_ns = 0;
    state_.active_attempt = true;
  }

  void EndAttempt() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    assert(state_.active_attempt);
    state_.active_attempt = false;
    // Preserve the observed flag. Only the next BeginAttempt may clear it.
  }

  static std::uint64_t NowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  std::atomic<bool>& requested_;
  std::mutex& stop_mutex_;
  std::mutex& queue_mutex_;
  Snapshot state_;
};

} // namespace ORB_SLAM3
