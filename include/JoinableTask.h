#pragma once
#include <atomic>
#include <exception>
#include <mutex>
#include <thread>
#include <utility>

namespace ORB_SLAM3 {
// Own one asynchronous task. Control operations are serialized; the worker
// never takes control_, so cancellation can join without holding a worker lock.
// The token is not reused until the previous thread has actually returned.
class JoinableTask {
public:
  JoinableTask() = default;
  JoinableTask(const JoinableTask&) = delete;
  JoinableTask& operator=(const JoinableTask&) = delete;
  ~JoinableTask() { CancelAndJoin(); }

  template<class F> void Start(F function) {
    std::lock_guard<std::mutex> lock(control_);
    JoinLocked();
    if (failure_) std::rethrow_exception(failure_);
    stop_.store(false);
    failure_ = nullptr;
    running_.store(true);
    try {
      thread_ = std::thread([this, function]() {
        try { function(stop_); }
        catch (...) { failure_ = std::current_exception(); }
        running_.store(false);
      });
    } catch (...) {
      running_.store(false);
      throw;
    }
  }

  void CancelAndJoin() {
    std::lock_guard<std::mutex> lock(control_);
    JoinLocked();
  }
  // Only joins a worker that already returned; never requests cancellation.
  // Called by the same control thread that owns Start/CancelAndJoin.
  bool JoinIfFinished() {
    std::lock_guard<std::mutex> lock(control_);
    if (running_.load()) return false;
    if (thread_.joinable()) thread_.join();
    return true;
  }
  bool Running() const { return running_.load(); }
  // Must be queried by the controlling thread after joining. Do not discard
  // asynchronous failures silently when deciding whether a map is saveable.
  void RethrowFailure() {
    std::lock_guard<std::mutex> lock(control_);
    if (thread_.joinable()) return;
    if (failure_) std::rethrow_exception(failure_);
  }

private:
  void JoinLocked() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
  }
  std::mutex control_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> running_{false};
  std::exception_ptr failure_;
};
}
