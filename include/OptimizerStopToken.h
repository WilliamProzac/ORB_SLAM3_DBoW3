#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include "Thirdparty/g2o/g2o/core/hyper_graph_action.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"

namespace ORB_SLAM3 {
// Only the optimizer thread writes localStop_. External cancellation uses the
// atomic token. Existing g2o iteration hooks avoid changing its bool* ABI.
class OptimizerStopToken : public g2o::HyperGraphAction {
public:
  OptimizerStopToken(g2o::SparseOptimizer& optimizer,
                     const std::atomic<bool>* stop,
                     std::uint64_t* first_observed_ns = nullptr)
      : optimizer_(optimizer), stop_(stop), first_observed_ns_(first_observed_ns) {
    if (stop_) {
      Requested();
      optimizer_.setForceStopFlag(&localStop_);
      optimizer_.addPreIterationAction(this);
      optimizer_.addPostIterationAction(this);
    }
  }
  ~OptimizerStopToken() {
    if (stop_) {
      optimizer_.removePreIterationAction(this);
      optimizer_.removePostIterationAction(this);
      optimizer_.setForceStopFlag(nullptr);
    }
  }
  bool Requested() {
    if (stop_ && stop_->load()) {
      if (!localStop_ && first_observed_ns_)
        *first_observed_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
      localStop_ = true;
    }
    return localStop_;
  }
  bool Observed() const { return localStop_; }
  HyperGraphAction* operator()(const g2o::HyperGraph*, Parameters*) override {
    Requested();
    return this;
  }
private:
  g2o::SparseOptimizer& optimizer_;
  const std::atomic<bool>* stop_;
  bool localStop_ = false;
  std::uint64_t* first_observed_ns_;
};
}
